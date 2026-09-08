#!/usr/bin/env python3
"""Learning curves from the r2dreamer training logs: per-seed online rollout outcomes binned on a fixed env-step grid,
then mean +/- SE across seeds, per arm and stage. Also reports ignition step per seed and classifies dead seeds.

WHAT THESE CURVES ARE, AND ARE NOT (this text is echoed into the CSV header and the figure caption):
  They are ONLINE TRAINING ROLLOUTS -- the exploring policy, resetting from the TRAINING bank. They are NOT the
  evaluation protocol: the eval cells use mode (or sampled) action selection on the pol-E / rnd30 start sets and exist
  only at the FINAL checkpoint. So a curve's endpoint is not the table number and must not be compared with it.
  Where a series is a proxy it is named as one: for the end-to-end runs `nested` in the training log is the TRAINING
  PROXY (sticky contact + grip commanded open + both upright), which over-counts the settled predicate ~2.5x, so it is
  emitted as `nested_proxy`.
usage: learning_curves.py <runs_dir> <out.csv> [n_bins]"""
import glob, json, os, sys
import numpy as np

runs, out_csv = sys.argv[1], sys.argv[2]
NB = int(sys.argv[3]) if len(sys.argv) > 3 else 40
FAM = [
    ("place",   "s2_r2d_place_state_dH_bnormclamp1ent5_s%d",            "s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s%d",
     ["placed_v2", "contact"]),
    ("slide",   "s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s%d", "s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s%d",
     ["contact", "task_success"]),
    ("e2e",     "full_r2d_state_dHfull_all_bnormclampS8ent5_s%d",       "full_r2d_state_dDPfull_bnormclampS8ent5_s%d",
     ["picked", "contact", "nested"]),
    # pick (user, 2026-09-08): the stage-1 arms of record. scope='pick' pays +1 and terminates on the pick grant, so
    # `picked` IS the task; `contact` is incidental and only logged.
    ("pick",    "s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s%d",        "s2_r2d_pick_state_dDP_bnormclamp1ent5_s%d",
     ["picked"]),
]
RENAME = {("e2e", "nested"): "nested_proxy"}     # never call the training proxy `nested` on a plot

# --- scope='full' ONLY: derive stages from episode/score, not from the log_* flags -----------------
# In scope='full' the adapter writes the stage flags only when the episode terminates INSIDE it (nested
# proxy or tip); a TimeLimit truncation happens outside, so a TRUNCATED episode logs all-zero flags even
# when it picked. Measured on dHfull_all s3: 1198/2911 episodes all-zero, every one of length exactly 300
# (the horizon), 608 of them with score >= 1 -- picked reads 0.480 by flag against 0.688 by score. The
# flag series was also degenerate: picked, contact and nested all ignited at the SAME step, because what
# it really measured was "terminated having reached the stage".
# episode/score accumulates the reward stream and survives truncation, and the staged ladder is
# picked +1 / placed +1 (stale band, ~never earned) / contact +2 / nested +4, so:
SCORE_THRESHOLDS = {"picked": 1.0, "contact": 3.0, "nested": 7.0}
# pick / place / slide are UNAFFECTED: each terminates on its own stage, so a truncated episode there
# genuinely did not achieve it and the flag is exact.

def seed_series(run_dir, stage, fam=None):
    f = os.path.join(run_dir, "metrics.jsonl")
    if not os.path.exists(f):
        return None
    steps, vals = [], []
    key = "episode/train_" + stage
    thr = SCORE_THRESHOLDS.get(stage) if fam == "e2e" else None      # see SCORE_THRESHOLDS above
    for line in open(f):
        try:
            d = json.loads(line)
        except Exception:
            continue
        if thr is not None:
            if "episode/score" in d and "step" in d:
                steps.append(float(d["step"])); vals.append(float(float(d["episode/score"]) >= thr))
            continue
        if key in d and "step" in d:
            steps.append(float(d["step"])); vals.append(float(d[key]))
    if len(steps) < 10:
        return None
    return np.asarray(steps), np.asarray(vals)

rows = []
summary = []
for fam, hp, mp, stages in FAM:
    # fixed grid: the max step reached by any seed of either arm in this family
    smax = 0.0
    for pat in (hp, mp):
        for s in range(8):
            r = seed_series(os.path.join(runs, pat % s), stages[0], fam)
            if r is not None:
                smax = max(smax, r[0].max())
    if smax <= 0:
        continue
    edges = np.linspace(0, smax, NB + 1)
    centers = (edges[:-1] + edges[1:]) / 2
    for arm, pat in (("human", hp), ("machine", mp)):
        for stage in stages:
            name = RENAME.get((fam, stage), stage)
            per_seed = {}
            for s in range(8):
                r = seed_series(os.path.join(runs, pat % s), stage, fam)
                if r is None:
                    continue
                st, v = r
                idx = np.clip(np.digitize(st, edges) - 1, 0, NB - 1)
                b = np.full(NB, np.nan)
                for i in range(NB):
                    m = idx == i
                    if m.any():
                        b[i] = v[m].mean()
                per_seed[s] = b
            if not per_seed:
                continue
            M = np.vstack([per_seed[s] for s in sorted(per_seed)])
            n = np.sum(~np.isnan(M), axis=0)
            mean = np.nanmean(M, axis=0)
            sd = np.nanstd(M, axis=0, ddof=1) if M.shape[0] > 1 else np.zeros(NB)
            se = np.where(n > 1, sd / np.sqrt(np.maximum(n, 1)), np.nan)
            for i in range(NB):
                rows.append(dict(family=fam, arm=arm, stage=name, bin=i, step=centers[i],
                                 mean=mean[i], se=se[i], n_seeds=int(n[i]),
                                 per_seed=" ".join("%.4f" % per_seed[s][i] if not np.isnan(per_seed[s][i]) else "nan"
                                                   for s in sorted(per_seed))))
            # ignition + dead-seed classification, per seed
            for s in sorted(per_seed):
                b = per_seed[s]; fin = b[~np.isnan(b)]
                ign = next((centers[i] for i in range(NB) if not np.isnan(b[i]) and b[i] >= 0.2), None)
                tail = fin[-3:] if len(fin) >= 3 else fin
                peak = float(np.nanmax(b)) if np.any(~np.isnan(b)) else float("nan")
                if ign is None:
                    cls = "never ignited (<0.2 throughout)"
                elif len(tail) and float(np.mean(tail)) < 0.1 <= peak:
                    cls = "ignited then collapsed"
                else:
                    cls = "ignited and held"
                summary.append(dict(family=fam, arm=arm, stage=name, seed=s,
                                    ignition_step=("" if ign is None else "%.0f" % ign),
                                    peak="%.3f" % peak, tail_mean="%.3f" % (float(np.mean(tail)) if len(tail) else float("nan")),
                                    classification=cls))
import csv
hdr = ("# Learning curves from ONLINE TRAINING ROLLOUTS (exploring policy, resetting from the TRAINING bank).\n"
       "# NOT the evaluation protocol: eval cells use mode/sampled actions on the polE / rnd30 start sets and exist only\n"
       "# at the FINAL checkpoint, so a curve endpoint is NOT the table number and must not be compared with it.\n"
       "# e2e 'nested_proxy' is the TRAINING PROXY (over-counts the settled predicate ~2.5x); it is never 'nested'.\n")
with open(out_csv, "w", newline="") as fh:
    fh.write(hdr)
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
with open(out_csv.replace(".csv", "_seeds.csv"), "w", newline="") as fh:
    fh.write(hdr)
    w = csv.DictWriter(fh, fieldnames=list(summary[0].keys())); w.writeheader(); w.writerows(summary)
print("wrote %s (%d rows) and %s (%d seed rows)" % (out_csv, len(rows), out_csv.replace(".csv", "_seeds.csv"), len(summary)))
for fam in sorted(set(r["family"] for r in summary)):
    print("\n== %s: seed classification" % fam)
    for arm in ("human", "machine"):
        sel = [r for r in summary if r["family"] == fam and r["arm"] == arm and r["stage"] in ("placed_v2", "contact", "picked")]
        if not sel: continue
        st = sel[0]["stage"]
        sel = [r for r in sel if r["stage"] == st]
        cls = {}
        for r in sel: cls[r["classification"]] = cls.get(r["classification"], 0) + 1
        ign = [float(r["ignition_step"]) for r in sel if r["ignition_step"]]
        print("   %-8s stage=%-11s %s | median ignition step %s" %
              (arm, st, cls, ("%.0f" % np.median(ign)) if ign else "n/a"))
