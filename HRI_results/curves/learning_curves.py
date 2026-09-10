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
usage: learning_curves.py <runs_dir> <out.csv> [n_bins] [--episode-record]

--episode-record reads only complete episode/train_ep_* full-scope records,
never substitutes score/legacy flags, and labels within-episode slide as a
diagnostic. Override --full-human-pattern/--full-machine-pattern for new runs.
Legacy invocation is retained for the existing frozen score-derived curves.
"""
import argparse, glob, json, os, sys
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('runs_dir')
parser.add_argument('out_csv')
parser.add_argument('n_bins', nargs='?', type=int, default=40)
parser.add_argument('--episode-record', action='store_true')
parser.add_argument('--allow-missing-sentinel', action='store_true',
                    help='Accept complete episode records that lack the record_valid certificate '
                         '(validated structurally instead). Needed for runs whose producer predates '
                         'the amendment (w) emitter. An explicit record_valid != 1 is still refused.')
parser.add_argument('--online-steps', action='store_true', help='Full-task only: subtract recorded prefill origin')
parser.add_argument('--online-budget', type=int, default=4000000)
parser.add_argument('--full-human-pattern', default='full_r2d_state_dHfull_all_bnormclampS8ent5_s%d')
parser.add_argument('--full-machine-pattern', default='full_r2d_state_dDPfull_bnormclampS8ent5_s%d')
parser.add_argument('--seeds', type=int, default=8)
args = parser.parse_args()
if args.online_steps and not args.episode_record:
    parser.error("--online-steps requires --episode-record")
runs, out_csv, NB = args.runs_dir, args.out_csv, args.n_bins
if args.seeds < 1 or NB < 1 or args.online_budget < 1:
    parser.error('seeds and n_bins must be positive')
if os.path.exists(out_csv) or os.path.exists(out_csv.replace('.csv', '_seeds.csv')):
    parser.error('output exists; choose a new filename to preserve previous curves')
if not out_csv.endswith('.csv'):
    parser.error('out_csv must end in .csv')
FAM = [
    ("place",   "s2_r2d_place_state_dH_bnormclamp1ent5_s%d",            "s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s%d",
     ["placed_v2", "contact"]),
    ("slide",   "s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s%d", "s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s%d",
     ["contact", "task_success"]),
    ("e2e", args.full_human_pattern, args.full_machine_pattern,
     (["picked", "placed_v2", "contact", "contact_push", "slide_success", "nested"]
      if args.episode_record else ["picked", "contact", "nested"])),
    # pick (user, 2026-09-08): the stage-1 arms of record. scope='pick' pays +1 and terminates on the pick grant, so
    # `picked` IS the task; `contact` is incidental and only logged.
    ("pick",    "s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s%d",        "s2_r2d_pick_state_dDP_bnormclamp1ent5_s%d",
     ["picked"]),
]
if args.online_steps:
    FAM = [f for f in FAM if f[0] == "e2e"]
RENAME = {("e2e", "nested"): "nested_proxy"}     # never call the training proxy `nested` on a plot
if args.episode_record:
    from episode_records import read_records
    RENAME[("e2e", "slide_success")] = 'slide_within_episode_diagnostic'

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
    if fam == 'e2e' and args.episode_record:
        steps, vals = read_records(f, stage, online=args.online_steps,
                               require_sentinel=not args.allow_missing_sentinel)
        if len(steps) < 10:
            return None
        return np.asarray(steps), np.asarray(vals)
    steps, vals = [], []
    # 2026-09-09: prefer the STICKY per-episode key. The old `episode/train_<stage>` is
    # written by the trainer's point-read of one transition, and until the adapter emitted a
    # sticky twin that read 0 for any episode ending in an OUTER-wrapper truncation -- 29
    # episodes scored the pick while the flag read zero on every one. `episode/train_ep_*`
    # carries the cumulative value at whatever step the point-read lands on. Fall back to the
    # old name so pre-2026-09-09 runs still plot.
    key_new = "episode/train_ep_" + stage
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
        k = key_new if key_new in d else key
        if k in d and "step" in d:
            steps.append(float(d["step"])); vals.append(float(d[k]))
    if len(steps) < 10:
        return None
    return np.asarray(steps), np.asarray(vals)

rows = []
summary = []
for fam, hp, mp, stages in FAM:
    # fixed grid: the max step reached by any seed of either arm in this family
    smax = 0.0
    for pat in (hp, mp):
        for s in range(args.seeds):
            r = seed_series(os.path.join(runs, pat % s), stages[0], fam)
            if r is not None:
                smax = max(smax, r[0].max())
    if smax <= 0:
        continue
    edges = np.linspace(0, args.online_budget if args.online_steps else smax, NB + 1)
    centers = (edges[:-1] + edges[1:]) / 2
    for arm, pat in (("human", hp), ("machine", mp)):
        for stage in stages:
            name = RENAME.get((fam, stage), stage)
            per_seed = {}
            for s in range(args.seeds):
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
                actual_seed = s
                if args.online_steps:
                    with open(os.path.join(runs, pat % s, 'run_manifest.json')) as manifest_file:
                        actual_seed = int(json.load(manifest_file)['spec']['seed'])
                    if actual_seed in per_seed:
                        raise ValueError('Duplicate training seed in arm')
                per_seed[actual_seed] = b
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
                if args.episode_record:
                    rows[-1]['seed_ids'] = ' '.join(str(s) for s in sorted(per_seed))
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
                    cls = "crossed (tail >=0.1)" if args.episode_record else "ignited and held"
                summary.append(dict(family=fam, arm=arm, stage=name, seed=s,
                                    ignition_step=("" if ign is None else "%.0f" % ign),
                                    peak="%.3f" % peak, tail_mean="%.3f" % (float(np.mean(tail)) if len(tail) else float("nan")),
                                    classification=cls))
# Legacy diagnostic retained after the truncation defect. Equal ignition bins
# are possible even for nested predicates; equality alone proves no violation.
# New records instead validate actual per-episode implications in read_records.
NESTED_ORDER = ["picked", "contact", "nested_proxy"]

def nested_stage_check(summary_rows):
    """Return a list of complaints: (family, arm, stage_a, stage_b) whose per-seed values are IDENTICAL."""
    bad = []
    by = {}
    for r in summary_rows:
        by.setdefault((r["family"], r["arm"]), {}).setdefault(r["stage"], {})[r["seed"]] = r["ignition_step"]
    for (fam, arm), stages in by.items():
        if fam != 'e2e':
            continue
        present = [s for s in NESTED_ORDER if s in stages]
        for i in range(len(present)):
            for j in range(i + 1, len(present)):
                a, b = present[i], present[j]
                ka, kb = stages[a], stages[b]
                shared = set(ka) & set(kb)
                if len(shared) >= 3 and all(ka[k] == kb[k] for k in shared):
                    bad.append((fam, arm, a, b, len(shared)))
    return bad

_bad = [] if args.episode_record else nested_stage_check(summary)
if _bad:
    print("\n*** DIAGNOSTIC: some stage ignition times are identical; inspect the original episode records.")
    print("*** Identical first-crossing bins can be legitimate and do NOT prove a subset violation.")
    for fam, arm, a, b, n in _bad:
        print(f"      {fam} {arm}: {a!r} == {b!r} on all {n} shared seeds")
else:
    print("\n[stage check] OK" + (": per-episode predicate implications verified" if args.episode_record
                                else ": no identical stage ignition series detected"))

import csv
hdr = ("# Learning curves from ONLINE TRAINING ROLLOUTS (exploring policy, resetting from the TRAINING bank).\n"
       "# NOT the evaluation protocol: eval cells use mode/sampled actions on the polE / rnd30 start sets and exist only\n"
       "# at the FINAL checkpoint, so a curve endpoint is NOT the table number and must not be compared with it.\n"
       "# e2e 'nested_proxy' is the TRAINING PROXY (over-counts the settled predicate ~2.5x); it is never 'nested'.\n")
if args.episode_record:
    hdr += ("# Full-scope source: valid episode/train_ep_* records; no reward-threshold fallback.\n"
            "# slide_within_episode_diagnostic is the legacy grant, NOT accepted-slide or settled success.\n")
if args.online_steps:
    hdr += "# Step axis: online simulator steps, subtracting each run's recorded prefill origin.\n"
    hdr += "# Seed-summary ignition is exploratory first crossing, NOT the proposed three-bin persistence analysis.\n"
if not rows:
    raise SystemExit('No eligible curves (at least 10 episodes per seed required); no output written')
with open(out_csv, "x", newline="") as fh:
    fh.write(hdr)
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
with open(out_csv.replace(".csv", "_seeds.csv"), "x", newline="") as fh:
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
