#!/usr/bin/env python3
"""IGNITION STEP, not just ignition likelihood (user, 2026-09-08).

Emits the RAW per-seed step at which each curve first reaches a given FRACTION OF ITS OWN STEADY STATE, so any threshold
can be reported later without recomputing ("step at which we hit 50% of steady-state operation" and similar).
"Steady state" here is a TAIL AVERAGE -- the mean of the final quarter of that seed's non-empty bins -- and that quarter
is itself still improving for the end-to-end runs, so it is NOT a converged value (independent review, 2026-09-08). Fractions: 0.25 / 0.50 / 0.75 / 0.90, plus the
absolute-0.2 ignition used previously. Nothing is thresholded away: the steady-state value and the raw bin series are
kept, so a different definition can be applied to this file directly.
Input: curves_2026-09-08.csv.  Output: ignition_steps.csv."""
import csv, itertools
import numpy as np

def read(fn):
    with open(fn) as fh:
        for line in fh:
            if line.startswith("#"):
                continue
            return list(csv.DictReader(itertools.chain([line], fh)))

rows = read("curves_2026-09-08.csv")
key = lambda r: (r["family"], r["arm"], r["stage"])
groups = {}
for r in rows:
    groups.setdefault(key(r), []).append(r)

FRACS = (0.25, 0.50, 0.75, 0.90)
out = []
for (fam, arm, stage), rs in sorted(groups.items()):
    rs.sort(key=lambda r: int(r["bin"]))
    steps = np.array([float(r["step"]) for r in rs])
    per = np.array([[float(v) if v != "nan" else np.nan for v in r["per_seed"].split()] for r in rs])  # bins x seeds
    for j in range(per.shape[1]):
        v = per[:, j]
        ok = ~np.isnan(v)
        if ok.sum() < 4:
            continue
        idx = np.where(ok)[0]
        tail = idx[-max(1, len(idx) // 4):]
        steady = float(np.mean(v[tail]))
        rec = dict(family=fam, arm=arm, stage=stage, seed=j, steady_state="%.4f" % steady,
                   peak="%.4f" % float(np.nanmax(v)), final="%.4f" % float(v[idx[-1]]),
                   n_bins=int(ok.sum()), bin_width_steps="%.0f" % (steps[1] - steps[0]))
        for f in FRACS:
            thr = f * steady
            hit = next((steps[i] for i in idx if v[i] >= thr), None) if steady > 0 else None
            rec["step_to_%d%%_of_steady" % int(f * 100)] = "" if hit is None else "%.0f" % hit
        hit02 = next((steps[i] for i in idx if v[i] >= 0.2), None)
        rec["step_to_abs_0.20"] = "" if hit02 is None else "%.0f" % hit02
        out.append(rec)

with open("ignition_steps.csv", "w", newline="") as fh:
    fh.write("# RAW per-seed ignition steps. steady_state = TAIL AVERAGE (mean of the final quarter of that seed's bins),\n"
             "# NOT a converged value: for the end-to-end runs that quarter is still improving. A registered comparison\n"
             "# should use a fixed performance threshold with an explicit persistence rule (independent review 2026-09-08).\n"
             "# Curves are ONLINE TRAINING ROLLOUTS (see README): these steps describe learning speed during training,\n"
             "# not the evaluation protocol. Keep the raw columns; do not collapse to a single threshold in the source.\n")
    w = csv.DictWriter(fh, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
print("wrote ignition_steps.csv (%d seed rows)" % len(out))

def perm(a, b, n=100000, seed=0):
    a, b = np.asarray(a, float), np.asarray(b, float)
    obs = a.mean() - b.mean(); pool = np.concatenate([a, b]); na = len(a)
    rng = np.random.default_rng(seed); c = 0; idx = np.arange(len(pool))
    for _ in range(n):
        rng.shuffle(idx)
        c += abs(pool[idx[:na]].mean() - pool[idx[na:]].mean()) >= abs(obs) - 1e-12
    return obs, (c + 1) / (n + 1)

print("\nstep to 50%% of steady state (per-seed medians; human vs machine)")
for fam, stage in (("place", "placed_v2"), ("slide", "contact"), ("e2e", "picked"), ("e2e", "contact")):
    g = {}
    for r in out:
        if r["family"] == fam and r["stage"] == stage and r["step_to_50%_of_steady"]:
            g.setdefault(r["arm"], []).append(float(r["step_to_50%_of_steady"]))
    if len(g) == 2 and min(len(v) for v in g.values()) >= 2:
        h, m = g["human"], g["machine"]
        o, p = perm(h, m)
        print("  %-6s %-11s human %8.0f (n=%d) | machine %8.0f (n=%d) | Δ %+9.0f steps, perm p = %.3f"
              % (fam, stage, np.median(h), len(h), np.median(m), len(m), o, p))
