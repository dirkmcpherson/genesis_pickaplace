#!/usr/bin/env python3
"""25k-env-step bins of dv3 `train_success_rate` (+ target/value maxima) per run; the P2 statistic of
paper/DV3_DEBUG_2026-09-05.md: a run 'ignites' at the first bin >= 0.6; 'wobbles' if any LATER bin < 0.6.
usage: dv3dbg_bins.py <run_dir>... (each holding <timestamp>/metrics.jsonl)"""
import json, sys, glob, os
BIN = 25000
for d in sys.argv[1:]:
    fs = sorted(glob.glob(os.path.join(d, "*", "metrics.jsonl")))
    if not fs:
        print(os.path.basename(d), "NO METRICS"); continue
    rows = [json.loads(l) for f in fs for l in open(f) if l.strip()]
    succ = [(r["step"], r["train_success_rate"]) for r in rows if "train_success_rate" in r]
    tr = [r for r in rows if "target_max" in r]
    nb = (max(s for s, _ in succ) // BIN + 1) if succ else 0
    bins = []
    for b in range(nb):
        v = [x for s, x in succ if b * BIN <= s < (b + 1) * BIN]
        bins.append(sum(v) / len(v) if v else float("nan"))
    ign = next((i for i, v in enumerate(bins) if v == v and v >= 0.6), None)
    wob = None if ign is None else any(v == v and v < 0.6 for v in bins[ign + 1:])
    tm = max(r["target_max"] for r in tr) if tr else float("nan")
    vm = max(r["value_max"] for r in tr) if tr else float("nan")
    vm50 = max((r["value_max"] for r in tr if r["step"] >= 50000), default=float("nan"))
    print(f"{os.path.basename(d)} last_step={max(s for s,_ in succ) if succ else 0} bins25k=[{' '.join('nan' if v!=v else f'{v:.2f}' for v in bins)}] "
          f"ignite_bin={ign} wobble={wob} target_max={tm:.2f} value_max={vm:.2f} value_max>=50k={vm50:.2f}")
