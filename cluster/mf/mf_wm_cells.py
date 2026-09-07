"""Machine-first arm readout helper: per-seed picked COUNTS of the r2dreamer stage-2 runs from their
fresh-process eval cells (fresh_eval_<set><N>_<mode>/metrics.json), for the existing dHv2raw / dDP arms
and the new dRL arm. Numbers only from these files. usage: python mf_wm_cells.py <runs dir> [arm ...]"""
import json
import sys

W = sys.argv[1]
ARMS = sys.argv[2:] or ["dHv2raw", "dDP", "dRL"]
CELLS = {"rnd30_mode": 30, "rnd30_sample": 30, "hold15_mode": 15, "hold15_sample": 15, "rnd300_mode": 300, "rnd300_sample": 300}
for arm in ARMS:
    for cell, N in CELLS.items():
        row = []
        for s in range(8):
            p = f"{W}/s2_r2d_pick_state_{arm}_bnormclamp1ent5_s{s}/fresh_eval_{cell}/metrics.json"
            try:
                d = json.load(open(p)); assert d["episodes"] == N, (p, d["episodes"]); row.append(round(d["picked"] * d["episodes"]))
            except FileNotFoundError:
                row.append(None)
        tot = sum(x for x in row if x is not None); n = sum(1 for x in row if x is not None)
        rate = tot / (n * N) if n else float('nan')
        print(f"WM {arm:8s} {cell:13s} per-seed {row} total {tot}/{n*N} = {rate:.3f}")
