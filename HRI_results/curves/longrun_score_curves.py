"""Score-derived learning curves for the {r2dreamer} 4.1M long runs.

SCORE-DERIVED, not stage-flag derived: these runs predate the 2026-09-09 emission fix,
so their episode/train_* keys read 0 for any episode ending at the horizon (measured: 29
episodes scored the pick while every flag read 0). episode/score accumulates the reward
stream and survives truncation, so thresholding it is the only honest route here.
Staged ladder is picked +1 / placed +1 / contact +2 / nested +4, so score>=1 => picked.
"""
import json, glob, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = sys.argv[1]; OUT = sys.argv[2]
NB = 25
series = {"human": [], "machine": []}
for d in sorted(glob.glob(os.path.join(R, "r2long_*"))):
    arm = "human" if "_h_" in os.path.basename(d) else "machine"
    pts = []
    for line in open(os.path.join(d, "metrics.jsonl")):
        try: o = json.loads(line)
        except Exception: continue
        if "episode/score" in o and "step" in o:
            pts.append((float(o["step"]), 1.0 if float(o["episode/score"]) >= 1 else 0.0))
    if len(pts) < 40: continue
    pts.sort()
    mx = pts[-1][0]
    edges = np.linspace(0, mx, NB + 1)
    xs, ys = [], []
    for i in range(NB):
        v = [p for s, p in pts if edges[i] <= s < edges[i + 1]]
        if v: xs.append((edges[i] + edges[i + 1]) / 2); ys.append(np.mean(v))
    series[arm].append((np.array(xs), np.array(ys), os.path.basename(d)[-3:]))

fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
for ax, (arm, col) in zip(axes, (("human", "#1f77b4"), ("machine", "#d62728"))):
    runs = series[arm]
    grid = np.linspace(0, 4.1e6, 60)
    stack = []
    for xs, ys, sd in runs:
        ax.plot(xs, ys, color=col, alpha=0.30, lw=1.0)
        stack.append(np.interp(grid, xs, ys, left=np.nan, right=np.nan))
    if stack:
        m = np.nanmean(np.vstack(stack), axis=0)
        ax.plot(grid, m, color=col, lw=2.6, label=f"{arm} mean (n={len(runs)})")
    ax.set_title(f"{{r2dreamer}} e2e — {arm} demos", fontsize=11)
    ax.set_xlabel("environment steps")
    ax.grid(alpha=0.25); ax.legend(loc="lower right", fontsize=9)
    ax.set_ylim(0, 1.0)
axes[0].set_ylabel("picked rate (training rollouts)")
fig.suptitle("{r2dreamer} end-to-end, 4.1M steps — SCORE-DERIVED picked rate, TRAINING rollouts",
             fontsize=12, y=1.02)
fig.text(0.5, -0.10, "Training rollouts (exploring policy, training bank) — NOT the evaluation protocol; "
         "endpoints are not the table numbers.\nScore-derived (score>=1 => picked): these runs predate the "
         "2026-09-09 stage-emission fix, so their per-stage flags are unusable.\nNeither arm has plateaued at "
         "4.1M: final-quarter gain +0.059 human (4/4 seeds), +0.050 machine (3/4).",
         ha="center", fontsize=8.5)
fig.tight_layout()
for ext in ("png", "pdf"):
    fig.savefig(f"{OUT}/r2dreamer_longrun_score_curves.{ext}", dpi=150, bbox_inches="tight")
print("wrote", OUT, {k: len(v) for k, v in series.items()})
