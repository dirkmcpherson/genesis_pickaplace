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
# cumulative staged ladder: picked +1, placed +1 (STALE -- never granted), contact +2,
# nested +4. So score>=1 => picked, >=3 => contact, >=7 => nested(proxy). placed_v2 and
# slide_success are NOT reward rungs and cannot be recovered from score at all.
THRESH = [("picked", 1.0), ("contact", 3.0), ("nested (proxy)", 7.0)]
series = {"human": [], "machine": []}
for d in sorted(glob.glob(os.path.join(R, "r2long_*"))):
    arm = "human" if "_h_" in os.path.basename(d) else "machine"
    pts = []
    for line in open(os.path.join(d, "metrics.jsonl")):
        try: o = json.loads(line)
        except Exception: continue
        if "episode/score" in o and "step" in o:
            pts.append((float(o["step"]), float(o["episode/score"])))
    if len(pts) < 40: continue
    pts.sort()
    mx = pts[-1][0]
    edges = np.linspace(0, mx, NB + 1)
    xs, ys = [], []
    for i in range(NB):
        v = [p for s, p in pts if edges[i] <= s < edges[i + 1]]
        if v:
            xs.append((edges[i] + edges[i + 1]) / 2)
            ys.append([float(np.mean([1.0 if q >= t else 0.0 for q in v])) for _, t in THRESH])
    series[arm].append((np.array(xs), np.array(ys), os.path.basename(d)[-3:]))

fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.4), sharey=True)
COLS = {"picked": "#1f77b4", "contact": "#ff7f0e", "nested (proxy)": "#2ca02c"}
grid = np.linspace(0, 4.1e6, 60)
for ax, arm in zip(axes, ("human", "machine")):
    runs = series[arm]
    for si, (name, _t) in enumerate(THRESH):
        stack = []
        for xs, ys, sd in runs:
            col = ys[:, si]
            ax.plot(xs, col, color=COLS[name], alpha=0.22, lw=0.9)
            stack.append(np.interp(grid, xs, col, left=np.nan, right=np.nan))
        if stack:
            with np.errstate(invalid="ignore"):
                m = np.nanmean(np.vstack(stack), axis=0)
            ax.plot(grid, m, color=COLS[name], lw=2.6, label=f"{name} (n={len(runs)})")
    ax.set_title(f"{{r2dreamer}} e2e -- {arm} demos", fontsize=11)
    ax.set_xlabel("environment steps"); ax.grid(alpha=0.25)
    ax.legend(loc="upper left", fontsize=8.5); ax.set_ylim(0, 1.0)
axes[0].set_ylabel("stage rate (training rollouts)")
fig.suptitle("{r2dreamer} end-to-end, 4.1M steps -- SCORE-DERIVED stage rates, TRAINING rollouts",
             fontsize=12, y=1.02)
fig.text(0.5, -0.13, "Score-derived from the staged ladder (picked+1, contact+2, nested+4): score>=1 picked, "
         ">=3 contact, >=7 nested. `placed` is a STALE rung, never granted.\nplaced_v2 and slide_success are NOT "
         "reward rungs and cannot be recovered from score -- they need the 2026-09-09 stage emission, which "
         "postdates these runs.\nTraining rollouts (exploring policy, training bank): endpoints are NOT the "
         "evaluation table numbers. Neither arm has plateaued at 4.1M.", ha="center", fontsize=8)
fig.tight_layout()
for ext in ("png", "pdf"):
    fig.savefig(f"{OUT}/r2dreamer_longrun_score_curves.{ext}", dpi=150, bbox_inches="tight")
print("wrote", OUT, {k: len(v) for k, v in series.items()})
