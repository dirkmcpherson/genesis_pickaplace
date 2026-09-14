#!/usr/bin/env python3
"""Pixel-observation runs (other bot's lane, amendment (af)): training curves mean +/- SE across seeds, eval cells at
0.5M/1M, with the STATE-based nested_sparse(+1) and nested_sparse10 4M arms overlaid at matched x for comparison.
Inputs: records/ (this dir), eval_cells_pixel.tsv, ../curves/ladderN_2026-09-13/records/ (state arms)."""
import csv, os, re, collections, math
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); STATE = os.path.join(HERE, "..", "curves", "ladderN_2026-09-13", "records")
BIN, XMAX, MIN_EP = 50_000, 1_000_000, 5
STAGES = ["picked", "placed_v2", "farside", "slide_event", "home", "tipped"]
COL = {"human": "#1f4e9c", "machine": "#d2691e"}
def arm(s): return "human" if s.startswith("dHfull_all") else "machine"
def load(rec_dir, pred):
    out = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(rec_dir, "INDEX.tsv")), delimiter="\t"):
        if not pred(r): continue
        rows = list(csv.DictReader(open(os.path.join(rec_dir, f"{r['learner']}__{r['set']}__s{r['seed']}.csv"))))
        steps = np.array([int(x["step"]) for x in rows]); st = {k: np.array([int(x[k]) for x in rows]) for k in STAGES}
        out[(r["set"], arm(r["set"]))].append((r["seed"], steps, st))
    return out
def binned(steps, vals):
    nb = XMAX // BIN; idx = np.minimum(steps // BIN, nb - 1); idx = idx[steps < XMAX]; vals = vals[steps < XMAX]
    cnt = np.bincount(idx, minlength=nb); s = np.bincount(idx, weights=vals, minlength=nb)
    return np.where(cnt >= MIN_EP, s / np.maximum(cnt, 1), np.nan)
def mse(mat):
    n = np.sum(~np.isnan(mat), 0); m = np.nanmean(mat, 0); sd = np.nanstd(mat, 0, ddof=1) if mat.shape[0] > 1 else np.full(mat.shape[1], np.nan)
    return m, np.where(n > 1, sd / np.sqrt(np.maximum(n, 1)), np.nan), n
px = load(os.path.join(HERE, "records"), lambda r: "pxsmoke" not in r["set"])
state = load(STATE, lambda r: r["learner"] == "r2dreamer" and r["set"].endswith(("_rnsh", "_rns10h")))
x = (np.arange(XMAX // BIN) + 0.5) * BIN
GROUPS = [("pixels, nested_sparse10, dreamer port (s4-7)", lambda s: s.endswith("rns10h_img_dreamer"), "-"),
          ("pixels, nested_sparse10, r2dreamer port (s0-2)", lambda s: s.endswith("rns10h_img_r2dreamer"), "-"),
          ("pixels, nested_ramp, dreamer port (s0)", lambda s: s.endswith("rnrh_img_dreamer"), ":")]
fig, axes = plt.subplots(3, len(STAGES), figsize=(3.0 * len(STAGES), 8.6), sharex=True, sharey=True)
for gi, (title, pred, ls) in enumerate(GROUPS):
    for si, stage in enumerate(STAGES):
        ax = axes[gi, si]
        for (setname, a), runs in px.items():
            if not pred(setname): continue
            mat = np.array([binned(steps, st[stage]) for _, steps, st in runs])
            for row in mat: ax.plot(x, row, color=COL[a], lw=0.5, alpha=0.3)
            m, se, n = mse(mat); ax.plot(x, m, color=COL[a], lw=1.8, label=f"{a} pixels (n={len(runs)})")
            ok = ~np.isnan(se); ax.fill_between(x[ok], (m - se)[ok], (m + se)[ok], color=COL[a], alpha=0.2, lw=0)
        # state-based sparse(+1) overlay, first 1M of its 4M, dashed grey-tinted
        for (setname, a), runs in state.items():
            if not setname.endswith("_rnsh"): continue
            m, se, n = mse(np.array([binned(steps, st[stage]) for _, steps, st in runs]))
            ax.plot(x, m, color=COL[a], lw=1.2, ls="--", alpha=0.7, label=f"{a} STATE sparse+1 (n={len(runs)}, first 1M of 4M)")
        ax.set_ylim(0, 1); ax.grid(alpha=0.25)
        if gi == 0: ax.set_title(stage)
        if si == 0: ax.set_ylabel(title.replace(", ", "\n"), fontsize=8)
        if gi == 2: ax.set_xlabel("online env steps"); ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v/1e3:g}k"))
axes[0, 0].legend(fontsize=6, frameon=False, loc="upper left")
fig.suptitle("Pixel-observation runs (amendment (af), 1M steps): training rollouts, mean +/- SE across seeds; dashed = the STATE-based nested_sparse(+1) arm over its first 1M (which reached home 0 by 1M and ignited only after 2M)", fontsize=8.5)
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(HERE, "curves_pixel_training.png"), dpi=140); fig.savefig(os.path.join(HERE, "curves_pixel_training.pdf"))
# ---- eval cells ----
cells = list(csv.DictReader(open(os.path.join(HERE, "eval_cells_pixel.tsv")), delimiter="\t"))
fig, axes = plt.subplots(1, 3, figsize=(11, 3.8), sharey=True)
for ax, stage in zip(axes, ["picked", "slide_event", "home"]):
    for title, pred, ls in GROUPS:
        for a in ("human", "machine"):
            per = collections.defaultdict(list)
            for c in cells:
                if c["cell"] != "rnd30_mode" or not c["milestone"].startswith("online_"): continue
                setname = re.sub(r"_s\d+$", "", c["run"].replace("full_r2d_state_", ""))
                if not pred(setname) or arm(setname) != a: continue
                k, n = c[stage].split("/"); per[int(c["milestone"].split("_")[1])].append(int(k) / int(n))
            if not per: continue
            xs = sorted(per); m = [np.mean(per[k]) for k in xs]; se = [np.std(per[k], ddof=1) / math.sqrt(len(per[k])) if len(per[k]) > 1 else 0 for k in xs]
            ax.errorbar(xs, m, yerr=se, color=COL[a], ls=ls, marker="o", ms=4, capsize=3, lw=1.3, label=f"{title.split(',')[2].strip()} {a} n={len(per[xs[0]])}" if False else f"{a} | {title}")
            for k, mm in zip(xs, m): ax.text(k, mm + 0.02, f"n={len(per[k])}", color=COL[a], fontsize=6, ha="center")
    ax.set_xticks([500000, 1000000]); ax.set_xticklabels(["0.5M", "1M"]); ax.set_xlim(300000, 1200000); ax.set_ylim(0, 1); ax.grid(alpha=0.25); ax.set_title(stage)
axes[0].set_ylabel("rnd30 MODE rate (30 starts/seed)"); axes[2].legend(fontsize=5.5, frameon=False, loc="upper left")
fig.suptitle("Pixel runs: EVALUATION cells at milestones (fresh process, mode actions, 30 random starts), mean +/- SE across seeds", fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "eval_pixel_milestones.png"), dpi=140); fig.savefig(os.path.join(HERE, "eval_pixel_milestones.pdf"))
# ---- per-seed eval table ----
with open(os.path.join(HERE, "eval_pixel_per_seed.md"), "w") as f:
    f.write("| run | 0.5M picked | 0.5M slide | 0.5M home | 1M picked | 1M slide | 1M home |\n|---|---|---|---|---|---|---|\n")
    byrun = collections.defaultdict(dict)
    for c in cells:
        if c["cell"] == "rnd30_mode" and c["milestone"].startswith("online_"): byrun[c["run"].replace("full_r2d_state_", "")][c["milestone"]] = c
    for run in sorted(byrun):
        g = lambda ms, k: byrun[run].get(ms, {}).get(k, "-")
        f.write(f"| {run} | {g('online_500000','picked')} | {g('online_500000','slide_event')} | **{g('online_500000','home')}** | {g('online_1000000','picked')} | {g('online_1000000','slide_event')} | **{g('online_1000000','home')}** |\n")
print("done")
