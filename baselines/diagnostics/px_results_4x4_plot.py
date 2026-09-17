#!/usr/bin/env python3
"""Performance chart for the pixel nested_sparse10 study: 4 learners x 4 demonstration datasets.

Reads px_results_4x4_per_seed.csv (written by px_phase_analysis.py) and draws, per learner, one bar per dataset
= mean over seeds of the statistic of record, with every seed as a dot, a 95 % bootstrap CI, and n/design under the bar.
A seed with no cell is absent (not drawn, not zero).

  python baselines/diagnostics/px_results_4x4_plot.py --csv paper/figures/px_phase_2026-09-14/px_results_4x4_per_seed.csv \
      --out paper/figures/px_phase_2026-09-14/fig_results_4x4
"""
import argparse, csv, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LEARNERS = ["{DreamerV3 losses}", "{r2dreamer}", "{RLPD}", "{Diffusion Policy}"]
STAT = {"{DreamerV3 losses}": "rnd30 MODE, mean of 1.5M + 2M cells",
        "{r2dreamer}": "rnd30 MODE, mean of 1.5M + 2M cells",
        "{RLPD}": "rnd30 MODE, 250k-decision checkpoint",
        "{Diffusion Policy}": "rnd30 SAMPLE, 100k updates"}
DATASETS = [("human", "human"), ("machine", "machine\n(DP teacher)"), ("planner72", "planner"),
            ("r2teacher", "r2dreamer\nteacher")]
COLORS = {"human": "#1f6fb4", "machine": "#c8553d", "planner72": "#3a9a5b", "r2teacher": "#8a5fb0"}


def boot_ci(v, rng, n=2000):
    if len(v) < 2:
        return None
    m = rng.choice(v, size=(n, len(v)), replace=True).mean(1)
    return np.percentile(m, [2.5, 97.5])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--metric", default="home", choices=["home", "picked"])
    ap.add_argument("--hold15", action="store_true", help="label panels for the hold15 (training-start) table")
    ap.add_argument("--sampled", action="store_true",
                    help="label panels for the all-SAMPLED-actions table (px_results_4x4_sampled.py)")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    if a.hold15:
        STAT.update({"{DreamerV3 losses}": "hold15 MODE*, mean of 1.5M + 2M",
                     "{r2dreamer}": "hold15 MODE*, mean of 1.5M + 2M",
                     "{RLPD}": "hold15 SAMPLE, 250k-decision checkpoint",
                     "{Diffusion Policy}": "hold15 SAMPLE, 100k updates"})
    if a.sampled:
        STAT.update({"{DreamerV3 losses}": "rnd30 SAMPLE, mean of 1.5M + 2M cells",
                     "{r2dreamer}": "rnd30 SAMPLE, mean of 1.5M + 2M cells",
                     "{RLPD}": "rnd30 SAMPLE, 250k-decision checkpoint"})
    vals = collections.defaultdict(list)
    design = collections.Counter()
    for r in rows:
        if r.get("in_design") not in ("True", "true", "1"):
            continue
        key = (r["learner"], r["dataset"])
        design[key] += 1
        if r.get(a.metric, "") not in ("", None):
            vals[key].append(float(r[a.metric]))
    rng = np.random.default_rng(20260917)
    fig, axes = plt.subplots(1, 4, figsize=(13, 4.3), sharey=True)
    for ax, lr in zip(axes, LEARNERS):
        ticklabels = []
        for i, (ds, label) in enumerate(DATASETS):
            v = np.asarray(vals.get((lr, ds), []), float)
            n, nd = len(v), design.get((lr, ds), 0)
            col = COLORS[ds]
            if n:
                ax.bar(i, v.mean(), width=0.7, color=col, alpha=0.35 if n < nd else 0.6,
                       edgecolor=col, hatch="//" if n < nd else None, linewidth=1)
                jit = rng.uniform(-0.18, 0.18, n)
                ax.scatter(i + jit, v, s=14, color=col, edgecolor="k", linewidth=0.4, zorder=3)
                ci = boot_ci(v, rng)
                if ci is not None:
                    ax.errorbar(i, v.mean(), yerr=[[v.mean() - ci[0]], [ci[1] - v.mean()]], color="k",
                                capsize=3, lw=1, zorder=4)
                top = max(v.max(), v.mean() if ci is None else ci[1])
                ax.text(i, min(top + 0.03, 1.04), f"{v.mean():.2f}", ha="center", va="bottom", fontsize=8,
                        fontweight="bold")
            else:
                ax.text(i, 0.02, "no\ncells", ha="center", va="bottom", fontsize=7, color="0.45")
            ticklabels.append(f"{label}\nn={n}/{nd}")
        ax.set_xticks(range(len(DATASETS)))
        ax.set_xticklabels(ticklabels, fontsize=8)
        ax.set_title(f"{lr}\n{STAT[lr]}", fontsize=9)
        ax.set_ylim(0, 1.12)
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel(f"`{a.metric}` rate (per-seed mean)")
    fig.suptitle(("TRAINING STARTS (hold15: 15 demonstration starts, 14 used in training; *world models have MODE cells only)\n" if a.hold15 else
                  "SAMPLED actions\n" if a.sampled else "") + f"Pixel observation, nested_sparse10: `{a.metric}` by learner and demonstration dataset "
                 "(dots = seeds; bar = mean; whisker = 95 % bootstrap CI; hatched = cell below its design n)",
                 fontsize=9.5)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    for ext in ("png", "pdf"):
        fig.savefig(f"{a.out}.{ext}", dpi=170)
    print(f"[fig] {a.out}.png / .pdf")


if __name__ == "__main__":
    main()
