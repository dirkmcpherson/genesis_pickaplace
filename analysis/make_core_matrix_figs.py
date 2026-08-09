#!/usr/bin/env python
"""Core-matrix figures for the human-vs-model-demonstrations paper.

Pulls `*-eval` runs fresh from wandb `jambotime/genesis_paper` (keys
`eval_indist/picked`, `eval_random/picked`; 15 episodes per eval) and renders:

  paper/figs/bc_central_picked.png      BC (DP learner) dH vs dDP, min/max bars
  paper/figs/bc_central_picked_se.png   same, mean +/- SE bars
  paper/figs/sacfd_picked.png           SACfD dH vs dDP, min/max bars
  paper/figs/sacfd_picked_se.png        same, mean +/- SE bars

Seeds are auto-discovered from run names `d{source}_{learner}_s{seed}-eval`
(unequal per-condition n supported). Every seed drawn as a point; y from 0;
runs created before 2026-07-30 (old eval semantics) are excluded.

Run:  .venv-eval/bin/python analysis/make_core_matrix_figs.py
"""
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG_DIR = os.path.join(REPO, "paper", "figs")

ENTITY_PROJECT = "jambotime/genesis_paper"
EVAL_SEMANTICS_CUTOFF = "2026-07-30"
RUN_RE = re.compile(r"^(dH|dDP)_(DP|SACfD)_s(\d+)-eval$")
METRICS = ("eval_indist/picked", "eval_random/picked")

SOURCE_LABEL = {"dH": "human demos (dH)", "dDP": "model demos (dDP)"}
SOURCE_COLOR = {"dH": "#4878cf", "dDP": "#d1652f"}


def fetch():
    """{(learner, source): {seed: {metric: rate}}} from finished eval runs."""
    import wandb
    api = wandb.Api()
    data = {}
    for r in api.runs(ENTITY_PROJECT):
        m = RUN_RE.match(r.name)
        if m is None or r.state != "finished":
            continue
        if str(r.created_at)[:10] < EVAL_SEMANTICS_CUTOFF:
            print(f"EXCLUDED (pre-{EVAL_SEMANTICS_CUTOFF} eval semantics): {r.name}")
            continue
        source, learner, seed = m.group(1), m.group(2), int(m.group(3))
        for reg in ("eval_indist", "eval_random"):
            n = r.summary.get(f"{reg}/n")
            if n is not None and int(n) != 15:
                raise SystemExit(f"FATAL: {r.name} {reg}/n = {n}, expected 15")
        entry = {k: r.summary[k] for k in METRICS}
        data.setdefault((learner, source), {})[seed] = entry
    return data


def make_fig(data, learner, out_png, errbar):
    """One grouped bar chart: metric on x, source as bar pair, seeds as dots."""
    fig, ax = plt.subplots(figsize=(6.0, 4.2))
    width = 0.32
    xs = np.arange(len(METRICS))
    ns = []
    for si, source in enumerate(("dH", "dDP")):
        seeds = sorted(data[(learner, source)])
        ns.append(len(seeds))
        for mi, metric in enumerate(METRICS):
            vals = np.array([data[(learner, source)][s][metric] for s in seeds])
            mean = vals.mean()
            if errbar == "range":
                lo, hi = mean - vals.min(), vals.max() - mean
            else:  # se
                se = vals.std(ddof=1) / np.sqrt(len(vals))
                lo = hi = se
            x = xs[mi] + (si - 0.5) * width
            ax.bar(x, mean, width * 0.9, color=SOURCE_COLOR[source], alpha=0.55,
                   label=SOURCE_LABEL[source] if mi == 0 else None, zorder=2)
            ax.errorbar(x, mean, yerr=[[lo], [hi]], fmt="none", ecolor="black",
                        capsize=4, lw=1.2, zorder=4)
            jit = (np.arange(len(vals)) - (len(vals) - 1) / 2) * 0.018
            ax.scatter(x + jit, vals, s=18, color="black", zorder=5)
    bar_desc = "min/max over seeds" if errbar == "range" else "mean +/- SE"
    learner_lbl = "BC (diffusion policy)" if learner == "DP" else "SACfD"
    n_desc = f"n={ns[0]}" if ns[0] == ns[1] else f"n={ns[0]} (dH) / n={ns[1]} (dDP)"
    ax.set_xticks(xs)
    ax.set_xticklabels(["eval_indist (demo-IC)", "eval_random (support-random IC)"])
    ax.set_ylabel("picked rate (15 episodes/eval)")
    ax.set_ylim(0, 1.0)
    ax.set_title(f"{learner_lbl}: pick rate by demo source\n"
                 f"{n_desc} seeds; bars={bar_desc}; dots=seeds", fontsize=9)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(axis="y", alpha=0.3, zorder=0)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    print(f"wrote {out_png}")


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    data = fetch()
    for learner, stem in (("DP", "bc_central_picked"), ("SACfD", "sacfd_picked")):
        for errbar, suffix in (("range", ""), ("se", "_se")):
            make_fig(data, learner, os.path.join(FIG_DIR, f"{stem}{suffix}.png"),
                     errbar)
    # stdout summary for the results note
    for (learner, source), by_seed in sorted(data.items()):
        seeds = sorted(by_seed)
        for metric in METRICS:
            vals = np.array([by_seed[s][metric] for s in seeds])
            print(f"{source}_{learner:6s} {metric:20s} n={len(vals)} "
                  f"mean={vals.mean():.3f} min={vals.min():.3f} max={vals.max():.3f} "
                  f"se={vals.std(ddof=1)/np.sqrt(len(vals)):.3f} "
                  f"vals={[round(float(v),3) for v in vals]}")


if __name__ == "__main__":
    main()
