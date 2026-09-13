#!/usr/bin/env python3
"""Ladder-N training curves, human v machine, mean +/- SE across seeds.

Inputs (this directory):
  records/<learner>__<set>__s<seed>.csv   per-episode sticky stage record (step, stages...) -- see extract_records.py
  records/INDEX.tsv                        learner, set, seed, ladder, budget, n_episodes, last_step
  eval_cells_r2dreamer.tsv                 fresh-process milestone cells (ln14_milestone_table.py output)
Outputs:
  curves_<learner>_<ladder>.{png,pdf}      one panel per stage; human v machine; band = +/- 1 SE across seeds;
                                           faint lines = individual seeds; n(seeds) per bin along the top
  home_by_ladder_<learner>.{png,pdf}       `home` (RLPD: nested_v2, its closest logged stage) for every ladder on one
                                           axis, so the reward-function x budget confound is visible
  eval_milestones_r2dreamer.{png,pdf}      rnd30-mode milestone cells (0.5/1/2/4M), mean +/- SE across seeds
  binned_<learner>.csv                     the binned per-seed rates behind the training-curve figures

WHAT THE TRAINING CURVES ARE: online training rollouts of the EXPLORING policy (sampled actions, training-bank starts,
r2dreamer resets from the demo starts; RLPD from its own reset distribution). They are not the evaluation protocol and a
curve's endpoint is not a table number. The eval-cell figure is the evaluation protocol (mode actions, rnd30 starts).
SE across seeds: std(ddof=1)/sqrt(n) over the seeds that have >= MIN_EP episodes in the bin; n is printed per bin.
"""
import csv, os, sys, math, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.join(HERE, "records")
MIN_EP = 5
BIN = {"r2dreamer": 100_000, "rlpd": 12_500}
XMAX = {"r2dreamer": 4_000_000, "rlpd": 500_000}
XUNIT = {"r2dreamer": "online env steps", "rlpd": "decisions"}
STAGES = {"r2dreamer": ["picked", "placed_v2", "farside", "slide_event", "home", "tipped"],
          "rlpd": ["picked", "placed_v2", "nested_v2", "slide_success", "tipped"]}
LADDERS = ["staged", "nested_ramp", "nested_sparse", "nested_sparse10"]
LADDER_LABEL = {"staged": "staged (control): picked1 placed_v2 1 contact_push2 slide_success4",
                "nested_ramp": "nested_ramp: picked1 placed_v2 1 ramp+3*min(1,slide/5cm) home4",
                "nested_sparse": "nested_sparse: home +1 only",
                "nested_sparse10": "nested_sparse10: home +10 only"}
ARM = {"dHfull_all": ("human", "#1f4e9c"), "dDPfull_first": ("machine", "#d2691e")}
LEARNER_LABEL = {"r2dreamer": "{r2dreamer}", "rlpd": "{RLPD}"}

def arm_of(setname):
    return "dHfull_all" if setname.startswith("dHfull_all") else "dDPfull_first"

index = list(csv.DictReader(open(os.path.join(REC, "INDEX.tsv")), delimiter="\t"))
runs = collections.defaultdict(list)  # (learner, ladder, arm) -> [(seed, budget_from_records, steps, {stage: arr})]
for r in index:
    p = os.path.join(REC, f"{r['learner']}__{r['set']}__s{r['seed']}.csv")
    rows = list(csv.DictReader(open(p)))
    if not rows:
        continue
    steps = np.array([int(x["step"]) for x in rows])
    st = {k: np.array([int(x[k]) for x in rows]) for k in rows[0] if k != "step"}
    runs[(r["learner"], r["ladder"], arm_of(r["set"]))].append((int(r["seed"]), int(steps.max()), steps, st))

def binned(learner, steps, vals):
    b = BIN[learner]; nb = XMAX[learner] // b
    idx = np.minimum(steps // b, nb - 1)
    cnt = np.bincount(idx, minlength=nb); s = np.bincount(idx, weights=vals, minlength=nb)
    rate = np.where(cnt >= MIN_EP, s / np.maximum(cnt, 1), np.nan)
    return rate, cnt

def mean_se(mat):
    n = np.sum(~np.isnan(mat), axis=0)
    m = np.nanmean(np.where(n > 0, mat, np.nan), axis=0) if mat.shape[0] else np.full(mat.shape[1], np.nan)
    sd = np.nanstd(mat, axis=0, ddof=1) if mat.shape[0] > 1 else np.full(mat.shape[1], np.nan)
    se = np.where(n > 1, sd / np.sqrt(np.maximum(n, 1)), np.nan)
    return m, se, n

binned_rows = collections.defaultdict(list)

def draw_curve(ax, learner, ladder, stage, show_n=True, label_prefix=""):
    b = BIN[learner]; x = (np.arange(XMAX[learner] // b) + 0.5) * b
    ntxt = {}
    for armkey, (armname, col) in ARM.items():
        rs = runs.get((learner, ladder, armkey), [])
        if not rs:
            continue
        mat = []
        for seed, budget, steps, st in rs:
            rate, cnt = binned(learner, steps, st[stage]); mat.append(rate)
            ax.plot(x, rate, color=col, lw=0.6, alpha=0.25)
            for i, (rt, c) in enumerate(zip(rate, cnt)):
                binned_rows[learner].append([ladder, armname, seed, stage, int(x[i]), "" if np.isnan(rt) else f"{rt:.4f}", int(c)])
        mat = np.array(mat); m, se, n = mean_se(mat)
        ax.plot(x, m, color=col, lw=1.8, label=f"{label_prefix}{armname} (n={len(rs)} seeds)")
        ok = ~np.isnan(se)
        ax.fill_between(x[ok], (m - se)[ok], (m + se)[ok], color=col, alpha=0.22, lw=0)
        ntxt[armname] = n
    if show_n and ntxt:
        for j, (armname, n) in enumerate(ntxt.items()):
            col = ARM["dHfull_all" if armname == "human" else "dDPfull_first"][1]
            for i in range(0, len(x), max(1, len(x) // 8)):
                ax.text(x[i], 1.02 + 0.07 * j, str(n[i]), color=col, fontsize=6, ha="center", va="bottom", transform=ax.get_xaxis_transform())
    ax.set_ylim(0, 1); ax.set_xlim(0, XMAX[learner]); ax.grid(alpha=0.25)
    ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v/1e6:g}M" if v >= 1e6 else f"{v/1e3:g}k"))

for learner in ("r2dreamer", "rlpd"):
    stages = STAGES[learner]
    for ladder in LADDERS:
        if not any((learner, ladder, a) in runs for a in ARM):
            continue
        fig, axes = plt.subplots(1, len(stages), figsize=(3.1 * len(stages), 3.4), sharey=True)
        for ax, stage in zip(axes, stages):
            draw_curve(ax, learner, ladder, stage)
            ax.set_title(stage, pad=16, fontsize=10)
            ax.set_xlabel(XUNIT[learner], fontsize=8)
        axes[0].set_ylabel("fraction of training episodes")
        axes[0].legend(loc="upper left", fontsize=7, frameon=False)
        seeds = {a: sorted(s for s, *_ in runs.get((learner, ladder, a), [])) for a in ARM}
        fig.suptitle(f"{LEARNER_LABEL[learner]} training rollouts -- {LADDER_LABEL[ladder]}\n"
                     f"human seeds {seeds['dHfull_all']}   machine seeds {seeds['dDPfull_first']}   "
                     f"band = +/-1 SE across seeds; small numbers = seeds contributing per bin", fontsize=8.5)
        fig.tight_layout(rect=(0, 0, 1, 0.9))
        for ext in ("png", "pdf"):
            fig.savefig(os.path.join(HERE, f"curves_{learner}_{ladder}.{ext}"), dpi=150)
        plt.close(fig)

    # `home` across ladders on one axis (the reward-function x budget confound, made visible)
    key_stage = "home" if learner == "r2dreamer" else "nested_v2"
    fig, ax = plt.subplots(figsize=(7.5, 4))
    styles = {"staged": ":", "nested_ramp": "--", "nested_sparse": "-", "nested_sparse10": "-."}
    b = BIN[learner]; x = (np.arange(XMAX[learner] // b) + 0.5) * b
    for ladder in LADDERS:
        for armkey, (armname, col) in ARM.items():
            rs = runs.get((learner, ladder, armkey), [])
            if not rs:
                continue
            mat = np.array([binned(learner, steps, st[key_stage])[0] for _, _, steps, st in rs])
            m, se, n = mean_se(mat)
            ax.plot(x, m, color=col, ls=styles[ladder], lw=1.6, label=f"{ladder} {armname} (n={len(rs)} seeds; runs reached {min(bud for _, bud, *_ in rs)/1e6:.2g}-{max(bud for _, bud, *_ in rs)/1e6:.2g}M)")
            ok = ~np.isnan(se); ax.fill_between(x[ok], (m - se)[ok], (m + se)[ok], color=col, alpha=0.12, lw=0)
    ax.set_ylim(0, 1); ax.set_xlim(0, XMAX[learner]); ax.grid(alpha=0.25)
    ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v/1e6:g}M" if v >= 1e6 else f"{v/1e3:g}k"))
    ax.set_xlabel(XUNIT[learner]); ax.set_ylabel(f"training-episode {key_stage} rate")
    ax.set_title(f"{LEARNER_LABEL[learner]}: `{key_stage}` during training, every ladder on one axis\n"
                 "(budgets differ BETWEEN ladders and WITHIN a ladder's seeds; past the shortest run the mean covers fewer seeds -- compare at matched x only)", fontsize=8.5)
    ax.legend(fontsize=7, frameon=False, loc="upper left")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(HERE, f"home_by_ladder_{learner}.{ext}"), dpi=150)
    plt.close(fig)

    with open(os.path.join(HERE, f"binned_{learner}.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["ladder", "arm", "seed", "stage", "bin_center_step", "rate", "n_episodes_in_bin"])
        w.writerows(binned_rows[learner])

# ---- r2dreamer milestone eval cells (evaluation protocol) ----
cells = list(csv.DictReader(open(os.path.join(HERE, "eval_cells_r2dreamer.tsv")), delimiter="\t"))
MS = [500_000, 1_000_000, 2_000_000, 4_000_000]
ladder_of_set = {"rnrh": "nested_ramp", "rnsh": "nested_sparse", "rns10h": "nested_sparse10"}
fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), sharey=True)
for ax, stage in zip(axes, ["picked", "slide_event", "home"]):
    for ladder, ls in (("nested_ramp", "--"), ("nested_sparse", "-"), ("nested_sparse10", "-.")):
        for armkey, (armname, col) in ARM.items():
            per_ms = {}
            for c in cells:
                if c["cell"] != "rnd30_mode" or not c["milestone"].startswith("online_"):
                    continue
                setname, seed = c["run"].replace("full_r2d_state_", "").rsplit("_s", 1)
                if ladder_of_set.get(setname.split("_")[-1]) != ladder or arm_of(setname) != armkey:
                    continue
                a, n = c[stage].split("/"); per_ms.setdefault(int(c["milestone"].split("_")[1]), []).append(int(a) / int(n))
            if not per_ms:
                continue
            xs = sorted(per_ms); m = [np.mean(per_ms[k]) for k in xs]
            se = [np.std(per_ms[k], ddof=1) / math.sqrt(len(per_ms[k])) if len(per_ms[k]) > 1 else 0 for k in xs]
            ax.errorbar(xs, m, yerr=se, color=col, ls=ls, marker="o", ms=4, capsize=3, lw=1.4,
                        label=f"{ladder} {armname}")
            for k, mm in zip(xs, m):
                ax.text(k, mm + 0.02, f"n={len(per_ms[k])}", color=col, fontsize=6, ha="center")
    ax.set_xscale("log"); ax.set_xticks(MS); ax.set_xticklabels(["0.5M", "1M", "2M", "4M"])
    ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    ax.set_ylim(0, 1); ax.grid(alpha=0.25); ax.set_title(stage); ax.set_xlabel("online env steps (milestone)")
axes[0].set_ylabel("rnd30 MODE success rate (30 starts/seed)")
axes[2].legend(fontsize=6.5, frameon=False, loc="upper left")
fig.suptitle("{r2dreamer} EVALUATION cells at milestones (fresh process, mode actions, rnd30 starts), mean +/- SE across seeds", fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.93))
for ext in ("png", "pdf"):
    fig.savefig(os.path.join(HERE, f"eval_milestones_r2dreamer.{ext}"), dpi=150)
print("done")
