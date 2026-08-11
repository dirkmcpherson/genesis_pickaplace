#!/usr/bin/env python
"""Full-matrix figures for paper/results_matrix_2026-08-11.md.

Pulls FRESH from wandb (entity jambotime; projects genesis_paper, dreamer_v3,
r2dreamer_genesis) and renders:

  paper/figs/bc_central_picked_20260811.png   BC (DP) dH vs dDP, n=8, both IC
      regimes; bars = wave-1 evals of record (created 2026-08-02..09), open
      diamonds = the 2026-08-10/11 hardened-predicate re-evals of the SAME
      checkpoints (robustness check).
  paper/figs/sacfd_collapse_20260811.png      the SACfD collapse story:
      old-predicate evals (08-02..08) -> hardened re-evals of the same
      checkpoints -> hardened-predicate retrains. All -> 0.
  paper/figs/r2dreamer_ckpt_stability_20260811.png  every valid checkpoint-eval
      of r2dreamer pick_delta25d4_s0 (runs created after 2026-08-11T03:00Z;
      earlier delta evals are VOID: delta policies were evaluated in absolute
      action mode), picked vs checkpoint step; seed 1 series included.

Also prints a machine-readable numbers block used to write the md tables.

Run:  .venv-eval/bin/python analysis/make_matrix_figs_20260811.py
"""
import json
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG_DIR = os.path.join(REPO, "paper", "figs")

ENTITY = "jambotime"
# Wave boundaries (UTC, string-comparable ISO timestamps).
BC_WAVE2_CUTOFF = "2026-08-10"            # >= this date -> hardened re-eval wave
SACFD_RETRAIN_CUTOFF = "2026-08-10T21:43" # hardened-predicate retrain launch
R2_VALID_CUTOFF = "2026-08-11T03:00"      # delta evals before this are VOID
DV3_EXCLUDE_BEFORE = "2026-08-01"         # eval-harness scope-restore fix

SOURCE_LABEL = {"dH": "human demos (dH)", "dDP": "model demos (dDP)"}
SOURCE_COLOR = {"dH": "#4878cf", "dDP": "#d1652f"}

DP_EVAL_RE = re.compile(r"^(dH|dDP)_DP_s(\d+)-eval$")
SACFD_RE = re.compile(r"^(dH|dDP)_SACfD_s(\d+)(-eval|-hardened-eval)?$")
DV3_EVAL_RE = re.compile(r"^(dH|dDP)_DV3_s(\d+)-eval-step(\d+)$")
R2_EVAL_RE = re.compile(r"^pick_delta25d4_s(\d)-eval-step(\d+)$")


def created_str(r):
    return str(r.created_at)[:19]


def pull():
    import wandb
    api = wandb.Api(timeout=60)
    out = {"bc": {}, "sacfd": {"old_eval": {}, "hardened_eval": {}, "retrain": {},
                              "retrain_eval": {}},
           "dv3": {}, "r2": []}

    for r in api.runs(f"{ENTITY}/genesis_paper"):
        c = created_str(r)
        m = DP_EVAL_RE.match(r.name)
        if m and r.state == "finished":
            wave = "wave2" if c >= BC_WAVE2_CUTOFF else "wave1"
            key = (m.group(1), int(m.group(2)))
            out["bc"].setdefault(wave, {})[key] = {
                "indist": r.summary.get("eval_indist/picked"),
                "random": r.summary.get("eval_random/picked"),
                "created": c,
            }
            continue
        m = SACFD_RE.match(r.name)
        if m:
            src, seed, suffix = m.group(1), int(m.group(2)), m.group(3)
            entry = {"indist": r.summary.get("eval_indist/picked"),
                     "random": r.summary.get("eval_random/picked"),
                     "created": c, "state": r.state}
            if suffix == "-hardened-eval" and r.state == "finished":
                out["sacfd"]["hardened_eval"][(src, seed)] = entry
            elif suffix == "-eval" and r.state == "finished":
                bucket = "retrain_eval" if c >= SACFD_RETRAIN_CUTOFF else "old_eval"
                out["sacfd"][bucket][(src, seed)] = entry
            elif suffix is None and c >= SACFD_RETRAIN_CUTOFF:
                # hardened-predicate retrain training runs (terminal eval in summary)
                cur = out["sacfd"]["retrain"].get((src, seed))
                if cur is None or (r.state == "finished" and cur["state"] != "finished"):
                    out["sacfd"]["retrain"][(src, seed)] = entry

    for r in api.runs(f"{ENTITY}/dreamer_v3"):
        m = DV3_EVAL_RE.match(r.name)
        if not m or r.state != "finished":
            continue
        c = created_str(r)
        if c[:10] < DV3_EXCLUDE_BEFORE:
            continue
        src, seed, step = m.group(1), int(m.group(2)), int(m.group(3))
        v = r.summary.get("policy_eval/picked")
        if v is None:
            continue
        out["dv3"].setdefault((src, seed), []).append(
            {"step": step, "picked": float(v), "created": c})

    for r in api.runs(f"{ENTITY}/r2dreamer_genesis"):
        m = R2_EVAL_RE.match(r.name)
        if not m or r.state != "finished":
            continue
        c = created_str(r)
        cfg = dict(r.config)
        ckpt = str(cfg.get("checkpoint", ""))
        mode = cfg.get("mode", "sample")
        cm = re.search(r"(?:ckpt_|CHAMPION_)(\d+)\.pt$", ckpt)
        step = int(cm.group(1)) if cm else int(m.group(2))
        out["r2"].append({
            "seed": int(m.group(1)), "name_step": int(m.group(2)), "ckpt_step": step,
            "ckpt": os.path.basename(ckpt), "mode": mode, "created": c,
            "picked": float(r.summary.get("eval/picked", float("nan"))),
            "episodes": r.summary.get("eval/episodes"), "id": r.id,
            "valid": c >= R2_VALID_CUTOFF,
        })
    return out


def seed_arrays(d, src, regime):
    seeds = sorted(s for (a, s) in d if a == src)
    return seeds, np.array([d[(src, s)][regime] for s in seeds], dtype=float)


def fig_bc(bc):
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.2), sharey=True)
    for ax, regime, title in zip(
            axes, ("indist", "random"),
            ("in-distribution (demo ICs)", "random ICs (generalization)")):
        for i, src in enumerate(("dH", "dDP")):
            seeds1, v1 = seed_arrays(bc["wave1"], src, regime)
            seeds2, v2 = seed_arrays(bc["wave2"], src, regime)
            mean = v1.mean()
            ax.bar(i, mean, width=0.55, color=SOURCE_COLOR[src], alpha=0.35, zorder=1)
            ax.errorbar(i, mean, yerr=[[mean - v1.min()], [v1.max() - mean]],
                        fmt="none", ecolor="0.25", capsize=5, lw=1.4, zorder=2)
            ax.scatter(np.full_like(v1, i - 0.10), v1, s=28, zorder=3,
                       color=SOURCE_COLOR[src], edgecolor="white", linewidth=0.6,
                       label=None)
            ax.scatter(np.full_like(v2, i + 0.10), v2, s=34, zorder=3,
                       facecolor="none", edgecolor=SOURCE_COLOR[src], marker="D",
                       linewidth=1.2)
        ax.set_xticks([0, 1])
        ax.set_xticklabels([SOURCE_LABEL["dH"], SOURCE_LABEL["dDP"]], fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.set_ylim(0, 1.0)
        ax.grid(axis="y", color="0.9", zorder=0)
        ax.set_axisbelow(True)
    axes[0].set_ylabel("picked rate (15 episodes/eval)")
    h = [plt.Line2D([], [], marker="o", ls="", color="0.4", markersize=6,
                    label="seed, evals of record (08-02..09)"),
         plt.Line2D([], [], marker="D", ls="", markerfacecolor="none",
                    markeredgecolor="0.4", markersize=6,
                    label="same checkpoint, hardened-predicate re-eval (08-10/11)")]
    axes[1].legend(handles=h, fontsize=8, loc="upper right", frameon=False)
    fig.suptitle("BC (Diffusion Policy): human vs model demos, n=8 seeds, "
                 "bar = mean, whiskers = min/max", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    p = os.path.join(FIG_DIR, "bc_central_picked_20260811.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def fig_sacfd(sacfd):
    stages = [("old_eval", "old evals\n(pre-hardening,\n08-02..08)"),
              ("hardened_eval", "same checkpoints,\nhardened predicate\n(08-10/11)"),
              ("retrain", "hardened-predicate\nretrains, 200k\n(08-10/11)")]
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    rng = np.random.default_rng(0)
    for k, (stage, _) in enumerate(stages):
        for src, off in (("dH", -0.16), ("dDP", 0.16)):
            d = sacfd[stage]
            seeds = sorted(s for (a, s) in d if a == src)
            vals = np.array([d[(src, s)]["indist"] for s in seeds], dtype=float)
            x = k + off + rng.uniform(-0.05, 0.05, size=len(vals))
            ax.scatter(x, vals, s=34, color=SOURCE_COLOR[src], zorder=3,
                       edgecolor="white", linewidth=0.6,
                       label=SOURCE_LABEL[src] if k == 0 else None)
    # connect old eval -> hardened re-eval for the same checkpoint
    for src, off in (("dH", -0.16), ("dDP", 0.16)):
        for (a, s), e in sacfd["old_eval"].items():
            if a != src or (src, s) not in sacfd["hardened_eval"]:
                continue
            ax.plot([0 + off, 1 + off],
                    [e["indist"], sacfd["hardened_eval"][(src, s)]["indist"]],
                    color=SOURCE_COLOR[src], alpha=0.25, lw=0.9, zorder=1)
    ax.set_xticks(range(len(stages)))
    ax.set_xticklabels([lbl for _, lbl in stages], fontsize=9)
    ax.set_ylim(-0.03, 0.7)
    ax.set_ylabel("picked, in-dist (demo ICs), 15 episodes/eval")
    ax.grid(axis="y", color="0.9", zorder=0)
    ax.set_axisbelow(True)
    ax.legend(fontsize=9, frameon=False, loc="upper right")
    ax.set_title("SACfD (absolute joint actions): the 0.07–0.60 band was a "
                 "predicate exploit;\nhonest row = 0.00 for every seed of both sources",
                 fontsize=10)
    fig.tight_layout()
    p = os.path.join(FIG_DIR, "sacfd_collapse_20260811.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def fig_r2(r2):
    valid = [e for e in r2 if e["valid"] and e["ckpt"] != "banked_peak.pt"]
    s0_sample = sorted((e for e in valid if e["seed"] == 0 and e["mode"] == "sample"),
                       key=lambda e: e["ckpt_step"])
    s0_mode = [e for e in valid if e["seed"] == 0 and e["mode"] == "mode"]
    s1 = sorted((e for e in valid if e["seed"] == 1), key=lambda e: e["ckpt_step"])
    champ = [e for e in s0_sample if e["ckpt_step"] == 1576820]

    fig, ax = plt.subplots(figsize=(9, 4.4))
    xs = [e["ckpt_step"] / 1e6 for e in s0_sample]
    ys = [e["picked"] for e in s0_sample]
    ax.plot(xs, ys, color=SOURCE_COLOR["dH"], lw=1.0, alpha=0.45, zorder=2)
    ax.scatter(xs, ys, s=34, color=SOURCE_COLOR["dH"], zorder=3,
               edgecolor="white", linewidth=0.6, label="seed 0, sampled actions")
    ax.scatter([e["ckpt_step"] / 1e6 for e in s0_mode],
               [e["picked"] for e in s0_mode], s=48, marker="s",
               facecolor="none", edgecolor="0.2", linewidth=1.3, zorder=4,
               label="seed 0, mode actions")
    ax.scatter([e["ckpt_step"] / 1e6 for e in s1], [e["picked"] for e in s1],
               s=30, marker="x", color="0.55", zorder=3,
               label="seed 1 (never discovers picking)")
    if champ:
        ax.annotate("champion ckpt 1.58M:\n1.00, 0.87, 0.87 sampled (n=45); 1.00 mode",
                    xy=(1.5768, 1.0), xytext=(1.72, 0.83), fontsize=8,
                    arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
    ax.set_xlabel("checkpoint step (millions of env steps)")
    ax.set_ylabel("picked (15 episodes/eval, demo ICs)")
    ax.set_ylim(-0.03, 1.05)
    ax.set_xlim(0, max(xs) * 1.04)
    ax.grid(color="0.92", zorder=0)
    ax.set_axisbelow(True)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    ax.set_title("r2dreamer delta-joint (pick_delta25d4): checkpoint-eval series is "
                 "BISTABLE\n(λ-return explosion phases); champion + independent "
                 "confirmations, hardened predicate", fontsize=10)
    fig.tight_layout()
    p = os.path.join(FIG_DIR, "r2dreamer_ckpt_stability_20260811.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def main():
    data = pull()
    os.makedirs(FIG_DIR, exist_ok=True)
    fig_bc(data["bc"])
    fig_sacfd(data["sacfd"])
    fig_r2(data["r2"])

    # numbers block for the md
    def ser(d):
        return {f"{a}_s{s}": v for (a, s), v in sorted(d.items())}
    dump = {
        "bc": {w: ser(d) for w, d in data["bc"].items()},
        "sacfd": {k: ser(d) for k, d in data["sacfd"].items()},
        "dv3": {f"{a}_s{s}": sorted(v, key=lambda e: e["step"])
                for (a, s), v in sorted(data["dv3"].items())},
        "r2": sorted(data["r2"], key=lambda e: (e["seed"], e["ckpt_step"], e["created"])),
    }
    out = os.path.join(FIG_DIR, "..", "results_matrix_2026-08-11_numbers.json")
    with open(os.path.abspath(out), "w") as f:
        json.dump(dump, f, indent=1)
    print("wrote", os.path.abspath(out))


if __name__ == "__main__":
    sys.exit(main())
