#!/usr/bin/env python
"""Full-matrix figures for paper/results_matrix_2026-08-12.md.

Supersedes analysis/make_matrix_figs_20260811.py (kept). Pulls FRESH from
wandb (entity jambotime; projects genesis_paper, dreamer_v3,
r2dreamer_genesis) plus the LOCAL r2dreamer run directories
(~/workspace/r2dreamer/runs/*) for training-run metrics that are not in
wandb, and renders:

  paper/figs/sacfd_collapse_20260812.png      the SACfD story, now FOUR
      stages: old-predicate evals -> hardened re-evals -> hardened absolute
      retrains -> delta-joint (-dj) retrains.
  paper/figs/r2dreamer_ckpt_lottery_20260812.png  checkpoint-eval lottery:
      unclamped pick_delta25d4_s0 vs clamped pick_d4clamp_s0 overlaid,
      local never-discovered seeds s1/s2, and the cluster R2D wave zeros.

  (BC figure figs/bc_central_picked_20260811.png is carried over unchanged;
   this script re-pulls the BC numbers to verify they are unchanged.)

Also writes paper/results_matrix_2026-08-12_numbers.json (raw per-run dump).

Run:  .venv-eval/bin/python analysis/make_matrix_figs_20260812.py
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
R2_LOCAL = os.path.expanduser("~/workspace/r2dreamer/runs")

ENTITY = "jambotime"
BC_WAVE2_CUTOFF = "2026-08-10"            # >= this date -> hardened re-eval wave
SACFD_RETRAIN_CUTOFF = "2026-08-10T21:43" # hardened-predicate retrain launch
R2_VALID_CUTOFF = "2026-08-11T03:00"      # delta evals before this are VOID
DV3_EXCLUDE_BEFORE = "2026-08-01"         # eval-harness scope-restore fix

SOURCE_LABEL = {"dH": "human demos (dH)", "dDP": "model demos (dDP)"}
SOURCE_COLOR = {"dH": "#4878cf", "dDP": "#d1652f"}

DP_EVAL_RE = re.compile(r"^(dH|dDP)_DP_s(\d+)-eval$")
SACFD_RE = re.compile(r"^(dH|dDP)_SACfD_s(\d+)(-eval|-hardened-eval)?$")
SACFD_DJ_RE = re.compile(r"^(dH|dDP)_SACfD-dj_s(\d+)(-eval)?$")
DV3_EVAL_RE = re.compile(r"^(dH|dDP)_DV3_s(\d+)-eval-step(\d+)$")
R2_EVAL_RE = re.compile(r"^(pick_delta25d4|pick_d4clamp)_s(\d)-eval-step(\d+)$")
R2D_TRAIN_RE = re.compile(r"^(dH|dDP)_R2D_s(\d)$")
R2D_EVAL_RE = re.compile(r"^(dH|dDP)_R2D_s(\d)-eval-step(\d+)$")


def created_str(r):
    return str(r.created_at)[:19]


def pull():
    import wandb
    api = wandb.Api(timeout=90)
    out = {"bc": {}, "sacfd": {"old_eval": {}, "hardened_eval": {}, "retrain": {},
                              "retrain_eval": {}},
           "sacfd_dj": {"train": {}, "eval": {}},
           "dv3": {}, "r2": [], "r2d_train": {}, "r2d_eval": []}

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
        m = SACFD_DJ_RE.match(r.name)
        if m:
            src, seed, suffix = m.group(1), int(m.group(2)), m.group(3)
            entry = {"indist": r.summary.get("eval_indist/picked"),
                     "random": r.summary.get("eval_random/picked"),
                     "ep_rew_mean": r.summary.get("rollout/ep_rew_mean"),
                     "timesteps": r.summary.get("time/total_timesteps"),
                     "created": c, "state": r.state}
            bucket = "eval" if suffix == "-eval" else "train"
            out["sacfd_dj"][bucket][(src, seed)] = entry
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
        c = created_str(r)
        m = R2_EVAL_RE.match(r.name)
        if m:
            cfg = dict(r.config)
            ckpt = str(cfg.get("checkpoint", ""))
            mode = cfg.get("mode", "sample")
            cm = re.search(r"(?:ckpt_|CHAMPION_)(\d+)\.pt$", ckpt)
            step = int(cm.group(1)) if cm else int(m.group(3))
            out["r2"].append({
                "family": m.group(1), "seed": int(m.group(2)),
                "name_step": int(m.group(3)), "ckpt_step": step,
                "ckpt": os.path.basename(ckpt), "mode": mode, "created": c,
                "picked": float(r.summary.get("eval/picked", float("nan"))),
                "episodes": r.summary.get("eval/episodes"), "id": r.id,
                "state": r.state, "valid": c >= R2_VALID_CUTOFF,
            })
            continue
        m = R2D_EVAL_RE.match(r.name)
        if m:
            cfg = dict(r.config)
            out["r2d_eval"].append({
                "src": m.group(1), "seed": int(m.group(2)),
                "name_step": int(m.group(3)),
                "ckpt": os.path.basename(str(cfg.get("checkpoint", ""))),
                "mode": cfg.get("mode", "sample"), "created": c,
                "picked": float(r.summary.get("eval/picked", float("nan"))),
                "episodes": r.summary.get("eval/episodes"), "id": r.id,
                "state": r.state,
            })
            continue
        m = R2D_TRAIN_RE.match(r.name)
        if m:
            src, seed = m.group(1), int(m.group(2))
            hist = []
            try:
                for row in r.scan_history(keys=["env_step", "eval/picked"]):
                    hist.append({"env_step": row["env_step"],
                                 "picked": row["eval/picked"]})
            except Exception as e:  # pragma: no cover
                hist = [{"error": str(e)}]
            out["r2d_train"][(src, seed)] = {
                "id": r.id, "state": r.state, "created": c,
                "env_step": r.summary.get("env_step"),
                "summary_eval_picked": r.summary.get("eval/picked"),
                "summary_train_picked": r.summary.get("episode/train_picked"),
                "eval_history": sorted(hist, key=lambda e: e.get("env_step", 0)),
            }
    return out


def pull_local_r2():
    """Local training-run metrics (NOT in wandb): eval/picked series and
    train-episode picked stats for the four local delta runs."""
    out = {}
    for run in ("pick_delta25d4_s0", "pick_delta25d4_s1", "pick_delta25d4_s2",
                "pick_d4clamp_s0"):
        p = os.path.join(R2_LOCAL, run, "metrics.jsonl")
        if not os.path.exists(p):
            continue
        evals, train_pts, last = [], [], 0
        for line in open(p):
            try:
                d = json.loads(line)
            except ValueError:
                continue
            s = d.get("step", 0)
            last = max(last, s)
            if "eval/picked" in d:
                evals.append({"step": s, "picked": d["eval/picked"],
                              "mode_flag": d.get("eval/mode_flag")})
            v = d.get("episode/train_picked")
            if v is not None:
                train_pts.append((s, v))
        train_pts.sort()
        first_nz = next((s for s, v in train_pts if v > 0), None)
        # sustained discovery: first step where a rolling 50-episode window
        # has mean train_picked >= 0.1
        disc = None
        for i in range(49, len(train_pts)):
            w = [v for _, v in train_pts[i - 49:i + 1]]
            if sum(w) / 50 >= 0.1:
                disc = train_pts[i][0]
                break
        out[run] = {
            "last_step": last,
            "eval_series": sorted(evals, key=lambda e: e["step"]),
            "n_train_eps": len(train_pts),
            "n_train_nonzero": sum(1 for _, v in train_pts if v > 0),
            "first_train_success_step": first_nz,
            "sustained_discovery_step_roll50_ge0.1": disc,
        }
    return out


def fisher_exact_greater(a, b, c, d):
    """One-sided (greater) Fisher exact for [[a,b],[c,d]] via hypergeometric."""
    from math import comb
    n = a + b + c + d
    row1, col1 = a + b, a + c
    denom = comb(n, col1)
    p = 0.0
    for x in range(a, min(row1, col1) + 1):
        if col1 - x > c + d:
            continue
        p += comb(row1, x) * comb(c + d, col1 - x) / denom
    return p


def seed_arrays(d, src, regime):
    seeds = sorted(s for (a, s) in d if a == src)
    return seeds, np.array([d[(src, s)][regime] for s in seeds], dtype=float)


def fig_sacfd(sacfd, sacfd_dj):
    stages = [("old_eval", "old evals\n(pre-hardening,\n08-02..08)"),
              ("hardened_eval", "same checkpoints,\nhardened predicate\n(08-10/11)"),
              ("retrain", "hardened retrains,\nabsolute joints,\n200k (08-10/11)"),
              ("dj", "hardened retrains,\nDELTA-joint (-dj),\n400k (08-11/12)")]
    fig, ax = plt.subplots(figsize=(8.4, 4.4))
    rng = np.random.default_rng(0)
    for k, (stage, _) in enumerate(stages):
        for src, off in (("dH", -0.16), ("dDP", 0.16)):
            if stage == "dj":
                d = sacfd_dj["eval"]
            else:
                d = sacfd[stage]
            seeds = sorted(s for (a, s) in d if a == src
                           and d[(a, s)]["indist"] is not None)
            vals = np.array([d[(src, s)]["indist"] for s in seeds], dtype=float)
            x = k + off + rng.uniform(-0.05, 0.05, size=len(vals))
            ax.scatter(x, vals, s=34, color=SOURCE_COLOR[src], zorder=3,
                       edgecolor="white", linewidth=0.6,
                       label=SOURCE_LABEL[src] if k == 0 else None)
    for src, off in (("dH", -0.16), ("dDP", 0.16)):
        for (a, s), e in sacfd["old_eval"].items():
            if a != src or (src, s) not in sacfd["hardened_eval"]:
                continue
            ax.plot([0 + off, 1 + off],
                    [e["indist"], sacfd["hardened_eval"][(src, s)]["indist"]],
                    color=SOURCE_COLOR[src], alpha=0.25, lw=0.9, zorder=1)
    ax.set_xticks(range(len(stages)))
    ax.set_xticklabels([lbl for _, lbl in stages], fontsize=8.5)
    ax.set_ylim(-0.03, 0.7)
    ax.set_ylabel("picked, in-dist (demo ICs), 15 episodes/eval")
    ax.grid(axis="y", color="0.9", zorder=0)
    ax.set_axisbelow(True)
    ax.legend(fontsize=9, frameon=False, loc="upper right")
    ax.set_title("SACfD: 0.07–0.60 band was a predicate exploit; honest rows are 0.00\n"
                 "for both action geometries (delta-joint: one 0.07 seed, dDP s5)",
                 fontsize=10)
    fig.tight_layout()
    p = os.path.join(FIG_DIR, "sacfd_collapse_20260812.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def fig_r2_lottery(r2, r2d_train, local):
    valid = [e for e in r2 if e["valid"] and e["ckpt"] != "banked_peak.pt"
             and e["state"] == "finished"]

    def series(family, seed, mode):
        return sorted((e for e in valid if e["family"] == family
                       and e["seed"] == seed and e["mode"] == mode),
                      key=lambda e: e["ckpt_step"])

    s0 = series("pick_delta25d4", 0, "sample")
    s0_mode = series("pick_delta25d4", 0, "mode")
    s1 = series("pick_delta25d4", 1, "sample")
    s2 = series("pick_delta25d4", 2, "sample")
    # clamp: wandb eval runs, patched with the local metrics.jsonl point for
    # the crashed 2.07M eval run (value 0.00 recorded locally)
    cl = series("pick_d4clamp", 0, "sample")
    cl_mode = series("pick_d4clamp", 0, "mode")
    cl_steps = {e["ckpt_step"] for e in cl}
    for e in local.get("pick_d4clamp_s0", {}).get("eval_series", []):
        if e["step"] not in cl_steps and not e.get("mode_flag"):
            cl.append({"ckpt_step": e["step"], "picked": e["picked"],
                       "local": True})
    cl.sort(key=lambda e: e["ckpt_step"])

    fig, ax = plt.subplots(figsize=(9.6, 4.6))
    CLAMP_C = "#3a9d5d"
    for pts, color, label, lw in (
            (s0, SOURCE_COLOR["dH"], "unclamped s0 (pick_delta25d4), sampled", 1.0),
            (cl, CLAMP_C, "return-clamp=100 s0 (pick_d4clamp), sampled", 1.0)):
        xs = [e["ckpt_step"] / 1e6 for e in pts]
        ys = [e["picked"] for e in pts]
        ax.plot(xs, ys, color=color, lw=lw, alpha=0.45, zorder=2)
        ax.scatter(xs, ys, s=30, color=color, zorder=3,
                   edgecolor="white", linewidth=0.6, label=label)
    for pts, color in ((s0_mode, "0.2"), (cl_mode, CLAMP_C)):
        if pts:
            ax.scatter([e["ckpt_step"] / 1e6 for e in pts],
                       [e["picked"] for e in pts], s=46, marker="s",
                       facecolor="none", edgecolor=color, linewidth=1.2,
                       zorder=4,
                       label="mode-action evals" if color == "0.2" else None)
    never = s1 + s2
    ax.scatter([e["ckpt_step"] / 1e6 for e in never],
               [e["picked"] for e in never], s=26, marker="x", color="0.6",
               zorder=3, label="local s1/s2 (never discover)")
    # cluster wave: periodic eval/picked from the training runs
    cxs, cys = [], []
    for (src, seed), tr in r2d_train.items():
        for e in tr["eval_history"]:
            if "env_step" in e:
                cxs.append(e["env_step"] / 1e6)
                cys.append(e["picked"])
    ax.scatter(cxs, cys, s=14, marker="v", color="#b05fbf", alpha=0.6, zorder=2,
               label="cluster wave dH/dDP_R2D s0-4 (9 runs)")
    ax.annotate("unclamped champion 1.58M:\n1.00/0.87/0.87 sampled; 1.00 mode",
                xy=(1.5768, 1.0), xytext=(0.15, 0.88), fontsize=8,
                arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
    ax.annotate("clamp best 1.87M: 1.00 discovery;\nconfirm 0.80/0.93/0.93; 0.93 mode",
                xy=(1.8724, 1.0), xytext=(2.35, 0.86), fontsize=8,
                arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
    ax.set_xlabel("checkpoint step (millions of env steps)")
    ax.set_ylabel("picked (15 episodes/eval, demo ICs)")
    ax.set_ylim(-0.04, 1.07)
    ax.set_xlim(0, 3.15)
    ax.grid(color="0.92", zorder=0)
    ax.set_axisbelow(True)
    ax.legend(fontsize=8, frameon=False, loc="center left")
    ax.set_title("r2dreamer delta-joint checkpoint lottery: clamped vs unclamped local runs;\n"
                 "cluster statistics wave (certified-identical inputs) has not re-discovered picking",
                 fontsize=10)
    fig.tight_layout()
    p = os.path.join(FIG_DIR, "r2dreamer_ckpt_lottery_20260812.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def main():
    data = pull()
    local = pull_local_r2()
    os.makedirs(FIG_DIR, exist_ok=True)

    fig_sacfd(data["sacfd"], data["sacfd_dj"])
    fig_r2_lottery(data["r2"], data["r2d_train"], local)

    def ser(d):
        return {f"{a}_s{s}": v for (a, s), v in sorted(d.items())}
    dump = {
        "bc": {w: ser(d) for w, d in data["bc"].items()},
        "sacfd": {k: ser(d) for k, d in data["sacfd"].items()},
        "sacfd_dj": {k: ser(d) for k, d in data["sacfd_dj"].items()},
        "dv3": {f"{a}_s{s}": sorted(v, key=lambda e: e["step"])
                for (a, s), v in sorted(data["dv3"].items())},
        "r2": sorted(data["r2"], key=lambda e: (e["family"], e["seed"],
                                                e["ckpt_step"], e["created"])),
        "r2d_train": ser(data["r2d_train"]),
        "r2d_eval": sorted(data["r2d_eval"],
                           key=lambda e: (e["src"], e["seed"], e["created"])),
        "r2_local": local,
    }
    out = os.path.join(REPO, "paper", "results_matrix_2026-08-12_numbers.json")
    with open(out, "w") as f:
        json.dump(dump, f, indent=1)
    print("wrote", out)

    # ---- console digest for the md ----
    print("\n== BC (verify unchanged) ==")
    for w in sorted(data["bc"]):
        for src in ("dH", "dDP"):
            for reg in ("indist", "random"):
                seeds, v = seed_arrays(data["bc"][w], src, reg)
                print(f"{w} {src} {reg}: n={len(v)} mean={v.mean():.3f} "
                      f"[{v.min():.2f},{v.max():.2f}] vals="
                      + " ".join(f"{x:.3f}" for x in v))
    print("\n== SACfD-dj ==")
    for bucket in ("train", "eval"):
        for (src, s), e in sorted(data["sacfd_dj"][bucket].items()):
            print(bucket, src, s, e)
    print("\n== dv3 series ranges ==")
    for (src, s), v in sorted(data["dv3"].items()):
        v = sorted(v, key=lambda e: e["step"])
        nz = [(e["step"], e["picked"]) for e in v if e["picked"] > 0]
        print(f"{src}_s{s}: n={len(v)} max_step={v[-1]['step']} nonzero={nz}")
    print("\n== r2 eval runs (valid) ==")
    for e in dump["r2"]:
        if e["valid"]:
            print(f"{e['family']}_s{e['seed']} ckpt={e['ckpt_step']} "
                  f"mode={e['mode']} picked={e['picked']:.3f} n={e['episodes']} "
                  f"{e['state']} {e['created']} {e['id']}")
    print("\n== cluster R2D train ==")
    for k, tr in sorted(dump["r2d_train"].items()):
        nz = [e for e in tr["eval_history"] if e.get("picked", 0) > 0]
        print(f"{k}: state={tr['state']} env_step={tr['env_step']} "
              f"n_evals={len(tr['eval_history'])} nonzero={nz}")
    print("\n== cluster R2D eval runs ==")
    for e in dump["r2d_eval"]:
        print(f"{e['src']}_s{e['seed']} step={e['name_step']} ckpt={e['ckpt']} "
              f"mode={e['mode']} picked={e['picked']:.3f} n={e['episodes']} "
              f"{e['state']} {e['created']}")
    print("\n== local r2 runs ==")
    for k, v in local.items():
        nzev = [(e["step"], round(e["picked"], 3)) for e in v["eval_series"]]
        print(f"{k}: last={v['last_step']} first_succ={v['first_train_success_step']} "
              f"sustained={v['sustained_discovery_step_roll50_ge0.1']} "
              f"train_eps={v['n_train_eps']} nz={v['n_train_nonzero']}")
        print("   evals:", nzev)

    # Fisher exact: clamp nonzero draws vs unclamped nonzero draws
    cl = [e["picked"] for e in local["pick_d4clamp_s0"]["eval_series"]]
    s0 = [e for e in dump["r2"] if e["valid"] and e["family"] == "pick_delta25d4"
          and e["seed"] == 0 and e["mode"] == "sample"
          and e["ckpt"] != "banked_peak.pt" and e["state"] == "finished"]
    # dedupe repeated evals of the same ckpt (champion confirms): keep all,
    # but ALSO report a per-checkpoint (first draw) variant
    by_ck = {}
    for e in sorted(s0, key=lambda x: x["created"]):
        by_ck.setdefault(e["ckpt_step"], e)
    s0_first = [by_ck[k]["picked"] for k in sorted(by_ck)]
    a, b = sum(1 for x in cl if x > 0), sum(1 for x in cl if x == 0)
    c, d = sum(1 for x in s0_first if x > 0), sum(1 for x in s0_first if x == 0)
    p = fisher_exact_greater(a, b, c, d)
    print(f"\nFisher exact (one-sided, clamp more nonzero): clamp {a}/{a+b} vs "
          f"unclamped(first-draw-per-ckpt) {c}/{c+d}  p={p:.4f}")
    # matched window >= 1.17M
    s0_m = [by_ck[k]["picked"] for k in sorted(by_ck) if k >= 1170000]
    cm, dm = sum(1 for x in s0_m if x > 0), sum(1 for x in s0_m if x == 0)
    pm = fisher_exact_greater(a, b, cm, dm)
    print(f"matched window >=1.17M: clamp {a}/{a+b} vs unclamped {cm}/{cm+dm} "
          f"p={pm:.4f}")


if __name__ == "__main__":
    sys.exit(main())
