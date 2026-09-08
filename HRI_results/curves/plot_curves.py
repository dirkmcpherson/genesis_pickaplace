#!/usr/bin/env python3
"""Plot the learning curves with the caveat carried IN THE FIGURE (coordinator's hard constraint 2026-09-08), and test
whether the arms differ in TIME TO IGNITION rather than final level. Input: curves_2026-09-08*.csv."""
import csv, itertools, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def read(fn):
    rows = []
    with open(fn) as fh:
        for line in fh:
            if line.startswith("#"):
                continue
            rows = list(csv.DictReader(itertools.chain([line], fh))); break
    return rows

curves, seeds = read("curves_2026-09-08.csv"), read("curves_2026-09-08_seeds.csv")
CAVEAT = ("WORLD MODEL (r2dreamer) ONLY — RLPD persists no training-time metrics and DP is offline (no online rollouts),\n"
          "so neither can produce this curve without new compute; the per-learner comparison exists for FINAL numbers only.\n"
          "TRAINING ROLLOUTS (exploring policy, resetting from the TRAINING bank) — NOT the evaluation protocol.\n"
          "Endpoints are NOT the table numbers: eval uses mode/sampled actions on the polE / rnd30 start sets, final "
          "checkpoint only.\nDo not compare against PHASE_RESULTS §2.y / §3 / §5.1 or the HRI_results tables.")
COL = {"human": "#1f77b4", "machine": "#d62728"}

def perm(a, b, n=200000, seed=0):
    a, b = np.asarray(a, float), np.asarray(b, float)
    obs = a.mean() - b.mean(); pool = np.concatenate([a, b]); na = len(a)
    rng = np.random.default_rng(seed); cnt = 0
    idx = np.arange(len(pool))
    for _ in range(n):
        rng.shuffle(idx)
        d = pool[idx[:na]].mean() - pool[idx[na:]].mean()
        cnt += abs(d) >= abs(obs) - 1e-12
    return obs, (cnt + 1) / (n + 1)

print("=== TIME TO IGNITION (first binned step at which the seed's online success reaches 0.2) ===")
ig_lines = []
for fam, stage in (("pick", "picked"), ("place", "placed_v2"), ("slide", "contact"), ("e2e", "picked")):
    g = {}
    for r in seeds:
        if r["family"] == fam and r["stage"] == stage and r["ignition_step"]:
            g.setdefault(r["arm"], []).append(float(r["ignition_step"]))
    if len(g) < 2:
        continue
    h, m = g.get("human", []), g.get("machine", [])
    o, p = perm(h, m)
    line = ("%-6s %-11s human n=%d median %.0f | machine n=%d median %.0f | Δ mean %+.0f steps, perm p = %.3f"
            % (fam, stage, len(h), np.median(h), len(m), np.median(m), o, p))
    print("  " + line); ig_lines.append((fam, stage, h, m, o, p))
    print("     human seeds  : %s" % np.sort(h).astype(int).tolist())
    print("     machine seeds: %s" % np.sort(m).astype(int).tolist())

print("\n=== seed classification (dead-seed check) ===")
for fam in ("pick", "place", "slide", "e2e"):
    for arm in ("human", "machine"):
        sel = [r for r in seeds if r["family"] == fam and r["arm"] == arm]
        if not sel: continue
        st = sorted({r["stage"] for r in sel})[0]
        sel = [r for r in sel if r["stage"] == st]
        cls = {}
        for r in sel: cls[r["classification"]] = cls.get(r["classification"], 0) + 1
        print("  %-6s %-8s stage=%-12s %s" % (fam, arm, st, cls))

for variant in ("mean", "seeds"):
    fams = [("pick", ["picked"]), ("place", ["placed_v2", "contact"]), ("slide", ["contact", "task_success"]),
            ("e2e", ["picked", "contact", "nested_proxy"])]
    ncol = max(len(s) for _, s in fams)
    fig, axes = plt.subplots(len(fams), ncol, figsize=(4.6 * ncol, 3.5 * len(fams)), squeeze=False)
    for r, (fam, stages) in enumerate(fams):
        for c in range(ncol):
            ax = axes[r][c]
            if c >= len(stages):
                ax.axis("off"); continue
            stage = stages[c]
            for arm in ("human", "machine"):
                sel = sorted([x for x in curves if x["family"] == fam and x["arm"] == arm and x["stage"] == stage],
                             key=lambda x: int(x["bin"]))
                if not sel: continue
                st = np.array([float(x["step"]) for x in sel])
                mu = np.array([float(x["mean"]) if x["mean"] not in ("", "nan") else np.nan for x in sel])
                se = np.array([float(x["se"]) if x["se"] not in ("", "nan") else np.nan for x in sel])
                if variant == "mean":
                    ax.plot(st, mu, color=COL[arm], lw=2, label="%s (mean ± SE, 8 seeds)" % arm)
                    ax.fill_between(st, mu - se, mu + se, color=COL[arm], alpha=0.22, lw=0)
                else:
                    per = np.array([[float(v) if v != "nan" else np.nan for v in x["per_seed"].split()] for x in sel])
                    for j in range(per.shape[1]):
                        ax.plot(st, per[:, j], color=COL[arm], lw=0.9, alpha=0.55,
                                label=("%s (per seed)" % arm) if j == 0 else None)
            ax.set_title("%s — %s%s" % (fam, stage, "  [TRAINING PROXY]" if stage == "nested_proxy" else ""), fontsize=10)
            ax.set_xlabel("env step (training)"); ax.set_ylabel("online rollout success")
            ax.set_ylim(-0.02, 1.02); ax.grid(alpha=0.25); ax.legend(fontsize=7, loc="upper left")
    fig.suptitle("Learning curves, WORLD MODEL (r2dreamer) only — ONLINE TRAINING ROLLOUTS, not evaluation" +
                 ("  (per-seed traces)" if variant == "seeds" else "  (mean ± SE across 8 seeds)"), fontsize=12)
    fig.text(0.5, 0.005, CAVEAT, ha="center", va="bottom", fontsize=8, color="#333333")
    fig.tight_layout(rect=[0, 0.075, 1, 0.97])
    for ext in ("png", "pdf"):
        fig.savefig("learning_curves_%s.%s" % (variant, ext), dpi=150)
    plt.close(fig)
    print("wrote learning_curves_%s.png/.pdf" % variant)
