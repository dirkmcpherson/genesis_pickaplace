#!/usr/bin/env python
"""Round-robin (submitted 2026-08-19 night) result figures, pulled from wandb 2026-08-22.
Style rule (paper/figs/STYLE_RULE.md): marker SHAPE = demo source, COLOR = algorithm. Verbatim dicts:"""
import json, re, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
ALG_COLOR = {"DP": "#1b7837", "RLPD": "#4878cf", "R2D": "#cb181d", "DV3": "#8e44ad"}
SRC_MARKER = {"dH": "o", "dDP": "s", "dR2D": "^"}
ALG_LABEL = {"RLPD": "RLPD", "DP": "DP", "R2D": "r2dreamer", "DV3": "dv3"}
OUT = "figs"
H = json.load(open("headline.json")); C = json.load(open("curves.json"))
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "0.9", "grid.linewidth": 0.6, "axes.axisbelow": True})

# ---------------- cells for Fig 1 ----------------
def rlpd_cells():
    core = [r for r in H["rlpd"] if r["where"] == "cluster" and not r["shaped"]]
    dense = [r for r in H["rlpd"] if r["where"] == "cluster" and r["shaped"]]
    return [dict(alg="RLPD", src="dDP", dense=False, label="dDP_RLPD",
                 seeds=[r["seed"] for r in core], y=[r["demoIC"]/r["demoIC_n"] for r in core],
                 y2=[r["randomIC"]/r["randomIC_n"] for r in core], ign=sum(r["demoIC"] >= 3 for r in core),
                 bar=3/15, note="sweep 15 demo-IC / 15 random-IC\nfresh process per ep; bar ≥3/15"),
            dict(alg="RLPD", src="dH", dense=True, label="dH_RLPD\n(dense)",
                 seeds=[r["seed"] for r in dense], y=[r["demoIC"]/r["demoIC_n"] for r in dense],
                 y2=[r["randomIC"]/r["randomIC_n"] for r in dense], ign=sum(r["demoIC"] >= 3 for r in dense),
                 bar=3/15, note="seeds 3-8; same sweep")]
def dp_cells():
    d = sorted(H["dp"], key=lambda r: r["seed"])
    return [dict(alg="DP", src="dR2D", dense=False, label="dR2D_DP", seeds=[r["seed"] for r in d],
                 y=[r["indist"] for r in d], y2=[r["random"] for r in d], ign=None, bar=None,
                 note="wandb_eval --ic-mode both\n15 in-dist / 15 random")]
def r2d_cells():
    out = []
    for src, dense in (("dR2D", False), ("dH", True)):
        d = sorted([r for r in H["r2d"] if r["src"] == src and r["shaped"] == dense], key=lambda r: r["seed"])
        ok = [r for r in d if r["best_confirm_mean"] is not None]
        miss = [r for r in d if r["best_confirm_mean"] is None]
        out.append(dict(alg="R2D", src=src, dense=dense, label=f"{src}_R2D" + ("\n(dense)" if dense else ""),
                        seeds=[r["seed"] for r in ok], y=[r["best_confirm_mean"] for r in ok], y2=None,
                        final=[r["final_ckpt_picked"] for r in ok], bstep=[r["best_ckpt_step"] for r in ok],
                        ign=sum(r["best_confirm_mean"] >= 0.2 for r in ok), bar=0.2,
                        miss=[(r["seed"], r["env_step_final"]) for r in miss],
                        note="BEST ckpt: mean of x3 confirm evals\n(15 eps, fresh seeds 1-3); bar ≥0.20"))
    return out
def dv3_cells():
    # best periodic eval (6 eps) per seed, from the -eval-step runs
    ev = {}
    for e in C["dv3_eval"]:
        m = re.match(r"^rr_(dH|dDP|dR2D)(_shaped)?_s(\d+)-eval-step(\d+)$", e["name"])
        h = e["hist"][0] if e["hist"] else {}
        ev.setdefault((m.group(1), bool(m.group(2)), int(m.group(3))), []).append((int(m.group(4)), h.get("policy_eval/picked")))
    out = []
    for src, dense in (("dH", False), ("dDP", False), ("dR2D", False), ("dH", True)):
        d = sorted([r for r in H["dv3"] if r["src"] == src and r["shaped"] == dense], key=lambda r: r["seed"])
        seeds, y, miss, bstep = [], [], [], []
        for r in d:
            e = sorted(ev.get((src, dense, r["seed"]), []))
            if not e: miss.append((r["seed"], r["final_step"])); continue
            b = max(e, key=lambda t: (t[1] or 0)); seeds.append(r["seed"]); y.append(b[1] or 0); bstep.append(b[0])
        out.append(dict(alg="DV3", src=src, dense=dense, label=f"{src}_DV3" + ("\n(dense)" if dense else ""),
                        seeds=seeds, y=y, y2=None, bstep=bstep, ign=sum(v > 0 for v in y), bar=None, miss=miss,
                        note="best of 2-3 periodic evals\n(6 eps each); bar = nonzero"))
    return out
CELLS = rlpd_cells() + dp_cells() + r2d_cells() + dv3_cells()

def draw_cells(ax, cells, key, title, ylabel):
    rng = np.random.default_rng(0)
    for i, c in enumerate(cells):
        col = ALG_COLOR[c["alg"]]; mk = SRC_MARKER[c["src"]]
        vals = c.get(key)
        if c["dense"]:
            ax.axvspan(i - 0.48, i + 0.48, facecolor="0.95", hatch="///", edgecolor="0.8", linewidth=0, zorder=0)
        if vals is None or len(vals) == 0:
            ax.text(i, 0.5, "not evaluated\non this IC set" if vals is None else "no eval", ha="center", va="center", color="0.5", fontsize=7.5)
            if c.get("miss"):
                ax.text(i, -0.12, f"({len(c['miss'])} seed{'s' if len(c['miss'])>1 else ''} no eval)", ha="center", fontsize=7, color="0.4")
            continue
        v = np.array(vals, float); jit = rng.uniform(-0.22, 0.22, len(v))
        nz = v > 0
        ax.scatter(i + jit[nz], v[nz], s=64, color=col, marker=mk, edgecolor="k", linewidth=0.6, zorder=3)
        ax.scatter(i + jit[~nz], v[~nz], s=64, facecolor="none", marker=mk, edgecolor=col, linewidth=1.4, zorder=3)
        m = v.mean()
        ax.plot([i - 0.32, i + 0.32], [m, m], color=col, lw=3, zorder=4)
        ax.text(i, 1.06, f"mean {m:.2f}", ha="center", fontsize=8, color=col, fontweight="bold")
        if c.get("ign") is not None and key == "y":
            ax.text(i, 1.12, f"{c['ign']}/{len(v)} ignited", ha="center", fontsize=8, color="k")
        else:
            ax.text(i, 1.12, f"n={len(v)}", ha="center", fontsize=8, color="k")
        if c.get("miss"):
            ax.text(i, -0.12, f"(+{len(c['miss'])} seed no eval)", ha="center", fontsize=7, color="0.4")
        if c.get("bar") and key == "y":
            ax.plot([i - 0.45, i + 0.45], [c["bar"]]*2, color="k", ls="--", lw=1, zorder=2)
    ax.set_xlim(-0.6, len(cells) - 0.4); ax.set_ylim(-0.16, 1.2)
    ax.set_yticks(np.arange(0, 1.01, 0.2))
    ax.set_xticks(range(len(cells))); ax.set_xticklabels([c["label"] for c in cells], fontsize=8.5)
    ax.set_ylabel(ylabel); ax.set_title(title, loc="left", fontsize=10, fontweight="bold")
    # algorithm group separators
    prev = None
    for i, c in enumerate(cells):
        if prev is not None and c["alg"] != prev:
            ax.axvline(i - 0.5, color="0.75", lw=0.8, zorder=1)
        prev = c["alg"]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(12.5, 9.2), gridspec_kw={"height_ratios": [1.15, 1]})
draw_cells(a1, CELLS, "y", "(A) Headline pick rate per seed — round robin submitted 2026-08-19 night (all 33 jobs finished)",
           "pick rate (headline protocol per algorithm)")
cells2 = [c for c in CELLS if c.get("y2") is not None]
draw_cells(a2, cells2, "y2", "(B) Random-IC generalization (only RLPD and DP evaluate random ICs)", "pick rate, random ICs")
GROUP_NOTE = {"RLPD": "RLPD headline: fresh-process sweep, 15 demo-IC eps (+15 random-IC in B); ignition ≥3/15",
              "DP": "DP headline: wandb_eval --ic-mode both,\n15 in-dist (+15 random in B); BC, no ignition bar",
              "R2D": "r2dreamer headline: BEST checkpoint (selected on in-train seed-0 eval),\nmean of x3 confirmation evals (15 eps, fresh seeds 1-3); ignition ≥0.20",
              "DV3": "dv3 headline: best of the 2-3 periodic evals (6 eps each, policy_eval/picked);\nround-robin ignition bar = nonzero picked at eval"}
a1.set_ylim(-0.2, 1.2)
# r2d final-ckpt annotation
for c in CELLS:
    if c["alg"] == "R2D" and c.get("final"):
        i = CELLS.index(c)
        a1.text(i, 0.55 if c["src"] == "dR2D" else 0.38, "final-3M ckpt:\n" + "/".join(f"{f:.2f}" for f in c["final"]) +
                "\nbest ckpt @\n" + "/".join(f"{b/1e6:.2f}" for b in c["bstep"]) + "M", ha="center", fontsize=6.2, color="0.3",
                bbox=dict(fc="white", ec="none", alpha=0.8))
    if c["alg"] == "DV3" and c.get("bstep") and any(v > 0 for v in c["y"]):
        i = CELLS.index(c)
        a1.text(i, 0.55, "hit @ " + "/".join(f"{b/1e3:.0f}k" for b, v in zip(c["bstep"], c["y"]) if v > 0) + "\n(one 6-ep eval;\nneighbours 0)",
                ha="center", fontsize=6.2, color="0.3", bbox=dict(fc="white", ec="none", alpha=0.8))
handles = [Line2D([], [], marker=SRC_MARKER[s], color="k", ls="", ms=8, mfc="white", label=f"{s}  (shape = demo source)") for s in SRC_MARKER]
handles += [Patch(facecolor=ALG_COLOR[a], label=f"{ALG_LABEL[a]}  (colour = algorithm)") for a in ALG_COLOR]
handles += [Line2D([], [], marker="o", color="0.4", ls="", ms=8, mfc="none", label="open marker = exactly 0.00"),
            Patch(facecolor="0.95", hatch="///", edgecolor="0.7", label="hatched cell = dense (potential-shaped) reward"),
            Line2D([], [], color="k", ls="--", label="ignition bar (where registered)")]
a2.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8, frameon=False)
fig.suptitle("d{source}_{algorithm} round robin — per-seed results (wandb pull 2026-08-22; entity jambotime)", fontsize=11, y=0.995)
fig.tight_layout(rect=(0, 0.02, 1, 0.98))
fig.subplots_adjust(hspace=0.62)
pos = a1.get_position()
fig.text(pos.x0, pos.y0 - 0.065, "Headline protocols:  " + "\n".join(GROUP_NOTE[a].replace("\n", " ") for a in ("RLPD", "DP", "R2D", "DV3")),
         ha="left", va="top", fontsize=6.8, color="0.35", linespacing=1.4)
fig.savefig(f"{OUT}/rr_headline_per_seed_20260822.png", dpi=200); plt.close(fig)

# ---------------- Fig 2: training-time eval curves ----------------
fig, axs = plt.subplots(1, 3, figsize=(16.5, 4.8))
# RLPD
ax = axs[0]
for k, d in sorted(C["rlpd_train"].items()):
    n = k.split("@")[0]
    m = re.match(r"^(dH|dDP|dR2D)_RLPD-(n20|dense)_s(\d+)(-shaped)?$", n)
    if m.group(2) == "dense": continue  # local dense-verdict retrains: not cluster round robin
    src, dense = m.group(1), bool(m.group(4))
    xs = [x[1] for x in d["eval_indist/picked"]]; ys = [x[2] for x in d["eval_indist/picked"]]
    ax.plot(xs, ys, color=ALG_COLOR["RLPD"], ls="--" if dense else "-", marker=SRC_MARKER[src], ms=5, lw=1.2, alpha=0.85,
            mfc="white" if dense else ALG_COLOR["RLPD"])
    if ys[-1] > 0: ax.annotate(f"s{m.group(3)}", (xs[-1], ys[-1]), xytext=(4, 0), textcoords="offset points", fontsize=6.5, color="0.3", va="center")
ax.set_title("RLPD — in-train eval (10 demo-IC eps, monitoring only)\nsolid = dDP core · dashed/open = dH dense", fontsize=9, loc="left")
ax.set_xlabel("gradient steps"); ax.set_ylabel("picked (in-train eval)"); ax.set_ylim(-0.03, 1.03)
# r2d
ax = axs[1]
for k, d in sorted(C["r2d_train"].items()):
    m = re.match(r"^pick_v5d4c_delta(_shaped)?_(dH|dDP|dR2D)_s(\d+)$", k)
    dense, src, seed = bool(m.group(1)), m.group(2), int(m.group(3))
    xs = [x[1] for x in d["eval/picked"]]; ys = [x[2] for x in d["eval/picked"]]
    ax.plot(xs, ys, color=ALG_COLOR["R2D"], ls="--" if dense else "-", marker=SRC_MARKER[src], ms=3.5, lw=1.1, alpha=0.8,
            mfc="white" if dense else ALG_COLOR["R2D"])
    if ys[-1] > 0 or seed == 43: ax.annotate(f"s{seed}" + (" (stopped 1.8M)" if seed == 43 else ""), (xs[-1], ys[-1]), xytext=(4, 0), textcoords="offset points", fontsize=6.5, color="0.3", va="center")
ax.set_title("r2dreamer — in-train eval (15 eps, seed-0 sample, every 100k env steps)\nsolid = dR2D core (all evals 0, 4 seeds) · dashed/open = dH dense (4/4 ignite; Fig 3)", fontsize=9, loc="left")
ax.set_xlabel("env steps"); ax.set_ylim(-0.03, 1.03)
# dv3
ax = axs[2]
ev = {}
for e in C["dv3_eval"]:
    m = re.match(r"^rr_(dH|dDP|dR2D)(_shaped)?_s(\d+)-eval-step(\d+)$", e["name"])
    h = e["hist"][0] if e["hist"] else {}
    ev.setdefault((m.group(1), bool(m.group(2)), int(m.group(3))), []).append((int(m.group(4)), h.get("policy_eval/picked") or 0))
for (src, dense, seed), pts in sorted(ev.items()):
    pts = sorted(pts); xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=ALG_COLOR["DV3"], ls="--" if dense else "-", marker=SRC_MARKER[src], ms=5, lw=1.1, alpha=0.8,
            mfc="white" if dense else ALG_COLOR["DV3"])
    if max(ys) > 0:
        j = int(np.argmax(ys)); ax.annotate(f"{src} s{seed}: 5/6 @ {xs[j]/1e3:.0f}k", (xs[j], ys[j]), xytext=(6, -2), textcoords="offset points", fontsize=6.5, color="0.3", va="center")
ax.set_title("dv3 — periodic eval (6 eps per point; dR2D s0 never evaluated)\ntwo isolated 5/6 hits; every other eval 0", fontsize=9, loc="left")
ax.set_xlabel("agent steps"); ax.set_ylim(-0.03, 1.03)
handles = [Line2D([], [], marker=SRC_MARKER[s], color="k", ls="", ms=7, mfc="white", label=s) for s in SRC_MARKER]
handles += [Line2D([], [], color=ALG_COLOR[a], lw=3, label=ALG_LABEL[a]) for a in ("RLPD", "R2D", "DV3")]
handles += [Line2D([], [], color="0.3", ls="--", label="dashed = dense (shaped) reward")]
fig.legend(handles=handles, loc="lower center", ncol=8, fontsize=8, frameon=False, bbox_to_anchor=(0.5, -0.01))
fig.suptitle("Training-time eval curves, round robin 2026-08-19 → 20 (shape = demo source, colour = algorithm; DP has no curve — single post-train eval)", fontsize=10)
fig.tight_layout(rect=(0, 0.05, 1, 0.95))
fig.savefig(f"{OUT}/rr_training_curves_20260822.png", dpi=200); plt.close(fig)
print("wrote figs")

# ---------------- Fig 3: r2dreamer dense dH per-seed detail ----------------
dense = sorted([r for r in H["r2d"] if r["shaped"]], key=lambda r: r["seed"])
fig, axs = plt.subplots(1, 4, figsize=(15, 3.9), sharey=True)
for ax, r in zip(axs, dense):
    d = C["r2d_train"][f"pick_v5d4c_delta_shaped_dH_s{r['seed']}"]
    xs = [x[1] for x in d["eval/picked"]]; ys = [x[2] for x in d["eval/picked"]]
    ax.plot(xs, ys, color=ALG_COLOR["R2D"], ls="--", marker=SRC_MARKER["dH"], ms=4, lw=1.2, mfc="white", label="in-train eval (seed-0 sample, 15 eps)")
    tp = d["episode/train_picked"]; first = next((x[1] for x in tp if x[2]), None)
    if first: ax.axvline(first, color="0.6", lw=0.8, ls=":"); ax.text(first, 0.02, f" first train pick\n {first/1e3:.0f}k", fontsize=6.5, color="0.4", va="bottom")
    b = r["best_ckpt_step"]
    ax.axvline(b, color="k", lw=0.8, alpha=0.5)
    ax.scatter([b]*3, r["best_confirm_sample"], s=30, color=ALG_COLOR["R2D"], marker=SRC_MARKER["dH"], edgecolor="k", linewidth=0.5, zorder=5, label="x3 confirm evals @ best ckpt (seeds 1-3)")
    ax.plot([b - 6e4, b + 6e4], [r["best_confirm_mean"]]*2, color="k", lw=2.2, zorder=6, label="confirm mean (headline)")
    ax.scatter([b], [r["best_mode_eval"]], s=40, color="white", marker=SRC_MARKER["dH"], edgecolor=ALG_COLOR["R2D"], linewidth=1.5, zorder=5, label="mode eval @ best ckpt")
    ax.scatter([3e6], [r["final_ckpt_picked"]], s=60, color=ALG_COLOR["R2D"], marker=SRC_MARKER["dH"], edgecolor="k", linewidth=1.8, zorder=5, label="final (3M) ckpt eval")
    ax.set_title(f"dH_R2D dense  s{r['seed']}\nheadline {r['best_confirm_mean']:.2f} @ {b/1e6:.2f}M  ·  final-3M {r['final_ckpt_picked']:.2f}", fontsize=9, loc="left")
    ax.set_xlabel("env steps"); ax.set_ylim(-0.03, 1.08); ax.set_xlim(0, 3.1e6)
axs[0].set_ylabel("picked (15-ep eval)")
axs[2].legend(loc="lower left", fontsize=6.5, frameon=True, framealpha=0.9)
fig.suptitle("r2dreamer with dense (potential-shaped) reward on human demos — all 4 seeds ignite; two collapse by 3M (bistability), one sustains\n"
             "(round robin 2026-08-19; circle = dH, red = r2dreamer; contrast: sparse dR2D r2d this round 0/4, sparse dH r2d prior rounds 8/34)", fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.9))
fig.savefig(f"{OUT}/rr_r2d_dense_dH_seeds_20260822.png", dpi=200); plt.close(fig)
print("wrote fig3")
