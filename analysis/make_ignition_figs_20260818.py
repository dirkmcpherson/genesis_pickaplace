#!/usr/bin/env python
"""Ignition-distribution and performance-with-SE figures, per demo source,
across ALL FOUR learners (DP, RLPD, r2dreamer, dv3).

Input : paper/figs/ignition_numbers_20260818.json (written by
        analysis/assemble_ignition_20260818.py, which pulls fresh from wandb).
Output: paper/figs/ignition_bimodal_{dH,dDP,dR2D}_20260818.png
        paper/figs/perf_all_{dH,dDP,dR2D}_20260818.png
        paper/figs/perf_ignited_{dH,dDP,dR2D}_20260818.png
        paper/figs/overview_ignition_20260818.png

No smoothing anywhere. y-axis always starts at 0. Every seed is a visible point.
"""
import json, os, collections, textwrap
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(REPO, "paper", "figs")
D = json.load(open(os.path.join(FIG, "ignition_numbers_20260818.json")))
BAR = 0.20                      # ignition bar (>=3/15 fresh demo-IC; rate form)

ALG_COLOR = {"DP": "#1b7837", "RLPD": "#4878cf", "R2D": "#cb181d", "DV3": "#8e44ad"}
R2D_SHADE = ["#67000d", "#cb181d", "#fb6a4a", "#e07b39", "#a3a3a3"]   # one per wave, in `order`
DASH = {0: "-", 1: "--", 2: ":", 3: "-.", 4: (0, (5, 1, 1, 1, 1, 1))}

# ------------------------------------------------------------------ series ---
def dedupe(rows, xk="step", yk="picked"):
    m = {}
    for r in rows:
        x, y = r.get(xk), r.get(yk)
        if x is None or y is None: continue
        m[int(x)] = float(y)
    return sorted(m.items())

def build(src):
    """Return list of series dicts for one demo source."""
    S = []
    # ---- DP -------------------------------------------------------------
    if src in D["dp"]:
        d = D["dp"][src]
        seeds = sorted(d, key=int)
        vals = [d[s]["record"]["indist"] for s in seeds]
        S.append(dict(alg="DP", label=f"{src}_DP", n=len(seeds), wave="eval-of-record wave",
                      era="trained 08-01..08-09; evals 08-02..08-09 (hardened re-eval 08-10/11 agrees)",
                      proto="fresh eval process, demo-IC, 15 ep/seed, 1200-step horizon",
                      seed_ids=[int(s) for s in seeds], finals=vals,
                      curves=None, point_step=100000, dash=0))
    # ---- RLPD -----------------------------------------------------------
    fam = {"dH": "dH_n20", "dR2D": "dR2D_n20", "dDP": "dDP_pair"}[src]
    if fam in D["rlpd"]:
        d = D["rlpd"][fam]
        seeds = sorted(d, key=int)
        curves = {int(s): dedupe([{"step": int(k), "picked": v["indist"]}
                                  for k, v in d[s]["curve"].items()]) for s in seeds}
        wave = "cluster n=16 wave (PRIMARY)" if fam.endswith("n20") else "pair wave (n=3, pre demo-RNG fix, superseded era)"
        S.append(dict(alg="RLPD", label=f"{src}_RLPD", n=len(seeds), wave=wave,
                      era=("cluster, 08-17, post demo-RNG fix" if fam.endswith("n20")
                           else "local, 08-14, PRE demo-RNG fix"),
                      proto="in-train sequence eval, demo-IC, 10 ep, 400-step horizon",
                      seed_ids=[int(s) for s in seeds],
                      finals=[d[s]["final_indist"] for s in seeds],
                      curves=curves, point_step=None, dash=0))
    # ---- r2dreamer ------------------------------------------------------
    waves = collections.defaultdict(list)
    for k, v in D["r2d"].items():
        if v["source"] != src or v["wave"] == "local pilot (VOID)": continue
        waves[v["wave"]].append(v)
    order = ["local (delta recipe)", "cluster wave 1", "cluster wave 2 (firming)",
             "cluster wave 3", "cluster 3M wave (SUPERSEDED)"]
    for i, w in enumerate([w for w in order if w in waves]):
        vs = sorted(waves[w], key=lambda v: (v["family"], v["seed"]))
        curves, finals, ids, fallback = {}, [], [], []
        for j, v in enumerate(vs):
            curves[j] = dedupe(v["inloop"])
            fb = v["fresh_best"] is None
            finals.append(v["inloop_best"] if fb else v["fresh_best"])
            fallback.append(fb)
            ids.append(f'{v["family"].replace("pick_","").replace("_delta","")}_s{v["seed"]}')
        S.append(dict(alg="R2D", color=R2D_SHADE[i % len(R2D_SHADE)],
                      label=f"{src}_R2D {w}", n=len(vs), wave=w,
                      era=vs[0]["era"],
                      proto="metric = BEST fresh-process checkpoint eval (15 ep, demo-IC, sampled); curve = in-loop eval (15 ep)",
                      seed_ids=ids, finals=finals, curves=curves, point_step=None, dash=i,
                      fallback=fallback))
    # ---- dv3 ------------------------------------------------------------
    if src == "dH" and D["dv3"]:
        seeds = sorted(D["dv3"], key=int)
        curves = {int(s): dedupe(D["dv3"][s]["inloop"]) for s in seeds}
        S.append(dict(alg="DV3", label="dH_DV3 (msrecipe)", n=len(seeds),
                      wave="msrecipe ar4 wave", era="cluster, 08-14, ~3.2e5 steps",
                      proto="metric = best fresh eval (6 ep); curve = in-loop eval (6 ep)",
                      seed_ids=[int(s) for s in seeds],
                      finals=[max([e["picked"] for e in D["dv3"][s]["fresh"]] or [0]) for s in seeds],
                      curves=curves, point_step=None, dash=0))
    return S

NOT_RUN = {
    "dH":  [],
    "dDP": [("DV3", "dDP_DV3 — NOT RUN (closed under the dH_DV3 null)")],
    "dR2D":[("DP",  "dR2D_DP — NOT RUN"),
            ("R2D", "dR2D_R2D — NOT RUN (dR2D demos come FROM r2dreamer)"),
            ("DV3", "dR2D_DV3 — NOT RUN")],
}
SRC_TITLE = {"dH": "dH — human demonstrations",
             "dDP": "dDP — DP-model demonstrations",
             "dR2D": "dR2D — r2dreamer-champion (clean in-sim) demonstrations"}

# --------------------------------------------------------------- FIGURE A ---
def fig_A(src):
    S = build(src)
    fig, ax = plt.subplots(figsize=(3.1 + 1.85 * (len(S) + len(NOT_RUN[src])), 6.4))
    rng = np.random.RandomState(7)
    xt, xl = [], []
    for i, s in enumerate(S):
        c = s.get("color") or ALG_COLOR[s["alg"]]
        vals = [v for v in s["finals"] if v is not None]
        miss = len(s["finals"]) - len(vals)
        fb = s.get("fallback") or [False] * len(s["finals"])
        fb = [fb[k] for k, v in enumerate(s["finals"]) if v is not None]
        jit = rng.uniform(-0.16, 0.16, len(vals))
        va = np.array(vals); fba = np.array(fb, dtype=bool)
        for mask, mk, lw in ((~fba, "o", 0.6), (fba, "^", 0.6)):
            z = (va == 0) & mask
            nz = (va != 0) & mask
            # zero-valued seeds get an open marker so a stack at y=0 stays countable
            ax.scatter(i + jit[nz], va[nz], s=70, color=c, marker=mk,
                       edgecolor="k", linewidth=lw, zorder=3)
            ax.scatter(i + jit[z], va[z], s=70, facecolor="none", marker=mk,
                       edgecolor=c, linewidth=1.4, zorder=3)
        m = float(np.mean(vals)) if vals else float("nan")
        if vals:
            ax.plot([i - 0.3, i + 0.3], [m, m], color=c, lw=3, zorder=4)
        ig = sum(1 for v in vals if v >= BAR)
        ax.text(i, -0.055, f"n={len(vals)}", ha="center", fontsize=9)
        ax.text(i, -0.095, f"ignited {ig}/{len(vals)}", ha="center", fontsize=9,
                color=c, fontweight="bold")
        ax.text(i, -0.132, f"mean {m:.2f}", ha="center", fontsize=8.5, color=c)
        if any(fb):
            ax.text(i, -0.168, f"^ {sum(fb)} in-loop fallback", ha="center", fontsize=7,
                    color="0.35")
        if miss: ax.text(i, -0.168, f"({miss} no eval at all)", ha="center", fontsize=7, color="0.4")
        xt.append(i)
        xl.append(s["alg"] + "\n" + "\n".join(textwrap.wrap(s["wave"], 24)))
    for j, (alg, txt) in enumerate(NOT_RUN[src]):
        i = len(S) + j
        ax.text(i, 0.5, "NOT RUN", ha="center", va="center", fontsize=11,
                color="0.55", rotation=90, fontweight="bold")
        ax.axvspan(i - 0.45, i + 0.45, color="0.93", zorder=0)
        xt.append(i); xl.append(f"{alg}\n(no runs)")
        ax.text(i, -0.095, txt.split("—")[0].strip(), ha="center", fontsize=7.5, color="0.45")
        ax.text(i, -0.132, "n=0", ha="center", fontsize=8.5, color="0.45")
    ax.axhline(BAR, color="k", ls="--", lw=1.2)
    ax.text(-0.55, 0.99, f"- - -  ignition bar = {BAR:.2f}   (RLPD registered bar: >=3/15 fresh-process demo-IC)",
            ha="left", va="top", fontsize=8.5,
            bbox=dict(fc="white", ec="0.7", lw=0.6, boxstyle="round,pad=0.35"))
    ax.set_xticks(xt); ax.set_xticklabels(xl, fontsize=8.5)
    ax.set_xlim(-0.6, len(xt) - 0.4)
    ax.set_ylim(-0.20, 1.06)
    ax.set_yticks(np.arange(0, 1.01, 0.1))
    ax.set_ylabel("per-seed final pick rate, demo ICs\n(open marker = exactly 0.00;  triangle = in-loop eval fallback)")
    ax.set_title(f"(A) Ignition distribution — {SRC_TITLE[src]}\n"
                 "one dot per seed; metrics are NOT protocol-matched across algorithms (see FIGNOTES)",
                 fontsize=10.5)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    p = os.path.join(FIG, f"ignition_bimodal_{src}_20260818.png")
    fig.savefig(p, dpi=170); plt.close(fig); print("wrote", p)
    return S

# ------------------------------------------------------------- FIGURE B/C ---
def fig_perf(src, S, ignited_only):
    fig, ax = plt.subplots(figsize=(11.0, 7.0))
    used = False
    for si, s in enumerate(S):
        c = s.get("color") or ALG_COLOR[s["alg"]]
        keep = list(range(len(s["finals"])))
        if ignited_only:
            keep = [i for i, v in enumerate(s["finals"]) if v is not None and v >= BAR]
        nlab = len(keep)
        if s["curves"] is None:                       # DP: single-point final eval
            vals = [s["finals"][i] for i in keep]
            if not vals:
                ax.plot([], [], color=c, ls=DASH[s["dash"]],
                        label=f'{s["label"]}  n=0/{s["n"]} ignited — no series'); continue
            m = float(np.mean(vals)); se = float(np.std(vals, ddof=1) / np.sqrt(len(vals))) if len(vals) > 1 else 0.0
            ax.axhspan(m - se, m + se, color=c, alpha=0.13, zorder=0)
            ax.axhline(m, color=c, ls=DASH[s["dash"]], lw=2,
                       label=f'{s["label"]}  n={nlab}  [{s["wave"]}; final-ckpt eval only, no step curve]')
            ax.errorbar([s["point_step"]], [m], yerr=[se], color=c, marker="D", ms=9,
                        capsize=5, lw=2, zorder=5)
            for v in vals:
                ax.plot([s["point_step"]], [v], marker="o", ms=4.5, mfc="none", mec=c, zorder=4)
            used = True; continue
        keys = sorted(s["curves"])
        keys = [keys[i] for i in keep]
        if not keys:
            ax.plot([], [], color=c, ls=DASH[s["dash"]],
                    label=f'{s["label"]}  n=0/{s["n"]} ignited — no series'); continue
        # bin onto a common grid (eval schedules are regular but offset by a few steps)
        G = 25000 if s["alg"] == "RLPD" else 100000
        per = {}
        for k in keys:
            for x, y in s["curves"][k]:
                per.setdefault(int(round(x / G)) * G, {})[k] = y
        bins = sorted(per)
        if not bins: continue
        nb = np.array([len(per[b]) for b in bins])          # seeds reporting per bin
        xs = np.array(bins, dtype=float)
        Yl = [[per[b].get(k) for b in bins] for k in keys]
        L = len(bins)
        m = np.array([float(np.mean([v for v in col if v is not None]))
                      for col in zip(*Yl)])
        se = np.array([(float(np.std([v for v in col if v is not None], ddof=1) /
                              np.sqrt(len([v for v in col if v is not None])))
                        if len([v for v in col if v is not None]) > 1 else 0.0)
                       for col in zip(*Yl)])
        for row in Yl:
            xr = [x for x, v in zip(xs, row) if v is not None]
            yr = [v for v in row if v is not None]
            ax.plot(xr, yr, color=c, alpha=0.20, lw=0.9, zorder=1)
        full = nb == len(keys)
        drop = None if full.all() else int(np.argmax(~full))
        ax.plot(xs[full], m[full], color=c, ls=DASH[s["dash"]], lw=2.2, marker="o", ms=4,
                zorder=3,
                label=f'{s["label"]}  n={nlab}' + (f'/{s["n"]}' if ignited_only else '') +
                      f'  [{s["era"]}]')
        if drop is not None:
            ax.plot(xs[drop - 1:], m[drop - 1:], color=c, ls=DASH[s["dash"]], lw=1.2,
                    alpha=0.65, marker="o", ms=3, zorder=3)
            ax.plot(xs[drop:], m[drop:], ls="", marker="x", ms=5, color=c, zorder=4)
        ax.fill_between(xs, np.maximum(m - se, 0), m + se, color=c, alpha=0.18, zorder=2)
        if float(np.nanmax(m)) == 0.0:   # flat-zero series: make it legible, do not drop it
            off = 0.006 + 0.011 * si          # small y-offset so flat-zero series stay separable
            ax.plot(xs, np.full(L, off), color=c, ls=DASH[s["dash"]], lw=2.6, zorder=3)
        used = True
    for alg, txt in NOT_RUN[src]:
        ax.plot([], [], color=ALG_COLOR[alg], ls="", marker="x", ms=8, label=txt)
    ax.set_xscale("log")
    ax.set_xlim(1.5e4, 3.6e6)
    ax.set_ylim(0, 1.02)
    ax.set_xlabel("environment steps (LOG scale — budgets differ by 30x across learners)")
    ax.set_ylabel("pick rate on demo ICs (mean over seeds, band = +/- 1 SE)")
    kind = "(C) performance GIVEN ignition" if ignited_only else "(B) performance, ALL seeds"
    extra = (f"ignited = final/best metric >= {BAR:.2f}" if ignited_only
             else "non-ignited seeds included — they pull the mean down; thin lines = individual seeds")
    ax.set_title(f"{kind} — {SRC_TITLE[src]}\n{extra}\n"
                 "No smoothing. Protocols differ per series; see FIGNOTES_ignition_20260818.md.",
                 fontsize=10)
    ax.grid(alpha=0.25)
    fig.text(0.012, 0.008,
             "Series that are flat 0.00 are drawn with a small y-offset so they stay visible "
             "(they are exactly 0.00). Thin lines = individual seeds.",
             fontsize=7.5, color="0.3")
    ax.plot([], [], ls="", marker="x", ms=6, color="0.35",
            label="x on a mean marker = that bin has fewer than n seeds reporting")
    ax.legend(fontsize=7.6, loc="upper left", framealpha=0.92)
    fig.tight_layout(rect=(0, 0.022, 1, 1))
    tag = "ignited" if ignited_only else "all"
    p = os.path.join(FIG, f"perf_{tag}_{src}_20260818.png")
    fig.savefig(p, dpi=170); plt.close(fig); print("wrote", p)

# -------------------------------------------------------------- OVERVIEW ---
def fig_overview(ALL):
    rows = []
    for src in ["dH", "dDP", "dR2D"]:
        for s in ALL[src]:
            vals = [v for v in s["finals"] if v is not None]
            rows.append((src, s, vals))
        for alg, txt in NOT_RUN[src]:
            rows.append((src, dict(alg=alg, label=txt, wave="", n=0), None))
    fig, ax = plt.subplots(figsize=(13.0, 0.52 * len(rows) + 2.9))
    rng = np.random.RandomState(3)
    yt, yl = [], []
    for i, (src, s, vals) in enumerate(rows):
        y = len(rows) - 1 - i
        if vals is None:
            ax.axhspan(y - 0.42, y + 0.42, color="0.94", zorder=0)
            ax.text(0.5, y, "NOT RUN", va="center", ha="center", fontsize=9.5,
                    color="0.5", fontweight="bold")
            yt.append(y); yl.append(f'{src} | {s["alg"]}'); continue
        c = s.get("color") or ALG_COLOR[s["alg"]]
        jit = rng.uniform(-0.19, 0.19, len(vals))
        v = np.array(vals); z = v == 0
        ax.scatter(v[~z], y + jit[~z], s=52, color=c, edgecolor="k", lw=0.5, zorder=3)
        ax.scatter(v[z], y + jit[z], s=52, facecolor="none", edgecolor=c, lw=1.3, zorder=3)
        m = float(np.mean(vals))
        ax.plot([m, m], [y - 0.33, y + 0.33], color=c, lw=3, zorder=4)
        ig = sum(1 for q in vals if q >= BAR)
        ax.text(1.015, y, f"n={len(vals)}  ignited {ig}/{len(vals)}  mean {m:.2f}",
                va="center", fontsize=8, color=c)
        yt.append(y); yl.append(f'{src} | {s["alg"]} | {s["wave"]}')
    ax.axvline(BAR, color="k", ls="--", lw=1.2)
    ax.text(BAR + 0.008, len(rows) - 0.4, f"ignition bar {BAR:.2f}", fontsize=8.5)
    ax.set_yticks(yt); ax.set_yticklabels(yl, fontsize=8)
    ax.set_xlim(-0.03, 1.52); ax.set_ylim(-0.7, len(rows) - 0.3)
    ax.set_xticks(np.arange(0, 1.01, 0.1))
    ax.set_xlabel("per-seed final pick rate, demo ICs (open marker = exactly 0.00)")
    ax.set_title("Overview — every demo source x learner cell that exists, one dot per seed\n"
                 "Metrics are NOT protocol-matched across rows\n"
                 "(episode counts 6/10/15, horizons 400/1200, budgets 1e5-3e6)\n"
                 "See paper/figs/FIGNOTES_ignition_20260818.md", fontsize=10.5)
    ax.grid(axis="x", alpha=0.25)
    fig.tight_layout()
    p = os.path.join(FIG, "overview_ignition_20260818.png")
    fig.savefig(p, dpi=170); plt.close(fig); print("wrote", p)

ALL = {}
for src in ["dH", "dDP", "dR2D"]:
    ALL[src] = fig_A(src)
    fig_perf(src, ALL[src], False)
    fig_perf(src, ALL[src], True)
fig_overview(ALL)
