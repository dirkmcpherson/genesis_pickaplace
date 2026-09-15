#!/usr/bin/env python3
"""By-phase analysis of the `nested_sparse10` PIXEL conditions.

Produces, for every learner x arm condition of amendments (ad)/(ae)/(af)/(ag):

  1. learning curves   -- rolling-30-episode mean of each per-episode stage flag
                          against online sim steps, mean + 95 % bootstrap CI over seeds;
  2. rise time         -- first online step at which the rolling-30 rate crosses a
                          threshold, per run, tabulated as median [min, max] over seeds;
  3. steady state      -- (a) training record over the last 20 % of each run,
                          (b) eval cells (rnd30 MODE) at >= 0.5 M online steps;
  4. ignition rate     -- fraction of seeds whose rolling-30 `home` reaches >= 0.5
                          inside budget, and fraction with >= 1 `home` in any rnd30 cell.

Two kinds of number live here and are never mixed:
  * TRAINING RECORD  -- sampled actions, the policy's own training starts, sticky
                        per-episode stage flags read from the trainer's console log.
  * EVAL CELLS       -- deterministic (mode) actions, fixed starts, a fresh process
                        per cell, read from each cell's metrics.json.

All input is a local rsync mirror of the cluster and of ~/runs_dv3_local; nothing on
the cluster is read or written by this script.

Usage:
    python3 baselines/diagnostics/px_phase_analysis.py \
        --data-root ~/data/genesis_pickaplace/px_analysis_2026-09-14 \
        --out-dir   paper/figures/px_phase_2026-09-14 \
        --tex       paper/figures/px_rise_time_by_phase.tex
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
import textwrap
from collections import OrderedDict
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ----------------------------------------------------------------------------- config

LADDER = "nested_sparse10"
OBS = "pixels"

#: phases reported, in ladder order.  `placed` prefers the `placed_v2` field.
PHASES = ["picked", "placed_v2", "farside", "slide_event", "home", "tipped"]
PHASE_LABEL = {
    "picked": "picked",
    "placed_v2": "placed (v2)",
    "farside": "farside",
    "slide_event": "slide_event",
    "home": "home",
    "tipped": "tipped",
}

ROLL = 30           # episodes in the rolling window
GRID = 50_000       # online-step grid for the CI bands
MIN_SEEDS_BAND = 3  # truncate a band where fewer than this many seeds have data
BOOT = 2000         # bootstrap resamples
BOOT_SEED = 20260914

ARM_OF_SET = {
    "dHfull_all": "human",
    "dDPfull_first": "machine",
    "dH": "human",
    "dDPfirst": "machine",
}
ARM_COLOR = {"human": "#1f6fb4", "machine": "#c3562a"}

#: compact axis-tick codes (the full {algorithm} names live in the captions)
TICK = {
    "dreamer_local": "dv3 loc",
    "dreamer_cluster": "dv3 clu",
    "r2dreamer_cluster": "r2d clu",
    "rlpd_cluster": "RLPD",
    "dreamer_ramp_control": "ramp ctl",
}
ARM_TICK = {"human": "H", "machine": "M"}

#: run-level notes that the data alone cannot carry.
RUN_NOTES = {
    # the (ad) run was launched at 2 M and deliberately stopped at its 1 M milestone
    # by the (ae) chain launcher (CLAUDE.md, 2026-09-12 19:20).  Not an interruption.
    ("dreamer_local", "human", 0): ("stopped_at_1M_by_chain", 1_000_000),
}

CONDITIONS = OrderedDict([
    ("dreamer_local", dict(
        algorithm="dv3 (DreamerV3 losses in the r2dreamer chassis), pixels",
        short="{dv3} local",
        hardware="pop-os workstation, AMD 5950X (AVX2) + local GPU",
        source="local")),
    ("dreamer_cluster", dict(
        algorithm="dv3 (DreamerV3 losses in the r2dreamer chassis), pixels",
        short="{dv3} cluster",
        hardware="Tufts pax cluster GPU nodes",
        source="cluster")),
    ("r2dreamer_cluster", dict(
        algorithm="r2dreamer (the port's contrastive representation loss), pixels",
        short="{r2dreamer} cluster",
        hardware="Tufts pax cluster GPU nodes",
        source="cluster")),
    ("rlpd_cluster", dict(
        algorithm="RLPD (DrQ-style shared encoder, LN critic ensemble), pixels",
        short="{RLPD} cluster",
        hardware="Tufts pax cluster GPU nodes",
        source="cluster")),
    ("dreamer_ramp_control", dict(
        algorithm="dv3, pixels -- nested_ramp CONTROL (not sparse10)",
        short="{dv3} ramp control",
        hardware="Tufts pax cluster GPU nodes",
        source="cluster")),
])

EPI_RE = re.compile(r"^\[(\d+)\]\s+episode/")
FIELD_RE = re.compile(r"episode/train_ep_([a-z0-9_]+)\s+([-\d.eE+]+)")
STEPS_RE = re.compile(r"env\.steps=(\d+)")


# ------------------------------------------------------------------------- data model

class Run:
    """One training run: its per-episode stage record and its eval cells."""

    def __init__(self, condition, arm, seed, name, path, source):
        self.condition = condition
        self.arm = arm
        self.seed = seed
        self.name = name
        self.path = path
        self.source = source
        self.origin = None
        self.nominal_budget = None
        self.online = np.zeros(0, dtype=np.int64)     # online sim steps, per episode
        self.flags = {}                               # phase -> array or None (absent)
        self.cells = []                               # list of dicts (eval cells)
        self.note = ""
        self.x_unit = "online sim steps"

    # -- derived ------------------------------------------------------------------
    @property
    def n_episodes(self):
        return int(self.online.size)

    @property
    def achieved(self):
        return int(self.online[-1]) if self.online.size else 0

    @property
    def budget(self):
        """Budget actually available to this run (see RUN_NOTES for overrides)."""
        key = (self.condition, self.arm, self.seed)
        if key in RUN_NOTES:
            return RUN_NOTES[key][1]
        return self.nominal_budget or self.achieved

    @property
    def complete(self):
        return self.achieved >= 0.99 * self.budget

    def rolling(self, phase):
        """(online steps, rolling-`ROLL`-episode mean) or (None, None) if absent."""
        v = self.flags.get(phase)
        if v is None or v.size < ROLL:
            return None, None
        c = np.cumsum(np.concatenate([[0.0], v]))
        roll = (c[ROLL:] - c[:-ROLL]) / ROLL
        return self.online[ROLL - 1:], roll

    def rise(self, phase, thresh):
        """First online step where the rolling mean reaches `thresh`; None if never."""
        x, y = self.rolling(phase)
        if x is None:
            return None
        hit = np.nonzero(y >= thresh)[0]
        return int(x[hit[0]]) if hit.size else None

    def first_event(self, phase):
        """Online step of the first episode that reached `phase`; None if never."""
        v = self.flags.get(phase)
        if v is None:
            return None
        hit = np.nonzero(v > 0.5)[0]
        return int(self.online[hit[0]]) if hit.size else None

    def steady_train(self, phase):
        """Mean of a phase flag over the last 20 % of the run's online steps."""
        v = self.flags.get(phase)
        if v is None or v.size == 0:
            return None
        cut = self.achieved * 0.8
        sel = self.online >= cut
        if sel.sum() < 5:
            return None
        return float(v[sel].mean())

    def eval_cells(self, cell="rnd30_mode", min_online=500_000):
        return [c for c in self.cells
                if c["cell"] == cell and c["online"] >= min_online]

    def steady_eval(self, phase, cell="rnd30_mode", min_online=500_000):
        vals = [c["stages"].get(phase) for c in self.eval_cells(cell, min_online)]
        vals = [v for v in vals if v is not None]
        return (float(np.mean(vals)), len(vals)) if vals else (None, 0)

    def any_eval_home(self, cell="rnd30_mode"):
        """True if >= 1 `home` episode in any cell of that kind (any milestone)."""
        got = False
        for c in self.cells:
            if c["cell"] != cell:
                continue
            h = c["stages"].get("home")
            if h is None:
                continue
            got = True
            if h > 0:
                return True
        return False if got else None


# ---------------------------------------------------------------------------- parsing

def parse_console(path: Path, phases=PHASES):
    """Read a trainer console.log -> (origin, nominal budget, online steps, flags)."""
    counters, rows = [], []
    nominal = None
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            if nominal is None:
                m = STEPS_RE.search(line)
                if m:
                    nominal = int(m.group(1))
            m = EPI_RE.match(line)
            if not m:
                continue
            counters.append(int(m.group(1)))
            rows.append(dict(FIELD_RE.findall(line)))
    if not counters:
        return None
    origin = counters[0]
    online = np.asarray(counters, dtype=np.int64) - origin
    flags = {}
    for p in phases:
        key = p
        if p == "placed_v2" and not any("placed_v2" in r for r in rows[:50]):
            key = "placed"                       # fall back only if v2 is absent
        if not any(key in r for r in rows[:50]):
            flags[p] = None                      # ABSENT is not zero
            continue
        flags[p] = np.asarray([float(r.get(key, "nan")) for r in rows], dtype=float)
    return origin, nominal, online, flags


def parse_rlpd_jsonl(path: Path, phases=PHASES, repeat=4):
    """Read episode_rollouts.jsonl -> (online sim frames, flags).  step = decisions."""
    steps, rows = [], []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "step" not in d:
                continue
            steps.append(int(d["step"]))
            rows.append(d)
    if not steps:
        return None
    online = np.asarray(steps, dtype=np.int64) * repeat
    flags = {}
    for p in phases:
        key = p
        probe = f"episode/train_ep_{key}"
        if p == "placed_v2" and not any(probe in r for r in rows[:50]):
            key, probe = "placed", "episode/train_ep_placed"
        if not any(probe in r for r in rows[:50]):
            flags[p] = None                      # farside / slide_event / home: ABSENT
            continue
        flags[p] = np.asarray([float(r.get(probe, np.nan)) for r in rows], dtype=float)
    return online, flags


def read_cell(mpath: Path):
    try:
        d = json.load(open(mpath))
    except Exception:
        return None
    st = d.get("headline_stages") or d.get("stages") or {}
    per = d.get("per_episode") or []
    return dict(stages={k: (float(v) if v is not None else None) for k, v in st.items()},
                episodes=int(d.get("episodes") or len(per)),
                path=str(mpath))


# --------------------------------------------------------------------------- discovery

WM_RE = re.compile(r"^full_r2d_state_(dHfull_all|dDPfull_first)_(rns10h|rnrh)_img_"
                   r"(dreamer|r2dreamer)_s(\d+)$")
LOCAL_RE = re.compile(r"^dv3px_sparse10_(dHfull_all|dDPfull_first)_rns10h_img_"
                      r"rlDreamer_s(\d+)$")
RLPD_RE = re.compile(r"^e2e_rlpd_px_(dH|dDPfirst)_s(\d+)$")


def discover(root: Path, verbose=True):
    runs = []

    # --- cluster world-model runs -------------------------------------------------
    rdir = root / "W" / "runs"
    cellroot = root / "W" / "ln_milestone_cells"
    for d in sorted(rdir.glob("full_r2d_state_*")):
        m = WM_RE.match(d.name)
        if not m or "smoke" in d.name:
            continue
        dset, ladder_tag, rep, seed = m.group(1), m.group(2), m.group(3), int(m.group(4))
        if ladder_tag == "rnrh":
            cond = "dreamer_ramp_control"
        else:
            cond = f"{rep}_cluster"
        arm = ARM_OF_SET[dset]
        log = d / "console.log"
        if not log.exists():
            continue
        parsed = parse_console(log)
        if parsed is None:
            continue
        origin, nominal, online, flags = parsed
        r = Run(cond, arm, seed, d.name, str(d), "cluster")
        r.origin, r.nominal_budget, r.online, r.flags = origin, nominal, online, flags
        sc = d / "step_contract.json"
        if sc.exists():
            try:
                j = json.load(open(sc))
                r.origin = int(j.get("prefill_counter_origin", r.origin))
                r.nominal_budget = int(j.get("requested_online_sim_steps",
                                             r.nominal_budget or 0)) or r.nominal_budget
            except Exception:
                pass
        # eval cells
        cd = cellroot / d.name
        if cd.is_dir():
            for md in sorted(cd.glob("online_*")):
                try:
                    ms = int(md.name.split("_")[1])
                except ValueError:
                    continue
                for cell in ("rnd30_mode", "hold15_mode"):
                    c = read_cell(md / cell / "metrics.json")
                    if c:
                        c.update(cell=cell, online=ms, milestone=md.name)
                        r.cells.append(c)
        runs.append(r)

    # --- local world-model runs ---------------------------------------------------
    ldir = root / "local" / "runs_dv3_local"
    for d in sorted(ldir.glob("dv3px_sparse10_*")):
        m = LOCAL_RE.match(d.name)
        if not m:
            continue
        dset, seed = m.group(1), int(m.group(2))
        arm = ARM_OF_SET[dset]
        log = d / "console.log"
        if not log.exists():
            continue
        parsed = parse_console(log)
        if parsed is None:
            continue
        origin, nominal, online, flags = parsed
        r = Run("dreamer_local", arm, seed, d.name, str(d), "local")
        r.origin, r.nominal_budget, r.online, r.flags = origin, nominal, online, flags
        # series cells: dv3px_sparse10_series[_<dH|dM>_s<seed>]/ck_<counter>/fresh_eval_*
        tag = "dH" if arm == "human" else "dM"
        sd = ldir / f"dv3px_sparse10_series_{tag}_s{seed}"
        if not sd.is_dir() and arm == "human" and seed == 0:
            sd = ldir / "dv3px_sparse10_series"          # human s0 = the (ad) run
        if sd.is_dir():
            for ck in sorted(sd.glob("ck_*")):
                try:
                    counter = int(ck.name.split("_")[1])
                except ValueError:
                    continue
                for cell in ("rnd30_mode", "hold15_mode"):
                    c = read_cell(ck / f"fresh_eval_{cell}" / "metrics.json")
                    if c:
                        c.update(cell=cell, online=counter - origin, milestone=ck.name)
                        r.cells.append(c)
        runs.append(r)

    # --- RLPD ---------------------------------------------------------------------
    pdir = root / "LAB" / "gp_pxr" / "e2e_px"
    for d in sorted(pdir.glob("e2e_rlpd_px_*")):
        m = RLPD_RE.match(d.name)
        if not m:
            continue
        armtag, seed = m.group(1), int(m.group(2))
        if seed >= 9000:                                   # smoke
            continue
        jl = d / "episode_rollouts.jsonl"
        if not jl.exists():
            continue
        parsed = parse_rlpd_jsonl(jl)
        if parsed is None:
            continue
        online, flags = parsed
        r = Run("rlpd_cluster", ARM_OF_SET[armtag], seed, d.name, str(d), "cluster")
        r.origin = 0
        r.nominal_budget = 250_000 * 4                     # 250 k decisions x repeat 4
        r.online, r.flags = online, flags
        r.x_unit = "online sim frames (decisions x 4)"
        for cell in ("rnd30_mode", "hold15_mode"):
            c = read_cell(d / f"fresh_eval_{cell}" / "metrics.json")
            if c:
                c.update(cell=cell, online=r.nominal_budget, milestone="final")
                r.cells.append(c)
        runs.append(r)

    for r in runs:
        key = (r.condition, r.arm, r.seed)
        if key in RUN_NOTES:
            r.note = RUN_NOTES[key][0]
        elif not r.complete:
            r.note = "incomplete (still running or interrupted)"

    if verbose:
        print(f"[discover] {len(runs)} runs")
    return runs


# --------------------------------------------------------------------------- analysis

def wilson(k, n, z=1.959963985):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def bootstrap_band(mat, rng, reps=BOOT):
    """mat: (seeds, grid) with NaN where a seed has no data.  -> mean, lo, hi, n."""
    n_pts = mat.shape[1]
    mean = np.full(n_pts, np.nan)
    lo = np.full(n_pts, np.nan)
    hi = np.full(n_pts, np.nan)
    nseed = np.zeros(n_pts, dtype=int)
    for j in range(n_pts):
        col = mat[:, j]
        col = col[~np.isnan(col)]
        nseed[j] = col.size
        if col.size == 0:
            continue
        mean[j] = col.mean()
        if col.size >= 2:
            idx = rng.integers(0, col.size, size=(reps, col.size))
            bs = col[idx].mean(axis=1)
            lo[j], hi[j] = np.percentile(bs, [2.5, 97.5])
    return mean, lo, hi, nseed


def condition_band(runs, phase, grid, rng):
    mat = np.full((len(runs), grid.size), np.nan)
    for i, r in enumerate(runs):
        x, y = r.rolling(phase)
        if x is None:
            continue
        inside = (grid >= x[0]) & (grid <= x[-1])
        mat[i, inside] = np.interp(grid[inside], x, y)
    mean, lo, hi, nseed = bootstrap_band(mat, rng)
    mask = nseed >= MIN_SEEDS_BAND
    return mean, lo, hi, nseed, mask


def group(runs):
    """condition -> arm -> [runs], seed-sorted."""
    out = OrderedDict()
    for cond in CONDITIONS:
        arms = OrderedDict()
        for arm in ("human", "machine"):
            sel = sorted([r for r in runs if r.condition == cond and r.arm == arm],
                         key=lambda r: r.seed)
            if sel:
                arms[arm] = sel
        if arms:
            out[cond] = arms
    return out


# ---------------------------------------------------------------------------- figures

def wrap_title(ax, text, width=112, fontsize=7.2):
    ax.set_title("\n".join(textwrap.wrap(text, width)), fontsize=fontsize)


def fig_caption(fig, text, width=120, fontsize=7.0, pad=0.008):
    """Bottom caption; returns the fraction of figure height it consumes."""
    lines = textwrap.wrap(text, width)
    fig.text(0.01, 0.005, "\n".join(lines), fontsize=fontsize, ha="left", va="bottom")
    return (len(lines) * fontsize * 1.25) / (fig.get_size_inches()[1] * 72.0) + pad


def fmt_k(v):
    return "n/a" if v is None else f"{v / 1000.0:.0f}k"


def caption_bits(cond, arms):
    meta = CONDITIONS[cond]
    ns = ", ".join(f"{a} n={len(rs)}" for a, rs in arms.items())
    buds = sorted({r.budget for rs in arms.values() for r in rs})
    bud = "/".join(f"{b/1e6:.2g}M" for b in buds)
    return meta, ns, bud


def fig_learning_curves(cond, arms, outdir, rng):
    meta, ns, bud = caption_bits(cond, arms)
    maxb = max(r.budget for rs in arms.values() for r in rs)
    grid = np.arange(0, maxb + GRID, GRID)
    ncol, nrow = 3, 2
    fig, axes = plt.subplots(nrow, ncol, figsize=(7.1, 4.5), sharex=True)
    # with fewer than MIN_SEEDS_BAND seeds per arm a CI band is not defensible;
    # draw the individual seed traces instead and say so in the caption.
    per_seed_mode = min(len(rs) for rs in arms.values()) < MIN_SEEDS_BAND
    rows = []
    for k, phase in enumerate(PHASES):
        ax = axes[k // ncol][k % ncol]
        any_data = False
        absent_all = all(r.flags.get(phase) is None
                         for rs in arms.values() for r in rs)
        for arm, rs in arms.items():
            mean, lo, hi, nseed, mask = condition_band(rs, phase, grid, rng)
            for j in range(grid.size):
                if nseed[j]:
                    rows.append(dict(condition=cond, arm=arm, phase=phase,
                                     online_step=int(grid[j]), mean=mean[j],
                                     ci_lo=lo[j], ci_hi=hi[j], n_seeds=int(nseed[j]),
                                     in_band=bool(mask[j])))
            if per_seed_mode:
                for r in rs:
                    x, y = r.rolling(phase)
                    if x is None:
                        continue
                    any_data = True
                    ax.plot(x / 1e6, y, color=ARM_COLOR[arm], lw=0.9, alpha=0.85)
                continue
            if mask.sum() == 0:
                continue
            any_data = True
            gx = grid[mask]
            ax.plot(gx / 1e6, mean[mask], color=ARM_COLOR[arm], lw=1.3, label=arm)
            ok = ~np.isnan(lo[mask])
            if ok.any():
                ax.fill_between(gx[ok] / 1e6, lo[mask][ok], hi[mask][ok],
                                color=ARM_COLOR[arm], alpha=0.18, lw=0)
        ax.set_title(PHASE_LABEL[phase], fontsize=8)
        ax.set_ylim(-0.03, 1.03)
        ax.set_xlim(0, maxb / 1e6)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.25, lw=0.4)
        if not any_data:
            msg = ("not recorded\n(absent, not zero)" if absent_all
                   else "no seed has\n%d episodes yet" % ROLL)
            ax.text(0.5, 0.5, msg, ha="center",
                    va="center", fontsize=7, color="0.4", transform=ax.transAxes)
        if k % ncol == 0:
            ax.set_ylabel("rolling-30 rate", fontsize=8)
        if k // ncol == nrow - 1:
            ax.set_xlabel("online sim steps (M)", fontsize=8)
    handles = [Line2D([], [], color=ARM_COLOR[a], lw=1.5, label=f"{a} (n={len(rs)})")
               for a, rs in arms.items()]
    axes[0][0].legend(handles=handles, fontsize=6.5, loc="lower right", frameon=False)
    extra = ""
    if cond == "rlpd_cluster":
        extra = (" x-axis = decisions x action_repeat 4 = sim frames;"
                 " farside/slide_event/home are ABSENT from the RLPD episode record"
                 " (absent, not zero).")
    if cond == "dreamer_local":
        extra = " Hardware: " + CONDITIONS[cond]["hardware"] + "."
    if cond == "dreamer_ramp_control":
        extra = (f" CONTROL condition: the ladder is nested_ramp, NOT {LADDER}; it is "
                 f"not comparable with the sparse10 conditions and is shown for "
                 f"reference only.")
    band = (f"one line per seed (no CI band: fewer than {MIN_SEEDS_BAND} seeds per arm)"
            if per_seed_mode else
            f"line = mean over seeds, band = 95 % bootstrap CI ({BOOT} resamples), "
            f"truncated where fewer than {MIN_SEEDS_BAND} seeds have data")
    ladder = "nested_ramp" if cond == "dreamer_ramp_control" else LADDER
    body = (f"observation: {OBS}; ladder: {ladder}; {ns}; budget {bud}; "
            f"TRAINING RECORD, sampled actions, policy's own training starts; "
            f"{band}." + extra)
    head = f"{meta['short']} -- {meta['algorithm']}"
    lines = [head] + textwrap.wrap(body, 118)
    fig.tight_layout()
    fig_h_pt = fig.get_size_inches()[1] * 72.0
    top = max(0.55, 1.0 - (len(lines) * 8.8 + 8) / fig_h_pt - 0.045)
    fig.subplots_adjust(top=top)
    fig.text(0.5, 0.995, "\n".join(lines), fontsize=7.2, ha="center", va="top")
    save(fig, outdir / f"fig_learning_curves_{cond}")
    return rows


def fig_steady_state(groups, outdir):
    conds = list(groups)
    fig, axes = plt.subplots(2, 1, figsize=(7.1, 6.4))
    rows = []

    # (a) training record, last 20 % of the run
    ax = axes[0]
    xt, xl = [], []
    pos = 0
    for cond in conds:
        for arm, rs in groups[cond].items():
            vals = [(r.seed, r.steady_train("home"), r.complete) for r in rs]
            vals = [(sd, v, cp) for sd, v, cp in vals if v is not None]
            for sd, v, cp in vals:
                rows.append(dict(kind="training_record_last20pct", condition=cond,
                                 arm=arm, seed=sd, phase="home", value=v,
                                 run_complete=cp))
                kw = (dict(color=ARM_COLOR[arm], alpha=0.85, mew=0) if cp else
                      dict(mfc="none", mec=ARM_COLOR[arm], mew=0.8, alpha=0.9))
                ax.plot(pos + (np.random.RandomState(sd).rand() - 0.5) * 0.28, v, "o",
                        ms=3.4, **kw)
            if vals:
                m = float(np.mean([v for _, v, _ in vals]))
                ax.plot([pos - 0.28, pos + 0.28], [m, m], "-", color=ARM_COLOR[arm], lw=2)
                ax.text(pos, 1.02, f"n={len(vals)}", ha="center", fontsize=6, color="0.3")
            else:
                ax.text(pos, 0.5, "`home`\nabsent", ha="center",
                        va="center", fontsize=5.6, color="0.45")
            xt.append(pos)
            xl.append(f"{TICK[cond]}\n{ARM_TICK[arm]}")
            pos += 1
        pos += 0.6
    ax.set_xticks(xt); ax.set_xticklabels(xl, fontsize=6.2)
    ax.set_ylabel("`home` rate", fontsize=8); ax.set_ylim(-0.05, 1.12)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    wrap_title(ax, "(a) TRAINING RECORD -- mean `home` over the last 20 % of each run's "
                   "online steps (sampled actions, the policy's own training starts); "
                   "one point per seed, bar = arm mean, n above each column")

    # (b) eval cells, rnd30 MODE at >= 0.5 M
    ax = axes[1]
    xt, xl = [], []
    pos = 0
    for cond in conds:
        for arm, rs in groups[cond].items():
            pts = []
            for r in rs:
                hv, nh = r.steady_eval("home")
                pv, npk = r.steady_eval("picked")
                if hv is None:
                    continue
                pts.append((r.seed, hv, pv, nh))
                rows.append(dict(kind="eval_rnd30_mode_ge0.5M", condition=cond, arm=arm,
                                 seed=r.seed, phase="home", value=hv, n_cells=nh))
                if pv is not None:
                    rows.append(dict(kind="eval_rnd30_mode_ge0.5M", condition=cond,
                                     arm=arm, seed=r.seed, phase="picked", value=pv,
                                     n_cells=npk))
            for s, hv, pv, _ in pts:
                j = (np.random.RandomState(s).rand() - 0.5) * 0.28
                ax.plot(pos + j, hv, "o", ms=3.4, color=ARM_COLOR[arm], alpha=0.9, mew=0)
                if pv is not None:
                    ax.plot(pos + j, pv, "^", ms=3.2, color=ARM_COLOR[arm],
                            alpha=0.45, mew=0)
            if pts:
                m = float(np.mean([h for _, h, _, _ in pts]))
                ax.plot([pos - 0.28, pos + 0.28], [m, m], "-", color=ARM_COLOR[arm], lw=2)
                ax.text(pos, 1.02, f"n={len(pts)}", ha="center", fontsize=6, color="0.3")
            xt.append(pos); xl.append(f"{TICK[cond]}\n{ARM_TICK[arm]}")
            pos += 1
        pos += 0.6
    ax.set_xticks(xt); ax.set_xticklabels(xl, fontsize=6.2)
    ax.set_ylabel("rate over cells", fontsize=8); ax.set_ylim(-0.05, 1.12)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    ax.legend(handles=[Line2D([], [], ls="", marker="o", ms=4, color="0.3", label="home"),
                       Line2D([], [], ls="", marker="^", ms=4, color="0.6",
                              label="picked")],
              fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2,
              frameon=False)
    wrap_title(ax, "(b) EVAL CELLS -- mean over every rnd30 MODE milestone cell at "
                   ">= 0.5 M online steps (deterministic actions, 30 fixed random "
                   "starts, fresh process); one point per seed, bar = arm mean of `home`")

    fig.tight_layout()
    cap = (f"Steady-state performance.  {{dv3}} = DreamerV3 losses, {{r2dreamer}} = the "
           f"port's contrastive loss, {{RLPD}} = DrQ-style encoder in the LN critic "
           f"ensemble; observation {OBS}, ladder {LADDER}.  Ticks: dv3 loc / dv3 clu / "
           f"r2d clu / RLPD / ramp ctl, H = human demonstrations (dHfull_all, 74 tapes), "
           f"M = machine (dDPfull_first, 72 tapes).  Budgets: 1 M (ae/af) or 2 M (ag) "
           f"online sim steps; {{RLPD}} 250k decisions = 1 M sim frames.  {{dv3}} loc ran "
           f"on the pop-os AVX2 GPU workstation, every other condition on the pax "
           f"cluster.  `ramp ctl' uses nested_ramp, not {LADDER}.  {{RLPD}} logs no "
           f"`home` flag in its episode record, so panel (a) has no RLPD column "
           f"(absent, not zero).  In (a) a HOLLOW point is a run that has not finished "
           f"its budget: its last 20 % is not a steady state.  A seed with no eval cell "
           f"at >= 0.5 M is absent from (b), not zero.")
    h = fig_caption(fig, cap)
    fig.subplots_adjust(bottom=h + 0.08, hspace=0.62)
    save(fig, outdir / "fig_steady_state")
    return rows


def fig_ignition(groups, outdir):
    fig, ax = plt.subplots(figsize=(7.1, 3.9))
    rows = []
    labels, pos = [], 0
    w = 0.34
    for cond in groups:
        for arm, rs in groups[cond].items():
            # (i) training: rolling-30 home >= 0.5 within budget.  A seed that has
            #     neither crossed nor finished its budget is UNDETERMINED: it leaves
            #     the denominator and is reported separately.
            elig = [r for r in rs if r.flags.get("home") is not None]
            ign = [r for r in elig if r.rise("home", 0.5) is not None]
            pend = [r for r in elig if r not in ign and not r.complete]
            k_tr, n_tr = len(ign), len(elig) - len(pend)
            n_pend = len(pend)
            # (ii) eval: >= 1 home episode in any rnd30 MODE cell (runs with no cell
            #     yet are absent from the denominator, not zeros)
            ev = [r.any_eval_home("rnd30_mode") for r in rs]
            ev = [e for e in ev if e is not None]
            k_ev, n_ev = sum(1 for e in ev if e), len(ev)
            for tag, k, n, npd in (("train_roll30_home_ge0.5", k_tr, n_tr, n_pend),
                                   ("eval_rnd30_any_home", k_ev, n_ev,
                                    len(rs) - n_ev)):
                lo, hi = wilson(k, n)
                rows.append(dict(condition=cond, arm=arm, criterion=tag, k=k, n=n,
                                 rate=(k / n if n else None), wilson_lo=lo, wilson_hi=hi,
                                 n_undetermined=npd, n_seeds_total=len(rs)))
            for off, (k, n, hatch) in enumerate([(k_tr, n_tr, ""), (k_ev, n_ev, "///")]):
                if off == 0 and n_pend:
                    ax.text(pos + (off - 0.5) * w, 1.21, f"+{n_pend}?", ha="center",
                            fontsize=5.5, color="0.45")
                if n == 0:
                    ax.text(pos + (off - 0.5) * w, 0.04, "n/a", ha="center", fontsize=6,
                            color="0.45", rotation=90)
                    continue
                p = k / n
                lo, hi = wilson(k, n)
                ax.bar(pos + (off - 0.5) * w, p, width=w * 0.92,
                       color=ARM_COLOR[arm], alpha=0.85 if off == 0 else 0.45,
                       hatch=hatch, edgecolor="white", lw=0.5)
                ax.errorbar(pos + (off - 0.5) * w, p,
                            yerr=[[max(0.0, p - lo)], [max(0.0, hi - p)]],
                            fmt="none", ecolor="0.25", elinewidth=0.8, capsize=2)
                ax.text(pos + (off - 0.5) * w, 1.04 + 0.09 * off, f"{k}/{n}",
                        ha="center", fontsize=5.8, color="0.2")
            labels.append((pos, f"{TICK[cond]}\n{ARM_TICK[arm]}"))
            pos += 1
        pos += 0.6
    ax.set_xticks([p for p, _ in labels])
    ax.set_xticklabels([l for _, l in labels], fontsize=6.2)
    ax.set_ylim(0, 1.28); ax.set_ylabel("fraction of seeds", fontsize=8)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    ax.legend(handles=[
        matplotlib.patches.Patch(facecolor="0.4", label="TRAINING RECORD (sampled): "
                                                        "rolling-30 `home` >= 0.5"),
        matplotlib.patches.Patch(facecolor="0.7", hatch="///",
                                 label="EVAL CELLS (mode): >= 1 `home` in any rnd30 "
                                       "cell")],
        fontsize=6.4, loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2,
        frameon=False)
    wrap_title(ax, f"Ignition rate by condition, observation {OBS}, ladder {LADDER}")
    fig.tight_layout()
    cap = (f"Ignition rate.  Solid bar: TRAINING RECORD (sampled actions) -- the fraction "
           f"of seeds whose rolling-{ROLL}-episode `home` rate reaches >= 0.5 inside "
           f"budget.  Hatched bar: EVAL CELLS (deterministic actions, rnd30, fresh "
           f"process) -- the fraction of seeds with at least one `home` episode in any "
           f"rnd30 MODE cell.  Error bars = Wilson 95 % CI; k/n above each bar.  `+p?' = "
           f"p seeds still running that have not yet crossed: undetermined, out of the "
           f"denominator.  A seed with no eval cell yet is out of the hatched "
           f"denominator too (absent, not zero).  Ticks: dv3 loc / dv3 clu / r2d clu / "
           f"RLPD / ramp ctl, H = human (dHfull_all, 74 tapes), M = machine "
           f"(dDPfull_first, 72 tapes).  {{dv3}} loc ran on the pop-os AVX2 GPU "
           f"workstation.  {{RLPD}} budget 250k decisions = 1 M sim frames and its "
           f"episode record carries no `home` flag, so it has no solid bar.  `ramp ctl' "
           f"uses nested_ramp, not {LADDER}.")
    h = fig_caption(fig, cap)
    fig.subplots_adjust(bottom=h + 0.155)
    save(fig, outdir / "fig_ignition")
    return rows


def save(fig, stem: Path):
    fig.savefig(str(stem) + ".png", dpi=200)
    fig.savefig(str(stem) + ".pdf")
    plt.close(fig)
    print(f"[fig] {stem}.png / .pdf")


# ------------------------------------------------------------------------ rise-time tex

RISE_COLS = [("picked", 0.5), ("placed_v2", 0.5), ("farside", 0.5),
             ("slide_event", 0.5), ("home", 0.5), ("home", 0.1), ("tipped", 0.5)]


def rise_rows(groups):
    per_seed, cells = [], []
    for cond in groups:
        for arm, rs in groups[cond].items():
            for phase, th in RISE_COLS:
                vals = []
                absent = 0
                for r in rs:
                    if r.flags.get(phase) is None:
                        absent += 1
                        per_seed.append(dict(condition=cond, arm=arm, seed=r.seed,
                                             run=r.name, phase=phase, threshold=th,
                                             rise_step="absent", first_event_step="absent",
                                             budget=r.budget, achieved=r.achieved,
                                             note=r.note))
                        continue
                    v = r.rise(phase, th)
                    fe = r.first_event(phase)
                    vals.append(v)
                    per_seed.append(dict(condition=cond, arm=arm, seed=r.seed, run=r.name,
                                         phase=phase, threshold=th,
                                         rise_step=("never" if v is None else v),
                                         first_event_step=("never" if fe is None else fe),
                                         budget=r.budget, achieved=r.achieved,
                                         note=r.note))
                got = [v for v in vals if v is not None]
                # a run that has not crossed but has not finished its budget is
                # UNDETERMINED (pending), not a failure to cross.
                pending = sum(1 for r in rs
                              if r.flags.get(phase) is not None
                              and r.rise(phase, th) is None and not r.complete)
                cells.append(dict(condition=cond, arm=arm, phase=phase, threshold=th,
                                  n_crossed=len(got), n_seeds=len(vals),
                                  n_never=len(vals) - len(got) - pending,
                                  n_pending=pending, n_absent=absent,
                                  median=(float(np.median(got)) if got else None),
                                  vmin=(min(got) if got else None),
                                  vmax=(max(got) if got else None)))
    return per_seed, cells


def write_tex(cells, groups, path: Path):
    tex_ladder = LADDER.replace("_", r"\_")
    idx = {(c["condition"], c["arm"], c["phase"], c["threshold"]): c for c in cells}
    hdr = ["picked", "placed", "farside", "slide", r"\textbf{home}", "home", "tipped"]
    sub = [r"$\geq$.5", r"$\geq$.5", r"$\geq$.5", r"$\geq$.5", r"$\geq$.5",
           r"$\geq$.1", r"$\geq$.5"]
    L = []
    L.append("% generated by baselines/diagnostics/px_phase_analysis.py -- do not edit")
    L.append(r"\begin{table*}[t]")
    L.append(r"\centering")
    L.append(r"\footnotesize")
    L.append(r"\setlength{\tabcolsep}{3pt}")
    L.append(r"\resizebox{\textwidth}{!}{%")
    L.append(r"\begin{tabular}{llr" + "c" * len(RISE_COLS) + "}")
    L.append(r"\toprule")
    L.append(r"learner & demonstrations & $n$ & " + " & ".join(hdr) + r" \\")
    L.append(r" &  &  & " + " & ".join(sub) + r" \\")
    L.append(r"\midrule")
    for cond in groups:
        first = True
        for arm, rs in groups[cond].items():
            lab = CONDITIONS[cond]["short"].replace("{", r"\{").replace("}", r"\}")
            name = lab if first else ""
            first = False
            ent = []
            for phase, th in RISE_COLS:
                c = idx.get((cond, arm, phase, th))
                if c is None or c["n_seeds"] == 0:
                    ent.append(r"\textemdash")
                elif c["n_absent"] == c["n_seeds"] + c["n_absent"] and c["n_seeds"] == 0:
                    ent.append("absent")
                elif c["n_crossed"] == 0:
                    ent.append(f"n/a\\,\\tiny(0/{c['n_seeds'] - c['n_pending']})")
                else:
                    e = (f"{c['median']/1e3:.0f} [{c['vmin']/1e3:.0f},"
                         f"\\,{c['vmax']/1e3:.0f}]")
                    det = c["n_seeds"] - c["n_pending"]
                    tail = f"\\,\\tiny({c['n_crossed']}/{det}"
                    tail += f"; {c['n_pending']} run.)" if c["n_pending"] else ")"
                    ent.append(e + tail)
            # absent phases (RLPD) get a single marker
            for i, (phase, th) in enumerate(RISE_COLS):
                c = idx.get((cond, arm, phase, th))
                if c and c["n_seeds"] == 0 and c["n_absent"] > 0:
                    ent[i] = r"\textemdash\,\tiny(absent)"
            L.append(f"{name} & {arm} & {len(rs)} & " + " & ".join(ent) + r" \\")
        L.append(r"\addlinespace[2pt]")
    L.append(r"\bottomrule")
    L.append(r"\end{tabular}}")
    L.append(r"\caption{Rise time by phase for the \texttt{" + tex_ladder + r"} "
             r"\textbf{pixel} conditions. Entry: the median over seeds of the first "
             r"online sim step (in thousands) at which the run's rolling-30-episode rate "
             r"for that phase crosses the threshold, with $[\min,\max]$ over seeds and "
             r"$(\textrm{crossed}/\textrm{seeds})$. TRAINING RECORD: sampled actions on "
             r"the policy's own training starts. `n/a' = no seed crossed inside budget. "
             r"\textemdash\,(absent) = the learner's episode record does not carry that "
             r"flag (RLPD logs no \texttt{farside}/\texttt{slide\_event}/\texttt{home}) "
             r"-- absent, not zero. Budgets: 1\,M (ae/af), 2\,M (ag) online sim steps; "
             r"\{RLPD\} 250k decisions $=$ 1\,M sim frames at action\_repeat 4. The "
             r"\{dv3\} local seeds ran on the pop-os AVX2 GPU workstation; every other "
             r"condition on the pax cluster. The ramp control uses "
             r"\texttt{nested\_ramp}, not \texttt{" + tex_ladder + r"}.}")
    L.append(r"\label{tab:px-rise-time}")
    L.append(r"\end{table*}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(L) + "\n")
    print(f"[tex] {path}")


# --------------------------------------------------------------------------------- csv

def write_csv(rows, path: Path, fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        print(f"[csv] {path} (empty)")
        return
    fields = fields or list(OrderedDict((k, None) for r in rows for k in r))
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[csv] {path} ({len(rows)} rows)")


# -------------------------------------------------------------------------------- main

def main(argv=None):
    global ROLL, GRID, BOOT
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-root", default="~/data/genesis_pickaplace/px_analysis_2026-09-14",
                    help="local rsync mirror of the cluster + local run trees")
    ap.add_argument("--out-dir", default="paper/figures/px_phase_2026-09-14")
    ap.add_argument("--tex", default="paper/figures/px_rise_time_by_phase.tex")
    ap.add_argument("--roll", type=int, default=ROLL)
    ap.add_argument("--grid", type=int, default=GRID)
    ap.add_argument("--boot", type=int, default=BOOT)
    ap.add_argument("--seed", type=int, default=BOOT_SEED)
    a = ap.parse_args(argv)

    ROLL, GRID, BOOT = a.roll, a.grid, a.boot

    root = Path(os.path.expanduser(a.data_root))
    outdir = Path(os.path.expanduser(a.out_dir))
    outdir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)

    runs = discover(root)
    groups = group(runs)

    census = [dict(condition=r.condition, arm=r.arm, seed=r.seed, run=r.name,
                   source=r.source, path=r.path, origin=r.origin,
                   nominal_budget=r.nominal_budget, budget_used=r.budget,
                   achieved_online=r.achieved, episodes=r.n_episodes,
                   complete=r.complete, note=r.note, x_unit=r.x_unit,
                   phases_absent=",".join(p for p in PHASES
                                          if r.flags.get(p) is None) or "",
                   n_rnd30_cells=len([c for c in r.cells if c["cell"] == "rnd30_mode"]),
                   n_hold15_cells=len([c for c in r.cells if c["cell"] == "hold15_mode"]))
              for r in sorted(runs, key=lambda r: (r.condition, r.arm, r.seed))]
    write_csv(census, outdir / "px_run_census.csv")

    curve_rows = []
    for cond, arms in groups.items():
        curve_rows += fig_learning_curves(cond, arms, outdir, rng)
    write_csv(curve_rows, outdir / "px_learning_curves.csv")

    per_seed, cells = rise_rows(groups)
    write_csv(per_seed, outdir / "px_rise_time_per_seed.csv")
    write_csv(cells, outdir / "px_rise_time_cells.csv")
    write_tex(cells, groups, Path(os.path.expanduser(a.tex)))

    write_csv(fig_steady_state(groups, outdir), outdir / "px_steady_state.csv")
    write_csv(fig_ignition(groups, outdir), outdir / "px_ignition.csv")

    # per-cell eval dump, so every eval number in the figures is traceable
    ev = []
    for r in runs:
        for c in r.cells:
            row = dict(condition=r.condition, arm=r.arm, seed=r.seed, run=r.name,
                       cell=c["cell"], milestone=c["milestone"], online=c["online"],
                       episodes=c["episodes"], path=c["path"])
            for p in PHASES + ["nested_v2", "contact_push", "slide_success"]:
                row[p] = c["stages"].get(p)
            ev.append(row)
    write_csv(sorted(ev, key=lambda r: (r["condition"], r["arm"], r["seed"],
                                        r["cell"], r["online"])),
              outdir / "px_eval_cells.csv")
    print("[done]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
