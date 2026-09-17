#!/usr/bin/env python3
"""By-phase analysis of the `nested_sparse10` PIXEL conditions.

Produces, for every learner x arm condition of amendments (ad)/(ae)/(af)/(ag):

  1. learning curves   -- rolling-30-episode mean of each per-episode stage flag
                          against online sim steps, mean + 95 % bootstrap CI over seeds;
  2. rise time         -- first online step at which the rolling-30 rate crosses a
                          threshold, per run, tabulated as median [min, max] over seeds;
  3. rise-time LAG     -- per run, rise_time(phase) - rise_time(picked), so the phases
                          are read as an ORDER rather than as absolute clock times;
  4. steady state      -- (a) training record over the last 20 % of each run,
                          (b) eval cells (rnd30 MODE) at >= 0.5 M online steps;
  5. ignition rate     -- fraction of seeds whose rolling-30 `home` reaches >= 0.5
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

#: the rise-time LAG figure: reference phase, the phases plotted against it, and the
#: learner conditions that carry all of them.  {RLPD} is excluded because its episode
#: record has no `farside`/`slide_event`/`home` (absent, not zero, PROVENANCE 7.4) and
#: the ramp control is excluded because it pays a different ladder.
LAG_REF = "picked"
LAG_PHASES = ["placed_v2", "farside", "slide_event", "home", "tipped"]
LAG_CONDITIONS = ["dreamer_local", "dreamer_cluster", "r2dreamer_cluster"]
LAG_THRESHOLDS = (0.5, 0.1)

ROLL = 30           # episodes in the rolling window
GRID = 50_000       # online-step grid for the CI bands
MIN_SEEDS_BAND = 3  # truncate a band where fewer than this many seeds have data
BOOT = 2000         # bootstrap resamples
BOOT_SEED = 20260914

#: demonstration datasets ("arms"), in table/legend order.  One colour per dataset,
#: used identically in every figure.
ARMS = ["human", "machine", "planner72", "r2teacher"]
ARM_OF_SET = {
    "dHfull_all": "human",
    "dDPfull_first": "machine",
    "dH": "human",
    "dDPfirst": "machine",
    "dM": "machine",                 # {Diffusion Policy} (ah) machine tag
    "dPlanner72": "planner72",
    "dR2fromH_px": "r2teacher",
}
ARM_COLOR = {"human": "#1f6fb4", "machine": "#c3562a",
             "planner72": "#2a9d5c", "r2teacher": "#8a4fbf"}
ARM_LABEL = {"human": "human", "machine": "machine (DP teacher)",
             "planner72": "planner", "r2teacher": "r2dreamer teacher"}
ARM_TICK = {"human": "H", "machine": "M", "planner72": "P", "r2teacher": "T"}
#: one sentence per dataset, reused in every caption
DATASET_DESC = ("H = human demonstrations (dHfull_all, 74 tapes), M = machine (dDPfull_first, "
                "72 tapes from a Diffusion Policy teacher), P = planner (planner72, 72 "
                "motion-planner tapes), T = r2dreamer teacher (72 tapes from one "
                "human-trained pixel r2dreamer)")

#: compact axis-tick codes (the full {algorithm} names live in the captions)
TICK = {
    "dreamer_local": "dv3 loc",
    "dreamer_cluster": "dv3 clu",
    "r2dreamer_cluster": "r2d clu",
    "rlpd_cluster": "RLPD",
    "dp_cluster": "DP",
    "dreamer_ramp_control": "ramp ctl",
}

#: the 4 x 4 results table: learner label -> the conditions pooled into it
LEARNERS = OrderedDict([
    ("{DreamerV3 losses}", ["dreamer_local", "dreamer_cluster"]),
    ("{r2dreamer}", ["r2dreamer_cluster"]),
    ("{RLPD}", ["rlpd_cluster"]),
    ("{Diffusion Policy}", ["dp_cluster"]),
])
#: conditions that have an episode-level TRAINING RECORD (DP has none)
TRAINED_RECORD_CONDITIONS = ["dreamer_local", "dreamer_cluster", "r2dreamer_cluster",
                             "rlpd_cluster", "dreamer_ramp_control"]
#: design seeds per (learner, dataset) where no job ledger is mirrored.  The new
#: datasets (planner72, r2teacher) take their seeds from SUBMISSIONS.jsonl instead.
DESIGN_SEEDS = {
    ("{DreamerV3 losses}", "human"): list(range(16)),     # local s0-3 + cluster s4-15
    ("{DreamerV3 losses}", "machine"): list(range(16)),
    ("{r2dreamer}", "human"): list(range(16)),
    ("{r2dreamer}", "machine"): list(range(16)),
    ("{RLPD}", "human"): list(range(8)),      # s8 is an extra seed outside the n=8 design
    ("{RLPD}", "machine"): list(range(8)),
    ("{Diffusion Policy}", "human"): list(range(8)),
    ("{Diffusion Policy}", "machine"): list(range(8)),
}
LEDGER_LEARNER = {"dreamer": "{DreamerV3 losses}", "r2dreamer": "{r2dreamer}",
                  "rlpd": "{RLPD}", "dp": "{Diffusion Policy}"}
#: {RLPD} budget in decisions; cells from a checkpoint past it are not used
RLPD_BUDGET_DECISIONS = 250_000
RLPD_EPISODE_SLACK = 1_200            # a final checkpoint may land < 1 episode past budget
STAT_MILESTONES = (500_000, 1_000_000)
STAT_TOL = 60_000                     # local series: nearest checkpoint within this
PERM_EXACT_MAX = 1_500_000
PERM_MC = 300_000

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
    ("dp_cluster", dict(
        algorithm="Diffusion Policy (lerobot, pixels), 100k updates, SAMPLED actions",
        short="{Diffusion Policy} cluster",
        hardware="Tufts pax cluster GPU nodes (train), CPU nodes (eval)",
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
        #: {RLPD} only: rolling-100 `home` rate DERIVED from the Slurm log's
        #: ep_rew_mean / 10 -> (online sim frames, rate); None if no log
        self.derived_home = None
        self.derived_home_logs = []
        #: eval cell kind of record and the minimum milestone for steady state
        self.primary_cell = "rnd30_sample" if condition == "dp_cluster" else "rnd30_mode"
        self.min_eval_online = 0 if condition in ("dp_cluster", "rlpd_cluster") else 500_000
        self.has_training_record = False
        self.excluded_cells = []                      # (path, reason)

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

    def eval_cells(self, cell=None, min_online=None):
        cell = cell or self.primary_cell
        min_online = self.min_eval_online if min_online is None else min_online
        return [c for c in self.cells
                if c["cell"] == cell and c["online"] >= min_online]

    def steady_eval(self, phase, cell=None, min_online=None):
        vals = [c["stages"].get(phase) for c in self.eval_cells(cell, min_online)]
        vals = [v for v in vals if v is not None]
        return (float(np.mean(vals)), len(vals)) if vals else (None, 0)

    def record_statistic(self):
        """Per-seed STATISTIC OF RECORD -> dict(home, picked, cells) or (None, reason).

        world models: mean of the rnd30 MODE cells at 0.5 M and 1 M online steps (nearest
        cell within STAT_TOL; both required).  {RLPD}: the final / 250 k rnd30 MODE cell.
        {Diffusion Policy}: the rnd30 SAMPLE cell.
        """
        cells = [c for c in self.cells if c["cell"] == self.primary_cell]
        if self.condition in ("rlpd_cluster", "dp_cluster"):
            if not cells:
                return None, "no %s cell" % self.primary_cell
            c = cells[-1]
            return dict(home=c["stages"].get("home"), picked=c["stages"].get("picked"),
                        cells=[c["path"]]), None
        got = []
        for ms in STAT_MILESTONES:
            near = [c for c in cells if abs(c["online"] - ms) <= STAT_TOL]
            if not near:
                return None, "no rnd30_mode cell at %dk" % (ms // 1000)
            got.append(min(near, key=lambda c: abs(c["online"] - ms)))
        hs = [c["stages"].get("home") for c in got]
        ps = [c["stages"].get("picked") for c in got]
        if any(h is None for h in hs):
            return None, "cell lacks `home`"
        return dict(home=float(np.mean(hs)),
                    picked=(float(np.mean(ps)) if all(p is not None for p in ps) else None),
                    cells=[c["path"] for c in got]), None

    def any_eval_home(self, cell=None):
        """True if >= 1 `home` episode in any cell of that kind (any milestone)."""
        cell = cell or self.primary_cell
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
    # a record that was appended to by a restarted process has a step that goes
    # backwards; keep only the last monotone segment (the live process).
    back = [i for i in range(1, len(steps)) if steps[i] < steps[i - 1]]
    if back:
        steps, rows = steps[back[-1]:], rows[back[-1]:]
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


SB3_RUN_RE = re.compile(r"== RLPD-E2E-PX (\S+) start .*?restart=(\d+)")
SB3_ROW_RE = re.compile(r"\|\s+(ep_rew_mean|total_timesteps)\s+\|\s+([-\d.eE+]+)\s+\|")


def parse_sb3_log(path: Path):
    """RLPD Slurm log -> (run name, restart, [(total_timesteps, ep_rew_mean)]) or None.

    SB3 prints `ep_rew_mean` (mean over its last 100 online episodes) before
    `total_timesteps` (decisions) inside each logger table.
    """
    name, restart, rows, pend = None, 0, [], None
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            if name is None:
                m = SB3_RUN_RE.search(line)
                if m:
                    name, restart = m.group(1), int(m.group(2))
                continue
            m = SB3_ROW_RE.search(line)
            if not m:
                continue
            if m.group(1) == "ep_rew_mean":
                pend = float(m.group(2))
            elif pend is not None:
                rows.append((int(float(m.group(2))), pend))
                pend = None
    if name is None:
        return None
    return name, restart, rows


def job_id_of(path: Path):
    m = re.search(r"_(\d{6,})\.out$", path.name)
    return int(m.group(1)) if m else -1


def read_cell(mpath: Path):
    try:
        d = json.load(open(mpath))
    except Exception:
        return None
    st = d.get("headline_stages") or d.get("stages") or {}
    per = d.get("per_episode") or []
    return dict(stages={k: (float(v) if v is not None else None) for k, v in st.items()},
                episodes=(len(per) if per else int(d.get("episodes") or 0)),
                checkpoint=d.get("checkpoint"),
                path=str(mpath))


def cell_expected_episodes(cell):
    return 30 if cell.startswith("rnd30") else 15 if cell.startswith("hold15") else None


# --------------------------------------------------------------------------- discovery

WM_RE = re.compile(r"^full_r2d_state_(dHfull_all|dDPfull_first)_(rns10h|rnrh)_img_"
                   r"(dreamer|r2dreamer)_s(\d+)$")
#: planner72 / r2teacher campaigns name their demonstration set `native`
NATIVE_WM_RE = re.compile(r"^full_r2d_state_native_(rns10h)_img_(dreamer|r2dreamer)_s(\d+)$")
LOCAL_RE = re.compile(r"^dv3px_sparse10_(dHfull_all|dDPfull_first)_rns10h_img_"
                      r"rlDreamer_s(\d+)$")
RLPD_RE = re.compile(r"^e2e_rlpd_px_(dH|dDPfirst|dPlanner72|dR2fromH_px)_s(\d+)$")
DP_RE = re.compile(r"^(?:ah|planner72|r2teacher)_dp_px_(dH|dM|dPlanner72|dR2fromH_px)_s(\d+)$")
DP_LOG_RE = re.compile(r"^(planner72|r2teacher)_dp_px_s(\d+)_(\d+)\.out$")
DP_STEP_RE = re.compile(r"ot_train\.py:\d+ step:")
SMOKE_SEED = 9000

#: mirror subtree -> dataset, for the two campaign roots (planner72 / r2teacher)
CAMPAIGNS = OrderedDict([("P72", "planner72"), ("R2T", "r2teacher")])


def _wm_run(d: Path, cellroot: Path, cond, arm, source="cluster"):
    log = d / "console.log"
    if not log.exists():
        return None
    parsed = parse_console(log)
    if parsed is None:
        return None
    origin, nominal, online, flags = parsed
    seed = int(d.name.rsplit("_s", 1)[1])
    r = Run(cond, arm, seed, d.name, str(d), source)
    r.origin, r.nominal_budget, r.online, r.flags = origin, nominal, online, flags
    r.has_training_record = r.n_episodes > 0
    sc = d / "step_contract.json"
    if sc.exists():
        try:
            j = json.load(open(sc))
            r.origin = int(j.get("prefill_counter_origin", r.origin))
            r.nominal_budget = int(j.get("requested_online_sim_steps",
                                         r.nominal_budget or 0)) or r.nominal_budget
        except Exception:
            pass
    cd = cellroot / d.name
    if cd.is_dir():
        for md in sorted(cd.glob("online_*")):
            try:
                ms = int(md.name.split("_")[1])
            except ValueError:
                continue
            for cell in ("rnd30_mode", "hold15_mode"):
                mp = md / cell / "metrics.json"
                c = read_cell(mp)
                if not c:
                    continue
                exp = cell_expected_episodes(cell)
                if exp and c["episodes"] != exp:
                    r.excluded_cells.append((str(mp), f"{c['episodes']}/{exp} episodes"))
                    continue
                c.update(cell=cell, online=ms, milestone=md.name)
                r.cells.append(c)
    return r


def _rlpd_logs(root: Path):
    """run name -> list of (job id, restart, rows, path), job-id sorted."""
    paths = list((root / "LAB" / "gp_pxr").glob("e2e_rlpd_px_*.out"))
    for sub in CAMPAIGNS:
        paths += list((root / sub / "slurm").glob("*_rlpd_px_s*.out"))
    out = {}
    for p in paths:
        got = parse_sb3_log(p)
        if got is None:
            continue
        name, restart, rows = got
        out.setdefault(name, []).append((job_id_of(p), restart, rows, str(p)))
    for v in out.values():
        v.sort(key=lambda t: t[0])
    return out


def _derived_home(entries, budget_decisions=RLPD_BUDGET_DECISIONS, repeat=4):
    """Concatenate the live log segment(s): the last restart=0 log that has data plus any
    later restart>0 logs; cap at the budget.  -> (frames, rate, [log paths]) or None."""
    live = [e for e in entries if e[2]]
    if not live:
        return None
    starts = [i for i, e in enumerate(live) if e[1] == 0]
    first = starts[-1] if starts else 0
    seg = live[first:]
    rows = [row for e in seg for row in e[2]]
    rows = [(t, v) for t, v in rows if t <= budget_decisions]
    if not rows:
        return None
    t = np.asarray([x[0] for x in rows], dtype=np.int64) * repeat
    y = np.asarray([x[1] for x in rows], dtype=float) / 10.0     # +10 only at `home`
    return t, y, [e[3] for e in seg]


def _rlpd_run(d: Path, arm, logs):
    m = RLPD_RE.match(d.name)
    seed = int(m.group(2))
    r = Run("rlpd_cluster", arm, seed, d.name, str(d), "cluster")
    r.origin = 0
    r.nominal_budget = RLPD_BUDGET_DECISIONS * 4          # 250 k decisions x repeat 4
    r.x_unit = "online sim frames (decisions x 4)"
    jl = d / "episode_rollouts.jsonl"
    max_dec = None
    parsed = parse_rlpd_jsonl(jl) if jl.exists() else None
    if parsed is not None:
        online, flags = parsed
        max_dec = int(online[-1] // 4)
        # 2026-09-16: the (ag) RLPD pixel jobs did not stop at --steps 250000 and trained on
        # to the 30 h wall clock (up to 592 k decisions). Everything past the registered budget
        # is outside the design, so the record is CUT at the budget; the 250 k checkpoint is
        # the one scored.
        keep = online <= r.nominal_budget
        if not keep.all():
            online = online[keep]
            flags = {k: (None if v is None else v[keep]) for k, v in flags.items()}
        r.online, r.flags = online, flags
        r.has_training_record = r.n_episodes > 0
    else:
        r.flags = {p: None for p in PHASES}
    dh = _derived_home(logs.get(d.name, []))
    if dh is not None:
        r.derived_home = (dh[0], dh[1])
        r.derived_home_logs = dh[2]
        r.has_training_record = True
    for cell in ("rnd30_mode", "hold15_mode"):
        mp = d / f"fresh_eval_{cell}" / "metrics.json"
        c = read_cell(mp)
        if not c:
            continue
        exp = cell_expected_episodes(cell)
        if exp and c["episodes"] != exp:
            r.excluded_cells.append((str(mp), f"{c['episodes']}/{exp} episodes"))
            continue
        ck = os.path.basename(str(c.get("checkpoint") or ""))
        if ck == f"rlpd_{RLPD_BUDGET_DECISIONS}_steps.zip":
            pass
        elif ck == "rlpd_final.zip" and max_dec is not None \
                and max_dec <= RLPD_BUDGET_DECISIONS + RLPD_EPISODE_SLACK:
            pass
        else:
            r.excluded_cells.append((str(mp), f"checkpoint {ck or '?'} not the "
                                              f"{RLPD_BUDGET_DECISIONS // 1000}k budget "
                                              f"(record reaches {max_dec} decisions)"))
            continue
        c.update(cell=cell, online=r.nominal_budget, milestone=ck)
        r.cells.append(c)
    return r


def _dp_run(d: Path, arm, seed):
    r = Run("dp_cluster", arm, seed, d.name, str(d), "cluster")
    r.origin, r.nominal_budget = 0, 100_000
    r.flags = {p: None for p in PHASES}
    r.x_unit = "gradient updates (no episode record)"
    for cell in ("rnd30_sample", "hold15_sample"):
        mp = d / f"fresh_eval_{cell}" / "metrics.json"
        c = read_cell(mp)
        if not c:
            continue
        exp = cell_expected_episodes(cell)
        if exp and c["episodes"] != exp:
            r.excluded_cells.append((str(mp), f"{c['episodes']}/{exp} episodes"))
            continue
        c.update(cell=cell, online=0, milestone="final")
        r.cells.append(c)
    return r


def read_ledgers(root: Path):
    """(learner label, dataset) -> sorted seeds submitted (SUBMISSIONS.jsonl)."""
    out = {}
    for sub, arm in CAMPAIGNS.items():
        p = root / sub / "SUBMISSIONS.jsonl"
        if not p.exists():
            continue
        for line in open(p):
            try:
                c = json.loads(line)["configuration"]
            except Exception:
                continue
            lab = LEDGER_LEARNER.get(c.get("learner"))
            if lab is None or int(c.get("seed", SMOKE_SEED)) >= SMOKE_SEED:
                continue
            out.setdefault((lab, arm), set()).add(int(c["seed"]))
    return {k: sorted(v) for k, v in out.items()}


def discover(root: Path, verbose=True):
    runs = []

    # --- cluster world-model runs, human / machine ----------------------------------
    rdir = root / "W" / "runs"
    cellroot = root / "W" / "ln_milestone_cells"
    for d in sorted(rdir.glob("full_r2d_state_*")):
        m = WM_RE.match(d.name)
        if not m or "smoke" in d.name or int(m.group(4)) >= SMOKE_SEED:
            continue
        dset, ladder_tag, rep = m.group(1), m.group(2), m.group(3)
        cond = "dreamer_ramp_control" if ladder_tag == "rnrh" else f"{rep}_cluster"
        r = _wm_run(d, cellroot, cond, ARM_OF_SET[dset])
        if r is not None:
            runs.append(r)

    # --- cluster world-model runs, planner72 / r2teacher campaigns -------------------
    for sub, arm in CAMPAIGNS.items():
        for d in sorted((root / sub / "runs").glob("full_r2d_state_native_*")):
            m = NATIVE_WM_RE.match(d.name)
            if not m or int(m.group(3)) >= SMOKE_SEED:
                continue
            r = _wm_run(d, root / sub / "evaluation", f"{m.group(2)}_cluster", arm)
            if r is not None:
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
        r.has_training_record = r.n_episodes > 0
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
    logs = _rlpd_logs(root)
    rl_dirs = list((root / "LAB" / "gp_pxr" / "e2e_px").glob("e2e_rlpd_px_*"))
    for sub in CAMPAIGNS:
        rl_dirs += list((root / sub / "runs" / "rlpd").glob("e2e_rlpd_px_*"))
    for d in sorted(rl_dirs, key=lambda p: p.name):
        m = RLPD_RE.match(d.name)
        if not m or int(m.group(2)) >= SMOKE_SEED:
            continue
        runs.append(_rlpd_run(d, ARM_OF_SET[m.group(1)], logs))

    # --- Diffusion Policy ------------------------------------------------------------
    dp = {}
    dp_dirs = list((root / "LAB" / "gp_ah" / "dp_px").glob("ah_dp_px_*"))
    for sub in CAMPAIGNS:
        dp_dirs += list((root / sub / "runs" / "dp").glob("*_dp_px_*"))
    for d in sorted(dp_dirs, key=lambda p: p.name):
        m = DP_RE.match(d.name)
        if not m or int(m.group(2)) >= SMOKE_SEED:
            continue
        arm, seed = ARM_OF_SET[m.group(1)], int(m.group(2))
        r = _dp_run(d, arm, seed)
        # (ah) human/machine: no Slurm log is mirrored; an eval cell proves training ran
        r.has_training_record = bool(r.cells)
        dp[(arm, seed)] = r
    # campaign DP: a Slurm log with lerobot `step:` lines is the training record
    for sub, arm in CAMPAIGNS.items():
        for p in sorted((root / sub / "slurm").glob("*_dp_px_s*.out")):
            m = DP_LOG_RE.match(p.name)
            if not m or int(m.group(2)) >= SMOKE_SEED:
                continue
            seed = int(m.group(2))
            with open(p, "r", errors="replace") as fh:
                trained = any(DP_STEP_RE.search(line) for line in fh)
            if not trained:
                continue
            if (arm, seed) not in dp:
                tag = "dPlanner72" if arm == "planner72" else "dR2fromH_px"
                d = root / sub / "runs" / "dp" / f"{CAMPAIGNS_PREFIX[sub]}_dp_px_{tag}_s{seed}"
                dp[(arm, seed)] = _dp_run(d, arm, seed)
            dp[(arm, seed)].has_training_record = True
    runs += [dp[k] for k in sorted(dp)]

    for r in runs:
        key = (r.condition, r.arm, r.seed)
        if key in RUN_NOTES:
            r.note = RUN_NOTES[key][0]
        elif r.condition == "dp_cluster":
            r.note = "" if r.cells else "no eval cell yet"
        elif not r.complete:
            r.note = "incomplete (still running or interrupted)"

    if verbose:
        print(f"[discover] {len(runs)} runs")
        for r in runs:
            for p, why in r.excluded_cells:
                print(f"[discover] EXCLUDED cell {p}: {why}")
    return runs


CAMPAIGNS_PREFIX = {"P72": "planner72", "R2T": "r2teacher"}


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
        for arm in ARMS:
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
    seed_mode_arms = {a for a, rs in arms.items() if len(rs) < MIN_SEEDS_BAND}
    per_seed_mode = bool(seed_mode_arms)
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
            if arm in seed_mode_arms:
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
    handles = [Line2D([], [], color=ARM_COLOR[a], lw=1.5,
                      label=f"{ARM_LABEL[a]} (n={len(rs)})"
                            + (" per seed" if a in seed_mode_arms else ""))
               for a, rs in arms.items()]
    axes[-1][-1].legend(handles=handles, fontsize=6.0, loc="upper right", frameon=False)
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
    band = (f"line = mean over seeds, band = 95 % bootstrap CI ({BOOT} resamples), "
            f"truncated where fewer than {MIN_SEEDS_BAND} seeds have data")
    if per_seed_mode:
        band += (f"; datasets with fewer than {MIN_SEEDS_BAND} seeds ("
                 + ", ".join(ARM_LABEL[a] for a in ARMS if a in seed_mode_arms)
                 + ") are drawn one thin line per seed, no band")
    ladder = "nested_ramp" if cond == "dreamer_ramp_control" else LADDER
    body = (f"observation: {OBS}; ladder: {ladder}; {ns}; budget {bud}; "
            f"TRAINING RECORD, sampled actions, policy's own training starts; "
            f"{band}.  Datasets: {DATASET_DESC}." + extra)
    head = f"{meta['short']} -- {meta['algorithm']}"
    lines = [head] + textwrap.wrap(body, 118)
    fig.tight_layout()
    fig_h_pt = fig.get_size_inches()[1] * 72.0
    top = max(0.55, 1.0 - (len(lines) * 8.8 + 8) / fig_h_pt - 0.045)
    fig.subplots_adjust(top=top)
    fig.text(0.5, 0.995, "\n".join(lines), fontsize=7.2, ha="center", va="top")
    save(fig, outdir / f"fig_learning_curves_{cond}")
    return rows


def group_ticks(ax, ticks, fontsize=6.2):
    """ticks: [(x, condition, arm)].  Arm letter on the tick, condition name centred
    underneath its group (avoids overlapping long tick labels)."""
    ax.set_xticks([t[0] for t in ticks])
    ax.set_xticklabels([ARM_TICK[t[2]] for t in ticks], fontsize=fontsize)
    for tl, t in zip(ax.get_xticklabels(), ticks):
        tl.set_color(ARM_COLOR[t[2]])
    trans = matplotlib.transforms.blended_transform_factory(ax.transData, ax.transAxes)
    conds = OrderedDict()
    for x, c, _ in ticks:
        conds.setdefault(c, []).append(x)
    for c, xs in conds.items():
        ax.text(float(np.mean(xs)), -0.13, TICK[c], transform=trans, ha="center",
                va="top", fontsize=fontsize, color="0.2")


def fig_steady_state(groups, outdir):
    conds = list(groups)
    fig, axes = plt.subplots(2, 1, figsize=(7.1, 6.4))
    rows = []

    # (a) training record, last 20 % of the run
    ax = axes[0]
    ticks = []
    pos = 0
    for cond in [c for c in conds if c in TRAINED_RECORD_CONDITIONS]:
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
            ticks.append((pos, cond, arm))
            pos += 1
        pos += 0.6
    group_ticks(ax, ticks)
    ax.set_ylabel("`home` rate", fontsize=8); ax.set_ylim(-0.05, 1.12)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    wrap_title(ax, "(a) TRAINING RECORD -- mean `home` over the last 20 % of each run's "
                   "online steps (sampled actions, the policy's own training starts); "
                   "one point per seed, bar = arm mean, n above each column")

    # (b) eval cells, rnd30 MODE at >= 0.5 M
    ax = axes[1]
    ticks = []
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
            ticks.append((pos, cond, arm))
            pos += 1
        pos += 0.6
    group_ticks(ax, ticks)
    ax.set_ylabel("rate over cells", fontsize=8); ax.set_ylim(-0.05, 1.12)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    ax.legend(handles=[Line2D([], [], ls="", marker="o", ms=4, color="0.3", label="home"),
                       Line2D([], [], ls="", marker="^", ms=4, color="0.6",
                              label="picked")],
              fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2,
              frameon=False)
    wrap_title(ax, "(b) EVAL CELLS -- world models: mean over every rnd30 MODE milestone "
                   "cell at >= 0.5 M online steps; {RLPD}: the 250k rnd30 MODE cell; "
                   "{Diffusion Policy}: the rnd30 SAMPLE cell (30 fixed random starts, "
                   "fresh process); one point per seed, bar = dataset mean of `home`")

    fig.tight_layout()
    cap = (f"Steady-state performance.  {{dv3}} = DreamerV3 losses, {{r2dreamer}} = the "
           f"port's contrastive loss, {{RLPD}} = DrQ-style encoder in the LN critic "
           f"ensemble, {{Diffusion Policy}} = lerobot DP (eval only, sampled actions, no "
           f"episode record so no point in (a)); observation {OBS}, ladder {LADDER}.  "
           f"Ticks: dv3 loc / dv3 clu / r2d clu / RLPD / DP / ramp ctl; {DATASET_DESC}.  "
           f"Budgets: 1 M (ae/af) or 2 M (ag) "
           f"online sim steps; {{RLPD}} 250k decisions = 1 M sim frames.  {{dv3}} loc ran "
           f"on the pop-os AVX2 GPU workstation, every other condition on the pax "
           f"cluster.  `ramp ctl' uses nested_ramp, not {LADDER}.  {{RLPD}} logs no "
           f"`home` flag in its episode record, so panel (a) has no RLPD column "
           f"(absent, not zero).  In (a) a HOLLOW point is a run that has not finished "
           f"its budget: its last 20 % is not a steady state.  A seed with no eval cell "
           f"at >= 0.5 M is absent from (b), not zero.")
    h = fig_caption(fig, cap)
    fig.subplots_adjust(bottom=h + 0.10, hspace=0.72)
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
            ev = [r.any_eval_home() for r in rs]
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
            labels.append((pos, cond, arm))
            pos += 1
        pos += 0.6
    group_ticks(ax, labels)
    ax.set_ylim(0, 1.28); ax.set_ylabel("fraction of seeds", fontsize=8)
    ax.grid(axis="y", alpha=0.25, lw=0.4); ax.tick_params(labelsize=7)
    ax.legend(handles=[
        matplotlib.patches.Patch(facecolor="0.4", label="TRAINING RECORD (sampled): "
                                                        "rolling-30 `home` >= 0.5"),
        matplotlib.patches.Patch(facecolor="0.7", hatch="///",
                                 label="EVAL CELLS: >= 1 `home` in any rnd30 cell "
                                       "(MODE; {Diffusion Policy}: SAMPLE)")],
        fontsize=6.4, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2,
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
           f"RLPD / DP / ramp ctl; {DATASET_DESC}.  {{Diffusion Policy}} (eval only) uses "
           f"its rnd30 SAMPLE cell and has no training record, so no solid bar.  {{dv3}} loc ran on the pop-os AVX2 GPU "
           f"workstation.  {{RLPD}} budget 250k decisions = 1 M sim frames and its "
           f"episode record carries no `home` flag, so it has no solid bar.  `ramp ctl' "
           f"uses nested_ramp, not {LADDER}.")
    h = fig_caption(fig, cap)
    fig.subplots_adjust(bottom=h + 0.21)
    save(fig, outdir / "fig_ignition")
    return rows


def lag_rows(groups, thresh, conditions=LAG_CONDITIONS):
    """Per run and phase: rise_time(phase) - rise_time(`picked`) at one threshold.

    Status of a row:
      ``crossed``      -- both the reference and the phase crossed; ``lag_steps`` is set;
      ``never``        -- the reference crossed, the phase never did inside the record;
      ``absent``       -- the learner's episode record has no such flag at all;
      ``no_reference`` -- `picked` itself never crossed, so no lag is defined for the run;
      ``pending``      -- the phase has not crossed but the run is still inside its budget.
    """
    rows = []
    for cond in conditions:
        if cond not in groups:
            continue
        for arm, rs in groups[cond].items():
            for r in rs:
                ref = (r.rise(LAG_REF, thresh)
                       if r.flags.get(LAG_REF) is not None else None)
                for phase in LAG_PHASES:
                    base = dict(condition=cond, arm=arm, seed=r.seed, run=r.name,
                                threshold=thresh, phase=phase, reference=LAG_REF,
                                rise_step_reference=("never" if ref is None else ref),
                                budget=r.budget, achieved=r.achieved,
                                complete=r.complete, note=r.note)
                    if r.flags.get(phase) is None:
                        rows.append(dict(base, rise_step_phase="absent",
                                         lag_steps="", status="absent"))
                        continue
                    v = r.rise(phase, thresh)
                    if ref is None:
                        rows.append(dict(base,
                                         rise_step_phase=("never" if v is None else v),
                                         lag_steps="", status="no_reference"))
                    elif v is None and not r.complete:
                        rows.append(dict(base, rise_step_phase="not_yet",
                                         lag_steps="", status="pending"))
                    elif v is None:
                        rows.append(dict(base, rise_step_phase="never",
                                         lag_steps="", status="never"))
                    else:
                        rows.append(dict(base, rise_step_phase=v, lag_steps=v - ref,
                                         status="crossed"))
    return rows


def fig_rise_lag(groups, outdir, thresh, conditions=LAG_CONDITIONS):
    """Per-phase rise-time lag relative to `picked`, one panel per learner condition."""
    rows = lag_rows(groups, thresh, conditions)
    conds = [c for c in conditions if c in groups]
    if not conds:
        return rows

    # shared y limits over every panel, from the lags that exist
    finite = [r["lag_steps"] / 1e3 for r in rows if r["status"] == "crossed"]
    if finite:
        lo, hi = min(finite), max(finite)
    else:
        lo, hi = -1.0, 1.0
    span = max(hi - lo, 1.0)
    # the "never crossed" band at the top edge is reserved only if some seed needs it;
    # an empty band would read as head-room that the data does not use.
    n_never = sum(1 for r in rows if r["status"] == "never")
    never_y = (hi + 0.13 * span) if n_never else None
    ytop = (hi + 0.26 * span) if n_never else (hi + 0.10 * span)
    ybot = lo - 0.13 * span

    nrow = len(conds)
    fig, axes = plt.subplots(nrow, 1, figsize=(7.1, 2.35 * nrow + 1.5), sharey=True)
    if nrow == 1:
        axes = [axes]
    dxs = {a: (i - 1.5) * 0.19 for i, a in enumerate(ARMS)}   # dataset offset in a phase
    for ax, cond in zip(axes, conds):
        arms = groups[cond]
        n_no_ref = {}
        for arm, rs in arms.items():
            n_no_ref[arm] = len({r["seed"] for r in rows
                                 if r["condition"] == cond and r["arm"] == arm
                                 and r["status"] == "no_reference"})
        ax.axhline(0.0, color="0.35", lw=0.9, zorder=1)
        if never_y is not None:
            ax.axhline(never_y, color="0.75", lw=0.6, ls=":", zorder=1)
        for k, phase in enumerate(LAG_PHASES):
            for arm in arms:
                x = k + dxs[arm]
                sel = [r for r in rows if r["condition"] == cond and r["arm"] == arm
                       and r["phase"] == phase]
                got = [r["lag_steps"] / 1e3 for r in sel if r["status"] == "crossed"]
                nev = [r for r in sel if r["status"] == "never"]
                for r in sel:
                    if r["status"] != "crossed":
                        continue
                    j = (np.random.RandomState(r["seed"]).rand() - 0.5) * 0.12
                    ax.plot(x + j, r["lag_steps"] / 1e3, "o", ms=3.2,
                            color=ARM_COLOR[arm], alpha=0.85, mew=0, zorder=3)
                for r in nev:
                    j = (np.random.RandomState(r["seed"]).rand() - 0.5) * 0.17
                    ax.plot(x + j, never_y, "o", ms=4.0, mfc="none",
                            mec=ARM_COLOR[arm], mew=0.9, alpha=0.95, zorder=3)
                if nev and never_y is not None:
                    ax.text(x, never_y + 0.045 * span + 0.05 * span * (ARMS.index(arm) % 2),
                        f"{len(nev)}x", ha="center", fontsize=5.0, color=ARM_COLOR[arm])
                if got:
                    m = float(np.median(got))
                    ax.plot([x - 0.08, x + 0.08], [m, m], "-",
                            color=ARM_COLOR[arm], lw=2.2, zorder=4)
                    ax.text(x, ybot + 0.035 * span, f"{len(got)}",
                            ha="center", fontsize=5.4, color="0.35")
        ax.set_xticks(range(len(LAG_PHASES)))
        ax.set_xticklabels([PHASE_LABEL[p] for p in LAG_PHASES], fontsize=7)
        ax.set_xlim(-0.55, len(LAG_PHASES) - 0.45)
        ax.set_ylim(ybot, ytop)
        ax.set_ylabel("lag after `picked'\n(k online sim steps)", fontsize=7.4)
        ax.grid(axis="y", alpha=0.22, lw=0.4)
        ax.tick_params(labelsize=7)
        nr = "; ".join(f"{a} {n} without a `picked' crossing"
                       for a, n in n_no_ref.items() if n)
        n_pend = {a: len({r["seed"] for r in rows if r["condition"] == cond
                          and r["arm"] == a and r["status"] == "pending"})
                  for a in arms}
        pd = "; ".join(f"{a} {n} still running, not yet crossed" for a, n in n_pend.items()
                       if n)
        nr = "; ".join(x for x in (nr, pd) if x)
        ns = ", ".join(f"{a} n={len(rs)}" for a, rs in arms.items())
        wrap_title(ax, f"{CONDITIONS[cond]['short']} -- {ns}"
                       + (f"  [{nr}: no lag defined, not plotted]" if nr else ""),
                   width=104, fontsize=7.2)
        if ax is axes[0]:
            handles = [Line2D([], [], ls="", marker="o", ms=4, color=ARM_COLOR[a],
                              label=f"{ARM_LABEL[a]}") for a in arms]
            handles.append(Line2D([], [], color="0.35", lw=2.2,
                                  label="median over crossers"))
            handles.append(Line2D([], [], ls="", marker="o", ms=4.5, mfc="none",
                                  mec="0.35",
                                  label="never crosses (top edge)"
                                        + ("" if n_never else " -- none here")))
            ax.legend(handles=handles, fontsize=6.0, loc="upper left", frameon=False,
                      ncol=2, handletextpad=0.4, columnspacing=1.2)
    axes[-1].set_xlabel("phase (task order; `tipped' is a failure mode, not a rung)",
                        fontsize=8)
    fig.tight_layout()
    cap = (f"Rise-time LAG relative to `{LAG_REF}'.  For each run, "
           f"lag(phase) = rise({phase_thresh_str(thresh)}, phase) "
           f"- rise({phase_thresh_str(thresh)}, `{LAG_REF}'), where rise is the first "
           f"online sim step at which the run's rolling-{ROLL}-episode rate for that flag "
           f"reaches the threshold.  TRAINING RECORD: sampled actions on the policy's own "
           f"training starts; observation {OBS}, ladder {LADDER}.  {{dv3}} = DreamerV3 "
           f"losses in the r2dreamer chassis, {{r2dreamer}} = the port's contrastive "
           f"representation loss.  Datasets: {DATASET_DESC}.  One point per seed (jittered), thick "
           f"bar = median over the seeds that crossed, the small number under each group "
           f"is how many crossed.  A seed whose phase never reaches the threshold inside "
           f"its record is an OPEN marker on the dotted top edge and is counted there; it "
           f"is never plotted as lag 0 and never enters the median; a run still inside its "
           f"budget that has not crossed is PENDING, not plotted, and counted in the panel "
           f"title.  A seed whose "
           f"`{LAG_REF}' itself never crosses has no lag at all and is reported in the "
           f"panel title.  {{RLPD}} is absent from this figure: its episode record carries "
           f"no `farside'/`slide_event'/`home' flag (absent, not zero).  The nested_ramp "
           f"control is absent: it pays a different ladder.  Budgets 1 M (ae/af) or 2 M "
           f"(ag) online sim steps; the {{dv3}} local seeds ran on the pop-os AVX2 GPU "
           f"workstation, the cluster conditions on pax GPU nodes.")
    h = fig_caption(fig, cap)
    fig.subplots_adjust(bottom=h + 0.06, hspace=0.42)
    save(fig, outdir / f"fig_rise_lag_thresh{thresh_tag(thresh)}")
    return rows


def thresh_tag(t):
    return f"{t:g}".replace(".", "p")


def phase_thresh_str(t):
    return f"rate>={t:g}"


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


# ------------------------------------------------------- {RLPD} derived `home` curves

def fig_rlpd_derived_home(runs, outdir, rng):
    """{RLPD} `home` training curve DERIVED from the Slurm log's ep_rew_mean / 10."""
    rs_all = [r for r in runs if r.condition == "rlpd_cluster" and r.derived_home is not None
              and r.seed < SMOKE_SEED]
    rows = []
    if not rs_all:
        return rows
    budget = RLPD_BUDGET_DECISIONS * 4
    grid = np.arange(0, budget + GRID, GRID // 2)
    fig, ax = plt.subplots(figsize=(7.1, 3.6))
    handles = []
    for arm in ARMS:
        rs = sorted([r for r in rs_all if r.arm == arm], key=lambda r: r.seed)
        if not rs:
            continue
        mat = np.full((len(rs), grid.size), np.nan)
        for i, r in enumerate(rs):
            x, y = r.derived_home
            if x.size == 0:
                continue
            inside = (grid >= x[0]) & (grid <= x[-1])
            mat[i, inside] = np.interp(grid[inside], x, y)
        mean, lo, hi, nseed = bootstrap_band(mat, rng)
        mask = nseed >= MIN_SEEDS_BAND
        for j in range(grid.size):
            if nseed[j]:
                rows.append(dict(condition="rlpd_cluster", arm=arm,
                                 phase="home_derived_from_ep_rew_mean_div10_roll100",
                                 online_step=int(grid[j]), mean=mean[j], ci_lo=lo[j],
                                 ci_hi=hi[j], n_seeds=int(nseed[j]), in_band=bool(mask[j])))
        if len(rs) < MIN_SEEDS_BAND:
            for r in rs:
                x, y = r.derived_home
                ax.plot(x / 1e6, y, color=ARM_COLOR[arm], lw=0.8, alpha=0.8)
            tag = " per seed"
        else:
            for r in rs:
                x, y = r.derived_home
                ax.plot(x / 1e6, y, color=ARM_COLOR[arm], lw=0.4, alpha=0.25)
            ax.plot(grid[mask] / 1e6, mean[mask], color=ARM_COLOR[arm], lw=1.5)
            ok = mask & ~np.isnan(lo)
            ax.fill_between(grid[ok] / 1e6, lo[ok], hi[ok], color=ARM_COLOR[arm],
                            alpha=0.16, lw=0)
            tag = ""
        handles.append(Line2D([], [], color=ARM_COLOR[arm], lw=1.5,
                              label=f"{ARM_LABEL[arm]} (n={len(rs)}){tag}"))
    ax.set_xlim(0, budget / 1e6)
    ax.set_ylim(-0.03, 1.03)
    ax.set_xlabel("online sim frames (M) = decisions x 4; capped at 250k decisions",
                  fontsize=8)
    ax.set_ylabel("`home` rate, rolling 100 episodes\n(derived: ep_rew_mean / 10)",
                  fontsize=7.5)
    ax.grid(alpha=0.25, lw=0.4)
    ax.tick_params(labelsize=7)
    ax.legend(handles=handles, fontsize=6.5, loc="upper left", frameon=False)
    wrap_title(ax, "{RLPD} cluster -- `home` TRAINING CURVE derived from ep_rew_mean/10 "
                   f"(pixels, {LADDER})", width=110, fontsize=7.6)
    fig.tight_layout()
    cap = (f"{{RLPD}} (DrQ-style encoder, LN critic ensemble, pixels, ladder {LADDER}).  "
           f"The RLPD episode record has no `home` flag, so this curve is DERIVED from the "
           f"Slurm log: SB3's `ep_rew_mean` is the mean return over the last 100 online "
           f"episodes, and the ladder pays only +10 at `home` (tip penalty 0), so "
           f"ep_rew_mean / 10 = the rolling-100 `home` rate.  It is a rolling-100 window, "
           f"not the rolling-30 window of the other learning curves; sampled actions, the "
           f"policy's own training starts.  Thin lines = seeds; thick line + band = mean "
           f"and 95 % bootstrap CI ({BOOT} resamples) where >= {MIN_SEEDS_BAND} seeds "
           f"have data.  Records past 250k decisions (13 (ag) seeds trained on) are cut.  "
           f"Every seed with a log is drawn, including human s8, which is outside the n=8 "
           f"design of the results table.  "
           f"Datasets: {DATASET_DESC}.")
    h = fig_caption(fig, cap, width=125)
    fig.subplots_adjust(bottom=h + 0.14)
    save(fig, outdir / "fig_learning_curves_rlpd_home_derived")
    return rows


# ------------------------------------------------------------------- 4 x 4 results

def boot_ci(vals, rng, reps=BOOT):
    v = np.asarray(vals, dtype=float)
    if v.size < 2:
        return (None, None)
    bs = v[rng.integers(0, v.size, size=(reps, v.size))].mean(axis=1)
    lo, hi = np.percentile(bs, [2.5, 97.5])
    return float(lo), float(hi)


def permutation_p(a, b, rng):
    """Two-sided permutation p on the difference of means.  Exact over all C(n, k)
    relabelings when that is <= PERM_EXACT_MAX, else PERM_MC Monte-Carlo relabelings."""
    from itertools import combinations
    a, b = np.asarray(a, float), np.asarray(b, float)
    pool = np.concatenate([a, b])
    n, k = pool.size, a.size
    obs = abs(a.mean() - b.mean())
    tot = pool.sum()
    eps = 1e-12
    ncomb = math.comb(n, k)
    if ncomb <= PERM_EXACT_MAX:
        hits, count = 0, 0
        it = combinations(range(n), k)
        while True:
            chunk = np.fromiter((i for c in _take(it, 200_000) for i in c), dtype=np.int64)
            if chunk.size == 0:
                break
            idx = chunk.reshape(-1, k)
            sa = pool[idx].sum(axis=1)
            d = np.abs(sa / k - (tot - sa) / (n - k))
            hits += int((d >= obs - eps).sum())
            count += idx.shape[0]
        return hits / count, "exact", count
    hits = 0
    left = PERM_MC
    while left:
        m = min(left, 50_000)
        perm = np.argsort(rng.random((m, n)), axis=1)[:, :k]
        sa = pool[perm].sum(axis=1)
        d = np.abs(sa / k - (tot - sa) / (n - k))
        hits += int((d >= obs - eps).sum())
        left -= m
    return (hits + 1) / (PERM_MC + 1), "monte_carlo", PERM_MC


def _take(it, m):
    from itertools import islice
    return islice(it, m)


def results_4x4(runs, root: Path, rng):
    ledgers = read_ledgers(root)
    per_seed, cells, status = [], [], []
    for lab, conds in LEARNERS.items():
        for arm in ARMS:
            rs = [r for r in runs if r.condition in conds and r.arm == arm]
            design = DESIGN_SEEDS.get((lab, arm)) or ledgers.get((lab, arm)) or []
            by_seed = {}
            for r in sorted(rs, key=lambda r: r.seed):
                by_seed.setdefault(r.seed, r)
            stat, missing = [], []
            for seed in sorted(set(design) | set(by_seed)):
                r = by_seed.get(seed)
                in_design = seed in design
                if r is None:
                    reason = "no run in mirror (queued / not started / dead)"
                    got = None
                else:
                    got, reason = r.record_statistic()
                row = dict(learner=lab, dataset=arm, seed=seed, in_design=in_design,
                           run=(r.name if r else ""),
                           training_record=(bool(r.has_training_record) if r else False),
                           home=(got["home"] if got else None),
                           picked=(got["picked"] if got else None),
                           cells=(";".join(got["cells"]) if got else ""),
                           excluded_reason=("" if got and in_design else
                                            reason if not got else "outside the n=8 design"))
                per_seed.append(row)
                if got and in_design and got["home"] is not None:
                    stat.append((seed, got["home"], got["picked"]))
                elif in_design:
                    missing.append(seed)
            n_rec = sum(1 for s in design if s in by_seed and by_seed[s].has_training_record)
            homes = [h for _, h, _ in stat]
            picks = [p for _, _, p in stat if p is not None]
            hlo, hhi = boot_ci(homes, rng)
            plo, phi = boot_ci(picks, rng)
            cells.append(dict(
                learner=lab, dataset=arm, n_stat=len(stat), n_design=len(design),
                n_training_record=n_rec,
                mean_home=(float(np.mean(homes)) if homes else None), home_ci_lo=hlo,
                home_ci_hi=hhi,
                mean_picked=(float(np.mean(picks)) if picks else None), picked_ci_lo=plo,
                picked_ci_hi=phi, n_picked=len(picks),
                seeds_with_stat=" ".join(str(s) for s, _, _ in stat),
                seeds_missing=" ".join(str(s) for s in missing),
                statistic=("rnd30_sample home" if lab == "{Diffusion Policy}" else
                           "250k rnd30_mode home" if lab == "{RLPD}" else
                           "mean rnd30_mode home at 0.5M & 1M"),
                interim=len(stat) < 8,
                _homes=homes))
            status.append((lab, arm, len(stat), n_rec, len(design)))
    pairs = []
    for lab in LEARNERS:
        byarm = {c["dataset"]: c for c in cells if c["learner"] == lab}
        for i, a in enumerate(ARMS):
            for b in ARMS[i + 1:]:
                ca, cb = byarm[a], byarm[b]
                row = dict(learner=lab, a=a, b=b, n_a=ca["n_stat"], n_b=cb["n_stat"],
                           mean_a=ca["mean_home"], mean_b=cb["mean_home"])
                if ca["n_stat"] >= 3 and cb["n_stat"] >= 3:
                    p, method, nperm = permutation_p(ca["_homes"], cb["_homes"], rng)
                    row.update(diff=ca["mean_home"] - cb["mean_home"], p=p, method=method,
                               n_relabelings=nperm,
                               interim=(ca["n_stat"] < 8 or cb["n_stat"] < 8))
                else:
                    row.update(diff=None, p=None, method="not tested (n < 3)",
                               n_relabelings=0, interim=True)
                pairs.append(row)
    for c in cells:
        c.pop("_homes")
    return per_seed, cells, pairs, status


def print_status(status):
    """One line per learner x dataset: seeds with the statistic / seeds with any
    training record (/ seeds in the design)."""
    print("[status] learner x dataset: seeds with statistic / seeds with any training "
          "record (design n)")
    for lab, arm, n_stat, n_rec, n_design in status:
        print(f"[status] {lab:20s} {arm:10s} {n_stat:2d} / {n_rec:2d}  (design {n_design})")


def _f(v, nd=3):
    return "" if v is None else f"{v:.{nd}f}"


def write_4x4(cells, pairs, outdir: Path, tex_path: Path):
    idx = {(c["learner"], c["dataset"]): c for c in cells}
    write_csv(cells, outdir / "px_results_4x4.csv")
    write_csv(pairs, outdir / "px_results_4x4_pairwise.csv")

    def cell_md(c, key):
        if c["n_stat"] == 0:
            return f"n 0/{c['n_design']} --"
        lo, hi = c[f"{key}_ci_lo"], c[f"{key}_ci_hi"]
        ci = f" [{lo:.2f}, {hi:.2f}]" if lo is not None else ""
        m = c[f"mean_{key}"]
        ms = "--" if m is None else f"{m:.3f}"
        return f"n {c['n_stat']}/{c['n_design']}: {ms}{ci}"

    L = ["# Pixel `nested_sparse10` results, 4 learners x 4 demonstration datasets", "",
         "Generated by `baselines/diagnostics/px_phase_analysis.py` -- do not edit.", "",
         "Cell = n seeds with the statistic / n seeds in the design (trained or queued), "
         "mean over seeds, [95 % bootstrap CI over seeds, "
         f"{BOOT} resamples]. A seed without a cell is absent, not zero "
         "(missing seeds listed below).", "",
         "Statistic of record: {DreamerV3 losses} and {r2dreamer} = per-seed mean of the "
         "rnd30 MODE `home` rate at 0.5 M and 1 M online steps (both cells required; local "
         "seeds: nearest series checkpoint within 60k); {RLPD} = rnd30 MODE `home` at the "
         "250k-decision checkpoint; {Diffusion Policy} = rnd30 SAMPLE `home` (100k updates).",
         "", "Datasets: " + DATASET_DESC + ".", ""]
    for key, title in (("home", "`home` (statistic of record)"),
                       ("picked", "`picked` (same cells)")):
        L += [f"## {title}", "",
              "| learner | " + " | ".join(ARM_LABEL[a] for a in ARMS) + " |",
              "|---|" + "---|" * len(ARMS)]
        for lab in LEARNERS:
            L.append(f"| {lab} | " + " | ".join(cell_md(idx[(lab, a)], key)
                                               for a in ARMS) + " |")
        L.append("")
    L += ["## Pairwise permutation tests on the per-seed `home` statistic", "",
          f"Two-sided, difference of means; exact when C(n_a+n_b, n_a) <= {PERM_EXACT_MAX:,},"
          f" else {PERM_MC:,} Monte-Carlo relabelings. Tested only where both cells have "
          "n >= 3. `interim` = a cell has n < 8.", "",
          "| learner | a | b | n_a | n_b | mean_a | mean_b | diff | p | method | |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in pairs:
        L.append(f"| {p['learner']} | {p['a']} | {p['b']} | {p['n_a']} | {p['n_b']} | "
                 f"{_f(p['mean_a'])} | {_f(p['mean_b'])} | {_f(p['diff'])} | "
                 f"{_f(p['p'], 4)} | {p['method']} | "
                 f"{'interim' if p['interim'] and p['p'] is not None else ''} |")
    L += ["", "## Missing seeds (in the design, no statistic yet)", ""]
    for c in cells:
        if c["seeds_missing"]:
            L.append(f"- {c['learner']} / {c['dataset']}: seeds {c['seeds_missing']}")
    (outdir / "px_results_4x4.md").write_text("\n".join(L) + "\n")
    print(f"[md] {outdir / 'px_results_4x4.md'}")

    # --- LaTeX
    def tx(c, key):
        if c["n_stat"] == 0:
            return rf"\textemdash\,\tiny(0/{c['n_design']})"
        m = c[f"mean_{key}"]
        if m is None:
            return rf"\textemdash\,\tiny({c['n_stat']}/{c['n_design']})"
        lo, hi = c[f"{key}_ci_lo"], c[f"{key}_ci_hi"]
        ci = rf" \tiny[{lo:.2f},\,{hi:.2f}]" if lo is not None else ""
        return rf"{m:.3f}{ci}\,\tiny({c['n_stat']}/{c['n_design']})"

    def esc(s):
        return s.replace("{", r"\{").replace("}", r"\}")

    T = ["% generated by baselines/diagnostics/px_phase_analysis.py -- do not edit",
         r"\begin{table*}[t]", r"\centering", r"\footnotesize",
         r"\setlength{\tabcolsep}{4pt}", r"\resizebox{\textwidth}{!}{%",
         r"\begin{tabular}{ll" + "c" * len(ARMS) + "}", r"\toprule",
         r" & learner & " + " & ".join(ARM_LABEL[a] for a in ARMS) + r" \\", r"\midrule"]
    for key, name in (("home", r"\texttt{home}"), ("picked", r"\texttt{picked}")):
        first = True
        for lab in LEARNERS:
            T.append((name if first else "") + f" & {esc(lab)} & "
                     + " & ".join(tx(idx[(lab, a)], key) for a in ARMS) + r" \\")
            first = False
        T.append(r"\midrule" if key == "home" else r"\bottomrule")
    T += [r"\end{tabular}}",
          r"\caption{Pixel \texttt{nested\_sparse10} results: four learners $\times$ four "
          r"demonstration datasets. Entry: mean over seeds of the per-seed statistic, "
          r"95\,\% bootstrap CI over seeds in brackets, (seeds with the statistic / seeds "
          r"in the design). A seed without a cell is absent, not zero. Statistic: "
          r"\{DreamerV3 losses\} and \{r2dreamer\} = mean rnd30 MODE \texttt{home} rate of "
          r"the 0.5\,M and 1\,M online-step cells; \{RLPD\} = rnd30 MODE at 250k decisions; "
          r"\{Diffusion Policy\} = rnd30 SAMPLE after 100k updates. \texttt{picked} is read "
          r"from the same cells. Datasets: human = dHfull\_all (74 tapes); machine = "
          r"dDPfull\_first (72 tapes, Diffusion Policy teacher); planner = planner72 (72 "
          r"motion-planner tapes); r2dreamer teacher = 72 tapes from one human-trained "
          r"pixel r2dreamer. Cells with fewer than 8 seeds are interim.}",
          r"\label{tab:px-results-4x4}", r"\end{table*}"]
    tex_path.parent.mkdir(parents=True, exist_ok=True)
    tex_path.write_text("\n".join(T) + "\n")
    print(f"[tex] {tex_path}")


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
    ap.add_argument("--tex-4x4", default="paper/figures/px_results_4x4.tex",
                    help="LaTeX output for the 4 learners x 4 datasets results table")
    ap.add_argument("--status", action="store_true",
                    help="only discover and print the per-cell status lines (no figures)")
    ap.add_argument("--roll", type=int, default=ROLL)
    ap.add_argument("--grid", type=int, default=GRID)
    ap.add_argument("--boot", type=int, default=BOOT)
    ap.add_argument("--seed", type=int, default=BOOT_SEED)
    ap.add_argument("--lag-thresholds",
                    default=",".join(f"{t:g}" for t in LAG_THRESHOLDS),
                    help="comma-separated rolling-rate thresholds for the rise-time "
                         "LAG figure (fig_rise_lag_thresh<t>); one figure per value")
    a = ap.parse_args(argv)

    ROLL, GRID, BOOT = a.roll, a.grid, a.boot

    root = Path(os.path.expanduser(a.data_root))
    outdir = Path(os.path.expanduser(a.out_dir))
    outdir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)

    runs = discover(root)
    groups = group(runs)
    trained = OrderedDict((c, a) for c, a in groups.items() if c in TRAINED_RECORD_CONDITIONS)

    if a.status:
        _, _, _, status = results_4x4(runs, root, rng)
        print_status(status)
        return 0

    census = [dict(condition=r.condition, arm=r.arm, seed=r.seed, run=r.name,
                   source=r.source, path=r.path, origin=r.origin,
                   nominal_budget=r.nominal_budget, budget_used=r.budget,
                   achieved_online=r.achieved, episodes=r.n_episodes,
                   complete=r.complete, note=r.note, x_unit=r.x_unit,
                   phases_absent=",".join(p for p in PHASES
                                          if r.flags.get(p) is None) or "",
                   n_rnd30_cells=len([c for c in r.cells if c["cell"] == "rnd30_mode"]),
                   n_hold15_cells=len([c for c in r.cells if c["cell"] == "hold15_mode"]),
                   n_rnd30_sample_cells=len([c for c in r.cells
                                             if c["cell"] == "rnd30_sample"]),
                   training_record=r.has_training_record,
                   derived_home_logs=";".join(r.derived_home_logs),
                   excluded_cells=";".join(f"{p} ({w})" for p, w in r.excluded_cells))
              for r in sorted(runs, key=lambda r: (r.condition, r.arm, r.seed))]
    write_csv(census, outdir / "px_run_census.csv")

    curve_rows = []
    for cond, arms in trained.items():
        curve_rows += fig_learning_curves(cond, arms, outdir, rng)
    curve_rows += fig_rlpd_derived_home(runs, outdir, rng)
    write_csv(curve_rows, outdir / "px_learning_curves.csv")

    per_seed, cells = rise_rows(trained)
    write_csv(per_seed, outdir / "px_rise_time_per_seed.csv")
    write_csv(cells, outdir / "px_rise_time_cells.csv")
    write_tex(cells, trained, Path(os.path.expanduser(a.tex)))

    lag_all = []
    for t in [float(s) for s in a.lag_thresholds.split(",") if s.strip()]:
        lag_all += fig_rise_lag(groups, outdir, t)
    write_csv(lag_all, outdir / "px_rise_lag_per_seed.csv")

    write_csv(fig_steady_state(groups, outdir), outdir / "px_steady_state.csv")
    write_csv(fig_ignition(groups, outdir), outdir / "px_ignition.csv")

    # per-cell eval dump, so every eval number in the figures is traceable
    ev = []
    for r in runs:
        for c in r.cells:
            row = dict(condition=r.condition, arm=r.arm, seed=r.seed, run=r.name,
                       cell=c["cell"], milestone=c["milestone"], online=c["online"],
                       episodes=c["episodes"], checkpoint=c.get("checkpoint"),
                       path=c["path"])
            for p in PHASES + ["nested_v2", "contact_push", "slide_success"]:
                row[p] = c["stages"].get(p)
            ev.append(row)
    write_csv(sorted(ev, key=lambda r: (r["condition"], r["arm"], r["seed"],
                                        r["cell"], r["online"])),
              outdir / "px_eval_cells.csv")

    res_seed, res_cells, res_pairs, status = results_4x4(runs, root, rng)
    write_csv(res_seed, outdir / "px_results_4x4_per_seed.csv")
    write_4x4(res_cells, res_pairs, outdir, Path(os.path.expanduser(a.tex_4x4)))
    print_status(status)
    print("[done]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
