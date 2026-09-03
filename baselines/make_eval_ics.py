#!/usr/bin/env python
"""Generate baselines/eval_ics.json -- THE shared evaluation IC file for the final
round robin (paper/PREREG_final_round_robin_2026-08-23.md §5).

WHY: the 08-19 round robin evaluated each learner on different ICs (RLPD: 15 hard-coded
demo uids + 15 single draws of rng(k); DP: demo_ics(env)[:15] + 15 draws of rng(0); dv3:
random uid per reset; r2d: its own) -- see paper/CRITIQUE_design_final_rr_2026-08-23.md
R6/E3. One file, committed, consumed by every harness, ends that.

CONTENTS
  sel   15 demo uids -- SELECTION set. Byte-identical to the 15 uids cluster/sbatch_rlpd.sh
        hard-coded through 08-19 AND to ic_sampling.demo_ics(env)[:15] (the DP indist set),
        so the new block's selection numbers stay comparable with history.
  hold  15 OTHER success uids -- CONFIRMATION set, drawn once with rng(0) from the
        remaining success uids (status ok|ok_batch AND label==success), never overlapping sel.
  rnd   30 support ICs -- drawn by ic_sampling.sample_support_ics(env, 30, seed=0), i.e.
        EXACTLY the function every --random eval has always used; the first 15 are the
        draws the 08-19 DP evals used (rng(0), n=15 is a prefix of n=30 because the
        generator draws x,y per episode in order).

NO ENV NEEDED: GenesisCanEnv.__init__ only reads can_pos_recovery/trial_placements.json
(status filter ok|ok_batch) and world_cfg; a duck-typed table shim reproduces exactly the
attributes ic_sampling reads (placements, solved_uids, world_cfg), so the SAME functions
run here -- nothing is re-implemented. Run with any python that has numpy:
    python baselines/make_eval_ics.py                 # writes baselines/eval_ics.json
    python baselines/make_eval_ics.py --check          # regenerate in memory, diff vs file
The file is deterministic (seed 0) and versioned; regenerate only with a logged reason.
"""
import argparse
import json
import os
import pathlib as pl
import subprocess
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
import ic_sampling  # noqa: E402

OUT_DEFAULT = REPO / 'baselines' / 'eval_ics.json'
# The 15 selection uids cluster/sbatch_rlpd.sh hard-coded through 08-19 (line
# `UIDS="232 234 ... 251"`). Asserted below to equal demo_ics(env)[:15].
SEL_HISTORIC = [232, 234, 235, 236, 237, 239, 242, 243, 244, 245, 246, 247, 248, 250, 251]
N_SEL, N_HOLD, N_RND = 15, 15, 30


class _TableEnv:
    """Duck-typed stand-in for GenesisCanEnv's IC-relevant attributes (no world built).
    Mirrors genesis_can_env.GenesisCanEnv.__init__ lines 75-77 exactly."""

    def __init__(self):
        table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
        self.world_cfg = table['world']
        self.placements = {int(u): r for u, r in table['trials'].items()
                           if r['status'] in ('ok', 'ok_batch')}

    @property
    def solved_uids(self):
        return sorted(self.placements)


def _git():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO),
                              capture_output=True, text=True, timeout=5).stdout.strip() or 'unknown'
    except Exception:
        return 'unknown'


def _ser(ep):
    """reset-kwarg dict -> JSON-safe dict (tuples -> lists, numpy -> float)."""
    o = {}
    for k, v in ep.items():
        if v is None:
            o[k] = None
        elif isinstance(v, (tuple, list, np.ndarray)):
            o[k] = [float(x) for x in v]
        else:
            o[k] = v
    return o


def build(seed=0):
    env = _TableEnv()
    demo = ic_sampling.demo_ics(env, reps=1)           # the DP indist construction
    succ = [e['uid'] for e in demo]
    sel = succ[:N_SEL]
    assert sel == SEL_HISTORIC, (
        f'selection set drifted from the 08-19 hard-coded list: {sel} != {SEL_HISTORIC} '
        f'(trial_placements.json changed?) -- refusing; history would stop being comparable')
    rest = [u for u in succ if u not in sel]
    rng = np.random.default_rng(seed)
    hold = sorted(int(u) for u in rng.choice(rest, N_HOLD, replace=False))
    assert not set(hold) & set(sel)
    rnd = [_ser(e) for e in ic_sampling.sample_support_ics(env, N_RND, seed=seed)]
    # sanity: the first 15 rnd draws == the 08-19 DP random set (same fn, same seed, prefix)
    first15 = [_ser(e) for e in ic_sampling.sample_support_ics(env, 15, seed=seed)]
    assert rnd[:15] == first15, 'rng prefix property broken -- sample_support_ics changed'
    lo, hi = ic_sampling.support_box(env)
    return dict(
        version=1, generator='baselines/make_eval_ics.py', seed=seed, git=_git(),
        sel=sel, hold=hold, rnd=rnd,
        sel_note='15 demo uids == cluster/sbatch_rlpd.sh 08-19 UIDS == ic_sampling.demo_ics(env)[:15]',
        hold_note=f'15 success uids drawn rng({seed}) from the {len(rest)} remaining success uids; disjoint from sel',
        rnd_note=f'ic_sampling.sample_support_ics(env, {N_RND}, seed={seed}); first 15 == the 08-19 DP random set',
        success_uid_universe=succ, support_box=dict(lo=[float(x) for x in lo], hi=[float(x) for x in hi]),
        world_cfg=env.world_cfg,
    )


def episodes_from_file(icf, ic_set, index=None):
    """-> list of env.reset kwarg dicts for `ic_set` ('sel'|'hold'|'rnd'); index=k -> [k-th]."""
    d = json.load(open(icf)) if not isinstance(icf, dict) else icf
    if ic_set in ('sel', 'hold'):
        eps = [dict(uid=int(u)) for u in d[ic_set]]
    elif ic_set == 'rnd':
        eps = [dict(can_pos=tuple(e['can_pos']), goal_pos=tuple(e['goal_pos']), uid=None)
               for e in d['rnd']]
    else:
        raise SystemExit(f'unknown ic set {ic_set!r} (sel|hold|rnd)')
    if index is not None:
        assert 0 <= index < len(eps), f'--ic-index {index} out of range for {ic_set} (n={len(eps)})'
        eps = [eps[index]]
    return eps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=str(OUT_DEFAULT))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--check', action='store_true', help='regenerate and compare with --out; exit 1 on diff')
    args = ap.parse_args()
    d = build(args.seed)
    if args.check:
        old = json.load(open(args.out))
        keys = ('sel', 'hold', 'rnd')
        same = all(old.get(k) == d[k] for k in keys)
        print('eval_ics.json', 'MATCHES' if same else 'DIFFERS', 'regenerated content on', keys)
        sys.exit(0 if same else 1)
    pl.Path(args.out).write_text(json.dumps(d, indent=1) + '\n')
    print(f'wrote {args.out}: sel={d["sel"]} hold={d["hold"]} rnd=n{len(d["rnd"])} '
          f'(support box {d["support_box"]})')


if __name__ == '__main__':
    main()
