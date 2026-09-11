#!/usr/bin/env python
"""Roll out a trained {RLPD} e2e policy with the TIP RULE DISABLED, one record per env frame.

    python baselines/eval_e2e_tipoff.py --ckpt <rlpd_final.zip> --tag dH_s940 \
        --out-dir <dir> [--ic-set rnd] [--procs 8] [--tip-deg 1e9] [--mode mode]

WHY A SEPARATE FILE (Lane 6, TIP_RULE_2026-09-11 §8)
-----------------------------------------------------
`paper/TIP_RULE_2026-09-11.md` §1-§7 measures the tip rule on DEMONSTRATIONS, where it turns out
to be nearly inert (it removes 8 decisions of 66,055). The coordinator's pilot numbers say the
opposite happens in TRAINING: the rule ends 33-52 % of {RLPD} episodes. A demonstration cannot
answer what the rule costs a LEARNER, so this file asks the same counterfactual question of a
policy: disable the rule, run the full 300-decision horizon, and watch what the can does after
the frame the rule would have fired on.

It is a deliberate COPY of `eval_e2e.py`'s env/IC/policy contract rather than an edit of it --
the same rule the 2026-09-10 annotators followed, because `eval_e2e.py` is read at run time by
jobs in flight. It reuses `make_eval_ics.episodes_from_file` and the env's OWN reset (redirected
through the same `genv.reset` hook), so no start or bookkeeping is re-implemented here. It
produces NO cell and writes no `metrics.json`: it is a diagnostic, not a score.

The override is `env.TIP_DEG = <big>` on the INSTANCE; `FullTaskEnv.TIP_DEG` is never written
and that is asserted. `env.terminal_stages = ()` likewise, so a slide does not cut the horizon.
"""
import argparse
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

TIP_OFF = 1.0e9
DJ_CAP, DJ_LEASH_MULT = 0.025, 5.0
FLAGS = ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push', 'nested_v2',
         'slide_success', 'picked', 'placed_v2', 'contact', 'can_goal_contact', 'in_band')


def build(sim_variant, max_steps, repeat, ladder, tip_deg, render=False):
    os.environ['GENESIS_SIM_VARIANT'] = sim_variant
    from sim_variant_hook import apply_pre, apply_post
    from full_env import FullTaskEnv, refuse_legacy_gates
    refuse_legacy_gates()
    apply_pre(sim_variant)
    env = FullTaskEnv(backend='cpu', max_steps=max_steps, scope='full', ladder=ladder,
                      action_mode='delta_joint', delta_cap=DJ_CAP,
                      delta_leash_mult=DJ_LEASH_MULT, action_repeat=repeat, delta_ref='target',
                      render_size=None)
    apply_post(env, sim_variant)
    assert env.scope == 'full' and env.action_repeat == repeat and env.delta_ref == 'target'
    assert env.max_steps == max_steps and not env.phase_sparse and not env.goalward_shaping
    cls = float(FullTaskEnv.TIP_DEG)
    env.TIP_DEG = float(tip_deg)
    env.terminal_stages = ()
    assert float(FullTaskEnv.TIP_DEG) == 60.0 == cls, 'the class default must not move'
    print(f'[tipoff] instance TIP_DEG {env.TIP_DEG:g} (class default {cls:g} untouched), '
          f'terminal_stages {env.terminal_stages}, ladder {env.ladder} {env.stage_reward}',
          flush=True)
    return env


def run_shard(args, ics, idx):
    import torch
    from stable_baselines3 import SAC
    sc = json.load(open(str(args.ckpt).rsplit('.zip', 1)[0] + '.action_mode.json'))
    assert sc['action_mode'] == 'delta_joint' and sc['delta_ref'] == 'target', sc
    assert int(sc['action_repeat']) == 4 and abs(float(sc['delta_cap']) - DJ_CAP) < 1e-9, sc
    env = build(sc['sim_variant'], args.max_steps, int(sc['action_repeat']),
                sc.get('ladder') or 'staged', args.tip_deg)
    _genv_reset, cur = env.genv.reset, {'ic': None}
    env.genv.reset = lambda *a, **k: _genv_reset(**cur['ic'])
    model = SAC.load(str(args.ckpt), device='cpu')
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    det = (args.mode == 'mode')

    out = pl.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for k, ic in zip(idx, ics):
        cur['ic'] = ic
        obs, _ = env.reset(options={'uid': int(sorted(env.success_uids)[0])})
        rec = {n: [] for n in ('dec', 'tilt', 'grip', 'can_z', 'dist_xy', 'lever')}
        flg = {n: [] for n in FLAGS}
        state = dict(dec=0)
        inner = env._step_once

        def wrapped(action):
            o = inner(action)
            tr, info = env._track, o[4]
            bp = np.asarray(env.genv.w['bottle'].get_pos(), np.float64).reshape(-1)
            rec['dec'].append(state['dec'])
            rec['tilt'].append(float(tr['can_tilt_deg']))
            rec['grip'].append(float(tr['grip_cmd']))
            rec['can_z'].append(float(bp[2]))
            rec['dist_xy'].append(float(tr['dist_xy_m']))
            rec['lever'].append(float(tr['lever_m']))
            for n in FLAGS:
                flg[n].append(1 if (tr[n] if n in tr else info.get(n)) else 0)
            return o

        env._step_once = wrapped
        t0, ep_r, end = time.time(), 0.0, 'timeout'
        n_dec = args.max_steps // int(sc['action_repeat'])
        try:
            for t in range(n_dec):
                state['dec'] = t
                a, _ = model.predict(obs, deterministic=det)
                obs, r, term, trunc, info = env.step(np.asarray(a, np.float32))
                ep_r += float(r)
                if term or trunc:
                    end = 'tipped' if info.get('tipped') else 'truncated' if trunc else 'terminated'
                    break
        finally:
            del env._step_once
        settle = env.genv.end_of_episode()
        from stage_predicates import tilt_from_quat
        final_tilt = tilt_from_quat(np.asarray(env.genv.w['bottle'].get_quat()).reshape(-1)[:4])
        arrs = {n: np.asarray(v, np.float32) for n, v in rec.items() if n != 'dec'}
        arrs['dec'] = np.asarray(rec['dec'], np.int32)
        for n in FLAGS:
            arrs['f_' + n] = np.asarray(flg[n], np.uint8)
        np.savez_compressed(out / f'{args.tag}_ep{k:03d}.npz', **arrs)
        g = set(env._granted)
        rows.append(dict(tag=args.tag, ep=int(k), ic=dict(can_pos=list(map(float, ic['can_pos']))
                                                          if ic.get('can_pos') is not None else None,
                                                          uid=ic.get('uid')),
                         end_reason=end, decisions=len(rec['dec']) // int(sc['action_repeat']),
                         frames=len(rec['dec']), reward=ep_r, tip_deg=float(env.TIP_DEG),
                         final_tilt_deg=round(float(final_tilt), 2),
                         settle_nested=bool(settle['nested']),
                         stages={s: bool(s in g) for s in
                                 ('picked', 'placed_v2', 'contact_push', 'nested_v2',
                                  'slide_success')},
                         seconds=round(time.time() - t0, 1)))
        print(f'[{args.tag}] ep{k}: {rows[-1]["frames"]} frames, end {end}, reward {ep_r:.1f}, '
              f'final tilt {final_tilt:.1f}, stages {[s for s in rows[-1]["stages"] if rows[-1]["stages"][s]]} '
              f'[{rows[-1]["seconds"]:.0f}s]', flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--ic-file', default='baselines/eval_ics.json')
    ap.add_argument('--ic-set', default='rnd')
    ap.add_argument('--mode', choices=('mode', 'sample'), default='mode')
    ap.add_argument('--max-steps', type=int, default=1200)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--tip-deg', type=float, default=TIP_OFF)
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--shard', type=int, default=None)
    ap.add_argument('--nshards', type=int, default=None)
    args = ap.parse_args()

    from make_eval_ics import episodes_from_file
    icf = args.ic_file if os.path.isabs(args.ic_file) else str(REPO / args.ic_file)
    ics = episodes_from_file(icf, args.ic_set)
    if args.shard is not None:
        pick = [(i, e) for i, e in enumerate(ics) if i % args.nshards == args.shard]
        rows = run_shard(args, [e for _, e in pick], [i for i, _ in pick])
        json.dump(rows, open(f'{args.out_dir}/_s{args.tag}_{args.shard}.json', 'w'), indent=1)
        return
    n = max(1, min(int(args.procs), 8, len(ics)))
    if n == 1:
        rows = run_shard(args, ics, list(range(len(ics))))
    else:
        pl.Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        base = [sys.executable, os.path.abspath(__file__), '--ckpt', str(args.ckpt),
                '--tag', args.tag, '--out-dir', args.out_dir, '--ic-file', args.ic_file,
                '--ic-set', args.ic_set, '--mode', args.mode, '--max-steps', str(args.max_steps),
                '--seed', str(args.seed), '--tip-deg', repr(float(args.tip_deg))]
        ps = [subprocess.Popen(base + ['--shard', str(i), '--nshards', str(n)]) for i in range(n)]
        rcs = [p.wait() for p in ps]
        assert all(r == 0 for r in rcs), f'shard failures: {rcs}'
        rows = []
        for i in range(n):
            p = f'{args.out_dir}/_s{args.tag}_{i}.json'
            rows += json.load(open(p))
            os.remove(p)
    rows.sort(key=lambda r: r['ep'])
    json.dump(dict(tag=args.tag, ckpt=os.path.abspath(args.ckpt), ic_set=args.ic_set,
                   mode=args.mode, tip_deg=args.tip_deg, n=len(rows), rows=rows),
              open(f'{args.out_dir}/_rollout_{args.tag}.json', 'w'), indent=1)
    print(f'wrote {args.out_dir}/_rollout_{args.tag}.json ({len(rows)} episodes)')


if __name__ == '__main__':
    main()
