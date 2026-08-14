"""Durable open-loop gate for the delta_joint demo pipeline shared by SACfD and RLPD.

Two guarantees, cheap to re-run before any cluster spend (RLPD_PLAN.md ladder (b)):

  1. TENSOR EQUALITY: RLPD's DemoData is fed the EXACT output of
     train_sacfd_full.delta_encode_transitions -- the same encoder SACfD uses -- so
     the human-vs-model demo-source comparison differs only in the algorithm, never
     in the data. Asserted by reconstructing both and comparing arrays bitwise.

  2. OPEN-LOOP REPLAY: integrating the delta-encoded demo actions through
     FullTaskEnv(action_mode='delta_joint') must RE-EARN THE DEMONSTRATED LIFT --
     the relabeler's own pick predicate (can_z > pick_z AND grip commanded closed),
     i.e. the exact condition the demo's pick REWARD is assigned on -- for gentle,
     resettable pick demos (see GATE_UIDS). If the delta cap/leash or the encoder
     drift, the replay stops lifting and this gate fails LOUD before a 7h train
     wastes GPU.

Usage:  .venv-eval/bin/python baselines/rl/sacfd_delta_gate.py
"""
import os
import argparse
import glob
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

# Gate uids empirically selected (2026-08-12) as gentle-speed, RESETTABLE pick demos
# whose delta-encoded replay reliably re-earns the env pick predicate. NOTE on why
# NOT 232/242/243 (an earlier guess): the episodes_pick_phase_all pick demos are all
# MARGINAL-TERMINAL -- truncated ~2 frames after the can first crosses pick_z, so max
# lift margin is ~7mm. Fast demos (max joint step > cap 0.025) whose replay saturates
# the delta cap lose that razor-thin margin (uid 232 lifts to only 0.1456), and some
# pick-phase uids (e.g. 259) are not in the env placements map at all. Of the gentlest
# resettable pick demos 9/11 re-earn the pick with lift ~1.0; the five below are the
# most robust (lift 0.99-1.16, joint-tracking error 0.03-0.05 rad, well within leash).
DEMO_DIR = 'baselines/episodes_pick_phase_all'
GATE_UIDS = [308, 325, 297, 326, 265]
MIN_PASS = 4        # >=4/5 re-earn (one physics flake tolerated); deterministic CPU sim


def _paths_for(uids, demo_dir=DEMO_DIR):
    out = []
    for u in uids:
        p = REPO / demo_dir / f'{u}.npz'
        assert p.exists(), f'missing gate demo {p}'
        out.append(str(p))
    return out


def _encode(paths, pick_z, scope, cap, repeat, delta_ref='target'):
    """stride x reference encoder, selected EXPLICITLY (no silent fallback).
    delta_ref='measured' pairs with FullTaskEnv(delta_ref='measured')."""
    from train_sacfd_full import (delta_encode_transitions,
                                  delta_encode_transitions_repeat,
                                  delta_encode_transitions_measured,
                                  delta_encode_transitions_measured_repeat)
    assert delta_ref in ('target', 'measured'), delta_ref
    if delta_ref == 'measured':
        if repeat > 1:
            return delta_encode_transitions_measured_repeat(paths, pick_z, scope,
                                                            cap, repeat)
        return delta_encode_transitions_measured(paths, pick_z, scope, cap)
    if repeat > 1:
        return delta_encode_transitions_repeat(paths, pick_z, scope, cap, repeat)
    return delta_encode_transitions(paths, pick_z, scope, cap)


def all_demos_sweep(demo_dir, repeat, shard_idx=0, shard_n=1, delta_ref='target'):
    """ALL-demos, ALL-phases replay at action-repeat=N, measured by the env's OWN
    stage grants (FullTaskEnv._granted), so this shares the exact measurement TRAINING
    credits -- no reimplemented stage ladder. Prints one parseable line per demo:
        SWEEP uid=<u> repeat=<N> stage=<name> rank=<k> label=<recorded>
    The honest comparison across N is repeat-1 vs repeat-N under this SAME measurement
    (run the sweep at N=1 too), which isolates subsampling from any predicate mismatch
    against the recorded d['stage'] label.
    """
    import glob
    from full_env import FullTaskEnv
    from train_sacfd_full import STAGE_RANK
    RANK2NAME = {v: k for k, v in STAGE_RANK.items()}

    env = FullTaskEnv(backend='cpu', max_steps=4000, scope='full',
                      action_mode='delta_joint', action_repeat=repeat,
                      delta_ref=delta_ref)
    print(f'[sweep] env built | repeat={repeat} ref={delta_ref} pick_z={env.pick_z:.4f} '
          f'cap={env.delta_cap} action_repeat={env.action_repeat}', flush=True)

    paths = sorted(glob.glob(str(REPO / demo_dir / '*.npz')),
                   key=lambda p: int(pl.Path(p).stem))
    if shard_n > 1:
        paths = paths[shard_idx::shard_n]     # every shard_n-th demo (interleaved balance)
    for p in paths:
        d = np.load(p, allow_pickle=True)
        uid = int(d['uid'])
        label = str(d['stage']) if 'stage' in d.files else '?'
        ep = _encode([p], env.pick_z, 'full', env.delta_cap, repeat, delta_ref)
        acts = [t[1] for t in ep]
        try:
            env.reset(options={'uid': uid})
        except Exception as e:
            print(f'SWEEP uid={uid} repeat={repeat} stage=UNRESETTABLE rank=-1 '
                  f'label={label} ({type(e).__name__})', flush=True)
            continue
        for a in acts:
            _o, _r, term, trunc, _i = env.step(a)
            if term or trunc:
                break
        rank = max((STAGE_RANK.get(s, 0) for s in env._granted), default=0)
        print(f'SWEEP uid={uid} repeat={repeat} ref={delta_ref} stage={RANK2NAME.get(rank, "no-pick")} '
              f'rank={rank} label={label} n_dec={len(acts)}', flush=True)
    print(f'[sweep] DONE repeat={repeat} demo_dir={demo_dir}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--action-repeat', type=int, default=1,
                    help='validate the decision-level (action-repeat N) delta pipeline: '
                         'demos encoded by delta_encode_transitions_repeat and replayed '
                         'through FullTaskEnv(action_repeat=N) must re-earn the pick. '
                         'N=1 is the original stride-1 gate.')
    ap.add_argument('--all-demos', metavar='DIR', default=None,
                    help='sweep mode: replay EVERY demo in DIR at --action-repeat and '
                         'report the env-granted stage (all phases), instead of the '
                         '5-uid pick gate. Used to find the coarsest N safe for the '
                         'full task.')
    ap.add_argument('--delta-ref', choices=['target', 'measured'], default='target',
                    help="what a delta is applied to: 'target' = running-target "
                         "integration (existing; P1 frozen drift), 'measured' = "
                         "re-referenced to measured qpos each step (self-healing). "
                         "Applies to gate AND sweep; encoder + env selected together.")
    ap.add_argument('--demo-dir', default=DEMO_DIR,
                    help='source of the 5 GATE_UIDS tapes. Default = the original '
                         'episodes_pick_phase_all (unchanged gate). Point it at '
                         'baselines/episodes_delta_rerecord to gate the closed-loop '
                         're-recorded tapes, which are NATIVE to delta_ref=measured '
                         '(pair with --delta-ref measured).')
    ap.add_argument('--shard-idx', type=int, default=0)
    ap.add_argument('--shard-n', type=int, default=1,
                    help='sweep mode: replay only paths[shard_idx::shard_n], so many '
                         'workers can split one repeat across cores (CPU sim is ~1 core '
                         'per proc).')
    args = ap.parse_args()
    repeat = max(1, int(args.action_repeat))

    if args.all_demos:
        all_demos_sweep(args.all_demos, repeat, args.shard_idx, args.shard_n,
                        args.delta_ref)
        return

    from full_env import FullTaskEnv, STAGE_REWARD
    from rlpd_sac import DemoData
    import torch as th

    # scope='full' so the env does NOT terminate early: the gate criterion is the
    # RELABELER's OWN pick predicate (can_z > pick_z AND grip commanded closed) -- the
    # exact condition the demo's pick REWARD is assigned on -- so this validates that
    # the delta ACTION labels are consistent with the delta REWARD labels. (The env's
    # scope=pick termination uses the HARDENED sustained-hold predicate, which the
    # marginal-terminal pick-phase demos -- truncated ~2 frames past pick_z -- can
    # never satisfy even under a perfect replay; see GATE_UIDS note above.)
    env = FullTaskEnv(backend='cpu', max_steps=1800, scope='full',
                      action_mode='delta_joint', action_repeat=repeat,
                      delta_ref=args.delta_ref)
    from pick_env import GRIP_CLOSED_FRAC
    print(f'[gate] env built | pick_z={env.pick_z:.4f} delta_cap={env.delta_cap} '
          f'delta_leash={env.delta_leash} delta_ref={env.delta_ref} '
          f'action_repeat={env.action_repeat} demo_dir={args.demo_dir}', flush=True)
    assert env.delta_ref == args.delta_ref, (env.delta_ref, args.delta_ref)

    # ---- (1) tensor equality: DemoData actions == encoder output ----
    all_paths = _paths_for(GATE_UIDS, args.demo_dir)
    trans = _encode(all_paths, env.pick_z, 'pick', env.delta_cap, repeat, args.delta_ref)
    dd = DemoData(trans, action_transform=None, device=th.device('cpu'))
    ref_act = np.stack([t[1] for t in trans]).astype(np.float32)
    ref_obs = np.stack([t[0] for t in trans]).astype(np.float32)
    assert np.array_equal(dd.actions.numpy(), ref_act), 'DemoData actions != encoder'
    assert np.array_equal(dd.observations.numpy(), ref_obs), 'DemoData obs != encoder'
    print(f'[gate] TENSOR-EQUALITY OK: {dd.n} transitions, {dd.n_rewarded} rewarded, '
          f'action dim {dd.actions.shape[1]}', flush=True)

    # ---- (2) open-loop replay: delta actions must re-earn the demonstrated LIFT ----
    passed = 0
    for uid, p in zip(GATE_UIDS, all_paths):
        # encode on the FULL episode (scope='full' => keep post-pick rows, no done cut)
        ep = _encode([p], env.pick_z, 'full', env.delta_cap, repeat, args.delta_ref)
        acts = [t[1] for t in ep]
        env.reset(options={'uid': uid})
        picked, canz_max, steps = False, -9.0, 0
        for a in acts:
            obs, _r, _term, trunc, _info = env.step(a)
            steps += 1
            canz = float(obs[10])
            canz_max = max(canz_max, canz)
            grip_cmd = (float(np.clip(a[6], -1.0, 1.0)) + 1.0) / 2.0   # -> [0,1]
            if canz > env.pick_z and grip_cmd > GRIP_CLOSED_FRAC:
                picked = True                # matches relabel_full's pick predicate
            if trunc:
                break
        passed += int(picked)
        print(f'[gate] uid {uid}: re-earned_lift={picked} canz_max={canz_max:.4f} '
              f'(pick_z={env.pick_z:.4f}) steps={steps} demo_len={len(acts)}',
              flush=True)

    print(f'[gate] OPEN-LOOP REPLAY {passed}/{len(GATE_UIDS)} re-earned the '
          f'demonstrated lift (need >={MIN_PASS})', flush=True)
    ok = passed >= MIN_PASS
    print('GATE PASS' if ok else 'GATE FAIL', flush=True)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
