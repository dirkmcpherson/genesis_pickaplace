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


def _paths_for(uids):
    out = []
    for u in uids:
        p = REPO / DEMO_DIR / f'{u}.npz'
        assert p.exists(), f'missing gate demo {p}'
        out.append(str(p))
    return out


def main():
    from full_env import FullTaskEnv, STAGE_REWARD
    from train_sacfd_full import delta_encode_transitions
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
                      action_mode='delta_joint')
    from pick_env import GRIP_CLOSED_FRAC
    print(f'[gate] env built | pick_z={env.pick_z:.4f} delta_cap={env.delta_cap}',
          flush=True)

    # ---- (1) tensor equality: DemoData actions == encoder output ----
    all_paths = _paths_for(GATE_UIDS)
    trans = delta_encode_transitions(all_paths, env.pick_z, 'pick', env.delta_cap)
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
        ep = delta_encode_transitions([p], env.pick_z, 'full', env.delta_cap)
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
