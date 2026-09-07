"""Machine-first arm (paper/MACHINE_FIRST_PLAN_2026-09-07.md §1, T1): measure the
pick-shaping potential's magnitude at the START states the reward-only world-model
teacher will see, so the r2dreamer lambda-return clamp can be registered at
1 + max|phi| BEFORE the T1 runs (the potential phi = -2*||eef - can|| is <= 0; with
phi(terminal) = 0 the shaped episode return telescopes to R + (-phi(s0)), so the
attainable shaped return is 1 + max|phi(s0)| and the clamp must cover it).

Start distributions measured (the same world w3 = gc_kp4_riser3_shelf6):
  * training resets: FullTaskEnv.reset() draws a success uid -> its recorded can placement
  * eval support box: eval_ics_rnd300.json (300 uniform starts in the demo bbox + 1 cm)
  * the 4 corners of that support box (the analytic worst case for the eval sets)
Prints every max from tool output; writes nothing.

usage: python baselines/mf_measure_phi.py [--rnd300 <json>] [--sim-variant gc_kp4_riser3_shelf6]
"""
import argparse
import json
import os
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rnd300', default=str(REPO / 'baselines' / 'eval_ics_rnd300.json'))
    ap.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
    ap.add_argument('--gamma', type=float, default=1.0 - 1.0 / 333)  # r2dreamer horizon 333
    args = ap.parse_args()

    from sim_variant_hook import apply_pre, apply_post
    from full_env import FullTaskEnv
    apply_pre(args.sim_variant)
    env = FullTaskEnv(backend='cpu', max_steps=1200, scope='pick', action_mode='delta_joint',
                      action_repeat=4, delta_ref='target', pick_shaping=True,
                      pick_shaping_gamma=args.gamma, pick_shaping_terminal_zero=True)
    apply_post(env, args.sim_variant)
    print(f'[world] {args.sim_variant} PICK_SHAPING_SCALE={env.PICK_SHAPING_SCALE} '
          f'gamma={args.gamma:.6f} success_uids={len(env.success_uids)}', flush=True)

    rows = []  # (label, uid_or_none, can_xy, |phi|)
    for u in sorted(int(x) for x in env.success_uids):
        env.reset(options={'uid': u})
        can = np.asarray(env.genv.w['bottle'].get_pos(), dtype=np.float64).reshape(-1)[:3]
        rows.append(('train_uid', u, can[:2].tolist(), -env._pick_phi()))
    ics = json.load(open(args.rnd300))
    box = ics['support_box']
    for ic in ics['rnd']:
        env.reset_to(dict(can_pos=ic['can_pos'], goal_pos=ic['goal_pos']))
        rows.append(('rnd300', None, list(ic['can_pos'][:2]), -env._pick_phi()))
    z = float(ics['rnd'][0]['can_pos'][2])
    for x in (box['lo'][0], box['hi'][0]):
        for y in (box['lo'][1], box['hi'][1]):
            env.reset_to(dict(can_pos=[x, y, z], goal_pos=ics['rnd'][0]['goal_pos']))
            rows.append(('box_corner', None, [x, y], -env._pick_phi()))
    ee = np.asarray(env.genv.tool_pos(), dtype=np.float64).reshape(-1)[:3]
    print(f'[eef@reset] tool_pos={np.round(ee, 4).tolist()} (same home pose every reset)')
    for lab in ('train_uid', 'rnd300', 'box_corner'):
        v = np.asarray([r[3] for r in rows if r[0] == lab])
        worst = max((r for r in rows if r[0] == lab), key=lambda r: r[3])
        print(f'[{lab}] n={v.size} |phi| min {v.min():.4f} median {np.median(v):.4f} max {v.max():.4f} '
              f'(worst uid={worst[1]} can_xy={np.round(worst[2], 4).tolist()})')
    m = max(r[3] for r in rows)
    print(f'[MAX] max|phi(s0)| over all {len(rows)} starts = {m:.4f} -> shaped return bound 1 + {m:.4f} = {1 + m:.4f}')


if __name__ == '__main__':
    main()
