"""Job #1 (new box): is the env-vs-replay arm divergence caused by per-step obs reads?

Runs the env path for a uid with selected per-step reads DISABLED and dumps the same
[bottle xyz, eef xyz] trace as trace_env.py / trace_replay.py for comparison.

    python baselines/diagnostics/trace_env_ablate.py <uid> <out.npy> [--no-contacts] [--no-effort] [--no-obs]

--no-contacts : skip w['bottle'].get_contacts(w['goal']) in step()   (contact metric off)
--no-effort   : skip get_dofs_control_force in _obs()                (grip_effort = 0)
--no-obs      : skip _obs() entirely (reset still returns one; step returns None)

The bottle/eef get_pos reads needed for the trace itself stay on in every variant
(trace_replay.py does those too, so they are common-mode).
"""
import sys, argparse, numpy as np, pathlib as pl, torch
sys.path.insert(0, 'baselines'); sys.path.insert(0, 'can_pos_recovery')

ap = argparse.ArgumentParser()
ap.add_argument('uid', type=int)
ap.add_argument('out')
ap.add_argument('--no-contacts', action='store_true')
ap.add_argument('--no-effort', action='store_true')
ap.add_argument('--no-obs', action='store_true')
ap.add_argument('--f64', action='store_true', help='feed float64 actions (no float32 cast)')
ap.add_argument('--raw-grip', nargs='?', const=True, default=False,
                help='bypass env.step gripper clip: command gripper_targets(raw gp) '
                     'exactly like the replay path (gp can be <0 or >100). '
                     "Pass 'env' to use the official env.step(grip_motor=...) path.")
args = ap.parse_args()

import genesis_can_env as gce
from genesis_can_env import GenesisCanEnv, np_
from replay_harness import load_episode

env = GenesisCanEnv(backend='cpu')

if args.no_contacts:
    class _NoContacts:
        def __init__(self, ent): self._ent = ent
        def __getattr__(self, k): return getattr(self._ent, k)
        def get_contacts(self, other): return {'position': np.zeros((0, 3))}
    env.w['bottle'] = _NoContacts(env.w['bottle'])
if args.no_effort:
    class _NoEffort:
        def __init__(self, ent): self._ent = ent
        def __getattr__(self, k): return getattr(self._ent, k)
        def get_dofs_control_force(self, *a, **kw): raise RuntimeError('effort read disabled')
    env.w['kinova'] = _NoEffort(env.w['kinova'])   # _obs() try/except -> grip_effort=0.0
if args.no_obs:
    env._obs = lambda: None

vel, gp = load_episode(args.uid)
env.reset(uid=args.uid)

if args.raw_grip:
    # command the gripper with the EXACT recorded motor value (replay-path convention);
    # env.step's [0,1] clip zeroes recorded negatives (fully-open overshoot, e.g. uid 300
    # min gp=-0.46) -> ~5e-3 rad finger-aperture error vs replay.
    from replay_harness import gripper_targets
    _orig_step = env.step

    def raw_step(a_arm, gp_raw):
        w = env.w
        w['kinova'].control_dofs_position(np.asarray(a_arm, dtype=np.float64),
                                          dofs_idx_local=w['kdofs'][:6])
        w['kinova'].control_dofs_position(np.array(gripper_targets(gp_raw)),
                                          dofs_idx_local=np.array(w['kdofs'][-4:]))
        for _ in range(3):
            w['scene'].step()
        return None, False, {}

rows = []
for i in range(len(vel) - 1):
    if args.raw_grip == 'env':
        # official fixed path: env.step(grip_motor=...) as the collector now uses
        a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]).astype(np.float32)
        env.step(a, grip_motor=gp[i], arm_cmd=vel[i])
    elif args.raw_grip:
        raw_step(vel[i], gp[i])
    else:
        a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]])
        if not args.f64:
            a = a.astype(np.float32)
        obs, done, info = env.step(a)
    bp = np_(env.w['bottle'].get_pos()); ep = np_(env.w['eef'].get_pos())
    rows.append([bp[0], bp[1], bp[2], ep[0], ep[1], ep[2]])
np.save(args.out, np.array(rows))
print(f"ablate-trace done uid={args.uid} n={len(rows)} "
      f"no_contacts={args.no_contacts} no_effort={args.no_effort} no_obs={args.no_obs}")
