"""SACfD on the FULL task: FullTaskEnv (staged reward) + demo transitions from
episodes_all relabeled with the SAME staged rewards.

Relabel per frame from the recorded 17-dim states, with per-frame geometric proxies
mirroring the env predicates:
    picked  : can_z > pick_z AND gripper cmd > 0.3        (exact mirror of env)
    placed  : picked-before AND can_z in shelf band       (proxy)
    contact : placed-before AND |can_xy - goal_xy| < 0.070 (proxy: cans touching)
    nested  : contact-before AND can tilt < 20 deg        (proxy)
Each stage's reward is GATED by the episode's env-measured final stage (npz 'stage'
field from collect_all_classified): an episode that never reached contact can never
grant contact reward, so proxy false-positives cannot manufacture reward. done=True
only at the nested frame (mirrors env termination).

Usage: train_sacfd_full.py --steps 400000 --demo-dir baselines/episodes_all
                           --out-dir baselines/rl/checkpoints/sacfd_full --device cuda
"""
import os
import argparse, glob, json, pathlib as pl, sys, time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
import demo_buffer  # noqa: E402
import pick_env  # noqa: E402
from full_env import FullTaskEnv, STAGE_REWARD  # noqa: E402

STAGE_RANK = {'no-pick': 0, 'picked': 1, 'placed': 2, 'contact': 3, 'nested': 4}
SHELF_LO, SHELF_HI = 0.12, 0.20    # can_z band for the placed proxy (BOX_TOP 0.11 + 1..7cm)
TOUCH_XY = 0.070                   # cans touching: centers ~1 diameter (0.066) apart


def tilt_from_quat(q):
    """Tilt (deg) of body z-axis from world z for wxyz quats (vectorized)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    cz = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(cz, -1.0, 1.0)))


def relabel_full(paths, pick_z):
    """-> list of (obs, action, reward, next_obs, done) with staged rewards."""
    transitions, stats = [], {k: 0 for k in STAGE_REWARD}
    for p in paths:
        d = np.load(p, allow_pickle=True)
        s, a = d['states'].astype(np.float32), d['actions'].astype(np.float32)
        ep_stage = str(d['stage']) if 'stage' in d.files else 'contact'
        rank = STAGE_RANK.get(ep_stage, 0)
        n = len(s) - 1
        if n < 2:
            continue
        can_z = s[:, 10]; grip = a[:, 6]
        picked_f = (can_z[:-1] > pick_z) & (grip[:-1] > pick_env.GRIP_CLOSED_FRAC)
        rew = np.zeros(n, dtype=np.float32)
        done = np.zeros(n, dtype=bool)
        granted_at = {}
        j_pick = int(np.argmax(picked_f)) if picked_f.any() and rank >= 1 else -1
        if j_pick == 0:
            continue   # bad state at frame 0
        if j_pick > 0:
            rew[j_pick] += STAGE_REWARD['picked']; granted_at['picked'] = j_pick
            stats['picked'] += 1
            if rank >= 2:
                pl_f = (np.arange(n) > j_pick) & (can_z[:-1] > SHELF_LO) & (can_z[:-1] < SHELF_HI)
                j_pl = int(np.argmax(pl_f)) if pl_f.any() else -1
                if j_pl > 0:
                    rew[j_pl] += STAGE_REWARD['placed']; granted_at['placed'] = j_pl
                    stats['placed'] += 1
                    if rank >= 3:
                        dxy = np.hypot(s[:-1, 8] - s[:-1, 15], s[:-1, 9] - s[:-1, 16])
                        c_f = (np.arange(n) > j_pl) & (dxy < TOUCH_XY)
                        j_c = int(np.argmax(c_f)) if c_f.any() else -1
                        if j_c > 0:
                            rew[j_c] += STAGE_REWARD['contact']; granted_at['contact'] = j_c
                            stats['contact'] += 1
                            if rank >= 4:
                                tilt = tilt_from_quat(s[:-1, 11:15])
                                n_f = (np.arange(n) > j_c) & (tilt < 20.0)
                                j_n = int(np.argmax(n_f)) if n_f.any() else -1
                                if j_n > 0:
                                    rew[j_n] += STAGE_REWARD['nested']
                                    done[j_n] = True
                                    stats['nested'] += 1
        end = int(np.argmax(done)) + 1 if done.any() else n
        for i in range(end):
            transitions.append((s[i], a[i], float(rew[i]), s[i + 1], bool(done[i])))
    return transitions, stats


def delta_encode_transitions(paths, pick_z, scope, cap):
    """relabel_full per episode, then replace each transition's RAW absolute
    action [6 rad, grip 0..1] with the delta-normalized action the delta_joint
    env expects: arm = clip((cmd_t - cmd_{t-1})/cap, -1, 1) (row 0 vs the
    measured start pose -- max |cmd_0 - q_0| over demos is 0.012 rad), grip =
    raw*2-1. Same encoding as to_dreamer_demos --action-encoding delta_joint;
    integrating these through FullTaskEnv(action_mode='delta_joint') reproduces
    the validated commanded replay (gate: sacfd_delta_gate below)."""
    out = []
    for p in paths:
        trans, _ = relabel_full([p], pick_z)
        if not trans:
            continue
        if scope == 'pick':
            trans = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                     (o, a, r, o2, d) for (o, a, r, o2, d) in trans]
        cmds = np.stack([t[1] for t in trans]).astype(np.float64)
        prev = np.concatenate([trans[0][0][None, :6].astype(np.float64),
                               cmds[:-1, :6]])
        dq = np.clip((cmds[:, :6] - prev) / cap, -1.0, 1.0)
        grip = np.clip(cmds[:, 6], 0.0, 1.0) * 2.0 - 1.0
        acts = np.concatenate([dq, grip[:, None]], axis=1).astype(np.float32)
        out.extend((o, acts[i], r, o2, d)
                   for i, (o, _, r, o2, d) in enumerate(trans))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=400_000)
    ap.add_argument('--demo-dir', default='baselines/episodes_all')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/sacfd_full')
    ap.add_argument('--duplicate', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--warm-start', default=None, metavar='CKPT',
                    help='initialize actor/critic from a trained pick-SACfD .zip (same '
                         '17/7 spaces) -- staged training: explore from "can already pick"')
    ap.add_argument('--train-max-steps', type=int, default=900,
                    help='training episode horizon. Cartesian needs ~1800: the 0.11 '
                         'm/s VCAP hard-caps the arm at human teleop speed (median '
                         'demo pick frame 666, 23/67 picks past 900, median full '
                         'demo 1707) -- joint SAC could outrun the demonstrator, '
                         'cartesian cannot by construction.')
    ap.add_argument('--control', choices=['vel', 'delta'], default='vel',
                    help='cartesian control mode; must match --demo-dir')
    ap.add_argument('--cartesian', action='store_true',
                    help='CartesianFullTaskEnv (5-dim ee-velocity actions, tip '
                         'termination) + episodes_cartesian demos relabeled on the '
                         '18-dim layout. The EEF-modality RLfD leg.')
    ap.add_argument('--no-wandb', action='store_true')
    ap.add_argument('--run-name', default=None, help='wandb run name (default: out-dir stem)')
    ap.add_argument('--eval-freq', type=int, default=25_000,
                    help='video-eval subprocess cadence (0 disables)')
    ap.add_argument('--eval-max-steps', type=int, default=1200,
                    help='eval rollout horizon (#21 lever: 400 for pick-only curves)')
    ap.add_argument('--action-mode', choices=['absolute', 'delta_joint'],
                    default='absolute',
                    help='delta_joint: env actions are per-step joint-target '
                         'deltas (cap 0.025 = demo p99 speed, leash 5*cap) and '
                         'demo-buffer actions are delta-encoded to match -- the '
                         'r2dreamer fix ported to SACfD (2026-08-11). A sidecar '
                         '<out>/sacfd_final.action_mode.json records the mode '
                         'for wandb_eval --action-mode auto.')
    ap.add_argument('--gamma', type=float, default=0.998,
                    help='discount. 0.998 ~ 500-step credit window; the old '
                         'build_model default 0.98 made the demo pick terminal '
                         '(median frame 662) worth ~1.6e-6 from start states -- '
                         'a silent cause of the uniform SACfD zeros.')
    ap.add_argument('--scope', choices=['full', 'pick'], default='full',
                    help='pick: +1 and terminate on the pick (phase-1 paper core)')
    ap.add_argument('--project', default='genesis_pickaplace', help='wandb project')
    args = ap.parse_args()

    t0 = time.time()
    if args.cartesian:
        from full_env import CartesianFullTaskEnv
        env = CartesianFullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                                   control=getattr(args, 'control', 'vel'),
                                   scope=args.scope)
    else:
        env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                          scope=args.scope, action_mode=args.action_mode)
    print(f'[env] {type(env).__name__} built in {time.time() - t0:.1f}s '
          f'| pick_z={env.pick_z:.4f}', flush=True)

    if args.warm_start:
        from stable_baselines3 import SAC
        model = SAC.load(args.warm_start, env=env, device=args.device)
        model.num_timesteps = 0   # fresh step budget for the full-task phase
        print(f'[warm-start] loaded {args.warm_start}', flush=True)
    else:
        from train_sacfd import build_model
        model = build_model(env, args.seed, args.device, gamma=args.gamma)
        print(f'[cfg] gamma={args.gamma} scope={args.scope} action_mode={args.action_mode}', flush=True)

    paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')))
    assert paths, f'no npz in {args.demo_dir}'
    if args.cartesian:
        from relabel_cartesian import relabel_cartesian
        transitions, stats = relabel_cartesian(paths, env.pick_z)
    else:
        transitions, stats = relabel_full(paths, env.pick_z)
    if args.scope == 'pick':
        # demo termination must match the env's: the pick grant ENDS the episode,
        # so its transition is terminal (done=True) or the critic bootstraps
        # through a state the env never continues from
        transitions = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                       (o, a, r, o2, d) for (o, a, r, o2, d) in transitions]
    n_r = sum(1 for t in transitions if t[2] > 0)
    print(f'[demos] {len(paths)} episodes -> {len(transitions)} transitions, '
          f'{n_r} rewarded | stage grants: {stats}', flush=True)
    if args.cartesian:
        from cartesian_env import CartesianCanEnv
        _norm = CartesianCanEnv.normalize_action
    elif args.action_mode == 'delta_joint':
        # Delta encoding needs per-episode context (delta_t = (cmd_t - cmd_{t-1})
        # / cap; row 0 from the measured start pose) -- a stateless per-transition
        # action_transform cannot do it, so re-encode the transitions' actions
        # here, per episode, with the SAME math as to_dreamer_demos (commanded
        # deltas, grip absolute [-1,1]). Transitions were built per-path in order
        # so episode boundaries are recoverable from the paths loop below.
        transitions = delta_encode_transitions(paths, env.pick_z, args.scope,
                                               env.delta_cap)
        _norm = None
    else:
        _norm = pick_env.normalize_action
    n_added = demo_buffer.inject_into_replay_buffer(
        model, transitions, action_transform=_norm,
        duplicate=args.duplicate)
    print(f'[demos] x{args.duplicate} -> {n_added} added; buffer pos={model.replay_buffer.pos} '
          f'full={model.replay_buffer.full}', flush=True)

    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from wandb_utils import init_wandb, WandbScalarCallback, VideoEvalCallback
    out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    run = init_wandb(args, name=args.run_name or out.name, tags=('sacfd',),
                     project=args.project)
    cbs = [CheckpointCallback(save_freq=50_000, save_path=str(out), name_prefix='sacfd'),
           WandbScalarCallback(run)]
    if args.eval_freq:
        cbs.append(VideoEvalCallback(run, out, eval_freq=args.eval_freq,
                                     max_steps=args.eval_max_steps, seed=args.seed,
                                     cartesian=args.cartesian,
                                     control=getattr(args, 'control', 'vel',
                                     action_mode=(args.action_mode if args.action_mode != 'absolute' else None))))
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / 'sacfd_final'))
    # sidecar: lets wandb_eval --action-mode auto pick the right control path
    # (the silent-default bug family rule: control mode travels WITH the artifact)
    (out / 'sacfd_final.action_mode.json').write_text(
        json.dumps({'action_mode': args.action_mode}))
    if run is not None:
        run.finish()
    print(f'[full] done in {(time.time() - t0)/3600:.1f}h -> {out}/sacfd_final.zip', flush=True)


if __name__ == '__main__':
    main()
