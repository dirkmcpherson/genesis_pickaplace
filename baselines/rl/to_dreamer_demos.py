"""Convert episodes_all_images/*.npz -> DreamerV3 demo episodes.

Dreamer's tools.load_episodes expects per-episode npz with aligned arrays:
    image (T,H,W,C) uint8, action (T,A) float32, reward (T,), is_first/is_last/
    is_terminal (T,) bool, discount (T,), logprob (T,)

Rewards use the SAME staged scheme as SACfD (full_env.STAGE_REWARD) via
train_sacfd_full.relabel_full, so a Dreamer-vs-SACfD comparison is reward-identical.
is_terminal is True only on the nested frame (true task termination); episodes that
never nest end with is_last=True / is_terminal=False so the value head bootstraps.

Usage: to_dreamer_demos.py [--src baselines/episodes_all_images]
                           [--dst ~/workspace/dreamerv3-torch/demonstrations/genesis]
"""
import os
import argparse, glob, pathlib as pl, sys
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser()
ap.add_argument('--src', default='baselines/episodes_all_images')
ap.add_argument('--dst', default='/home/j/workspace/dreamerv3-torch/demonstrations/genesis')
ap.add_argument('--pick-only', action='store_true',
                help='keep only demos that reached >= picked (drop no-pick negatives)')
ap.add_argument('--scope', choices=['full', 'pick'], default='full',
                help='pick: truncate each demo at the pick grant (first rewarded '
                     'frame) with is_terminal there, matching the genesis_scope=pick '
                     'env that terminates +1 on pick. Leaving demos full-length would '
                     'teach the cont head that pick states continue AND show '
                     'post-pick rewards the env never pays. No-pick episodes stay '
                     'full-length (zero-reward dynamics data).')
ap.add_argument('--action-encoding', choices=['abs', 'delta_joint'], default='abs',
                help='delta_joint: arm dims = clip((cmd_t - cmd_{t-1})/cap, -1, 1) '
                     'from the COMMANDED joint targets (npz actions[:, :6]; row 0 = '
                     '(cmd_0 - q_0)/cap ~ 0), grip stays absolute (raw 0..1 -> '
                     '[-1,1]). Commanded (not measured-q) deltas: integrating them '
                     'reproduces the validated commanded-replay trajectory, whereas '
                     'measured-q deltas make the replay arm chase its own PD lag '
                     '(~40 raw frames behind by the lift -- gate-tested FAIL '
                     '2026-08-10). Matches the r2dreamer GenesisPick '
                     'action_mode=delta_joint adapter, which integrates a*cap onto a '
                     'persistent leashed joint target each SIM step. Downsample '
                     'composition is exact by construction: demo_prefill mean-pools '
                     'the window\'s per-frame actions, which telescopes to '
                     '(cmd_{t+N}-cmd_t)/(N*cap), and the adapter re-integrates it '
                     'as N sim steps of a*cap -- no converter-side downsampling.')
ap.add_argument('--grant-slack', type=int, default=0,
                help='scope=pick only: keep this many RAW frames past the recorded '
                     '(old-predicate) pick-grant frame, moving the +1/is_terminal to '
                     'the new final row. Needed for delta_joint replay: the recorded '
                     'grant frame is the OLD z+grip predicate, the hardened env '
                     'predicate (2026-08-09) adds a 10-frame sustain, and open-loop '
                     'delta replay lags the demo by ~4-8 agent steps (PD following '
                     'error on measured-q targets) -- a tape cut at the grant frame '
                     'ends before the replayed lift can cross pick_z. Slack frames '
                     'are the demo\'s own continued lift (all pick demos carry on '
                     'toward place). Benign for training: online pick episodes '
                     'TERMINATE at the grant, so the higher-lift demo states do not '
                     'contradict any online reward label. 0 = old behavior.')
ap.add_argument('--delta-cap', type=float, default=0.04,
                help='rad per SIM step per arm dim (delta_joint only). MUST equal the '
                     'adapter\'s env.delta_cap. Calibrated 2026-08-10 on '
                     'episodes_pick_pruned_img (118,194 frames), COMMANDED deltas: '
                     'global p99 0.0249, max per-dim p99 0.0307 (joint 4), 0.17%% of '
                     'values clip at 0.04 (max ever 0.059). Leash headroom: |cmd-q| '
                     'lead p99 0.126 ~ 3*0.04.')
ap.add_argument('--demo-terminal-guard', choices=['on', 'off'], default='on',
                help='on (DEFAULT, 2026-08-23, PREREG §4.3): relabel_full ends every tape '
                     'where the env would have terminated -- the tip rule (grip commanded '
                     'open & can tilt>60 after the step) cuts no-pick tapes with '
                     'is_terminal=True there (the env would have terminated, value 0), and '
                     'the scope=pick terminal sits on the pick frame itself (grant slack '
                     'is refused: a slack window ends the tape AFTER the env would have). '
                     'off = pre-08-23 output (no-pick tapes whole, is_terminal never set '
                     'on a tip; --grant-slack honoured).')
args = ap.parse_args()
GUARD = (args.demo_terminal_guard == 'on')
if GUARD and args.grant_slack:
    raise SystemExit('--grant-slack > 0 contradicts --demo-terminal-guard on (the env '
                     'terminates at the pick; slack frames are post-terminal). Pass '
                     '--demo-terminal-guard off to restore the old slack behaviour.')
if args.scope == 'pick' and args.dst.rstrip('/').endswith('/genesis'):
    args.dst = args.dst.rstrip('/') + '_pick'   # never mix scopes in one demo dir
if args.action_encoding == 'delta_joint' and 'delta' not in pl.Path(args.dst).name:
    args.dst = args.dst.rstrip('/') + '_delta'  # never mix encodings in one demo dir

import pick_env  # noqa: E402
from train_sacfd_full import relabel_full  # noqa: E402
from full_env import FullTaskEnv  # noqa: E402

PICK_Z = 0.1505   # FullTaskEnv.pick_z under the current world (avoids building a world)

DST = pl.Path(args.dst); DST.mkdir(parents=True, exist_ok=True)
# Filenames encode episode length (genesis-<uid>-<T>.npz), so any change to T orphans
# the previous generation instead of overwriting it -- the directory then holds BOTH
# and dreamer loads both (once silently inflated total demo reward 272 -> 476).
for _stale in DST.glob('genesis-*.npz'):
    _stale.unlink()
paths = sorted(glob.glob(str(REPO / args.src / '*.npz')))
assert paths, f'no npz in {args.src}'

n_out = n_rew = 0
for p in paths:
    d = np.load(p, allow_pickle=True)
    stage = str(d['stage']) if 'stage' in d.files else 'picked'  # old harvests lack it
    if args.pick_only and stage == 'no-pick':
        continue
    # per-frame staged rewards, gated by the episode's env-measured stage
    # scope='full' here on purpose: this script places the pick terminal itself
    # (below, first rewarded frame); the guard still cuts tip-terminated tapes and
    # marks done=True there (-> is_terminal).
    trans, _ = relabel_full([p], PICK_Z, scope='full', terminal_guard=GUARD)
    if not trans:
        continue
    T = len(trans)                       # transitions = frames - 1 (or up to nested/tip)
    img = d['images'][:T].astype(np.uint8)
    # NORMALIZE to the env's Box(-1,1) action convention. The stored npz actions are
    # RAW (joint radians + grip 0..1); dreamer's actor emits [-1,1]. Feeding raw actions
    # made the world model learn dynamics conditioned on a scale the policy can never
    # produce (62.6% of demo action values fell outside [-1,1]) -- present in every run
    # v6-v13. SACfD never hit this because demo_buffer applies normalize_action.
    if args.action_encoding == 'delta_joint':
        # Forward-looking delta action at t: the target the demo COMMANDED at
        # obs_t, as a delta from the previous commanded target (row 0: from the
        # measured start pose, ~0 -- max |cmd_0 - q_0| over demos is 0.012 rad).
        # Integrated through the adapter (target init = measured qpos at reset)
        # this reproduces the commanded-target trajectory, i.e. the validated
        # commanded replay. The shift below then makes row t carry the delta
        # that PRODUCED obs_t, dreamer's convention.
        # Grip stays ABSOLUTE: raw commanded 0..1 -> [-1,1] (adapter maps back).
        cmds = np.stack([t[1] for t in trans]).astype(np.float64)  # raw [6 rad, grip]
        cmd = cmds[:, :6]
        prev = np.concatenate([d['states'][:1, :6].astype(np.float64), cmd[:-1]])
        dq = np.clip((cmd - prev) / args.delta_cap, -1.0, 1.0)
        act = np.concatenate(
            [dq, (np.clip(cmds[:, 6], 0.0, 1.0) * 2.0 - 1.0)[:, None]], axis=1
        ).astype(np.float32)
    else:
        act = pick_env.normalize_action(
            np.stack([t[1] for t in trans]).astype(np.float32)).astype(np.float32)
    rew = np.array([t[2] for t in trans], dtype=np.float32)
    done = np.array([t[4] for t in trans], dtype=bool)
    is_first = np.zeros(T, dtype=bool); is_first[0] = True
    is_last = np.zeros(T, dtype=bool); is_last[-1] = True
    is_terminal = done.copy()            # nested frame, or (guard on) the env tip terminal
    discount = (1.0 - is_terminal.astype(np.float32))
    # --- SHIFT into dreamer's convention -------------------------------------
    # tools.simulate stores: index 0 = (reset obs, action=ZEROS, reward=0), and for
    # t>=1, action[t]/reward[t] are what led INTO obs[t] (backward-looking).
    # relabel_full gives forward-looking tuples (s_t, a_t, r_t) with a_t taken FROM
    # s_t. Feeding those unshifted put demo transitions one step out of phase with
    # collected ones -- the world model saw two contradictory dynamics.
    # EXTEND by one frame rather than truncate: the episode becomes length T+1 with
    # obs s_0..s_T, so the FINAL reward (the nested +4 -- our scarcest signal) is kept.
    n_avail = len(d['images'])
    img = d['images'][:min(T + 1, n_avail)].astype(np.uint8)
    m = len(img)                                    # T+1 normally
    act = np.concatenate([np.zeros_like(act[:1]), act])[:m]
    rew = np.concatenate([np.zeros_like(rew[:1]), rew])[:m]
    is_terminal = np.concatenate([[False], is_terminal])[:m]
    is_first = np.zeros(m, dtype=bool); is_first[0] = True
    is_last = np.zeros(m, dtype=bool); is_last[-1] = True
    discount = (1.0 - is_terminal.astype(np.float32))
    T = m
    if args.scope == 'pick':
        # Stage rewards are ordered grants (pick pays first), so the first rewarded
        # frame IS the pick. Truncate there and terminate, exactly like the env.
        ridx = np.flatnonzero(rew > 0)
        if len(ridx):
            k = min(int(ridx[0]) + args.grant_slack, len(img) - 1)
            img, act = img[:k + 1], act[:k + 1]
            rew = np.zeros(k + 1, dtype=np.float32)
            rew[k] = 1.0                       # scope=pick pays +1 regardless of stage value
            is_terminal = np.zeros(k + 1, dtype=bool); is_terminal[k] = True
            is_first = is_first[:k + 1]
            is_last = np.zeros(k + 1, dtype=bool); is_last[k] = True
            discount = (1.0 - is_terminal.astype(np.float32))
            T = k + 1
    uid = int(d['uid'])
    np.savez_compressed(
        DST / f'genesis-{uid:04d}-{T}.npz',
        image=img, action=act, reward=rew, discount=discount,
        is_first=is_first, is_last=is_last, is_terminal=is_terminal,
        logprob=np.zeros(T, dtype=np.float32))
    n_out += 1; n_rew += int((rew > 0).sum())
    print(f'{uid}: {stage:<8} T={T:<5} reward_frames={int((rew>0).sum())}', flush=True)
print(f'\n{n_out} demo episodes -> {DST}  ({n_rew} rewarded frames total)')
