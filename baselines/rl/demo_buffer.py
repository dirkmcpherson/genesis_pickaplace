"""Demo loading + replay-buffer injection for SACfD on the PICK subtask.

Demos are sim-verified replays in baselines/episodes_raw_v4/*.npz with keys
states (n,17), actions (n,7), uid, n. v4 may still be collecting, so paths are
globbed at RUNTIME; if fewer than 10 v4 files exist we fall back to
baselines/episodes_raw_v3/*.npz, whose states are 16-dim (no grip_effort) --
a zero grip_effort column is padded in at index 7 to produce the 17-dim layout
[6 joints, gripper, grip_effort, can xyz, can quat, goal xy].

Reward relabeling for PICK (mirrors GenesisCanEnv's picked test):
    picked at frame j  <=>  can_z (state idx 10) > pick_z (0.1505)
                            AND gripper (state idx 6) > 0.3
    NOTE: the task spec said "can z at state index 9" -- that is the 16-dim v3
    layout; in the 17-dim layout can xyz sits at indices 8,9,10, so can z = 10.
    (The env's own check uses the *commanded* gripper; the state motor value
    tracks the command closely under position control, so state idx 6 is the
    faithful offline analog.)
The transition ARRIVING at frame j (i = j-1) gets r=1, done=True, and the demo
is truncated there -- the pick subtask ends at the pick. Demos that never
satisfy the condition contribute all their transitions with r=0/done=False
(pure state coverage; done=False so the critic still bootstraps).
"""
import glob
import pathlib as pl

import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
RL_DIR = REPO / 'baselines/episodes_raw_rl'   # ALL demos incl failures (broad coverage)
V4_DIR = REPO / 'baselines/episodes_raw_v4'
V3_DIR = REPO / 'baselines/episodes_raw_v3'

PICK_Z = 0.1505          # table 0.05 + can_h/2 0.0505 + 0.05 (replay_harness pick_z)
GRIP_CLOSED_FRAC = 0.3   # GP_CLOSE=30 on the 0..1 gripper scale
STATE_DIM = 17


def find_demo_paths(min_v4=10):
    """Glob demo dirs at runtime. Prefer episodes_raw_rl (ALL demos incl failures ->
    broad state coverage for off-policy RL; sparse-reward SAC learns from r=0
    transitions too). Fall back to v4 (sim-verified successes), then v3."""
    rl = sorted(glob.glob(str(RL_DIR / '*.npz')))
    if len(rl) >= min_v4:
        print(f'[demo_buffer] using {len(rl)} RL episodes (all demos incl failures)')
        return rl
    v4 = sorted(glob.glob(str(V4_DIR / '*.npz')))
    if len(v4) >= min_v4:
        return v4
    v3 = sorted(glob.glob(str(V3_DIR / '*.npz')))
    print(f'[demo_buffer] only {len(v4)} v4 episodes (<{min_v4}); '
          f'falling back to {len(v3)} v3 episodes (16-dim states, padding grip_effort)')
    return v3


def _to_17(states):
    """Pad 16-dim v3 states with a zero grip_effort column at index 7."""
    if states.shape[1] == STATE_DIM:
        return states
    assert states.shape[1] == 16, f'unexpected state dim {states.shape[1]}'
    return np.insert(states, 7, 0.0, axis=1)


def load_demo_transitions(paths=None, pick_z=PICK_Z, grip_closed=GRIP_CLOSED_FRAC):
    """-> list of (obs, action, reward, next_obs, done); actions are PHYSICAL
    (rad / 0..1) -- normalize via an action_transform at injection time."""
    if paths is None:
        paths = find_demo_paths()
    transitions = []
    n_picked_eps = 0
    for p in paths:
        d = np.load(p, allow_pickle=True)
        s = _to_17(np.asarray(d['states'], dtype=np.float32))
        a = np.asarray(d['actions'], dtype=np.float32)
        n = len(s)
        if n < 2:
            continue
        picked = (s[:, 10] > pick_z) & (s[:, 6] > grip_closed)
        j = int(np.argmax(picked)) if picked.any() else -1
        if j == 0:
            print(f'[demo_buffer] {p}: picked at frame 0?! skipping episode')
            continue
        last = j if j > 0 else n - 1        # transitions cover frames 0..last
        if j > 0:
            n_picked_eps += 1
        for i in range(last):
            done = (j > 0) and (i == j - 1)
            transitions.append((s[i], a[i], 1.0 if done else 0.0, s[i + 1], done))
    n_r1 = sum(1 for t in transitions if t[2] > 0)
    print(f'[demo_buffer] {len(paths)} episodes -> {len(transitions)} transitions, '
          f'{n_r1} with r=1 ({n_picked_eps} episodes reach pick)')
    return transitions


def inject_into_replay_buffer(model, transitions, action_transform=None, duplicate=1):
    """Add demo transitions to an SB3 off-policy model's replay buffer.

    SB3 ReplayBuffer.add signature is (obs, next_obs, action, reward, done,
    infos), each batched over n_envs -- with n_envs=1 that means a leading
    (1, ...) axis and infos as a 1-list of dicts. infos=[{}] deliberately
    carries no 'TimeLimit.truncated' key: demo done=True is a TRUE terminal
    (the pick), so no timeout bootstrapping should be applied.

    action_transform maps stored physical actions into the env's action space
    (pass pick_env.normalize_action for PickOnlyEnv's [-1,1] convention --
    REQUIRED, since SAC's critic/actor operate in that space).

    duplicate>1 re-adds each demo transition that many times: a cheap SACfD-ish
    oversampling of demos early in training (they get overwritten/diluted as
    fresh experience streams in; no prioritized replay needed).
    """
    buf = model.replay_buffer
    assert buf.n_envs == 1, 'demo injection assumes n_envs=1'
    n_added = 0
    for _ in range(max(1, int(duplicate))):
        for obs, act, rew, next_obs, done in transitions:
            a = action_transform(act) if action_transform is not None else act
            buf.add(
                np.asarray(obs, dtype=np.float32)[None],
                np.asarray(next_obs, dtype=np.float32)[None],
                np.asarray(a, dtype=np.float32)[None],
                np.array([rew], dtype=np.float32),
                np.array([done], dtype=bool),
                [{}],
            )
            n_added += 1
    return n_added
