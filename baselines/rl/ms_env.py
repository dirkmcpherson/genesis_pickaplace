"""ManiSkill PickCube-v1 POSITIVE CONTROL for our RLPD implementation.

WHY THIS FILE EXISTS
--------------------
Our RLPD arm ignites rarely on the genesis pick task (~1/3 of seeds, 5-7 picks
per 45 eval episodes; four independent waves, invariant to reward density,
demo source, action encoding -- see FABLE_HANDOFF_2026-08-13.md 19-21).  Two
explanations survive: (a) our RLPD implementation has a defect, (b) the genesis
task is simply that hard.  This file runs OUR RLPD machinery (rlpd_sac.py,
unmodified) on a PUBLISHED benchmark with published-quality demos.  If it hits
the published success band, (a) is falsified and the task difficulty becomes
the paper's claim; if it does not, we have another bug to find.

It is ADDITIVE.  Nothing here is imported by train_rlpd.py / full_env.py /
train_sacfd_full.py, and no genesis default is touched.

INTERPRETER: the mani_skill package lives in the dreamerv3-torch venv
(mani_skill 3.0.0b21, editable from ~/workspace/ManiSkill; sapien 3.0.2,
gymnasium 0.29.1, torch 2.9.1).  stable_baselines3==2.8.0 was pip-installed
into that same venv (dependency-clean: pip reported "Would install
stable_baselines3-2.8.0" and nothing else).  Run everything here with
    /home/j/workspace/dreamerv3-torch/venv/bin/python
The genesis .venv-eval is NOT touched -- it has no mani_skill and runs the live
genesis waves.

ENV CONTRACT (documented deltas from stock ManiSkill and from RLPD's Adroit)
---------------------------------------------------------------------------
* obs_mode 'state'      -> Box(42,) float32.  Layout (verified against a live
  state_dict env, not hardcoded): agent{qpos 9, qvel 9} + extra{is_grasped 1,
  tcp_pose 7, goal_pos 3, obj_pose 7, tcp_to_obj_pos 3, obj_to_goal_pos 3}.
* control_mode 'pd_ee_delta_pos' -> Box(4,) in [-1,1]: dx, dy, dz, grip.  This
  is the control mode the dv3 ManiSkill runs used and the one the shipped
  demos are recorded in, so demo actions are NATIVE (no re-encoding).
* robot_uids 'panda_wristcam' -> matches the demo files exactly.  (ManiSkill
  warns it is not in PickCube's SUPPORTED_ROBOTS list; it falls back to the
  'panda' task config.  Benign, and it is what every demo/dv3 run used.)
* reward_mode 'sparse' -> ManiSkill's own sparse reward is the success
  INDICATOR per step (BaseEnv.compute_sparse_reward).  We pay exactly that,
  with the success predicate below.  DELTA vs RLPD's Adroit: Adroit pays +1
  on every solved step and runs the episode to the horizon; we TERMINATE on
  the first solved step (so a successful episode collects exactly one +1,
  max return 1.0).  Terminating is what the dv3 ManiSkill runs did, and it is
  what our genesis pick scope does -- keeping the two arms' reward semantics
  identical is the point of a control.  Consequence: the Q-watchdog's 2.0
  threshold (= 2x max task return) is correct here unchanged.
* success predicate, --success-mode:
    'relaxed' (DEFAULT) = is_grasped AND is_obj_placed.  This is the predicate
      the dv3 ManiSkill runs used (envs/maniskill.py custom_success_check:
      grasped and ||obj_to_goal|| < goal_thresh, NO static requirement).  Using
      it keeps this control comparable to the dv3 MS arm.
    'native'  = ManiSkill's own info['success'] = is_obj_placed AND
      is_robot_static.  Published PickCube numbers use this one.
  Both are computed and logged EVERY step; only the selected one drives reward
  and termination.  A caveat that must travel with any number from this file:
  under terminate-on-relaxed the robot is mid-motion at the terminal step, so
  'native' success is mechanically rare and is a diagnostic, not the metric.
* horizon 100 decision steps (truncation, NOT termination).  dv3's MS runs used
  time_limit 100; stock ManiSkill registers PickCube at 50 and PPO baselines
  solve it there, so 100 is generous.  Motion-planning demos are 50-98 steps,
  i.e. all fit inside it.

DEMOS
-----
load_ms_demos() reads ManiSkill's shipped PickCube demo h5s.  It does NOT
replay them: the stored obs/state rows are what the very same env config
emitted when the file was recorded, and the actions are already native
pd_ee_delta_pos, so the tuples are on-MDP as they stand.  Reward/done are
recomputed from the STORED OBSERVATIONS with THE SAME predicate the env
wrapper uses (one definition, two call sites -- the import structure is the
guarantee, same discipline as full_env.pick_hold_held), and each tape is CUT
at its first solved frame.  Self-check: ManiSkill's own recorded native
success flags must be a SUBSET of our computed native flags (asserted) -- if
the 42-dim index map were wrong, that assert fires.

Independently, verify_demo_replay() steps the demo actions through THIS
wrapper from the recorded episode seed and reports how many terminate in
success.  That is the smoke gate's "success termination fires on a
motion-planned rollout", and it double-checks env/demo agreement.
"""
import argparse
import json
import pathlib as pl

import gymnasium as gym
import numpy as np

# --- fixed env identity (every caller gets the same MDP; nothing silently defaults) ---
ENV_ID = 'PickCube-v1'
ROBOT_UIDS = 'panda_wristcam'
OBS_MODE = 'state'
CONTROL_MODE = 'pd_ee_delta_pos'
REWARD_MODE = 'sparse'
SIM_BACKEND = 'physx_cpu'
HORIZON = 100
STATE_DIM = 42
ACTION_DIM = 4

DEMO_ROOT = pl.Path.home() / '.maniskill' / 'demos' / 'PickCube-v1'
DEMO_FILES = {
    # 1000 motion-planned episodes, 100% success, 50-98 steps, native
    # pd_ee_delta_pos, robot_uids panda_wristcam.  ManiSkill's own solver ->
    # "MS-quality demos" in the most literal sense.
    'motionplanning':
        DEMO_ROOT / 'motionplanning'
        / 'trajectory.state+rgb+depth.pd_ee_delta_pos.physx_cpu.h5',
    # the 10 human-teleop episodes the working dv3 ManiSkill runs trained on
    # (83-133 steps; three of them exceed the 100-step horizon).
    'teleop':
        DEMO_ROOT / 'teleop' / 'trajectory.state+rgb.pd_ee_delta_pos.physx_cpu.h5',
}

_INDEX_MAP = None


def state_index_map():
    """Slices into the 42-dim state obs, DERIVED from a live env (never hardcoded).

    Builds a throwaway obs_mode='state_dict' env and walks the same key order
    ManiSkill flattens (agent fields, then extra fields).  Cached per process.
    """
    global _INDEX_MAP
    if _INDEX_MAP is not None:
        return _INDEX_MAP
    import mani_skill  # noqa: F401  (registers the envs)
    import torch as th
    env = gym.make(ENV_ID, num_envs=1, obs_mode='state_dict',
                   control_mode=CONTROL_MODE, reward_mode=REWARD_MODE,
                   robot_uids=ROBOT_UIDS, max_episode_steps=HORIZON,
                   render_mode=None, sim_backend=SIM_BACKEND)
    obs, _ = env.reset(seed=0)
    idx, off = {}, 0
    for group in ('agent', 'extra'):
        for k, v in obs[group].items():
            v = v if isinstance(v, th.Tensor) else th.as_tensor(np.asarray(v))
            n = 1 if v.ndim <= 1 else int(np.prod(v.shape[1:]))
            idx[k] = slice(off, off + n)
            off += n
    env.close()
    assert off == STATE_DIM, f'state layout is {off}-dim, expected {STATE_DIM}: {idx}'
    for need in ('is_grasped', 'obj_to_goal_pos', 'qvel'):
        assert need in idx, (need, idx)
    _INDEX_MAP = idx
    return idx


def solved_from_state(state, goal_thresh=0.025, qvel_thresh=0.2, mode='relaxed'):
    """THE success predicate, evaluated on the 42-dim state obs.

    Used by the demo relabeler.  MSPickCubeEnv uses the env's own info dict for
    the identical quantities (is_grasped / is_obj_placed / is_robot_static), so
    the two agree by construction; test_predicate_agreement() checks it.

    state: (..., 42).  Returns a bool array of the leading shape.
    """
    idx = state_index_map()
    s = np.asarray(state, dtype=np.float32)
    grasped = s[..., idx['is_grasped']].reshape(s.shape[:-1]) > 0.5
    placed = (np.linalg.norm(s[..., idx['obj_to_goal_pos']], axis=-1) <= goal_thresh)
    if mode == 'relaxed':
        return grasped & placed
    if mode == 'native':
        # Panda.is_static: max|qvel[..., :-2]| <= threshold -- the two FINGER joints
        # are excluded (they are still moving while the grasp holds).  Using all 9
        # here made the recorded-success self-check fail, which is exactly what that
        # assert is for.
        static = np.max(np.abs(s[..., idx['qvel']][..., :-2]), axis=-1) <= qvel_thresh
        return placed & static
    raise ValueError(f'unknown success mode {mode!r}')


class MSPickCubeEnv(gym.Env):
    """Gymnasium single env: Box(42,) obs, Box(4,) action, sparse +1 on the
    selected success predicate, terminate on success, truncate at HORIZON.

    A plain gym.Env rather than a gym.Wrapper subclass on purpose -- the reward
    and termination logic then reads top-to-bottom in one place.  gym.Env (not a
    bare object) because SB3's Monitor reaches for .spec / .render_mode.
    """

    metadata = {'render_modes': []}
    render_mode = None
    spec = None

    def __init__(self, success_mode='relaxed', horizon=HORIZON, seed=None,
                 reward_mode='sparse'):
        # reward_mode (ADDITIVE param, 2026-08-16 dense-pair discriminator):
        #   'sparse' (DEFAULT) = the original behavior, byte-identical: +1 on
        #     the selected success predicate, 0 otherwise (regression-gated
        #     against the pre-edit file in paper/ms_dense_pair_2026-08-16.md).
        #   'normalized_dense' = ManiSkill's own shaped reward passed through
        #     unmodified.  Termination/truncation/success logic UNCHANGED in
        #     both modes (still terminate on the selected predicate).  This is
        #     the reward the March dv3 MS positive trained on.
        import mani_skill  # noqa: F401
        from mani_skill.utils.wrappers import CPUGymWrapper
        assert success_mode in ('relaxed', 'native'), success_mode
        assert reward_mode in ('sparse', 'normalized_dense'), reward_mode
        self.success_mode = success_mode
        self.reward_mode = reward_mode
        self.horizon = int(horizon)
        base = gym.make(ENV_ID, num_envs=1, obs_mode=OBS_MODE,
                        control_mode=CONTROL_MODE, reward_mode=reward_mode,
                        robot_uids=ROBOT_UIDS, max_episode_steps=self.horizon,
                        render_mode=None, sim_backend=SIM_BACKEND)
        # CPUGymWrapper unbatches the (1, ...) torch tensors to numpy scalars/arrays.
        # ignore_terminations=True: WE decide termination from the selected
        # predicate, so ManiSkill's own (native-only) terminated flag must not
        # pre-empt the relaxed one.  Truncation still comes from the TimeLimit.
        self._env = CPUGymWrapper(base, ignore_terminations=True, record_metrics=False)
        self.observation_space = self._env.observation_space
        self.action_space = self._env.action_space
        assert self.observation_space.shape == (STATE_DIM,), self.observation_space
        assert self.action_space.shape == (ACTION_DIM,), self.action_space
        self._seed = seed
        self._t = 0

    # -- gymnasium API -----------------------------------------------------
    def reset(self, *, seed=None, options=None):
        if seed is None and self._seed is not None:
            seed, self._seed = self._seed, None      # honour the first seeding only
        obs, info = self._env.reset(seed=seed, options=options)
        self._t = 0
        return np.asarray(obs, dtype=np.float32), dict(info)

    def step(self, action):
        obs, _msr, _term, trunc, info = self._env.step(np.asarray(action, np.float32))
        self._t += 1
        relaxed = bool(info['is_grasped']) and bool(info['is_obj_placed'])
        native = bool(info['success'])          # is_obj_placed AND is_robot_static
        solved = relaxed if self.success_mode == 'relaxed' else native
        if self.reward_mode == 'sparse':
            reward = 1.0 if solved else 0.0     # ManiSkill's sparse = success indicator
        else:
            reward = float(_msr)                # normalized_dense: MS's own shaping
        terminated = solved
        truncated = bool(trunc) or self._t >= self.horizon
        if terminated:
            truncated = False                   # terminal wins; no double-flagging
        info = dict(info)
        info.update(success=solved, success_relaxed=relaxed, success_native=native,
                    is_grasped=bool(info['is_grasped']),
                    is_obj_placed=bool(info['is_obj_placed']))
        return np.asarray(obs, dtype=np.float32), reward, terminated, truncated, info

    def close(self):
        self._env.close()

    @property
    def unwrapped(self):
        return self._env.unwrapped


# ---------------------------------------------------------------------------
# demos
# ---------------------------------------------------------------------------
def load_ms_demos(source='motionplanning', n_episodes=50, success_mode='relaxed',
                  path=None, verbose=True):
    """-> (transitions, census).

    transitions: list of (obs, action, reward, next_obs, done) with EXACTLY the
    reward/termination semantics of MSPickCubeEnv -- fed straight to
    rlpd_sac.DemoData(transitions, action_transform=None, ...) because the
    actions are already native and normalized to [-1, 1].
    Each tape is cut at its first solved frame (that transition carries r=1,
    done=True); nothing after it is kept.
    """
    import h5py
    path = pl.Path(path) if path is not None else DEMO_FILES[source]
    assert path.exists(), (
        f'demo file missing: {path}\nRegenerate with ManiSkill\'s own tooling, e.g.\n'
        f'  python -m mani_skill.trajectory.replay_trajectory --traj-path '
        f'{path.parent}/trajectory.h5 -c {CONTROL_MODE} -o state --save-traj '
        f'--use-first-env-state --record-rewards -n 16')
    meta = json.loads((path.parent / (path.name[:-3] + '.json')).read_text())
    env_kwargs = meta['env_info']['env_kwargs']
    # provenance asserts: a demo file from a different control mode / robot would
    # silently produce garbage actions (the silent-default bug family).
    assert meta['env_info']['env_id'] == ENV_ID, meta['env_info']['env_id']
    assert env_kwargs['control_mode'] == CONTROL_MODE, env_kwargs['control_mode']
    assert env_kwargs['robot_uids'] == ROBOT_UIDS, env_kwargs['robot_uids']
    assert env_kwargs.get('use_single_goal') in (False, None), env_kwargs

    transitions, lens, cut_lens, n_native_at_cut = [], [], [], 0
    n_unsolved, seeds = 0, []
    with h5py.File(path, 'r') as f:
        keys = sorted(f.keys(), key=lambda k: int(k.split('_')[1]))[:n_episodes]
        for k in keys:
            g = f[k]
            obs = np.asarray(g['obs/state'], dtype=np.float32)     # (T+1, 42)
            act = np.asarray(g['actions'], dtype=np.float32)       # (T, 4)
            rec_success = np.asarray(g['success']).astype(bool)    # (T,) ManiSkill native
            assert obs.shape[0] == act.shape[0] + 1, (obs.shape, act.shape)
            assert obs.shape[1] == STATE_DIM and act.shape[1] == ACTION_DIM
            lens.append(act.shape[0])

            nxt = obs[1:]                                          # post-step states
            native = solved_from_state(nxt, mode='native')
            # index-map self-check: whatever ManiSkill recorded as success must be
            # recoverable from the stored state with our index map.
            assert np.all(native[rec_success]), (
                f'{k}: recorded native success not reproduced from obs -- the 42-dim '
                f'index map is wrong')
            solved = (solved_from_state(nxt, mode=success_mode) if success_mode != 'native'
                      else native)
            hits = np.flatnonzero(solved)
            if hits.size == 0:
                n_unsolved += 1
                continue
            c = int(hits[0])                                       # first solved step
            n_native_at_cut += int(native[c])
            cut_lens.append(c + 1)
            for t in range(c + 1):
                done = bool(t == c)
                transitions.append((obs[t], act[t], 1.0 if done else 0.0,
                                    obs[t + 1], done))
    for e in meta['episodes'][:n_episodes]:
        seeds.append(e.get('episode_seed'))

    n_eps = len(cut_lens)
    n_rew = sum(1 for t in transitions if t[2] > 0)
    census = dict(
        source=source, path=str(path), success_mode=success_mode,
        n_episodes_requested=int(n_episodes), n_episodes_used=n_eps,
        n_episodes_unsolved=n_unsolved, n_transitions=len(transitions),
        n_rewarded=n_rew,
        density_pct=100.0 * n_rew / max(1, len(transitions)),
        raw_len_min=int(min(lens)) if lens else 0,
        raw_len_median=float(np.median(lens)) if lens else 0.0,
        raw_len_max=int(max(lens)) if lens else 0,
        cut_len_min=int(min(cut_lens)) if cut_lens else 0,
        cut_len_median=float(np.median(cut_lens)) if cut_lens else 0.0,
        cut_len_max=int(max(cut_lens)) if cut_lens else 0,
        n_over_horizon=int(sum(1 for c in cut_lens if c > HORIZON)),
        native_success_at_cut=n_native_at_cut,
        seeds=seeds[:n_eps],
    )
    if verbose:
        print_demo_census(census)
    return transitions, census


def print_demo_census(c):
    print('[demo census] ---------------------------------------------------------')
    print(f"  source           : {c['source']}  ({c['path']})")
    print(f"  success predicate: {c['success_mode']}  (reward = +1 on it, then done)")
    print(f"  episodes         : {c['n_episodes_used']} used "
          f"/ {c['n_episodes_requested']} requested "
          f"({c['n_episodes_unsolved']} never solved -> dropped)")
    print(f"  raw lengths      : min {c['raw_len_min']} / med "
          f"{c['raw_len_median']:.0f} / max {c['raw_len_max']} steps")
    print(f"  cut-at-success   : min {c['cut_len_min']} / med "
          f"{c['cut_len_median']:.0f} / max {c['cut_len_max']} steps "
          f"({c['n_over_horizon']} exceed the {HORIZON}-step env horizon)")
    print(f"  transitions      : {c['n_transitions']}")
    print(f"  rewarded frames  : {c['n_rewarded']}  "
          f"=> DENSITY {c['density_pct']:.3f}%   "
          f"(genesis terminal-only: 0.079%; genesis hold-reward: 1.96%)")
    exp = 128.0 * c['n_rewarded'] / max(1, c['n_transitions'])
    print(f"  expected rewarded demo frames per 128-sample demo half-batch: {exp:.2f}")
    print(f"  ManiSkill-native success also true at the cut frame: "
          f"{c['native_success_at_cut']}/{c['n_episodes_used']}")
    print('-----------------------------------------------------------------------')


def verify_demo_replay(source='motionplanning', n_episodes=5,
                       success_mode='relaxed', path=None):
    """GATE: step the recorded demo actions through THIS wrapper, from each
    episode's recorded reset seed, and report how many terminate in success.

    This is an on-policy-of-the-demonstrator rollout, so it exercises the real
    env reward + termination path (the 'success termination fires on a
    motion-planned rollout' gate).  Exact frame-for-frame reproduction is NOT
    expected in general (ManiSkill's own replay tool re-plans for a reason), so
    the gate is on SUCCESS, not on trajectory identity.
    """
    import h5py
    path = pl.Path(path) if path is not None else DEMO_FILES[source]
    meta = json.loads((path.parent / (path.name[:-3] + '.json')).read_text())
    env = MSPickCubeEnv(success_mode=success_mode, horizon=1000)  # long: no truncation
    out = []
    with h5py.File(path, 'r') as f:
        keys = sorted(f.keys(), key=lambda k: int(k.split('_')[1]))[:n_episodes]
        for k, ep in zip(keys, meta['episodes'][:n_episodes]):
            act = np.asarray(f[k]['actions'], dtype=np.float32)
            env.reset(seed=int(ep['episode_seed']))
            hit, r_sum = None, 0.0
            for t in range(act.shape[0]):
                _o, r, term, trunc, _i = env.step(act[t])
                r_sum += r
                if term:
                    hit = t + 1
                    break
                if trunc:
                    break
            out.append(dict(traj=k, seed=int(ep['episode_seed']), n_actions=act.shape[0],
                            solved_at=hit, reward_sum=r_sum))
    env.close()
    n_ok = sum(1 for o in out if o['solved_at'] is not None)
    print(f'[replay gate] {n_ok}/{len(out)} demo action-tapes terminate in '
          f'{success_mode} success when stepped through MSPickCubeEnv')
    for o in out:
        print(f"    {o['traj']:>8} seed={o['seed']:<4} actions={o['n_actions']:<4} "
              f"solved_at={o['solved_at']}  reward_sum={o['reward_sum']}")
    return n_ok, out


# ---------------------------------------------------------------------------
# smoke gate
# ---------------------------------------------------------------------------
def smoke(n_random=500, source='motionplanning', n_episodes=50,
          success_mode='relaxed', n_replay=5):
    import time
    print('=== GATE (a): env wrapper ===')
    env = MSPickCubeEnv(success_mode=success_mode)
    print(f'  obs_space    {env.observation_space}')
    print(f'  action_space {env.action_space}')
    assert env.observation_space.shape == (STATE_DIM,)
    assert env.action_space.shape == (ACTION_DIM,)
    assert np.allclose(env.action_space.low, -1) and np.allclose(env.action_space.high, 1)
    o, _ = env.reset(seed=0)
    assert o.shape == (STATE_DIM,) and o.dtype == np.float32 and np.isfinite(o).all()

    # consecutive un-seeded resets must give a NEW cube/goal (IC diversity check)
    idx = state_index_map()
    starts = []
    for _ in range(5):
        s, _ = env.reset()
        starts.append(np.concatenate([s[idx['obj_pose']], s[idx['goal_pos']]]))
    spread = float(np.std(np.stack(starts), axis=0).max())
    print(f'  IC diversity : max std over 5 resets of (obj_pose, goal_pos) = {spread:.4f}')
    assert spread > 1e-3, 'resets are not re-randomizing the cube/goal'

    rng = np.random.default_rng(0)
    rews, terms, truncs, eplen, t0 = [], 0, 0, 0, time.time()
    env.reset(seed=0)
    for i in range(n_random):
        a = rng.uniform(-1, 1, size=ACTION_DIM).astype(np.float32)
        o, r, term, trunc, info = env.step(a)
        assert o.shape == (STATE_DIM,) and np.isfinite(o).all(), i
        assert np.isfinite(r) and r >= 0.0, (i, r)
        rews.append(r)
        eplen += 1
        if term or trunc:
            terms += int(term); truncs += int(trunc)
            assert eplen <= HORIZON, eplen
            eplen = 0
            env.reset()
    dt = time.time() - t0
    print(f'  {n_random} random steps in {dt:.1f}s = {n_random/dt:.0f} steps/s')
    print(f'  reward min={min(rews)} max={max(rews)} sum={sum(rews)}  (all >= 0: OK)')
    print(f'  terminations={terms} truncations={truncs} '
          f'(random policy should never solve PickCube: terminations expected 0)')
    env.close()

    print('\n=== GATE (a2): success termination on demo action tapes ===')
    n_ok, _ = verify_demo_replay(source=source, n_episodes=n_replay,
                                 success_mode=success_mode)
    assert n_ok > 0, 'no demo tape terminated in success -- env/demo mismatch'

    print('\n=== GATE (b): demo census ===')
    transitions, census = load_ms_demos(source=source, n_episodes=n_episodes,
                                        success_mode=success_mode)
    assert census['density_pct'] > 0.5, (
        f"demo reward density {census['density_pct']:.3f}% is not percent-level")
    o, a, r, o2, d = transitions[0]
    assert o.shape == (STATE_DIM,) and a.shape == (ACTION_DIM,) and o2.shape == (STATE_DIM,)
    assert abs(a).max() <= 1.0 + 1e-6, 'demo actions outside the [-1,1] action space'
    assert sum(1 for t in transitions if t[4]) == census['n_episodes_used']
    print('  all gate asserts passed')
    return census


def test_predicate_agreement(n=200, seed=0):
    """The demo relabeler reads the predicate off the 42-dim state; the env reads
    it off ManiSkill's info dict.  Assert they never disagree over a rollout."""
    env = MSPickCubeEnv(success_mode='relaxed')
    rng = np.random.default_rng(seed)
    o, _ = env.reset(seed=seed)
    dis_r = dis_n = 0
    for _ in range(n):
        o, r, term, trunc, info = env.step(rng.uniform(-1, 1, ACTION_DIM).astype(np.float32))
        dis_r += int(bool(solved_from_state(o, mode='relaxed')) != info['success_relaxed'])
        dis_n += int(bool(solved_from_state(o, mode='native')) != info['success_native'])
        if term or trunc:
            o, _ = env.reset()
    env.close()
    print(f'[predicate agreement] over {n} steps: relaxed disagreements={dis_r} '
          f'native disagreements={dis_n}')
    assert dis_r == 0, 'state-derived relaxed predicate disagrees with the env info dict'
    return dis_r, dis_n


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--random-steps', type=int, default=500)
    ap.add_argument('--source', choices=sorted(DEMO_FILES), default='motionplanning')
    ap.add_argument('--n-episodes', type=int, default=50)
    ap.add_argument('--success-mode', choices=['relaxed', 'native'], default='relaxed')
    ap.add_argument('--n-replay', type=int, default=5)
    ap.add_argument('--predicate-test', action='store_true')
    a = ap.parse_args()
    smoke(a.random_steps, a.source, a.n_episodes, a.success_mode, a.n_replay)
    if a.predicate_test:
        print()
        test_predicate_agreement()
