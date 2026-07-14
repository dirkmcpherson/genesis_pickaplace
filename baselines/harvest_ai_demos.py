"""Harvest AI demonstrations from a trained teacher (STEP 2 of the human-vs-AI study).

Teacher-agnostic: roll a trained policy out over RANDOMIZED ICs sampled from the SAME
support as eval (ic_sampling.sample_support_ics), keep only trajectories that reach the
success predicate, and serialize the kept ones to integer-named npz that BOTH downstream
consumers already read:
  * convert_to_lerobot.py  -> LeRobotDataset for DP / (same npz) for BC
  * demo_buffer.py         -> SACfD replay-buffer seeding
(they read only states/actions/n, so one file feeds all three students.)

Anti-manufacturing guardrails (honesty protocol):
  * --verify: independently REPLAY each kept trajectory open-loop from a fresh env reset
    at its own IC and re-confirm the success predicate before serializing. Because the
    sim is deterministic, this reproduces success iff the recorded (state_i, action_i)
    pairs are correctly aligned -- it is the structural guard against the state/action
    off-by-one that inflated an earlier dataset.
  * --teacher-type random is the NEGATIVE-CONTROL teacher: uniform actions. It MUST yield
    ~0 kept. If it doesn't, the pick/contact predicate is broken, not the teacher.

Actions recorded are PHYSICAL ([6 joint rad, grip 0..1]) exactly like
collect_lerobot_dataset.py / collect_all_rl.py, so no rescaling is needed anywhere.

Usage (.venv-eval, CPU):
  python baselines/harvest_ai_demos.py --teacher-type sac \
      --checkpoint baselines/rl/checkpoints/sacfd_50000_steps.zip \
      --n 200 --scope pick --verify --outdir baselines/episodes_ai/sac_pick
"""
import argparse
import json
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

import ic_sampling  # noqa: E402
from genesis_can_env import GenesisCanEnv  # noqa: E402
from pick_env import denormalize_action, ACT_DIM  # noqa: E402

PICK_TAIL = 10        # frames to keep after the pick lands (captures the lift)
CONTACT_TAIL = 15     # frames to keep after first slide-contact
PICK_CAP = 300        # per-rollout step cap for pick scope (matches PickOnlyEnv)
NO_PICK_ABORT = 700   # full scope: abort a rollout with no pick by here (no contact possible)

# integer filename base so convert_to_lerobot's int(p.stem) sort works
STEM_BASE = 100000


def load_teacher(teacher_type, checkpoint, seed):
    """-> (policy_action(obs)->physical 7-vec, policy_reset()).

    dp    : lerobot Diffusion Policy (dp_runner). policy_reset clears its action queue.
    sac   : SB3 SAC greedy, denormalized to physical.
    random: uniform physical action -- the negative-control teacher.
    """
    if teacher_type == 'dp':
        from dp_runner import load_dp_runner
        policy_action, policy_reset, proprio = load_dp_runner(checkpoint)
        print(f'[teacher] DP loaded, PROPRIO={proprio}', flush=True)
        return policy_action, policy_reset
    if teacher_type == 'sac':
        from stable_baselines3 import SAC
        model = SAC.load(checkpoint)
        print('[teacher] SAC loaded', flush=True)

        def policy_action(obs):
            a_norm, _ = model.predict(obs['state'], deterministic=True)
            return denormalize_action(a_norm)
        return policy_action, (lambda: None)
    if teacher_type == 'random':
        rng = np.random.default_rng(seed)

        def policy_action(obs):
            return denormalize_action(rng.uniform(-1.0, 1.0, ACT_DIM))
        return policy_action, (lambda: None)
    raise ValueError(f'unknown teacher-type {teacher_type}')


def rollout(env, policy_action, policy_reset, ic, scope):
    """Closed-loop rollout from `ic`. Returns (states, actions, kept:bool).

    Records aligned (state_i, action_i): state_i is the obs the teacher saw, action_i the
    physical action stepped from it. Truncates shortly after the success event.
    """
    obs = env.reset(**ic)
    policy_reset()
    states, actions = [], []
    picked_at = contact_at = -1
    cap = PICK_CAP if scope == 'pick' else env.max_steps
    for i in range(cap):
        a = np.asarray(policy_action(obs), dtype=np.float32)
        states.append(obs['state'])
        actions.append(a)
        obs, done, info = env.step(a)
        if picked_at < 0 and info['picked']:
            picked_at = i
        if scope == 'pick':
            if picked_at >= 0 and i >= picked_at + PICK_TAIL:
                return np.array(states), np.array(actions), True
        else:  # full
            if contact_at < 0 and info['contact']:
                contact_at = i
            if contact_at >= 0 and i >= contact_at + CONTACT_TAIL:
                return np.array(states), np.array(actions), True
            if picked_at < 0 and i >= NO_PICK_ABORT:
                break  # no pick this late -> no contact possible, abort
        if done:
            break
    return np.array(states), np.array(actions), False


def verify(env, ic, actions, scope):
    """Independent open-loop replay from a fresh reset at `ic`; re-confirm the predicate.
    Deterministic sim -> reproduces success iff the recording is aligned."""
    env.reset(**ic)
    picked = contact = False
    info = {}
    for a in actions:
        _obs, _done, info = env.step(np.asarray(a, dtype=np.float64))
        picked = picked or info['picked']
        contact = contact or info['contact']
    return picked if scope == 'pick' else contact


def ic_coverage(kept_ics, env, bins=4):
    """Per-bin histogram of kept can-xy over the demo-support box (visibility, not a gate)."""
    if not kept_ics:
        return {}
    lo, hi = ic_sampling.support_box(env)
    xy = np.array([ic['can_pos'][:2] for ic in kept_ics])
    ix = np.clip(((xy - lo) / (hi - lo + 1e-9) * bins).astype(int), 0, bins - 1)
    grid = np.zeros((bins, bins), int)
    for a, b in ix:
        grid[a, b] += 1
    return {'bins': bins, 'grid': grid.tolist(),
            'occupied_cells': int((grid > 0).sum()), 'total_cells': bins * bins}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--teacher-type', required=True, choices=['dp', 'sac', 'random'])
    ap.add_argument('--checkpoint', default=None)
    ap.add_argument('--n', type=int, default=200, help='number of ICs to roll out')
    ap.add_argument('--scope', choices=['pick', 'full'], default='pick')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--outdir', required=True)
    ap.add_argument('--verify', action='store_true',
                    help='independently replay each kept trajectory before serializing')
    args = ap.parse_args()
    if args.teacher_type != 'random' and not args.checkpoint:
        ap.error('--checkpoint required for dp/sac teachers')

    outdir = REPO / args.outdir if not pl.Path(args.outdir).is_absolute() else pl.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    env = GenesisCanEnv(backend='cpu')
    policy_action, policy_reset = load_teacher(args.teacher_type, args.checkpoint, args.seed)
    episodes = ic_sampling.sample_support_ics(env, args.n, seed=args.seed)
    print(f'[harvest] teacher={args.teacher_type} scope={args.scope} n={args.n} '
          f'verify={args.verify} -> {outdir}', flush=True)

    kept = kept_ics = 0
    kept_ic_list = []
    n_reject_verify = 0
    for idx, ic in enumerate(episodes):
        states, actions, ok = rollout(env, policy_action, policy_reset, ic, args.scope)
        if not ok:
            print(f'  ic{idx}: no {args.scope}-success ({len(states)} steps)', flush=True)
            continue
        if args.verify and not verify(env, ic, actions, args.scope):
            n_reject_verify += 1
            print(f'  ic{idx}: KEPT-by-rollout but FAILED verify -> rejected '
                  f'(recording misalignment?)', flush=True)
            continue
        stem = STEM_BASE + kept
        np.savez_compressed(outdir / f'{stem}.npz', states=states, actions=actions,
                            n=len(states), uid=stem)
        kept += 1
        kept_ic_list.append(ic)
        print(f'  ic{idx}: KEPT ({len(states)} frames) -> {stem}.npz', flush=True)

    yield_frac = kept / max(args.n, 1)
    cov = ic_coverage(kept_ic_list, env)
    manifest = dict(teacher_type=args.teacher_type, checkpoint=args.checkpoint,
                    scope=args.scope, seed=args.seed, n_rollouts=args.n, kept=kept,
                    yield_frac=round(yield_frac, 4), rejected_by_verify=n_reject_verify,
                    ic_coverage=cov)
    (outdir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(f'\n[harvest] kept {kept}/{args.n} (yield {yield_frac:.2%}), '
          f'{n_reject_verify} rejected by verify | coverage '
          f"{cov.get('occupied_cells','?')}/{cov.get('total_cells','?')} cells -> {outdir}",
          flush=True)
    if args.teacher_type == 'random' and kept > max(1, int(0.05 * args.n)):
        print('[harvest] WARNING: random teacher kept >5% -- success predicate may be '
              'too loose (negative control failing).', flush=True)


if __name__ == '__main__':
    main()
