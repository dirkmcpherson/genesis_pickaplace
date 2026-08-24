#!/usr/bin/env python
"""Contract-v1 demo dir -> dreamer-format demo dir, with NO re-encoding.

WHY (paper/PREREG_final_round_robin_2026-08-23.md §4.1): the final round robin records
every demo set through the learners' own MDP at the decision clock (action_repeat 4),
so one tape row IS one dreamer transition. The stride-1 converters
(baselines/rl/to_dreamer_demos.py, dreamerv3-torch/convert_genesis_demos_repeat.py)
re-window 30 Hz tapes and place a relabelled terminal with a grant slack; for contract-v1
tapes none of that applies: the action is `actions_delta` as executed, the reward is the
env's own sparse reward, the terminal is the env's `terminated` row. This script only
changes LAYOUT (forward-looking tape -> dreamer's backward-looking episode) and scales
the terminal reward (x --terminal-reward, MS parity 100).

Output mirrors convert_genesis_demos_repeat.py's writer byte-for-byte in key names and
conventions:
    image        (T,H,W,C) uint8   obs BEFORE each decision + the final obs  (T = n+1)
    action       (T,7)   float32   backward-shifted: action[t] led INTO image[t]; action[0] = 0
    reward       (T,)    float32   reward[t] = tape rewards[t-1] * terminal_reward; reward[0] = 0
    discount     (T,)    float32   1 - is_terminal
    is_first     (T,)    bool      [0] only
    is_last      (T,)    bool      [-1] only
    is_terminal  (T,)    bool      [-1] = tape terminated[-1] (pick OR tip); a cap-truncated
                                   tape ends is_terminal=False so the value head BOOTSTRAPS
    logprob      (T,)    float32   zeros
filename genesis-{uid:06d}-{T}.npz (uids are rollout indices >= 100000), plus
repeat.json {action_repeat, contract 'v1', delta_cap, scope, terminal_reward, src, src_sha,
census} -- dreamer.py asserts config.action_repeat == the stamp (unchanged mechanism).

Fails (label != success): reward all 0; is_terminal[-1] = True iff the env terminated
the rollout (tipped); False at the cap. No grant slack anywhere (PREREG §4.3).

Usage (numpy only; any python):
  python baselines/rl/to_dreamer_native.py --src baselines/matched_sets/dH/episodes \
      --dst ~/workspace/dreamerv3-torch/demonstrations/genesis_final_dH_r4 --repeat 4
"""
import argparse, datetime, glob, hashlib, json, os, sys
import numpy as np


def tape_sha(files):
    h = hashlib.sha256()
    for f in files:
        h.update(os.path.basename(f).encode())
        with open(f, 'rb') as fh:
            h.update(fh.read())
    return h.hexdigest()


def convert_one(z, terminal_reward):
    """Contract-v1 npz (np.load'ed) -> dict of dreamer arrays, or raise ValueError."""
    n = int(z['n']) if 'n' in z.files else int(len(z['actions_delta']))
    img = np.asarray(z['images'])
    act = np.asarray(z['actions_delta'], np.float32)
    rew = np.asarray(z['rewards'], np.float32).reshape(-1)
    term = np.asarray(z['terminated'], bool).reshape(-1)
    trunc = np.asarray(z['truncated'], bool).reshape(-1)
    if act.shape[0] != n or rew.shape[0] != n or term.shape[0] != n:
        raise ValueError(f'length mismatch n={n} actions_delta={act.shape} rewards={rew.shape} terminated={term.shape}')
    if img.shape[0] != n + 1:
        raise ValueError(f'images must hold n+1={n+1} frames (obs before each decision + final), got {img.shape[0]}')
    if not (bool(term[-1]) or bool(trunc[-1])):
        raise ValueError('last row must be terminated or truncated (tape ends where the env ended)')
    if term[:-1].any() or trunc[:-1].any():
        raise ValueError('terminated/truncated set before the last row -- the tape continues past the env terminal')
    # sparse stage rewards only (shaping is never baked into tapes). A FAST pick can land the
    # stage grant AND the hardened-pick terminal inside ONE repeat-4 window -> reward 2.0 on the
    # terminal row (seen on r2d-teacher tapes, ~68-step picks); the dreamer tape still carries a
    # single terminal_reward there. Positive reward anywhere BEFORE the terminal row is refused.
    if not np.all(np.isin(np.round(rew, 6), [0.0, 1.0, 2.0])):
        raise ValueError(f'rewards must be sparse {{0,1,2}} (shaping is never baked into tapes); seen {np.unique(rew)[:6]}')
    if (rew[:-1] > 0).any():
        raise ValueError('positive reward before the terminal row -- pick scope pays only at the terminal decision')
    if np.abs(act).max() > 1.0 + 1e-6:
        raise ValueError('actions_delta outside [-1,1]')
    T = n + 1
    action = np.concatenate([np.zeros((1, act.shape[1]), np.float32), act])
    reward = np.concatenate([[0.0], rew * float(terminal_reward)]).astype(np.float32)
    is_terminal = np.zeros(T, bool); is_terminal[-1] = bool(term[-1])
    is_first = np.zeros(T, bool); is_first[0] = True
    is_last = np.zeros(T, bool); is_last[-1] = True
    return dict(image=img.astype(np.uint8), action=action, reward=reward,
                discount=(1.0 - is_terminal.astype(np.float32)), is_first=is_first, is_last=is_last,
                is_terminal=is_terminal, logprob=np.zeros(T, np.float32))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', required=True, help='contract-v1 episode dir (npz with images)')
    ap.add_argument('--dst', required=True, help='dreamer demo dir to create (refuses to overwrite unless --force)')
    ap.add_argument('--repeat', type=int, required=True, help='MUST equal the tapes\' action_repeat stamp and the training run\'s action_repeat')
    ap.add_argument('--terminal-reward', type=float, default=100.0, help='MS-parity terminal scale (env genesis_reward_scale)')
    ap.add_argument('--scope', default='pick')
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.src, '*.npz')))
    if not files:
        sys.exit(f'FATAL: no npz in {args.src}')
    if os.path.exists(args.dst) and os.listdir(args.dst) and not args.force:
        sys.exit(f'FATAL: {args.dst} exists and is not empty (use --force)')
    census = dict(n_written=0, n_pick=0, n_nopick=0, n_tipped_terminal=0, n_cap_truncated=0)
    lens, total_reward, cap_seen, errors = [], 0.0, set(), []
    plan = []
    for f in files:
        z = np.load(f, allow_pickle=True)
        need = ['images', 'actions_delta', 'rewards', 'terminated', 'truncated']
        miss = [k for k in need if k not in z.files]
        if miss:
            errors.append(f'{os.path.basename(f)}: missing {miss} (not a contract-v1 tape with images)'); continue
        rep = int(z['action_repeat']) if 'action_repeat' in z.files else None
        if rep != args.repeat:
            errors.append(f'{os.path.basename(f)}: action_repeat stamp {rep} != --repeat {args.repeat}'); continue
        if 'delta_cap' in z.files: cap_seen.add(round(float(z['delta_cap']), 6))
        try:
            ep = convert_one(z, args.terminal_reward)
        except ValueError as e:
            errors.append(f'{os.path.basename(f)}: {e}'); continue
        uid = int(z['uid']) if 'uid' in z.files else int(os.path.splitext(os.path.basename(f))[0])
        T = len(ep['reward'])
        picked = bool(ep['reward'].sum() > 0)
        census['n_pick' if picked else 'n_nopick'] += 1
        if ep['is_terminal'][-1] and not picked: census['n_tipped_terminal'] += 1
        if not ep['is_terminal'][-1]: census['n_cap_truncated'] += 1
        lens.append(T); total_reward += float(ep['reward'].sum())
        plan.append((f, uid, T, ep))
    if errors:
        print('\n'.join('  ' + e for e in errors), file=sys.stderr)
        sys.exit(f'FATAL: {len(errors)} tape(s) rejected -- fix the source set, do not convert a partial set')
    if len(cap_seen) > 1:
        sys.exit(f'FATAL: mixed delta_cap stamps in source set: {sorted(cap_seen)}')
    print(f'[to_dreamer_native] {len(plan)} tapes from {args.src}: pick {census["n_pick"]} / no-pick {census["n_nopick"]} '
          f'(tipped-terminal {census["n_tipped_terminal"]}, cap-truncated {census["n_cap_truncated"]}); '
          f'T p50 {int(np.median(lens))} max {max(lens)}; total reward {total_reward:.0f}')
    if args.dry_run:
        print('[dry-run] nothing written'); return
    os.makedirs(args.dst, exist_ok=True)
    for f, uid, T, ep in plan:
        np.savez_compressed(os.path.join(args.dst, f'genesis-{uid:06d}-{T}.npz'), **ep)
        census['n_written'] += 1
    src_manifest = os.path.join(args.src, 'manifest.json')
    meta = dict(
        action_repeat=int(args.repeat), contract='v1', action_encoding='delta_joint',
        delta_cap=(sorted(cap_seen)[0] if cap_seen else None), scope=args.scope,
        terminal_reward=float(args.terminal_reward), grant_slack_decisions=0,
        src=os.path.abspath(args.src), src_sha=tape_sha(files),
        src_manifest_sha=(json.load(open(src_manifest)).get('content_sha256') if os.path.exists(src_manifest) else None),
        generator='baselines/rl/to_dreamer_native.py',
        created=datetime.datetime.now().isoformat(timespec='seconds'),
        total_reward=total_reward, decisions_min=int(min(lens)), decisions_median=int(np.median(lens)),
        decisions_max=int(max(lens)), **census)
    with open(os.path.join(args.dst, 'repeat.json'), 'w') as fh:
        json.dump(meta, fh, indent=1)
    print(f'wrote {census["n_written"]} episodes + repeat.json -> {args.dst}')


if __name__ == '__main__':
    main()
