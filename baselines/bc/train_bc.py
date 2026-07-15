"""Plain behavior-cloning baseline: a small MLP that regresses state -> action.

The simplest possible demo consumer -- no diffusion, no RL, no chunking -- so it isolates
"what can you get from the demonstrations alone." Reads per-episode npz with keys
states (n, S), actions (n, A). Dimension-flexible: works for the joint action space
(17-dim state, 7-dim action = [6 joints, gripper]) AND the 4-DOF Cartesian action space
(18-dim ee-centric state, 5-dim action = [vx,vy,vz,v_pitch, gripper]).

Inputs standardized by dataset mean/std. Actions are standardized on every dim EXCEPT the
last, which is the gripper (already 0..1) and is regressed directly -- this holds for both
layouts since the gripper is the last action dim. MSE, Adam. Saves state_dict + norm stats
+ dims to bc.pt. CPU, .venv-eval.

Usage:
  python baselines/bc/train_bc.py <raw_episode_dir> [--out bc.pt] [--seed 0]
                                  [--epochs 200] [--batch 256] [--hidden 256]
                                  [--picked-only]   # keep only episodes flagged picked=True
"""
import argparse
import glob
import pathlib as pl

import numpy as np
import torch
import torch.nn as nn


class BCNet(nn.Module):
    def __init__(self, state_dim, act_dim, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, act_dim),
        )

    def forward(self, x):
        return self.net(x)


def load_dataset(raw_dir, picked_only=False):
    paths = sorted(glob.glob(str(pl.Path(raw_dir) / '*.npz')))
    assert paths, f'no *.npz in {raw_dir}'
    S, A = [], []
    n_used = 0
    for p in paths:
        d = np.load(p, allow_pickle=True)
        s = np.asarray(d['states'], dtype=np.float32)
        a = np.asarray(d['actions'], dtype=np.float32)
        if len(s) < 2:
            continue
        if picked_only and 'picked' in d and not bool(d['picked']):
            continue
        # tolerate 16-dim legacy joint states by padding grip_effort at idx 7
        if s.shape[1] == 16:
            s = np.insert(s, 7, 0.0, axis=1)
        S.append(s)
        A.append(a)
        n_used += 1
    assert S, f'no usable episodes in {raw_dir} (picked_only={picked_only})'
    S, A = np.concatenate(S), np.concatenate(A)
    assert len({x.shape[1] for x in [S]}) == 1
    return S, A, n_used


def compute_norm(S, A):
    in_mean = S.mean(0)
    in_std = S.std(0) + 1e-6
    out_mean = A[:, :-1].mean(0)          # every action dim except the gripper (last)
    out_std = A[:, :-1].std(0) + 1e-6
    return in_mean, in_std, out_mean, out_std


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw_dir')
    ap.add_argument('--out', default='baselines/bc/bc.pt')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--epochs', type=int, default=200)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--hidden', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--picked-only', action='store_true')
    args = ap.parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    S, A, n_eps = load_dataset(args.raw_dir, picked_only=args.picked_only)
    state_dim, act_dim = S.shape[1], A.shape[1]
    in_mean, in_std, out_mean, out_std = compute_norm(S, A)
    X = (S - in_mean) / in_std
    Y = A.copy()
    Y[:, :-1] = (A[:, :-1] - out_mean) / out_std     # non-gripper dims in z-space; gripper 0..1
    X = torch.from_numpy(X.astype(np.float32))
    Y = torch.from_numpy(Y.astype(np.float32))
    print(f'[bc] {n_eps} episodes, {len(X)} transitions, state_dim={state_dim} '
          f'act_dim={act_dim} from {args.raw_dir}', flush=True)

    net = BCNet(state_dim, act_dim, args.hidden)
    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    lossf = nn.MSELoss()
    n = len(X)
    for ep in range(args.epochs):
        perm = torch.randperm(n)
        tot = 0.0
        for i in range(0, n, args.batch):
            idx = perm[i:i + args.batch]
            opt.zero_grad()
            loss = lossf(net(X[idx]), Y[idx])
            loss.backward()
            opt.step()
            tot += loss.item() * len(idx)
        if ep % 20 == 0 or ep == args.epochs - 1:
            print(f'[bc] epoch {ep}: mse {tot / n:.5f}', flush=True)

    out = pl.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save({'state_dict': net.state_dict(), 'hidden': args.hidden,
                'state_dim': state_dim, 'act_dim': act_dim,
                'in_mean': in_mean, 'in_std': in_std,
                'out_mean': out_mean, 'out_std': out_std,
                'raw_dir': str(args.raw_dir), 'seed': args.seed}, out)
    print(f'[bc] saved -> {out}', flush=True)


def load_bc_runner(path):
    """-> policy_action(obs)->physical action vector. Mirrors dp_runner.load_dp_runner's
    contract (returns just the closure; BC is stateless so no reset needed). Standardizes
    every output dim except the last (gripper), which is clipped to 0..1."""
    ck = torch.load(path, map_location='cpu', weights_only=False)
    sd = int(ck.get('state_dim', len(ck['in_mean'])))
    ad = int(ck.get('act_dim', len(ck['out_mean']) + 1))
    net = BCNet(sd, ad, ck['hidden'])
    net.load_state_dict(ck['state_dict'])
    net.eval()
    in_mean, in_std = ck['in_mean'], ck['in_std']
    out_mean, out_std = ck['out_mean'], ck['out_std']

    def policy_action(obs):
        x = ((obs['state'] - in_mean) / in_std).astype(np.float32)
        with torch.no_grad():
            y = net(torch.from_numpy(x).unsqueeze(0)).squeeze(0).numpy()
        a = y.copy()
        a[:-1] = y[:-1] * out_std + out_mean
        a[-1] = float(np.clip(y[-1], 0.0, 1.0))
        return a

    return policy_action


if __name__ == '__main__':
    main()
