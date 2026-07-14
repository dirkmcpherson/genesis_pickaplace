"""Plain behavior-cloning baseline: a small MLP that regresses state -> action.

The simplest possible demo consumer -- no diffusion, no RL, no chunking -- so it isolates
"what can you get from the demonstrations alone." Reads the SAME per-episode npz that the
harvester and collect_lerobot_dataset.py produce (keys states (n,17), actions (n,7)), so
HUMAN (episodes_raw_v4) and AI (episodes_ai/<tag>) conditions are consumed byte-identically
and match the SACfD input format too. CPU, .venv-eval.

Model: full 17-dim state -> 256 -> 256 -> 7 (ReLU). Inputs standardized by dataset
mean/std; the 6 arm-joint action dims standardized too (regressed in z-space); the gripper
(dim 6, already 0..1) regressed directly. MSE, Adam. Saves state_dict + norm stats to bc.pt.

Usage:
  python baselines/bc/train_bc.py <raw_episode_dir> [--out bc.pt] [--seed 0]
                                  [--epochs 200] [--batch 256] [--hidden 256]
"""
import argparse
import glob
import pathlib as pl

import numpy as np
import torch
import torch.nn as nn

STATE_DIM = 17
ACT_DIM = 7


class BCNet(nn.Module):
    def __init__(self, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(STATE_DIM, hidden), nn.ReLU(),
            nn.Linear(hidden, hidden), nn.ReLU(),
            nn.Linear(hidden, ACT_DIM),
        )

    def forward(self, x):
        return self.net(x)


def load_dataset(raw_dir):
    paths = sorted(glob.glob(str(pl.Path(raw_dir) / '*.npz')))
    assert paths, f'no *.npz in {raw_dir}'
    S, A = [], []
    for p in paths:
        d = np.load(p)
        s = np.asarray(d['states'], dtype=np.float32)
        a = np.asarray(d['actions'], dtype=np.float32)
        if len(s) < 2:
            continue
        # tolerate 16-dim legacy states by padding grip_effort at idx 7 (matches demo_buffer)
        if s.shape[1] == 16:
            s = np.insert(s, 7, 0.0, axis=1)
        assert s.shape[1] == STATE_DIM and a.shape[1] == ACT_DIM, \
            f'{p}: state {s.shape} action {a.shape}'
        S.append(s)
        A.append(a)
    return np.concatenate(S), np.concatenate(A), len(paths)


def compute_norm(S, A):
    in_mean = S.mean(0)
    in_std = S.std(0) + 1e-6
    out_mean = A[:, :6].mean(0)          # arm joints
    out_std = A[:, :6].std(0) + 1e-6
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
    args = ap.parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    S, A, n_eps = load_dataset(args.raw_dir)
    in_mean, in_std, out_mean, out_std = compute_norm(S, A)
    X = (S - in_mean) / in_std
    Y = A.copy()
    Y[:, :6] = (A[:, :6] - out_mean) / out_std      # joints in z-space; gripper stays 0..1
    X = torch.from_numpy(X.astype(np.float32))
    Y = torch.from_numpy(Y.astype(np.float32))
    print(f'[bc] {n_eps} episodes, {len(X)} transitions from {args.raw_dir}', flush=True)

    net = BCNet(args.hidden)
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
                'in_mean': in_mean, 'in_std': in_std,
                'out_mean': out_mean, 'out_std': out_std,
                'raw_dir': str(args.raw_dir), 'seed': args.seed}, out)
    print(f'[bc] saved -> {out}', flush=True)


def load_bc_runner(path):
    """-> policy_action(obs)->physical 7-vec. Mirrors dp_runner.load_dp_runner's contract
    (returns just the closure; BC is stateless so no reset needed)."""
    ck = torch.load(path, map_location='cpu', weights_only=False)
    net = BCNet(ck['hidden'])
    net.load_state_dict(ck['state_dict'])
    net.eval()
    in_mean = ck['in_mean']; in_std = ck['in_std']
    out_mean = ck['out_mean']; out_std = ck['out_std']

    def policy_action(obs):
        x = ((obs['state'] - in_mean) / in_std).astype(np.float32)
        with torch.no_grad():
            y = net(torch.from_numpy(x).unsqueeze(0)).squeeze(0).numpy()
        a = y.copy()
        a[:6] = y[:6] * out_std + out_mean
        a[6] = float(np.clip(y[6], 0.0, 1.0))
        return a

    return policy_action


if __name__ == '__main__':
    main()
