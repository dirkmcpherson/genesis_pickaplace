"""Bridge experiment (BATCHED_ENV_PLAN §5.4): does batched-GPU physics reproduce the
validated CPU stage outcomes on real demo tapes?

Replays N solved success demos SIMULTANEOUSLY (one per env, raw commands, padded to
the longest tape by holding the final command -- which is exactly the collector's
settle behavior) and compares each env's reached stage to the CPU-truth manifest
(baselines/demo_manifest_auth.json).

Expectation is NOT bit-parity (GPU vs CPU is a different numeric realization; the
historical transfer cliff). The deliverable is the AGREEMENT RATE + the direction of
disagreements, which tells us how to interpret batched-collected training data.

Usage: bridge_batched_gpu.py [--n 32] [--out bridge_report.json]
"""
import os
import argparse, json, sys, time, pathlib as pl

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

ap = argparse.ArgumentParser()
ap.add_argument('--n', type=int, default=32)
ap.add_argument('--out', default=None)
args = ap.parse_args()

from genesis_vec_env import BatchedCanWorld  # noqa: E402
from replay_harness import load_episode  # noqa: E402

manifest = json.loads((REPO / 'baselines/demo_manifest_auth.json').read_text())
# solved success demos with CPU-truth stages (skip fk-seed: their ICs aren't in placements)
cands = [int(u) for u, r in manifest.items()
         if r['label'] == 'success' and r['source'] == 'solved']
cands = sorted(cands)[:args.n]
N = len(cands)
print(f'bridging {N} solved success demos on batched GPU', flush=True)

w = BatchedCanWorld(n_envs=N, pixels=False, workspace_limit=False, seed=0)
w.reset_envs(np.arange(N), uids=cands)

tapes = [load_episode(u) for u in cands]
lens = [len(v) - 1 for v, _ in tapes]
T = max(lens)
print(f'tape lengths {min(lens)}..{T}; lockstep replay of {T} steps', flush=True)

t0 = time.time()
for t in range(T):
    arm = np.stack([tapes[i][0][min(t, lens[i] - 1)] for i in range(N)])
    gp = np.array([tapes[i][1][min(t, lens[i] - 1)] for i in range(N)])
    w.replay_step_batch(arm, gp)
    if t % 500 == 0:
        print(f'  t={t}/{T} picked={int(w.picked.sum())} ({time.time()-t0:.0f}s)', flush=True)
stages = w.stages_after_settle()
dt = time.time() - t0

rows, agree = [], 0
order = ['no-pick', 'picked', 'placed', 'contact', 'nested']
for i, u in enumerate(cands):
    cpu = manifest[str(u)]['stage']
    gpu = str(stages[i])
    ok = cpu == gpu
    agree += ok
    rows.append(dict(uid=u, cpu=cpu, gpu=gpu, match=ok,
                     delta=order.index(gpu) - order.index(cpu)))
    if not ok:
        print(f'  MISMATCH uid {u}: cpu={cpu} gpu={gpu}')

deltas = [r['delta'] for r in rows]
print(f'\nAGREEMENT: {agree}/{N} = {agree/N:.2f}  (replay {dt:.0f}s for {N}x{T} steps '
      f'= {N*T/dt:.0f} env-steps/s)')
print(f'stage-delta mean {np.mean(deltas):+.2f} (neg = GPU under-achieves vs CPU truth)')
from collections import Counter
print('CPU stage dist :', dict(Counter(r['cpu'] for r in rows)))
print('GPU stage dist :', dict(Counter(r['gpu'] for r in rows)))
if args.out:
    pl.Path(args.out).write_text(json.dumps(rows, indent=1))
