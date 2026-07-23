"""Benchmark genesis NATIVE batched envs (scene.build(n_envs=N)) vs our current
one-world-per-process approach.

Produces the numbers needed to size both this box and a cluster job:
  n_envs -> GPU MB used by the SIMULATOR, env-steps/s aggregate, per-env steps/s.

The sim's GPU memory matters because on a shared card it competes directly with the
training batch (batch_size x batch_length x 64x64x6 activations). Collection
throughput and gradient batch size are different resources -- this measures the first
and reports what's left for the second.

Usage: bench_batched_envs.py [--envs 1 8 16 32] [--steps 200] [--backend gpu]
"""
import argparse, json, sys, time, pathlib as pl
import numpy as np

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

ap = argparse.ArgumentParser()
ap.add_argument('--envs', type=int, nargs='+', default=[1, 8, 16, 32])
ap.add_argument('--steps', type=int, default=200)
ap.add_argument('--backend', default='gpu')
ap.add_argument('--out', default=None)
args = ap.parse_args()


def gpu_mb():
    try:
        import subprocess
        r = subprocess.run(['nvidia-smi', '--query-gpu=memory.used', '--format=csv,noheader,nounits'],
                           capture_output=True, text=True)
        return int(r.stdout.strip().split('\n')[0])
    except Exception:
        return -1


results = []
base_mb = gpu_mb()
print(f'GPU in use before build: {base_mb} MB (other jobs included)', flush=True)

for n in args.envs:
    # one process per measurement: genesis allows a single gs.init per process
    import subprocess
    code = f'''
import sys, time, json
sys.path.insert(0, "{REPO}/can_pos_recovery")
import numpy as np
from batch_harness import build_batched_world
import subprocess as sp

def mb():
    r = sp.run(["nvidia-smi","--query-gpu=memory.used","--format=csv,noheader,nounits"],
               capture_output=True, text=True)
    return int(r.stdout.strip().split("\\n")[0])

before = mb()
w = build_batched_world({n}, finger_force=50.0, finger_kp=40.0, can_height=0.101,
                        can_rho=1000, substeps=8, table=True, can_radius=0.033)
after = mb()
scene = w["scene"]; kin = w["kinova"]; kdofs = w["kdofs"]
q = np.zeros(({n}, 6), dtype=np.float32)
scene.step()
t0 = time.time()
for i in range({args.steps}):
    kin.control_dofs_position(q, dofs_idx_local=kdofs[:6])
    scene.step()
dt = time.time() - t0
print(json.dumps(dict(n={n}, sim_mb=after-before, total_mb=after,
                      steps_per_s={args.steps}/dt,
                      env_steps_per_s={n}*{args.steps}/dt)))
'''
    r = subprocess.run([str(REPO / '.venv-eval/bin/python'), '-c', code],
                       capture_output=True, text=True)
    line = [l for l in r.stdout.strip().split('\n') if l.startswith('{')]
    if not line:
        print(f'n_envs={n}: FAILED\n{r.stderr[-400:]}', flush=True)
        continue
    d = json.loads(line[-1])
    results.append(d)
    print(f"n_envs={d['n']:<3} sim_gpu={d['sim_mb']:>5} MB  "
          f"scene.step={d['steps_per_s']:>6.1f}/s  "
          f"env-steps={d['env_steps_per_s']:>7.1f}/s", flush=True)

if results:
    print('\n--- scaling vs n_envs=1 ---')
    b = results[0]
    for d in results:
        print(f"n_envs={d['n']:<3} throughput x{d['env_steps_per_s']/b['env_steps_per_s']:.2f}  "
              f"sim GPU {d['sim_mb']} MB")
if args.out:
    pl.Path(args.out).write_text(json.dumps(results, indent=1))
