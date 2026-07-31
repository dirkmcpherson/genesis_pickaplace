"""Find which construction step segfaults on the cluster.

A minimal genesis scene (plane, 2 envs, no cameras) prints GENESIS OK there, while
every dreamer run segfaults ~3 min in at scene build. This walks from one to the
other, one variable at a time, flushing after each step so the LAST PRINTED LINE
names the culprit even though a segfault produces no traceback.

Run inside a GPU allocation:
  srun -p preempt --gres=gpu:1 --constraint="l40s|l40" -n 8 --mem=32g --time=0:40:00 \
    --pty bash -c 'module load anaconda/2025.06.0 && conda activate <env> && \
      cd <genesis_pickaplace> && PYTHONUNBUFFERED=1 python cluster/bisect_genesis_crash.py'

Each stage is independent of the ones after it, so the first stage that fails to
print its DONE line is where the fault is.
"""
import os
import sys

os.environ.setdefault('PYTHONUNBUFFERED', '1')
REPO = os.environ.get('GENESIS_PICKAPLACE_ROOT', os.getcwd())
for p in (f'{REPO}/baselines', f'{REPO}/baselines/rl', f'{REPO}/can_pos_recovery'):
    if p not in sys.path:
        sys.path.insert(0, p)


def stage(n, what):
    print(f'\n=== STAGE {n}: {what}', flush=True)


stage(1, 'import genesis + gs.init(gpu)')
import genesis as gs
gs.init(backend=gs.gpu, precision='32', logging_level='warning')
print('STAGE 1 DONE', flush=True)

stage(2, 'trivial scene, 2 envs, NO cameras (the case known to work)')
s = gs.Scene(show_viewer=False)
s.add_entity(gs.morphs.Plane())
s.build(n_envs=2)
print('STAGE 2 DONE', flush=True)
del s

stage(3, 'our world via replay_harness.build_world, NO cameras, single env')
from replay_harness import build_world
w = build_world(show_viewer=False, backend='gpu', camera=False)
print('STAGE 3 DONE (kinova URDF + can + shelf built)', flush=True)

stage(4, 'BatchedCanWorld n_envs=2, NO pixels')
from genesis_vec_env import BatchedCanWorld
print('  (a second gs.init in one process is not supported -- if this stage dies,')
print('   rerun with STAGE_ONLY=4 to build it first in a clean process)', flush=True)
print('STAGE 4 SKIPPED unless STAGE_ONLY=4', flush=True)

if os.environ.get('STAGE_ONLY') == '4':
    w2 = BatchedCanWorld(n_envs=2, pixels=False, workspace_limit=False, seed=0,
                         control='cart_abs6')
    print('STAGE 4 DONE (batched, no pixels)', flush=True)
    w2.reset_envs([0, 1])
    print('STAGE 4b DONE (reset)', flush=True)

if os.environ.get('STAGE_ONLY') == '5':
    w3 = BatchedCanWorld(n_envs=2, pixels=True, workspace_limit=False, seed=0,
                         control='cart_abs6')
    print('STAGE 5 DONE (batched WITH cameras, 2 envs)', flush=True)
    w3.reset_envs([0, 1])
    img = w3.render_batch()
    print(f'STAGE 5b DONE (render {img.shape})', flush=True)

if os.environ.get('STAGE_ONLY') == '6':
    n = int(os.environ.get('NENVS', '32'))
    w4 = BatchedCanWorld(n_envs=n, pixels=True, workspace_limit=False, seed=0,
                         control='cart_abs6')
    print(f'STAGE 6 DONE (batched, cameras, n_envs={n})', flush=True)
    w4.reset_envs(list(range(n)))
    img = w4.render_batch()
    print(f'STAGE 6b DONE (render {img.shape})', flush=True)

print('\nALL REQUESTED STAGES COMPLETED', flush=True)
