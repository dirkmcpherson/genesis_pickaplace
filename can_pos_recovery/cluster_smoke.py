"""Cluster readiness probe for Genesis headless: run on a GPU node.
Checks CUDA backend init, batched build, physics throughput, and EGL camera render.

  PYOPENGL_PLATFORM=egl python cluster_smoke.py

Pass criteria printed at the end. No repo assets needed (primitives only).
"""
import os, sys, time
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import numpy as np

print(f"python {sys.version.split()[0]}")
try:
    import torch
    print(f"torch {torch.__version__} cuda={torch.cuda.is_available()} "
          f"dev={torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'n/a'}")
except Exception as e:
    print(f"torch: FAIL ({e})")

import genesis as gs
print(f"genesis {gs.__version__}")

t0 = time.time()
gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
scene = gs.Scene(show_viewer=False,
                 sim_options=gs.options.SimOptions(dt=0.01, substeps=4))
scene.add_entity(gs.morphs.Plane())
box = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                       morph=gs.morphs.Box(size=(0.4, 0.75, 0.12), pos=(0.75, -0.19, 0.05)))
can = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.2),
                       morph=gs.morphs.Cylinder(pos=(0.45, 0.1, 0.113),
                                                radius=0.033, height=0.101))
cam = scene.add_camera(res=(320, 240), pos=(1.5, 0.8, 0.6), lookat=(0.5, 0, 0.1),
                       fov=35, GUI=False)
B = 32
scene.build(n_envs=B)
print(f"build(n_envs={B}): {time.time()-t0:.1f}s")

t0 = time.time()
N = 300
for _ in range(N):
    scene.step()
dt = time.time() - t0
print(f"physics: {N} steps x {B} envs in {dt:.1f}s = {N*B/dt:.0f} env-steps/s")

t0 = time.time()
rgb = cam.render()[0]
arr = np.asarray(rgb)
print(f"EGL render: {arr.shape} dtype={arr.dtype} in {time.time()-t0:.2f}s "
      f"(nonzero={int((arr>0).sum())})")

ok = arr.size > 0 and (arr > 0).sum() > 1000
print("\nPASS" if ok else "\nFAIL: camera rendered empty image (EGL/driver issue)")
