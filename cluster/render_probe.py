"""Render-domain probe: reset GenesisPick to fixed uids, save the raw 64x64x6
obs image + compare against the demo npz frame-0 (rendered on the DEV BOX at
collection time). Run on both machines; compare printed stats + saved npy."""
import sys, pathlib
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, '/home/j/workspace/r2dreamer')
try:
    from envs.genesis import GenesisPick
except ImportError:
    sys.path.insert(0, str(pathlib.Path.cwd()))
    from envs.genesis import GenesisPick
import glob, os
DEMO = os.environ.get('DELTA25', '/home/j/workspace/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta25')
env = GenesisPick('pick', size=(64,64), seed=0, scope='pick', action_mode='delta_joint',
                  action_repeat=4, delta_cap=0.025, delta_leash_mult=5)
env.reset()
out = {}
for uid in (232, 242, 243):
    env._env.reset(options={'uid': uid}); env.sync_delta_target()
    img = env._image().astype(np.int16)          # (64,64,6) uint8 -> int16
    npz = glob.glob(f'{DEMO}/genesis-{uid:04d}-*.npz')[0]
    ref = np.load(npz)['image'][0].astype(np.int16)   # frame 0, rendered locally at collection
    diff = np.abs(img - ref)
    print(f'uid {uid}: env-vs-demoframe0  MAE={diff.mean():.2f}  max={diff.max()}  '
          f'frac>8={(diff>8).mean():.3f}  env_mean={img.mean():.1f} ref_mean={ref.mean():.1f}')
    out[str(uid)] = img.astype(np.uint8)
np.savez(os.environ.get('OUT', '/tmp/render_probe.npz'), **out)
print('saved renders ->', os.environ.get('OUT', '/tmp/render_probe.npz'))
