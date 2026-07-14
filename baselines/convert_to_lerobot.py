"""Stage 2: pack baselines/episodes_raw/*.npz into a LeRobotDataset (v3, lerobot 0.4.x).

Run with the LEROBOT venv:
    ~/workspace/lerobot/.venv/bin/python baselines/convert_to_lerobot.py

Writes to baselines/lerobot_dataset/genesis_pickaplace (local root, no hub push).
Train Diffusion Policy on it with lerobot's CLI, e.g.:

    ~/workspace/lerobot/.venv/bin/lerobot-train \
      --dataset.repo_id=local/genesis_pickaplace \
      --dataset.root=baselines/lerobot_dataset/genesis_pickaplace \
      --policy.type=diffusion \
      --output_dir=baselines/outputs/dp_state \
      --policy.push_to_hub=false
"""
import sys
import pathlib as pl
import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
# argv: <raw_episode_dir> <dataset_root> [proprio_dim] -- parametrized so version
# switches are explicit at the call site, not silent file edits (lesson: the v3 chain
# once trained on stale v1 data because a path edit lived in a dead chain)
RAW = REPO / (sys.argv[1] if len(sys.argv) > 1 else 'baselines/episodes_raw_v3')
ROOT = REPO / (sys.argv[2] if len(sys.argv) > 2 else 'baselines/lerobot_dataset/genesis_pickaplace')
FPS = 30
TASK = 'pick the can and slide it against the can on the shelf'

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# drop degenerate episodes. Default 100 (a real full-task pick->place->slide is 300+
# frames), but pick-SCOPE AI-harvest rollouts truncate shortly after the lift and can be
# <100 frames -- pass a smaller 4th argv there so they aren't silently dropped.
MIN_FRAMES = int(sys.argv[4]) if len(sys.argv) > 4 else 100
files = [f for f in sorted(RAW.glob('*.npz'), key=lambda p: int(p.stem))
         if int(np.load(f)['n']) >= MIN_FRAMES]
assert files, f'no episodes >= {MIN_FRAMES} frames in {RAW} - run collect_lerobot_dataset.py first'
probe = np.load(files[0])
has_images = 'images' in probe
sdim = probe['states'].shape[1]; adim = probe['actions'].shape[1]

# split the recorded state: proprio -> observation.state, world (can pose + goal xy)
# -> observation.environment_state. Diffusion Policy requires an image or an
# environment_state input; this split makes state-only training work out of the box.
# v4+: proprio is 8 (6 joints, gripper pos, grip effort); v1-v3 were 7.
PROPRIO = int(sys.argv[3]) if len(sys.argv) > 3 else (probe['states'].shape[1] - 9)
features = {
    'observation.state': {'dtype': 'float32', 'shape': (PROPRIO,), 'names': None},
    'observation.environment_state': {'dtype': 'float32', 'shape': (sdim - PROPRIO,),
                                      'names': None},
    'action': {'dtype': 'float32', 'shape': (adim,), 'names': None},
}
if has_images:
    features['observation.images.cam'] = {'dtype': 'video',
                                          'shape': tuple(probe['images'].shape[1:]),
                                          'names': ['height', 'width', 'channels']}

ds = LeRobotDataset.create(repo_id='local/genesis_pickaplace', fps=FPS, root=ROOT,
                           features=features, use_videos=has_images)
for f in files:
    d = np.load(f)
    n = int(d['n'])
    for i in range(n):
        frame = {'observation.state': d['states'][i][:PROPRIO],
                 'observation.environment_state': d['states'][i][PROPRIO:],
                 'action': d['actions'][i],
                 'task': TASK}
        if has_images:
            frame['observation.images.cam'] = d['images'][i]
        ds.add_frame(frame)
    ds.save_episode()
    print(f'{f.stem}: {n} frames', flush=True)
print(f'\ndataset at {ROOT}: {len(files)} episodes')
