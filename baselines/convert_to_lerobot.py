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
import os
import sys
import pathlib as pl
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
# argv: <raw_episode_dir> <dataset_root> [proprio_dim] [min_frames] [cameras] [img_dtype] [fps] -- parametrized so version
# switches are explicit at the call site, not silent file edits (lesson: the v3 chain
# once trained on stale v1 data because a path edit lived in a dead chain)
RAW = REPO / (sys.argv[1] if len(sys.argv) > 1 else 'baselines/episodes_raw_v3')
ROOT = REPO / (sys.argv[2] if len(sys.argv) > 2 else 'baselines/lerobot_dataset/genesis_pickaplace')
TASK = 'pick the can and slide it against the can on the shelf'

# --- contract-v1 (decision-rate) input, final round robin 2026-08-23 ------------------
# paper/PREREG_final_round_robin_2026-08-23.md §4.1: tapes recorded by
# baselines/record_demos.py carry one row per DECISION (action_repeat 4 -> 7.5 Hz) with
# `states` (obs before each decision) and `actions` (absolute window-end command) -- the
# same two keys this converter has always read, so the frame loop below is unchanged.
# Detection is by the v1 keys; legacy 30 Hz dirs never carry them, so legacy behaviour
# (FPS 30, MIN_FRAMES 100) is byte-identical. FPS override: 7th argv or LEROBOT_FPS
# (explicit beats detection; a float like 7.5 is accepted -- if a lerobot version
# insists on an int fps, pass 8 and say so in the run's notes).
_first = sorted(RAW.glob('*.npz'))
_probe0 = np.load(_first[0]) if _first else None
CONTRACT_V1 = bool(_probe0 is not None and 'actions_delta' in _probe0.files and 'terminated' in _probe0.files)
_fps_arg = (sys.argv[7] if len(sys.argv) > 7 else os.environ.get('LEROBOT_FPS'))
if _fps_arg is not None:
    FPS = float(_fps_arg); FPS = int(FPS) if float(FPS).is_integer() else FPS
elif CONTRACT_V1:
    _rep = int(_probe0['action_repeat']) if 'action_repeat' in _probe0.files else 4
    FPS = 30 / _rep; FPS = int(FPS) if float(FPS).is_integer() else FPS
else:
    FPS = 30
if CONTRACT_V1:
    print(f'[convert] contract-v1 (decision-rate) input detected -> fps {FPS}; '
          f'states/actions per decision; actions_delta/sim_*/eef_pos/terminated ignored here', flush=True)

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# drop degenerate episodes. Default 100 (a real full-task pick->place->slide is 300+
# frames), but pick-SCOPE AI-harvest rollouts truncate shortly after the lift and can be
# <100 frames -- pass a smaller 4th argv there so they aren't silently dropped.
# contract-v1 tapes are ~4x shorter (decisions, not frames): a champion pick is ~30
# decisions, so the legacy 100 would silently drop every model tape -> default 4 for v1.
MIN_FRAMES = int(sys.argv[4]) if len(sys.argv) > 4 else (4 if CONTRACT_V1 else 100)
files = [f for f in sorted(RAW.glob('*.npz'), key=lambda p: int(p.stem))
         if int(np.load(f)['n']) >= MIN_FRAMES]
assert files, f'no episodes >= {MIN_FRAMES} frames in {RAW} - run collect_lerobot_dataset.py first'
probe = np.load(files[0])
# --- PHASE_PLAN amendment (ag), 2026-09-14: images from a SEPARATE native `_img` set -----------
# Two opt-in env vars; unset = byte-identical behaviour to every previous call.
#   LEROBOT_IMAGES_FROM=<dir>   an r2dreamer-native `_img` set (genesis-<uid>-<ic>-<T>.npz with
#                               `image` (T,64,64,6) uint8, `action` (T,7) backward-shifted delta,
#                               `state` (T,17)). The recorder tapes in RAW keep supplying `states`
#                               and `actions` -- so the ACTION COLUMN of the dataset is the same
#                               bytes as the state-based DP set of record built from the same RAW
#                               dir -- and only the pixels come from the re-execution. Three gates
#                               per tape (T == n+1; the native delta stream equals the tape's
#                               `actions_delta` byte for byte; the native `state` rows equal the
#                               tape's `states` byte for byte) make a mismatched pairing impossible.
#   LEROBOT_NO_ENV_STATE=1      omit observation.environment_state (the ground-truth can pose and
#                               goal xy) from the dataset entirely -- a pixel policy must not see it.
IMAGES_FROM = os.environ.get('LEROBOT_IMAGES_FROM') or None
NO_ENV_STATE = os.environ.get('LEROBOT_NO_ENV_STATE') == '1'
_NATIVE = {}
if IMAGES_FROM:
    _nd = pl.Path(IMAGES_FROM)
    for _f in sorted(_nd.glob('genesis-*.npz')):
        _uid = int(_f.name.split('-')[1])
        assert _uid not in _NATIVE, f'two native tapes for uid {_uid} in {_nd}'
        _NATIVE[_uid] = _f
    assert _NATIVE, f'no genesis-*.npz under LEROBOT_IMAGES_FROM={_nd}'
    print(f'[convert] images from native set {_nd} ({len(_NATIVE)} tapes); actions/states still '
          f'from the recorder tapes in {RAW}', flush=True)


def native_images(rawfile, d, n):
    """(n,H,W,6) uint8 for one recorder tape, from the native `_img` set + the three gates."""
    uid = int(rawfile.stem)
    assert uid in _NATIVE, f'{rawfile.name}: no native tape for uid {uid} in {IMAGES_FROM}'
    z = np.load(_NATIVE[uid], allow_pickle=True)
    im = np.asarray(z['image']); act = np.asarray(z['action'], np.float32); st = np.asarray(z['state'], np.float32)
    assert im.dtype == np.uint8 and im.ndim == 4 and im.shape[3] == 6, (rawfile.name, im.shape, im.dtype)
    assert im.shape[0] == n + 1, f'{rawfile.name}: native T={im.shape[0]} != n+1={n + 1}'
    assert act.shape[0] == n + 1 and st.shape[0] == n + 1, (rawfile.name, act.shape, st.shape)
    assert act[1:].tobytes() == np.asarray(d['actions_delta'], np.float32).tobytes(), (
        f'{rawfile.name}: native action stream != the tape actions_delta -- wrong pairing')
    assert st[:n].tobytes() == np.asarray(d['states'], np.float32).tobytes(), (
        f'{rawfile.name}: native state rows != the tape states -- wrong pairing')
    assert im.any(), f'{rawfile.name}: native images are the zero placeholder, not renders'
    return im[:n]


has_images = ('images' in probe) or bool(IMAGES_FROM)
PROBE_IMG = (native_images(files[0], probe, int(probe['n'])) if IMAGES_FROM
             else (probe['images'] if 'images' in probe else None))
# 5th argv: which cameras from the (H,W,6) rig stack (ch 0:3 = top, 3:6 = wrist).
# 'top' | 'top,wrist' | 'none' (ignore images even if present). Split into separate
# 3-channel streams: video codecs are RGB -- a 6-channel "video" feature would be
# silently mangled.
CAMERAS = (sys.argv[5] if len(sys.argv) > 5 else ('top,wrist' if has_images else 'none'))
CAMERAS = [] if CAMERAS == 'none' else CAMERAS.split(',')
CAM_SLICE = {'top': slice(0, 3), 'wrist': slice(3, 6)}
has_images = has_images and bool(CAMERAS)
# 6th argv: image storage codec. "image" = PNG frames (PIL/torchvision decode, no
# torchcodec/NPP -- avoids the video-stack ABI mess for small 64x64 frames);
# "video" = mp4. Default "image".
IMG_DTYPE = sys.argv[6] if len(sys.argv) > 6 else 'image'
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
if NO_ENV_STATE:
    del features['observation.environment_state']
    print('[convert] LEROBOT_NO_ENV_STATE=1: observation.environment_state is NOT a dataset '
          'feature (the can pose and goal xy never reach the policy)', flush=True)
for cam in (CAMERAS if has_images else []):
    h, w = PROBE_IMG.shape[1:3]
    features[f'observation.images.{cam}'] = {'dtype': IMG_DTYPE, 'shape': (h, w, 3),
                                             'names': ['height', 'width', 'channels']}

# image_writer_threads: async PNG writing (dtype=image is otherwise synchronous and
# slow -- ~80 img/s single-threaded x 236k frames). Threads parallelize it.
ds = LeRobotDataset.create(repo_id='local/genesis_pickaplace', fps=FPS, root=ROOT,
                           features=features,
                           use_videos=(has_images and IMG_DTYPE == 'video'),
                           image_writer_threads=(8 if IMG_DTYPE == 'image' and has_images else 0))
for f in files:
    d = np.load(f)
    n = int(d['n'])
    IMGS = (native_images(f, d, n) if IMAGES_FROM else (d['images'] if has_images else None))
    for i in range(n):
        frame = {'observation.state': d['states'][i][:PROPRIO],
                 'action': d['actions'][i],
                 'task': TASK}
        if not NO_ENV_STATE:
            frame['observation.environment_state'] = d['states'][i][PROPRIO:]
        for cam in (CAMERAS if has_images else []):
            frame[f'observation.images.{cam}'] = IMGS[i][:, :, CAM_SLICE[cam]]
        ds.add_frame(frame)
    ds.save_episode()
    print(f'{f.stem}: {n} frames', flush=True)
# CRITICAL: flush parquet footers + metadata while pyarrow is still alive. Without
# this, cleanup falls to __del__ at interpreter exit, where module globals (pa) are
# already None -> the metadata flush crashes and the dataset is left with a truncated
# episodes table (info.json says 67, only ~60 committed) that lerobot-train can't load.
ds.finalize()
# INTEGRITY GATE: a dataset whose episodes table is shorter than info.json's
# total_episodes trains fine on some `datasets` versions and raises
# "Invalid key: N is out of bounds" on others -- a corrupt dataset that only fails
# on another machine. Assert here so it can never be shipped.
import json as _json
import glob as _glob
import pyarrow.parquet as _pq
_info = _json.loads((ROOT / 'meta' / 'info.json').read_text())
_rows = sum(_pq.read_table(f).num_rows
            for f in _glob.glob(str(ROOT / 'meta' / 'episodes' / '**' / '*.parquet'),
                                recursive=True))
assert _rows == _info['total_episodes'], (
    f'CORRUPT DATASET: info.json says {_info["total_episodes"]} episodes but the '
    f'metadata table has {_rows} rows. Do not use it.')
_json.dump(dict(src=str(RAW), fps=FPS, contract=('v1' if CONTRACT_V1 else 'legacy'), episodes=[f.name for f in files],
                min_frames=MIN_FRAMES, proprio=PROPRIO, cameras=CAMERAS if has_images else [], img_dtype=IMG_DTYPE,
                images_from=IMAGES_FROM, no_env_state=NO_ENV_STATE),
           open(ROOT / 'genesis_source.json', 'w'), indent=1)
print(f'\ndataset at {ROOT}: {len(files)} episodes (finalized, metadata verified; fps {FPS}, '
      f'{"contract-v1" if CONTRACT_V1 else "legacy"} source; genesis_source.json written)')
