"""Watcher: render each collected demo episode to mp4 as it appears. Replays the saved
actions from the same initial condition (can pose + goal from states[0]) through the
camera env, so we SEE what the collection produced -- picks, carry-drops, never-picks,
fall-overs -- without slowing the collection or SACfD.

Runs on CPU (GPU is busy with DP training). Idempotent: skips episodes already rendered.
Usage: render_collected.py <episode_dir> <video_dir> [--once]
"""
import sys, time, glob, pathlib as pl
import numpy as np, cv2, torch
REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv
from replay_harness import tilt_deg, BOX_TOP_Z, in_shelf_footprint
def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

EP_DIR = REPO / sys.argv[1]; VID_DIR = REPO / sys.argv[2]; VID_DIR.mkdir(parents=True, exist_ok=True)
ONCE = '--once' in sys.argv
env = GenesisCanEnv(backend='cpu', render_size=(480, 640))

def rendered_uids():
    return {pl.Path(p).stem.split('_')[0] for p in glob.glob(str(VID_DIR / '*.mp4'))}

def render_one(npz):
    d = np.load(npz, allow_pickle=True)
    s = np.asarray(d['states']); a = np.asarray(d['actions']); uid = str(d['uid'])
    s0 = s[0]
    # 17-dim layout: [6 joints, gripper, grip_effort, can xyz(8:11), can quat(11:15), goal xy(15:17)]
    can_pos = tuple(s0[8:11]); can_quat = list(s0[11:15]); goal = (float(s0[15]), float(s0[16]), env.w['goal_start_z'])
    obs = env.reset(can_pos=can_pos, can_quat=can_quat, goal_pos=goal)
    frames = []; picked = placed = False
    for i in range(len(a)):
        obs, done, info = env.step(a[i])
        picked = picked or info['picked']; placed = placed or info['placed']
        frames.append(np.asarray(env.w['cam'].render()[0])[:, :, ::-1])
    bp = np_(env.w['bottle'].get_pos()); upright = tilt_deg(np_(env.w['bottle'].get_quat())) < 20
    oc = ('placed-upright' if (placed and in_shelf_footprint(bp) and bp[2] > BOX_TOP_Z and upright)
          else 'placed' if placed else 'picked-drop' if picked else 'no-pick')
    out = VID_DIR / f'{uid}_{oc}.mp4'
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 20, (frames[0].shape[1], frames[0].shape[0]))
    for fr in frames: vw.write(fr.astype(np.uint8))
    vw.release()
    print(f'{uid}: {oc} ({len(frames)}f) -> {out.name}', flush=True)

while True:
    done_uids = rendered_uids()
    todo = [p for p in sorted(glob.glob(str(EP_DIR / '*.npz'))) if pl.Path(p).stem not in done_uids]
    for npz in todo:
        try: render_one(npz)
        except Exception as e: print(f'{pl.Path(npz).stem}: render FAILED {e}', flush=True)
    if ONCE or (not todo and ONCE): break
    time.sleep(90)   # poll for newly-collected episodes
