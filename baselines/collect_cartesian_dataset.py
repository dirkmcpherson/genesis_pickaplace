"""Build the 4-DOF Cartesian BC dataset: (sim_obs, human commanded ee-velocity).

The demos' native action is the human's Cartesian velocity command (from the bags,
<id>_cartesian.npy: cartesian_velocity [vx,vy,vz, wx,wy,wz], joint_pos, gripper_pos --
all frame-aligned). We do NOT reconstruct velocities from joints (that's circular + lossy);
we use the commanded velocity DIRECTLY as the action, and use the joint replay only to
generate a faithful SIM observation trajectory.

Per frame i we step the sim to joint_pos[i] (so the sim reaches config q_i), read the
ee-centric obs at q_i, and pair it with the command issued at q_i:
    obs_i  = [ee_pos(3), ee_quat(4), gripper(1), grip_effort(1), can xyz(3), can quat(4), goal xy(2)]  (18)
    act_i  = [vx, vy, vz, v_pitch(=cartesian_velocity wy), gripper_norm]                                (5)
This is the correct (s_i, a_i): state where the human was, command they then issued (which
moves toward q_{i+1}). No off-by-one -- the command at frame i is paired with the state at
frame i, not the post-motion state.

Coherence check (prints per-episode): mean cosine similarity between the SIM ee linear
velocity and the commanded linear velocity -- if ~1, the command and the joint motion agree
(they are two views of one trajectory), confirming the pairing is in-sync, not disjointed.

Output: baselines/episodes_cartesian/<uid>.npz  states(n,18) actions(n,5) n uid picked placed contact coh
Usage: python baselines/collect_cartesian_dataset.py [--uids ...] [--outdir ...]
"""
import argparse, json, glob, pathlib as pl
import numpy as np, torch
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
import sys
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))
from genesis_can_env import GenesisCanEnv

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser()
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--outdir', default='baselines/episodes_cartesian')
args = ap.parse_args()
OUT = REPO / args.outdir; OUT.mkdir(parents=True, exist_ok=True)
DT = 0.025   # demo frame period (cartesian_velocity integrates to tool_pose at this dt)
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}

tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
env = GenesisCanEnv(backend='cpu')
CANZ, GOALZ = env.w['can_start_z'], env.w['goal_start_z']
STATIC_GOAL = (0.6, -0.2)

# all trials that have a cartesian npy
avail = sorted(int(pl.Path(p).stem.split('_')[0]) for p in glob.glob(str(REPO / 'inthewild_trials/*_cartesian.npy')))
uids = args.uids or avail


def place(uid):
    r = tbl.get(str(uid), {})
    solved = r.get('status') in ('ok', 'ok_batch')
    cq = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
    if solved:
        cx, cy = r['can_pos'][0], r['can_pos'][1]
        gx, gy = (r['goal_pos'][0], r['goal_pos'][1]) if (r.get('label') == 'success' and not r.get('goal_moved')) else STATIC_GOAL
    else:
        f = fk.get(uid, {})
        seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED') else BUCKET[f.get('pos')])
        cx, cy = seed[0], seed[1]; gx, gy = STATIC_GOAL
    return (cx, cy, CANZ), cq, (gx, gy, GOALZ)


n_ok = n_pick = 0
for uid in uids:
    d = np.load(REPO / f'inthewild_trials/{uid}_cartesian.npy', allow_pickle=True).item()
    jp = np.asarray(d['joint_pos'], float); cv = np.asarray(d['cartesian_velocity'], float)
    gp = np.asarray(d['gripper_pos'], float)[:, 0]
    n = len(jp)
    can_pos, can_quat, goal_pos = place(uid)
    env.reset(can_pos=can_pos, can_quat=can_quat, goal_pos=goal_pos)
    states, actions = [], []; picked = placed = contact = False
    ee_prev = None; sim_v = []
    for i in range(n):
        grip = float(np.clip(gp[i] / 100.0, 0, 1))
        obs, done, info = env.step(np.concatenate([jp[i], [grip]]))
        ee = np_(env.w['eef'].get_pos()); eq = np_(env.w['eef'].get_quat())
        s = obs['state']
        states.append(np.concatenate([ee, eq, s[6:8], s[8:17]]).astype(np.float32))
        actions.append(np.array([cv[i, 0], cv[i, 1], cv[i, 2], cv[i, 4], grip], np.float32))
        picked = picked or info['picked']; placed = placed or info['placed']; contact = contact or info['contact']
        if ee_prev is not None:
            sim_v.append((ee - ee_prev) / DT)
        ee_prev = ee
    # coherence: cosine sim between sim ee lin-vel and commanded lin-vel, on moving frames
    sim_v = np.array(sim_v); cmd_v = cv[1:, :3]
    mask = (np.linalg.norm(sim_v, axis=1) > 1e-3) & (np.linalg.norm(cmd_v, axis=1) > 1e-3)
    if mask.sum():
        cs = (sim_v[mask] * cmd_v[mask]).sum(1) / (np.linalg.norm(sim_v[mask], axis=1) * np.linalg.norm(cmd_v[mask], axis=1) + 1e-9)
        coh = float(np.mean(cs))
    else:
        coh = float('nan')
    np.savez_compressed(OUT / f'{uid}.npz', states=np.array(states), actions=np.array(actions),
                        n=n, uid=uid, picked=picked, placed=placed, contact=contact, coh=coh)
    n_ok += 1; n_pick += picked
    print(f"{uid}: n={n} picked={picked} placed={placed} contact={contact} coherence(cos)={coh:.3f}", flush=True)
print(f"\ncollected {n_ok} episodes ({n_pick} pick) -> {OUT}")
