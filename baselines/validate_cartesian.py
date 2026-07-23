"""Validate the Cartesian action interface + check 4-DOF sufficiency, in ONE process/env:
  PASS 1: joint-replay a demo (GenesisCanEnv), record the achieved ee pose trajectory + pick.
  ANALYSIS: how much do roll/yaw/pitch vary vs the start? (4-DOF assumes roll/yaw ~fixed.)
  PASS 2: re-drive the SAME env through IK pose-control of the recorded poses; does it still
          pick, and how closely does the ee track?
"""
import sys, argparse, pathlib as pl
import numpy as np, torch
from scipy.spatial.transform import Rotation as R
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO/'baselines')); sys.path.insert(0, str(REPO/'can_pos_recovery'))
from cartesian_env import CartesianCanEnv, _gs_to_xyzw
from replay_harness import load_episode

def np_(x): return x.detach().cpu().numpy() if isinstance(x,torch.Tensor) else np.asarray(x)

ap = argparse.ArgumentParser(); ap.add_argument('--uid', type=int, default=232); args = ap.parse_args()
cenv = CartesianCanEnv(backend='cpu')
vel, gp = load_episode(args.uid)
grip = [float(np.clip(g/100.0,0,1)) for g in gp]

# PASS 1: joint replay, record achieved ee poses
cenv.env.reset(uid=args.uid)
rec_pos, rec_quat = [], []; picked_j = False
for i in range(len(vel)):
    obs, done, info = cenv.env.step(np.concatenate([vel[i], [grip[i]]]))
    rec_pos.append(np_(cenv.eef.get_pos()).copy()); rec_quat.append(np_(cenv.eef.get_quat()).copy())
    picked_j = picked_j or info['picked']
rec_pos = np.array(rec_pos); rec_quat = np.array(rec_quat)

# ANALYSIS: orientation drift relative to start, in euler (deg). Which axis moves?
r0 = R.from_quat(_gs_to_xyzw(rec_quat[0]))
rel = np.array([( R.from_quat(_gs_to_xyzw(q)) * r0.inv() ).as_euler('xyz', degrees=True) for q in rec_quat])
amp = rel.max(0) - rel.min(0)   # peak-to-peak per axis
print(f"[{args.uid}] joint-replay pick={picked_j}  n={len(vel)}")
print(f"  ee orientation peak-to-peak (deg): roll={amp[0]:.1f} pitch={amp[1]:.1f} yaw={amp[2]:.1f}")
print(f"  ee travel (m): x={np.ptp(rec_pos[:,0]):.3f} y={np.ptp(rec_pos[:,1]):.3f} z={np.ptp(rec_pos[:,2]):.3f}")

# PASS 2: IK pose-control replay of the recorded poses on the SAME env
cenv.env.reset(uid=args.uid)
picked_c = False; errs = []
for i in range(len(vel)):
    joints = cenv._pose_step(rec_pos[i], rec_quat[i])
    obs, done, info = cenv.env.step(np.concatenate([joints, [grip[i]]]))
    picked_c = picked_c or info['picked']
    errs.append(np.linalg.norm(np_(cenv.eef.get_pos()) - rec_pos[i]))
print(f"  IK pose-replay pick={picked_c}  mean ee track err={np.mean(errs)*1000:.1f}mm  max={np.max(errs)*1000:.1f}mm")
print(f"  VERDICT: IK pipeline {'REPRODUCES' if picked_c==picked_j else 'DIVERGES from'} joint replay; "
      f"4-DOF {'SUFFICIENT' if max(amp[0],amp[2])<15 else 'INSUFFICIENT (roll/yaw move >15deg)'}")

# PASS 3: VELOCITY action replay (the policy action path). Derive [vx,vy,vz,pitch_rate] from
# the recorded pose trajectory and feed them through cenv.step (setpoint integration + IK).
pitch_rad = np.radians(rel[:, 1])
acts = []
for i in range(len(vel)):
    p_prev = rec_pos[i-1] if i > 0 else rec_pos[0]
    v = (rec_pos[i] - p_prev) / cenv.DT
    pr = (pitch_rad[i] - (pitch_rad[i-1] if i > 0 else pitch_rad[0])) / cenv.DT
    acts.append([v[0], v[1], v[2], pr, grip[i]])
acts = np.array(acts)
clip_frac = float(np.mean(np.abs(acts[:, :3]) > cenv.VCAP))
cenv.VCAP = 1.0   # isolate the env from the stopgap's overshoot: real commanded vels are capped+clean
cenv.reset(uid=args.uid); picked_v = False; verr = []
for i in range(len(vel)):
    obs, done, info = cenv.step(acts[i])
    picked_v = picked_v or info['picked']
    verr.append(np.linalg.norm(cenv._sp - rec_pos[i]))
print(f"  VELOCITY-action replay pick={picked_v}  setpoint drift vs recorded: "
      f"mean={np.mean(verr)*1000:.1f}mm max={np.max(verr)*1000:.1f}mm  "
      f"(|v|>VCAP clipped on {clip_frac*100:.0f}% of steps)")
