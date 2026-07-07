"""Real-vs-sim EEF trajectory comparison (stage 2). Loads pre-extracted tool_pose npz
(from extract_tp.py, run in the rosbags venv) and overlays on the sim EEF under replay.
Reports whether the sim arm TRACKS the recorded motion and where it diverges.
Usage: compare_real_sim.py <toolpose_dir> <uid> [uid...]
"""
import os, sys, pathlib as pl
os.environ.setdefault('PYOPENGL_PLATFORM','egl')
import numpy as np, json, torch
sys.path.insert(0, str(pl.Path(__file__).parent))
from replay_harness import build_world, load_episode, gripper_targets, HARDCODED_START, REPO
def np_(x): return x.detach().cpu().numpy() if isinstance(x,torch.Tensor) else np.asarray(x)
TPDIR = pl.Path(sys.argv[1])
t = json.load(open(REPO/'can_pos_recovery/trial_placements.json')); wc=t['world']; MOUNT=np.array([0,0,0.05])
w = build_world(backend='cpu', finger_force=wc['finger_force'], finger_kp=wc['finger_kp'],
                can_height=wc['can_height'], can_rho=wc['can_rho'], substeps=wc['substeps'],
                table=wc['table'], can_radius=wc['can_radius'])
scene,kin,bottle,goal,kdofs,eef=w['scene'],w['kinova'],w['bottle'],w['goal'],w['kdofs'],w['eef']
print(f"{'uid':>4} {'status':>9} {'resid_mean':>10} {'p90':>6} {'max':>6} {'diverge@':>10}  verdict")
for uid in sys.argv[2:]:
    r=t['trials'][uid]; solved=r['status'] in ('ok','ok_batch')
    real=np.load(TPDIR/f'{uid}.npz')['tool_pose'][:,1:4]
    vel,gp=load_episode(uid)
    cp=r.get('can_pos') or [0.45,0.1,w['can_start_z']]; gpos=r.get('goal_pos') or [0.6,-0.2,w['goal_start_z']]
    kin.set_dofs_position(np.array(HARDCODED_START),kdofs); kin.zero_all_dofs_velocity()
    bottle.set_pos(cp); bottle.set_quat(r.get('can_quat') or [1,0,0,0]); goal.set_pos(gpos); goal.set_quat([1,0,0,0]); scene.step()
    sim=[]
    for i in range(len(vel)):
        kin.control_dofs_position(vel[i],dofs_idx_local=kdofs[:6])
        kin.control_dofs_position(np.array(gripper_targets(gp[i])),dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3): scene.step()
        sim.append(np_(eef.get_pos())-MOUNT)
    sim=np.array(sim)
    ri=np.linspace(0,len(real)-1,len(sim)).astype(int); real_rs=real[ri]
    off=np.median(sim-real_rs,axis=0); resid=np.linalg.norm((sim-off)-real_rs,axis=1)
    brk=int(np.argmax(resid>0.03)) if (resid>0.03).any() else -1
    verdict='tracks well' if resid.mean()<0.025 else ('loses trajectory' if brk<0.5*len(sim) else 'tracks then diverges late')
    print(f"{uid:>4} {'SOLVED' if solved else 'unsolved':>9} {resid.mean()*100:>9.1f}cm {np.percentile(resid,90)*100:>5.1f} {resid.max()*100:>5.1f} {brk:>5}/{len(sim):<4}  {verdict}",flush=True)
