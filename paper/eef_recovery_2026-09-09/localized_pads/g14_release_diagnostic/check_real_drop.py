"""Compare observed lid motion with the wrist motion under a fixed grasp."""
from pathlib import Path
import json,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;REPO=R.parents[3];sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics
fit=json.loads((R/'113_carry_seating_fit.json').read_text())['nominal'];c={k:np.array(v) for k,v in fit['camera'].items()};rel=np.array(fit['can_from_tool'])
obs=json.loads((R/'113_release_rim_centers.json').read_text());matches=json.loads((R/'real_frames.json').read_text())['records']
source=np.load(REPO/'paper/eef_recovery_2026-09-09/early_yaw_pool/timed/113_timed.npz');offset=float(source['t_frame'][0]);fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
phi=np.linspace(0,2*np.pi,128,endpoint=False);rim=np.c_[.033*np.cos(phi),.033*np.sin(phi),np.full(128,.0505)]
fig,axes=plt.subplots(1,4,figsize=(13,5),layout='constrained');rows=[]
for ax,r in zip(axes,obs['records']):
    match=next(m for m in matches if m['requested_replay_time_s']==r['time_s']);err=next(m['time_error_s'] for m in match['matches'] if m['camera']==4);t=offset+r['time_s']+err
    q=np.array([np.interp(t,source['t_frame'],source['q_frame'][:,j]) for j in range(6)]);pose=fk.tool(q,np.eye(4))@rel;world=rim@pose[:3,:3].T+pose[:3,3]
    raw=cv2.projectPoints(world,c['rv'],c['tv'],c['K'],c['dist'])[0].reshape(-1,2);uv=np.c_[raw[:,1],1279-raw[:,0]];center=np.array(cv2.fitEllipse(uv.astype(np.float32))[0])
    observed=np.array(r['rim_center_px']);im=cv2.cvtColor(cv2.imread(str(R/f'113_{r["time_s"]:g}_cam4.jpg')),cv2.COLOR_BGR2RGB)
    ax.imshow(im);ax.plot(*np.vstack([uv,uv[:1]]).T,color='lime');ax.plot(*observed,'yo');ax.set(xlim=(230,460),ylim=(635,400),title=f'{r["time_s"]:g}s');ax.axis('off')
    rows.append(dict(**r,actual_camera_time_s=t,fixed_grasp_predicted_center_px=center.tolist(),observed_minus_predicted_px=(observed-center).tolist()))
base=rows[0]
for r in rows:r['relative_motion_since20p04_px']=(np.array(r['observed_minus_predicted_px'])-base['observed_minus_predicted_px']).tolist()
fig.suptitle('113: real lid moves downward relative to the opening hand\nYellow: observed lid center. Green: preceding fixed-grasp model transported with recorded wrist motion.\nThis measures image motion; it does not establish metric drop height or exact shelf-contact timing.')
fig.savefig(R/'113_real_drop_check.png',dpi=150)
(R/'113_real_drop_check.json').write_text(json.dumps(dict(records=rows,qualification='Conditional fixed-grasp projection using the preceding independent real-rim fit. Approximately5px center annotations;20.34s is blurred. Motion after opening contradicts treating20.04s as established settled support. No source, initial pose or shelf change follows from this image check.'),indent=2));print(rows)
