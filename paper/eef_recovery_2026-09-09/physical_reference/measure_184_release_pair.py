"""Reviewable image-space pad orientation; never a recovered 3D joint angle."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;REPO=R.parents[2]
sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics
# Manually selected midpoints at visible top and distal end of the same grey pad.
# Coordinates refer to full-resolution counterclockwise-rotated camera0 imagery.
points=[np.array([[282.,484.],[273.,611.]]),np.array([[315.,478.],[359.,603.]])]
names=['rigid_landmark_30.9s.jpg','rigid_landmark_33s.jpg']
match=json.loads((R/'184/rigid_landmark_matches.json').read_text())[:2]
times=[x['actual_camera_time_s'] for x in match]
angle=lambda p:float(np.degrees(np.arctan2(*(p[1]-p[0])[::-1])))
angles=[angle(p) for p in points]
rng=np.random.default_rng(184);sensitivity=[]
for _ in range(10000):sensitivity.append(angle(points[1]+rng.uniform(-8,8,(2,2)))-angle(points[0]+rng.uniform(-8,8,(2,2))))
with np.load(R.parent/'early_yaw_pool/timed/184_timed.npz') as z:
 ids=[int(np.argmin(abs(z['fb_t']-t))) for t in times]
 pos=z['fb_tool'][ids,:3];grip=z['fb_grip'][ids]
 q=np.array([[np.interp(t,z['t_frame'],z['q_frame'][:,j]) for j in range(6)] for t in times])
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf');poses=[fk.tool(x,np.eye(4)) for x in q]
rotation=float(np.degrees(Rotation.from_matrix(poses[1][:3,:3]@poses[0][:3,:3].T).magnitude()))
result=dict(uid=184,images=[dict(file=str(R/'184'/n),sha256=hashlib.sha256((R/'184'/n).read_bytes()).hexdigest(),pad_axis_points_xy=p.tolist(),projected_axis_angle_deg=a) for n,p,a in zip(names,points,angles)],camera_bag_times_s=times,projected_axis_change_deg=angles[1]-angles[0],point_selection_sensitivity_95pct_deg=np.quantile(sensitivity,[.025,.975]).tolist(),sensitivity_definition='Independent uniform +/-8 pixel endpoint perturbations; diagnostic sensitivity, not a statistical confidence interval',tool_translation_mm=float(np.linalg.norm(pos[1]-pos[0])*1000),urdf_fk_tool_rotation_deg=rotation,recorded_motor_positions=grip.tolist(),interpretation='Visible pad rotates in image while rigid tool pose changes little during motor opening. This does not isolate passive curl, since motor position and contact load both change.',calibration_status='No camera extrinsics, 3D angle or stiffness fitted. Rigid pivot cap is visible in only a small pose range here, insufficient for independent camera calibration.')
(R/'184/release_pair_measurement.json').write_text(json.dumps(result,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2
fig,axs=plt.subplots(1,2,figsize=(8,7))
for ax,name,p,t in zip(axs,names,points,times):
 im=cv2.cvtColor(cv2.imread(str(R/'184'/name)),cv2.COLOR_BGR2RGB);ax.imshow(im);ax.plot(p[:,0],p[:,1],'o-',color='lime',lw=2,ms=4);ax.set_xlim(80,440);ax.set_ylim(750,180);ax.set_title(f'{t:.3f} s | visible grey pad');ax.set_xlabel('Image x (pixels)');ax.set_ylabel('Image y (pixels)')
fig.suptitle('Trial 184: projected pad-axis measurement, not a joint angle')
fig.tight_layout();fig.savefig(R/'184/release_pair_measurement.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k!='images'},indent=2))
