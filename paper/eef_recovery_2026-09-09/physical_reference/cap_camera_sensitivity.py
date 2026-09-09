"""Additional unused-pose checks and conditional lens sensitivity; no model adoption."""
from pathlib import Path
import json,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;REPO=R.parents[2];sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
pilot=json.loads((R/'233/rigid_cap_camera_pilot.json').read_text());oldcheck=json.loads((R/'233/rigid_cap_unused_pose_check.json').read_text());timelist=json.loads((R/'233/cap_check_times.json').read_text())
# Fixed before inspecting projections. Ten-second view excluded: one cap occluded.
clicks={16:[[426,484],[502,479]],22:[[417,518],[493,513]],25:[[316,496],[392,491]]}
checks=[dict(time_s=oldcheck['camera_time_s'],centers=oldcheck['manual_centers_rotated_pixels'],label='14s')]+[dict(time_s=next(r['time_s'] for r in timelist if r['requested_s']==t),centers=p,label=f'{t}s') for t,p in clicks.items()]
meta=json.loads((R.parent/'timestamp_full_pool/233/collection/233_eef_delta.json').read_text());fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf');tool=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix());local=np.array([[.0152,-.0305,.070003],[.0152,.0305,.070003]])
with np.load(meta['timing_reconstruction']['source']) as z:
 times=pilot['camera_times_s']+[r['time_s'] for r in checks];q=np.array([[np.interp(t,z['t_frame'],z['q_frame'][:,j]) for j in range(6)] for t in times])
poses=[fk.tool(j,np.eye(4))@np.linalg.inv(tool) for j in q];xyz=np.array([(p[:3,:3]@local.T).T+p[:3,3] for p in poses]);train=xyz[:3].reshape(-1,3);test=xyz[3:].reshape(-1,3)
raw=lambda p:np.ascontiguousarray(np.c_[1279-np.asarray(p).reshape(-1,2)[:,1],np.asarray(p).reshape(-1,2)[:,0]],dtype=np.float64)
trainpix=raw(list(pilot['manual_cap_centers_rotated_pixels'].values()));testpix=raw([r['centers'] for r in checks]);fx,fy,cx,cy,kold=pilot['intrinsics_prior'];rows=[]
for scale in [.8,.9,1.,1.1,1.2]:
 for k in [-.25,float(kold),0.]:
  K=np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1.]]);dist=np.array([k,0,0,0,0.]);ok,rv,tv=cv2.solvePnP(train,trainpix,K,dist,flags=cv2.SOLVEPNP_SQPNP);assert ok;rv,tv=cv2.solvePnPRefineLM(train,trainpix,K,dist,rv,tv)
  proj=lambda x:cv2.projectPoints(x,rv,tv,K,dist)[0].reshape(-1,2)
  trainerr=np.linalg.norm(proj(train)-trainpix,axis=1);testerr=np.linalg.norm(proj(test)-testpix,axis=1);camera=-(Rotation.from_rotvec(rv.ravel()).as_matrix().T@tv.ravel())
  rows.append(dict(focal_multiplier=scale,k1=k,train_rms_px=float(np.sqrt(np.mean(trainerr**2))),unused_pose_rms_px=float(np.sqrt(np.mean(testerr**2))),unused_pose_errors_px=testerr.reshape(-1,2).tolist(),camera_xyz_robot_base_m=camera.tolist()))
base=next(r for r in rows if r['focal_multiplier']==1 and r['k1']==kold);near=[r for r in rows if r['train_rms_px']<=3]
report=dict(unused_pose_annotations=checks,excluded_view='10 seconds: second cap occluded, not guessed',baseline_intrinsics_result=base,lens_sensitivity=rows,train_rms_le_3px_camera_z_span_m=[min(r['camera_xyz_robot_base_m'][2] for r in near),max(r['camera_xyz_robot_base_m'][2] for r in near)],scope='Camera extrinsics refitted only on original three poses per lens assumption. Four unused poses assessed without selecting a preferred lens. Focal +/-20% and k1 bracket are sensitivity assumptions, not measured lens uncertainty or a confidence interval. Same trial, not held-out-trial validation.')
(R/'233/cap_camera_sensitivity.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['lens_sensitivity','unused_pose_annotations']},indent=2))
