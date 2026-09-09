"""Conditional camera fit from candidate rigid cap centers; no can-position anchors."""
from pathlib import Path
import json,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;REPO=R.parents[2];sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
# Image-left then image-right centers, manually inspected at full resolution.
annotations={'maximum_sim_can_height':[[367,582],[450,558]],'before_motor_opening':[[423,498],[498,495]],'final':[[219,638],[306,632]]}
match=json.loads((R/'233/view_matches.json').read_text())['matches'];times=[next(x['actual_camera_time_s'] for x in match if x['event']==e and x['camera']==4) for e in annotations]
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf');tool_from_gripper=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix())
meta=json.loads((R.parent/'timestamp_full_pool/233/collection/233_eef_delta.json').read_text())
with np.load(meta['timing_reconstruction']['source']) as z:
 q=np.array([[np.interp(t,z['t_frame'],z['q_frame'][:,j]) for j in range(6)] for t in times])
poses=[fk.tool(j,np.eye(4))@np.linalg.inv(tool_from_gripper) for j in q]
with np.load(REPO/'can_pos_recovery/camera_audit/cam4_model_final.npz') as z:intrinsic=z['camp'][:5]
fx,fy,cx,cy,k1=intrinsic;K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
dist=np.array([k1,0,0,0,0.]);rotated=np.array(list(annotations.values())).reshape(-1,2);pixels=np.ascontiguousarray(np.c_[1279-rotated[:,1],rotated[:,0]],dtype=np.float64)
results=[]
for surface in [-.0152,.0152]:
 for order in [-1,1]:
  local=np.array([[surface,order*.0305,.070003],[surface,-order*.0305,.070003]])
  xyz=np.concatenate([(pose[:3,:3]@local.T).T+pose[:3,3] for pose in poses])
  ok,rvec,tvec=cv2.solvePnP(xyz,pixels,K,dist,flags=cv2.SOLVEPNP_SQPNP)
  if not ok:continue
  rvec,tvec=cv2.solvePnPRefineLM(xyz,pixels,K,dist,rvec,tvec)
  uv=cv2.projectPoints(xyz,rvec,tvec,K,dist)[0].reshape(-1,2);errors=np.linalg.norm(uv-pixels,axis=1)
  camera_xyz=-(Rotation.from_rotvec(rvec.ravel()).as_matrix().T@tvec.ravel())
  depth=(Rotation.from_rotvec(rvec.ravel()).apply(xyz)+tvec.ravel())[:,2]
  results.append(dict(surface_x_m=surface,image_left_local_y_sign=order,errors_px=errors.tolist(),rms_px=float(np.sqrt(np.mean(errors**2))),camera_position_robot_base_m=camera_xyz.tolist(),positive_depth=bool((depth>0).all()),rvec=rvec.ravel().tolist(),tvec=tvec.ravel().tolist(),landmarks_robot_base_m=xyz.tolist(),projected_raw_pixels=uv.tolist()))
results.sort(key=lambda r:r['rms_px'])
report=dict(uid=233,events=list(annotations),camera_times_s=times,manual_cap_centers_rotated_pixels=annotations,nominal_click_uncertainty_px=3,intrinsics_prior=intrinsic.tolist(),intrinsic_source='Existing December18 camera fit, reused as conditional intrinsics only; not an independent calibration',landmark_assumption='Visible circles are proximal pivot cap centers on gripper-base surface x=+/-15.2 mm, at y=+/-30.5 mm,z=70.003 mm; CAD support bounds but correspondence not independently measured',fits=results,status='Pilot identifiability/correspondence check only. Three poses fitted; no held-out image checked, no physical geometry or simulator parameter changed.')
(R/'233/rigid_cap_camera_pilot.json').write_text(json.dumps(report,indent=2))
print(json.dumps([{k:r[k] for k in ['surface_x_m','image_left_local_y_sign','rms_px','camera_position_robot_base_m','positive_depth']} for r in results],indent=2))

