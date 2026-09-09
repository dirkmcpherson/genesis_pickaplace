from pathlib import Path
import json,sys
import numpy as np,cv2
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;REPO=R.parents[2];sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
m=json.loads((R/'233/rigid_cap_camera_pilot.json').read_text());extra=json.loads((R/'233/rigid_cap_unused_14s_time.json').read_text());times=m['camera_times_s']+[extra['camera_bag_time_s']]
meta=json.loads((R.parent/'timestamp_full_pool/233/collection/233_eef_delta.json').read_text())
with np.load(meta['timing_reconstruction']['source']) as z:q=np.array([[np.interp(t,z['t_frame'],z['q_frame'][:,j]) for j in range(6)] for t in times])
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf');tool=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix());poses=[fk.tool(j,np.eye(4))@np.linalg.inv(tool) for j in q]
fx,fy,cx,cy,k1=m['intrinsics_prior'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
# Annotated before inspecting the predicted pixel locations of the unused pose.
observed=np.array([[362.,497.],[440.,490.]]);results=[]
for fit in m['fits']:
 if fit['image_left_local_y_sign']!= -1:continue
 x=fit['surface_x_m'];local=np.array([[x,-.0305,.070003],[x,.0305,.070003]]);pose=poses[-1];xyz=(pose[:3,:3]@local.T).T+pose[:3,3]
 uv=cv2.projectPoints(xyz,np.array(fit['rvec']),np.array(fit['tvec']),K,np.array([k1,0,0,0,0.]))[0].reshape(-1,2);rotated=np.c_[uv[:,1],1279-uv[:,0]]
 camera=np.array(fit['camera_position_robot_base_m']);facing=[]
 for pose in poses:
  center=pose[:3,:3]@local.mean(axis=0)+pose[:3,3];normal=pose[:3,0]*np.sign(x);facing.append(float(np.dot(camera-center,normal)))
 results.append(dict(surface_x_m=x,unused_pose_errors_px=np.linalg.norm(rotated-observed,axis=1).tolist(),predicted_rotated_pixels=rotated.tolist(),surface_facing_dot_m=facing,all_caps_face_camera=bool(np.all(np.array(facing)>0))))
(R/'233/rigid_cap_unused_pose_check.json').write_text(json.dumps(dict(camera_time_s=extra['camera_bag_time_s'],manual_centers_rotated_pixels=observed.tolist(),results=results,scope='One unused pose from same calibration trial; not a held-out-trial or intrinsic calibration. Surface-normal check assumes visible circles lie on corresponding CAD faces.'),indent=2));print(json.dumps(results,indent=2))
