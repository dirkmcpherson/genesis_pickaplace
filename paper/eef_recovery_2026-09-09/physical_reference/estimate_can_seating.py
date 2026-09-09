"""Conditional metric interpretation of observed rims, separate from projection check.

Fit real can translation to manually annotated rim contours under stated radius,
upright/tilt and camera assumptions. Never use simulation success as a target.
These are measurement hypotheses, not adopted object or simulator corrections.
"""
from pathlib import Path
import json,sys
import cv2,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent;REPO=R.parents[2]
sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
comparison=json.loads((R/'233/can_seating_comparison.json').read_text())
pilot=json.loads((R/'233/rigid_cap_camera_pilot.json').read_text())
source=np.load(comparison['source']);fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
gripper_to_tool=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix())
def tool_pose(t):
 q=np.array([np.interp(t,source['t_frame'],source['q_frame'][:,j]) for j in range(6)])
 return fk.tool(q,np.eye(4))
def apply(p,x):return (p[:3,:3]@x.T).T+p[:3,3]
caps=np.array([[.0152,-.0305,.070003],[.0152,.0305,.070003]])
train=np.concatenate([apply(tool_pose(t)@np.linalg.inv(gripper_to_tool),caps) for t in pilot['camera_times_s']])
pix=np.array(list(pilot['manual_cap_centers_rotated_pixels'].values())).reshape(-1,2)
pix=np.ascontiguousarray(np.c_[1279-pix[:,1],pix[:,0]],dtype=np.float64)
fx,fy,cx,cy,kold=pilot['intrinsics_prior']
phi=np.linspace(0,2*np.pi,128,endpoint=False)
rim=np.c_[.033*np.cos(phi),.033*np.sin(phi),np.zeros(len(phi))]
rows=[]
for scale in [.8,.9,1.,1.1,1.2]:
 for k in [-.25,kold,0.]:
  K=np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1.]])
  dist=np.array([k,0,0,0,0.])
  ok,rv,tv=cv2.solvePnP(train,pix,K,dist,flags=cv2.SOLVEPNP_SQPNP);assert ok
  rv,tv=cv2.solvePnPRefineLM(train,pix,K,dist,rv,tv)
  for rec in comparison['records']:
   tool=tool_pose(rec['time_s']);simrelative=np.array(rec['sim_can_from_tool'])
   simworld=tool@simrelative
   observed=np.array(rec['manual_outer_rim_points_px'])
   for roll,pitch in [(0,0),(5,0),(-5,0),(0,5),(0,-5)]:
    orientation=Rotation.from_euler('xy',[roll,pitch],degrees=True).as_matrix()
    shape=(orientation@rim.T).T
    def residual(center):
     uv=cv2.projectPoints(shape+center,rv,tv,K,dist)[0].reshape(-1,2)
     upright=np.c_[uv[:,1],1279-uv[:,0]].astype(np.float32)
     ctr,diam,angle=cv2.fitEllipse(upright)
     a=np.deg2rad(angle);rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
     unit=((observed-ctr)@rot)/(np.array(diam)/2)
     return (np.linalg.norm(unit,axis=1)-1)*np.mean(diam)/2
    # Translation is initialized from the simulator but fitted solely to real pixels.
    initial=simworld[:3,3]+orientation[:,2]*.0505
    opt=least_squares(residual,initial,diff_step=1e-4,xtol=1e-10,ftol=1e-10,gtol=1e-8,max_nfev=150)
    center=opt.x-orientation[:,2]*.0505
    estimated_tool=(np.linalg.inv(tool)@np.r_[center,1])[:3]
    delta=estimated_tool-simrelative[:3,3]
    rows.append(dict(label=rec['label'],time_s=rec['time_s'],focal_multiplier=scale,k1=k,
     assumed_roll_pitch_deg=[roll,pitch],converged=bool(opt.success),rim_fit_rms_px=float(np.sqrt(np.mean(opt.fun**2))),
     estimated_real_can_center_robot_base_m=center.tolist(),estimated_real_can_center_tool_m=estimated_tool.tolist(),
     real_minus_sim_center_tool_m=delta.tolist()))
summaries=[]
for rec in comparison['records']:
 selected=[r for r in rows if r['label']==rec['label']]
 nominal=next(r for r in selected if r['focal_multiplier']==1 and r['k1']==kold and r['assumed_roll_pitch_deg']==[0,0])
 shifts=np.array([r['real_minus_sim_center_tool_m'] for r in selected])
 summaries.append(dict(label=rec['label'],nominal=nominal,
  all_assumption_shift_tool_m_min=shifts.min(axis=0).tolist(),all_assumption_shift_tool_m_max=shifts.max(axis=0).tolist(),
  rim_fit_rms_px_range=[min(r['rim_fit_rms_px'] for r in selected),max(r['rim_fit_rms_px'] for r in selected)]))
report=dict(summaries=summaries,fits=rows,
 interpretation='Tool +z points away from wrist along the gripper. Negative real-minus-sim z means the real can sits closer to the wrist, conditional on camera/CAD and cylinder assumptions.',
 assumptions='66 mm outer rim diameter; 101 mm height; cylinder normal upright or tilted +/-5 degrees around robot-base x or y; same cap-based cameras and focal/k1 sensitivity as prior checks. These brackets are not measured uncertainty or confidence intervals. Manual points and cap correspondence carry additional error.',
 scope='Real translations fitted only to real rim contours. Does not fit mechanics, starting positions or pass labels. No correction adopted. Hidden finger shape and cause of seating difference remain unresolved.')
(R/'233/can_seating_estimate.json').write_text(json.dumps(report,indent=2))
print(json.dumps(summaries,indent=2))
