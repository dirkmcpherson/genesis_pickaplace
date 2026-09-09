"""Compare visible real rims to simulated can/tool transforms; fit no object pose.

All camera fits use the original three cap poses. Rim annotations are observations
for evaluation only, recorded before inspecting the simulated can projection.
"""
from pathlib import Path
import argparse, hashlib, json, sys
import cv2, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp
R=Path(__file__).resolve().parent; REPO=R.parents[2]
sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics, transform

rim_clicks={
 '14s':[[389,401],[422,412],[439,433],[422,451],[395,460],[368,451],[351,430],[365,410]],
 '16s':[[458,391],[488,403],[505,422],[492,442],[465,451],[437,442],[419,421],[430,403]],
 '22s':[[460,407],[488,418],[504,439],[489,457],[463,465],[435,456],[420,436],[432,416]],
 '25s':[[469,408],[496,418],[511,439],[497,457],[472,466],[444,457],[429,437],[442,417]],
}
pilot=json.loads((R/'233/rigid_cap_camera_pilot.json').read_text())
checks=json.loads((R/'233/cap_camera_sensitivity.json').read_text())['unused_pose_annotations']
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--trace',type=Path,default=R.parent/'timestamp_full_pool/233/collection/233_eef_delta.npz')
parser.add_argument('--out-prefix',type=Path,default=R/'233/can_seating_comparison')
args=parser.parse_args()
trace_path=args.trace
meta=json.loads(trace_path.with_suffix('.json').read_text())
assert meta['uid']==233,'This reference annotation set is specific to trial 233'
source_path=Path(meta['timing_reconstruction']['source'])
source=np.load(source_path); trace=np.load(trace_path); tr=trace['trajectory']
trace_times=source['t_frame'][0]+(np.arange(len(tr))+1)*.03
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
tool_from_gripper=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix())
def real_tool(t):
 q=np.array([np.interp(t,source['t_frame'],source['q_frame'][:,j]) for j in range(6)])
 return fk.tool(q,np.eye(4))
def real_gripper(t):return real_tool(t)@np.linalg.inv(tool_from_gripper)
localcaps=np.array([[.0152,-.0305,.070003],[.0152,.0305,.070003]])
def apply(p,x):return (p[:3,:3]@x.T).T+p[:3,3]
train=np.concatenate([apply(real_gripper(t),localcaps) for t in pilot['camera_times_s']])
clicks=np.array(list(pilot['manual_cap_centers_rotated_pixels'].values())).reshape(-1,2)
pixels=np.ascontiguousarray(np.c_[1279-clicks[:,1],clicks[:,0]],dtype=np.float64)
fx,fy,cx,cy,kold=pilot['intrinsics_prior'];cameras=[]
for scale in [.8,.9,1.,1.1,1.2]:
 for k in [-.25,kold,0.]:
  K=np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1.]])
  dist=np.array([k,0,0,0,0.])
  ok,rv,tv=cv2.solvePnP(train,pixels,K,dist,flags=cv2.SOLVEPNP_SQPNP);assert ok
  rv,tv=cv2.solvePnPRefineLM(train,pixels,K,dist,rv,tv)
  cameras.append((scale,k,K,dist,rv,tv))
def project(x,camera):
 _,_,K,dist,rv,tv=camera
 uv=cv2.projectPoints(x,rv,tv,K,dist)[0].reshape(-1,2)
 return np.c_[uv[:,1],1279-uv[:,0]].astype(np.float32)
def sim_pose(t,pos,quat):
 xyz=np.array([np.interp(t,trace_times,tr[:,j]) for j in pos])
 rot=Slerp(trace_times,Rotation.from_quat(tr[:,quat]))([t]).as_matrix()[0]
 return transform(xyz,rot)
def ellipse(pts):
 center,axes,angle=cv2.fitEllipse(np.asarray(pts,np.float32))
 return dict(center_px=list(center),axes_px=list(axes),angle_deg=float(angle))

a=np.linspace(0,2*np.pi,128,endpoint=False)
top=np.c_[.033*np.cos(a),.033*np.sin(a),np.full(len(a),.0505)]
bottom=top.copy();bottom[:,2]=-.0505
records=[];fig,axes=plt.subplots(4,2,figsize=(10,14))
for row,check in enumerate(checks):
 t=check['time_s'];label=check['label']
 path=R/'233'/('rigid_cap_unused_14s.jpg' if label=='14s' else f'cap_check_{label}.jpg')
 # Stored quaternions are wxyz; scipy receives xyzw.
 simtool=sim_pose(t,range(6,9),[10,11,12,9])
 simcan=sim_pose(t,range(13,16),[17,18,19,16])
 relative=np.linalg.inv(simtool)@simcan
 placed=real_tool(t)@relative
 topxyz=apply(placed,top);bottomxyz=apply(placed,bottom)
 img=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
 observed=np.asarray(rim_clicks[label]);realellipse=ellipse(observed)
 capcenter=np.asarray(check['centers']).mean(axis=0)
 center=np.array(realellipse['center_px'])
 for ax in axes[row]:
  ax.imshow(img);ax.set_xlim(center[0]-150,center[0]+150);ax.set_ylim(center[1]+200,center[1]-150);ax.axis('off')
 axes[row,0].plot(observed[:,0],observed[:,1],'yo',ms=3)
 axes[row,0].set_title(f'Real rim annotation, {t:.3f} s')
 axes[row,1].set_title('Simulated can relative to real wrist')
 predictions=[]
 for cam in cameras:
  nominal=cam[0]==1 and cam[1]==kold
  uv=project(topxyz,cam);bv=project(bottomxyz,cam)
  predicted=ellipse(uv);predcenter=np.array(predicted['center_px'])
  caps=project(apply(real_gripper(t),localcaps),cam)
  delta=predcenter-center
  relative_delta=(predcenter-caps.mean(axis=0))-(center-capcenter)
  predictions.append(dict(focal_multiplier=cam[0],k1=cam[1],top_rim_ellipse=predicted,
   rim_center_error_px=delta.tolist(),rim_center_error_norm_px=float(np.linalg.norm(delta)),
   cap_relative_rim_error_px=relative_delta.tolist(),cap_relative_error_norm_px=float(np.linalg.norm(relative_delta))))
  for outline,color in [(uv,'red'),(bv,'orange')]:
   loop=np.vstack([outline,outline[:1]])
   axes[row,1].plot(loop[:,0],loop[:,1],color=color,lw=1.5 if nominal else .5,alpha=1 if nominal else .12)
  if nominal:
   axes[row,1].plot(caps[:,0],caps[:,1],'c+',ms=7)
   axes[row,1].plot(center[0],center[1],'yo',ms=4)
 nominal=next(p for p in predictions if p['focal_multiplier']==1 and p['k1']==kold)
 axes[row,1].text(.02,.02,f"Rim center difference {nominal['rim_center_error_norm_px']:.1f} px",transform=axes[row,1].transAxes,color='white',bbox=dict(facecolor='black',alpha=.65))
 records.append(dict(time_s=t,label=label,image=str(path),image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
  manual_outer_rim_points_px=observed.tolist(),manual_point_sensitivity_scale_px=5,real_rim_ellipse=realellipse,
  sim_can_from_tool=relative.tolist(),predictions=predictions,nominal=nominal))
fig.suptitle('233: object seating/placement comparison without fitting can positions\nRed simulated top rim; orange bottom rim; yellow observed top center; cyan predicted caps\nFaint lines: declared lens alternatives, not confidence bounds. Hidden rims remain drawn.',fontsize=10)
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(args.out_prefix.with_suffix('.png'),dpi=160);plt.close(fig)
report=dict(uid=233,records=records,trace=str(trace_path),trace_sha256=hashlib.sha256(trace_path.read_bytes()).hexdigest(),
 source=str(source_path),source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
 scope='No object pose fitting. Simulated can/tool transform attached to real FK tool, using interpolated translation and quaternion SLERP. Camera fits only original cap annotations. Rim points annotated before seeing can projections. Lens alternatives are assumptions, not confidence bounds. Cap-relative image differences subtract mean cap projection error, not a 3D geometry correction.',
 cylinder_dimensions_m=dict(radius=.033,height=.101))
args.out_prefix.with_suffix('.json').write_text(json.dumps(report,indent=2))
print(json.dumps([dict(label=r['label'],nominal=r['nominal'],cap_relative_error_range_px=[min(p['cap_relative_error_norm_px'] for p in r['predictions']),max(p['cap_relative_error_norm_px'] for p in r['predictions'])]) for r in records],indent=2))
