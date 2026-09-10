"""Conditional113 camera fit from rigid caps, then evaluate can seating separately."""
from pathlib import Path
import hashlib,json,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;REPO=R.parents[3];REF=R.parent.parent/'physical_reference';sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
anno=json.loads((R/'113_landmarks.json').read_text());path=R.parent/'g14_feedback_full/113_elliptic10_soft/113_eef_delta.npz';meta=json.loads(path.with_suffix('.json').read_text());source=np.load(meta['timing_reconstruction']['source']);z=np.load(path);tr=z['trajectory'];offset=float(source['t_frame'][0]);origin=json.loads((REF/'bag_origins.json').read_text())['113'];record=next(x for x in json.loads((REF/'manifest.json').read_text())['records'] if x['uid']==113);cam=next(x for x in record['cameras'] if x['camera']==4);times=(np.loadtxt(cam['timestamps'],dtype=np.int64)-origin)/1e9
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf');tool_from_gripper=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix());localcaps=np.array([[.0152,-.0305,.070003],[.0152,.0305,.070003]])
def tool(t):return fk.tool(np.array([np.interp(t,source['t_frame'],source['q_frame'][:,j]) for j in range(6)]),np.eye(4))
def gripper(t):return tool(t)@np.linalg.inv(tool_from_gripper)
def apply(p,v):return v@p[:3,:3].T+p[:3,3]
for row in anno['records']:
 i=int(np.argmin(abs(times-offset-row['time_s'])));row['actual_camera_time_s']=float(times[i]);row['time_match_error_s']=float(times[i]-offset-row['time_s'])
train=[x for x in anno['records'] if x['time_s'] in anno['train_times_s']];xyz=np.concatenate([apply(gripper(x['actual_camera_time_s']),localcaps) for x in train]);points=np.concatenate([x['caps'] for x in train]);uv=np.ascontiguousarray(np.c_[1279-points[:,1],points[:,0]],dtype=np.float64)
prior=json.loads((REF/'233/rigid_cap_camera_pilot.json').read_text())['intrinsics_prior'];fx,fy,cx,cy,k0=prior;fits=[]
for scale in [.8,.9,1.,1.1,1.2]:
 for k in [-.25,k0,0.]:
  K=np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1.]]);dist=np.array([k,0,0,0,0.]);ok,rv,tv=cv2.solvePnP(xyz,uv,K,dist,flags=cv2.SOLVEPNP_SQPNP);assert ok;rv,tv=cv2.solvePnPRefineLM(xyz,uv,K,dist,rv,tv)
  train_uv=cv2.projectPoints(xyz,rv,tv,K,dist)[0].reshape(-1,2);fits.append(dict(scale=scale,k=k,K=K,dist=dist,rv=rv,tv=tv,train_rms_px=float(np.sqrt(np.mean(np.sum((train_uv-uv)**2,axis=1))))))
def project(v,c):
 p=cv2.projectPoints(v,c['rv'],c['tv'],c['K'],c['dist'])[0].reshape(-1,2);return np.c_[p[:,1],1279-p[:,0]]
trtimes=offset+(np.arange(len(tr))+1)*.03
rots={key:Slerp(trtimes,Rotation.from_quat(tr[:,cols])) for key,cols in [('tool',[10,11,12,9]),('can',[17,18,19,16])]}
def simpose(t,key,cols):return transform(np.array([np.interp(t,trtimes,tr[:,j]) for j in cols]),rots[key]([t]).as_matrix()[0])
a=np.linspace(0,2*np.pi,128,endpoint=False);top=np.c_[.033*np.cos(a),.033*np.sin(a),np.full(len(a),.0505)];bottom=top.copy();bottom[:,2]=-.0505
rows=[];fig,axes=plt.subplots(2,3,figsize=(13,9),layout='constrained')
for ax,obs in zip(axes.ravel(),anno['records']):
 t=obs['actual_camera_time_s'];rel=np.linalg.inv(simpose(t,'tool',range(6,9)))@simpose(t,'can',range(13,16));realcan=tool(t)@rel;pred=[];im=cv2.cvtColor(cv2.imread(str(R/f'113_{obs["time_s"]:g}_cam4.jpg')),cv2.COLOR_BGR2RGB);ax.imshow(im);center=np.array(obs['rim_center']);ax.set(xlim=(center[0]-125,center[0]+125),ylim=(center[1]+200,center[1]-125),title=f'{obs["time_s"]:g}s');ax.axis('off')
 for c in fits:
  caps=project(apply(gripper(t),localcaps),c);rim=project(apply(realcan,top),c);ellipse=cv2.fitEllipse(rim.astype(np.float32));pc=np.array(ellipse[0]);delta=pc-center;relative=delta-(caps.mean(axis=0)-np.array(obs['caps']).mean(axis=0));nominal=c['scale']==1 and c['k']==k0
  pred.append(dict(scale=c['scale'],k1=c['k'],train_rms_px=c['train_rms_px'],cap_errors_px=np.linalg.norm(caps-np.array(obs['caps']),axis=1).tolist(),rim_delta_px=delta.tolist(),cap_relative_rim_delta_px=relative.tolist(),cap_relative_error_norm_px=float(np.linalg.norm(relative))))
  if nominal:
   ax.plot(*np.vstack([rim,rim[:1]]).T,color='red');b=project(apply(realcan,bottom),c);ax.plot(*np.vstack([b,b[:1]]).T,color='orange');ax.plot(caps[:,0],caps[:,1],'c+');ax.plot(center[0],center[1],'yo');ax.text(.02,.02,f'Cap-relative error {np.linalg.norm(relative):.1f}px',transform=ax.transAxes,color='white',bbox=dict(facecolor='black',alpha=.6))
 nominal=next(p for p in pred if p['scale']==1 and p['k1']==k0);rows.append(dict(**obs,nominal=nominal,predictions=pred,sim_can_from_tool=rel.tolist()))
fig.suptitle('113: elliptic soft hand relative to the real wrist\nCamera fitted only to caps at8/10/14s; can rim observations are evaluation only\nRed/orange: simulated top/bottom rims. Yellow: observed rim center. Cyan: predicted caps.')
fig.savefig(R/'113_conditional_seating.png',dpi=160)
report=dict(records=rows,train_times_s=anno['train_times_s'],nominal_camera=next({k:v.tolist() if hasattr(v,'tolist') else v for k,v in c.items()} for c in fits if c['scale']==1 and c['k']==k0),annotation_sha256=hashlib.sha256((R/'113_landmarks.json').read_bytes()).hexdigest(),trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),scope='Conditional camera/cap correspondence, approximate5px landmarks; lens alternatives are sensitivity hypotheses, not confidence intervals. No can pose, hand, shelf or simulator parameter fitted. Held-out cap errors determine whether can comparisons are interpretable.')
(R/'113_conditional_seating.json').write_text(json.dumps(report,indent=2));print(json.dumps([dict(time=x['time_s'],nominal=x['nominal']) for x in rows],indent=2))
