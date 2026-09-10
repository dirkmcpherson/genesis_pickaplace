"""Fit a conditional fixed grasp to several real rim views; never fit recovery."""
from pathlib import Path
import hashlib, json, sys
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[3]
sys.path.insert(0, str(REPO / 'baselines'))
from eef_delta_control import ArmKinematics, transform

previous = json.loads((ROOT / '113_conditional_seating.json').read_text())
annotations = json.loads((ROOT / '113_carry_rims.json').read_text())
caps = json.loads((ROOT / '113_landmarks.json').read_text())
tracepath = ROOT.parent / 'g14_feedback_full/113_elliptic10_soft/113_eef_delta.npz'
meta = json.loads(tracepath.with_suffix('.json').read_text())
source = np.load(meta['timing_reconstruction']['source'])
fk = ArmKinematics(REPO / 'gen3_lite_2f_robotiq_85.urdf')
def tool(t):
    q = np.array([np.interp(t, source['t_frame'], source['q_frame'][:,j]) for j in range(6)])
    return fk.tool(q, np.eye(4))
def apply(p, x):
    return x @ p[:3,:3].T + p[:3,3]
records = []
for obs in annotations['records']:
    old = next(r for r in previous['records'] if r['time_s'] == obs['time_s'])
    records.append(dict(**obs, pose=tool(old['actual_camera_time_s']), previous=old))
training = [r for r in records if r['time_s'] in annotations['train_times_s']]
captraining = [r for r in records if r['time_s'] in caps['train_times_s']]
tool_from_gripper = transform(np.array([0,0,.13]), Rotation.from_euler('z',np.pi/2).as_matrix())
localcaps = np.array([[.0152,-.0305,.070003],[.0152,.0305,.070003]])
capxyz = np.concatenate([apply(r['pose'] @ np.linalg.inv(tool_from_gripper), localcaps) for r in captraining])
capuv = np.concatenate([r['previous']['caps'] for r in captraining])
capuv = np.ascontiguousarray(np.c_[1279-capuv[:,1],capuv[:,0]],dtype=float)
nominalcamera = previous['nominal_camera']
baseK = np.array(nominalcamera['K']); kold = nominalcamera['k']
phi = np.linspace(0,2*np.pi,128,endpoint=False)
rim = np.c_[.033*np.cos(phi),.033*np.sin(phi),np.full(128,.0505)]
# A can is axially symmetric; only its axis and center are identifiable here.
reference_rotation = training[0]['pose'][:3,:3].T
def relative(x):
    return transform(x[:3], Rotation.from_rotvec([x[3],x[4],0]).as_matrix() @ reference_rotation)
def project(x,c):
    p = cv2.projectPoints(x,c['rv'],c['tv'],c['K'],c['dist'])[0].reshape(-1,2)
    return np.c_[p[:,1],1279-p[:,0]]
def ellipse_error(pred,obs):
    center,diam,deg = cv2.fitEllipse(pred.astype(np.float32))
    a = np.deg2rad(deg); rot = np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    unit = ((obs-center) @ rot)/(np.array(diam)/2)
    return (np.linalg.norm(unit,axis=1)-1)*np.mean(diam)/2
def residual(x,c,rows,perturb=(0,0)):
    return np.concatenate([ellipse_error(project(apply(r['pose']@relative(x),rim),c),np.array(r['rim_points_px'])+perturb) for r in rows])

fits=[]; sensitivity=[]
initial = np.r_[np.array(training[0]['previous']['sim_can_from_tool'])[:3,3],0,0]
for scale in [.8,.9,1,1.1,1.2]:
    for k in [-.25,kold,0.]:
        K=baseK.copy(); K[0,0]*=scale; K[1,1]*=scale
        dist=np.array([k,0,0,0,0.])
        ok,rv,tv=cv2.solvePnP(capxyz,capuv,K,dist,flags=cv2.SOLVEPNP_SQPNP); assert ok
        rv,tv=cv2.solvePnPRefineLM(capxyz,capuv,K,dist,rv,tv)
        c=dict(K=K,dist=dist,rv=rv,tv=tv)
        # Multiple orientation starts expose the planar circle's pose ambiguity.
        starts=[]
        for pitch in [-.3,0,.3]:
            start=initial.copy();start[4]=pitch
            fit=least_squares(residual,start,args=(c,training),diff_step=1e-3,
                max_nfev=250,xtol=1e-9,ftol=1e-9,gtol=1e-8,
                bounds=([-.2,-.2,-.15,-1.3,-1.3],[.2,.2,.2,1.3,1.3]))
            starts.append(dict(x=fit.x.tolist(),success=bool(fit.success),rms_px=float(np.sqrt(np.mean(fit.fun**2)))))
        best=min(starts,key=lambda r:r['rms_px']); x=np.array(best['x'])
        poses=[]
        for r in records:
            sim=np.array(r['previous']['sim_can_from_tool'])
            err=residual(x,c,[r]);center=np.array(x[:3])
            poses.append(dict(time_s=r['time_s'],role='fit' if r['time_s'] in annotations['train_times_s'] else 'evaluation',
                rim_rms_px=float(np.sqrt(np.mean(err**2))),
                real_minus_sim_center_tool_m=(center-sim[:3,3]).tolist(),
                axis_difference_deg=float(np.rad2deg(np.arccos(np.clip(relative(x)[:3,2]@sim[:3,2],-1,1))))))
        fits.append(dict(scale=scale,k1=k,fit=best,multistarts=starts,records=poses,
            can_from_tool=relative(x).tolist(),camera={key:value.tolist() for key,value in c.items()}))
        for label,shift,radial in [('x-3',[-3,0],0),('x+3',[3,0],0),
                                  ('y-3',[0,-3],0),('y+3',[0,3],0),
                                  ('radial-2',[0,0],-2),('radial+2',[0,0],2)]:
            perturbed=[]
            for r in training:
                pts=np.array(r['rim_points_px'],float); direction=pts-pts.mean(axis=0)
                pts=pts+shift+radial*direction/np.linalg.norm(direction,axis=1)[:,None]
                perturbed.append(dict(r,rim_points_px=pts.tolist()))
            fit=least_squares(residual,x,args=(c,perturbed),diff_step=1e-3,
                max_nfev=200,xtol=1e-9,ftol=1e-9,gtol=1e-8,
                bounds=([-.2,-.2,-.15,-1.3,-1.3],[.2,.2,.2,1.3,1.3]))
            sensitivity.append(dict(scale=scale,k1=k,perturbation=label,
                success=bool(fit.success),rms_px=float(np.sqrt(np.mean(fit.fun**2))),
                can_center_tool_m=fit.x[:3].tolist()))
        print('FIT',scale,k,best['rms_px'],flush=True)

nominal=next(f for f in fits if f['scale']==1 and f['k1']==kold)
# Preserve lens alternatives; a lowest residual does not establish true intrinsics.
centers=np.array([np.array(f['can_from_tool'])[:3,3] for f in fits])
allcenters=np.vstack([centers,np.array([v['can_center_tool_m'] for v in sensitivity])])
report=dict(nominal=nominal,fits=fits,center_tool_sensitivity_min_m=centers.min(axis=0).tolist(),
    center_tool_sensitivity_max_m=centers.max(axis=0).tolist(),
    annotation_sensitivity=sensitivity,
    combined_sensitivity_min_m=allcenters.min(axis=0).tolist(),
    combined_sensitivity_max_m=allcenters.max(axis=0).tolist(),
    annotation_sha256=hashlib.sha256((ROOT/'113_carry_rims.json').read_bytes()).hexdigest(),
    trace_sha256=hashlib.sha256(tracepath.read_bytes()).hexdigest(),
    qualification='Conditional fixed can-to-tool pose, fit only to real rims at10/12/14s. Existing cap-only camera hypotheses. Three orientation starts per lens. Lens spread is sensitivity, not a confidence interval. Manual rim/CAD/camera assumptions and relative can motion remain possible errors. No simulator or initial pose changed.')
(ROOT/'113_carry_seating_fit.json').write_text(json.dumps(report,indent=2))
fig,axes=plt.subplots(2,3,figsize=(12,9),layout='constrained')
c={k:np.array(v) for k,v in nominal['camera'].items()}
for ax,r in zip(axes.ravel(),records):
    im=cv2.cvtColor(cv2.imread(str(ROOT/f'113_{r["time_s"]:g}_cam4.jpg')),cv2.COLOR_BGR2RGB)
    ax.imshow(im);pts=np.array(r['rim_points_px']);ctr=pts.mean(axis=0)
    pred=project(apply(r['pose']@np.array(nominal['can_from_tool']),rim),c)
    sim=project(apply(r['pose']@np.array(r['previous']['sim_can_from_tool']),rim),c)
    ax.plot(*np.vstack([pred,pred[:1]]).T,color='lime',label='Real fitted fixed grasp')
    ax.plot(*np.vstack([sim,sim[:1]]).T,color='red',label='Simulation')
    ax.plot(pts[:,0],pts[:,1],'yo',ms=2)
    role='fit' if r['time_s'] in annotations['train_times_s'] else 'evaluation'
    ax.set(xlim=(ctr[0]-80,ctr[0]+110),ylim=(ctr[1]+150,ctr[1]-70),title=f'{r["time_s"]:g}s ({role})');ax.axis('off')
axes[0,0].legend(fontsize=7)
fig.suptitle('113: conditional fixed seating fit from real can rims\nGreen: one can-to-tool pose fitted at10/12/14s; red: saved simulation; yellow: manual rim points')
fig.savefig(ROOT/'113_carry_seating_fit.png',dpi=150)
print('SUMMARY',json.dumps({k:v for k,v in report.items() if k not in ['fits','nominal','annotation_sensitivity']}),flush=True)
