"""Conditional image-only initial XY fit with explicit table-plane sensitivity."""
from pathlib import Path
import hashlib,json
import cv2,numpy as np
from scipy.optimize import least_squares
R=Path(__file__).resolve().parent
cameras=json.loads((R/'113_carry_seating_fit.json').read_text())
obs=json.loads((R/'113_initial_rim.json').read_text());points=np.array(obs['rim_points_px'],float)
trace=R.parent/'g14_feedback_full/113_elliptic10_soft/113_eef_delta.npz'
z=np.load(trace);m=json.loads(trace.with_suffix('.json').read_text());inv=np.linalg.inv(z['mount'])
initial=np.array(m['can_pos']);phi=np.linspace(0,2*np.pi,128,endpoint=False)
circle=np.c_[.033*np.cos(phi),.033*np.sin(phi),np.full(128,.0505)]
def project(v,c):
    v=v@inv[:3,:3].T+inv[:3,3]
    raw=cv2.projectPoints(v,c['rv'],c['tv'],c['K'],c['dist'])[0].reshape(-1,2)
    return np.c_[raw[:,1],1279-raw[:,0]]
def residual(xy,height,c,pts):
    pred=project(circle+np.r_[xy,height],c);ctr,diam,deg=cv2.fitEllipse(pred.astype(np.float32));a=np.deg2rad(deg)
    rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);u=((pts-ctr)@rot)/(np.array(diam)/2)
    return (np.linalg.norm(u,axis=1)-1)*np.mean(diam)/2
perturbed=[('nominal',points),('x-3',points+[-3,0]),('x+3',points+[3,0]),('y-3',points+[0,-3]),('y+3',points+[0,3])]
unit=points-points.mean(axis=0);unit/=np.linalg.norm(unit,axis=1)[:,None]
perturbed.extend([(f'radial{d:+g}',points+d*unit) for d in [-2,2]])
rows=[]
for f in cameras['fits']:
    c={k:np.array(v) for k,v in f['camera'].items()}
    for height in [.0905,.1005,.1105]:
        for label,pts in perturbed:
            result=least_squares(residual,initial[:2],args=(height,c,pts),diff_step=1e-3,max_nfev=200,
                xtol=1e-9,ftol=1e-9,gtol=1e-8)
            rows.append(dict(scale=f['scale'],k1=f['k1'],can_center_world_z_m=height,annotation=label,
                success=bool(result.success),rim_rms_px=float(np.sqrt(np.mean(result.fun**2))),
                estimated_initial_xy_m=result.x.tolist(),delta_from_archived_xy_m=(result.x-initial[:2]).tolist()))
nominal=next(r for r in rows if r['scale']==1 and r['k1']==cameras['nominal']['k1'] and r['can_center_world_z_m']==.1005 and r['annotation']=='nominal')
xy=np.array([r['estimated_initial_xy_m'] for r in rows])
report=dict(nominal=nominal,records=rows,all_hypotheses_xy_min_m=xy.min(axis=0).tolist(),all_hypotheses_xy_max_m=xy.max(axis=0).tolist(),
    source_initial_xyz_m=initial.tolist(),annotation_sha256=hashlib.sha256((R/'113_initial_rim.json').read_bytes()).hexdigest(),
    qualification='Conditional diagnostic only. Existing model places upright can center at100.5mm on the pick table; +/-10mm tests uncertain table-to-base alignment, not measured bounds. Camera extrinsics use only rigid caps; circle dimensions are existing assumptions. No success term, no simulator or initial-pose change. Do not adopt a correction without resolving these assumptions independently.')
(R/'113_initial_plane_sensitivity.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
