"""Vision-only initial circle fit, conditional on the independently fitted cap camera."""
from pathlib import Path
import json,sys,hashlib
import cv2,numpy as np
from scipy.optimize import least_squares
R=Path(__file__).resolve().parent
# Reuse the declared cap-only camera sensitivity set. It fits no can pose.
exec(compile((R/'project113.py').read_text(),str(R/'project113.py'),'exec'))
obs=json.loads((R/'113_initial_rim.json').read_text());points=np.array(obs['rim_points_px'],float);mount=z['mount'];inv=np.linalg.inv(mount);initial_world=np.array(meta['can_pos']);initial_world[2]=.1005;initial=apply(inv,initial_world[None,:])[0]
angle=np.linspace(0,2*np.pi,128,endpoint=False);circle=np.c_[.033*np.cos(angle),.033*np.sin(angle),np.full(128,.0505)]
def residual(center,c,observed):
 uv=project(circle+center,c);cen,axes,deg=cv2.fitEllipse(uv.astype(np.float32));a=np.deg2rad(deg);rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);delta=(observed-np.array(cen))@rot;norm=np.sqrt(np.sum((delta/(np.array(axes)/2))**2,axis=1));return (norm-1)*np.mean(axes)/2
records=[]
perturbations=[('nominal',points),('x-3',points+[-3,0]),('x+3',points+[3,0]),('y-3',points+[0,-3]),('y+3',points+[0,3])]
center=points.mean(axis=0)
for radius_delta in [-2,2]:
 radial=points-center;radial=radial/np.linalg.norm(radial,axis=1)[:,None];perturbations.append((f'radial{radius_delta:+g}',points+radius_delta*radial))
for c in fits:
 for name,observed in perturbations:
  result=least_squares(residual,initial,args=(c,observed),diff_step=1e-3,xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=300)
  world=apply(mount,result.x[None,:])[0];records.append(dict(focal_multiplier=c['scale'],k1=c['k'],annotation_perturbation=name,fit_success=bool(result.success),rim_rms_px=float(np.sqrt(np.mean(residual(result.x,c,observed)**2))),can_center_arm_m=result.x.tolist(),can_center_world_m=world.tolist(),difference_from_archived_settled_world_m=(world-initial_world).tolist()))
nominal=next(v for v in records if v['focal_multiplier']==1 and v['k1']==k0 and v['annotation_perturbation']=='nominal');positions=np.array([v['can_center_world_m'] for v in records]);out=dict(nominal=nominal,records=records,all_hypotheses_world_min_m=positions.min(axis=0).tolist(),all_hypotheses_world_max_m=positions.max(axis=0).tolist(),archived_initial_xyz_m=meta['can_pos'],archived_settled_xyz_m=initial_world.tolist(),rim_annotation_sha256=hashlib.sha256((R/'113_initial_rim.json').read_bytes()).hexdigest(),qualification='Vision-only circle fit under cap correspondence, upright can, known radius and camera hypotheses. Ranges are sensitivity scenarios, not statistical confidence bounds. World coordinates inherit the existing mount transform. No can, shelf or simulator change made; no recovered-success term in objective.')
(R/'113_initial_can_fit.json').write_text(json.dumps(out,indent=2));print('INITIAL_CAN_FIT',json.dumps({k:v for k,v in out.items() if k!='records'}))
