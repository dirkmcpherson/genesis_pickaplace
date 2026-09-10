"""Isolate localized pad contact on the original fixed-coupling recovery hand.

This is an attribution control, not a claim that fixed mimic coupling models
loaded Kinova finger curl. It changes no gripper mechanics or motion commands.
"""
from pathlib import Path
import argparse,sys,runpy,json,hashlib
import numpy as np
REPO=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--pad-timeconst',type=float,choices=[.02,.03],required=True)
a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
import sim_variant_hook
from surface_pad_candidate import SurfacePads
from gripper_lab import contact_stats
original=sim_variant_hook.apply_post
pad=None;rows=[];contact_rows=[];audit=None

def post(env,variant):
 global pad,audit
 result=original(env,variant);w=env.w;solver=w['scene'].sim.rigid_solver
 before=solver.geoms_info.sol_params.to_numpy().copy()
 pad=SurfacePads(w,a.pad_timeconst,.003)
 fingers={g.idx for link in w['kinova'].links if 'finger' in link.name for g in link.geoms}
 can=set(range(w['bottle'].geom_start,w['bottle'].geom_end))
 assert len(fingers)==4 and len(can)==1
 assert np.array_equal(before,solver.geoms_info.sol_params.to_numpy())
 indices=np.array(w['kdofs'][-4:])+w['kinova'].dof_start
 audit=dict(scope='Original fixed-coupling hand with localized surface pads',surface_pad=pad.audit(),
  source=str(a.source.resolve()),source_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),
  urdf_sha256=hashlib.sha256((REPO/'gen3_lite_2f_robotiq_85.urdf').read_bytes()).hexdigest(),
  kp=solver.dofs_info.kp.to_numpy()[indices].tolist(),kv=solver.dofs_info.kv.to_numpy()[indices].tolist(),
  finger_equalities=[dict(name=e.name,data=np.asarray(e.eq_data).tolist()) for e in solver.equalities if e.entity is w['kinova']],
  original_geom_parameters=before[sorted(fingers|can)].tolist(),
  qualification='Attribution control: retains original mimic coupling, which is not a validated model of passive loaded curl.')
 step=w['scene'].step
 def observed(*args,**kwargs):
  count=pad.calls;result=step(*args,**kwargs)
  assert pad.calls-count==w['scene'].sim.substeps
  rows.append(pad.counts.to_numpy().copy());contact_rows.append(contact_stats(w,can,fingers))
  return result
 w['scene'].step=observed
 return result
sim_variant_hook.apply_post=post
sys.argv=['repair_eef_slide.py',str(a.source),'--out',str(a.out),'--max-extension','0','--polish-ik']
try:
 runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'),run_name='__main__')
finally:
 if audit is not None:
  audit['detection_calls']=pad.calls
  (a.out/'pad_audit.json').write_text(json.dumps(audit,indent=2))
 if rows:np.savez_compressed(a.out/'pad_observations.npz',counts=np.asarray(rows),contact=np.asarray(contact_rows))
for path in a.out.glob('*_eef_delta.json'):
 meta=json.loads(path.read_text());meta['physics_treatment']=audit
 meta['provenance']='Original-coupling reference hand with exploratory localized pad contact; no motion repair or bank admission'
 path.write_text(json.dumps(meta,indent=2))
