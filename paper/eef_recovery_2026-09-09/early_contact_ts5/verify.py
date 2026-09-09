"""Independent saved-EEF replay for this isolated contact-physics probe."""
from pathlib import Path
import hashlib,json,runpy,sys
import numpy as np
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
import sim_variant_hook,sim_variants
from score_recovered_slides import adapt
uid=int(sys.argv[1]);dst=ROOT/str(uid);metadata=json.loads((dst/'source.json').read_text())
variant=metadata['variant'];sim_variants.VARIANTS[variant]=dict(sim_variants.VARIANTS[metadata['parent_variant']],grasp_timeconst=.005,n_grasp_geoms=5)
original=sim_variant_hook.apply_post

def post(env,name):
 result=original(env,name);w=env.w;robot=w['kinova'];solver=w['scene'].sim.rigid_solver
 def arr(v):return v.detach().cpu().numpy() if hasattr(v,'detach') else np.asarray(v)
 q=arr(robot.get_quat()).reshape(-1);yaw=float(np.rad2deg(2*np.arctan2(q[3],q[0])))
 assert abs(yaw-{'12-16':-9.7,'12-17':-19.2}[metadata['day']])<.01
 assert np.allclose(solver.dofs_info.kp.to_numpy()[np.asarray(w['kdofs'][:6])+robot.dof_start].reshape(-1),[800,800,600,400,240,240])
 geoms=[g.idx for link in robot.links if 'finger' in link.name for g in link.geoms]+list(range(w['bottle'].geom_start,w['bottle'].geom_end))
 params=solver.geoms_info.sol_params.to_numpy()[geoms];assert len(geoms)==5 and np.allclose(params[:,0],.005)
 assert 2*float(solver._substep_dt)<=.005
 from replay_harness import BOX_SIZE
 shelf=next(e for e in w['scene'].entities if e.morph.__class__.__name__=='Box' and np.allclose(e.morph.size,BOX_SIZE))
 assert np.allclose(arr(shelf.get_pos()).reshape(-1),[.75,-.1875,.11],atol=1e-6)
 assert abs(w['goal_start_z']-.263)<1e-6 and abs(arr(robot.get_pos()).reshape(-1)[2]-.08)<1e-6
 (dst/'verification_world_audit.json').write_text(json.dumps(dict(yaw_deg=yaw,sol_params=params.tolist(),full_post=True,wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2))
 return result

sim_variant_hook.apply_post=post
source=dst/'collection'/f'{uid}_eef_delta.npz';output=dst/'verification'
assert not output.exists(),f'Inspect previous or partial verification: {output}'
sys.argv=['repair_eef_slide.py',str(source),'--out',str(output),'--max-extension','0','--verify-actions']
runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'),run_name='__main__')
with np.load(source) as a,np.load(output/source.name) as b:
 comparison={k:bool(np.array_equal(a[k],b[k])) for k in ['actions_eef','trajectory','observations','finger_joint','contact_counts']}
assert all(comparison.values()),comparison
metric=adapt(output/source.name,output/'metric_adapter')['metric'];expected=json.loads((dst/'result.json').read_text())['score']['metric'];assert metric==expected
(dst/'verification_execution.json').write_text(json.dumps(dict(uid=uid,verified=True,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),array_comparison=comparison,metric=metric),indent=2))
print('VERIFIED',uid,flush=True)
