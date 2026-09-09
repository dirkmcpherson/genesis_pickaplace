"""Early-day replay with built-world yaw/gain/shelf readback before execution."""
import json
from pathlib import Path
import runpy
import sys
import numpy as np

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
import sim_variant_hook
uid=int(sys.argv[1]);dst=ROOT/str(uid);meta=json.loads((dst/'source.json').read_text())
import sim_variants
sim_variants.VARIANTS[meta['variant']]=dict(sim_variants.VARIANTS[meta['parent_variant']],grasp_timeconst=.005,n_grasp_geoms=5)
original=sim_variant_hook.apply_post


def post(env,variant):
    result=original(env,variant)
    from replay_harness import BOX_SIZE,BOX_POS
    w=env.w;robot=w['kinova'];solver=w['scene'].sim.rigid_solver
    def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
    q=arr(robot.get_quat()).reshape(-1);yaw=float(np.rad2deg(2*np.arctan2(q[3],q[0])))
    expected={'12-16':-9.7,'12-17':-19.2}[meta['day']]
    assert abs(yaw-expected)<.01
    indices=np.asarray(w['kdofs'][:6])+robot.dof_start
    kp=solver.dofs_info.kp.to_numpy()[indices].reshape(-1)
    assert np.allclose(kp,[800,800,600,400,240,240])
    shelf=next(e for e in w['scene'].entities if e.morph.__class__.__name__=='Box' and np.allclose(e.morph.size,BOX_SIZE))
    shelf_pos=arr(shelf.get_pos()).reshape(-1)
    assert np.allclose(shelf_pos[:2],BOX_POS[:2],atol=.001)
    assert abs(shelf_pos[2]-(BOX_POS[2]+.06))<.001
    assert abs(float(arr(robot.get_pos()).reshape(-1)[2])-.08)<.001
    (dst/'world_audit.json').write_text(json.dumps(dict(uid=uid,day=meta['day'],built_yaw_deg=yaw,
        arm_kp=kp.tolist(),shelf_pos=shelf_pos.tolist(),goal_start_z=w['goal_start_z'],
        can_pos=meta['can_pos'],ic_note='already in world frame; not rotated twice',full_post_hook=True),indent=2))
    geoms=[g.idx for link in robot.links if 'finger' in link.name for g in link.geoms]+list(range(w['bottle'].geom_start,w['bottle'].geom_end))
    params=solver.geoms_info.sol_params.to_numpy()[geoms]
    assert len(geoms)==5 and np.allclose(params[:,0],.005)
    assert 2*float(solver._substep_dt)<=.005
    (dst/'contact_audit.json').write_text(json.dumps(dict(geom_indices=geoms,sol_params=params.tolist(),substep_dt_s=float(solver._substep_dt),variant=meta['variant']),indent=2))
    return result


sim_variant_hook.apply_post=post
sys.argv=['repair_eef_slide.py',str(dst/'source.npz'),'--out',str(dst/'collection'),'--max-extension','0','--polish-ik']
runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'),run_name='__main__')
