"""Isolated integration-resolution control derived from run_g14_hand.py; physical parameters fixed."""
from pathlib import Path
import argparse, hashlib, json, os, runpy, sys
import numpy as np
REPO=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
os.environ.setdefault('QD_NUM_THREADS','1');os.environ.setdefault('OMP_NUM_THREADS','1')
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--preset',type=Path,required=True)
p.add_argument('--cone',choices=['pyramidal','elliptic'],required=True)
p.add_argument('--impratio',type=float,required=True)
p.add_argument('--substeps',type=int,choices=[8,16,32],required=True)
p.add_argument('--verify-actions',action='store_true')
p.add_argument('--audit-only',action='store_true',help='Build and verify mechanics, then exit before reset/stepping.')
a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
import genesis as gs
assert gs.__version__=='1.4.0'
from genesis.utils.misc import qd_to_numpy
from genesis.utils.geom import quat_to_R
from adaptive_gripper_candidate import Parameters,Transmission,make_urdf
from align_finger_inertia import align,AUDIT
from surface_pad_candidate_g14 import SurfacePads
import genesis_can_env,sim_variant_hook
preset=json.loads(a.preset.read_text());parameters=Parameters(**preset['transmission'])
urdf=a.out/'gen3_lite_2f_g14_candidate.urdf'
urdf_info=make_urdf(urdf,distal_lower_limit=preset['distal_lower_limit_rad'])
urdf_info['finger_inertia_alignment']=align(urdf)
urdf_info['candidate_sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
opts=dict(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,
          ls_iterations=50,ls_tolerance=.01,use_contact_island=False,
          integrator=gs.integrator.approximate_implicitfast,enable_mujoco_compatibility=True,
          friction_cone=getattr(gs.friction_cone,a.cone),impratio=a.impratio,
          contact_resolution=gs.contact_resolution.convex,
          enable_torsional_friction=False,enable_rolling_friction=False)
def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
build=genesis_can_env.build_world
def candidate_world(*args,**kwargs):
    kwargs.update(urdf_file=str(urdf.resolve()),urdf_extra={'default_armature':.1},rigid_extra=opts,substeps=a.substeps)
    w=build(*args,**kwargs);solver=w['scene'].sim.rigid_solver
    assert solver.substeps==a.substeps and np.isclose(solver._substep_dt,.01/a.substeps)
    assert np.isclose(w['scene'].sim._dt,.01)
    params=np.tile(np.array([.02,1,.9,.95,.001,.5,2],dtype=np.float32),(solver.n_geoms,1))
    solver.set_sol_params(params);np.testing.assert_array_equal(arr(solver.get_sol_params()),params)
    return w
genesis_can_env.build_world=candidate_world
post=sim_variant_hook.apply_post
state=None;pad=None;audit=None;pad_rows=[];pad_geom_rows=[]
def candidate_post(env,variant):
    global state,pad,audit
    result=post(env,variant);w=env.w;robot=w['kinova'];solver=w['scene'].sim.rigid_solver
    indices=np.array(w['kdofs'][-4:]);global_indices=indices+robot.dof_start
    equations=[e for e in solver.equalities if e.entity is robot]
    assert len(equations)==1 and 'left_finger_bottom' in equations[0].name
    robot.set_dofs_armature(np.full(4,.0001),dofs_idx_local=indices)
    np.testing.assert_allclose(arr(robot.get_dofs_armature(w['kdofs'])),[.1]*6+[.0001]*4,rtol=1e-6)
    limits=robot.get_dofs_limit(indices)
    # Public getter returns lower and upper arrays.
    lim=np.stack([arr(v) for v in limits],axis=-1)
    np.testing.assert_allclose(lim[2:],[[-1.03,.21]]*2,atol=1e-6)
    records=[]
    for row in json.loads(AUDIT.read_text())['records']:
        idx=robot.get_link(row['link']).idx
        pos=qd_to_numpy(solver.dyn_info.links.inertial_pos)[idx]
        inertia=qd_to_numpy(solver.dyn_info.links.inertial_i)[idx]
        inertial_quat=qd_to_numpy(solver.dyn_info.links.inertial_quat)[idx]
        # 1.4 stores the principal tensor and its orientation separately.
        # Compare in the authored link frame, as the pinned-engine audit does.
        rotation=quat_to_R(inertial_quat)
        link_inertia=rotation@inertia@rotation.T
        mass=float(qd_to_numpy(solver.dyn_info.links.inertial_mass)[idx])
        np.testing.assert_allclose(pos,row['proposed_com'],atol=1e-8)
        np.testing.assert_allclose(link_inertia,row['proposed_inertia'],atol=1e-11)
        np.testing.assert_allclose(mass,row['mass_kg'],rtol=1e-6)
        records.append(dict(link=row['link'],com=pos.tolist(),inertia=link_inertia.tolist(),
                            inertial_frame_tensor=inertia.tolist(),
                            inertial_quat_wxyz=inertial_quat.tolist(),mass_kg=mass))
    model=Transmission(parameters)
    state=dict(target=np.array([-.96,.96,-.5,-.5]),substep_calls=0,max_abs_torque=np.zeros(4),max_abs_velocity=np.zeros(4),
               max_cached_feedback_difference_rad=0.,max_used_feedback_difference_rad=0.)
    qindices=np.concatenate([np.asarray(robot.get_joint(n).qs_idx_local).reshape(-1) for n in
        ['left_finger_bottom_joint','right_finger_bottom_joint','left_finger_tip_joint','right_finger_tip_joint']])
    qzero=qd_to_numpy(solver.rigid_info.qpos0)[qindices+robot.q_start,0]
    original_control=robot.control_dofs_position
    def control(position,dofs_idx_local=None,*args,**kwargs):
        if dofs_idx_local is not None and np.array_equal(np.asarray(dofs_idx_local),indices):
            state['target']=arr(position).reshape(4).copy();return
        return original_control(position,dofs_idx_local,*args,**kwargs)
    robot.control_dofs_position=control
    robot.set_dofs_kp(np.zeros(4),indices);robot.set_dofs_kv(np.zeros(4),indices)
    damping=np.array([parameters.proximal_damping]*2+[parameters.distal_damping]*2)
    robot.set_dofs_damping(damping,indices)
    np.testing.assert_allclose(arr(solver.get_dofs_damping(global_indices)),damping,rtol=1e-6)
    assert np.all(arr(solver.get_dofs_kp(global_indices))==0) and np.all(arr(solver.get_dofs_kv(global_indices))==0)
    substep=solver.substep
    def driven(*args,**kwargs):
        cached=arr(robot.get_dofs_position(indices)).reshape(4)
        # MuJoCo-compatible stepping leaves the kinematic cache at the preceding
        # substep. The external spring law must use the integrated current pose.
        solver.update_forward_pos()
        q=arr(robot.get_dofs_position(indices)).reshape(4);v=arr(robot.get_dofs_velocity(indices)).reshape(4)
        physical=arr(robot.get_qpos())[qindices]-qzero
        difference=float(np.max(abs(q-physical)))
        if difference>1e-7:raise RuntimeError('Controller feedback differs from integrated joint position')
        state['max_cached_feedback_difference_rad']=max(state['max_cached_feedback_difference_rad'],float(np.max(abs(cached-physical))))
        state['max_used_feedback_difference_rad']=max(state['max_used_feedback_difference_rad'],difference)
        torque=model.torque(q,state['target'])
        if not np.isfinite(torque).all() or np.max(abs(v))>100:raise RuntimeError('Transmission instability guard')
        robot.control_dofs_force(torque,indices);state['substep_calls']+=1
        state['max_abs_torque']=np.maximum(state['max_abs_torque'],abs(torque))
        state['max_abs_velocity']=np.maximum(state['max_abs_velocity'],abs(v))
        return substep(*args,**kwargs)
    solver.substep=driven
    pad=SurfacePads(w,preset['pad_timeconst_s'],preset['max_pad_normal_depth_m'],
        compliant_part=preset.get('compliant_part','all'))
    step=w['scene'].step
    def observed(*args,**kwargs):
        before=state['substep_calls'];pc=pad.calls;result=step(*args,**kwargs)
        # Downstream saved poses and task predicates also read the current state.
        solver.update_forward_pos()
        assert state['substep_calls']-before==a.substeps and pad.calls-pc==a.substeps
        pad_rows.append(pad.counts.to_numpy().copy())
        pad_geom_rows.append(pad.activation_by_geom.to_numpy().copy());return result
    w['scene'].step=observed
    audit=dict(integration_resolution=dict(substeps=a.substeps,substep_dt_s=float(solver._substep_dt),scene_dt_s=.01,decision_dt_s=.03,qualification='Integration refinement only; same physical hand/contact laws evaluated each numerical substep. Original source duration and commands.'),engine_version=gs.__version__,engine_options={k:str(v) for k,v in opts.items()},
        collision_policy=dict(enable_self_collision=solver._enable_self_collision,
            enable_neutral_collision=solver._enable_neutral_collision,
            valid_geom_pairs=solver.collider._valid_collision_pairs.tolist()),
        feedback='Refresh forward positions before every external spring-control read and after every scene step; assert readback against integrated qpos.',
        declared_preset=dict(configuration=preset,path=str(a.preset),sha256=hashlib.sha256(a.preset.read_bytes()).hexdigest()),
        urdf=urdf_info,surface_pad_treatment=pad.audit(),finger_inertial_readback=records,
        armature_readback=arr(robot.get_dofs_armature(w['kdofs'])).tolist(),limits_readback=lim.tolist(),
        normal_parameters=arr(solver.get_sol_params()).tolist(),
        geometry=[dict(link=link.name,geom=g.idx,vertices=len(g._init_verts),
                      vertices_sha256=hashlib.sha256(np.asarray(g._init_verts).tobytes()).hexdigest()) for link in robot.links for g in link.geoms],
        can_mass_kg=float(w['bottle'].get_mass()),goal_mass_kg=float(w['goal'].get_mass()),
        qualification='Experimental engine and contact-formulation comparison. Analytic primitive masses differ from pinned mesh-derived masses. No full-task or hardware-fidelity claim before scoring and visual checks.')
    (a.out/'transmission_audit.json').write_text(json.dumps(audit,indent=2))
    if a.audit_only:
        print('AUDIT_ONLY_COMPLETE',flush=True)
        raise SystemExit(0)
    return result
sim_variant_hook.apply_post=candidate_post
sys.argv=['repair_eef_slide.py',str(a.source),'--out',str(a.out),'--max-extension','0','--polish-ik']
if a.verify_actions:sys.argv.append('--verify-actions')
try:runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'),run_name='__main__')
finally:
    if state is not None:(a.out/'transmission_stats.json').write_text(json.dumps({k:v.tolist() if hasattr(v,'tolist') else v for k,v in state.items()},indent=2))
    if pad_rows:np.savez_compressed(a.out/'surface_pad_observations.npz',counts=np.asarray(pad_rows),activation_by_geom=np.asarray(pad_geom_rows))
for path in a.out.glob('*_eef_delta.json'):
    meta=json.loads(path.read_text());meta['physics_treatment']=audit;meta['provenance']='Isolated Genesis1.4 adaptive-hand candidate; not admitted';path.write_text(json.dumps(meta,indent=2))
