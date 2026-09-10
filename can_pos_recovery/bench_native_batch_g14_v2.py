"""Full-scene native batching benchmark using frozen recorded arm/grip commands.

This implementation study is separate from the running CPU verification panel.
Normal gain1 only. Duplicated episodes test environment independence, not sample size.
"""
from pathlib import Path
import argparse,hashlib,json,os,sys,time
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'baselines'),str(R/'can_pos_recovery')]
os.environ.setdefault('QD_NUM_THREADS','1');os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np,torch,genesis as gs
from genesis.utils.misc import qd_to_numpy
from adaptive_gripper_candidate import Parameters,Transmission,make_urdf
from align_finger_inertia import align
from batched_surface_pad_candidate_g14 import BatchedSurfacePads
from sim_variant_hook import apply_pre,apply_post
import replay_harness as h
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trace',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
p.add_argument('--backend',choices=['cpu','gpu'],required=True);p.add_argument('--n-envs',type=int,required=True)
p.add_argument('--other-trace',type=Path,action='append',default=[]);p.add_argument('--steps',type=int,default=400);p.add_argument('--preset',type=Path,required=True)
a=p.parse_args();
if a.backend=='gpu':assert torch.cuda.is_available(), 'CUDA required; refuse CPU fallback'
assert not a.out.exists();a.out.mkdir();assert a.n_envs>=0
B=max(1,a.n_envs);paths=[a.trace]+a.other_trace;assert B>=len(paths)
zs=[np.load(p) for p in paths];metas=[json.loads(p.with_suffix('.json').read_text()) for p in paths]
assert len({m['variant'] for m in metas})==1, 'Batch only matching mount/collision policy groups'
assignment=[i%len(paths) for i in range(B)];z=zs[0];meta=metas[0]
preset=json.loads(a.preset.read_text());assert gs.__version__=='1.4.0'
N=min(a.steps,min(len(z['trajectory']) for z in zs));params=Parameters(**preset['transmission']);model=Transmission(params)
urdf=a.out/'gen3_lite_2f_native_batch.urdf';ui=make_urdf(urdf,distal_lower_limit=-1.03);align(urdf)
apply_pre(meta['variant']);original_build=gs.Scene.build
def batch_build(self,*args,**kwargs):kwargs['n_envs']=a.n_envs;return original_build(self,*args,**kwargs)
gs.Scene.build=batch_build
cfg=json.loads((R/'can_pos_recovery/trial_placements.json').read_text())['world']
opts=dict(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,ls_iterations=50,ls_tolerance=.01,use_contact_island=False,integrator=gs.integrator.approximate_implicitfast,enable_mujoco_compatibility=True,friction_cone=gs.friction_cone.elliptic,impratio=10,contact_resolution=gs.contact_resolution.convex,enable_torsional_friction=False,enable_rolling_friction=False)
begin=time.perf_counter()
w=h.build_world(backend=a.backend,finger_force=cfg['finger_force'],finger_kp=cfg['finger_kp'],can_height=cfg['can_height'],can_rho=cfg['can_rho'],substeps=cfg.get('substeps',1),table=cfg.get('table',False),can_radius=cfg.get('can_radius',.035),urdf_file=str(urdf.resolve()),urdf_extra={'default_armature':.1},rigid_extra=opts)
assert (gs.device.type=='cuda')==(a.backend=='gpu'), 'Unexpected execution device'
apply_post(w,meta['variant']);robot=w['kinova'];solver=w['scene'].sim.rigid_solver
sp=np.tile(np.array([.02,1,.9,.95,.001,.5,2],np.float32),(solver.n_geoms,1));solver.set_sol_params(sp)
fd=np.array(w['kdofs'][-4:]);robot.set_dofs_armature(np.full(4,.0001),fd);robot.set_dofs_kp(np.zeros(4),fd);robot.set_dofs_kv(np.zeros(4),fd)
robot.set_dofs_damping(np.array([params.proximal_damping]*2+[params.distal_damping]*2),fd)
assert len([e for e in solver.equalities if e.entity is robot])==1
pad=BatchedSurfacePads(w,preset['pad_timeconst_s'],preset['max_pad_normal_depth_m'],preset.get('compliant_part','all'))
device=gs.device
grad=torch.tensor(model.motor_gradient,dtype=torch.float64,device=device);ret=torch.tensor(model.return_gradient,dtype=torch.float64,device=device)
target=torch.tensor(h.HARDCODED_START[-4:],dtype=torch.float64,device=device).expand(B,-1).clone()
def bat(x):return x.reshape(B,-1)
def envshape(x):return x if a.n_envs else x[0]
calls=0;max_torque=torch.zeros(4,dtype=torch.float64,device=device);substep=solver.substep
# No CPU readback or synchronization in this controller hook.
def driven(*args,**kwargs):
 global calls,max_torque
 solver.update_forward_pos()
 q=bat(robot.get_dofs_position(fd)).to(torch.float64)
 error=((q-target)*grad).sum(-1);rest=q@ret.T-.149
 tension=torch.clamp(params.actuator_stiffness*error,-params.actuator_force_limit,params.actuator_force_limit)
 torque=-tension[:,None]*grad-params.return_stiffness*(rest@ret)
 max_torque=torch.maximum(max_torque,torque.abs().amax(0))
 robot.control_dofs_force(envshape(torque),fd);calls+=1;return substep(*args,**kwargs)
solver.substep=driven
robot.set_dofs_position(np.array(h.HARDCODED_START),w['kdofs']);robot.zero_all_dofs_velocity()
robot.control_dofs_position(np.array(h.HARDCODED_START[:6]),w['kdofs'][:6])
w['bottle'].set_pos(envshape(np.array([metas[j]['can_pos'] for j in assignment])));w['bottle'].set_quat(envshape(np.array([metas[j].get('can_quat',[1,0,0,0]) for j in assignment])));w['bottle'].zero_all_dofs_velocity()
w['goal'].set_pos([*h.STATIC_BOTTLE_POSITION[:2],w['goal_start_z']]);w['goal'].set_quat([1,0,0,0]);w['goal'].zero_all_dofs_velocity()
w['scene'].step();solver.update_forward_pos()
arm_np=np.stack([zs[j]['actions_joint'][:N] for j in assignment],axis=1)
grip_np=np.stack([zs[j]['source_grip'][:N] for j in assignment],axis=1)
commands=torch.tensor(arm_np,dtype=torch.float64,device=device)
fingers=torch.tensor(np.array([[h.gripper_targets(float(g)) for g in row] for row in grip_np]),dtype=torch.float64,device=device)
def sync():
 if a.backend=='gpu':torch.cuda.synchronize()
sync();build_seconds=time.perf_counter()-begin;times=[];saved=[];start=time.perf_counter();warm=None
for i in range(N):
 target=fingers[i]
 robot.control_dofs_position(envshape(commands[i]),w['kdofs'][:6])
 for _ in range(3):w['scene'].step()
 solver.update_forward_pos()
 row=torch.cat([bat(robot.get_dofs_position(w['kdofs'])),bat(w['bottle'].get_pos()),bat(w['bottle'].get_quat()),bat(w['goal'].get_pos()),bat(w['goal'].get_quat())],dim=1)
 saved.append(row.detach().cpu().numpy())
 if i==49:sync();warm=time.perf_counter()
 if (i+1)%100==0:print('PROGRESS',i+1,'envs',B,flush=True)
sync();end=time.perf_counter();arr=np.asarray(saved)
assert np.isfinite(arr).all() and calls==8+24*N
max_duplicate=0.
for j in range(len(paths)):
 ids=[i for i,k in enumerate(assignment) if k==j]
 max_duplicate=max(max_duplicate,float(np.max(abs(arr[:,ids]-arr[:,ids[:1]]))))
np.savez_compressed(a.out/'trace.npz',state=arr,arm_commands=arm_np,source_grip=grip_np,env_uids=np.array([metas[j]['uid'] for j in assignment]))
report=dict(env_uids=[metas[j]['uid'] for j in assignment],source_trace_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},backend=a.backend,n_envs=a.n_envs,actual_envs=B,steps=N,build_reset_seconds=build_seconds,replay_seconds=end-start,warm_steps=max(0,N-50),warm_seconds=end-warm if warm else None,aggregate_decisions_per_second=B*(N-50)/(end-warm) if warm and N>50 else None,max_duplicate_state_difference=max_duplicate,max_torque=max_torque.cpu().tolist(),calls=calls,pad_calls=pad.calls,pad_counts=pad.counts.to_numpy().tolist(),engine=gs.__version__,torch_version=torch.__version__,device=str(device),source_trace=str(a.trace),source_sha256=hashlib.sha256(a.trace.read_bytes()).hexdigest(),preset_sha256=hashlib.sha256(a.preset.read_bytes()).hexdigest(),candidate_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),geometry=[dict(link=l.name,geom=g.idx,vertices_sha256=hashlib.sha256(np.asarray(g._init_verts).tobytes()).hexdigest()) for l in robot.links for g in l.geoms],collision_pairs=solver.collider._valid_collision_pairs.tolist(),qualification='Benchmark only: declared heterogeneous episode assignments with duplicate controls, frozen archived joint/grip commands, normal damping1. Stops at common prefix length; not full individual episodes unless explicitly verified. GPU and batch equivalence and full-task export not yet established; no recovery claim or bank admission.')
(a.out/'report.json').write_text(json.dumps(report,indent=2));print('RESULT',json.dumps({k:v for k,v in report.items() if k not in ['geometry','collision_pairs','pad_counts']}),flush=True)
