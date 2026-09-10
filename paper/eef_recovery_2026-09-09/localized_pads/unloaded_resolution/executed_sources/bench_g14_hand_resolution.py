"""Isolate controller feedback freshness: no collision, gravity, arm, or objects."""
from pathlib import Path
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'can_pos_recovery'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--refresh-before-read',action='store_true');p.add_argument('--substeps',type=int,choices=[8,16,32],required=True);a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
import genesis as gs
from genesis.utils.misc import qd_to_numpy
from adaptive_gripper_candidate import make_urdf,Parameters,Transmission
from align_finger_inertia import align
assert gs.__version__=='1.4.0'
preset_path=R/'paper/eef_recovery_2026-09-09/localized_pads/critical_return/presets/tc0.02.json';preset=json.loads(preset_path.read_text());params=Parameters(**preset['transmission']);model=Transmission(params)
urdf=a.out/'hand.urdf';make_urdf(urdf,distal_lower_limit=-1.03);align(urdf);tree=ET.parse(urdf);root=tree.getroot();keep={'gripper_base_link','left_finger_prox_link','right_finger_prox_link','left_finger_dist_link','right_finger_dist_link'}
for el in list(root):
 if el.tag=='link' and el.attrib['name'] not in keep:root.remove(el)
 elif el.tag=='joint' and (el.find('parent').attrib['link'] not in keep or el.find('child').attrib['link'] not in keep):root.remove(el)
 elif el.tag not in ['link','joint']:root.remove(el)
tree.write(urdf,encoding='utf-8',xml_declaration=True)
gs.init(backend=gs.cpu,seed=0,precision='32',logging_level='warning')
scene=gs.Scene(show_viewer=False,sim_options=gs.options.SimOptions(dt=.01,substeps=a.substeps,gravity=(0,0,0)),rigid_options=gs.options.RigidOptions(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,ls_iterations=50,ls_tolerance=.01,use_contact_island=False,integrator=gs.integrator.approximate_implicitfast,enable_mujoco_compatibility=True,friction_cone=gs.friction_cone.pyramidal,impratio=1,contact_resolution=gs.contact_resolution.convex,enable_torsional_friction=False,enable_rolling_friction=False))
robot=scene.add_entity(morph=gs.morphs.URDF(file=str(urdf.resolve()),fixed=True,collision=False,default_armature=.1));scene.build();solver=scene.sim.rigid_solver
names=['left_finger_bottom_joint','right_finger_bottom_joint','left_finger_tip_joint','right_finger_tip_joint']
indices=np.array([robot.get_joint(n).dofs_idx_local[0] for n in names]);qi=np.array([robot.get_joint(n).q_idx_local for n in names]).reshape(4)
def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
q0=qd_to_numpy(solver.rigid_info.qpos0)[qi+robot.q_start,0]
robot.set_dofs_armature(np.full(4,.0001),indices);robot.set_dofs_kp(np.zeros(4),indices);robot.set_dofs_kv(np.zeros(4),indices);robot.set_dofs_damping(np.array([.05,.05,.002,.002]),indices)
np.testing.assert_allclose(arr(robot.get_dofs_armature(indices)),.0001,rtol=1e-6)
def target(g):
 theta=.96-1.05*g/100;return np.array([-theta,theta,.149-.676*theta,.149-.676*theta])
robot.set_dofs_position(target(0),indices);robot.set_dofs_velocity(np.zeros(4),indices)
current_target=target(0);subrows=[];step=solver.substep

def observed(f):
 cached=arr(robot.get_dofs_position(indices));physical=arr(robot.get_qpos())[qi]-q0
 if a.refresh_before_read:solver.update_forward_pos()
 q=arr(robot.get_dofs_position(indices));v=arr(robot.get_dofs_velocity(indices))
 np.testing.assert_array_equal(q,arr(robot.get_dofs_position())[indices])
 torque=model.torque(q,current_target)
 if not np.isfinite(torque).all() or abs(v).max()>100:raise RuntimeError('Transmission instability guard')
 robot.control_dofs_force(torque,indices)
 np.testing.assert_allclose(qd_to_numpy(solver.dyn_state.dofs.ctrl_force)[indices+robot.dof_start,0],torque,rtol=1e-6,atol=1e-7)
 if a.refresh_before_read:np.testing.assert_allclose(q,physical,atol=1e-7)
 subrows.append(np.r_[len(subrows)*(.01/a.substeps),cached,physical,q,v,torque]);return step(f)
solver.substep=observed
rows=[];error=None;times=np.arange(951)*.01;motors=np.interp(times,[0,.5,1.5,3.5,4,6,7.5,9.5],[0,0,60,60,90,90,0,0])
try:
 for t,g in zip(times,motors):
  current_target=target(g);scene.step();rows.append(np.r_[t,g,arr(robot.get_qpos())[qi]-q0,arr(robot.get_dofs_velocity(indices)),current_target])
except RuntimeError as e:error=str(e)
values=np.asarray(rows);sub=np.asarray(subrows);np.savez_compressed(a.out/'trace.npz',values=values,substeps=sub)
report=dict(numerical_substeps=a.substeps,substep_dt_s=.01/a.substeps,refresh_before_read=a.refresh_before_read,frames=len(rows),planned_frames=951,error=error,substep_count=len(sub),indices=indices.tolist(),q_indices=qi.tolist(),qpos0=q0.tolist(),max_cached_physical_q_difference_rad=float(abs(sub[:,1:5]-sub[:,5:9]).max()),max_used_physical_q_difference_rad=float(abs(sub[:,9:13]-sub[:,5:9]).max()),max_abs_velocity_rad_s=float(abs(sub[:,13:17]).max()),max_target_error_deg=float(np.rad2deg(abs(values[:,2:6]-values[:,10:14])).max()),preset_sha256=hashlib.sha256(preset_path.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),qualification='Only controller read freshness varies. Fixed hand, no collisions, zero gravity. Same spring law, gains, damping, armature, limits, 951-step source schedule and substep timing.')
(a.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
