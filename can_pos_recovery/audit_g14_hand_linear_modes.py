"""Local unloaded spring/damping stability of the current surrogate, not hardware calibration."""
from pathlib import Path
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
import numpy as np
from scipy.linalg import eigh
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'can_pos_recovery'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
assert not (a.out/'report.json').exists();a.out.mkdir(parents=True,exist_ok=True)
import genesis as gs
from adaptive_gripper_candidate import make_urdf,Parameters,Transmission
from align_finger_inertia import align
preset_path=R/'paper/eef_recovery_2026-09-09/localized_pads/proximal_compliance/presets/all_soft_control.json'
preset=json.loads(preset_path.read_text());parameters=Parameters(**preset['transmission']);model=Transmission(parameters)
urdf=a.out/'hand.urdf';make_urdf(urdf,distal_lower_limit=-1.03);align(urdf)
tree=ET.parse(urdf);root=tree.getroot();keep={'gripper_base_link','left_finger_prox_link','right_finger_prox_link','left_finger_dist_link','right_finger_dist_link'}
for el in list(root):
 if el.tag=='link' and el.attrib['name'] not in keep:root.remove(el)
 elif el.tag=='joint' and (el.find('parent').attrib['link'] not in keep or el.find('child').attrib['link'] not in keep):root.remove(el)
 elif el.tag not in ['link','joint']:root.remove(el)
tree.write(urdf,encoding='utf-8',xml_declaration=True)
gs.init(backend=gs.cpu,seed=0,precision='32',logging_level='warning')
scene=gs.Scene(show_viewer=False,sim_options=gs.options.SimOptions(dt=.01,substeps=8,gravity=(0,0,0)),rigid_options=gs.options.RigidOptions(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,ls_iterations=50,ls_tolerance=.01,use_contact_island=False,integrator=gs.integrator.approximate_implicitfast,enable_mujoco_compatibility=True,enable_torsional_friction=False,enable_rolling_friction=False))
robot=scene.add_entity(morph=gs.morphs.URDF(file=str(urdf.resolve()),fixed=True,collision=False,default_armature=.1));scene.build();solver=scene.sim.rigid_solver
names=['left_finger_bottom_joint','right_finger_bottom_joint','left_finger_tip_joint','right_finger_tip_joint'];idx=np.array([robot.get_joint(n).dofs_idx_local[0] for n in names]);assert solver.n_dofs==4
def arr(x):return x.detach().cpu().numpy()
D4=np.diag([parameters.proximal_damping]*2+[parameters.distal_damping]*2)
robot.set_dofs_armature(np.full(4,.0001),idx);robot.set_dofs_kp(np.zeros(4),idx);robot.set_dofs_kv(np.zeros(4),idx);robot.set_dofs_damping(np.diag(D4),idx)
np.testing.assert_allclose(arr(robot.get_dofs_armature(idx)),.0001,rtol=1e-6)
# q4=B q3 enforces the unchanged proximal opposition exactly.
B=np.array([[-1,0,0],[1,0,0],[0,1,0],[0,0,1.]])
K4=parameters.actuator_stiffness*np.outer(model.motor_gradient,model.motor_gradient)+parameters.return_stiffness*(model.return_gradient.T@model.return_gradient)
K=B.T@K4@B;D=B.T@D4@B;rows=[]
for grip in [0,45,90]:
 theta=.96-1.05*grip/100;q=np.array([-theta,theta,.149-.676*theta,.149-.676*theta])
 robot.set_dofs_position(q,idx);robot.zero_all_dofs_velocity();robot.control_dofs_force(np.zeros(4),idx);scene.step();solver.update_forward_pos()
 observed=arr(robot.get_dofs_position(idx));assert abs(observed-q).max()<1e-5
 # The current fast integrator adds h*D to the cached matrix. Remove it.
 cached=arr(solver.get_mass_mat())[np.ix_(idx,idx)].astype(float);M4=cached-solver._substep_dt*D4
 assert np.linalg.eigvalsh(M4).min()>0
 M=B.T@M4@B;omega=np.sqrt(eigh(K,M,eigvals_only=True));rates=[]
 for ss in [8,16,32,64]:
  h=.01/ss;Vq=-h*np.linalg.solve(M+h*D,K);Vv=np.linalg.solve(M+h*D,M)
  A=np.block([[np.eye(3)+h*Vq,h*Vv],[Vq,Vv]]);ev=np.linalg.eigvals(A)
  rates.append(dict(substeps=ss,substep_dt_s=h,spectral_radius=float(abs(ev).max()),fastest_mode_samples_per_period=float(2*np.pi/(omega.max()*h)),omega_h_max=float(omega.max()*h),discrete_eigenvalues=[[float(v.real),float(v.imag)] for v in ev]))
 rows.append(dict(grip=grip,observed_q=observed.tolist(),physical_mass_matrix=M4.tolist(),reduced_mass=M.tolist(),natural_angular_frequencies_rad_s=omega.tolist(),rates=rates))
report=dict(records=rows,stiffness_matrix=K4.tolist(),damping_matrix=D4.tolist(),proximal_constraint_basis=B.tolist(),preset_sha256=hashlib.sha256(preset_path.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
 assumptions='Small perturbations about unloaded targets, motor below force cap, fixed gripper base, no collisions/limits/gravity, constant local mass. Explicit spring force and implicit passive damping, matching the current hook/fast integrator locally.',
 qualification='Local linear stability/resolution diagnostic only. It excludes contact, saturation, moving targets and arm dynamics; it cannot prove the cause of full-task divergence or measured hardware stiffness/inertia. No physical parameter or task reconstruction changed.')
(a.out/'report.json').write_text(json.dumps(report,indent=2))
for row in rows:print('GRIP',row['grip'],'omega',row['natural_angular_frequencies_rad_s'],'rates',[(r['substeps'],r['spectral_radius'],r['fastest_mode_samples_per_period']) for r in row['rates']],flush=True)
