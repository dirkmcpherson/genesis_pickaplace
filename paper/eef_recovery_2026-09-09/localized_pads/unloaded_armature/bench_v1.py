"""Zero-contact, zero-gravity hand bench for default armature attribution.

Not a demo or a material calibration. All treatments use identical declared
transmission, source ramp and URDF link inertias; only added joint inertia varies.
"""
from pathlib import Path
import sys,os,argparse,json,xml.etree.ElementTree as ET,hashlib
import numpy as np
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO/'can_pos_recovery'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--armature',choices=['default','distal_zero','all_1e-4','all_zero'],required=True)
a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
os.environ['TI_CPU_MAX_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
import genesis as gs
from adaptive_gripper_candidate import make_urdf,install,Parameters
preset=json.loads((REPO/'paper/eef_recovery_2026-09-09/localized_pads/fast_return_presets/tc0.03.json').read_text())
urdf=a.out/'hand.urdf';info=make_urdf(urdf,distal_lower_limit=-1.03);tree=ET.parse(urdf);root=tree.getroot();keep={'gripper_base_link','left_finger_prox_link','right_finger_prox_link','left_finger_dist_link','right_finger_dist_link'}
for el in list(root):
 if el.tag=='link' and el.attrib['name'] not in keep:root.remove(el)
 elif el.tag=='joint' and (el.find('parent').attrib['link'] not in keep or el.find('child').attrib['link'] not in keep):root.remove(el)
 elif el.tag not in ['link','joint']:root.remove(el)
tree.write(urdf,encoding='utf-8',xml_declaration=True)
gs.init(backend=gs.cpu,seed=0,precision='32',logging_level='warning')
scene=gs.Scene(show_viewer=False,sim_options=gs.options.SimOptions(dt=.01,substeps=8,gravity=(0,0,0)))
robot=scene.add_entity(morph=gs.morphs.URDF(file=str(urdf.resolve()),fixed=True,collision=False))
scene.build();names=['left_finger_bottom_joint','right_finger_bottom_joint','left_finger_tip_joint','right_finger_tip_joint'];indices=[robot.get_joint(n).dof_idx_local for n in names]
# Scalar dof_idx_local in this pinned version.
indices=np.asarray(indices).reshape(4);w=dict(scene=scene,kinova=robot,kdofs=indices)
state,audit=install(w,Parameters(**preset['transmission']))
def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
before=arr(robot.get_dofs_armature(indices));after=before.copy()
assert np.allclose(before,.1)
if a.armature=='distal_zero':after[2:]=0
elif a.armature=='all_1e-4':after[:]=.0001
elif a.armature=='all_zero':after[:]=0
robot.set_dofs_armature(after,indices);assert np.array_equal(arr(robot.get_dofs_armature(indices)),after)
def target(g):
 theta=.96-1.05*g/100;return np.array([-theta,theta,.149-.676*theta,.149-.676*theta])
robot.set_dofs_position(target(0),indices);robot.set_dofs_velocity(np.zeros(4),indices)
times=np.arange(951)*.01;motors=np.interp(times,[0,.5,1.5,3.5,4,6,7.5,9.5],[0,0,60,60,90,90,0,0]);rows=[];error=None
try:
 for t,g in zip(times,motors):
  robot.control_dofs_position(target(g),indices);scene.step();q=arr(robot.get_dofs_position(indices));v=arr(robot.get_dofs_velocity(indices));rows.append(np.r_[t,g,q,v,target(g)])
except RuntimeError as e:error=str(e)
values=np.asarray(rows);np.savez_compressed(a.out/'trace.npz',values=values,columns=np.array(['time','motor','qL','qR','qLT','qRT','vL','vR','vLT','vRT','targetL','targetR','targetLT','targetRT']))
M=scene.sim.rigid_solver.mass_mat.to_numpy()[:,:,0];global_indices=indices+robot.dof_start
mass=M[np.ix_(global_indices,global_indices)]
report=dict(treatment=a.armature,armature_before=before.tolist(),armature_after=after.tolist(),mass_matrix_at_end=mass.tolist(),frames=len(rows),planned_frames=len(times),error=error,transmission=audit,stats={k:v.tolist() if hasattr(v,'tolist') else v for k,v in state.items()},urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),bench='Hand only, fixed base, collisions disabled, zero gravity, original link inertia. Not a demo or proof of hardware actuator inertia.')
if len(rows):
 report['max_abs_target_error_rad']=np.max(abs(values[:,2:6]-values[:,10:14]),axis=0).tolist()
 report['checkpoints']=[dict(time_s=float(values[i,0]),motor=float(values[i,1]),q=values[i,2:6].tolist(),target_error=(values[i,2:6]-values[i,10:14]).tolist()) for t in [1.5,3.5,4,6,7.5,9.5] if (i:=round(t/.01))<len(rows)]
(a.out/'report.json').write_text(json.dumps(report,indent=2));print('RESULT',a.armature,'frames',len(rows),'error',error,flush=True)
