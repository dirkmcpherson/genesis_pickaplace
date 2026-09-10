"""Uncalibrated shared-actuator / return-spring surrogate for the Gen3 Lite hand.

The two proximal joints remain coupled by the URDF. Distal joints are independent.
One scalar actuator loads both fingers through constant assumed moment arms. Each
distal joint has a return spring about the original unloaded mimic relation, with
the equal-and-opposite generalized reaction included at its proximal joint.
Neither a can detector nor a task-phase switch changes the closure law.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np

REPO=Path(__file__).resolve().parents[1]

@dataclass(frozen=True)
class Parameters:
    actuator_stiffness: float=80.0
    actuator_force_limit: float=100.0
    distal_moment_ratio: float=.2
    return_stiffness: float=2.0
    proximal_damping: float=1.0
    distal_damping: float=.02

class Transmission:
    def __init__(self, parameters=Parameters()):
        self.p=parameters
        self.motor_gradient=np.array([-.5,.5,parameters.distal_moment_ratio/2,parameters.distal_moment_ratio/2])
        self.return_gradient=np.array([[-.676,0,1,0],[0,.676,0,1]])

    def terms(self,q,target):
        return float(self.motor_gradient@(q-target)),self.return_gradient@q-.149

    def torque(self,q,target):
        error,rest=self.terms(np.asarray(q),np.asarray(target))
        tension=np.clip(self.p.actuator_stiffness*error,-self.p.actuator_force_limit,self.p.actuator_force_limit)
        return -tension*self.motor_gradient-self.p.return_stiffness*(self.return_gradient.T@rest)

    def energy(self,q,target):
        error,rest=self.terms(np.asarray(q),np.asarray(target))
        k,limit=self.p.actuator_stiffness,self.p.actuator_force_limit
        threshold=limit/k;absolute=abs(error)
        motor=.5*k*error**2 if absolute<=threshold else limit*(absolute-.5*threshold)
        return motor+.5*self.p.return_stiffness*float(rest@rest)

def make_urdf(path):
    source=REPO/'gen3_lite_2f_robotiq_85.urdf'
    tree=ET.parse(source);root=tree.getroot()
    removed=[]
    for joint in root.findall('joint'):
        if joint.attrib['name'] in ('left_finger_tip_joint','right_finger_tip_joint'):
            mimic=joint.find('mimic');assert mimic is not None
            joint.remove(mimic);removed.append(joint.attrib['name'])
    assert len(removed)==2
    root.insert(0,ET.Comment('EXPERIMENTAL: distal mimic locks removed; requires adaptive_gripper_candidate.py transmission. Not a calibrated Kinova mechanism.'))
    # Resolve package meshes explicitly so the candidate may live outside repo root.
    for mesh in root.iter('mesh'):
        name=mesh.attrib['filename']
        if name.startswith('package://'):
            mesh.set('filename',str(REPO/name[len('package://'):]))
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tree.write(path,encoding='utf-8',xml_declaration=True)
    return dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                candidate=str(path.resolve()),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),removed_mimics=removed)

def install(world, parameters=Parameters()):
    """Install only on a fresh world loaded from the candidate URDF, before reset.

    Update transmission forces before every original physics substep. Scene dt,
    substeps, contact parameters and recorded commands are unchanged. Passive
    damping uses the solver's existing implicit damping support.
    """
    robot=world['kinova'];solver=world['scene'].sim.rigid_solver
    indices=np.array(world['kdofs'][-4:]);global_indices=indices+robot.dof_start
    equations=[e for e in solver.equalities if e.entity is robot]
    assert len(equations)==1 and 'left_finger_bottom' in equations[0].name,[(e.name,e.eq_data) for e in equations]
    def array(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
    model=Transmission(parameters)
    state=dict(target=np.array([-.96,.96,-.5,-.5]),substep_calls=0,max_abs_torque=np.zeros(4),max_abs_velocity=np.zeros(4))
    original_control=robot.control_dofs_position
    def control(position,dofs_idx_local=None,*args,**kwargs):
        if dofs_idx_local is not None and np.array_equal(np.asarray(dofs_idx_local),indices):
            target=array(position).reshape(4);assert np.isfinite(target).all()
            state['target']=target.copy()
            return
        return original_control(position,dofs_idx_local,*args,**kwargs)
    robot.control_dofs_position=control
    robot.set_dofs_kp(np.zeros(4),dofs_idx_local=indices)
    robot.set_dofs_kv(np.zeros(4),dofs_idx_local=indices)
    damping=np.array([parameters.proximal_damping]*2+[parameters.distal_damping]*2)
    robot.set_dofs_damping(damping,dofs_idx_local=indices)
    # The pinned rigid-only fast path bypasses substep_pre_coupling entirely.
    # Hook the actual rigid substep, shared by both simulator dispatch paths.
    original_substep=solver.substep
    def substep():
        q=array(robot.get_dofs_position(dofs_idx_local=indices)).reshape(4)
        vel=array(robot.get_dofs_velocity(dofs_idx_local=indices)).reshape(4)
        torque=model.torque(q,state['target'])
        if not np.isfinite(q).all() or not np.isfinite(torque).all() or np.max(abs(vel))>100:
            raise RuntimeError('Candidate transmission unstable; stopping without admitting a tape')
        robot.control_dofs_force(torque,dofs_idx_local=indices)
        state['substep_calls']+=1
        state['max_abs_torque']=np.maximum(state['max_abs_torque'],abs(torque))
        state['max_abs_velocity']=np.maximum(state['max_abs_velocity'],abs(vel))
        return original_substep()
    solver.substep=substep
    original_step=world['scene'].step
    def step(*args,**kwargs):
        before=state['substep_calls']
        result=original_step(*args,**kwargs)
        assert state['substep_calls']-before==world['scene'].sim.substeps, 'Transmission callback was bypassed'
        return result
    world['scene'].step=step
    kp=solver.dofs_info.kp.to_numpy()[global_indices];kv=solver.dofs_info.kv.to_numpy()[global_indices]
    actual_damping=solver.dofs_info.damping.to_numpy()[global_indices].reshape(4)
    assert np.all(kp==0) and np.all(kv==0) and np.allclose(actual_damping,damping)
    audit=dict(parameters=asdict(parameters),remaining_equalities=[e.name for e in equations],
               finger_dof_indices=indices.tolist(),kp_readback=kp.tolist(),kv_readback=kv.tolist(),
               damping_readback=actual_damping.tolist(),substep_dt_s=float(solver._substep_dt),
               status='Uncalibrated transmission candidate. No phase/contact-conditioned targets.')
    return state,audit
