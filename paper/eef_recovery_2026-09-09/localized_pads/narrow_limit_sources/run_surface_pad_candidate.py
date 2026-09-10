"""Replay wrapper for localized contact law on unchanged collision geometry."""
import argparse,sys,runpy,json
from pathlib import Path
import numpy as np
from dataclasses import replace
REPO=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--pad-timeconst',type=float,default=.03);p.add_argument('--depth',type=float,default=.003)
p.add_argument('--return-stiffness',type=float,default=2.)
p.add_argument('--match-reference-drive',action='store_true')
a=p.parse_args()
if a.return_stiffness not in (2.,4.,8.):raise ValueError('Outside declared spring bracket')
import adaptive_gripper_candidate as adaptive
original=adaptive.install;pad=None;rows=[]
def install(world,*args,**kwargs):
    global pad
    assert not args and not kwargs
    solver=world['scene'].sim.rigid_solver;robot=world['kinova']
    indices=np.array(world['kdofs'][-4:])+robot.dof_start
    kp=solver.dofs_info.kp.to_numpy()[indices].reshape(4)
    kv=solver.dofs_info.kv.to_numpy()[indices].reshape(4)
    parameters=adaptive.Parameters(return_stiffness=a.return_stiffness)
    modal=np.array([-1.,1.,-.676,-.676])
    gradient=adaptive.Transmission(parameters).motor_gradient
    reference_k=float(kp@(modal**2));reference_d=float(kv@(modal**2))
    if a.match_reference_drive:
        assert np.allclose(kp,40) and np.allclose(kv,10)
        motor_k=reference_k/float(gradient@modal)**2
        proximal_d=(reference_d-parameters.distal_damping*float(modal[2:]@modal[2:]))/2
        parameters=replace(parameters,actuator_stiffness=motor_k,proximal_damping=proximal_d)
    state,audit=original(world,parameters=parameters)
    audit['reference_drive']=dict(matched=a.match_reference_drive,reference_kp=kp.tolist(),reference_kv=kv.tolist(),
        coupled_joint_derivative=modal.tolist(),reference_modal_stiffness=reference_k,reference_modal_damping=reference_d,
        adaptive_modal_stiffness=parameters.actuator_stiffness*float(gradient@modal)**2,
        adaptive_modal_damping=2*parameters.proximal_damping+parameters.distal_damping*float(modal[2:]@modal[2:]),
        qualification='Matching is only along the unloaded coupled direction, below force saturation. Finite passive joints and implicit damping still differ. This matches a simulation reference, not measured hardware.')
    from surface_pad_candidate import SurfacePads
    pad=SurfacePads(world,a.pad_timeconst,a.depth)
    audit['surface_pad_treatment']=pad.audit()
    step=world['scene'].step
    def observed_step(*args,**kwargs):
        before=pad.calls;result=step(*args,**kwargs)
        assert pad.calls-before==world['scene'].sim.substeps
        rows.append(pad.counts.to_numpy().copy())
        return result
    world['scene'].step=observed_step
    return state,audit
adaptive.install=install
sys.argv=['run_adaptive_gripper_candidate.py',str(a.source),'--out',str(a.out),'--record-contact']
try:
    runpy.run_path(str(REPO/'can_pos_recovery/run_adaptive_gripper_candidate.py'),run_name='__main__')
finally:
    if pad is not None:
        (a.out/'surface_pad_audit.json').write_text(json.dumps(dict(**pad.audit(),detection_calls=pad.calls),indent=2))
    if rows:
        np.savez_compressed(a.out/'surface_pad_observations.npz',counts=np.asarray(rows),
            columns=np.array(['classified_pad_contacts','softened_contacts','backing_engaged_contacts']))
