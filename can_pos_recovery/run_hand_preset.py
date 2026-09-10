"""Replay a declared hand-mechanics preset with localized surface-pad contact."""
import argparse,sys,runpy,json,hashlib
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--preset',type=Path,required=True)
p.add_argument('--verify-actions',action='store_true')
a=p.parse_args();preset=json.loads(a.preset.read_text())
import adaptive_gripper_candidate as adaptive
parameters=adaptive.Parameters(**preset['transmission'])
assert parameters.actuator_stiffness>0 and parameters.actuator_force_limit>0
assert 0<parameters.distal_moment_ratio<1/.676 and parameters.return_stiffness>0
assert parameters.proximal_damping>=0 and parameters.distal_damping>=0
original=adaptive.install;pad=None;rows=[]
def install(world):
    global pad
    state,audit=original(world,parameters=parameters)
    audit['declared_preset']=dict(path=str(a.preset.resolve()),sha256=hashlib.sha256(a.preset.read_bytes()).hexdigest(),configuration=preset)
    robot=world['kinova'];solver=world['scene'].sim.rigid_solver
    local=np.asarray(world['kdofs'][-4:]);global_indices=local+robot.dof_start
    before=solver.dofs_info.armature.to_numpy().copy()
    if 'finger_armature_kg_m2' in preset:
        requested=np.asarray(preset['finger_armature_kg_m2'],dtype=float)
        assert requested.shape==(4,) and np.all(requested>=0)
        robot.set_dofs_armature(requested,local)
        actual=solver.dofs_info.armature.to_numpy()
        assert np.allclose(actual[global_indices],requested,rtol=1e-6,atol=1e-12)
        other=np.ones(len(before),dtype=bool);other[global_indices]=False
        assert np.array_equal(actual[other],before[other])
    audit['finger_armature']=dict(before_kg_m2=before[global_indices].tolist(),
        after_kg_m2=solver.dofs_info.armature.to_numpy()[global_indices].tolist(),
        qualification='Added joint inertia, separate from URDF link inertia. A small nonzero value is numerical regularization, not measured actuator inertia.')
    from surface_pad_candidate import SurfacePads
    pad=SurfacePads(world,preset['pad_timeconst_s'],preset['max_pad_normal_depth_m'])
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
sys.argv=['run_adaptive_gripper_candidate.py',str(a.source),'--out',str(a.out),'--record-contact',
          '--distal-lower-limit',str(preset['distal_lower_limit_rad'])]
if a.verify_actions:sys.argv.append('--verify-actions')
try:
    runpy.run_path(str(REPO/'can_pos_recovery/run_adaptive_gripper_candidate.py'),run_name='__main__')
finally:
    if pad is not None:
        (a.out/'surface_pad_audit.json').write_text(json.dumps(dict(**pad.audit(),detection_calls=pad.calls),indent=2))
    if rows:
        np.savez_compressed(a.out/'surface_pad_observations.npz',counts=np.asarray(rows),
            columns=np.array(['classified_pad_contacts','softened_contacts','backing_engaged_contacts']))
