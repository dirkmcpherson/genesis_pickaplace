"""Replay wrapper for localized contact law on unchanged collision geometry."""
import argparse,sys,runpy,json
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--pad-timeconst',type=float,default=.03);p.add_argument('--depth',type=float,default=.003)
p.add_argument('--return-stiffness',type=float,default=2.)
a=p.parse_args()
if a.return_stiffness not in (2.,4.,8.):raise ValueError('Outside declared spring bracket')
import adaptive_gripper_candidate as adaptive
original=adaptive.install;pad=None;rows=[]
def install(world,*args,**kwargs):
    global pad
    assert not args and not kwargs
    state,audit=original(world,parameters=adaptive.Parameters(return_stiffness=a.return_stiffness))
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
