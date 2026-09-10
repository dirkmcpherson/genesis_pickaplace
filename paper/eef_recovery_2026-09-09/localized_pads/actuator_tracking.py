"""Audit tracking of the assumed actuator coordinate against recorded feedback.

This diagnoses the surrogate's own kinematic consistency, not measured hardware
joint angles or an independently calibrated motor-to-finger transformation.
"""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parent

def read(path,label):
    z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text())
    parameters=meta.get('physics_treatment',{}).get('parameters',{})
    ratio=parameters.get('distal_moment_ratio',.2)
    q=z['finger_joint'];g=z['source_grip'];j=np.array([-.5,.5,ratio/2,ratio/2])
    actual=q@j
    theta=.96-1.05*np.clip(g/100,0,1)
    target=theta+ratio*(.149-.676*theta)
    implied_percent=100*(.96-(actual-ratio*.149)/(1-.676*ratio))/1.05
    tr=z['trajectory'];counts=z['contact_counts']
    carry=(tr[:,15]>.1205)&(counts[:,1]>0)&(counts[:,0]==0)
    error=implied_percent-g
    def quantiles(x):
        return dict(median=float(np.median(x)),p95=float(np.quantile(x,.95)),maximum=float(np.max(x))) if len(x) else None
    indices=[min(len(q)-1,round(t/.03)-1) for t in [14,16,22,25]]
    return dict(name=label,trace=str(path),ratio=ratio,
        carry_abs_actuator_coordinate_error_rad=quantiles(abs(actual[carry]-target[carry])),
        carry_abs_implied_motor_error_percentage_points=quantiles(abs(error[carry])),
        checkpoints=[dict(time_s=(i+1)*.03,recorded_motor_percent=float(g[i]),implied_motor_percent=float(implied_percent[i]),
            actuator_coordinate=float(actual[i]),target_coordinate=float(target[i]),finger_q_rad=q[i].tolist()) for i in indices],
        qualification='Implied motor percentage assumes this surrogate coordinate and unloaded mapping. Large error diagnoses failure to follow its measured-position input under that hypothesis; it is not a measured hardware actuator error. Fixed-coupling reference is projected through the same coordinate for comparison.')

records=[]
for label,path in [('original_fixed',ROOT.parent/'timestamp_full_pool/233/collection/233_eef_delta.npz'),
                   ('original_adaptive',ROOT.parent/'adaptive_transmission/233_active/233_eef_delta.npz')]:
    records.append(read(path,label))
for status in sorted((ROOT/'logs').glob('233*_execution.json')):
    data=json.loads(status.read_text())
    if data['returncode']==0:
        records.append(read(ROOT/data['name']/'233_eef_delta.npz',data['name']))
out=ROOT/'actuator_tracking.json';out.write_text(json.dumps(dict(records=records,
    feedback_provenance='extract_real_timed.py reads base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position, not the gripper command topic.'),indent=2))
for r in records:
    print(r['name'],r['carry_abs_implied_motor_error_percentage_points'])
