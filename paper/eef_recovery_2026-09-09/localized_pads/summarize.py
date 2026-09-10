"""Read only successful terminal pad experiments; keep strict and supplied scores."""
from pathlib import Path
import json, sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt

def quantiles(x,scale=1):
    return dict(median=float(np.median(x)*scale),p95=float(np.quantile(x,.95)*scale),maximum=float(np.max(x)*scale)) if len(x) else None

records=[]
for status_path in sorted((ROOT/'logs').glob('*_execution.json')):
    status=json.loads(status_path.read_text());folder=ROOT/status['name']
    if status['returncode']:
        records.append(dict(name=status['name'],error='execution failed',status=status));continue
    path=next(folder.glob('*_eef_delta.npz'));uid=int(path.name.split('_')[0])
    z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());n=len(z['actions_eef'])
    reference=ROOT.parent/f'adaptive_transmission/{uid}_active/{uid}_eef_delta.npz'
    reference_is_adaptive=reference.exists()
    if not reference.exists():
        reference=ROOT.parent/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/collection/{uid}_eef_delta.npz'
    previous=np.load(reference)
    identity={key:bool(np.array_equal(z[key],previous[key])) for key in ['actions_eef','actions_joint','source_grip','mount']}
    assert all(identity.values()),identity
    stats=json.loads((folder/'transmission_stats.json').read_text())
    assert stats['substep_calls']==8+24*n
    contact=np.load(folder/'contact_observations.npz')['values'];assert contact.shape==(1+3*n,3)
    tr=z['trajectory'];counts=z['contact_counts']
    carry=(tr[:,15]>.1205)&(counts[:,1]>0)&(counts[:,0]==0)
    samples=contact[1:].reshape(n,3,3)[carry].reshape(-1,3);samples=samples[samples[:,2]>0]
    picked=np.flatnonzero(z['stages'][:,0]);separation=None
    if len(picked):
        for i in range(picked[0],n-9):
            if not (counts[i:i+10,1]>0).any():separation=(i+1)*.03;break
    regions={}
    if (folder/'pad_contact_observations.npz').exists():
        r=np.load(folder/'pad_contact_observations.npz');values=r['values'];assert values.shape==(1+3*n,3,3)
        for j,label in enumerate(r['regions']):
            s=values[1:].reshape(n,3,3,3)[carry,:,j,:].reshape(-1,3);s=s[s[:,2]>0]
            regions[str(label)]=dict(carry_samples=len(s),penetration_mm=quantiles(s[:,0],1000),force_N=quantiles(s[:,1]))
    record=dict(name=status['name'],uid=uid,source_identity=identity,
        max_abs_joint_velocity_rad_s=stats['max_abs_velocity'],max_abs_generalized_torque=stats['max_abs_torque'],
        baseline_reference=str(reference),baseline_is_prior_adaptive=reference_is_adaptive,
        transmission_parameters=meta['physics_treatment']['parameters'],
        distal_lower_limit_rad=meta['physics_treatment']['urdf'].get('distal_lower_limit_rad',-.50),
        reference_drive=meta['physics_treatment'].get('reference_drive'),
        baseline_trajectory_exact=bool(np.array_equal(z['trajectory'],previous['trajectory'])),
        baseline_arm_max_difference_rad=float(np.max(abs(tr[:,:6]-previous['trajectory'][:,:6]))),
        sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],
        first_post_pick_ten_frame_hand_separation_s=separation,
        carry_overlap_mm=quantiles(samples[:,0],1000),carry_contact_force_N=quantiles(samples[:,1]),
        regions=regions,contact_treatment=meta['physics_treatment']['contact_treatment'],
        qualification='Per-run contact sample populations differ. Solver overlap is not material strain; post-pick separation alone is not shelf placement.')
    if (folder/'surface_pad_audit.json').exists():
        a=json.loads((folder/'surface_pad_audit.json').read_text());record['surface_pad_treatment']=a
        s=np.load(folder/'surface_pad_observations.npz')['counts'];assert s.shape==(1+3*n,3)
        assert a['detection_calls']>=8+24*n
        record['surface_pad_contact_counts_sum']=s.sum(axis=0).tolist()
        if reference_is_adaptive and a['pad_timeconst_s']==.02 and record['transmission_parameters']['return_stiffness']==2. and record['transmission_parameters']['actuator_stiffness']==80. and record['distal_lower_limit_rad']==-.50:
            assert record['baseline_trajectory_exact'],'Unchanged-material surface control must reproduce original adaptive trace'
        elif a['pad_timeconst_s']>.02:assert s[:,1].sum()>0,'Candidate did not soften any contacts'
    (folder/'readout.json').write_text(json.dumps(record,indent=2));records.append(record)
(ROOT/'summary.json').write_text(json.dumps(dict(records=records),indent=2))
print(json.dumps([dict(name=r['name'],error=r.get('error'),sequence=r.get('sequence'),metric=r.get('metric'),separation=r.get('first_post_pick_ten_frame_hand_separation_s'),overlap=r.get('carry_overlap_mm')) for r in records],indent=2))
