"""Score every declared run and verify spatial treatment and archived controls."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
plan=json.loads((D/'plan.json').read_text());records=[]
for f,sha in plan['source_code_sha256'].items():
 assert hashlib.sha256((D/'executed_sources'/Path(f).name).read_bytes()).hexdigest()==sha
for job in plan['jobs']:
 name=job['name'];execution=D/'logs'/f'{name}_execution.json'
 if not execution.exists():records.append(dict(**job,status='pending'));continue
 status=json.loads(execution.read_text())
 if status['returncode']!=0:records.append(dict(**job,status='execution_failed',execution=status));continue
 folder=D/name;path=folder/f'{job["uid"]}_eef_delta.npz';z=np.load(path);m=json.loads(path.with_suffix('.json').read_text())
 audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text())
 source=np.load(job['source']);n=len(z['source_grip'])
 assert hashlib.sha256(Path(job['source']).read_bytes()).hexdigest()==job['source_sha256']
 assert audit['declared_preset']['sha256']==plan['preset_sha256'][job['condition']]
 assert hashlib.sha256((folder/'gen3_lite_2f_g14_candidate.urdf').read_bytes()).hexdigest()==audit['urdf']['candidate_sha256']
 assert stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
 assert m['extension_m']==0 and not m['hold_contact'];assert np.all(z['action_kind']=='recorded_path')
 np.testing.assert_array_equal(z['source_frame_index'],np.arange(n))
 baseline=R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if job["uid"]<233 else "timestamp_full_pool"}/{job["uid"]}/collection/{job["uid"]}_eef_delta.npz'
 b=np.load(baseline);assert n==len(b['source_grip'])
 for key in ['source_grip','mount']:np.testing.assert_array_equal(z[key],b[key])
 np.testing.assert_allclose(z['target_tool'],b['target_tool'],atol=1e-12)
 np.testing.assert_allclose(z['actions_joint'],b['actions_joint'],atol=1e-8)
 material=np.load(folder/'surface_pad_observations.npz');counts=material['activation_by_geom'];assert len(counts)==1+3*n
 activity={r['link']:int(counts[:,r['geom']].sum()) for r in audit['surface_pad_treatment']['regions']}
 if job['condition']=='rigid':assert counts.sum()==0
 if job['condition']=='proximal':
  assert audit['surface_pad_treatment']['compliant_part']=='proximal'
  assert all(v==0 for k,v in activity.items() if 'dist' in k)
  assert all(r['receives_compliance']==('prox' in r['link']) for r in audit['surface_pad_treatment']['regions'])
  # Zero pad contacts remain a possible task failure, not a reason to drop a run.
 archived=None;agreement=None
 if job['uid'] in [113,233] and job['condition']=='rigid':archived=P/f'g14_feedback_full/{job["uid"]}_elliptic10_rigid/{job["uid"]}_eef_delta.npz'
 if job['condition']=='all_soft_control':archived=P/'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
 if archived:
  old=np.load(archived);assert z.files==old.files
  agreement={key:bool(np.array_equal(z[key],old[key],equal_nan=True)) if z[key].dtype.kind in 'fc' else bool(np.array_equal(z[key],old[key])) for key in z.files}
  assert all(agreement.values()),agreement
 goal=z['goal_pose'];shift=np.linalg.norm(goal[:,:2]-goal[0,:2],axis=1)*1000
 row=dict(**job,status='complete',sequence=m['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],
  final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),material_activity=activity,
  archived_control=str(archived) if archived else None,exact_control_arrays=agreement,
  collision_policy=audit['collision_policy'])
 records.append(row);(folder/'readout.json').write_text(json.dumps(row,indent=2))
for uid in [113,184,233]:
 pair=[r for r in records if r['uid']==uid and r['condition'] in ['rigid','proximal']]
 if all(r['status']=='complete' for r in pair):assert pair[0]['collision_policy']==pair[1]['collision_policy']
totals={}
for condition in ['rigid','proximal']:
 selected=[r for r in records if r['condition']==condition and r['status']=='complete']
 totals[condition]=dict(completed=len(selected),declared=3,metric_pass=sum(r['metric']['slide_success'] for r in selected),strict_complete=sum(r['sequence']['complete'] for r in selected))
result=dict(records=records,totals=totals,all_terminal=all(r['status']!='pending' for r in records),qualification='Selected calibration trio. Counts do not establish population yield. Retain all failures and goal movement; original source commands and metrics unchanged.')
(D/'summary.json').write_text(json.dumps(result,indent=2))
print(json.dumps(totals))
for r in records:print(r['name'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'),r.get('final_goal_shift_mm'))
