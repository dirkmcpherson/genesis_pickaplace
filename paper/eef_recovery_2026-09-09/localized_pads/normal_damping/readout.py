"""Verify normal-only intervention, exact controls and complete task outcomes."""
from pathlib import Path
import argparse, hashlib, json, sys
import numpy as np
HERE=Path(__file__).resolve().parent;R=HERE.parents[3];P=HERE.parent;POOL=P.parent
p=argparse.ArgumentParser();p.add_argument('--batch',type=Path,default=HERE);a=p.parse_args();D=a.batch
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan=json.loads((D/'plan.json').read_text())
for f,h in plan['source_code_sha256'].items():assert sha(D/'executed_sources'/Path(f).name)==h
for f,h in plan['engine_source_sha256'].items():assert sha(D/'executed_sources'/('engine_'+Path(f).name))==h
def score(path,out):
 z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());goal=z['goal_pose'];shift=np.linalg.norm(goal[:,:2]-goal[0,:2],axis=1)*1000
 return dict(path=str(path),sha256=sha(path),sequence=m['sequence'],metric=adapt(path,out)['metric'],final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()))
rows=[]
for j in plan['jobs']:
 folder=D/j['name'];execution=D/'logs'/f'{j["name"]}_execution.json'
 if not execution.exists():rows.append(dict(**j,status='pending'));continue
 ex=json.loads(execution.read_text());path=folder/f'{j["uid"]}_eef_delta.npz'
 if ex['returncode']:
  rows.append(dict(**j,status='execution_failed',has_full_trace=path.exists(),execution=ex));continue
 z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text());damp=json.loads((folder/'normal_damping_audit.json').read_text())
 assert sha(j['source'])==j['source_sha256'] and sha(Path(j['source']).with_suffix('.json'))==j['source_metadata_sha256']
 assert sha(j['preset'])==j['preset_sha256']==audit['declared_preset']['sha256']
 assert sha(folder/'gen3_lite_2f_g14_candidate.urdf')==audit['urdf']['candidate_sha256']
 b=np.load(POOL/f'{"early_yaw_pool" if j["uid"]<233 else "timestamp_full_pool"}/{j["uid"]}/collection/{j["uid"]}_eef_delta.npz');n=len(z['trajectory'])
 assert n==len(b['trajectory']) and stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
 assert m['extension_m']==0 and not m['hold_contact']
 assert m['can_pos']==json.loads(Path(j['source']).with_suffix('.json').read_text())['can_pos']
 np.testing.assert_array_equal(z['source_grip'],b['source_grip']);np.testing.assert_array_equal(z['mount'],b['mount'])
 np.testing.assert_allclose(z['target_tool'],b['target_tool'],atol=1e-12);np.testing.assert_allclose(z['actions_joint'],b['actions_joint'],atol=1e-8)
 np.testing.assert_array_equal(z['source_frame_index'],np.arange(n));assert np.all(z['action_kind']=='recorded_path')
 assert damp['gain']==j['gain'] and damp['resolve_calls']==8+24*n
 assert max(damp['max_normal_mapping_tangent_mapping_normal_target_relative_errors'])<1e-5
 counts=damp['counts_selected_normal_changed_normal_checked_unchanged_violations']
 assert counts[0]>0 and counts[2]>0 and counts[3]==0
 assert audit['surface_pad_treatment']['normal_only_damping']['gain']==j['gain']
 if j['gain']==2:assert counts[1]==counts[0]
 else:assert counts[1]==0
 exact=None
 if j['reference']:
  assert sha(j['reference'])==j['reference_sha256'];ref=np.load(j['reference']);assert z.files==ref.files
  exact={k:bool(np.array_equal(z[k],ref[k])) for k in z.files};assert all(exact.values())
 row=dict(**j,status='complete',**score(path,folder/'metric_adapter'),frames=n,damping_audit=damp,exact_reference_arrays=exact,collision_policy=audit['collision_policy'])
 rows.append(row);(folder/'readout.json').write_text(json.dumps(row,indent=2))
for uid,placement in [(113,'conditional'),(113,'original'),(184,'original'),(233,'original')]:
 pair=[r for r in rows if r['uid']==uid and r['placement']==placement and r['gain']==2]
 if all(r['status']=='complete' for r in pair):assert pair[0]['collision_policy']==pair[1]['collision_policy']
refs=[]
for uid,placement in [(113,'conditional'),(113,'original'),(184,'original'),(233,'original')]:
 if placement=='conditional':path=P/'vision_initial_probe/original_fixed/113_eef_delta.npz'
 else:path=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/collection/{uid}_eef_delta.npz'
 refs.append(dict(uid=uid,placement=placement,condition='original_fixed',**score(path,D/'reference_metrics'/f'{uid}_{placement}_original_fixed')))
 for condition in ['rigid','soft']:
  if placement=='conditional':path=P/f'vision_initial_probe/{condition}/113_eef_delta.npz'
  elif uid==184:path=P/f'proximal_compliance/184_{condition}/184_eef_delta.npz'
  else:path=P/f'g14_feedback_full/{uid}_elliptic10_{condition}/{uid}_eef_delta.npz'
  if path.exists():refs.append(dict(uid=uid,placement=placement,condition=condition,gain=1.,**score(path,D/'reference_metrics'/f'{uid}_{placement}_{condition}_g1')))
totals={}
for condition in ['rigid','soft']:
 selected=[r for r in rows if r['placement']=='original' and r['gain']==2 and r['condition']==condition and r['status']=='complete']
 totals[condition]=dict(completed=len(selected),declared=3,metric_pass=sum(r['metric']['slide_success'] for r in selected),strict_complete=sum(r['sequence']['complete'] for r in selected))
report=dict(records=rows,archived_references=refs,original_placement_totals=totals,completed_full_replays=sum(r['status']=='complete' for r in rows),all_terminal=all(r['status']!='pending' for r in rows),qualification='All failures retained. Conditional113 placement is diagnostic only and not adopted. Only normal reference damping changes; normal elastic term, tangent and all other references, contact parameters and friction remain unchanged by the intervention. Frozen chosen calibration cases are not a population estimate. No treatment adoption from these outcomes alone.')
(D/'summary.json').write_text(json.dumps(report,indent=2))
print('TOTALS',json.dumps(totals))
for r in rows:print(r['name'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'),r.get('final_goal_shift_mm'))
