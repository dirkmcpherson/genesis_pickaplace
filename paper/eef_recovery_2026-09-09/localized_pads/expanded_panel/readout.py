"""Report frozen panel results without scoring unfinished conditions as failures."""
from pathlib import Path
import json,hashlib,sys
import numpy as np
D=Path(__file__).resolve().parent;R=D.parents[3]
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan=json.loads((D/'plan.json').read_text());rows=[]
for f,h in plan['source_code_sha256'].items():assert sha(D/'executed_sources'/Path(f).name)==h
def score(path,out):
 z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());g=z['goal_pose'];shift=np.linalg.norm(g[:,:2]-g[0,:2],axis=1)*1000
 return dict(path=str(path),sha256=sha(path),sequence=m['sequence'],metric=adapt(path,out)['metric'],final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()))
for j in plan['jobs']:
 status=D/'logs'/f'{j["name"]}_execution.json';folder=D/j['name'];path=folder/f'{j["uid"]}_eef_delta.npz'
 if not status.exists():rows.append(dict(**j,status='pending'));continue
 ex=json.loads(status.read_text())
 if ex['returncode']:
  rows.append(dict(**j,status='execution_failed',has_full_trace=path.exists(),execution=ex));continue
 z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());n=len(z['trajectory'])
 audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text());normal=json.loads((folder/'normal_damping_audit.json').read_text())
 assert sha(j['source'])==j['source_sha256'] and sha(Path(j['source']).with_suffix('.json'))==j['source_metadata_sha256']
 assert sha(j['baseline'])==j['baseline_sha256'];b=np.load(j['baseline']);assert n==len(b['trajectory'])
 assert sha(j['preset'])==j['preset_sha256']==audit['declared_preset']['sha256']
 assert sha(folder/'gen3_lite_2f_g14_candidate.urdf')==audit['urdf']['candidate_sha256']
 assert m['can_pos']==json.loads(Path(j['source']).with_suffix('.json').read_text())['can_pos']
 assert m['extension_m']==0 and not m['hold_contact'] and np.all(z['action_kind']=='recorded_path')
 for k in ['source_grip','mount']:np.testing.assert_array_equal(z[k],b[k])
 np.testing.assert_array_equal(z['source_frame_index'],np.arange(n))
 np.testing.assert_allclose(z['target_tool'],b['target_tool'],atol=1e-12);np.testing.assert_allclose(z['actions_joint'],b['actions_joint'],atol=1e-8)
 assert stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
 assert normal['gain']==j['gain'] and normal['resolve_calls']==8+24*n
 assert max(normal['max_input_scaled_normal_tangent_target_errors'])<1e-5
 assert normal['max_jacobian_entry_relative_error']<1e-6
 counts=normal['counts_selected_normal_changed_normal_checked_unchanged_violations']
 assert counts[2]>0 and counts[3]==0
 # No material exposure is a possible real task outcome; report it, never drop the trial.
 assert counts[1]==(counts[0] if j['gain']==2 else 0)
 assert audit['surface_pad_treatment']['normal_only_damping']['gain']==j['gain']
 row=dict(**j,status='complete',**score(path,folder/'metric_adapter'),normal_damping_audit=normal,collision_policy=audit['collision_policy'],geometry=audit['geometry'])
 rows.append(row);(folder/'readout.json').write_text(json.dumps(row,indent=2))
quartets=[];references=[]
for uid in plan['uids']:
 group=[r for r in rows if r['uid']==uid];baseline=Path(group[0]['baseline'])
 reference=dict(uid=uid,day=group[0]['day'],**score(baseline,D/'baseline_metrics'/str(uid)))
 references.append(reference)
 if not all(r['status']=='complete' for r in group):continue
 for r in group[1:]:
  assert r['collision_policy']==group[0]['collision_policy']
  assert r['geometry']==group[0]['geometry']
 q=dict(uid=uid,day=group[0]['day'],original_fixed=reference)
 for r in group:q[f"{r['condition']}_g{r['gain']:g}"]=r
 quartets.append(q)
labels=['original_fixed','rigid_g1','soft_g1','rigid_g2','soft_g2']
def totals(group):
 return {label:dict(n=len(group),metric_pass=sum(r[label]['metric']['slide_success'] for r in group),strict_complete=sum(r[label]['sequence']['complete'] for r in group)) for label in labels}
comparisons={}
for candidate in ['soft_g1','soft_g2']:
 comparisons[candidate]={}
 for control in ['original_fixed',candidate.replace('soft','rigid')]+(['soft_g1'] if candidate=='soft_g2' else []):
  comparisons[candidate][control]=dict(strict_wins=[r['uid'] for r in quartets if r[candidate]['sequence']['complete'] and not r[control]['sequence']['complete']],strict_losses=[r['uid'] for r in quartets if not r[candidate]['sequence']['complete'] and r[control]['sequence']['complete']],metric_wins=[r['uid'] for r in quartets if r[candidate]['metric']['slide_success'] and not r[control]['metric']['slide_success']],metric_losses=[r['uid'] for r in quartets if not r[candidate]['metric']['slide_success'] and r[control]['metric']['slide_success']])
report=dict(records=rows,original_references=references,fully_completed_uids=len(quartets),declared_uids=len(plan['uids']),
 completed_full_replays=sum(r['status']=='complete' for r in rows),all_terminal=all(r['status']!='pending' for r in rows),
 totals_on_complete_quartets=totals(quartets),per_day_on_complete_quartets={day:totals([r for r in quartets if r['day']==day]) for day in ['12-16','12-17','12-18']},paired_comparisons=comparisons,
 qualification='Aggregate comparisons use only complete four-condition UID groups, with denominator stated. Pending and implementation-failed runs retained separately. 36 additionally sampled historical recordings,12 per day, selected without reading outcomes. Previously used source pools; not pristine unseen data, independent participants or population recovery. Initial defects retained. No admission without real-video/action verification.')
(D/'summary.json').write_text(json.dumps(report,indent=2))
print('COMPLETED',report['completed_full_replays'],'/144; complete UID groups',len(quartets),'/36')
print(json.dumps(report['totals_on_complete_quartets']))
for r in rows:
 if r['status']!='pending':print(r['name'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'),r.get('final_goal_shift_mm'))
