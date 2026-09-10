"""Retain every frozen validation pair and compare original, rigid and soft outcomes."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
plan=json.loads((ROOT/'plan.json').read_text())
for f,sha in plan['source_code_sha256'].items():assert hashlib.sha256((ROOT/'executed_sources'/Path(f).name).read_bytes()).hexdigest()==sha
uids=list(dict.fromkeys(j['uid'] for j in plan['jobs']));rows=[]
def score(path,out):
 z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());goal=z['goal_pose'];shift=np.linalg.norm(goal[:,:2]-goal[0,:2],axis=1)*1000
 return dict(sequence=m['sequence'],metric=adapt(path,out)['metric'],final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),path=str(path))
for uid in uids:
 baseline=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/collection/{uid}_eef_delta.npz';base=np.load(baseline)
 row=dict(uid=uid,day='Dec16' if uid<=181 else 'Dec17' if uid<233 else 'Dec18',original_fixed=score(baseline,ROOT/'baseline_metrics'/str(uid)))
 for label,tc in [('rigid',.02),('soft',.03)]:
  name=f'{uid}_tc{tc:g}';status=ROOT/'logs'/f'{name}_execution.json'
  if not status.exists():row[label]=dict(status='pending');continue
  st=json.loads(status.read_text())
  if st['returncode']:row[label]=dict(status='execution_failed',execution=st);continue
  folder=ROOT/name;path=folder/f'{uid}_eef_delta.npz';z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());n=len(z['source_grip']);a=json.loads((folder/'transmission_audit.json').read_text())
  assert n==len(base['source_grip']) and m['extension_m']==0 and not m['hold_contact']
  diffs={k:float(np.max(abs(z[k]-base[k]))) for k in ['target_tool','actions_eef','actions_joint','source_grip','mount']}
  np.testing.assert_allclose(z['target_tool'],base['target_tool'],atol=1e-12);np.testing.assert_allclose(z['actions_joint'],base['actions_joint'],atol=1e-8)
  np.testing.assert_array_equal(z['source_grip'],base['source_grip']);np.testing.assert_array_equal(z['mount'],base['mount']);np.testing.assert_array_equal(z['source_frame_index'],np.arange(n));assert np.all(z['action_kind']=='recorded_path')
  assert hashlib.sha256((folder/'gen3_lite_2f_g14_candidate.urdf').read_bytes()).hexdigest()==a['urdf']['candidate_sha256']
  assert a['declared_preset']['sha256']==plan['preset_sha256'][str(tc)]
  stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
  counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
  if tc==.02:assert counts[:,1].sum()==0
  elif counts[:,0].sum()>0:assert counts[:,1].sum()>0
  assert a['collision_policy']['enable_neutral_collision']==False
  row[label]=dict(status='complete',**score(path,folder/'metric_adapter'),source_command_max_differences=diffs,material_counts_sum=counts.sum(axis=0).tolist(),max_abs_joint_velocity_rad_s=stats['max_abs_velocity'],collision_policy=a['collision_policy'])
  (folder/'readout.json').write_text(json.dumps(row[label],indent=2))
 if all(row[k].get('status')=='complete' for k in ['rigid','soft']):assert row['rigid']['collision_policy']==row['soft']['collision_policy']
 rows.append(row)
complete=[r for r in rows if all(r[k].get('status')=='complete' for k in ['rigid','soft'])]
totals={k:dict(n=len(complete),metric_pass=sum(r[k]['metric']['slide_success'] for r in complete),strict_complete=sum(r[k]['sequence']['complete'] for r in complete)) for k in ['original_fixed','rigid','soft']}
comparisons={}
for ref in ['original_fixed','rigid']:
 comparisons[ref]=dict(soft_strict_wins=[r['uid'] for r in complete if r['soft']['sequence']['complete'] and not r[ref]['sequence']['complete']],soft_strict_losses=[r['uid'] for r in complete if not r['soft']['sequence']['complete'] and r[ref]['sequence']['complete']],soft_metric_wins=[r['uid'] for r in complete if r['soft']['metric']['slide_success'] and not r[ref]['metric']['slide_success']],soft_metric_losses=[r['uid'] for r in complete if not r['soft']['metric']['slide_success'] and r[ref]['metric']['slide_success']])
result=dict(records=rows,totals_on_completed_pairs=totals,paired_comparisons=comparisons,fully_completed_pairs=len(complete),declared_pairs=len(uids),qualification='Frozen existing selected panel; earlier model outcomes known. Do not score pending jobs as failures. Positive task metrics require separate goal-motion and real-video fidelity checks. Not a population estimate or material calibration.')
(ROOT/'summary.json').write_text(json.dumps(result,indent=2));print('complete_pairs',len(complete),'/',len(uids),json.dumps(totals))
for r in rows:print(r['uid'],[(k,r[k].get('sequence',{}).get('reason',r[k].get('status')),round(r[k].get('metric',{}).get('final_dist',-1)*1000,2),round(r[k].get('final_goal_shift_mm',-1),2)) for k in ['original_fixed','rigid','soft']])
