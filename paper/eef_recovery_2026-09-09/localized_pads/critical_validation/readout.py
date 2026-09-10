"""Frozen validation readout; score terminal runs and retain missing/failed cases."""
from pathlib import Path
import json,sys,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
selection=json.loads((ROOT/'selection.json').read_text());uids=selection['old_reserved']+[r['uid'] for r in selection['new_panel']]
records=[];verifications=[]
def score(path,metric_out):
 z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());goal=z['goal_pose'];shift=np.linalg.norm(goal[:,:2]-goal[0,:2],axis=1)
 return dict(path=str(path),sequence=meta['sequence'],metric=adapt(path,metric_out)['metric'],final_goal_shift_mm=float(shift[-1]*1000),max_goal_shift_mm=float(shift.max()*1000),final_can_to_initial_goal_mm=float(np.linalg.norm(z['trajectory'][-1,13:15]-goal[0,:2])*1000))
for uid in uids:
 baseline=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/collection/{uid}_eef_delta.npz';base=np.load(baseline)
 row=dict(uid=uid,cohort='old_reserved' if uid in selection['old_reserved'] else 'new_reserved',original_fixed=score(baseline,ROOT/'baseline_metrics'/str(uid)))
 for label,tc in [('matching_rigid',.02),('soft_candidate',.03)]:
  name=f'{uid}_tc{tc:g}';status=ROOT/'logs'/f'{name}_execution.json'
  if not status.exists():row[label]=dict(status='pending');continue
  st=json.loads(status.read_text())
  if st['returncode']:row[label]=dict(status='execution_failed',execution=st);continue
  folder=ROOT/name;path=folder/f'{uid}_eef_delta.npz';z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());n=len(z['actions_eef'])
  identity={k:bool(np.array_equal(z[k],base[k])) for k in ['actions_eef','actions_joint','source_grip','mount']};assert all(identity.values()),identity
  preset=meta['physics_treatment']['declared_preset'];assert preset['sha256']==selection['preset_sha256'][f'tc{tc:g}.json']
  stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n
  counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
  if tc>.02 and counts[:,0].sum()>0:assert counts[:,1].sum()>0
  row[label]=dict(status='complete',**score(path,folder/'metric_adapter'),source_identity=identity,material_counts_sum=counts.sum(axis=0).tolist(),max_abs_joint_velocity_rad_s=stats['max_abs_velocity'])
  (folder/'readout.json').write_text(json.dumps(row[label],indent=2))
 records.append(row)
status=ROOT/'logs/233_verify_execution.json'
if status.exists():
 st=json.loads(status.read_text());assert st['returncode']==0
 parent=ROOT.parent/'critical_return/233_tc0.03/233_eef_delta.npz';replay=ROOT/'233_verify/233_eef_delta.npz';a=np.load(parent);b=np.load(replay);meta=json.loads(replay.with_suffix('.json').read_text());assert meta['action_replay_verification'] and meta['extension_m']==0 and not meta['hold_contact']
 assert a.files==b.files;equality={k:bool(np.array_equal(a[k],b[k])) for k in a.files};assert all(equality.values()),equality
 verifications.append(dict(uid=233,source=str(parent),replay=str(replay),source_sha256=hashlib.sha256(parent.read_bytes()).hexdigest(),replay_sha256=hashlib.sha256(replay.read_bytes()).hexdigest(),all_arrays_exact=equality))
complete=[r for r in records if all(r[k].get('status')=='complete' for k in ['matching_rigid','soft_candidate'])]
totals={k:dict(n=len(complete),metric_pass=sum(r[k]['metric']['slide_success'] for r in complete),strict_complete=sum(r[k]['sequence']['complete'] for r in complete)) for k in ['original_fixed','matching_rigid','soft_candidate']}
result=dict(records=records,totals_on_completed_pairs=totals,declared_validation_n=len(uids),fully_completed_pairs=len(complete),verifications=verifications,qualification='Do not interpret incomplete pairs as failures or independent reruns as new demos. Frozen selected panel; no population-rate claim. Retain strict sequence, endpoint metric, goal motion and real-video fidelity separately.')
(ROOT/'summary.json').write_text(json.dumps(result,indent=2));print('completed pairs',len(complete),'/',len(uids),'verified',len(verifications));print(json.dumps(totals))
for r in records:
 print(r['uid'],[(k,r[k].get('status','baseline'),r[k].get('sequence',{}).get('reason'),r[k].get('metric',{}).get('final_dist')) for k in ['original_fixed','matching_rigid','soft_candidate']])
