"""Score completed fresh-feedback replays against unchanged source and task gates."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
plan=json.loads((ROOT/'plan.json').read_text())
for filename,sha in plan['source_code_sha256'].items():assert hashlib.sha256((ROOT/'executed_sources'/Path(filename).name).read_bytes()).hexdigest()==sha
executions=[]
for f in sorted(ROOT.glob('execution*.json')):executions+=json.loads(f.read_text())
rows=[]
for entry in executions:
 name=entry['name'];uid=entry['uid'];folder=ROOT/name
 if entry['returncode']:
  rows.append(dict(name=name,uid=uid,error='Execution failed; not scored'));continue
 path=folder/f'{uid}_eef_delta.npz';z=np.load(path);n=len(z['source_grip'])
 ref=np.load(ROOT.parent/f'finger_inertia_alignment/{uid}_tc0.02/{uid}_eef_delta.npz')
 meta=json.loads(path.with_suffix('.json').read_text());audit=json.loads((folder/'transmission_audit.json').read_text())
 assert n==len(ref['source_grip']) and meta['extension_m']==0 and not meta['hold_contact']
 diffs={k:float(np.max(abs(z[k]-ref[k]))) for k in ['target_tool','actions_eef','actions_joint','mount','source_grip']}
 np.testing.assert_allclose(z['target_tool'],ref['target_tool'],atol=1e-12)
 np.testing.assert_allclose(z['actions_joint'],ref['actions_joint'],atol=1e-8)
 np.testing.assert_array_equal(z['source_grip'],ref['source_grip']);np.testing.assert_array_equal(z['mount'],ref['mount'])
 np.testing.assert_array_equal(z['source_frame_index'],np.arange(n));assert np.all(z['action_kind']=='recorded_path')
 stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
 counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
 tc=audit['surface_pad_treatment']['pad_timeconst_s'];assert counts[:,0].sum()>0
 if tc==.02:assert counts[:,1].sum()==0
 else:assert counts[:,1].sum()>0
 assert hashlib.sha256((folder/'gen3_lite_2f_g14_candidate.urdf').read_bytes()).hexdigest()==audit['urdf']['candidate_sha256']
 shift=np.linalg.norm(z['goal_pose'][:,:2]-z['goal_pose'][0,:2],axis=1)*1000
 tool_error=np.linalg.norm(z['trajectory'][:,6:9]-z['target_tool'][:,:3,3],axis=1)*1000
 t=(np.arange(n)+1)*.03;sel=(t>=.3)&(t<=3);theta=.96-1.05*z['source_grip']/100;target=np.stack([-theta,theta,.149-.676*theta,.149-.676*theta],axis=-1);error=np.rad2deg(abs(z['finger_joint']-target))
 row=dict(name=name,uid=uid,frames=n,sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],
  source_command_max_differences=diffs,material_contact_count_sums=counts.sum(axis=0).tolist(),
  final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),
  measured_tool_target_error_mm_p50_p95_max=np.percentile(tool_error,[50,95,100]).tolist(),
  approach_max_finger_target_error_deg=float(error[sel].max()),max_abs_joint_velocity_rad_s=stats['max_abs_velocity'],
  max_used_feedback_difference_rad=stats['max_used_feedback_difference_rad'])
 (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
 print(name,row['sequence']['reason'],row['metric']['slide_success'],round(row['metric']['final_dist']*1000,2),'goal_shift_mm',round(row['final_goal_shift_mm'],2))
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows,qualification='Explicitly selected calibration demonstrations. Not population validation or hardware calibration. Same-engine contrasts retain processed geometry and analytic mass; cross-engine comparison changes those as well.'),indent=2))
