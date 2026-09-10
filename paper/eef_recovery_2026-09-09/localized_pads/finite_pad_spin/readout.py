"""Report full-task spin pilot, source invariants and zero-spin layout effects."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[3]
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
from eef_task_sequence import score_sequence
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan=json.loads((D/'plan.json').read_text());rows=[]
for f,h in plan['source_code_sha256'].items():assert sha(D/'executed_sources'/Path(f).name)==h
for j in plan['jobs']:
 folder=D/j['name'];ex=D/'logs'/f'{j["name"]}_execution.json'
 if not ex.exists():rows.append(dict(**j,status='pending'));continue
 execution=json.loads(ex.read_text())
 if execution['returncode']:rows.append(dict(**j,status='execution_failed',execution=execution));continue
 path=folder/f'{j["uid"]}_eef_delta.npz';z=np.load(path);m=json.loads(path.with_suffix('.json').read_text())
 audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text());spin=json.loads((folder/'pad_spin_audit.json').read_text())
 assert sha(j['source'])==j['source_sha256'] and sha(Path(j['source']).with_suffix('.json'))==j['source_metadata_sha256']
 assert sha(j['preset'])==j['preset_sha256']==audit['declared_preset']['sha256']
 assert sha(folder/'gen3_lite_2f_g14_candidate.urdf')==audit['urdf']['candidate_sha256']
 ref=np.load(j['reference']);n=len(z['trajectory']);assert n==len(ref['trajectory'])
 assert stats['substep_calls']==spin['force_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
 assert m['extension_m']==0 and not m['hold_contact']
 assert m['can_pos']==json.loads(Path(j['source']).with_suffix('.json').read_text())['can_pos']
 for k in ['source_grip','mount','source_frame_index','action_kind']:np.testing.assert_array_equal(z[k],ref[k])
 np.testing.assert_allclose(z['target_tool'],ref['target_tool'],atol=1e-12,rtol=0)
 np.testing.assert_allclose(z['actions_joint'],ref['actions_joint'],atol=1e-8,rtol=0)
 counts=spin['all_selected_changed_unexpected_counts'];assert counts[0]>0 and counts[1]>0 and counts[3]==0
 assert counts[2]==(counts[1] if j['spin_length_m'] else 0)
 assert spin['actual_rows_per_contact']==4 and spin['spin_length_m']==j['spin_length_m']
 seq=score_sequence(z);assert seq==m['sequence']
 metric=adapt(path,folder/'metric_adapter')['metric'];goal=z['goal_pose'];shift=1000*np.linalg.norm(goal[:,:2]-goal[0,:2],axis=-1)
 force=np.load(folder/'pad_spin_contacts.npz')['contacts'];assert np.isfinite(force).all()
 zero=None
 if j['spin_length_m']==0:
  zero=dict(exact_arrays={k:bool(np.array_equal(z[k],ref[k])) for k in z.files},
   max_can_position_difference_mm=float(1000*np.linalg.norm(z['trajectory'][:,13:16]-ref['trajectory'][:,13:16],axis=-1).max()))
 row=dict(**j,status='complete',frames=n,path=str(path),sha256=sha(path),sequence=seq,metric=metric,
  final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),spin_audit=spin,
  zero_spin_vs_archived_three_row=zero,observed_spin_rows=len(force),
  max_abs_observed_spin_torque_Nm=float(abs(force[:,7]).max()) if len(force) else 0.,
  collision_policy=audit['collision_policy'],geometry=audit['geometry'])
 rows.append(row);(folder/'readout.json').write_text(json.dumps(row,indent=2))
for uid in [233,113]:
 complete=[r for r in rows if r['uid']==uid and r['status']=='complete']
 for r in complete[1:]:
  assert r['collision_policy']==complete[0]['collision_policy'] and r['geometry']==complete[0]['geometry']
out=dict(records=rows,completed_full_replays=sum(r['status']=='complete' for r in rows),all_terminal=all(r['status']!='pending' for r in rows),qualification='Selected two-demo exploratory pilot, not independent validation. Four-row zero-spin control versus archived three-row simulation retained. No treatment admission from pilot scores.')
(D/'summary.json').write_text(json.dumps(out,indent=2))
for r in rows:print(r['name'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'))
