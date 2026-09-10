"""Check source/physical invariants and report successive integration refinements."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
D=Path(__file__).resolve().parent;R=D.parents[3]
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
from eef_task_sequence import score_sequence
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan=json.loads((D/'plan.json').read_text());reference=np.load(plan['reference']);rows=[];traces={};audits={}
execution={r['substeps']:r for r in json.loads((D/'execution.json').read_text())} if (D/'execution.json').exists() else {}
for ss in plan['substeps']:
 folder=D/f'233_soft_ss{ss}'
 if ss not in execution:rows.append(dict(substeps=ss,status='pending'));continue
 if execution[ss]['returncode']:rows.append(dict(substeps=ss,status='execution_failed',execution=execution[ss]));continue
 path=folder/'233_eef_delta.npz';z=np.load(path);m=json.loads(path.with_suffix('.json').read_text())
 audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text());n=len(z['trajectory'])
 assert n==len(reference['trajectory']) and stats['substep_calls']==ss*(1+3*n)
 assert stats['max_used_feedback_difference_rad']==0
 assert sha(plan['source'])==plan['source_sha256'] and sha(Path(plan['source']).with_suffix('.json'))==plan['source_metadata_sha256']
 assert sha(plan['preset'])==plan['preset_sha256']==audit['declared_preset']['sha256']
 assert audit['integration_resolution']['substeps']==ss and np.isclose(audit['integration_resolution']['substep_dt_s'],.01/ss)
 assert m['decision_dt_s']==.03 and m['frames']==n and m['extension_m']==0 and not m['hold_contact']
 for k in ['source_grip','mount','source_frame_index','action_kind']:np.testing.assert_array_equal(z[k],reference[k])
 np.testing.assert_allclose(z['actions_joint'],reference['actions_joint'],atol=1e-8,rtol=0)
 np.testing.assert_allclose(z['target_tool'],reference['target_tool'],atol=1e-12,rtol=0)
 assert score_sequence(z)==m['sequence']
 if ss==8:assert all(np.array_equal(z[k],reference[k]) for k in z.files)
 shift=1000*np.linalg.norm(z['goal_pose'][:,:2]-z['goal_pose'][0,:2],axis=-1)
 row=dict(substeps=ss,status='complete',frames=n,sha256=sha(path),sequence=m['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),substep_dt_s=.01/ss)
 rows.append(row);traces[ss]=z;audits[ss]=audit;(folder/'readout.json').write_text(json.dumps(row,indent=2))
comparisons=[]
for lo,hi in [(8,16),(16,32),(8,32)]:
 if lo not in traces or hi not in traces:continue
 for k in ['geometry','collision_policy','armature_readback','normal_parameters','finger_inertial_readback','can_mass_kg','goal_mass_kg']:assert audits[lo][k]==audits[hi][k],k
 a=traces[lo];b=traces[hi];diff=1000*np.linalg.norm(a['trajectory'][:,13:16]-b['trajectory'][:,13:16],axis=-1)
 comparisons.append(dict(substeps=[lo,hi],max_can_position_difference_mm=float(diff.max()),final_can_position_difference_mm=float(diff[-1]),max_finger_difference_rad=float(abs(a['finger_joint']-b['finger_joint']).max()),contact_count_disagreement_frames=int(np.any(a['contact_counts']!=b['contact_counts'],axis=1).sum()),first_over_1mm_s=float((np.flatnonzero(diff>1)[0]+1)*.03) if np.any(diff>1) else None))
out=dict(records=rows,comparisons=comparisons,completed_full_replays=sum(r['status']=='complete' for r in rows),all_terminal=all(r['status']!='pending' for r in rows),qualification='Numerical refinement study on one selected recording, not new independent demonstrations or a material improvement claim. Success at one resolution is not convergence.')
(D/'summary.json').write_text(json.dumps(out,indent=2))
for r in rows:print(r['substeps'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'))
for r in comparisons:print(r)
