"""Compare available frozen GPU cases without accepting a numerical tolerance."""
from pathlib import Path
import hashlib,json
import numpy as np
D=Path(__file__).resolve().parent
names=['supported_single233','supported_single262','supported_mixed16','supported_mixed64','supported_lane1_233','supported_lane1_262','supported_repeat_single233','supported_repeat_mixed16','deterministic_single233','deterministic_mixed16','deterministic_repeat16','deterministic_mixed64']
reports={n:json.loads((D/n/'report.json').read_text()) for n in names if (D/n/'report.json').exists()}
traces={n:np.load(D/n/'trace.npz') for n in reports}
rows=[]
for batch in ['supported_mixed16','supported_mixed64','supported_repeat_single233','supported_repeat_mixed16','deterministic_single233','deterministic_mixed16','deterministic_repeat16','deterministic_mixed64']:
 if batch not in traces:continue
 for reference in reports:
  if reference==batch:continue
  for uid in [233,262]:
   x=traces[batch];y=traces[reference]
   ids=np.flatnonzero(x['env_uids']==uid);refs=np.flatnonzero(y['env_uids']==uid)
   if not len(ids) or not len(refs):continue
   state=x['state'][:,ids];base=y['state'][:,refs[0]:refs[0]+1];delta=abs(state-base)
   assert state.shape[0]==base.shape[0]
   commands=all(np.array_equal(x[key][:,i],y[key][:,refs[0]]) for i in ids for key in ['arm_commands','source_grip'])
   invariants={k:reports[batch][k]==reports[reference][k] for k in ['engine','torch_version','device','geometry','collision_pairs','preset_sha256','candidate_urdf_sha256']}
   diff=np.flatnonzero(np.any(delta!=0,axis=(1,2)))
   rows.append(dict(batch=batch,reference=reference,uid=uid,commands_exact=commands,invariants=invariants,
    state_exact=not len(diff),first_different_decision=int(diff[0]) if len(diff) else None,
    max_arm_rad=float(delta[:,:,:6].max()),max_finger_rad=float(delta[:,:,6:10].max()),
    max_can_position_mm=float(1000*np.linalg.norm(state[:,:,10:13]-base[:,:,10:13],axis=-1).max()),
    final_can_position_mm=float(1000*np.linalg.norm(state[-1,:,10:13]-base[-1,:,10:13],axis=-1).max()),
    max_can_quat_component=float(delta[:,:,13:17].max()),max_goal_component=float(delta[:,:,17:].max())))
out=dict(cases={n:{k:r[k] for k in ['actual_envs','steps','build_reset_seconds','replay_seconds','aggregate_decisions_per_second','max_duplicate_state_difference']} for n,r in reports.items()},
 comparisons=rows,trace_sha256={n:hashlib.sha256((D/n/'trace.npz').read_bytes()).hexdigest() for n in reports},
 qualification='400-decision prefixes; duplicated lanes are not independent demonstrations. No tolerance-based equivalence or task recovery admission. CPU material panel remains separate.')
(D/'supported_comparison.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out['cases'],indent=2))
for r in rows:print(r['batch'],r['reference'],r['uid'],'exact',r['state_exact'],'max can mm',r['max_can_position_mm'])
