"""Score only terminal, successful runs; preserve fixed slide predicate."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt

records=[]
for uid in [233,113]:
 previous_path=ROOT.parent/f'adaptive_transmission/{uid}_active/{uid}_eef_delta.npz'
 previous=np.load(previous_path)
 for scope in ['none','fingers','can']:
  name=f'{uid}_{scope}';execution=ROOT/'logs'/f'{name}_execution.json'
  if not execution.exists():continue
  status=json.loads(execution.read_text())
  if status['returncode']!=0:
   records.append(dict(uid=uid,scope=scope,error='Worker failed',execution=status));continue
  folder=ROOT/name;path=folder/f'{uid}_eef_delta.npz'
  data=np.load(path);meta=json.loads(path.with_suffix('.json').read_text())
  stats=json.loads((folder/'transmission_stats.json').read_text())
  n=len(data['actions_eef']);assert stats['substep_calls']==8+24*n
  identical={key:bool(np.array_equal(data[key],previous[key])) for key in ['actions_eef','actions_joint','source_grip','mount']}
  assert all(identical.values()),identical
  baseline_exact=None
  if scope=='none':
   baseline_exact={key:bool(np.array_equal(data[key],previous[key])) for key in ['trajectory','finger_joint','observations','contact_counts']}
   assert all(baseline_exact.values()),baseline_exact
  contact=np.load(folder/'contact_observations.npz')['values']
  assert contact.shape==(1+3*n,3)
  per_step=contact[1:].reshape(n,3,3)
  any_contact=contact[:,2]>0
  tr=data['trajectory'];counts=data['contact_counts']
  carry=(tr[:,15]>.1205)&(counts[:,1]>0)&(counts[:,0]==0)
  carry_contact=per_step[carry].reshape(-1,3);carry_contact=carry_contact[carry_contact[:,2]>0]
  def quantiles(values,scale):
   return dict(median=float(np.median(values)*scale),p95=float(np.quantile(values,.95)*scale),max=float(np.max(values)*scale)) if len(values) else None
  picked=np.flatnonzero(data['stages'][:,0]);separation=None
  if len(picked):
   for i in range(int(picked[0]),n-9):
    if not (counts[i:i+10,1]>0).any():separation=(i+1)*.03;break
  curl=-np.rad2deg(data['finger_joint'][:,2:]-(.149-.676*data['finger_joint'][:,1,None]))
  score=adapt(path,folder/'metric_adapter')
  report=dict(uid=uid,scope=scope,source_identity=identical,baseline_trace_exact=baseline_exact,
   contact_treatment=meta['physics_treatment']['contact_treatment'],strict_sequence=meta['sequence'],metric=score['metric'],
   first_post_pick_ten_frame_hand_separation_s=separation,final_tilt_deg=float(tr[-1,20]),
   max_independent_inward_curl_deg=curl.max(axis=0).tolist(),
   all_contact_penetration_mm=quantiles(contact[any_contact,0],1000),all_contact_force_N=quantiles(contact[any_contact,1],1),
   carry_penetration_mm=quantiles(carry_contact[:,0],1000),carry_max_contact_force_N=quantiles(carry_contact[:,1],1),
   carry_scene_samples=len(carry_contact),contact_scene_samples=int(any_contact.sum()),
   qualification='Penetration is solver contact overlap, not measured pad compression or can strain. Contact-dependent sample populations change between treatments; quantiles are descriptive. Separation after pickup alone is not successful shelf release.')
  (folder/'readout.json').write_text(json.dumps(report,indent=2));records.append(report)
report=dict(declared=6,finished=len(records),records=records,scope='Two selected mechanism trials, not broad recovery yield or physical material calibration')
(ROOT/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps([dict(uid=r['uid'],scope=r['scope'],error=r.get('error'),sequence=r.get('strict_sequence'),metric=r.get('metric'),carry_penetration_mm=r.get('carry_penetration_mm')) for r in records],indent=2))
