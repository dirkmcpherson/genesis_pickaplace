"""Separate early tipping by pickup history and preceding recorded grip change."""
from pathlib import Path
from collections import Counter
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
funnel=json.loads((ROOT/'final_failure_funnel.json').read_text())
records=[]
for day,group in funnel['days'].items():
 for r in group['records']:
  if r['initial_overlap'] or not r['final_tipped'] or not r['tip_before_support']:continue
  uid=r['uid'];p=ROOT/str(uid)/'collection'/f'{uid}_eef_delta.npz'
  with np.load(p) as z:
   i=r['first_tip_frame'];picked=np.flatnonzero(z['stages'][:i+1,0]);a=max(0,i-33)
   grip=z['source_grip'];tr=z['trajectory'];c=z['contact_counts']
   records.append(dict(uid=uid,day=day,source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),tip_frame=i,tip_time_s=(i+1)*.03,
    picked_before_tip=bool(len(picked)),first_pick_frame=int(picked[0]) if len(picked) else None,
    grip_at_tip=float(grip[i]),grip_one_second_before=float(grip[a]),grip_change_last_second=float(grip[i]-grip[a]),
    prior_second_grip_min=float(grip[a:i+1].min()),prior_second_grip_max=float(grip[a:i+1].max()),
    robot_contact_fraction_last_second=float((c[a:i+1,1]>0).mean()),shelf_contact_fraction_last_second=float((c[a:i+1,0]>0).mean()),
    can_z_at_tip=float(tr[i,15]),can_z_max_before_tip=float(tr[:i+1,15].max())))
summary={day:dict(n=sum(r['day']==day for r in records),picked_before_tip=sum(r['day']==day and r['picked_before_tip'] for r in records)) for day in funnel['days']}
(ROOT/'tip_grip_audit.json').write_text(json.dumps(dict(summary=summary,records=records,limitation='Continuous grip values are recorded motor positions, not independently measured finger curl or intended commands. One-second window is descriptive, not an acceptance threshold. Pick flag is only a diagnostic.'),indent=2))
print(json.dumps(summary))
for r in records:print(r['uid'],'picked',r['picked_before_tip'],'tip_s',round(r['tip_time_s'],2),'grip',round(r['grip_at_tip'],1),'delta',round(r['grip_change_last_second'],1),'hand_contact',round(r['robot_contact_fraction_last_second'],2))
