"""Retain all declared treatment outcomes and paired regressions."""
from collections import Counter
import json
from pathlib import Path

root=Path(__file__).resolve().parent
plan=json.loads((root/'plan.json').read_text())
rows=[]
for uid in plan['uids']:
    p=root/f'{uid}_execution.json'
    r=dict(uid=uid,status='pending')
    if p.exists():
        r.update(json.loads(p.read_text()))
        r['status']='finished' if r['returncode']==0 else 'worker_failed'
    rows.append(r)
done=[r for r in rows if r['status']=='finished']
summary=dict(cohort='16 stratified December-18 trials; uncalibrated distal-compliance surrogate',
             declared=len(rows),finished=len(done),
             worker_failed=sum(r['status']=='worker_failed' for r in rows),
             outcomes=dict(Counter(r['sequence']['reason'] for r in done)),
             new_complete=[r['uid'] for r in done if r['sequence']['complete'] and not r['control_sequence']['complete']],
             lost_complete=[r['uid'] for r in done if not r['sequence']['complete'] and r['control_sequence']['complete']],
             stage_changes=[dict(uid=r['uid'],before=r['control_sequence']['reason'],after=r['sequence']['reason'])
                            for r in done if r['sequence']['reason']!=r['control_sequence']['reason']],
             records=rows)
(root/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='records'}))
