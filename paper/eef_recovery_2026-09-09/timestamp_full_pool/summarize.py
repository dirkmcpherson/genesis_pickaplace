"""Report all declared trials, including unfinished work; never infer failure from absence."""
from collections import Counter
import json
from pathlib import Path

root=Path(__file__).resolve().parent
plan=json.loads((root/'plan.json').read_text())
rows=[]
for uid in plan['uids']:
    p=root/str(uid)/'execution.json'
    r=dict(uid=uid,status='pending')
    if p.exists():
        result=json.loads(p.read_text())
        r.update(result)
        r['status']='finished' if result['returncode']==0 else 'worker_failed'
    control=root.parent/('upright_ic_probe' if uid in (234,318) else 'full_pool')/str(uid)/'collection'/f'{uid}_eef_delta.json'
    r['control_sequence']=json.loads(control.read_text())['sequence']
    rows.append(r)
done=[r for r in rows if r['status']=='finished']
transitions=Counter((r['control_sequence']['reason'],r['sequence']['reason']) for r in done)
summary=dict(declared=len(rows),finished=len(done),pending=sum(r['status']=='pending' for r in rows),
             worker_failed=sum(r['status']=='worker_failed' for r in rows),
             sequence_counts=dict(Counter(r['sequence']['reason'] for r in done)),
             newly_complete=[r['uid'] for r in done if r['sequence']['complete'] and not r['control_sequence']['complete']],
             lost_complete=[r['uid'] for r in done if not r['sequence']['complete'] and r['control_sequence']['complete']],
             stage_changes=[dict(uid=r['uid'],before=r['control_sequence']['reason'],after=r['sequence']['reason'])
                            for r in done if r['control_sequence']['reason']!=r['sequence']['reason']],
             transition_counts=[dict(before=a,after=b,count=n) for (a,b),n in sorted(transitions.items())],
             scope='Incomplete counts are provisional; no automatic admission or fidelity claim',records=rows)
(root/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='records'}))
