"""Account for every early candidate, separately by recording day."""
from collections import Counter
import json
from pathlib import Path

root=Path(__file__).resolve().parent
plan=json.loads((root/'plan.json').read_text())
rows=[]
for uid in plan['uids']:
    d=root/str(uid);m=json.loads((d/'source.json').read_text())
    r=dict(uid=uid,day=m['day'],status='pending',world_checked=(d/'world_audit.json').exists())
    if (d/'execution.json').exists():
        r.update(json.loads((d/'execution.json').read_text()))
        r['status']='finished' if r['returncode']==0 else 'worker_failed'
    r['independent_replay_checked']=(d/'verification/array_comparison.json').exists()
    rows.append(r)
days={}
for day in sorted({r['day'] for r in rows}):
    group=[r for r in rows if r['day']==day]
    done=[r for r in group if r['status']=='finished']
    days[day]=dict(declared=len(group),finished=len(done),
                   worker_failed=sum(r['status']=='worker_failed' for r in group),
                   reasons=dict(Counter(r['sequence']['reason'] for r in done)),
                   complete_candidates=[r['uid'] for r in done if r['sequence']['complete']])
summary=dict(cohort='December 16/17 early candidates; full world with day-specific yaw',
             by_day=days,records=rows,
             limitation='Numerical completion is not bank admission or evidence of calibrated physical fidelity')
(root/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='records'}))
