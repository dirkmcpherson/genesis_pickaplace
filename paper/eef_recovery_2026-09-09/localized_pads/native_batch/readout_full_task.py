"""Retain full native outcomes and repeat checks without admitting GPU tapes."""
from pathlib import Path
import json,hashlib
import numpy as np
D=Path(__file__).resolve().parent;plan=json.loads((D/'full_task_plan.json').read_text())
executed={r['name']:r for r in json.loads((D/'full_task_execution.json').read_text())};rows=[]
for case in plan['cases']:
 name=case['name'];path=D/name
 if name not in executed:rows.append(dict(name=name,status='pending'));continue
 if executed[name]['returncode']:rows.append(dict(name=name,status='execution_failed',execution=executed[name]));continue
 r=json.loads((path/'report.json').read_text());z=np.load(path/'trace.npz');diag=np.load(path/'full_task_diagnostic.npz')
 assert r['full_source_length_verified'] and len(z['state'])==len(np.load(case['paths'][0])['trajectory'])
 assert np.isfinite(z['state']).all() and np.isfinite(diag['trajectory']).all()
 reference=r['task_results'][0]
 assert all(x['sequence']==reference['sequence'] and x['metric']==reference['metric'] for x in r['task_results'])
 rows.append(dict(name=name,status='complete',uid=reference['uid'],steps=r['steps'],lanes=r['actual_envs'],sequence=reference['sequence'],metric=reference['metric'],final_goal_shift_mm=reference['final_goal_shift_mm'],final_tilt_deg=float(diag['trajectory'][-1,0,20]),aggregate_decisions_per_second=r['aggregate_decisions_per_second'],trace_sha256=hashlib.sha256((path/'trace.npz').read_bytes()).hexdigest(),diagnostic_sha256=hashlib.sha256((path/'full_task_diagnostic.npz').read_bytes()).hexdigest()))
repeats={}
for uid in [233,262]:
 a=D/f'full_gpu16_{uid}';b=D/f'full_gpu16_{uid}_repeat'
 if not (a/'report.json').exists() or not (b/'report.json').exists():continue
 checks={}
 for fn in ['trace.npz','full_task_diagnostic.npz']:
  x=np.load(a/fn);y=np.load(b/fn);checks[fn]={k:bool(np.array_equal(x[k],y[k])) for k in x.files}
 repeats[str(uid)]=checks
out=dict(records=rows,repeats=repeats,completed_full_source_cases=sum(r['status']=='complete' for r in rows),all_terminal=all(r['status']!='pending' for r in rows),qualification='Each case is one source replay/control even when it duplicates16 lanes. Repeatability does not imply CPU equivalence or real-world fidelity. Diagnostic arrays are not admitted training tapes.')
(D/'full_task_summary.json').write_text(json.dumps(out,indent=2))
for r in rows:print(r['name'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'))
