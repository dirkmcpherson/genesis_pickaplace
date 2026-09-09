import json, subprocess, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]/'can_pos_recovery'))
from score_recovered_slides import adapt
for uid in json.loads((ROOT/'plan.json').read_text())['uids']:
    dst=ROOT/str(uid)
    if (dst/'execution.json').exists(): continue
    if (dst/'collection').exists(): raise RuntimeError(f'Inspect partial collection {uid}')
    with (dst/'run.log').open('w') as log:
        p=subprocess.run([sys.executable,str(ROOT/'worker.py'),str(uid)],stdout=log,stderr=subprocess.STDOUT)
    result=dict(uid=uid,returncode=p.returncode)
    (dst/'execution.json').write_text(json.dumps(result,indent=2))
    if p.returncode: raise RuntimeError(f'Worker {uid} failed')
    result['score']=adapt(dst/'collection'/f'{uid}_eef_delta.npz',dst/'metric_adapter')
    (dst/'metric_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
