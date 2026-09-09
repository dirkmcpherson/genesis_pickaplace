from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json,subprocess,sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]/'can_pos_recovery'))
from score_recovered_slides import adapt

def run(uid):
 d=ROOT/str(uid)
 if (d/'execution.json').exists():return
 if (d/'collection').exists():raise RuntimeError(f'Inspect partial output {uid}')
 with (d/'run.log').open('w') as log:p=subprocess.run([sys.executable,str(ROOT/'worker.py'),str(uid)],stdout=log,stderr=subprocess.STDOUT)
 result=dict(uid=uid,returncode=p.returncode)
 (d/'execution.json').write_text(json.dumps(result,indent=2))
 if p.returncode:raise RuntimeError(f'Worker {uid} failed')
 result['score']=adapt(d/'collection'/f'{uid}_eef_delta.npz',d/'metric_adapter')
 (d/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

plan=json.loads((ROOT/'plan.json').read_text())
# Confirm complete world and contact readback in first index case before broader dispatch.
run(113)
with ThreadPoolExecutor(max_workers=plan['workers']) as pool:list(pool.map(run,[u for u in plan['uids'] if u!=113]))
