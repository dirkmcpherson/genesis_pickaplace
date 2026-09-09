"""Collect the entire declared early pool with fresh processes and retained outcomes."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parent
PYTHON='/home/james/workspace/genesis_sim2real/venv/bin/python'


def run(uid):
    dst=ROOT/str(uid)
    if (dst/'execution.json').exists():return
    if (dst/'collection').exists():raise RuntimeError(f'Inspect partial run {uid} before resuming')
    with (dst/'run.log').open('w') as log:
        p=subprocess.run([PYTHON,str(ROOT/'worker.py'),str(uid)],stdout=log,stderr=subprocess.STDOUT)
    r=dict(uid=uid,returncode=p.returncode)
    if p.returncode==0:r['sequence']=json.loads((dst/'collection'/f'{uid}_eef_delta.json').read_text())['sequence']
    (dst/'execution.json').write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)
    if p.returncode:raise RuntimeError(f'Worker {uid} failed; inspect log')


if __name__=='__main__':
    while not (ROOT/'preparation_complete.json').exists():time.sleep(5)
    plan=json.loads((ROOT/'plan.json').read_text())
    # One per day first; reject setup errors before dispatching the corpus.
    for uid in [118,183]:run(uid)
    with ThreadPoolExecutor(max_workers=plan['workers']) as pool:
        list(pool.map(run,[u for u in plan['uids'] if u not in [118,183]]))
