"""Run a declared, stratified compliance diagnostic after the timing controls finish."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
CONTROL=ROOT.parent/'timestamp_full_pool'
PYTHON='/home/james/workspace/genesis_sim2real/venv/bin/python'


def run(uid):
    out=ROOT/str(uid)
    if out.exists():raise FileExistsError(out)
    with (ROOT/f'{uid}.log').open('w') as log:
        p=subprocess.run([PYTHON,str(REPO/'can_pos_recovery/probe_passive_fingers.py'),
                          str(CONTROL/str(uid)/'source.npz'),'--out',str(out),'--timeconst','.3'],
                         stdout=log,stderr=subprocess.STDOUT)
    result=dict(uid=uid,returncode=p.returncode)
    if p.returncode==0:
        path=out/f'{uid}_eef_delta.npz'
        result['sequence']=json.loads(path.with_suffix('.json').read_text())['sequence']
        result['control_sequence']=json.loads((CONTROL/str(uid)/'execution.json').read_text())['sequence']
        with np.load(path) as d:
            f=d['finger_joint'];c=d['contact_counts'];res=np.rad2deg(f[:,2:]-(-.676*f[:,1,None]+.149))
            for label,mask in [('can_contact',c[:,1]>0),('no_can_contact',c[:,1]==0)]:
                result[label]=dict(frames=int(mask.sum()),min_residual_deg=res[mask].min(axis=0).tolist(),
                                  max_residual_deg=res[mask].max(axis=0).tolist()) if mask.any() else None
    (ROOT/f'{uid}_execution.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    plan=json.loads((ROOT/'plan.json').read_text())
    control_uids=json.loads((CONTROL/'plan.json').read_text())['uids']
    print('Waiting for all declared timestamp controls; no treatment running yet',flush=True)
    while not all((CONTROL/str(u)/'execution.json').exists() for u in control_uids):
        time.sleep(10)
    assert all(json.loads((CONTROL/str(u)/'execution.json').read_text())['returncode']==0 for u in control_uids)
    print('Timestamp controls finished; starting declared treatments',flush=True)
    with ThreadPoolExecutor(max_workers=plan['workers']) as pool:list(pool.map(run,plan['uids']))
