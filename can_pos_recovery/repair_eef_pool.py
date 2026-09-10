"""Apply one preregistered bounded slide repair to every eligible census failure."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from eef_task_sequence import sustained_starts


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('census',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--python',default='/home/james/workspace/genesis_sim2real/venv/bin/python')
    a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    repo=Path(__file__).resolve().parents[1]
    uids=json.loads((a.census/'plan.json').read_text())['uids']
    plan=dict(uids=uids,source=str(a.census.resolve()),
              selection='Failed census sequence, absolute final surface gap <=12 mm, and a supported robot contact after physical release with instantaneous gap strictly between 2 and 25 mm.',
              treatment='Existing repair_eef_slide: last eligible contact, up to 10 mm push in 0.5 mm steps toward live goal; up to 0.3 s hold; precision IK; resume original path. No changes to objects, physics or grip.',
              acceptance='Same physical scorer; retain every result; independent action replay and visual review required before bank admission')
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2))
    seen=set();results=[]
    while len(seen)<len(uids):
        for uid in uids:
            if uid in seen:continue
            folder=a.census/str(uid);execution=folder/'execution.json'
            if not execution.exists():continue
            base=json.loads(execution.read_text());seen.add(uid)
            result=dict(uid=uid,eligible=False,reason='worker_failed')
            if base['returncode']==0:
                seq=base['sequence'];result['reason']='already_complete' if seq['complete'] else 'outside_gap_or_no_push'
                if not seq['complete'] and abs(seq['final_surface_gap_m'])<=.012:
                    source=folder/'collection'/f'{uid}_eef_delta.npz'
                    with np.load(source) as d:
                        r=d['trajectory'];c=d['contact_counts'];g=d['goal_pose']
                        supported=(c[:,0]>0)&(abs(r[:,15]-.2205)<.004)&(r[:,20]<20)
                        release=sustained_starts(supported&(c[:,1]==0)&d['stages'][:,0])
                        gap=np.linalg.norm(r[:,13:15]-g[:,:2],axis=1)-.066
                        possible=np.flatnonzero(supported&(c[:,1]>0)&(gap>.002)&(gap<.025))
                        possible=possible[possible>release[0]+3] if len(release) else []
                    if len(possible):
                        out=a.out/str(uid)
                        cmd=[a.python,str(repo/'can_pos_recovery/repair_eef_slide.py'),str(source.resolve()),
                             '--out',str(out.resolve()),'--max-extension','.01','--polish-ik','--hold-contact']
                        with (a.out/f'{uid}.log').open('w') as log:
                            run=subprocess.run(cmd,cwd=repo,stdout=log,stderr=subprocess.STDOUT)
                        result.update(eligible=True,reason='attempted',returncode=run.returncode)
                        if run.returncode==0:
                            meta=json.loads((out/f'{uid}_eef_delta.json').read_text())
                            result.update(sequence=meta['sequence'],extension_m=meta['extension_m'],repair_note=meta['repair_note'])
                        print(json.dumps(result),flush=True)
            results.append(result)
            (a.out/'execution.json').write_text(json.dumps(results,indent=2))
        if len(seen)<len(uids):time.sleep(5)


if __name__=='__main__':main()
