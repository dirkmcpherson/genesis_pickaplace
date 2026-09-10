"""Collect a declared UID pool through the precise EEF controller, retaining all outcomes."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('plan',type=Path)
    p.add_argument('--workers',type=int,default=6)
    p.add_argument('--python',default='/home/james/workspace/genesis_sim2real/venv/bin/python')
    a=p.parse_args()
    import numpy as np
    root=a.plan.resolve().parent;repo=Path(__file__).resolve().parents[1]
    plan=json.loads(a.plan.read_text())
    placements=json.loads((repo/'can_pos_recovery/trial_placements.json').read_text())
    if hashlib.sha256((repo/'can_pos_recovery/trial_placements.json').read_bytes()).hexdigest()!=plan['placements_sha256']:
        raise ValueError('Placement table changed after pool registration')
    def run(uid):
        out=root/str(uid);out.mkdir(exist_ok=False)
        raw=repo/f'inthewild_trials/{uid}_episodes.npy'
        tape=np.load(raw,allow_pickle=True).item();rec=placements['trials'][str(uid)]
        np.savez_compressed(out/'source.npz',joint_waypoints=np.asarray(tape['vel_cmd']),
                            source_grip=np.asarray(tape['gripper_pos'])[:,0])
        meta=dict(uid=uid,variant=plan['variant'],variant_post_hook=True,source_kind='kinematic_tape',
                  can_pos=rec['can_pos'],can_quat=rec.get('can_quat') or [1,0,0,0],
                  source_frames=len(tape['vel_cmd']),source_sha256=hashlib.sha256(raw.read_bytes()).hexdigest())
        (out/'source.json').write_text(json.dumps(meta,indent=2))
        cmd=[a.python,str(repo/'can_pos_recovery/repair_eef_slide.py'),str(out/'source.npz'),
             '--out',str(out/'collection'),'--max-extension','0','--polish-ik']
        with (out/'run.log').open('w') as log:r=subprocess.run(cmd,cwd=repo,stdout=log,stderr=subprocess.STDOUT)
        result=dict(uid=uid,returncode=r.returncode)
        result_path=out/'collection'/f'{uid}_eef_delta.json'
        if r.returncode==0:
            result['sequence']=json.loads(result_path.read_text())['sequence']
        (out/'execution.json').write_text(json.dumps(result,indent=2))
        return result
    results=[]
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        futures=[pool.submit(run,uid) for uid in plan['uids']]
        for f in as_completed(futures):
            result=f.result();results.append(result)
            print(json.dumps(result),flush=True)
            (root/'execution.json').write_text(json.dumps(results,indent=2))
    if any(r['returncode'] for r in results):
        raise RuntimeError('One or more workers failed; inspect retained logs')


if __name__=='__main__':
    main()
