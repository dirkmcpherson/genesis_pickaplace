"""Checkpointed timing census; every source UID retains an explicit disposition."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import importlib.util
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent
spec=importlib.util.spec_from_file_location('timing_probe',PARENT/'timestamp_probe/run.py')
probe=importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
probe.ROOT=ROOT


def main():
    uids=json.loads((PARENT/'full_pool/plan.json').read_text())['uids']
    pending=[254,295,300]
    plan=dict(uids=uids,pending_source_alignment=pending,
              treatment='Resample original measurements by matched bag window times, 30 ms actions',
              physics='w3 unchanged; no grip filter or added slide',
              initial_poses='Archived except video-supported upright orientation/z for 234 and 318; xy unchanged',
              prior_runs_reused=[233,234,235],workers=6,
              interpretation='Development corpus, not held-out; compare 234/318 to corrected-IC controls',
              admission='No automatic bank admission; require independent replay and visual review')
    plan_path=ROOT/'plan.json'
    if plan_path.exists():assert json.loads(plan_path.read_text())==plan
    else:plan_path.write_text(json.dumps(plan,indent=2))
    todo=[]
    for uid in uids:
        if uid in pending:continue
        dst=ROOT/str(uid)
        if not dst.exists():
            if uid in plan['prior_runs_reused']:
                shutil.copytree(PARENT/'timestamp_probe'/str(uid),dst)
                (dst/'reuse.json').write_text(json.dumps(dict(source=str(PARENT/'timestamp_probe'/str(uid))),indent=2))
            else:
                probe.prepare(uid)
                if uid==318:
                    p=dst/'source.json';m=json.loads(p.read_text())
                    correction=json.loads((PARENT/'upright_ic_probe/318/source.json').read_text())
                    for key in ('can_pos','can_quat','initial_pose_correction'):
                        m[key]=correction[key]
                    p.write_text(json.dumps(m,indent=2))
        result=dst/'execution.json'
        if result.exists():
            assert json.loads(result.read_text())['returncode']==0
            continue
        # Never overwrite partial outputs after a killed worker.
        if (dst/'collection').exists():
            raise RuntimeError(f'Inspect partial output before resuming {uid}')
        todo.append(uid)
    print(json.dumps(dict(to_run=todo,pending_source_alignment=pending)),flush=True)
    with ThreadPoolExecutor(max_workers=6) as pool:
        jobs={pool.submit(probe.run,uid):uid for uid in todo}
        for job in as_completed(jobs):job.result()


if __name__=='__main__':main()
