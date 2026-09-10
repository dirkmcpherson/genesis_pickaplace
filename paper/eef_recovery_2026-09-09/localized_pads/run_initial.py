"""Declared initial 233 geometry/softness comparison; every run retained."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import os, json, subprocess, time, sys
ROOT=Path(__file__).resolve().parent; REPO=ROOT.parents[2]
jobs=[('hull',.02),('split',.02),('split',.025),('split',.03),('split',.04)]
plan=dict(uid=233,jobs=jobs,pad_depth_m=.003,
    purpose='Establish geometry effect and whether localized compliance improves both grasp and release before extending to early-day calibration trials.',
    fixed='Original EEF/gripper inputs, yaw, initial conditions, adaptive transmission, friction, scene timestep and strict/supplied metrics.',
    limitations='Pad boundaries and depth are assumptions. Trial previously inspected; no population-yield claim. Geometry subdivision can change contacts even with identical exterior support.')
(ROOT/'initial_plan.json').write_text(json.dumps(plan,indent=2))
(ROOT/'logs').mkdir(exist_ok=True)
def run(job):
    geometry,tc=job;name=f'233_{geometry}_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_adaptive_gripper_candidate.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--pad-geometry',geometry,'--pad-timeconst',str(tc),'--record-contact']
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'initial_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
