"""Fixed early-day factorial: travel range x localized pad compliance."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
jobs=[(uid,limit,tc) for uid in [113,184] for limit in [-.5,-1.03] for tc in [.02,.03]]
plan=dict(jobs=[dict(uid=u,distal_lower_limit_rad=l,pad_timeconst_s=tc) for u,l,tc in jobs],
    motivation='Broaden to both early-day calibration trials rather than treating success on one known late-day trial as a necessary gate for all useful configurations. Demonstration recoverability can vary. This revises the earlier 233-first gate, not the end-to-end success definition.',
    fixed='Original adaptive drive (stiffness 80, proximal damping 1), return stiffness 2, distal damping 0.02, original collision geometry, pad depth 3 mm, source actions/clocks, corrected early yaw and initial conditions. Only declared travel/pad settings vary.',
    qualification='Calibration only. Earlier 233 outcomes are known. Evaluate all three calibration trials jointly, with failed stages and regressions visible. No final-distance-only winner selection. Reserved 176/185/237 remain excluded from new fitting.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/run_surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'early_calibration_plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
    uid,limit,tc=job;name=f'{uid}_calibration_{"narrow" if limit==-.5 else "wide"}_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_surface_pad_candidate.py'),
        str(ROOT.parent/f'early_yaw_pool/{uid}/source.npz'),'--out',str(out),'--pad-timeconst',str(tc),'--distal-lower-limit',str(limit)]
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'early_calibration_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
