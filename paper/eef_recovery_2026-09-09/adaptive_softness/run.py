"""Fixed six-run comparison: adaptive baseline, softer fingers, softer can."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,sys,time
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
PYTHON=Path('/home/james/workspace/genesis_sim2real/venv/bin/python')
jobs=[(233,'none'),(233,'fingers'),(233,'can'),(113,'none'),(113,'fingers'),(113,'can')]
plan=dict(trials=[233,113],treatments=['none','fingers','can'],geom_timeconst_baseline_s=.02,
 geom_timeconst_softened_s=.04,effective_finger_can_pair_timeconst_softened_s=.03,
 fixed='Adaptive transmission parameters, joint limits, original EEF/arm/grip commands, recorded clocks, corrected early yaw, initial positions, contact impedance/friction/damping ratio, shelf and goal.',
 scope='Initial mechanism comparison, not a recovery-rate estimate. Single predefined softness level. No mesh deformation; solver penetration is a contact-compression proxy. Soft can affects its table, shelf and goal contacts too.',
 baseline_repeats='Needed for new contact penetration/force observations; saved trajectories must reproduce previous adaptive baseline.',
 diagnostics=['callback counts','source identity','retention/separation','upright supported release','strict physical sequence','supplied unchanged slide metric','contact penetration and force','real-image seating comparison for 233'])
(ROOT/'plan.json').write_text(json.dumps(plan,indent=2))
(ROOT/'logs').mkdir(exist_ok=True)
def run(job):
 uid,scope=job;name=f'{uid}_{scope}';out=ROOT/name
 if out.exists():raise FileExistsError(out)
 source=REPO/f'paper/eef_recovery_2026-09-09/{"timestamp_full_pool" if uid==233 else "early_yaw_pool"}/{uid}/source.npz'
 command=[str(PYTHON),str(REPO/'can_pos_recovery/run_adaptive_gripper_candidate.py'),str(source),'--out',str(out),'--soft-contact',scope,'--record-contact']
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 print('START',name,flush=True);start=time.time()
 with (ROOT/'logs'/f'{name}.log').open('w') as log:
  p=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
 result=dict(uid=uid,scope=scope,returncode=p.returncode,elapsed_s=time.time()-start,command=command)
 (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(result,indent=2))
 print('FINISHED',name,p.returncode,flush=True);return result
with ThreadPoolExecutor(max_workers=2) as executor:
 results=[future.result() for future in as_completed([executor.submit(run,job) for job in jobs])]
(ROOT/'execution.json').write_text(json.dumps(results,indent=2))
if any(r['returncode']!=0 for r in results):sys.exit(1)
