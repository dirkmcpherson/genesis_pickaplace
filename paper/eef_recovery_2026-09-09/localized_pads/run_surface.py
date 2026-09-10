"""Second declared comparison: layered contact without subdividing geometry."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
jobs=[.02,.025,.03,.04]
plan=dict(uid=233,pad_timeconstants_s=jobs,max_normal_depth_m=.003,
    motivation='Every divided-geometry treatment failed supported release, including rigid control. Isolate material law using the original collision geometry and contact generation.',
    law='Local pad surface classification; softer restoring stiffness over assumed thickness, original incremental stiffness beyond backing plane. Damping follows effective time constant.',
    controls='0.02 s must reproduce the previous adaptive trace exactly. Actions, geometry, world, friction and transmission unchanged.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/run_surface_pad_candidate.py',
        'can_pos_recovery/run_adaptive_gripper_candidate.py','can_pos_recovery/slide_predicate.py']})
(ROOT/'surface_plan.json').write_text(json.dumps(plan,indent=2))
def run(tc):
    name=f'233_surface_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_surface_pad_candidate.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--pad-timeconst',str(tc)]
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'surface_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
