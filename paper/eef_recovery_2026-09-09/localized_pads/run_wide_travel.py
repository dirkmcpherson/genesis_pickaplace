"""Test the mirrored CAD-right travel alternative with paired pad controls."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
jobs=[(matched,tc) for matched in [False,True] for tc in [.02,.03,.04]]
plan=dict(uid=233,jobs=[dict(match_reference_drive=m,pad_timeconst_s=tc) for m,tc in jobs],
    distal_lower_limit_rad=-1.03,return_stiffness=2.,max_pad_normal_depth_m=.003,
    motivation='Prior narrow-limit candidates saturate the inherited -0.50 rad tip stops. Official right-tip CAD export permits -1.03 rad. Test the additional travel before more material tuning.',
    qualification='Mirror the CAD right-tip range as a finger-symmetry hypothesis. Raw-left CAD limit signs differ from the ROS model; this is not an axis-converted or measured hardware correction. All 233 runs are calibration attempts.',
    fixed='Only distal lower stops change relative to the paired prior drive/pad configurations. Same geometry, inertia, source actions/clocks, initial positions, friction, actuator force limit and timestep.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/run_surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'wide_travel_plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
    matched,tc=job;name=f'233_wide_{"matched" if matched else "original"}_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_surface_pad_candidate.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--pad-timeconst',str(tc),'--distal-lower-limit','-1.03']
    if matched:command.append('--match-reference-drive')
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'wide_travel_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
