"""Match reference coupled stiffness/damping, with paired pad controls."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
jobs=[(k,tc) for k in [2.,4.,8.] for tc in [.02,.03,.04]]
plan=dict(uid=233,jobs=[dict(return_stiffness=k,pad_timeconst_s=tc) for k,tc in jobs],
    motivation='Prior adaptive motor/damping scales do not match the original position-controlled hand even in its coupled direction. All six spring-only bracket runs failed the full sequence. Match those reference coefficients before judging the spring/pad combination.',
    derivation='j=[-1,1,-0.676,-0.676], a=[-0.5,0.5,0.1,0.1]. K_motor=sum(kp*j^2)/(a.j)^2. D_prox=(sum(kv*j^2)-D_dist*sum(j_dist^2))/2.',
    expected=dict(actuator_stiffness=155.85151310030028,proximal_damping=14.56062048),
    fixed='Same original geometry, pad surface/depth, recorded actions, source clocks, initial positions, friction, actuator force limit and distal damping. No goal-conditioned or phase-conditioned forces.',
    qualification='Reference-mode match below saturation, not full dynamic equivalence or manufacturer calibration. Compare each soft treatment to its matched-drive original-contact control; all are calibration attempts on 233.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/run_surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'matched_drive_plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
    k,tc=job;name=f'233_drive_k{k:g}_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_surface_pad_candidate.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--pad-timeconst',str(tc),'--return-stiffness',str(k),'--match-reference-drive']
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'matched_drive_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
