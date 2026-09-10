"""Frozen hand validation plus independent saved-action replay; no parameter fitting."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
jobs=[(u,tc) for u in [176,185,237] for tc in [.02,.03]]+[(233,.03)]
presets={tc:ROOT/f'fast_return_presets/tc{tc:g}.json' for tc in [.02,.03]}
plan=dict(jobs=[dict(uid=u,pad_timeconst_s=tc) for u,tc in jobs],
    candidate='fast_return_presets/tc0.03.json',
    evidence='Early calibration 113 and 184 both failed pickup with this frozen hand. Reserved 176/185/237 assess generalization without new fitting; historical baseline outcomes were already known.',
    selection='Least compliant tested setting that restores actual supported release/slide and supplied-metric success in 233. 0.04 also passes; its sub-mm endpoint difference is not used for selection.',
    qualification='233 still lacks solver-confirmed goal-can contact (4.19 mm surface gap). Keep that strict failure explicit. New early-day comparison evaluates e2e fidelity and every stage, not just proximity.',
    fixed='All hand parameters frozen; only paired rigid versus soft pad setting differs. Recorded source motion, clocks, corrected early yaw, ICs and world unchanged.',
    preset_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in presets.values()},
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/run_hand_preset.py','can_pos_recovery/surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'validation_plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
    uid,tc=job;name=f'{uid}_validation_tc{tc:g}' if uid!=233 else '233_saved_action_verification';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),
        str(ROOT/'233_fastreturn_tc0.03/233_eef_delta.npz') if uid==233 else str(ROOT.parent/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'),'--out',str(out),'--preset',str(presets[tc])]
    if uid==233:command.append('--verify-actions')
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2))
    print('FINISHED',name,result.returncode,flush=True);return record
with ThreadPoolExecutor(max_workers=2) as pool:
    records=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'validation_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
