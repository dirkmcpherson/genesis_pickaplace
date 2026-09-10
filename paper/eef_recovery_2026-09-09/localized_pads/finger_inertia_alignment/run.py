"""Full calibration pairs after the independent geometry/inertia audit."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib, json, os, subprocess, sys, time

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[3]
POOL = ROOT.parents[1]
assert json.loads((ROOT/'bench_readout.json').read_text())['passed_schedule']
jobs = [(233, .02, True)] + [(uid, tc, False) for uid in [233, 184, 113] for tc in [.02, .03]]
plan = dict(jobs=jobs,
    hypothesis='Align finger COM/inertia coordinate signs to their corresponding meshes; determine whether corrected dynamics change the full-task pad comparison.',
    fixed='Critical-return presets, original collision meshes, source EEF/grip/timing, corrected early yaw, initial placements, world and both immutable scorers.',
    control='233 original-inertia wrapper must reproduce critical_return/233_tc0.02 every saved array exactly.',
    qualification='Mass magnitudes preserved; sign hypothesis supported by mesh integration and mirror geometry, not real mass measurements. No new parameter fit or validation outcomes used to choose signs.',
    code_sha256={n:hashlib.sha256((REPO/n).read_bytes()).hexdigest() for n in ['can_pos_recovery/run_aligned_finger_inertia.py','can_pos_recovery/align_finger_inertia.py','can_pos_recovery/run_hand_preset.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/surface_pad_candidate.py']},
    audit_sha256=hashlib.sha256((ROOT/'audit.json').read_bytes()).hexdigest())
(ROOT/'full_task_plan.json').write_text(json.dumps(plan,indent=2))
(ROOT/'logs').mkdir(exist_ok=True)

def run(job):
    uid,tc,original=job
    name=f'{uid}_'+('original_control' if original else f'tc{tc:g}')
    out=ROOT/name
    assert not out.exists(), out
    source=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
    preset=ROOT.parent/f'critical_return/presets/tc{tc:g}.json'
    cmd=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_aligned_finger_inertia.py'),str(source),'--out',str(out),'--preset',str(preset)]
    if original:cmd.append('--original-inertia')
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    start=time.time();print('START',name,flush=True)
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(cmd,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    row=dict(uid=uid,tc=tc,original=original,name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=cmd,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),preset_sha256=hashlib.sha256(preset.read_bytes()).hexdigest())
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2))
    print('FINISHED',name,result.returncode,flush=True)
    return row

with ThreadPoolExecutor(max_workers=2) as pool:
    rows=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
if any(r['returncode'] for r in rows):sys.exit(1)
