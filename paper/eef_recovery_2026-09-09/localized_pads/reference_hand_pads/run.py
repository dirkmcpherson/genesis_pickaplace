"""Paired attribution comparison on the calibration set only."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,hashlib,os,subprocess,time,sys
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
jobs=[(uid,tc) for uid in [113,184,233] for tc in [.02,.03]]
plan=dict(purpose='Separate pad benefit from regressions introduced by the uncalibrated adaptive hand.',jobs=jobs,
 fixed='Original full-world fixed-coupling hand, source motion/grip, timestamps, early yaws, initial placements, geometry, friction and all non-pad parameters.',
 qualification='Original coupling remains a known loaded-curl fidelity limitation; an attribution control is not sufficient to declare the full objective achieved.',
 gate='Each .02 control must reproduce the original reference trajectory exactly. Compare paired full sequences and real-image checks; do not fit on reserved 176/185/237.',
 code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in ['can_pos_recovery/run_reference_hand_pads.py','can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/repair_eef_slide.py']})
(ROOT/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 uid,tc=job;name=f'{uid}_tc{tc:g}';out=ROOT/name
 assert not out.exists()
 source=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_reference_hand_pads.py'),str(source),'--out',str(out),'--pad-timeconst',str(tc)]
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 print('START',name,flush=True);start=time.time()
 with (ROOT/'logs'/f'{name}.log').open('w') as log:result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
 row=dict(name=name,uid=uid,pad_timeconst_s=tc,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
 (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2));print('FINISHED',name,result.returncode,flush=True);return row
with ThreadPoolExecutor(max_workers=2) as pool:rows=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
if any(r['returncode'] for r in rows):sys.exit(1)
