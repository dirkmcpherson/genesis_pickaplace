"""Isolate numerical refinement of the passive hand without contact/gravity."""
from pathlib import Path
import concurrent.futures,hashlib,json,os,shutil,subprocess,time
import numpy as np
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[3]
assert not (D/'plan.json').exists();(D/'executed_sources').mkdir()
files=['can_pos_recovery/bench_g14_hand_resolution.py','can_pos_recovery/bench_g14_feedback.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/align_finger_inertia.py']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
reference=P/'g14_feedback/refreshed/trace.npz'
plan=dict(substeps=[8,16,32],source_code_sha256={f:sha(R/f) for f in files},reference=str(reference),reference_sha256=sha(reference),
 purpose='Same951-step unloaded ramp/hold schedule and hand, no collisions/gravity. Isolate integration sensitivity before attributing full-scene divergence to the controller. Not a real demonstration.',
 gate='Eight-substep control must reproduce archived scene/substep arrays exactly before16/32 run. Physical stiffness/damping/inertia/limits, command schedule and scene interval fixed.')
(D/'plan.json').write_text(json.dumps(plan,indent=2));rows=[]
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
def run(ss):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 out=D/f'ss{ss}';cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/bench_g14_hand_resolution.py','--out',str(out),'--refresh-before-read','--substeps',str(ss)]
 start=time.time();print('START',ss,flush=True)
 with (D/f'ss{ss}.log').open('w') as log:r=subprocess.run(cmd,env=env,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(substeps=ss,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd);(D/f'ss{ss}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',ss,r.returncode,flush=True);return row
rows.append(run(8));(D/'execution.json').write_text(json.dumps(rows,indent=2));assert rows[0]['returncode']==0
a=np.load(D/'ss8/trace.npz');b=np.load(reference);checks={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
(D/'identity.json').write_text(json.dumps(checks,indent=2));assert all(checks.values()),checks
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for f in concurrent.futures.as_completed([pool.submit(run,ss) for ss in [16,32]]):
  rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('UNLOADED_TERMINAL',len(rows),flush=True)
