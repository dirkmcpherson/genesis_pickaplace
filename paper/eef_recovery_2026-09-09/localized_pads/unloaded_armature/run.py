from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,os,subprocess,time,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
cases=['default','distal_zero','all_1e-4','all_zero']
(ROOT/'plan.json').write_text(json.dumps(dict(cases=cases,purpose='Test whether generic added joint inertia causes pre-contact curl and slow passive response. No contact, gravity, arm motion or outcome fitting.',candidate_code_sha256=hashlib.sha256((REPO/'can_pos_recovery/bench_unloaded_hand.py').read_bytes()).hexdigest()),indent=2))
def run(case):
 cmd=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/bench_unloaded_hand.py'),'--out',str(ROOT/case),'--armature',case]
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 start=time.time();print('START',case,flush=True)
 with (ROOT/'logs'/f'{case}.log').open('w') as log:r=subprocess.run(cmd,env=env,cwd=REPO,stdout=log,stderr=subprocess.STDOUT)
 row=dict(case=case,returncode=r.returncode,elapsed_s=time.time()-start,command=cmd);(ROOT/'logs'/f'{case}_execution.json').write_text(json.dumps(row,indent=2));print('FINISHED',case,r.returncode,flush=True);return row
with ThreadPoolExecutor(max_workers=2) as pool:rows=[f.result() for f in as_completed([pool.submit(run,c) for c in cases])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
