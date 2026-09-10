"""Run the frozen supported-CUDA cases after a verified stack assembly."""
from pathlib import Path
import os,json,hashlib,subprocess,time
D=Path(__file__).resolve().parent;R=D.parents[3];P=D.parent
assert json.loads((D/'supported_stack.json').read_text())['probe']['returncode']==0
plan=json.loads((D/'supported_plan.json').read_text())
cases=[{**c,'paths':[Path(p) for p in c['paths']]} for c in plan['cases']]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (D/'supported_execution.json').exists()
env=dict(os.environ,PYTHONPATH='/tmp/genesis-cuda28',OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
probe=subprocess.run(['/tmp/genesis-contact-1.4/bin/python','-c','import torch,genesis; assert torch.__version__.startswith("2.8.0+"); assert torch.cuda.is_available(); print(torch.__version__,genesis.__version__,torch.cuda.get_device_name(0))'],env=env,cwd=R,text=True,capture_output=True)
(D/'supported_stack_probe_resumed.json').write_text(json.dumps(dict(returncode=probe.returncode,stdout=probe.stdout,stderr=probe.stderr),indent=2));assert probe.returncode==0,probe.stderr
rows=[]
for c in cases:
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 for p in c['paths']:assert sha(p)==plan['source_trace_sha256'][str(p)]
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/bench_native_batch_g14_v2.py','--trace',str(c['paths'][0]),'--preset',str(P/'proximal_compliance/presets/all_soft_control.json'),'--out',str(D/c['name']),'--backend','gpu','--n-envs',str(c['n_envs']),'--steps','400']
 for p in c['paths'][1:]:cmd+=['--other-trace',str(p)]
 start=time.time();print('START',c['name'],flush=True)
 with (D/f'{c["name"]}.log').open('w') as log:r=subprocess.run(cmd,env=env,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(name=c['name'],returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd);rows.append(row)
 (D/'supported_execution.json').write_text(json.dumps(rows,indent=2));print('DONE',c['name'],r.returncode,flush=True)
 if r.returncode:break
print('SUPPORTED_TERMINAL',len(rows),flush=True)
