"""Gate the full native task exporter on CPU identity and GPU prefix repeatability."""
from pathlib import Path
import hashlib,json,os,subprocess,time
import numpy as np
D=Path(__file__).resolve().parent;R=D.parents[3];P=D.parent
plan=json.loads((D/'full_task_plan.json').read_text());assert (D/'full_task_execution.json').exists()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=json.loads((D/'full_task_execution.json').read_text())
assert len(rows)==1 and rows[0]['name']=='full_cpu233' and rows[0]['returncode']==0
checks=json.loads((D/'full_task_cpu_equivalence.json').read_text())
assert all(checks[k] for k in ['physical_states_exact','contacts_exact','picked_exact','strict_sequence_exact']) and checks['tool_max_abs_difference']==0
assert len(json.loads((D/'deterministic_execution.json').read_text()))==4
for i,c in enumerate(plan['cases'][1:],start=1):
 if i==1:
  deadline=time.monotonic()+900
  while True:
   p=D/'deterministic_execution.json';ex=json.loads(p.read_text()) if p.exists() else []
   if any(r['returncode'] for r in ex):raise RuntimeError('Deterministic prefix implementation failed; GPU full cases not started')
   if len(ex)==4:break
   if time.monotonic()>deadline:raise TimeoutError('Prefix still incomplete; no automatic restart')
   time.sleep(5)
  a=np.load(D/'deterministic_mixed16/trace.npz');b=np.load(D/'deterministic_repeat16/trace.npz')
  exact={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
  (D/'full_task_gpu_gate.json').write_text(json.dumps(dict(exact_prefix_arrays=exact),indent=2))
  assert all(exact.values()),'Deterministic prefix repeat differs; GPU full cases not started'
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 for path in c['paths']:assert sha(path)==plan['source_trace_sha256'][path]
 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 if c['backend']=='gpu':env['PYTHONPATH']='/tmp/genesis-cuda28'
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/bench_native_full_task_g14.py','--trace',c['paths'][0],'--preset',str(P/'proximal_compliance/presets/all_soft_control.json'),'--out',str(D/c['name']),'--backend',c['backend'],'--n-envs',str(c['n_envs']),'--steps',str(plan['steps'])]
 print('START',c['name'],flush=True);start=time.time()
 with (D/f'{c["name"]}.log').open('w') as log:r=subprocess.run(cmd,env=env,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 rows.append(dict(name=c['name'],returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd));(D/'full_task_execution.json').write_text(json.dumps(rows,indent=2))
 print('DONE',c['name'],r.returncode,flush=True)
 assert r.returncode==0,'Export execution failed; later cases not started'
 if i==0:
  z=np.load(D/c['name']/'trace.npz');d=np.load(D/c['name']/'full_task_diagnostic.npz');ref=np.load(c['paths'][0])
  physical=np.c_[ref['trajectory'][:,:6],ref['finger_joint'],ref['trajectory'][:,13:20],ref['goal_pose']]
  checks=dict(physical_states_exact=bool(np.array_equal(z['state'][:,0],physical)),contacts_exact=bool(np.array_equal(d['contact_counts'][:,0],ref['contact_counts'])),picked_exact=bool(np.array_equal(d['picked'][:,0],ref['stages'][:,0])),tool_max_abs_difference=float(abs(d['trajectory'][:,0,6:13]-ref['trajectory'][:,6:13]).max()))
  report=json.loads((D/c['name']/'report.json').read_text());meta=json.loads(Path(c['paths'][0]).with_suffix('.json').read_text())
  checks['strict_sequence_exact']=report['task_results'][0]['sequence']==meta['sequence']
  (D/'full_task_cpu_equivalence.json').write_text(json.dumps(checks,indent=2))
  assert checks['physical_states_exact'] and checks['contacts_exact'] and checks['picked_exact'] and checks['strict_sequence_exact'] and checks['tool_max_abs_difference']<1e-12,checks
print('FULL_TASK_TERMINAL',len(rows),flush=True)
