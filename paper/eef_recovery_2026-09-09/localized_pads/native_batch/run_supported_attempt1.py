"""Queue native GPU equivalence/throughput tests after the current CUDA installation exits."""
from pathlib import Path
import os,json,hashlib,subprocess,time,shutil
D=Path(__file__).resolve().parent;R=D.parents[3];P=D.parent
assert not (D/'supported_plan.json').exists()
expected=['/tmp/genesis-contact-1.4/bin/python','-m','pip','install','--target','/tmp/genesis-cuda28']
installer=[]
for p in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  args=p.read_bytes().split(b'\0');args=[a.decode() for a in args if a]
  if args[:len(expected)]==expected:installer.append(int(p.parent.name))
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
assert len(installer)==1,installer
pid=installer[0];proc=Path(f'/proc/{pid}');identity=(proc/'stat').read_text().split()[21]
source233=P/'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
source262=P/'expanded_panel/262_soft_g1/262_eef_delta.npz'
cases=[dict(name='supported_single233',n_envs=0,paths=[source233]),dict(name='supported_single262',n_envs=0,paths=[source262]),dict(name='supported_mixed16',n_envs=16,paths=[source233,source262]),dict(name='supported_mixed64',n_envs=64,paths=[source233,source262])]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=['can_pos_recovery/bench_native_batch_g14_v2.py','can_pos_recovery/batched_surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/replay_harness.py','baselines/sim_variants.py']
A=D/'supported_sources';A.mkdir()
for f in files:shutil.copy2(R/f,A/Path(f).name)
plan=dict(cases=[dict(**c,paths=[str(p) for p in c['paths']]) for c in cases],steps=400,source_code_sha256={f:sha(R/f) for f in files},source_trace_sha256={str(p):sha(p) for p in [source233,source262]},installer_pid=pid,installer_proc_start_ticks=identity,
 purpose='Supported Torch2.8 CUDA stack; compare each distinct source alone against its16/64-lane batch copies. Measure warmed wall time with one GPU test at a time.',
 limits='400-decision common prefixes only; two late-day sources with matching mount/geometry. No e2e recovery claim, no extra unique recordings from duplicate lanes, no GPU admission into the CPU material panel.',
 checks=['Refuse CPU fallback and unsupported Torch.','Duplicate copies within each UID must agree; record exact and numerical errors.','Compare each UID to its standalone GPU trace, with collision geometry/policies identical.','Report all state discrepancies rather than relaxing gates to declare equivalence.','Full-source task export and gain2 batching remain future work.'])
(D/'supported_plan.json').write_text(json.dumps(plan,indent=2));print('WAIT_INSTALLER',pid,flush=True)
while proc.exists():
 try:
  if (proc/'stat').read_text().split()[21]!=identity:break
 except FileNotFoundError:break
 time.sleep(5)
env=dict(os.environ,PYTHONPATH='/tmp/genesis-cuda28',OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
probe=subprocess.run(['/tmp/genesis-contact-1.4/bin/python','-c','import torch,genesis; assert torch.__version__.startswith("2.8.0+"); assert torch.cuda.is_available(); print(torch.__version__,genesis.__version__,torch.cuda.get_device_name(0))'],env=env,cwd=R,text=True,capture_output=True)
(D/'supported_stack_probe.json').write_text(json.dumps(dict(returncode=probe.returncode,stdout=probe.stdout,stderr=probe.stderr),indent=2));assert probe.returncode==0,probe.stderr
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
