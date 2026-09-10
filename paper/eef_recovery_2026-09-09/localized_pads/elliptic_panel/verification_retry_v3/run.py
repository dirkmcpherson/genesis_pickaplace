"""Audit-only UID181 repeat: unchanged physics and unchanged numerical tolerance."""
from pathlib import Path
import concurrent.futures,hashlib,json,shutil,subprocess,time
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[4]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (D/'plan.json').exists()
old=json.loads((P/'plan.json').read_text());jobs=[j for j in old['jobs'] if j['uid']==181 or (j['uid']==176 and j['condition']=='soft' and j['gain']==2)]
(D/'logs').mkdir();(D/'executed_sources').mkdir()
files=list(old['source_code_sha256'])+['can_pos_recovery/run_g14_normal_pad_damping_v3.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
for f,h in old['source_code_sha256'].items():assert sha(R/f)==h
engine=Path('/tmp/genesis-contact-1.4/lib/python3.10/site-packages/genesis/engine/solvers/rigid/constraint/solver.py')
shutil.copy2(engine,D/'executed_sources/engine_solver.py')
plan=dict(jobs=jobs,source_code_sha256={f:sha(R/f) for f in files},engine_source_sha256={str(engine):sha(engine)},
 purpose='Diagnose v1 ascending J@qvel cancellation by native ancestor-order reconstruction; input-scaled1e-5 reference error plus1e-6 independent Jacobian entry verification, unchanged physical delta. Preserve every original attempt.',
 checks=['All four181 conditions rerun.','181 soft1 and176 soft2 must reproduce every original NPZ array exactly.','Input-scaled reference and target errors must pass at1e-5; independently reconstructed Jacobian entries must match at1e-6. Output-scaled and legacy errors retained.','All original source, timing, geometry and nonmutation invariants remain required.'])
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(j):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_normal_pad_damping_v3.py',j['source'],'--out',str(D/j['name']),'--preset',j['preset'],'--gain',str(j['gain'])]
 start=time.time();print('START',j['name'],flush=True)
 with (D/'logs'/f'{j["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**j,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{j["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',j['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
