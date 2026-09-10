"""Audit-only UID181 repeat: unchanged physics and unchanged numerical tolerance."""
from pathlib import Path
import concurrent.futures,hashlib,json,shutil,subprocess,time
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[4]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (D/'plan.json').exists()
old=json.loads((P/'plan.json').read_text());jobs=[j for j in old['jobs'] if j['uid']==181]
(D/'logs').mkdir();(D/'executed_sources').mkdir()
files=list(old['source_code_sha256'])+['can_pos_recovery/run_g14_normal_pad_damping_v2.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
for f,h in old['source_code_sha256'].items():assert sha(R/f)==h
engine=Path('/tmp/genesis-contact-1.4/lib/python3.10/site-packages/genesis/engine/solvers/rigid/constraint/solver.py')
shutil.copy2(engine,D/'executed_sources/engine_solver.py')
plan=dict(jobs=jobs,source_code_sha256={f:sha(R/f) for f in files},engine_source_sha256={str(engine):sha(engine)},
 purpose='Diagnose v1 ascending J@qvel cancellation by native ancestor-order reconstruction; same1e-5 threshold, unchanged physical delta. Preserve every original attempt.',
 checks=['All four181 conditions rerun.','181 soft1 must reproduce every original NPZ array exactly.','Native reference checks and existing normal target checks must pass; legacy errors and exceedances recorded.','All original source, timing, geometry and nonmutation invariants remain required.'])
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(j):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_normal_pad_damping_v2.py',j['source'],'--out',str(D/j['name']),'--preset',j['preset'],'--gain',str(j['gain'])]
 start=time.time();print('START',j['name'],flush=True)
 with (D/'logs'/f'{j["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**j,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{j["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',j['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
