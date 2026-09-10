"""Two exact-reference full replays with local every-substep wrench observation."""
from pathlib import Path
import concurrent.futures,subprocess,json,hashlib,shutil,time
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert not (D/'plan.json').exists();(D/'logs').mkdir();(D/'executed_sources').mkdir()
files=['can_pos_recovery/observe_g14_release_forces.py','can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
jobs=[]
for uid,windows in [(113,[8,8.1,19.8,20.5]),(233,[14,14.1,20,25])]:
 source=R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz';ref=P/f'g14_feedback_full/{uid}_elliptic10_soft/{uid}_eef_delta.npz'
 jobs.append(dict(uid=uid,windows=windows,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),reference=str(ref),reference_sha256=hashlib.sha256(ref.read_bytes()).hexdigest()))
plan=dict(jobs=jobs,preset=str(P/'proximal_compliance/presets/all_soft_control.json'),source_code_sha256={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in files},purpose='Distinguish loss of support from finger ejection and twisting during release. Every-substep observation only; require exact archived arrays and force/momentum sign check before interpretation. No treatment or new recovery claim.')
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(j):
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/observe_g14_release_forces.py',j['source'],'--out',str(D/str(j['uid'])),'--preset',plan['preset'],'--windows',*[str(t) for t in j['windows']]];start=time.time();print('START',j['uid'],flush=True)
 with (D/'logs'/f'{j["uid"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**j,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd);(D/'logs'/f'{j["uid"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',j['uid'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for f in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
