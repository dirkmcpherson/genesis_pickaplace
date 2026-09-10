from pathlib import Path
import concurrent.futures,subprocess,json,hashlib,time
R=Path('/home/james/workspace/genesis_pickaplace');D=R/'paper/eef_recovery_2026-09-09/localized_pads/g14_feedback_full';base=json.loads((D/'plan.json').read_text())
for path,sha in base['source_code_sha256'].items():assert hashlib.sha256((R/path).read_bytes()).hexdigest()==sha
jobs=[(113,tc,f'113_elliptic10_{"rigid" if tc==.02 else "soft"}') for tc in [.02,.03]]
(D/'plan_ell113.json').write_text(json.dumps({'conditions':jobs,'cone':'elliptic','impratio':10,'selection':'Corrected pyramidal113 pair still loses carry before12s; direct test of previously declared fixed-pad creep lead under fresh controller feedback. Same parameters as completed233 elliptic pair, no tuning to113 outcomes.','source_code_sha256':base['source_code_sha256']},indent=2))
def run(job):
 uid,tc,name=job;cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py',f'paper/eef_recovery_2026-09-09/early_yaw_pool/{uid}/source.npz','--out',str(D/name),'--preset',str(D.parent/f'critical_return/presets/tc{tc:g}.json'),'--cone','elliptic','--impratio','10'];start=time.time();print('START',name,flush=True)
 with (D/'logs'/f'{name}.log').open('w') as f:r=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 print('DONE',name,r.returncode,flush=True);return dict(name=name,uid=uid,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution_ell113.json').write_text(json.dumps(rows,indent=2))
