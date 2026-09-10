from pathlib import Path
import concurrent.futures,subprocess,json,hashlib,time
R=Path('/home/james/workspace/genesis_pickaplace');D=R/'paper/eef_recovery_2026-09-09/localized_pads/g14_feedback_full';base=json.loads((D/'plan.json').read_text())
for path,sha in base['source_code_sha256'].items():assert hashlib.sha256((R/path).read_bytes()).hexdigest()==sha
jobs=[(uid,tc,f'{uid}_pyramid_{"rigid" if tc==.02 else "soft"}') for uid in [113,184] for tc in [.02,.03]]
(D/'plan_early.json').write_text(json.dumps({'conditions':jobs,'cone':'pyramidal','impratio':1,'selection':'Both prior early-day calibration demos, frozen critical-return hand/pad parameters. The fresh-feedback233 pyramidal soft condition completes; its rigid pair misses retention. No early outcomes inspected before this declaration. Earlier pinned outcomes known.','source_code_sha256':base['source_code_sha256'],'max_workers':2},indent=2))
def run(job):
 uid,tc,name=job;cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py',f'paper/eef_recovery_2026-09-09/early_yaw_pool/{uid}/source.npz','--out',str(D/name),'--preset',str(D.parent/f'critical_return/presets/tc{tc:g}.json'),'--cone','pyramidal','--impratio','1']
 print('START',name,flush=True);start=time.time()
 with (D/'logs'/f'{name}.log').open('w') as f:r=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 print('DONE',name,r.returncode,flush=True);return dict(name=name,uid=uid,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 futures=[pool.submit(run,j) for j in jobs]
 for future in concurrent.futures.as_completed(futures):
  rows.append(future.result());(D/'execution_early.json').write_text(json.dumps(rows,indent=2))
