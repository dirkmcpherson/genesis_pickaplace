from pathlib import Path
import subprocess,json,hashlib,time,shutil
R=Path('/home/james/workspace/genesis_pickaplace');D=R/'paper/eef_recovery_2026-09-09/localized_pads/g14_feedback_full'
conditions=[('pyramidal',1,.02,'233_pyramid_rigid'),('pyramidal',1,.03,'233_pyramid_soft'),('elliptic',10,.02,'233_elliptic10_rigid'),('elliptic',10,.03,'233_elliptic10_soft')]
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py'];arc=D/'executed_sources';arc.mkdir(exist_ok=True)
for f in files:shutil.copy2(R/f,arc/Path(f).name)
(D/'plan.json').write_text(json.dumps({'uid':233,'conditions':conditions,'source_code_sha256':{s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in files},'qualification':'Fresh feedback fixes verified controller delay. Same material mapping, source path/grip, scoring, geometry inputs and transmission as preceding full-world port. No candidate adoption from one demo.'},indent=2))
records=[]
for cone,ratio,tc,name in conditions:
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py','paper/eef_recovery_2026-09-09/timestamp_full_pool/233/source.npz','--out',str(D/name),'--preset',str(D.parent/f'critical_return/presets/tc{tc:g}.json'),'--cone',cone,'--impratio',str(ratio)]
 print('START',name,flush=True);start=time.time()
 with (D/'logs'/f'{name}.log').open('w') as f:result=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(name=name,uid=233,returncode=result.returncode,elapsed_s=time.time()-start,cmd=cmd))
 (D/'execution.json').write_text(json.dumps(records,indent=2));print('DONE',name,result.returncode,flush=True)
 if result.returncode:break
