from pathlib import Path
import subprocess,json,hashlib,time
R=Path('/home/james/workspace/genesis_pickaplace');D=R/'paper/eef_recovery_2026-09-09/localized_pads/g14_full_replay'
conditions=[('pyramidal',1,.02,'233_pyramid_rigid_port3'),('pyramidal',1,.03,'233_pyramid_soft_port3'),('elliptic',10,.02,'233_elliptic10_rigid_port3'),('elliptic',10,.03,'233_elliptic10_soft_port3')]
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py']
(D/'plan_port3.json').write_text(json.dumps({'uid':233,'conditions':conditions,'source_code_sha256':{s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in files},'qualification':'Corrected material coordinate mapping; unchanged source path/grip, scoring, geometry inputs and transmission. Rigid control expected to repeat port2 physics exactly. Soft port2 deliberately interrupted after material frame mismatch discovered.'},indent=2))
records=[]
for cone,ratio,tc,name in conditions:
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py','paper/eef_recovery_2026-09-09/timestamp_full_pool/233/source.npz','--out',str(D/name),'--preset',str(D.parent/f'critical_return/presets/tc{tc:g}.json'),'--cone',cone,'--impratio',str(ratio)]
 print('START',name,flush=True);start=time.time()
 with (D/'logs'/f'{name}.log').open('w') as f:result=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,cmd=cmd))
 (D/'execution_port3.json').write_text(json.dumps(records,indent=2));print('DONE',name,result.returncode,flush=True)
 if result.returncode:break
