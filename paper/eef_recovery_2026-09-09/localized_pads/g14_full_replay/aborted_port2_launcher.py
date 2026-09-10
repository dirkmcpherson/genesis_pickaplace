from pathlib import Path
import subprocess,json,hashlib,time
R=Path('/home/james/workspace/genesis_pickaplace');D=R/'paper/eef_recovery_2026-09-09/localized_pads/g14_full_replay'
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py']
plan={'uid':233,'purpose':'First full-world compatibility and four-condition contact comparison; no e2e adoption from one demo','conditions':['233_pyramid_rigid_port2','233_pyramid_soft','233_elliptic10_rigid','233_elliptic10_soft'],'source_code_sha256':{s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in files},'known_control':'pyramid rigid completed, not_picked; remaining conditions declared before execution'}
(D/'plan.json').write_text(json.dumps(plan,indent=2))
records=[{'name':'233_pyramid_rigid_port1','returncode':1,'session_id':88092,'qualification':'Pre-step assertion compared principal tensor against link-frame tensor; no physics result.'},{'name':'233_pyramid_rigid_port2','returncode':0,'session_id':56061}]
for cone,ratio,tc,name in [('pyramidal',1,.03,'233_pyramid_soft'),('elliptic',10,.02,'233_elliptic10_rigid'),('elliptic',10,.03,'233_elliptic10_soft')]:
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py','paper/eef_recovery_2026-09-09/timestamp_full_pool/233/source.npz','--out',str(D/name),'--preset',str(D.parent/f'critical_return/presets/tc{tc:g}.json'),'--cone',cone,'--impratio',str(ratio)]
 print('START',name,flush=True);start=time.time()
 with (D/'logs'/f'{name}.log').open('w') as f:result=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,cmd=cmd))
 (D/'execution.json').write_text(json.dumps(records,indent=2));print('DONE',name,result.returncode,flush=True)
 if result.returncode:break
