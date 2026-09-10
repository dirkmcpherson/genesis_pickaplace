"""Frozen spatial compliance hypothesis: proximal pads soft, distal pads original."""
from pathlib import Path
import concurrent.futures,subprocess,json,hashlib,time,shutil
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert (R/'can_pos_recovery').exists()
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py',
 'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/align_finger_inertia.py',
 'can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py',
 'can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
assert not (D/'plan.json').exists()
arc=D/'executed_sources';arc.mkdir();(D/'logs').mkdir();(D/'presets').mkdir()
presets={}
for name,tc,part in [('rigid',.02,'all'),('proximal',.03,'proximal'),('all_soft_control',.03,'all')]:
 p=json.loads((P/f'critical_return/presets/tc{tc:g}.json').read_text());p['compliant_part']=part
 p['spatial_hypothesis']='Blue covering is visible predominantly on proximal fingers in real113. Extra proximal compliance is an uncalibrated material hypothesis; thickness and stiffness of the covering are unknown. Original contact response at distal tips. No extra collision geometry.'
 path=D/'presets'/f'{name}.json';path.write_text(json.dumps(p,indent=2));presets[name]=str(path)
for f in files:shutil.copy2(R/f,arc/Path(f).name)
jobs=[]
for uid in [113,184,233]:
 source=R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 for condition in ['rigid','proximal']+(['all_soft_control'] if uid==233 else []):
  jobs.append(dict(uid=uid,condition=condition,name=f'{uid}_{condition}',source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),preset=presets[condition]))
plan=dict(jobs=jobs,workers=3,source_code_sha256={s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in files},
 preset_sha256={k:hashlib.sha256(Path(v).read_bytes()).hexdigest() for k,v in presets.items()},
 fixed=dict(engine='1.4.0',cone='elliptic',impratio=10,neutral_collision=False,feedback='fresh',source_motion='unchanged',early_yaw='existing corrected mount',initial_positions='unchanged'),
 hypothesis='Restrict the existing extra compliance to proximal inner pads; distal inner pads retain original normal response. Same geometry and shared-actuator model. Chosen from visible wrapping location and early seating evidence, not new outcomes.',
 checks=['All seven terminal executions retained.','Rigid113/233 and all-soft233 must reproduce their archived fresh-feedback elliptic reference arrays exactly.','Per-geometry material activity must exclude distal tips in proximal treatment.','Score unchanged supplied metric, strict full sequence, goal displacement, and seating against independent real annotations.','Advance only if full-task evidence improves; verify saved actions and reserve broader panel before adoption.'],
 qualification='Exploratory selected calibration trio, not population yield. Material values and contact law remain uncalibrated. No can placement, shelf, gripper command, terminal hold or goal threshold adjustment.')
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 for f,sha in plan['source_code_sha256'].items():assert hashlib.sha256((R/f).read_bytes()).hexdigest()==sha
 assert hashlib.sha256(Path(job['source']).read_bytes()).hexdigest()==job['source_sha256']
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py',job['source'],'--out',str(D/job['name']),'--preset',job['preset'],'--cone','elliptic','--impratio','10']
 start=time.time();print('START',job['name'],flush=True)
 with (D/'logs'/f'{job["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**job,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{job["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',job['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 for f in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
