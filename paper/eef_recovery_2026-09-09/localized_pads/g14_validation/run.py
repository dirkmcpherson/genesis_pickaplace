"""Frozen broader validation: existing nine-demo panel, paired rigid/soft replays."""
from pathlib import Path
import concurrent.futures,subprocess,json,hashlib,time,shutil
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert (R/'can_pos_recovery').exists();selection_path=P/'critical_validation/selection.json';sel=json.loads(selection_path.read_text());uids=sel['old_reserved']+[x['uid'] for x in sel['new_panel']]
assert uids==[176,185,237,156,181,224,198,243,246]
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/align_finger_inertia.py','can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py'];arc=D/'executed_sources';arc.mkdir(exist_ok=True)
for f in files:shutil.copy2(R/f,arc/Path(f).name)
jobs=[]
for uid in uids:
 source=R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 for tc in [.02,.03]:jobs.append(dict(uid=uid,tc=tc,name=f'{uid}_tc{tc:g}',source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
plan=dict(jobs=jobs,workers=4,selection_sha256=hashlib.sha256(selection_path.read_bytes()).hexdigest(),source_code_sha256={s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in files},preset_sha256={str(tc):hashlib.sha256((P/f'critical_return/presets/tc{tc:g}.json').read_bytes()).hexdigest() for tc in [.02,.03]},fixed=dict(engine='1.4.0',cone='pyramidal',impratio=1,neutral_collision=False,feedback='refresh before each spring read and after each scene step'),qualification='Existing closure/day-stratified selected panel, with previous outcomes known. No new-engine results for these9 demos used for selection. Freeze all settings before execution; include all failures. Compare both same-hand rigid and existing original fixed-gripper controls. No population or calibrated-material claim from this panel alone.')
assert not (D/'plan.json').exists();(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 uid,tc,name=job['uid'],job['tc'],job['name'];assert hashlib.sha256(Path(job['source']).read_bytes()).hexdigest()==job['source_sha256']
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py',job['source'],'--out',str(D/name),'--preset',str(P/f'critical_return/presets/tc{tc:g}.json'),'--cone','pyramidal','--impratio','1'];start=time.time();print('START',name,flush=True)
 with (D/'logs'/f'{name}.log').open('w') as f:r=subprocess.run(cmd,cwd=R,stdout=f,stderr=subprocess.STDOUT)
 row=dict(**job,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd);(D/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',name,r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
