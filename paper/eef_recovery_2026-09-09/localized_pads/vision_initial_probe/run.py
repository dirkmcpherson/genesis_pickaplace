"""Conditional image-derived initial XY, frozen before any changed-pose replay."""
from pathlib import Path
import concurrent.futures,subprocess,hashlib,json,shutil,time
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert not (D/'plan.json').exists();(D/'logs').mkdir();(D/'source').mkdir();(D/'executed_sources').mkdir()
measurement=P/'g14_release_diagnostic/113_initial_plane_sensitivity.json';fit=json.loads(measurement.read_text())
original=R/'paper/eef_recovery_2026-09-09/early_yaw_pool/113/source.npz';source=D/'source/113.npz';shutil.copy2(original,source)
meta=json.loads(original.with_suffix('.json').read_text());old=meta['can_pos'].copy();meta['can_pos'][:2]=fit['nominal']['estimated_initial_xy_m']
meta['initial_pose_provenance']='Conditional image-only nominal XY fit; assumes existing pick-table plane and cap-camera model. Reset height unchanged. Not adopted or fitted to simulation success.'
meta['initial_pose_measurement']=dict(path=str(measurement),sha256=hashlib.sha256(measurement.read_bytes()).hexdigest(),nominal=fit['nominal'],original_can_pos=old)
source.with_suffix('.json').write_text(json.dumps(meta,indent=2));assert source.read_bytes()==original.read_bytes()
files=['can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
jobs=[dict(condition='rigid',preset=str(P/'proximal_compliance/presets/rigid.json')),
      dict(condition='soft',preset=str(P/'proximal_compliance/presets/all_soft_control.json')),
      dict(condition='original_fixed',preset=None)]
plan=dict(uid=113,source=str(source),source_npz_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_metadata_sha256=hashlib.sha256(source.with_suffix('.json').read_bytes()).hexdigest(),original_can_pos=old,conditional_can_pos=meta['can_pos'],measurement_sha256=hashlib.sha256(measurement.read_bytes()).hexdigest(),source_code_sha256={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in files},jobs=jobs,
 qualification='Nominal image-only estimate declared before outcome. No XY search, no passing-value selection, no change to source q/grip/times, yaw, goal, shelf, reset z or scoring. Pinned original fixed hand is a cross-engine reference; rigid/soft1.4 pair isolates pad response at identical conditional placement. Any improvement remains conditional on unresolved table/camera assumptions and needs independent visual and broader validation. No bank adoption.')
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 out=D/job['condition']
 if job['condition']=='original_fixed':cmd=['/home/james/workspace/genesis_sim2real/venv/bin/python','can_pos_recovery/repair_eef_slide.py',str(source),'--out',str(out),'--max-extension','0','--polish-ik']
 else:cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand.py',str(source),'--out',str(out),'--preset',job['preset'],'--cone','elliptic','--impratio','10']
 start=time.time();print('START',job['condition'],flush=True)
 with (D/'logs'/f'{job["condition"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**job,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd);(D/'logs'/f'{job["condition"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',job['condition'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 for f in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
