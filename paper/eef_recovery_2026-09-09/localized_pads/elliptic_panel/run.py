"""Frozen day-balanced elliptic contact panel, separating softness and normal damping."""
from pathlib import Path
import concurrent.futures,hashlib,json,shutil,subprocess,time
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert not (D/'plan.json').exists()
# Finish the implementation comparison before starting the broader panel.
previous=json.loads((P/'normal_damping/summary.json').read_text())
assert previous['all_terminal'] and previous['completed_full_replays']==10
assert all(r['status']=='complete' for r in previous['records'])
(D/'logs').mkdir();(D/'executed_sources').mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=['can_pos_recovery/run_g14_normal_pad_damping.py','can_pos_recovery/run_g14_hand.py',
       'can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py',
       'can_pos_recovery/align_finger_inertia.py','can_pos_recovery/repair_eef_slide.py',
       'can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py',
       'baselines/eef_delta_control.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
jobs=[]
uids=[176,185,237,156,181,224,198,243,246]
for uid in uids:
 source=R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 baseline=source.parent/'collection'/f'{uid}_eef_delta.npz'
 for gain in [1.,2.]:
  for condition in ['rigid','soft']:
   preset=P/f'proximal_compliance/presets/{"all_soft_control" if condition=="soft" else "rigid"}.json'
   jobs.append(dict(uid=uid,day='Dec16' if uid<=181 else 'Dec17' if uid<233 else 'Dec18',condition=condition,gain=gain,name=f'{uid}_{condition}_g{gain:g}',source=str(source),source_sha256=sha(source),source_metadata_sha256=sha(source.with_suffix('.json')),preset=str(preset),preset_sha256=sha(preset),baseline=str(baseline),baseline_sha256=sha(baseline)))
plan=dict(jobs=jobs,uids=uids,workers=3,source_code_sha256={f:sha(R/f) for f in files},
 purpose='Evaluate the previously unmeasured broad elliptic-contact effect separately from pad softness and normal damping. Four same-engine conditions across the existing balanced9-demo panel; original fixed-hand references retained.',
 selection='Same9 recordings used in prior pyramidal panel:3 per recording day. These are not pristine unseen cases or independent participants. No new UID selection or parameter choice from this panel outcome.',
 rationale='Repeated fitting to113/184/233 risks a narrow conclusion, especially with independently identified initial-pose uncertainty. Testing a frozen broader panel is evidence gathering, not adoption or a claim that the current calibration near miss completes the task.',
 fixed='Genesis1.4 elliptic ratio10; same adaptive mechanics and collision geometry; original placements, corrected early yaws, source arm/grip motion, shelf, goal and both scoring definitions. No conditional initial-position variants.',
 checks=['Every normal/tangent mapping and unchanged-reference/material invariant must pass.','Verify all source/metadata/URDF hashes, targets, grip, frame coverage, no holds/extensions and fresh feedback.','Within eachUID all four collision policies and geometry hashes match.','Report original/rigid1/soft1/rigid2/soft2 complete tasks, supplied metric, goal motion, and wins/losses perday.','Retain all declared runs and implementation failures. Do not count pending runs as failures.','Any apparent improvement needs independent saved-action and annotated real-video validation; this panel is not a population estimate.'])
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 assert sha(job['source'])==job['source_sha256'] and sha(Path(job['source']).with_suffix('.json'))==job['source_metadata_sha256']
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_normal_pad_damping.py',job['source'],'--out',str(D/job['name']),'--preset',job['preset'],'--gain',str(job['gain'])]
 start=time.time();print('START',job['name'],flush=True)
 with (D/'logs'/f'{job["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**job,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{job["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',job['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
