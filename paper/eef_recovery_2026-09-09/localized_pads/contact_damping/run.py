"""Frozen twofold pad damping: original calibration trio and conditional diagnostic."""
from pathlib import Path
import concurrent.futures, hashlib, json, shutil, subprocess, time
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
assert not (D/'plan.json').exists()
(D/'logs').mkdir();(D/'executed_sources').mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=['can_pos_recovery/run_g14_pad_damping.py','can_pos_recovery/run_g14_hand.py',
       'can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py',
       'can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py',
       'can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
engine=Path('/tmp/genesis-contact-1.4/lib/python3.10/site-packages/genesis')
engine_files=['utils/geom.py','engine/solvers/rigid/constraint/solver.py']
for f in engine_files:shutil.copy2(engine/f,D/'executed_sources'/('engine_'+Path(f).name))
jobs=[]
for uid,placement in [(113,'conditional'),(233,'original'),(113,'original'),(184,'original')]:
 source=P/'vision_initial_probe/source/113.npz' if placement=='conditional' else R/f'paper/eef_recovery_2026-09-09/{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 for condition,gain in ([('soft',1.)] if (uid,placement) in [(113,'conditional'),(233,'original')] else [])+[('rigid',2.),('soft',2.)]:
  preset=P/f'proximal_compliance/presets/{"all_soft_control" if condition=="soft" else "rigid"}.json'
  reference=None
  if gain==1:
   reference=P/'vision_initial_probe/soft/113_eef_delta.npz' if placement=='conditional' else P/'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
  jobs.append(dict(uid=uid,placement=placement,condition=condition,gain=gain,name=f'{uid}_{placement}_{condition}_g{gain:g}',source=str(source),source_sha256=sha(source),source_metadata_sha256=sha(source.with_suffix('.json')),preset=str(preset),preset_sha256=sha(preset),reference=str(reference) if reference else None,reference_sha256=sha(reference) if reference else None))
plan=dict(jobs=jobs,workers=3,source_code_sha256={f:sha(R/f) for f in files},engine_source_sha256={f:sha(engine/f) for f in engine_files},
 hypothesis='Double classified inner-pad contact velocity damping at fixed elastic coefficient. Both normal and tangential damping change. Soft .03 and matching rigid .02 controls; no extra geometry or friction-coefficient change.',
 fixed='Genesis1.4 elliptic ratio10; fresh feedback, existing hand, original source motion/early yaw, shelf, goal and metrics. Original placements for calibration. One previously declared image-derived conditional113 placement remains diagnostic and unadopted.',
 checks=['Two identity controls must match every archived array.','All treated contacts verify engine reference-acceleration static term unchanged, normal and tangent velocity terms doubled, and time constant above safety floor.','All ten executions and failures retained; full source commands, goal displacement, supplied metric and strict sequence audited.','No adoption from conditional placement or one selected success; compare original and same-damping rigid controls and expand validation before admission.'],
 qualification='One declared damping level, exploratory selected calibration trio plus conditional113 diagnostic. Material coefficients unmeasured. Preset prose spatial_hypothesis is stale; authoritative compliant_part=all and runtime audits apply to all four fingers.')
(D/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 assert sha(job['source'])==job['source_sha256'] and sha(Path(job['source']).with_suffix('.json'))==job['source_metadata_sha256']
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_pad_damping.py',job['source'],'--out',str(D/job['name']),'--preset',job['preset'],'--gain',str(job['gain'])]
 start=time.time();print('START',job['name'],flush=True)
 with (D/'logs'/f'{job["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**job,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{job["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',job['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
 for f in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
