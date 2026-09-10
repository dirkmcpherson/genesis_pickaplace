"""Frozen expansion to36 additional recordings, four unchanged contact conditions."""
from pathlib import Path
import concurrent.futures,hashlib,json,shutil,subprocess,time,collections
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent;POOL=P.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (D/'plan.json').exists()
used={113,156,176,181,184,185,198,224,233,237,243,246}
by=collections.defaultdict(list);eligible=[]
for pool in ['early_yaw_pool','timestamp_full_pool']:
 for f in sorted((POOL/pool).glob('[0-9]*/source.json')):
  m=json.loads(f.read_text());uid=m['uid']
  if uid in used:continue
  assert f.with_suffix('.npz').exists() and (f.parent/'collection'/f'{uid}_eef_delta.npz').exists()
  day=m.get('day','12-18');row=dict(uid=uid,day=day,source=str(f.with_suffix('.npz')),frames=m['source_frames'],selection_key=hashlib.sha256(f'soft-pad-expanded-2026-09-10:{uid}'.encode()).hexdigest())
  by[day].append(row);eligible.append(row)
selected={day:sorted(v,key=lambda x:x['selection_key'])[:12] for day,v in by.items()}
assert len(selected)==3 and all(len(v)==12 for v in selected.values())
# Interleave days and conditions so a progress snapshot is not a single-day batch.
ordered=[selected[day][k] for k in range(12) for day in sorted(selected)]
files=['can_pos_recovery/run_g14_normal_pad_damping_v3.py','can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/align_finger_inertia.py','can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
(D/'logs').mkdir();(D/'executed_sources').mkdir()
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
jobs=[]
for row in ordered:
 uid=row['uid'];source=Path(row['source']);baseline=source.parent/'collection'/f'{uid}_eef_delta.npz'
 for gain in [1.,2.]:
  for condition in ['rigid','soft']:
   preset=P/f'proximal_compliance/presets/{"all_soft_control" if condition=="soft" else "rigid"}.json'
   jobs.append(dict(**row,condition=condition,gain=gain,name=f'{uid}_{condition}_g{gain:g}',source_sha256=sha(source),source_metadata_sha256=sha(source.with_suffix('.json')),preset=str(preset),preset_sha256=sha(preset),baseline=str(baseline),baseline_sha256=sha(baseline)))
plan=dict(jobs=jobs,uids=[j['uid'] for j in ordered],workers=8,source_code_sha256={f:sha(R/f) for f in files},eligible=eligible,selection=selected,
 purpose='Expand verification to36 additional unique recordings with unchanged four-condition elliptic-contact settings and original fixed references.',
 sampling='Deterministic SHA256 ordering with prefix soft-pad-expanded-2026-09-10; first12 per recording day from existing full-source pools, excluding the12 recordings already used in this material experiment. Selection does not read task outcomes, duration or initial geometry. Existing pools are historical development data; not pristine population sampling.',
 retained='Keep geometric initial-condition defects, long trajectories, task failures and verification stops. No outcome-based exclusions. Report pose-defect flags separately.',
 fixed='Same source motions/times, early corrected yaws, initial poses, original goal/shelf and two metrics. Rigid/soft times.02/.03s and normal gains1/2. No spin friction, new textures, og4 or terminal holds.',
 resource_policy='Eight independent one-thread CPU workers. Prior host check:12CPU threads,31GiB RAM,13GiB available while two1.3GiB verification replays running. Launch only after verification gate is complete; no cluster writes or costs.',
 gate='Native-order input-scaled verifier must finish all five replays and pass exact181 soft1 and176 soft2 controls before workers start.')
(D/'plan.json').write_text(json.dumps(plan,indent=2));print('FROZEN',len(jobs),'jobs',len(ordered),'new UIDs',flush=True)
gate=P/'elliptic_panel/verification_retry_v3'
# This is a queued batch, not a running replay, until the verified gate exists.
while True:
 paths=[gate/'logs'/f'{j["name"]}_execution.json' for j in json.loads((gate/'plan.json').read_text())['jobs']]
 if all(f.exists() for f in paths):break
 time.sleep(5)
assert all(json.loads(f.read_text())['returncode']==0 for f in paths),'Verification gate execution failed'
subprocess.run(['/tmp/genesis-contact-1.4/bin/python',str(gate/'readout.py')],cwd=R,check=True,stdout=(D/'gate_readout.log').open('w'),stderr=subprocess.STDOUT)
verified=json.loads((gate/'merged_summary.json').read_text());assert verified['fully_completed_uids']==9
(D/'gate_passed.json').write_text(json.dumps(dict(merged_summary_sha256=sha(gate/'merged_summary.json'),identities_sha256=sha(gate/'identities.json'),time=time.time()),indent=2))
def run(j):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 for path,key in [(j['source'],'source_sha256'),(str(Path(j['source']).with_suffix('.json')),'source_metadata_sha256'),(j['preset'],'preset_sha256'),(j['baseline'],'baseline_sha256')]:assert sha(path)==j[key]
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_normal_pad_damping_v3.py',j['source'],'--out',str(D/j['name']),'--preset',j['preset'],'--gain',str(j['gain'])]
 start=time.time();print('START',j['name'],flush=True)
 with (D/'logs'/f'{j["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**j,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{j["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',j['name'],r.returncode,flush=True);return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
 for future in concurrent.futures.as_completed([pool.submit(run,j) for j in jobs]):
  rows.append(future.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
print('ALL_TERMINAL',len(rows),flush=True)
