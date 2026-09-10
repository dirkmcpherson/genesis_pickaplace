"""Frozen finite-area pad spin pilot; controls separate row layout from material."""
from pathlib import Path
import concurrent.futures,hashlib,json,os,shutil,subprocess,time
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[3]
assert not (D/'plan.json').exists()
(D/'logs').mkdir();(D/'executed_sources').mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=['can_pos_recovery/run_g14_pad_spin.py','can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
engine=Path('/tmp/genesis-contact-1.4/lib/python3.10/site-packages/genesis')
efiles=['utils/geom.py','engine/solvers/rigid/constraint/solver.py','engine/solvers/rigid/rigid_solver.py']
for f in efiles:shutil.copy2(engine/f,D/'executed_sources'/('engine_'+Path(f).name))
jobs=[]
for uid in [233,113]:
 source=P.parent/f'{"early_yaw_pool" if uid==113 else "timestamp_full_pool"}/{uid}/source.npz'
 for spin in [0.,.002]:
  for condition in ['rigid','soft']:
   preset=P/f'proximal_compliance/presets/{"rigid" if condition=="rigid" else "all_soft_control"}.json'
   jobs.append(dict(uid=uid,name=f'{uid}_{condition}_spin{spin:g}',condition=condition,spin_length_m=spin,
    source=str(source),source_sha256=sha(source),source_metadata_sha256=sha(source.with_suffix('.json')),
    preset=str(preset),preset_sha256=sha(preset),reference=str(P/f'g14_feedback_full/{uid}_elliptic10_{condition}/{uid}_eef_delta.npz')))
plan=dict(jobs=jobs,workers=2,source_code_sha256={f:sha(R/f) for f in files},engine_source_sha256={f:sha(engine/f) for f in efiles},
 hypothesis='Finite pad patch supports torsional friction capacity spin_length times normal force. Test0 and0.002m with matching rigid/soft normal laws. Effective3mm uniform disk at mu1 motivates0.002m; patch size is unmeasured, not calibrated.',
 selection='Exploratory233/113: preserved late completion and early release failure, selected from existing diagnostics. Not independent validation.',
 fixed='Recorded joint/grip commands and duration, early yaw, initial can pose, goal/shelf, meshes, hand transmission, sliding friction and both task metrics. Normal damping gain1. No stage-conditioned material or terminal holds.',
 checks=['All global geom torsional coefficients zero; only existing inner-pad contacts get treatment at every time/object.','Four-row zero-spin controls quantify engine-layout effects relative to archived three-row references; do not assume exact equivalence.','Selected/unselected coefficients and row counts verified during replay; retain every failed attempt.','Compare rigid versus soft at the same spin length; no adoption from selected pilot.'],
 progress_classification='New physical hypothesis execution, separate from unchanged expanded panel and GPU port.')
(D/'plan.json').write_text(json.dumps(plan,indent=2))
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
def run(j):
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 for f,h in plan['engine_source_sha256'].items():assert sha(engine/f)==h
 assert sha(j['source'])==j['source_sha256'] and sha(j['preset'])==j['preset_sha256']
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_pad_spin.py',j['source'],'--out',str(D/j['name']),'--preset',j['preset'],'--spin-length',str(j['spin_length_m'])]
 start=time.time();print('START',j['name'],flush=True)
 with (D/'logs'/f'{j["name"]}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
 row=dict(**j,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd)
 (D/'logs'/f'{j["name"]}_execution.json').write_text(json.dumps(row,indent=2));print('DONE',j['name'],r.returncode,flush=True);return row
rows=[]
# First controls gate runtime correctness before treated runs begin.
for wave in [jobs[:2],jobs[2:]]:
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
  for f in concurrent.futures.as_completed([pool.submit(run,j) for j in wave]):
   rows.append(f.result());(D/'execution.json').write_text(json.dumps(rows,indent=2))
 if any(r['returncode'] for r in rows):
  (D/'implementation_gate_failed.json').write_text(json.dumps(dict(completed=len(rows),reason='Execution/verification failed. Unstarted jobs are not task failures.'),indent=2));raise SystemExit(1)
print('ALL_TERMINAL',len(rows),flush=True)
