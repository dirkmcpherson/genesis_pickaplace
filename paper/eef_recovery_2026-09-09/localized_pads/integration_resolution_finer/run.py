"""Check full-source233 sensitivity to numerical substeps without material tuning."""
from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
import numpy as np
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[3]
assert all(json.loads((P/'integration_resolution/identity.json').read_text()).values())
assert not (D/'plan.json').exists();(D/'executed_sources').mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=['can_pos_recovery/run_g14_hand_resolution_v2.py','can_pos_recovery/run_g14_hand.py','can_pos_recovery/surface_pad_candidate_g14.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/repair_eef_slide.py','can_pos_recovery/eef_task_sequence.py','can_pos_recovery/slide_predicate.py','baselines/eef_delta_control.py']
for f in files:shutil.copy2(R/f,D/'executed_sources'/Path(f).name)
source=P.parent/'timestamp_full_pool/233/source.npz';preset=P/'proximal_compliance/presets/all_soft_control.json';reference=P/'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
plan=dict(uid=233,substeps=[64],source=str(source),source_sha256=sha(source),source_metadata_sha256=sha(source.with_suffix('.json')),preset=str(preset),preset_sha256=sha(preset),reference=str(reference),reference_sha256=sha(reference),source_code_sha256={f:sha(R/f) for f in files},workers=1,
 purpose='Follow-up after nonmonotonic8/16/32 full-task results: add64-substep refinement, without selecting resolution by success. Prior8-substep implementation identity is exact; v2 only extends accepted CLI choices.',
 fixed='Same source duration/commands, initial geometry, masses, hand springs/damping, contact normal parameters, elliptic ratio10 and normal gain1. Scene dt.01, decision dt.03 fixed. This refinement uses64 substeps,0.15625ms integration; prior8/16/32 results remain the references.',
 gates=['First8-substep control must reproduce all14 archived arrays exactly.','Every scene step checks actual substep counts and refreshed feedback; runtime dt verified.','Compare full endpoints, strict sequence, supplied metric and real-image seating before claiming improvement.','Do not choose resolution to increase success count; assess successive refinement and report nonconvergence.'])
(D/'plan.json').write_text(json.dumps(plan,indent=2));rows=[]
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',QD_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
for ss in plan['substeps']:
 for f,h in plan['source_code_sha256'].items():assert sha(R/f)==h
 assert sha(source)==plan['source_sha256'] and sha(preset)==plan['preset_sha256']
 name=f'233_soft_ss{ss}';out=D/name
 cmd=['/tmp/genesis-contact-1.4/bin/python','can_pos_recovery/run_g14_hand_resolution_v2.py',str(source),'--out',str(out),'--preset',str(preset),'--cone','elliptic','--impratio','10','--substeps',str(ss)]
 print('START',name,flush=True);start=time.time()
 with (D/f'{name}.log').open('w') as log:r=subprocess.run(cmd,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
 rows.append(dict(name=name,substeps=ss,returncode=r.returncode,elapsed_s=time.time()-start,cmd=cmd));(D/'execution.json').write_text(json.dumps(rows,indent=2));print('DONE',name,r.returncode,flush=True)
 assert r.returncode==0,'Integration diagnostic failed; later cases not started'
 if ss==8:
  a=np.load(out/'233_eef_delta.npz');b=np.load(reference);assert a.files==b.files
  exact={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
  (D/'identity.json').write_text(json.dumps(exact,indent=2));assert all(exact.values()),exact
print('INTEGRATION_TERMINAL',len(rows),flush=True)
