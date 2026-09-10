"""Calibrations only: test the inertia hypothesis with paired pad controls."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,hashlib,os,subprocess,time,sys
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
jobs=[(233,'default_control')]+[(uid,f'tc{tc:g}') for uid in [113,184,233] for tc in [.02,.03]]
(ROOT/'plan.json').write_text(json.dumps(dict(jobs=jobs,
 hypothesis='Generic 0.1 kg m2 armature on every finger introduces large unloaded curl/oscillation; this may explain early pickup regressions and slow release.',
 selection='1e-4 kg m2 passed the declared no-contact/no-gravity benchmark; zero and distal-only zero were unstable with current force integration. Selection uses no new full-task outcomes.',
 qualification='Numerical regularization, not calibrated hardware rotor inertia. Same link inertias, mechanics, material regions, source path/grip, clocks, ICs, early yaws and full world.',
 gates='Default-control must reproduce prior 233 rigid trace exactly. Assess pickup, loaded real-image seating, supported release/slide and both immutable scores on calibration113/184/233 before any new reserved comparison.',
 preset_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'presets').glob('*.json')},
 code_sha256={n:hashlib.sha256((REPO/n).read_bytes()).hexdigest() for n in ['can_pos_recovery/run_hand_preset.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/surface_pad_candidate.py']}),indent=2))
def run(job):
 uid,label=job;name=f'{uid}_{label}';out=ROOT/name;assert not out.exists()
 source=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),str(source),'--out',str(out),'--preset',str(ROOT/'presets'/f'{label}.json')]
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 start=time.time();print('START',name,flush=True)
 with (ROOT/'logs'/f'{name}.log').open('w') as log:r=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
 row=dict(name=name,uid=uid,preset=label,returncode=r.returncode,elapsed_s=time.time()-start,command=command);(ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2));print('FINISHED',name,r.returncode,flush=True);return row
with ThreadPoolExecutor(max_workers=2) as pool:rows=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
if any(r['returncode'] for r in rows):sys.exit(1)
