"""Frozen expanded validation, paired material controls and saved-action replay."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,hashlib,os,subprocess,time,sys
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1];presets=ROOT.parent/'critical_return/presets'
selection=json.loads((ROOT/'selection.json').read_text());uids=selection['old_reserved']+[r['uid'] for r in selection['new_panel']]
for name,sha in selection['preset_sha256'].items():assert hashlib.sha256((presets/name).read_bytes()).hexdigest()==sha
jobs=[(233,.03,True)]+[(uid,tc,False) for uid in uids for tc in [.02,.03]]
(ROOT/'execution_plan.json').write_text(json.dumps(dict(jobs=jobs,workers=4,selection_sha256=hashlib.sha256((ROOT/'selection.json').read_bytes()).hexdigest(),code_sha256={n:hashlib.sha256((REPO/n).read_bytes()).hexdigest() for n in ['can_pos_recovery/run_hand_preset.py','can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/surface_pad_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']}),indent=2))
def run(job):
 uid,tc,verify=job;name=f'{uid}_verify' if verify else f'{uid}_tc{tc:g}';out=ROOT/name;assert not out.exists()
 source=ROOT.parent/'critical_return/233_tc0.03/233_eef_delta.npz' if verify else POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),str(source),'--out',str(out),'--preset',str(presets/f'tc{tc:g}.json')]
 if verify:command.append('--verify-actions')
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 start=time.time();print('START',name,flush=True)
 with (ROOT/'logs'/f'{name}.log').open('w') as log:r=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
 row=dict(name=name,uid=uid,tc=tc,verify_actions=verify,returncode=r.returncode,elapsed_s=time.time()-start,command=command);(ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2));print('FINISHED',name,r.returncode,flush=True);return row
with ThreadPoolExecutor(max_workers=4) as pool:rows=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
if any(r['returncode'] for r in rows):sys.exit(1)
