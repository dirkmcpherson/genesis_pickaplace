"""Independent saved-action replay of the frozen passing validation candidate."""
from pathlib import Path
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
name='176_saved_action_verification';out=ROOT/name;source=ROOT/'176_validation_tc0.03/176_eef_delta.npz';preset=ROOT/'fast_return_presets/tc0.03.json'
assert not out.exists()
command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),str(source),'--out',str(out),'--preset',str(preset),'--verify-actions']
plan=dict(name=name,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),preset_sha256=hashlib.sha256(preset.read_bytes()).hexdigest(),purpose='Replay saved actions without source path reconstruction or parameter fitting.',command=command)
(ROOT/'verify_176_plan.json').write_text(json.dumps(plan,indent=2))
env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
start=time.time();print('START',name,flush=True)
with (ROOT/'logs'/f'{name}.log').open('w') as log:result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
(ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2));print('FINISHED',name,result.returncode,flush=True);sys.exit(result.returncode)
