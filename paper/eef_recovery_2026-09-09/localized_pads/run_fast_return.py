"""Reduce passive return-mode damping while retaining the lower-force grasp."""
from pathlib import Path
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
presets=ROOT/'fast_return_presets';presets.mkdir(exist_ok=False)
source_preset=json.loads((ROOT/'feedback_coordinate_presets/tc0.02.json').read_text())
source_preset['transmission'].update(proximal_damping=.05,distal_damping=.002)
plan=dict(uid=233,pad_timeconstants_s=[.02,.03,.04],transmission=source_preset['transmission'],
    motivation='Previous lower-force hand matches recorded motor position and real loaded-can projection, but its slow internal relaxation leaves fingers catching the can during withdrawal.',
    derivation='At fixed actuator coordinate, symmetric null direction [-1,1,-2,-2] has return stiffness 0.7011904. Damping falls from 1.16 to 0.116, reducing quasistatic D/K from 1.654 to 0.165 s. This is a mode approximation, not an exact contact settling prediction.',
    fixed='Same actuator stiffness/force cap, transmission ratio, return stiffness, geometry, travel, pad depth, source commands/clocks, ICs and friction as feedback_coordinate_plan. Only passive damping differs, with paired pad settings.',
    guard='Original substep callback and finite-state/velocity guards remain. Inspect maximum joint velocity before interpreting recovery.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/run_hand_preset.py','can_pos_recovery/surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'fast_return_plan.json').write_text(json.dumps(plan,indent=2))
records=[]
for tc in plan['pad_timeconstants_s']:
    name=f'233_fastreturn_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    preset=presets/f'tc{tc:g}.json';configuration={**source_preset,'pad_timeconst_s':tc}
    preset.write_text(json.dumps(configuration,indent=2))
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--preset',str(preset)]
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2));records.append(record)
    print('FINISHED',name,result.returncode,flush=True)
(ROOT/'fast_return_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
