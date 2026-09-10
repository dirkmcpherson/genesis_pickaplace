"""Fixed lower-force actuator-coordinate candidate, three paired pad settings."""
from pathlib import Path
import os,json,subprocess,time,sys,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
presets=ROOT/'feedback_coordinate_presets';presets.mkdir(exist_ok=False)
transmission=dict(actuator_stiffness=320.,actuator_force_limit=2.,distal_moment_ratio=.5,
                  return_stiffness=.2,proximal_damping=.5,distal_damping=.02)
plan=dict(uid=233,pad_timeconstants_s=[.02,.03,.04],transmission=transmission,
    motivation='Recorded gripper input is motor position feedback. Existing surrogate misses its implied actuator position by roughly 20-36 percentage points while carrying. Test a feasible stroke-sharing ratio and lower return preload before interpreting pad tuning.',
    construction='At representative qp=0.5 and qt=-0.8 rad, ratio 0.5 gives actuator coordinate 0.1, near the unloaded mapping of recorded ~88% closure. This is a feasibility illustration, not an angle measurement or fitted real mechanism.',
    force_scale='Motor generalized-force limit 2 bounds direct proximal actuator torque to 1 Nm and distal to 0.5 Nm. Return stiffness is 0.2 rather than 2. These are uncalibrated exploratory bounds, not manufacturer ratings; resulting contact forces must be inspected.',
    position_control='Stiffness 320 reduces coordinate error while the force cap limits actuator loading. Passive damping remains finite; no teleporting or phase-specific forces.',
    fixed='Same original collision geometry, 3 mm surface-pad depth, mirrored right-CAD wider travel hypothesis, source actions/clocks, world, yaw, initial conditions and friction.',
    qualification='Whole hand candidate changes multiple mechanics together. Only pad effect is isolated within this three-case family. No hardware-calibration or e2e-improvement claim until observed.',
    concurrency='One worker in this diagnostic batch, alongside the separately declared two-worker early-day batch.',
    code_sha256={name:hashlib.sha256((REPO/name).read_bytes()).hexdigest() for name in [
        'can_pos_recovery/run_hand_preset.py','can_pos_recovery/surface_pad_candidate.py',
        'can_pos_recovery/adaptive_gripper_candidate.py','can_pos_recovery/run_adaptive_gripper_candidate.py']})
(ROOT/'feedback_coordinate_plan.json').write_text(json.dumps(plan,indent=2))
records=[]
for tc in plan['pad_timeconstants_s']:
    name=f'233_feedback_tc{tc:g}';out=ROOT/name
    if out.exists():raise FileExistsError(out)
    preset=presets/f'tc{tc:g}.json'
    preset.write_text(json.dumps(dict(transmission=transmission,pad_timeconst_s=tc,max_pad_normal_depth_m=.003,
                                    distal_lower_limit_rad=-1.03,qualification=plan['qualification']),indent=2))
    command=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_hand_preset.py'),
        str(ROOT.parent/'timestamp_full_pool/233/source.npz'),'--out',str(out),'--preset',str(preset)]
    env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    print('START',name,flush=True);start=time.time()
    with (ROOT/'logs'/f'{name}.log').open('w') as log:
        result=subprocess.run(command,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
    record=dict(name=name,returncode=result.returncode,elapsed_s=time.time()-start,command=command)
    (ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(record,indent=2));records.append(record)
    print('FINISHED',name,result.returncode,flush=True)
(ROOT/'feedback_coordinate_execution.json').write_text(json.dumps(records,indent=2))
if any(r['returncode'] for r in records):sys.exit(1)
