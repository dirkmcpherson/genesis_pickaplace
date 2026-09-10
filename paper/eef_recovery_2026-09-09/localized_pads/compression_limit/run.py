"""Separate pad extent from soft compression travel on calibration cases only."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json,hashlib,os,subprocess,time,sys
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
preset=ROOT.parent/'critical_return/presets/tc0.03.json';jobs=[(233,0)]+[(uid,h) for uid in [184,233,113] for h in [.0005,.001]]
plan=dict(jobs=jobs,preset=str(preset),preset_sha256=hashlib.sha256(preset.read_bytes()).hexdigest(),
 hypothesis='Allowing soft response through the full assumed 3 mm layer may overstate usable compression. Preserve material extent and test earlier stiffening at .5/1 mm; zero limit is exact original-contact control.',
 fixed='Frozen critical-return mechanics, .03 pad time constant, same 3 mm material region and original mesh, source path/grip, full world, clocks, early yaw and ICs. No validation outcome used for fitting.',
 qualification='Compression limits are declared exploratory material settings, not measured rubber strain. No torsional friction or impedance changes. Original incremental stiffness after the limit, not a hard geometric stop.',
 gate='Zero limit must reproduce critical_return/233_tc0.02 exactly. Compare each candidate with same-hand rigid and full-3mm-soft controls on full sequences and real-video fidelity. Do not relax either score.',
 code_sha256={n:hashlib.sha256((REPO/n).read_bytes()).hexdigest() for n in ['can_pos_recovery/finite_compression_pads.py','can_pos_recovery/run_limited_pad_compression.py','can_pos_recovery/run_hand_preset.py']})
(ROOT/'plan.json').write_text(json.dumps(plan,indent=2))
def run(job):
 uid,h=job;name=f'{uid}_limit{h:g}';out=ROOT/name;assert not out.exists()
 source=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/source.npz'
 cmd=['/home/james/workspace/genesis_sim2real/venv/bin/python',str(REPO/'can_pos_recovery/run_limited_pad_compression.py'),str(source),'--out',str(out),'--preset',str(preset),'--compression-limit',str(h)]
 env=os.environ.copy();env.update(TI_CPU_MAX_NUM_THREADS='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1',MPLCONFIGDIR='/tmp/eef-recovery-mpl')
 start=time.time();print('START',name,flush=True)
 with (ROOT/'logs'/f'{name}.log').open('w') as log:r=subprocess.run(cmd,cwd=REPO,env=env,stdout=log,stderr=subprocess.STDOUT)
 row=dict(name=name,uid=uid,compression_limit=h,returncode=r.returncode,elapsed_s=time.time()-start,command=cmd);(ROOT/'logs'/f'{name}_execution.json').write_text(json.dumps(row,indent=2));print('FINISHED',name,r.returncode,flush=True);return row
with ThreadPoolExecutor(max_workers=2) as pool:rows=[f.result() for f in as_completed([pool.submit(run,j) for j in jobs])]
(ROOT/'execution.json').write_text(json.dumps(rows,indent=2))
if any(r['returncode'] for r in rows):sys.exit(1)
