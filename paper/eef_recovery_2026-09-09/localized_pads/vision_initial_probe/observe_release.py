"""Observe the declared conditional rigid/soft release pair without mutation."""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
PARENT = Path(__file__).resolve().parent
OUT = PARENT / 'release_wrenches'
OUT.mkdir(exist_ok=False)
(OUT / 'executed_sources').mkdir()
original = json.loads((PARENT / 'plan.json').read_text())
source = Path(original['source'])
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
assert sha(source) == original['source_npz_sha256']
assert sha(source.with_suffix('.json')) == original['source_metadata_sha256']
files = ['can_pos_recovery/observe_g14_release_forces.py',
         'can_pos_recovery/run_g14_hand.py',
         'can_pos_recovery/surface_pad_candidate_g14.py']
for name in files:
    shutil.copy2(ROOT / name, OUT / 'executed_sources' / Path(name).name)
jobs = []
for job in original['jobs']:
    if job['condition'] == 'original_fixed':
        continue
    ref = PARENT / job['condition'] / '113_eef_delta.npz'
    jobs.append(dict(**job, reference=str(ref), reference_sha256=sha(ref),
                     preset_sha256=sha(job['preset'])))
plan = dict(jobs=jobs, source=str(source), source_npz_sha256=sha(source),
            source_metadata_sha256=sha(source.with_suffix('.json')),
            windows_s=[19.8, 20.6],
            source_code_sha256={f: sha(ROOT / f) for f in files},
            qualification='Exact-reference observation of the conditional image-derived initial pose. No new treatment, pose search or adoption. Require every saved trace array to match before interpreting impulses. Diagnose soft failure versus rigid supported release; both full tasks fail.')
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2))

def run(job):
    cmd = ['/tmp/genesis-contact-1.4/bin/python',
           'can_pos_recovery/observe_g14_release_forces.py', str(source),
           '--out', str(OUT / job['condition']), '--preset', job['preset'],
           '--windows', *map(str, plan['windows_s'])]
    start = time.time()
    print('START', job['condition'], flush=True)
    with (OUT / (job['condition'] + '.log')).open('w') as log:
        result = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    row = dict(**job, cmd=cmd, returncode=result.returncode,
               elapsed_s=time.time()-start)
    (OUT / (job['condition'] + '_execution.json')).write_text(json.dumps(row, indent=2))
    print('DONE', job['condition'], result.returncode, flush=True)
    return row

rows = []
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for future in concurrent.futures.as_completed([pool.submit(run, job) for job in jobs]):
        rows.append(future.result())
        (OUT / 'execution.json').write_text(json.dumps(rows, indent=2))
print('ALL_TERMINAL', len(rows), flush=True)
