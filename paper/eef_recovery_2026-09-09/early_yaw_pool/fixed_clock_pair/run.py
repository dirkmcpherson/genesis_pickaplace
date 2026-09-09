"""Paired early-day clock diagnostic; keep full world, EEF control and recorded grip."""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
repo = root.parents[3]
python = '/home/james/workspace/genesis_sim2real/venv/bin/python'
for uid in json.loads((root / 'plan.json').read_text())['uids']:
    d = root / str(uid)
    if (d / 'execution.json').exists():
        continue
    if (d / 'collection').exists():
        raise RuntimeError(f'Inspect partial collection for {uid}')
    with (d / 'run.log').open('w') as log:
        result = subprocess.run([
            python, str(repo / 'can_pos_recovery/repair_eef_slide.py'),
            str(d / 'source.npz'), '--out', str(d / 'collection'),
            '--max-extension', '0', '--polish-ik',
        ], stdout=log, stderr=subprocess.STDOUT)
    record = dict(uid=uid, returncode=result.returncode)
    if result.returncode == 0:
        record['sequence'] = json.loads(
            (d / 'collection' / f'{uid}_eef_delta.json').read_text())['sequence']
    (d / 'execution.json').write_text(json.dumps(record, indent=2))
    print(json.dumps(record), flush=True)
    if result.returncode:
        raise RuntimeError(f'Failed {uid}; inspect log')
