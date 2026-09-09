"""Serial 10 ms target-cadence experiment with independent replay of successes."""
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

root = Path(__file__).resolve().parent
python = '/home/james/workspace/genesis_sim2real/venv/bin/python'
for entry in json.loads((root / 'code_manifest.json').read_text()):
    p = root / Path(entry['experiment']).name
    assert hashlib.sha256(p.read_bytes()).hexdigest() == entry['experiment_sha256']
for entry in json.loads((root / 'plan.json').read_text())['records']:
    uid = entry['uid']
    d = root / str(uid)
    for mode in ['collection', 'verification']:
        record_path = d / f'{mode}_execution.json'
        if record_path.exists():
            record = json.loads(record_path.read_text())
            if record['returncode']:
                raise RuntimeError(f'Previous failure: {record_path}')
        else:
            if (d / mode).exists():
                raise RuntimeError(f'Inspect partial run {d / mode}')
            source = d / 'source.npz' if mode == 'collection' else d / 'collection' / f'{uid}_eef_delta.npz'
            args = [python, str(root / 'fine_replay.py'), str(source), '--out', str(d / mode),
                    '--max-extension', '0', '--polish-ik']
            if mode == 'verification':
                args.append('--verify-actions')
            with (d / f'{mode}.log').open('w') as log:
                result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
            record = dict(uid=uid, mode=mode, returncode=result.returncode)
            if result.returncode == 0:
                record['sequence'] = json.loads((d / mode / f'{uid}_eef_delta.json').read_text())['sequence']
            record_path.write_text(json.dumps(record, indent=2))
            print(json.dumps(record), flush=True)
            if result.returncode:
                raise RuntimeError(f'Failed {uid} {mode}')
        if not record['sequence']['complete']:
            if mode == 'verification':
                raise RuntimeError(f'Independent replay lost completion for {uid}')
            break
        if mode == 'verification':
            with np.load(d / 'collection' / f'{uid}_eef_delta.npz') as a, np.load(d / mode / f'{uid}_eef_delta.npz') as b:
                comparison = {k: bool(np.array_equal(a[k], b[k])) for k in ('trajectory', 'actions_eef', 'observations')}
            assert all(comparison.values()), comparison
            (d / mode / 'array_comparison.json').write_text(json.dumps(comparison, indent=2))
