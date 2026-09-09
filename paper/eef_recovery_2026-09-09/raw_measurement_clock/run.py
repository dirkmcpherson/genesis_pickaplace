"""Replay the registered raw-clock sources and verify metric-of-record successes."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

root = Path(__file__).resolve().parent
repo = root.parents[2]
sys.path.insert(0, str(repo / 'can_pos_recovery'))
from score_recovered_slides import adapt

python = '/home/james/workspace/genesis_sim2real/venv/bin/python'
replay = root.parent / 'fine_interval/fine_replay.py'
for row in json.loads((root / 'plan.json').read_text())['records']:
    uid = row['uid']
    d = root / str(uid)
    for mode in ['collection', 'verification']:
        record_path = d / f'{mode}_execution.json'
        if record_path.exists():
            record = json.loads(record_path.read_text())
            if record['returncode']:
                raise RuntimeError(f'Inspect failed {record_path}')
        else:
            if (d / mode).exists():
                raise RuntimeError(f'Inspect partial {d / mode}')
            source = d / 'source.npz' if mode == 'collection' else d / 'collection' / f'{uid}_eef_delta.npz'
            args = [python, str(replay), str(source), '--out', str(d / mode), '--max-extension', '0', '--polish-ik']
            if mode == 'verification':
                args.append('--verify-actions')
            with (d / f'{mode}.log').open('w') as log:
                result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
            record = dict(uid=uid, mode=mode, returncode=result.returncode)
            if result.returncode == 0:
                metric = adapt(d / mode / f'{uid}_eef_delta.npz', d / mode / 'metric_adapter')
                record.update(metric=metric['metric'], strict_contact_diagnostic=metric['strict_contact_diagnostic'])
            record_path.write_text(json.dumps(record, indent=2))
            print(json.dumps(record), flush=True)
            if result.returncode:
                raise RuntimeError(f'Failed {uid} {mode}')
        if not record['metric']['slide_success']:
            if mode == 'verification':
                raise RuntimeError(f'Independent replay lost metric success for {uid}')
            break
        if mode == 'verification':
            with np.load(d / 'collection' / f'{uid}_eef_delta.npz') as a, np.load(d / mode / f'{uid}_eef_delta.npz') as b:
                comparison = {k: bool(np.array_equal(a[k], b[k])) for k in ('trajectory', 'actions_eef', 'observations')}
            assert all(comparison.values()), comparison
            (d / mode / 'array_comparison.json').write_text(json.dumps(comparison, indent=2))
