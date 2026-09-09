"""Independently replay metric-of-record passes not covered by the contact gate."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
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


def verify(row):
    source = Path(row['source'])
    directory = source.parent.parent / 'metric_verification'
    record_path = source.parent.parent / 'metric_verification_execution.json'
    if record_path.exists():
        prior = json.loads(record_path.read_text())
        if prior['returncode'] or not prior.get('verified'):
            raise RuntimeError(f'Inspect failed verification {record_path}')
        return prior
    if directory.exists():
        raise RuntimeError(f'Inspect partial verification {directory}')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == row['source_sha256']
    with (source.parent.parent / 'metric_verification.log').open('w') as log:
        result = subprocess.run([python, str(repo / 'can_pos_recovery/repair_eef_slide.py'),
            str(source), '--out', str(directory), '--max-extension', '0', '--verify-actions'],
            stdout=log, stderr=subprocess.STDOUT)
    record = dict(uid=row['uid'], returncode=result.returncode, verified=False)
    if result.returncode == 0:
        checked = directory / source.name
        with np.load(source) as a, np.load(checked) as b:
            comparison = {k: bool(np.array_equal(a[k], b[k]))
                          for k in ('trajectory', 'actions_eef', 'observations')}
        metric = adapt(checked, directory / 'metric_adapter')
        record.update(array_comparison=comparison, metric=metric['metric'])
        record['verified'] = all(comparison.values()) and metric['metric'] == row['metric']
    record_path.write_text(json.dumps(record, indent=2))
    print(json.dumps(record), flush=True)
    if not record['verified']:
        raise RuntimeError(f'Independent replay mismatch for {row["uid"]}')
    return record


if __name__ == '__main__':
    report = json.loads((root / 'dec18_timestamp/results.json').read_text())
    selected = [r for r in report['records']
                if r['metric']['slide_success'] and not r['strict_contact_diagnostic']['complete']]
    (root / 'additional_verification_plan.json').write_text(json.dumps(
        dict(uids=[r['uid'] for r in selected], workers=2,
             basis='Passes unchanged slide_predicate with physical tilt >60 degree mapping; existing strict successes already independently verified.'), indent=2))
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(verify, selected))
    (root / 'additional_verification_results.json').write_text(json.dumps(results, indent=2))
