"""Independently replay each numerically complete census trace as it arrives."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np


def check_replay(source, checked):
    """Completion alone is insufficient: the saved action stream must reproduce the trace."""
    report_path=checked.parent/'array_comparison.json'
    if report_path.exists():
        return
    meta=json.loads(checked.read_text())
    assert meta['action_replay_verification'] and meta['sequence']['complete']
    with np.load(source) as original, np.load(checked.with_suffix('.npz')) as replay:
        comparison={key:bool(np.array_equal(original[key],replay[key]))
                    for key in ('trajectory','actions_eef','observations')}
    assert all(comparison.values()), (source,comparison)
    report_path.write_text(json.dumps(comparison,indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    p.add_argument('--python', default='/home/james/workspace/genesis_sim2real/venv/bin/python')
    a = p.parse_args()
    repo = Path(__file__).resolve().parents[1]
    uids = json.loads((a.root/'plan.json').read_text())['uids']
    while True:
        collection_finished = 0
        for uid in uids:
            folder = a.root/str(uid)
            execution = folder/'execution.json'
            if not execution.exists():
                continue
            collection_finished += 1
            result = json.loads(execution.read_text())
            if result['returncode'] or not result['sequence']['complete']:
                continue
            source = folder/'collection'/f'{uid}_eef_delta.npz'
            out = folder/'verification'
            checked = out/f'{uid}_eef_delta.json'
            attempt = folder/'verification_execution.json'
            if checked.exists():
                check_replay(source,checked)
                continue
            if attempt.exists():
                continue
            if out.exists():
                raise RuntimeError(f'Unfinished verification requires review: {out}')
            cmd = [a.python, str(repo/'can_pos_recovery/repair_eef_slide.py'), str(source.resolve()),
                   '--out', str(out.resolve()), '--verify-actions']
            with (folder/'verify.log').open('w') as log:
                r = subprocess.run(cmd, cwd=repo, stdout=log, stderr=subprocess.STDOUT)
            report = dict(uid=uid, returncode=r.returncode)
            attempt.write_text(json.dumps(report, indent=2))
            if r.returncode==0:
                check_replay(source,checked)
            print(json.dumps(report), flush=True)
        if collection_finished == len(uids):
            break
        time.sleep(5)


if __name__ == '__main__':
    main()
