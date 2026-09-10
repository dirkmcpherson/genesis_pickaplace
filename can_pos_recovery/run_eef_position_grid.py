"""Execute an explicitly saved position-search plan in fresh simulator processes."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('plan', type=Path)
    p.add_argument('--workers', type=int, default=3)
    p.add_argument('--python', default='/home/james/workspace/genesis_sim2real/venv/bin/python')
    a = p.parse_args()
    plan = json.loads(a.plan.read_text())
    root = a.plan.resolve().parent
    repo = Path(__file__).resolve().parents[1]
    if not plan['full_post_hook'] or plan['mode'] != 'joint':
        raise ValueError('This search requires full-hook joint replay')

    def run(item):
        i, offset = item
        out = root / f'candidate_{i:02d}'
        out.mkdir(exist_ok=False)
        args = [a.python, str(repo / 'can_pos_recovery/eef_recovery_probe.py'),
                '--uid', str(plan['uid']), '--mode', plan['mode'], '--out', str(out),
                '--can-offset', *map(str, offset)]
        with (out / 'run.log').open('w') as log:
            proc = subprocess.run(args, cwd=repo, stdout=log, stderr=subprocess.STDOUT)
        return dict(candidate=i, offset=offset, returncode=proc.returncode)

    results = []
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        futures = [pool.submit(run, item) for item in enumerate(plan['offsets_m'])]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(json.dumps(result), flush=True)
    (root / 'execution.json').write_text(json.dumps(results, indent=2))
    if any(r['returncode'] for r in results):
        sys.exit(1)


if __name__ == '__main__':
    main()
