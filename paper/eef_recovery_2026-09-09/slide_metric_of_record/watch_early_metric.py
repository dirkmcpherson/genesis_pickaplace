"""Score finished early collectors and verify new metric passes as they arrive."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time

root = Path(__file__).resolve().parent
pool = root.parent / 'early_yaw_pool'
repo = root.parents[2]
sys.path.insert(0, str(repo / 'can_pos_recovery'))
from score_recovered_slides import adapt
from verify_additional import verify

uids = json.loads((pool / 'plan.json').read_text())['uids']
overlaps = set(json.loads((pool / 'initial_geometry_audit.json').read_text())['overlap_uids'])
scored, submitted, verified, failures = {}, {}, {}, {}
with ThreadPoolExecutor(max_workers=2) as executor:
    while True:
        for uid in uids:
            if uid in scored or uid in failures:
                continue
            d = pool / str(uid)
            if not (d / 'execution.json').exists():
                continue
            execution = json.loads((d / 'execution.json').read_text())
            if execution['returncode']:
                failures[uid] = 'Collector failed; inspect execution.json'
                continue
            row = adapt(d / 'collection' / f'{uid}_eef_delta.npz', root / 'early_watcher_adapters')
            scored[uid] = row
            if row['metric']['slide_success']:
                # Verification can still describe a bad-IC trace, but do not
                # silently treat geometry-induced outcomes as new recovery.
                if uid in overlaps:
                    failures[uid] = 'Metric pass has initial shelf overlap; geometry reconciliation required'
                else:
                    submitted[uid] = executor.submit(verify, row)
        for uid, future in list(submitted.items()):
            if future.done():
                try:
                    verified[uid] = future.result()
                except Exception as error:
                    failures[uid] = str(error)
                del submitted[uid]
        status = dict(declared=len(uids), scored=len(scored),
                      metric_passes=sorted(uid for uid, row in scored.items() if row['metric']['slide_success']),
                      verified=sorted(verified), pending_verification=sorted(submitted),
                      failures=failures,
                      completion='Watcher complete' if len(scored) + sum(uid not in scored for uid in failures) == len(uids) and not submitted else 'Waiting for collectors or independent replay')
        temp = root / 'early_metric_watcher_status.json.tmp'
        temp.write_text(json.dumps(status, indent=2))
        temp.replace(root / 'early_metric_watcher_status.json')
        if status['completion'] == 'Watcher complete':
            print(json.dumps(status), flush=True)
            break
        time.sleep(20)
