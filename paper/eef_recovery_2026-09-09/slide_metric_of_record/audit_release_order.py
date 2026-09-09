"""Diagnose predicate event order without changing its classification or thresholds."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
records=[]
for cohort,pool in [('dec18','timestamp_full_pool'),('early','early_yaw_pool')]:
    for path in sorted((ROOT.parent/pool).glob('*/collection/*_eef_delta.npz')):
        if not path.with_suffix('.json').exists(): continue
        row=adapt(path,ROOT/'release_order_adapters'/cohort)
        with np.load(ROOT/'release_order_adapters'/cohort/f"{row['uid']}.npz") as z:
            release=row['metric']['release_frame']
            frame=None if release is None else int(z['source_frame_index'][release])
        with np.load(path) as z:
            picked=np.flatnonzero(z['stages'][:,0])
            first_pick=int(picked[0]) if len(picked) else None
            contacts=None if frame is None else z['contact_counts'][frame].tolist()
            height=None if frame is None else float(z['trajectory'][frame,15])
        records.append(dict(cohort=cohort,uid=row['uid'],source_sha256=row['source_sha256'],
            metric=row['metric'],release_trace_frame=frame,first_legacy_picked_frame=first_pick,
            release_before_legacy_pick=bool(frame is not None and first_pick is not None and frame<first_pick),
            release_without_any_legacy_pick=bool(frame is not None and first_pick is None),
            release_contacts_shelf_hand_goal=contacts,release_can_z_m=height,
            strict_contact_diagnostic=row['strict_contact_diagnostic']))
counts={}
for cohort in ['dec18','early']:
    rows=[r for r in records if r['cohort']==cohort]
    counts[cohort]=dict(scored=len(rows),metric_passes=sum(r['metric']['slide_success'] for r in rows),
       release_before_legacy_pick=sum(r['release_before_legacy_pick'] for r in rows),
       passes_release_before_legacy_pick=[r['uid'] for r in rows if r['metric']['slide_success'] and r['release_before_legacy_pick']],
       released_without_any_legacy_pick=[r['uid'] for r in rows if r['release_without_any_legacy_pick']])
report=dict(counts=counts,records=records,predicate_sha256=hashlib.sha256((REPO/'can_pos_recovery/slide_predicate.py').read_bytes()).hexdigest(),
 limitation='Legacy picked flag is itself imperfect, especially for initial overlap. Event ordering is diagnostic only; predicate and all its thresholds remain unchanged. Completed trace snapshot, not a final census.')
(ROOT/'release_order_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(counts,indent=2))
