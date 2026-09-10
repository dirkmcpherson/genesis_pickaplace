"""Package reviewed timestamp EEF reconstructions under the supplied slide metric."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

repo = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cohort', choices=('dec18', 'early'), default='dec18')
args = parser.parse_args()
root = repo / 'paper/eef_recovery_2026-09-09/slide_metric_of_record'
bank = repo / 'baselines/demos_eef_recovery_2026-09-09'
report = json.loads((root / f'{args.cohort}_timestamp/results.json').read_text())
if args.cohort == 'early':
    pool = root.parent / 'early_yaw_pool'
    reviews = [json.loads(p.read_text()) for p in pool.glob('*/metric_visual_review.json')]
    overlaps = json.loads((pool / 'initial_geometry_audit.json').read_text())['overlap_uids']
else:
    reviews = json.loads((root / 'visual_reviews.json').read_text())
    overlaps = []
predicate = repo / 'can_pos_recovery/slide_predicate.py'
assert hashlib.sha256(predicate.read_bytes()).hexdigest() == report['predicate_sha256']
records = []
pending = []
for review in reviews:
    if not review['reviewed']:
        continue
    uid = review['uid']
    assert uid not in overlaps, 'Initial geometry needs reconciliation before admission'
    scored = next(r for r in report['records'] if r['uid'] == uid)
    assert scored['metric']['slide_success']
    source = Path(scored['source'])
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == scored['source_sha256'] == review['source_sha256']
    folder = source.parent.parent
    verification = folder / 'metric_verification' / source.name
    if not verification.exists():
        verification = folder / 'verification' / source.name
    if not verification.with_suffix('.json').exists():
        pending.append(uid)
        continue
    meta = json.loads(verification.with_suffix('.json').read_text())
    original = json.loads((folder / 'source.json').read_text())
    assert meta['action_replay_verification'] and meta['precision_ik']
    assert meta['extension_m'] == 0 and meta['decision_dt_s'] == .03
    assert not any(meta.get(k) for k in ('grip_transform', 'terminal_hold_frames', 'physics_treatment', 'hold_contact'))
    assert meta['can_pos'] == original['can_pos'] and meta['can_quat'] == original['can_quat']
    expected_variant = 'gc_kp4_riser3_shelf6'
    if args.cohort == 'early':
        expected_variant += '_yaw' + original['day'][-2:]
        audit = json.loads((folder / 'world_audit.json').read_text())
        assert audit['full_post_hook']
        assert abs(audit['built_yaw_deg'] - {'12-16': -9.7, '12-17': -19.2}[original['day']]) < .01
    assert meta['variant'] == expected_variant
    with np.load(source) as a, np.load(verification) as b:
        for key in ('trajectory', 'actions_eef', 'observations'):
            assert np.array_equal(a[key], b[key]), (uid, key)
        assert b['observations'].shape == (len(b['actions_eef']) + 1, 17)
        assert b['actions_eef'].shape[1] == 7
        assert np.isfinite(b['actions_eef']).all() and np.isfinite(b['observations']).all()
    assert Path(review['video']).exists() and Path(review['sheet']).exists()
    target = bank / f'{uid}_timestamp_reconstruction.npz'
    sha = hashlib.sha256(verification.read_bytes()).hexdigest()
    if target.exists():
        assert hashlib.sha256(target.read_bytes()).hexdigest() == sha
    else:
        shutil.copyfile(verification, target)
    sidecar = target.with_suffix('.json')
    if sidecar.exists():
        assert json.loads(sidecar.read_text()) == meta
    else:
        sidecar.write_text(json.dumps(meta, indent=2))
    records.append(dict(uid=uid, file=target.name, sha256=sha,
        reconstruction_class='timestamp_reconstruction', role='primary',
        frames=meta['frames'], metric=scored['metric'],
        strict_contact_diagnostic=scored['strict_contact_diagnostic'],
        final_tilt_deg=scored['final_tilt_deg'], final_solver_contacts=scored['final_solver_contacts'],
        timing_reconstruction=meta['timing_reconstruction'], can_pos=meta['can_pos'],
        day=original.get('day', '12-18'), variant=meta['variant'],
        initial_pose_provenance=original.get('initial_pose_provenance', 'Archived estimated initial position'),
        can_quat=meta['can_quat'], precision_ik=True,
        generation=str(source.with_suffix('.json')), verification=str(verification.with_suffix('.json')),
        visual_review=review))
manifest = dict(status='Verified and visually reviewed development reconstructions; physical parameters and initial positions are not ground-truth calibrated',
    reconstruction_class='timestamp_reconstruction', decision_dt_s=.03,
    cohort=args.cohort,
    world='gc_kp4_riser3_shelf6' if args.cohort == 'dec18' else 'Full world with recorded-day yaw; see each record',
    metric='slide_predicate.py, unchanged constants, 120 ms schema adapter with physical tilt >60 degree flag',
    predicate_sha256=report['predicate_sha256'], independent_source_trials=len(records),
    primary_count=len(records), alternative_count=0,
    qualification='Metric pass, exact independent saved-EEF-action replay, and recorded visual review; proximity is not persistent solver contact',
    records=sorted(records, key=lambda r: r['uid']))
destination = bank / ('manifest_slide_metric_timestamp.json' if args.cohort == 'dec18'
                      else 'manifest_early_slide_metric_timestamp.json')
if destination.exists():
    previous = json.loads(destination.read_text())
    assert {r['uid'] for r in previous['records']} <= {r['uid'] for r in records}, 'Refuse to silently remove admitted records'
temporary = destination.with_suffix('.json.tmp')
temporary.write_text(json.dumps(manifest, indent=2))
temporary.replace(destination)
print(json.dumps(dict(admitted=[r['uid'] for r in records], pending_verification=pending)))
