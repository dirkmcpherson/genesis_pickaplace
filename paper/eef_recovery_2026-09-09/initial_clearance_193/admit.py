"""Package the single declared geometric correction with explicit provenance."""
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

root = Path(__file__).resolve().parent
bank = root.parents[2] / 'baselines/demos_eef_recovery_2026-09-09'
source = root / 'collection/193_eef_delta.npz'
verified = root / 'metric_verification/193_eef_delta.npz'
review = json.loads((root / 'visual_review.json').read_text())
result = json.loads((root / 'metric_verification_execution.json').read_text())
meta = json.loads(verified.with_suffix('.json').read_text())
original = json.loads((root / 'source.json').read_text())
audit = json.loads((root / 'verification_world_audit.json').read_text())
metric = json.loads((root / 'metric_result.json').read_text())
assert result['verified'] and result['metric']['slide_success']
assert result['metric'] == metric['metric']
assert meta['action_replay_verification'] and meta['precision_ik']
assert meta['extension_m'] == 0 and meta['decision_dt_s'] == .03
assert not any(meta.get(k) for k in ('grip_transform','terminal_hold_frames','hold_contact','physics_treatment'))
assert meta['initial_pose_correction'] == original['initial_pose_correction']
assert meta['can_pos'] == original['can_pos'] and meta['can_pos'][0] == .516
assert abs(audit['built_yaw_deg'] + 19.2) < .01 and audit['full_post_hook']
assert review['reviewed'] and review['source_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
assert Path(review['video']).exists() and Path(review['sheet']).exists()
with np.load(source) as a, np.load(verified) as b:
    for key in ('trajectory','actions_eef','observations'):
        assert np.array_equal(a[key],b[key])
    assert b['observations'].shape == (len(b['actions_eef'])+1,17)
    assert np.isfinite(b['observations']).all() and np.isfinite(b['actions_eef']).all()
    assert abs(float(b['trajectory'][0,13]) - .516) < .001
    assert b['contact_counts'][0,0] == 0
target = bank / '193_clearance_timestamp_reconstruction.npz'
digest = hashlib.sha256(verified.read_bytes()).hexdigest()
if target.exists():
    assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
else:
    shutil.copyfile(verified, target)
sidecar = target.with_suffix('.json')
if sidecar.exists():
    assert json.loads(sidecar.read_text()) == meta
else:
    sidecar.write_text(json.dumps(meta,indent=2))
record = dict(uid=193, role='primary', file=target.name, sha256=digest,
    reconstruction_class='timestamp_with_initial_clearance_correction',
    variant=meta['variant'], day='12-17', metric=metric['metric'],
    strict_contact_diagnostic=meta['sequence'], final_tilt_deg=metric['final_tilt_deg'],
    final_solver_contacts=metric['final_solver_contacts'],
    initial_pose_correction=meta['initial_pose_correction'],
    can_pos=meta['can_pos'], can_quat=meta['can_quat'],
    timing_reconstruction=meta['timing_reconstruction'],
    verification=str(verified.with_suffix('.json')), visual_review=review)
manifest = dict(status='Verified, reviewed development reconstruction with disclosed initial-position correction; not measured ground-truth placement',
    reconstruction_class=record['reconstruction_class'], decision_dt_s=.03,
    independent_source_trials=1, primary_count=1, alternative_count=0, records=[record])
destination = bank / 'manifest_initial_clearance_timestamp.json'
if destination.exists():
    prior = json.loads(destination.read_text())
    assert [r['uid'] for r in prior['records']] == [193]
destination.write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(admitted=193, initial_position_shift_mm=-2.3563592787040344,
                     final_center_distance_mm=metric['metric']['final_dist']*1000)))
