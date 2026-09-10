"""Adapt saved EEF traces to the unchanged slide_predicate metric of record.

Use 120 ms samples to match the original repeat-four human tapes. Keep the actual
endpoint. Report physical tilt and legacy grip-gated tip-proxy results separately.
Neither result automatically admits a tape or establishes physical goal contact.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from slide_predicate import classify


def adapt(path, destination):
    meta = json.loads(path.with_suffix('.json').read_text())
    dt = float(meta['decision_dt_s'])
    stride = round(.12 / dt)
    if stride < 1 or not np.isclose(stride * dt, .12):
        raise ValueError(f'Cannot map {dt} s cadence exactly to 120 ms')
    with np.load(path) as z:
        trajectory = z['trajectory']
        states = z['observations'][1:]
        assert states.shape == (len(trajectory), 17)
        np.testing.assert_allclose(states[:, 8:11], trajectory[:, 13:16], atol=1e-7)
        np.testing.assert_allclose(states[:, 11:15], trajectory[:, 16:20], atol=1e-7)
        np.testing.assert_allclose(states[:, 15:17], z['goal_pose'][:, :2], atol=1e-7)
        # End-of-interval samples; append the actual final observation if the
        # episode duration is not an exact multiple of the reference interval.
        indices = np.arange(stride - 1, len(states), stride)
        if not len(indices) or indices[-1] != len(states) - 1:
            indices = np.r_[indices, len(states) - 1]
        physical_tip = trajectory[indices, 20] > 60.
        # Existing FullTaskEnv.TIP_DEG/GRIP_OPEN values. This is a proxy because
        # recovery does not execute that environment's termination state machine.
        legacy_tip = physical_tip & (z['source_grip'][indices] < 30.)
        payload = dict(uid=meta['uid'], states=states[indices],
                       eef_pos=trajectory[indices, 6:9],
                       sample_time_s=(indices + 1) * dt,
                       source_frame_index=indices)
        destination.mkdir(parents=True, exist_ok=True)
        physical = destination / f"{meta['uid']}.npz"
        legacy = destination / f"{meta['uid']}_legacy_tip_proxy.npz"
        np.savez_compressed(physical, **payload, tipped=physical_tip)
        np.savez_compressed(legacy, **payload, tipped=legacy_tip)
        primary, comparison = classify(physical), classify(legacy)
        return dict(
            uid=meta['uid'], source=str(path.resolve()),
            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_dt_s=dt, scoring_dt_s=.12,
            final_partial_interval=bool(len(indices) > 1 and indices[-1] - indices[-2] != stride),
            mapping='post-action observations; measured URDF tool xyz; physical tilt >60 degrees',
            metric=primary, legacy_tip_proxy_metric=comparison,
            strict_contact_diagnostic=meta['sequence'],
            final_tilt_deg=float(trajectory[-1, 20]),
            final_solver_contacts=z['contact_counts'][-1].tolist(),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pool', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for path in sorted(args.pool.glob('*/collection/*_eef_delta.npz')):
        if path.with_suffix('.json').exists():
            rows.append(adapt(path, args.out / 'adapted'))
    predicate = Path(__file__).with_name('slide_predicate.py')
    report = dict(pool=str(args.pool.resolve()), count=len(rows),
                  predicate_sha256=hashlib.sha256(predicate.read_bytes()).hexdigest(),
                  counts={k: sum(r['metric'][k] for r in rows)
                          for k in ('released', 'pushed', 'arrived', 'slide_success')},
                  success_uids=[r['uid'] for r in rows if r['metric']['slide_success']],
                  tip_mapping_disagreements=[r['uid'] for r in rows
                      if r['metric'] != r['legacy_tip_proxy_metric']],
                  records=rows)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'results.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}))


if __name__ == '__main__':
    main()
