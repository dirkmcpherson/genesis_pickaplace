"""Describe completed early slide failures without changing their score or commands."""
import hashlib
import json
from pathlib import Path

import numpy as np

root = Path(__file__).resolve().parent
rows = []
for uid in json.loads((root / 'plan.json').read_text())['uids']:
    directory = root / str(uid)
    execution = directory / 'execution.json'
    if not execution.exists():
        continue
    result = json.loads(execution.read_text())
    sequence = result.get('sequence', {})
    if sequence.get('reason') != 'no_supported_push_to_contact':
        continue
    path = directory / 'collection' / f'{uid}_eef_delta.npz'
    with np.load(path) as data:
        trajectory = data['trajectory']
        contacts = data['contact_counts']
        goals = data['goal_pose']
        can = trajectory[:, 13:16]
        release = sequence['release_start']
        after = slice(release + 3, None)
        gap = np.linalg.norm(can[:, :2] - goals[:, :2], axis=1) - .066
        supported = ((contacts[:, 0] > 0)
                     & (np.abs(can[:, 2] - .2205) <= .004)
                     & (trajectory[:, 20] < 20))
        recontacts = np.flatnonzero(contacts[after, 1] > 0) + release + 3
        rows.append(dict(
            uid=uid,
            day=json.loads((directory / 'source.json').read_text())['day'],
            trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            release_frame=release,
            post_release_robot_contact_frames=int(len(recontacts)),
            first_recontact_frame=int(recontacts[0]) if len(recontacts) else None,
            post_release_supported_robot_contact_frames=int(
                np.sum(supported[after] & (contacts[after, 1] > 0))),
            post_release_goal_contact_frames=int(np.sum(contacts[after, 2] > 0)),
            pre_release_supported_goal_contact_frames=int(
                np.sum(supported[:release] & (contacts[:release, 2] > 0))),
            minimum_post_release_surface_gap_mm=float(gap[after].min() * 1000),
            final_surface_gap_mm=float(gap[-1] * 1000),
            max_post_release_xy_displacement_mm=float(np.max(
                np.linalg.norm(can[after, :2] - can[release, :2], axis=1)) * 1000),
            final_tilt_deg=float(trajectory[-1, 20]),
        ))
report = dict(
    scope='Completed December 16/17 full-world timestamp EEF trials failing the slide',
    limitations='Simulation contact evidence only; no inferred real finger angle or causal calibration.',
    count=len(rows),
    no_recontact_uids=[r['uid'] for r in rows if not r['post_release_robot_contact_frames']],
    records=rows,
)
(root / 'slide_failure_audit.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
