"""Account for every declared census UID, including pending and failed workers."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def summarize(root):
    plan = json.loads((root/'plan.json').read_text())
    records = []
    for uid in plan['uids']:
        folder = root/str(uid)
        sidecar = folder/'collection'/f'{uid}_eef_delta.json'
        row = dict(uid=uid, status='pending')
        if not sidecar.exists():
            execution = folder/'execution.json'
            if execution.exists():
                row.update(status='worker_failed', execution=json.loads(execution.read_text()))
            records.append(row)
            continue
        meta = json.loads(sidecar.read_text())
        with np.load(sidecar.with_suffix('.npz')) as data:
            rows = data['trajectory']; contacts = data['contact_counts']
            fingers = data['finger_joint']; goals = data['goal_pose']
            residual = fingers[:, 2:] - (-.676*fingers[:, 1, None] + .149)
            row.update(status=meta['sequence']['reason'], sequence=meta['sequence'],
                       frames=meta['frames'], picked=bool(data['stages'][:, 0].any()),
                       mimic_max_deviation_deg=float(np.rad2deg(np.abs(residual).max())),
                       observation_shape=list(data['observations'].shape),
                       command_xyz_m_p99=meta['command_xyz_m_p99'])
            row['final_tilt_deg'] = float(rows[-1, 20])
            row['final_goal_z_m'] = float(goals[-1, 2])
            row['diagnostic'] = row['status']
            if row['status'] == 'no_supported_release':
                upright_shelf = (contacts[:, 0]>0)&(rows[:, 20]<20)
                row['diagnostic'] = ('no_sustained_upright_separation' if upright_shelf.any()
                                     else 'no_upright_shelf_contact')
            release = meta['sequence']['release_start']
            if release is not None:
                after = np.arange(len(rows)) > release+3
                row['goal_contact_after_release'] = bool(((contacts[:, 2]>0)&after).any())
                if row['status'] == 'no_supported_push_to_contact':
                    if not ((contacts[:, 1]>0)&after).any():
                        row['diagnostic'] = 'no_robot_reengagement_after_release'
                    elif not row['goal_contact_after_release']:
                        row['diagnostic'] = 'no_goal_contact_after_release'
                    else:
                        row['diagnostic'] = 'goal_contact_but_support_or_motion_rule_failed'
                supported = (contacts[:, 0]>0)&(np.abs(rows[:, 15]-.2205)<.004)&(rows[:, 20]<20)
                push = np.flatnonzero(supported & (contacts[:, 1]>0))
                push = push[push >= release+3]
                if len(push):
                    gaps = np.linalg.norm(rows[push, 13:15]-goals[push, :2], axis=1)-.066
                    row.update(last_supported_robot_contact=int(push[-1]),
                               min_gap_during_supported_robot_contact_m=float(gaps.min()))
            verification = folder/'verification'/sidecar.name
            if verification.exists():
                checked = json.loads(verification.read_text())
                with np.load(verification.with_suffix('.npz')) as replay:
                    row['action_verification'] = dict(
                        complete=checked['sequence']['complete'],
                        trajectory_identical=bool(np.array_equal(rows, replay['trajectory'])),
                        actions_identical=bool(np.array_equal(data['actions_eef'], replay['actions_eef'])),
                        observations_identical=bool(np.array_equal(data['observations'], replay['observations'])))
        records.append(row)
    counts = dict(Counter(r['status'] for r in records))
    return dict(declared=len(records), finished=sum(r['status'] not in ('pending', 'worker_failed') for r in records),
                counts=counts, diagnostics=dict(Counter(r.get('diagnostic',r['status']) for r in records)),
                acceptance='Numerical census only; complete is not visual bank acceptance', records=records)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path)
    a=p.parse_args()
    result=summarize(a.root)
    (a.root/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))


if __name__=='__main__':
    main()
