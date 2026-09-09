"""Check reported real tool xyz against URDF FK of real joint measurements."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parent
repo = root.parents[2]
sys.path.insert(0, str(repo / 'baselines'))
from eef_delta_control import ArmKinematics

urdf = repo / 'gen3_lite_2f_robotiq_85.urdf'
fk = ArmKinematics(urdf)
early = json.loads((root.parent / 'early_yaw_pool/plan.json').read_text())['uids']
later = json.loads((root.parent / 'timestamp_full_pool/plan.json').read_text())['uids']


def batch_tool(q):
    transforms = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
    for name, kind, origin, axis, _ in fk.chain:
        transforms = transforms @ origin
        if kind != 'fixed':
            j = int(name.split('_')[-1]) - 1
            rotation = Rotation.from_rotvec(q[:, j, None] * axis).as_matrix()
            transforms[:, :3, :3] = transforms[:, :3, :3] @ rotation
    return transforms[:, :3, 3]


rows = []
for uid in early + later:
    path = repo / 'inthewild_trials' / f'{uid}_cartesian.npy'
    data = np.load(path, allow_pickle=True).item()
    q = np.asarray(data['joint_pos'], dtype=np.float64)
    reported = np.asarray(data['tool_pose'], dtype=np.float64)[:, :3]
    assert q.shape == (len(reported), 6)
    actual = batch_tool(q)
    # Cross-check the vectorized calculation against the controller's scalar FK.
    for index in sorted({0, len(q)//2, len(q)-1}):
        np.testing.assert_allclose(actual[index], fk.tool(q[index], np.eye(4))[:3, 3], atol=1e-12)
    error = actual - reported
    assert np.isfinite(error).all()
    norm = np.linalg.norm(error, axis=1)
    if uid in early:
        day = json.loads((root.parent / 'early_yaw_pool' / str(uid) / 'source.json').read_text())['day']
    else:
        day = '12-18'
    rows.append(dict(uid=uid, day=day, frames=len(q),
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        mean_xyz_error_mm=(error.mean(axis=0)*1000).tolist(),
        position_error_median_mm=float(np.median(norm)*1000),
        position_error_p95_mm=float(np.quantile(norm,.95)*1000),
        position_error_max_mm=float(norm.max()*1000)))
summary = {}
for day in sorted({r['day'] for r in rows}):
    group = [r for r in rows if r['day']==day]
    summary[day] = dict(trials=len(group),
        median_trial_p95_mm=float(np.median([r['position_error_p95_mm'] for r in group])),
        worst_trial_p95_mm=max(r['position_error_p95_mm'] for r in group),
        worst_mean_bias_norm_mm=max(np.linalg.norm(r['mean_xyz_error_mm']) for r in group))
report = dict(urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
    scope='89 early candidates and 74 December18 candidates, real arm base frame',
    limitations='Window-averaged joint states and reported tool xyz can have message timing differences. This checks internal kinematic consistency, not external camera/world calibration or real finger geometry. Euler-angle window means are deliberately not interpreted as an orientation reference.',
    by_day=summary, records=rows)
(root / 'results.json').write_text(json.dumps(report, indent=2))
print(json.dumps(summary, indent=2))
