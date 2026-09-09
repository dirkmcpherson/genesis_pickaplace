"""Compare raw measurement timestamps with window means on identical replay times."""
import hashlib
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

root = Path(__file__).resolve().parent
repo = root.parents[2]
joint_topic = '/my_gen3_lite/joint_states'
feedback_topic = '/my_gen3_lite/base_feedback'
records = []
for uid in [233, 235, 124, 185]:
    control = root.parent / 'fine_interval' / str(uid)
    meta = json.loads((control / 'source.json').read_text())
    d = root / str(uid)
    d.mkdir(exist_ok=False)
    bag = repo / 'inthewild_trials/raw' / f'user_{uid}' / 'trial_data.bag'
    qt, q, gt, g = [], [], [], []
    start = None
    with AnyReader([bag]) as reader:
        for conn, stamp, raw in reader.messages():
            seconds = float(stamp // 1000000000) + float(stamp % 1000000000) / 1e9
            if start is None:
                start = seconds
            if conn.topic not in (joint_topic, feedback_topic):
                continue
            message = reader.deserialize(raw, conn.msgtype)
            if conn.topic == joint_topic:
                qt.append(seconds - start)
                q.append(message.position[:6])
            else:
                gt.append(seconds - start)
                g.append(message.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)
    qt, q, gt, g = map(np.asarray, (qt, q, gt, g))
    assert np.all(np.diff(qt) > 0) and np.all(np.diff(gt) > 0)
    assert np.max(np.abs(np.diff(q, axis=0))) < np.pi
    timed = np.load(meta['timing_reconstruction']['source'])
    old = np.load(control / 'source.npz')
    sample = old['sample_time_s'] + timed['t_frame'][0]
    new_q = np.stack([np.interp(sample, qt, q[:, j]) for j in range(6)], axis=1)
    new_g = np.interp(sample, gt, g)
    np.savez_compressed(d / 'raw_measurements.npz', joint_time_s=qt, joint_position=q,
                        grip_time_s=gt, grip_position=g)
    np.savez_compressed(d / 'source.npz', joint_waypoints=new_q, source_grip=new_g,
                        sample_time_s=old['sample_time_s'], bag_sample_time_s=sample)
    inherited = meta.pop('timing_reconstruction')
    meta['timing_reconstruction'] = dict(
        method='Separate linear interpolation of raw joint_states and base_feedback motor measurements at their bag receipt timestamps',
        source=str((d / 'raw_measurements.npz').resolve()),
        sha256=hashlib.sha256((d / 'raw_measurements.npz').read_bytes()).hexdigest(),
        decision_dt_s=.01, replay_duration_s=len(sample) * .01,
        paired_window_mean_timing=inherited,
        start_and_end='Same sample times and endpoint as the 10 ms window-mean control',
        limitation='Bag receipt timestamps are not hardware acquisition times; these are measured positions, not original hardware commands',
    )
    meta['source_sha256'] = hashlib.sha256(bag.read_bytes()).hexdigest()
    (d / 'source.json').write_text(json.dumps(meta, indent=2))
    record = dict(uid=uid, joint_samples=len(q), grip_samples=len(g),
                  replay_frames=len(sample),
                  joint_delta_vs_window_mean_p95_rad=float(np.quantile(np.abs(new_q-old['joint_waypoints']), .95)),
                  joint_delta_vs_window_mean_max_rad=float(np.max(np.abs(new_q-old['joint_waypoints']))),
                  grip_delta_vs_window_mean_max=float(np.max(np.abs(new_g-old['source_grip']))),
                  clamped_joint_samples=int(np.sum((sample < qt[0]) | (sample > qt[-1]))),
                  clamped_grip_samples=int(np.sum((sample < gt[0]) | (sample > gt[-1]))))
    records.append(record)
    print(json.dumps(record), flush=True)
(root / 'plan.json').write_text(json.dumps(dict(records=records,
    selection='Same four diagnostics as fine_interval; no outcome-based reselection',
    world='Identical full world, initial positions and 10 ms EEF execution; no og4 or added motion',
    changed='Remove window averaging and window-close timestamp assignment from joint and grip measurements together'), indent=2))
