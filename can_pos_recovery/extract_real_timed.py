"""Timed re-extraction of the real demos from the local ROS bags (pure-python `rosbags`,
runs in .venv-eval). Reproduces trial_reader.py's ~60 Hz windowing EXACTLY (window closes at
the first message of ANY topic with dt >= 1/60 after the window start; a frame is emitted only
if joint_states AND base_feedback both landed in the window), so frame k here == frame k of
inthewild_trials/<uid>_episodes.npy == waypoint k of the contract-v1 src tapes
(baselines/episodes_all_v2r/<uid>.npz drops the LAST real frame) -- but now every frame also
carries its wall-clock time. Nothing in inthewild_trials/<uid>_episodes.npy is touched.

Writes inthewild_trials/<uid>_timed.npz:
  t_frame   (n,)    window-close time [s] of each emitted frame, relative to the bag's first msg
  q_frame   (n,6)   mean joint_states.position[:6] in the window  (== _episodes 'vel_cmd')
  g_frame   (n,)    mean gripper motor position in the window     (== _episodes 'gripper_pos')
  fb_t      (m,)    every base_feedback message time [s]
  fb_tool   (m,6)   tool_pose x,y,z [m], theta_x,y,z [deg] (kortex base frame)
  fb_grip   (m,)    gripper motor position (0 open .. ~80-100 closed)
  cv_t      (c,)    every /cartesian_velocity command message time [s] (joystick activity)
  meta      json: uid, bag duration, message counts

  .venv-eval/bin/python can_pos_recovery/extract_real_timed.py 232 233 ...   (default: all bags)
"""
import json, sys
from pathlib import Path
from collections import defaultdict
import numpy as np
from rosbags.highlevel import AnyReader

REPO = Path(__file__).resolve().parent.parent
RAW = REPO / 'inthewild_trials' / 'raw'
OUT = REPO / 'inthewild_trials'
JOINT_TOPIC = '/my_gen3_lite/joint_states'
FEEDBACK_TOPIC = '/my_gen3_lite/base_feedback'
CARTVEL_TOPIC = '/my_gen3_lite/in/cartesian_velocity'
HZ = 60


def extract(uid):
    path = RAW / f'user_{uid}' / 'trial_data.bag'
    if not path.exists():
        return None
    fr = defaultdict(list); frames = dict(t=[], q=[], g=[])
    fb_t, fb_tool, fb_grip, cv_t = [], [], [], []
    t0 = None; bag_start = None; counts = defaultdict(int); tlast = None
    with AnyReader([path]) as reader:
        want = {c.topic for c in reader.connections if c.topic in (JOINT_TOPIC, FEEDBACK_TOPIC, CARTVEL_TOPIC)}
        conns_by_id = {c.id: c for c in reader.connections}
        for conn, t, raw in reader.messages():        # ALL topics drive the windowing (trial_reader.py)
            # Match ROS1 Time.to_sec(): dividing the combined nanosecond
            # integer rounds differently at rare window boundaries.
            tsec = float(t // 1000000000) + float(t % 1000000000) / 1e9
            if bag_start is None:
                bag_start = tsec
            if not t0:
                t0 = tsec
            tlast = tsec
            if tsec - t0 >= 1.0 / HZ:
                if fr[JOINT_TOPIC] and fr[FEEDBACK_TOPIC]:
                    frames['t'].append(tsec - bag_start)
                    frames['q'].append(np.mean(fr[JOINT_TOPIC], axis=0))
                    frames['g'].append(float(np.mean(fr[FEEDBACK_TOPIC])))
                t0 = tsec
                fr = defaultdict(list)
            if conn.topic not in want:
                continue
            counts[conn.topic] += 1
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == JOINT_TOPIC:
                fr[JOINT_TOPIC].append(list(m.position[:6]))
            elif conn.topic == FEEDBACK_TOPIC:
                b = m.base
                g = m.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
                fr[FEEDBACK_TOPIC].append(float(g))
                fb_t.append(tsec - bag_start); fb_grip.append(float(g))
                fb_tool.append([b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                                b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z])
            elif conn.topic == CARTVEL_TOPIC:
                cv_t.append(tsec - bag_start)
    meta = dict(uid=int(uid), bag_duration=float(tlast - bag_start), counts=dict(counts))
    out = dict(t_frame=np.asarray(frames['t'], np.float64), q_frame=np.asarray(frames['q'], np.float64),
               g_frame=np.asarray(frames['g'], np.float64), fb_t=np.asarray(fb_t, np.float64),
               fb_tool=np.asarray(fb_tool, np.float64), fb_grip=np.asarray(fb_grip, np.float64),
               cv_t=np.asarray(cv_t, np.float64), meta=json.dumps(meta))
    np.savez_compressed(OUT / f'{uid}_timed.npz', **out)
    return out


def check(uid, out):
    """frame-for-frame identity with the reader of record (the tapes the follower consumes)."""
    p = OUT / f'{uid}_episodes.npy'
    if not p.exists():
        return 'no _episodes.npy'
    d = np.load(p, allow_pickle=True).item()
    v = np.asarray(d['vel_cmd']); g = np.asarray(d['gripper_pos'])[:, 0]
    if len(v) != len(out['q_frame']):
        return f'FRAME COUNT {len(out["q_frame"])} vs episodes {len(v)}'
    dq = float(np.abs(v[:, :6] - out['q_frame']).max()); dg = float(np.abs(g - out['g_frame']).max())
    return f'match n={len(v)} dq={dq:.1e} dg={dg:.1e}' if dq < 1e-5 and dg < 1e-3 else f'MISMATCH dq={dq:.2e} dg={dg:.2e}'


if __name__ == '__main__':
    uids = [int(a) for a in sys.argv[1:]] or sorted(int(p.name.split('_')[1]) for p in RAW.glob('user_*'))
    ok = 0
    for u in uids:
        try:
            out = extract(u)
        except Exception as e:
            print(f'{u}: FAILED {type(e).__name__}: {e}', flush=True); continue
        if out is None:
            print(f'{u}: no bag', flush=True); continue
        n = len(out['t_frame']); span = out['t_frame'][-1] - out['t_frame'][0] if n else 0
        print(f'{u}: frames {n} span {span:.1f}s ({n / max(span, 1e-9):.1f} fps) fb {len(out["fb_t"])} '
              f'cv {len(out["cv_t"])} | {check(u, out)}', flush=True)
        ok += 1
    print(f'done {ok}/{len(uids)}')
