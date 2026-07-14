"""Extended bag reader: extract the NATIVE 4-DOF Cartesian action signal + the ee pose,
in addition to the joint positions the original trial_reader.py already emits.

Uses the pure-Python `rosbags` library (reads ROS1 bags via the message definitions embedded
in the bag -- NO ROS install needed, handles the custom kortex_driver types). Runs in the
project's .venv-eval. Reads the repo-local bags under inthewild_trials/raw/user_<id>/, writes
a SUPERSET dict to inthewild_trials/<id>_cartesian.npy (does NOT touch <id>_episodes.npy).

Per ~60Hz frame (same windowing + mean as the original reader):
  joint_pos          (6)  mean joint_states.position[:6]   (== original 'vel_cmd'/'joint_pos')
  gripper_pos        (1)  gripper motor position (base_feedback)
  tool_pose          (6)  ee pose x,y,z, theta_x,y,z  (kortex; theta in DEGREES, raw)
  tool_twist         (6)  ACHIEVED ee velocity (base_feedback tool_twist_*)
  cartesian_velocity (6)  COMMANDED ee velocity (/cartesian_velocity TwistCommand.twist),
                          zeros on frames with no command (joystick centered)

Per study lead: /reward and /success are intentionally NOT extracted (untrusted auto-detector).
tool_pose at gripper-open = the real per-demo PLACE location.

Usage:
  .venv-eval/bin/python trial_reader_cartesian.py            # all trials
  .venv-eval/bin/python trial_reader_cartesian.py 232 233    # a subset
"""
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np
from rosbags.highlevel import AnyReader

CARTVEL_TOPIC = '/my_gen3_lite/in/cartesian_velocity'
JOINT_TOPIC = '/my_gen3_lite/joint_states'
FEEDBACK_TOPIC = '/my_gen3_lite/base_feedback'
NEED = (JOINT_TOPIC, FEEDBACK_TOPIC, CARTVEL_TOPIC)
HZ = 60


def _bag_path(trial_id):
    local = Path(__file__).parent / 'inthewild_trials' / 'raw' / f'user_{trial_id}' / 'trial_data.bag'
    if local.exists():
        return local
    return Path('~').expanduser() / f'user_{trial_id}' / 'trial_data.bag'


def feedback13(m):
    b = m.base
    return [b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
            b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z,
            b.tool_twist_linear_x, b.tool_twist_linear_y, b.tool_twist_linear_z,
            b.tool_twist_angular_x, b.tool_twist_angular_y, b.tool_twist_angular_z,
            m.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position]


def twist6(m):
    t = getattr(m, 'twist', m)   # kortex TwistCommand wraps a flat-named Twist
    return [t.linear_x, t.linear_y, t.linear_z, t.angular_x, t.angular_y, t.angular_z]


def read_bag(trial_id, outdir):
    path = _bag_path(trial_id)
    if not path.exists():
        print(f'  {trial_id}: MISSING bag {path}')
        return None
    ep = defaultdict(list)
    frame = defaultdict(list)
    t0 = None
    counts = defaultdict(int)
    ref_frame = None
    with AnyReader([path]) as reader:
        conns = [c for c in reader.connections if c.topic in NEED]
        for conn, t, raw in reader.messages(connections=conns):
            tsec = t / 1e9
            if t0 is None:
                t0 = tsec
            if tsec - t0 >= 1.0 / HZ:
                if frame[JOINT_TOPIC] and frame[FEEDBACK_TOPIC]:
                    jp = np.mean(frame[JOINT_TOPIC], axis=0)
                    fb = np.mean(frame[FEEDBACK_TOPIC], axis=0)
                    cv = (np.mean(frame[CARTVEL_TOPIC], axis=0)
                          if frame[CARTVEL_TOPIC] else np.zeros(6))
                    ep['joint_pos'].append(jp)
                    ep['vel_cmd'].append(jp)
                    ep['tool_pose'].append(fb[:6])
                    ep['tool_twist'].append(fb[6:12])
                    ep['gripper_pos'].append([fb[12]])
                    ep['cartesian_velocity'].append(cv)
                t0 = tsec
                frame = defaultdict(list)
            counts[conn.topic] += 1
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == JOINT_TOPIC:
                frame[conn.topic].append(list(m.position[:6]))
            elif conn.topic == FEEDBACK_TOPIC:
                frame[conn.topic].append(feedback13(m))
            elif conn.topic == CARTVEL_TOPIC:
                frame[conn.topic].append(twist6(m))
                if ref_frame is None:
                    ref_frame = int(getattr(m, 'reference_frame', -1))

    out = {k: np.asarray(v, dtype=np.float32) for k, v in ep.items()}
    out['cartvel_reference_frame'] = np.asarray(ref_frame if ref_frame is not None else -1)
    n = len(out.get('joint_pos', []))
    nz = int(np.any(np.abs(out['cartesian_velocity']) > 1e-6, axis=1).sum()) if n else 0
    Path(outdir).mkdir(parents=True, exist_ok=True)
    np.save(Path(outdir) / f'{trial_id}_cartesian.npy', out)
    print(f'  {trial_id}: {n} frames | cartvel msgs={counts[CARTVEL_TOPIC]} '
          f'(nonzero frames={nz}) | ref_frame={ref_frame}')
    return out


if __name__ == '__main__':
    outdir = Path(__file__).parent / 'inthewild_trials'
    if len(sys.argv) > 1:
        trials = [int(a) for a in sys.argv[1:]]
    else:
        p0 = [232, 235, 242, 245, 248, 251, 254, 257, 261, 265, 269, 273, 276, 279, 283,
              293, 297, 301, 304, 308, 315, 316, 319, 320, 325, 328, 331, 335,
              238, 249, 260, 268, 282, 288, 307, 312, 324, 334]
        p1 = [233, 236, 239, 243, 246, 252, 255, 258, 262, 266, 274, 277, 280, 284, 287,
              294, 298, 302, 305, 309, 317, 321, 326, 329, 322, 270, 313]
        p2 = [327, 330, 333, 300, 303, 306, 311, 318, 278, 281, 286, 290, 295, 299, 256,
              259, 263, 267, 275, 234, 237, 244, 247, 250, 240, 253, 285, 289, 296, 310, 314]
        trials = sorted(set(p0 + p1 + p2))
    print(f'extracting {len(trials)} trials via rosbags -> {outdir}/<id>_cartesian.npy')
    ok = 0
    for tid in trials:
        try:
            if read_bag(tid, outdir) is not None:
                ok += 1
        except Exception as e:
            print(f'  {tid}: FAILED {type(e).__name__}: {e}')
    print(f'done: {ok}/{len(trials)} extracted')
