"""Extended bag reader: extract the NATIVE 4-DOF Cartesian action signal + the ee pose,
in addition to the joint positions the original trial_reader.py already emits.

Runs in the ROS1 env (needs `rosbag`), against ~/user_<id>/trial_data.bag OR the local
copies under inthewild_trials/raw/user_<id>/trial_data.bag. Writes a SUPERSET dict to
inthewild_trials/<id>_cartesian.npy -- it does NOT touch the original <id>_episodes.npy.

Per frame (same ~60Hz windowing + mean as the original), it records:
  joint_pos          (6)  mean joint_states.position[:6]        -- == original 'vel_cmd'
  gripper_pos        (1)  gripper motor position (base_feedback)
  tool_pose          (6)  ee pose: x,y,z, theta_x,theta_y,theta_z  (kortex; theta in DEG)
  tool_twist         (6)  ACHIEVED ee velocity (base_feedback tool_twist_*)
  cartesian_velocity (6)  COMMANDED ee velocity (/my_gen3_lite/in/cartesian_velocity),
                          zeros on frames with no command (joystick centered)

Per the study lead: /reward and /success are intentionally NOT extracted (auto-detector,
untrusted). tool_pose at the moment the gripper opens = the real PLACE location per demo.

Notes on robustness (message types can't be introspected outside ROS1):
  * cartesian_velocity may be geometry_msgs/Twist (.linear.x) OR kortex TwistCommand
    (.twist.linear_x) -- `_twist6` handles both.
  * theta_* are degrees in kortex tool_pose; left raw here, convert downstream if needed.

Usage:
  python trial_reader_cartesian.py                # all trials in the lists below
  python trial_reader_cartesian.py 232 233 245    # a subset
"""
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np
import rosbag

CARTVEL_TOPIC = '/my_gen3_lite/in/cartesian_velocity'
JOINT_TOPIC = '/my_gen3_lite/joint_states'
FEEDBACK_TOPIC = '/my_gen3_lite/base_feedback'
HZ = 60


def _bag_path(trial_id):
    """Prefer the repo-local copy, fall back to the home dir (original layout)."""
    local = Path(__file__).parent / 'inthewild_trials' / 'raw' / f'user_{trial_id}' / 'trial_data.bag'
    if local.exists():
        return local
    return Path('~').expanduser() / f'user_{trial_id}' / 'trial_data.bag'


def joints6(msg):
    return list(msg.position[:6])


def feedback13(msg):
    """[tool_pose(6), tool_twist(6), gripper(1)] from BaseCyclic_Feedback."""
    b = msg.base
    tool_pose = [b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                 b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z]
    tool_twist = [b.tool_twist_linear_x, b.tool_twist_linear_y, b.tool_twist_linear_z,
                  b.tool_twist_angular_x, b.tool_twist_angular_y, b.tool_twist_angular_z]
    gripper = msg.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
    return tool_pose + tool_twist + [gripper]


def _twist6(msg):
    """Commanded cartesian velocity -> [lx,ly,lz,ax,ay,az]. Handles geometry_msgs/Twist
    and kortex TwistCommand (which wraps a flat-named Twist under .twist)."""
    t = getattr(msg, 'twist', msg)
    if hasattr(t, 'linear_x'):   # kortex flat Twist
        return [t.linear_x, t.linear_y, t.linear_z, t.angular_x, t.angular_y, t.angular_z]
    return [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]


def read_bag(trial_id, outdir):
    path = _bag_path(trial_id)
    if not Path(path).exists():
        print('  MISSING bag: %s' % path)
        return None
    bag = rosbag.Bag(str(path))
    must_have = [JOINT_TOPIC, FEEDBACK_TOPIC]   # cartesian_velocity is optional (zeros when idle)
    ep = defaultdict(list)
    seen = defaultdict(int)
    t0 = None
    frame = defaultdict(list)
    for topic, msg, t in bag.read_messages():
        if t0 is None:
            t0 = t.to_sec()
        seen[topic] += 1
        if topic == JOINT_TOPIC:
            frame[topic].append(joints6(msg))
        elif topic == FEEDBACK_TOPIC:
            frame[topic].append(feedback13(msg))
        elif topic == CARTVEL_TOPIC:
            frame[topic].append(_twist6(msg))

        if t.to_sec() - t0 >= 1.0 / HZ:
            if all(len(frame[k]) > 0 for k in must_have):
                jp = np.mean(frame[JOINT_TOPIC], axis=0)
                fb = np.mean(frame[FEEDBACK_TOPIC], axis=0)
                cv = (np.mean(frame[CARTVEL_TOPIC], axis=0)
                      if frame[CARTVEL_TOPIC] else np.zeros(6))
                ep['joint_pos'].append(jp)
                ep['vel_cmd'].append(jp)                 # backward-compat alias
                ep['tool_pose'].append(fb[:6])
                ep['tool_twist'].append(fb[6:12])
                ep['gripper_pos'].append([fb[12]])
                ep['cartesian_velocity'].append(cv)
            t0 = t.to_sec()
            frame = defaultdict(list)
    bag.close()

    out = {k: np.asarray(v, dtype=np.float32) for k, v in ep.items()}
    n = len(out.get('joint_pos', []))
    n_cmd = int(np.any(np.abs(out.get('cartesian_velocity', np.zeros((1, 6)))) > 1e-6, axis=1).sum()) if n else 0
    Path(outdir).mkdir(parents=True, exist_ok=True)
    np.save(Path(outdir) / ('%d_cartesian.npy' % trial_id), out)
    print('  %d: %d frames | cartvel topic msgs=%d (nonzero frames=%d) | tool_pose seen=%d'
          % (trial_id, n, seen[CARTVEL_TOPIC], n_cmd, seen[FEEDBACK_TOPIC]))
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
    print('extracting %d trials -> %s/<id>_cartesian.npy' % (len(trials), outdir))
    ok = 0
    for tid in trials:
        try:
            if read_bag(tid, outdir) is not None:
                ok += 1
        except Exception as e:
            print('  %d: FAILED %s' % (tid, e))
    print('done: %d/%d extracted' % (ok, len(trials)))
