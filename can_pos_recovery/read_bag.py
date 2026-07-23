"""Extract tool_pose / gripper closures / joint timing from a kortex trial bag.

Needs the pure-python `rosbags` package (not ROS):
  pip install --target <dir> rosbags ; PYTHONPATH=<dir> python read_bag.py <uid> [...]

Prints, per gripper closure >=0.5s: tool_pose at close, min-z during closure (the grasp),
and at release. tool_pose is in the ROBOT BASE frame; sim world z = this + 0.05.
"""
import os
import sys
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def read(uid):
    path = (f'/home/j/workspace/genesis_pickaplace/inthewild_trials/raw/'
            f'user_{uid}/trial_data.bag')
    ts = get_typestore(Stores.ROS1_NOETIC)
    with Reader(path) as r:
        add = {}
        for c in r.connections:
            if 'kortex' in c.msgtype:
                mdef = c.msgdef.data if hasattr(c.msgdef, 'data') else c.msgdef
                add.update(get_types_from_msg(mdef, c.msgtype))
        ts.register(add)
        tp, grip = [], []
        for c, t, raw in r.messages():
            if c.topic == '/my_gen3_lite/base_feedback':
                m = ts.deserialize_ros1(raw, c.msgtype)
                b = m.base
                tp.append((t / 1e9, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z))
                try:
                    grip.append((t / 1e9, m.interconnect.oneof_tool_feedback
                                 .gripper_feedback[0].motor[0].position))
                except Exception:
                    pass
    tp = np.array(tp); grip = np.array(grip)
    t0 = tp[0, 0]; tp[:, 0] -= t0; grip[:, 0] -= t0
    return tp, grip


def closures(tp, grip, thresh=30.0, min_s=0.5):
    closed = grip[:, 1] > thresh
    out = []
    i = 0
    while i < len(closed):
        if closed[i]:
            j = i
            while j < len(closed) and closed[j]:
                j += 1
            if grip[j - 1, 0] - grip[i, 0] >= min_s:
                out.append((i, j - 1))
            i = j
        else:
            i += 1
    return out


if __name__ == '__main__':
    for uid in sys.argv[1:]:
        tp, grip = read(uid)
        print(f"--- {uid}: {tp[-1,0]:.1f}s, gripper {grip[:,1].min():.0f}..{grip[:,1].max():.0f}")
        for a, b in closures(tp, grip):
            ta, tb = grip[a, 0], grip[b, 0]
            ia = np.searchsorted(tp[:, 0], ta); ib = min(np.searchsorted(tp[:, 0], tb), len(tp) - 1)
            seg = tp[ia:ib + 1]
            k = np.argmin(seg[:, 3])
            print(f"  closure {ta:5.1f}-{tb:5.1f}s  close@({tp[ia,1]:+.3f},{tp[ia,2]:+.3f},{tp[ia,3]:+.3f})"
                  f"  minz@({seg[k,1]:+.3f},{seg[k,2]:+.3f},{seg[k,3]:+.3f})"
                  f"  release@({tp[ib,1]:+.3f},{tp[ib,2]:+.3f},{tp[ib,3]:+.3f})")
        if not closures(tp, grip):
            print("  NO closure >=0.5s (gripper never held anything)")
