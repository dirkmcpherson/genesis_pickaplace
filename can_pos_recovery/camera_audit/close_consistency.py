import json
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
RAW='/home/james/workspace/genesis_pickaplace/inthewild_trials/raw'
PLC = json.load(open('/home/james/workspace/genesis_pickaplace/can_pos_recovery/trial_placements.json'))['trials']

def read(uid):
    ts = get_typestore(Stores.ROS1_NOETIC)
    tp, grip = [], []
    with Reader(f'{RAW}/user_{uid}/trial_data.bag') as r:
        add = {}
        for c in r.connections:
            if 'kortex' in c.msgtype:
                mdef = c.msgdef.data if hasattr(c.msgdef,'data') else c.msgdef
                add.update(get_types_from_msg(mdef, c.msgtype))
        ts.register(add)
        for c, t, raw in r.messages():
            if c.topic == '/my_gen3_lite/base_feedback':
                m = ts.deserialize_ros1(raw, c.msgtype)
                b = m.base
                tp.append((t/1e9, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z))
                try: grip.append(m.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)
                except Exception: grip.append(np.nan)
    return np.array(tp), np.array(grip)

for uid in ['242','246','250','256','260','261','274','283','286','293','295','297','311','312','320']:
    tp, g = read(uid)
    closed = g > 30
    i0 = int(np.argmax(closed))       # first close
    if not closed[i0]: print(uid, 'no closure'); continue
    z0 = tp[i0,3]
    lift = np.where((tp[:,3] > z0 + 0.03) & (np.arange(len(tp)) > i0))[0]
    il = int(lift[0]) if len(lift) else len(tp)-1
    dxy = np.hypot(tp[il,1]-tp[i0,1], tp[il,2]-tp[i0,2])
    e = PLC[uid]
    print(f"{uid}: pos-class {e['pos']} placement ({e['can_pos'][0]:.3f},{e['can_pos'][1]:.3f}) "
          f"| close@t={tp[i0,0]-tp[0,0]:.1f}s xy=({tp[i0,1]:.3f},{tp[i0,2]:.3f}) z={z0:.3f} "
          f"-> lift drag {dxy*100:.1f}cm | close-xy vs placement "
          f"{np.hypot(tp[i0,1]-e['can_pos'][0], tp[i0,2]-e['can_pos'][1])*100:.1f}cm")
