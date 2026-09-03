"""Can-top anchors at (close_time - dt): the position the grasp actually happened at.
Excludes blobs near the PROJECTED wrist (current model); gates can search near the
projected placement (loose 260px). Writes anchors_close.json + top-close overlays."""
import json, os
import numpy as np, cv2
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

RAW = '/home/james/workspace/genesis_pickaplace/inthewild_trials/raw'
D = os.path.dirname(os.path.abspath(__file__))
PLC = json.load(open('/home/james/workspace/genesis_pickaplace/can_pos_recovery/trial_placements.json'))['trials']
m = np.load(f'{D}/cam4_model_final.npz')
camp, O, CONV = m['camp'], m['O'], str(m['conv'])
OK = ['242','246','250','256','260','261','274','283','286','293','295','297','311','312','320']
ZTOP = 0.113 + 0.0505 - 0.05

def proj(Xw):
    Xc = Rotation.from_rotvec(camp[5:8]).apply(np.atleast_2d(Xw)) + camp[8:11]
    x, y = Xc[:,0]/Xc[:,2], Xc[:,1]/Xc[:,2]
    r2 = x*x+y*y
    return np.stack([camp[0]*x*(1+camp[4]*r2)+camp[2], camp[1]*y*(1+camp[4]*r2)+camp[3]], 1)

def read_bag(uid):
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
                mm = ts.deserialize_ros1(raw, c.msgtype)
                b = mm.base
                tp.append((t, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                           b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z))
                try: grip.append(mm.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)
                except Exception: grip.append(np.nan)
    return np.array(tp), np.array(grip)

def blue_blobs(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mm = cv2.inRange(hsv, (95, 90, 60), (135, 255, 255))
    mm = cv2.morphologyEx(mm, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mm)
    return [(cent[i], stats[i]) for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 150]

def silver_top_raw(img, bbox):
    # raw orientation: can top is toward -x (left) of the label
    x, y, w, h = bbox[cv2.CC_STAT_LEFT], bbox[cv2.CC_STAT_TOP], bbox[cv2.CC_STAT_WIDTH], bbox[cv2.CC_STAT_HEIGHT]
    x0, x1 = max(0, x-55), min(img.shape[1], x+10)
    y0, y1 = max(0, y-10), min(img.shape[0], y+h+10)
    win = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(win, cv2.COLOR_BGR2HSV)
    mm = cv2.inRange(hsv, (0, 0, 110), (180, 70, 255))
    mm = cv2.morphologyEx(mm, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mm)
    best = None
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        if a >= 200 and (best is None or a > best[1]):
            best = (cent[i], a)
    return None if best is None else (best[0][0]+x0, best[0][1]+y0)

res = {}
for uid in OK:
    f = f'{RAW}/user_{uid}/cam_dev_video4/output.mp4'
    if not os.path.exists(f): continue
    tp, g = read_bag(uid)
    closed = g > 30
    if not closed.any(): continue
    i0 = int(np.argmax(closed))
    vts = np.loadtxt(f'{RAW}/user_{uid}/cam_dev_video4/video_frame_timestamps.txt', dtype=np.int64)
    cap = cv2.VideoCapture(f)
    e = PLC[uid]
    exp_px = proj(np.array([e['can_pos'][0], e['can_pos'][1], ZTOP]))[0]
    found = None
    for dt in [0.8, 1.5, 2.5, 4.0]:
        tq = tp[i0,0] - dt*1e9
        j = np.searchsorted(tp[:,0], tq); j = min(j, len(tp)-1)
        fi = int(np.searchsorted(vts, tq))
        if fi >= len(vts): continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok_, fr = cap.read()
        if not ok_: continue
        wr = [proj(tp[j,1:4] + Rotation.from_euler(CONV, tp[j,4:7], degrees=True).apply(o))[0] for o in O]
        blobs = blue_blobs(fr)
        blobs = [b for b in blobs if all(np.hypot(b[0][0]-w[0], b[0][1]-w[1]) > 70 for w in wr)
                 and np.hypot(b[0][0]-exp_px[0], b[0][1]-exp_px[1]) < 260]
        blobs.sort(key=lambda b: -b[1][cv2.CC_STAT_AREA])
        if not blobs: continue
        top = silver_top_raw(fr, blobs[0][1])
        if top is None: continue
        found = (top, fi, fr, dt)
        break
    if found:
        top, fi, fr, dt = found
        res[uid] = {'can_px': [top[0], top[1]], 'dt': dt}
        vis = fr.copy()
        cv2.circle(vis, (int(top[0]), int(top[1])), 10, (0,0,255), 2)
        cv2.circle(vis, (int(exp_px[0]), int(exp_px[1])), 6, (255,255,0), 2)
        cv2.imwrite(f'{D}/topc_{uid}.jpg', vis)
        d = np.hypot(top[0]-exp_px[0], top[1]-exp_px[1])
        print(f'{uid}: det ({top[0]:.0f},{top[1]:.0f}) dt-{found[3]}s vs projected-placement {d:.0f}px')
    else:
        print(f'{uid}: no detection')
json.dump(res, open(f'{D}/anchors_close.json','w'), indent=1)
