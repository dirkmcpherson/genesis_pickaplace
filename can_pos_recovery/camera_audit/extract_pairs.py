"""Extract (tool_pose 3D, blue-tape pixel) calibration pairs for one trial.

Usage: python extract_pairs.py <uid> [debug_n]
Writes pairs_<uid>.npz. Strategy: sample frames ~0.4s apart while the gripper is OPEN
(outside closure windows, so no held blue can rides with the wrist); veto static blue
regions (can labels, shelf cans) via a median-frame mask; take the largest surviving
blue blob as the wrist tape.
"""
import sys, os
import numpy as np
import cv2
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

RAW = '/home/james/workspace/genesis_pickaplace/inthewild_trials/raw'
OUT = os.path.dirname(os.path.abspath(__file__))

def read_bag(uid):
    ts = get_typestore(Stores.ROS1_NOETIC)
    rows, grip = [], []
    with Reader(f'{RAW}/user_{uid}/trial_data.bag') as r:
        add = {}
        for c in r.connections:
            if 'kortex' in c.msgtype:
                mdef = c.msgdef.data if hasattr(c.msgdef, 'data') else c.msgdef
                add.update(get_types_from_msg(mdef, c.msgtype))
        ts.register(add)
        for c, t, raw in r.messages():
            if c.topic == '/my_gen3_lite/base_feedback':
                m = ts.deserialize_ros1(raw, c.msgtype)
                b = m.base
                rows.append((t, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                             b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z))
                try:
                    g = (m.interconnect.oneof_tool_feedback
                         .gripper_feedback[0].motor[0].position)
                except Exception:
                    g = np.nan
                grip.append(g)
    return np.array(rows, dtype=np.float64), np.array(grip, dtype=np.float64)

def blue_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (95, 90, 60), (135, 255, 255))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

def main(uid, debug_n=0):
    bag, grip = read_bag(uid)
    vts = np.loadtxt(f'{RAW}/user_{uid}/cam_dev_video4/video_frame_timestamps.txt',
                     dtype=np.int64)
    cap = cv2.VideoCapture(f'{RAW}/user_{uid}/cam_dev_video4/output.mp4')
    t0, t1 = bag[0, 0], bag[-1, 0]
    print(f'uid {uid}: bag {(t1-t0)/1e9:.1f}s; video {len(vts)} frames; '
          f'vstart-bstart {(vts[0]-t0)/1e9:+.2f}s')

    # candidate samples: every 0.4s, gripper OPEN (pos<20), pose moved >=1cm since last
    step_ns = int(0.25e9)
    cands, last_xyz = [], None
    for tq in range(int(t0), int(t1), step_ns):
        i = min(np.searchsorted(bag[:, 0], tq), len(bag) - 1)
        row, g = bag[i], grip[i]
        if not np.isnan(g) and g > 20:      # gripper not open -> may hold the blue can
            continue
        fi = int(np.searchsorted(vts, row[0]))
        if fi >= len(vts) or abs(int(vts[fi]) - int(row[0])) > 40e6:
            continue
        xyz = row[1:4]
        if last_xyz is not None and np.linalg.norm(xyz - last_xyz) < 0.005:
            continue
        last_xyz = xyz.copy()
        cands.append((fi, row))
    # read frames once, build static-blue veto from the median frame
    frames = {}
    for fi, _ in cands:
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, fr = cap.read()
        if ok:
            frames[fi] = fr
    if len(frames) < 8:
        print('  too few frames'); return
    med = np.median(np.stack(list(frames.values())[:40]), axis=0).astype(np.uint8)
    static = cv2.dilate(blue_mask(med), np.ones((15, 15), np.uint8))
    pairs, dbg = [], 0
    for fi, row in cands:
        if fi not in frames: continue
        m = blue_mask(frames[fi])
        m[static > 0] = 0
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
        blobs = [(cent[i][0], cent[i][1], float(stats[i, cv2.CC_STAT_AREA]))
                 for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 80]
        blobs.sort(key=lambda b: -b[2])
        if not blobs: continue
        row3 = []
        for j in range(3):
            if j < len(blobs): row3 += [blobs[j][0], blobs[j][1], blobs[j][2]]
            else: row3 += [np.nan, np.nan, 0.0]
        pairs.append((row[0], *row[1:7], *row3, len(blobs)))
        if dbg < debug_n:
            vis = frames[fi].copy()
            vis[static > 0] //= 2
            for j, (bx, by, ba) in enumerate(blobs[:3]):
                cv2.circle(vis, (int(bx), int(by)), 12,
                           (0, 0, 255) if j == 0 else (0, 255, 255), 2)
            cv2.imwrite(f'{OUT}/dbg_{uid}_{fi}.jpg', vis)
            dbg += 1
    cap.release()
    arr = np.array(pairs, dtype=np.float64)
    np.savez(f'{OUT}/pairs3_{uid}.npz', pairs=arr)
    if len(arr):
        print(f'  kept {len(arr)} pairs; tool z [{arr[:, 3].min():.3f}, '
              f'{arr[:, 3].max():.3f}]; multi-blob frames '
              f'{int((arr[:, 16] > 1).sum())}/{len(arr)}')
    else:
        print('  EMPTY')

if __name__ == '__main__':
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 0)
