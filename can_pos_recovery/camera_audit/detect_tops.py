"""Detect start-can + goal-can top centers in t0 frames of ok-class trials.
Rotated frame: xr=y_raw, yr=1279-x_raw. Inverse: x_raw=1279-yr, y_raw=xr.
Writes anchors.json {uid: {can_px:[x,y] raw, goal_px:[x,y] raw}} + overlay jpgs."""
import json, os
import numpy as np, cv2

RAW = '/home/james/workspace/genesis_pickaplace/inthewild_trials/raw'
OK = ['242','246','250','256','260','261','274','283','286','293','295','297','311','312','320']

def rot(fr):  # 90 deg CCW
    return cv2.rotate(fr, cv2.ROTATE_90_COUNTERCLOCKWISE)

def blue_blobs(img, mask_gate):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (95, 90, 60), (135, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    m[~mask_gate] = 0
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= 150:
            out.append((cent[i], stats[i]))
    out.sort(key=lambda b: -b[1][cv2.CC_STAT_AREA])
    return out

def silver_top(img, bbox):
    x, y, w, h = bbox[cv2.CC_STAT_LEFT], bbox[cv2.CC_STAT_TOP], bbox[cv2.CC_STAT_WIDTH], bbox[cv2.CC_STAT_HEIGHT]
    y0, y1 = max(0, y-55), min(img.shape[0], y+10)
    x0, x1 = max(0, x-10), min(img.shape[1], x+w+10)
    win = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(win, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 0, 110), (180, 70, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    best = None
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        if a >= 200 and (best is None or a > best[1]):
            best = (cent[i], a)
    if best is None: return None
    return (best[0][0] + x0, best[0][1] + y0)

res = {}
H, W = 1280, 720  # rotated dims
gate_can = np.zeros((H, W), bool); gate_can[620:1200, 40:520] = True
gate_goal = np.zeros((H, W), bool); gate_goal[380:630, 480:700] = True
for uid in OK:
    f = f'{RAW}/user_{uid}/cam_dev_video4/output.mp4'
    if not os.path.exists(f): continue
    cap = cv2.VideoCapture(f); ok_, fr = cap.read(); cap.release()
    if not ok_: continue
    img = rot(fr)
    entry, vis = {}, img.copy()
    cb = blue_blobs(img, gate_can)
    # the wrist tape sits at the same home-pose pixel in every t0 frame: exclude it
    cb = [b for b in cb if np.hypot(b[0][0]-350, b[0][1]-818) > 90
          and np.hypot(b[0][0]-200, b[0][1]-915) > 110
          and 35 <= b[1][2] <= 140]  # label width sanity (b[1]=stats: [left,top,width,height,area])
    if cb:
        top = silver_top(img, cb[0][1])
        if top:
            entry['can_px'] = [1279 - top[1], top[0]]
            cv2.circle(vis, (int(top[0]), int(top[1])), 10, (0,0,255), 2)
    gb = blue_blobs(img, gate_goal)
    if gb:
        topg = silver_top(img, gb[0][1])
        if topg:
            entry['goal_px'] = [1279 - topg[1], topg[0]]
            cv2.circle(vis, (int(topg[0]), int(topg[1])), 10, (0,255,0), 2)
    res[uid] = entry
    cv2.imwrite(f'top_{uid}.jpg', vis)
    print(uid, entry)
json.dump(res, open('anchors.json','w'), indent=1)
