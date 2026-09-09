"""Camera-only views to identify rigid gripper-base landmarks before calibration."""
from pathlib import Path
import json,cv2,numpy as np
R=Path(__file__).resolve().parent;REPO=R.parents[2];uid=184
orig=json.loads((R/'bag_origins.json').read_text())[str(uid)]
d=REPO/f'inthewild_trials/raw/user_{uid}/cam_dev_video0';ts=(np.loadtxt(d/'video_frame_timestamps.txt',dtype=np.int64)-orig)/1e9
cap=cv2.VideoCapture(str(d/'output.mp4'));pictures=[];matches=[]
for t in [30.9,33,36,45,60,90]:
 i=int(np.argmin(abs(ts-t)));cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();assert ok
 im=cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE);cv2.imwrite(str(R/'184'/f'rigid_landmark_{t:g}s.jpg'),im)
 small=cv2.resize(im,(360,640));cv2.putText(small,f'184 cam0 {ts[i]:.3f}s',(5,20),0,.55,(255,255,255),2);pictures.append(small)
 matches.append(dict(requested_bag_time_s=t,camera_frame=i,actual_camera_time_s=float(ts[i])))
cap.release();cv2.imwrite(str(R/'184/rigid_landmark_sheet.jpg'),np.vstack([np.hstack(pictures[:3]),np.hstack(pictures[3:])]))
(R/'184/rigid_landmark_matches.json').write_text(json.dumps(matches,indent=2))
