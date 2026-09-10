"""Inventory all selected real cameras and ROS clock origins without selecting by outcome."""
from pathlib import Path
import json,hashlib,cv2,numpy as np
from rosbags.highlevel import AnyReader
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[4]
plan=json.loads((P/'plan.json').read_text());records=[];origins={};errors=[]
for uid in plan['uids']:
 j=next(j for j in plan['jobs'] if j['uid']==uid);cams=[]
 try:
  bag=R/f'inthewild_trials/raw/user_{uid}/trial_data.bag'
  with AnyReader([bag]) as reader:_,t,_=next(reader.messages());origins[str(uid)]=int(t)
  for cam in [4,0]:
   p=bag.parent/f'cam_dev_video{cam}';v=p/'output.mp4';tsfile=p/'video_frame_timestamps.txt';ts=np.loadtxt(tsfile,dtype=np.int64)
   cap=cv2.VideoCapture(str(v));assert cap.isOpened();n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));assert n==len(ts) and np.all(np.diff(ts)>0)
   for i in [0,n//2,n-1]:cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();assert ok
   cams.append(dict(camera=cam,video=str(v),timestamps=str(tsfile),timestamp_sha256=hashlib.sha256(tsfile.read_bytes()).hexdigest(),frames=n,shape=list(im.shape),fps=cap.get(cv2.CAP_PROP_FPS)));cap.release()
  records.append(dict(uid=uid,role='expanded_verification',day=j['day'],sim_trace=j['baseline'],sim_trace_sha256=j['baseline_sha256'],cameras=cams))
 except Exception as e:errors.append(dict(uid=uid,error=repr(e)))
print('REFERENCE_CAMERAS',len(records),'/',len(plan['uids']),'errors',errors,flush=True)
(D/'manifest.json').write_text(json.dumps(dict(records=records,errors=errors,selection='All36 previously frozen expanded-sample UIDs. Missing/corrupt video retained as an inventory error, never a physics-outcome exclusion.'),indent=2))
(D/'bag_origins.json').write_text(json.dumps(origins,indent=2))
