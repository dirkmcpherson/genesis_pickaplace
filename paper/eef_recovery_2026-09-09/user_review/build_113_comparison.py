from pathlib import Path
import json
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
uid=113
raw=REPO/f'inthewild_trials/raw/user_{uid}/cam_dev_video4'
origin=json.loads((ROOT/'camera_bag_origins.json').read_text())[str(uid)]['bag_start_ns']
rt=(np.loadtxt(raw/'video_frame_timestamps.txt',dtype=np.int64)-origin)/1e9
with np.load(ROOT.parent/f'early_yaw_pool/timed/{uid}_timed.npz') as z:offset=float(z['t_frame'][0])
real=cv2.VideoCapture(str(raw/'output.mp4'));sim=cv2.VideoCapture(str(ROOT/'113_grasp_failure_sim.mp4'))
n=int(sim.get(cv2.CAP_PROP_FRAME_COUNT));fps=sim.get(cv2.CAP_PROP_FPS);assert n>0
out=ROOT/'113_real_sim_grasp_failure.mp4';assert not out.exists()
w=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,760));assert w.isOpened()
def fit(im):
 h,ww=im.shape[:2];s=min(640/ww,640/h);small=cv2.resize(im,(round(ww*s),round(h*s)));o=np.zeros((640,640,3),np.uint8);y=(640-small.shape[0])//2;x=(640-small.shape[1])//2;o[y:y+small.shape[0],x:x+small.shape[1]]=small;return o
pics=[];matches=[]
with np.load(ROOT.parent/'early_yaw_pool/113/collection/113_eef_delta.npz') as z:
 for j in range(n):
  i=3*j;ok,s=sim.read();assert ok
  desired=offset+i*.03;ri=int(np.argmin(abs(rt-desired)));real.set(cv2.CAP_PROP_POS_FRAMES,ri);ok,r=real.read();assert ok
  canvas=np.zeros((760,1280,3),np.uint8);canvas[55:695,:640]=fit(cv2.rotate(r,cv2.ROTATE_90_COUNTERCLOCKWISE));canvas[55:695,640:]=fit(s)
  labels=[(12,24,'113 | REAL camera 4'),(652,24,'SIM | full world, Day 1 yaw -9.7 degrees'),(12,47,'Per-frame timestamps matched to bag receipt clock; acquisition delay uncalibrated'),
    (12,718,f'Sim {(i+1)*.03:.2f}s | tilt {z["trajectory"][i,20]:.1f} deg | recorded grip {z["source_grip"][i]:.1f}/100 | sim shelf/hand/goal {z["contact_counts"][i].tolist()}'),
    (12,743,'Failure diagnosis: first simulated tilt >60 degrees at 13.38s; grasp fidelity is not established.')]
  for x,y,label in labels:cv2.putText(canvas,label,(x,y),0,.51,(255,255,255),1,cv2.LINE_AA)
  w.write(canvas);matches.append(dict(trace_frame=i,camera_frame=ri,time_error_s=float(rt[ri]-desired)))
  if j in [0,70,110,140,150,n-1]:pics.append(cv2.resize(canvas,(768,456)))
w.release();real.release();sim.release();assert len(pics)==6
cv2.imwrite(str(ROOT/'113_failure_comparison_phases.jpg'),np.vstack(pics))
(ROOT/'113_failure_comparison.json').write_text(json.dumps(dict(matches=matches,max_abs_time_error_s=max(abs(r['time_error_s']) for r in matches),scope='First 18 seconds; saved measured-pose simulation, no independent successful replay claim; real timestamp matching is not acquisition calibration'),indent=2))
print('BUILT 113')
