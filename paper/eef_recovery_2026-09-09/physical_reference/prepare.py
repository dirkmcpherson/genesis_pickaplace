"""Declare reference roles and inventory real camera data without fitting physics."""
from pathlib import Path
import hashlib,json
import cv2,numpy as np
R=Path(__file__).resolve().parent;REPO=R.parents[2]
roles={113:'calibration',184:'calibration',233:'calibration',176:'validation',185:'validation',237:'validation'}
records=[]
for uid,role in roles.items():
 pool='early_yaw_pool' if uid<232 else 'timestamp_full_pool'
 src=R.parent/pool/str(uid)/'collection'/f'{uid}_eef_delta.npz'
 assert src.exists();meta=json.loads(src.with_suffix('.json').read_text());cameras=[]
 for cam in [4,0]:
  d=REPO/f'inthewild_trials/raw/user_{uid}/cam_dev_video{cam}';v=d/'output.mp4';t=d/'video_frame_timestamps.txt'
  ts=np.loadtxt(t,dtype=np.int64);cap=cv2.VideoCapture(str(v));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));assert n==len(ts) and np.all(np.diff(ts)>0)
  for i in [0,n//2,n-1]:cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,frame=cap.read();assert ok
  shape=frame.shape;fps=cap.get(cv2.CAP_PROP_FPS);cap.release()
  cameras.append(dict(camera=cam,video=str(v),timestamps=str(t),timestamp_sha256=hashlib.sha256(t.read_bytes()).hexdigest(),frames=n,shape=list(shape),fps=fps))
 records.append(dict(uid=uid,role=role,day={113:'12-16',176:'12-16',184:'12-17',185:'12-17',233:'12-18',237:'12-18'}[uid],sim_trace=str(src),sim_trace_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),variant=meta['variant'],cameras=cameras))
report=dict(records=records,selection='One calibration and one validation reference per recording day; calibration spans known carry loss, placement loss and a complete sequence. All six have both high-resolution cameras and matching timestamp files.',calibration_uids=[113,184,233],validation_uids=[176,185,237],validation_scope='Excluded from subsequent parameter fitting; previously inspected records, not a statistically pristine unseen holdout.',fit_policy='No per-trial mechanical parameters; no fitting to pass/fail labels. Direct visual measurements and uncertainty first.',state='Camera counts, monotonic timestamps and first/middle/last decode checked; visibility and quantitative observability remain to be reviewed.')
(R/'manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
