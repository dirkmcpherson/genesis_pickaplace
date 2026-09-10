from pathlib import Path
import json,hashlib
import cv2,numpy as np
R=Path(__file__).resolve().parent;REF=R.parent.parent/'physical_reference';TRACE=R.parent/'g14_feedback_full/113_elliptic10_soft/113_eef_delta.npz'
m=json.loads(TRACE.with_suffix('.json').read_text());timed=np.load(m['timing_reconstruction']['source']);offset=float(timed['t_frame'][0]);origin=json.loads((REF/'bag_origins.json').read_text())['113'];record=next(r for r in json.loads((REF/'manifest.json').read_text())['records'] if r['uid']==113)
requests=[19.8,20.04,20.16,20.34,20.52,20.7,21.,21.5,22.,22.5,23.,24.]
rows=[];frames=[]
for t in requests:
 panels=[];matches=[]
 for camera in [4,0]:
  c=next(c for c in record['cameras'] if c['camera']==camera);times=(np.loadtxt(c['timestamps'],dtype=np.int64)-origin)/1e9;i=int(np.argmin(abs(times-(offset+t))));cap=cv2.VideoCapture(c['video']);cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();assert ok;cap.release();im=cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE);cv2.imwrite(str(R/f'113_{t:g}_cam{camera}.jpg'),im)
  # Overview retained: support cannot be inferred from a floating hand crop.
  panel=cv2.resize(im,(360,640));cv2.putText(panel,f'{t:.2f}s camera{camera}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2);panels.append(panel);matches.append(dict(camera=camera,frame=i,time_error_s=float(times[i]-(offset+t)),video=c['video']))
 frames.append(np.hstack(panels));rows.append(dict(requested_replay_time_s=t,matches=matches))
for j in range(3):cv2.imwrite(str(R/f'113_real_release_sheet{j+1}.jpg'),np.vstack(frames[j*4:j*4+4]))
(R/'real_frames.json').write_text(json.dumps(dict(uid=113,source_trace=str(TRACE),trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),records=rows,qualification='Clock-matched raw cameras, no simulation fitting or inferred real contact labels.'),indent=2))
