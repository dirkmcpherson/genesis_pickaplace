from pathlib import Path
import json
import cv2,numpy as np
R=Path(__file__).resolve().parent
manifest=json.loads((R/'manifest.json').read_text());origins=json.loads((R/'bag_origins.json').read_text())
for record in manifest['records']:
 if record['role']!='calibration':continue
 uid=record['uid'];path=Path(record['sim_trace']);meta=json.loads(path.with_suffix('.json').read_text())
 with np.load(path) as z:
  g=z['source_grip'];n=len(g);tr=z['trajectory'];closing=np.flatnonzero(g>30);close=int(closing[0]);picked=np.flatnonzero(z['stages'][:,0]);pick=int(picked[0]) if len(picked) else close
  running_max=np.maximum.accumulate(g[pick:]);lower=np.flatnonzero(running_max-g[pick:]>=10);opening=int(lower[0]+pick) if len(lower) else n-1
  high=int(np.argmax(tr[:opening+1,15]));events=[('pre_closure',max(0,close-10)),('loaded_early',min(close+33,n-1)),('maximum_sim_can_height',high),('before_motor_opening',max(0,opening-10)),('after_motor_opening',min(opening+10,n-1)),('final',n-1)]
 timed=Path(meta['timing_reconstruction']['source'])
 with np.load(timed) as z:offset=float(z['t_frame'][0])
 pictures=[];matches=[];d=R/str(uid);d.mkdir(exist_ok=True)
 for label,i in events:
  pair=[];desired=offset+(i+1)*.03
  for camera in record['cameras']:
   ts=(np.loadtxt(camera['timestamps'],dtype=np.int64)-origins[str(uid)])/1e9;j=int(np.argmin(abs(ts-desired)))
   cap=cv2.VideoCapture(camera['video']);cap.set(cv2.CAP_PROP_POS_FRAMES,j);ok,im=cap.read();assert ok;cap.release()
   im=cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE)
   cv2.imwrite(str(d/f"{label}_cam{camera['camera']}.jpg"),im)
   h,w=im.shape[:2];s=min(480/w,640/h);small=cv2.resize(im,(round(w*s),round(h*s)));canvas=np.zeros((690,480,3),np.uint8);y=50+(640-small.shape[0])//2;x=(480-small.shape[1])//2;canvas[y:y+small.shape[0],x:x+small.shape[1]]=small
   cv2.putText(canvas,f"{uid} camera {camera['camera']} | {label}",(7,20),0,.45,(255,255,255),1);cv2.putText(canvas,f"source post-step clock {desired:.2f}s; match error {(ts[j]-desired)*1000:+.1f}ms",(7,42),0,.4,(255,255,255),1)
   pair.append(canvas);matches.append(dict(event=label,trace_frame=i,camera=camera['camera'],camera_frame=j,desired_bag_time_s=desired,actual_camera_time_s=float(ts[j]),error_s=float(ts[j]-desired)))
  pictures.append(np.hstack(pair))
 cv2.imwrite(str(d/'camera_visibility_sheet.jpg'),np.vstack(pictures))
 (d/'view_matches.json').write_text(json.dumps(dict(events=events,matches=matches,clock='Source action-start offset + post-step time; hardware acquisition/follower delays uncalibrated',event_basis='First >30 motor closure, then first 10-unit decrease from running post-pick maximum, and simulation height select review times only; not observed real release/height labels'),indent=2))
 print('EXTRACTED',uid)
