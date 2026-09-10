"""Clock-matched images of the retained196 regression; no pose fitting."""
from pathlib import Path
import json,hashlib
import cv2,numpy as np
D=Path(__file__).resolve().parent;E=D.parent;P=E.parent
ref=E/'physical_reference';record=next(r for r in json.loads((ref/'manifest.json').read_text())['records'] if r['uid']==196)
origin=json.loads((ref/'bag_origins.json').read_text())['196']
paths=[P.parent/'early_yaw_pool/196/collection/196_eef_delta.npz',E/'196_soft_g2/196_eef_delta.npz']
traces=[np.load(p) for p in paths];meta=json.loads(paths[1].with_suffix('.json').read_text());offset=float(np.load(meta['timing_reconstruction']['source'])['t_frame'][0])
times=[40.,46.32,53.58,59.1,60.63,68.,73.47];matches=[];images=[]
for t in times:
 panels=[]
 for camera in record['cameras']:
  ts=(np.loadtxt(camera['timestamps'],dtype=np.int64)-origin)/1e9;i=int(np.argmin(abs(ts-(offset+t))))
  cap=cv2.VideoCapture(camera['video']);cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();cap.release();assert ok
  im=cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE);im=cv2.resize(im,(360,640))
  cv2.putText(im,f'Cam {camera["camera"]} | {t:.2f}s',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
  panels.append(im);matches.append(dict(time_s=t,camera=camera['camera'],frame=i,error_s=float(ts[i]-(offset+t))))
 canvas=np.zeros((725,720,3),np.uint8);canvas[:640]=np.hstack(panels)
 i=min(len(traces[0]['trajectory'])-1,round(t/.03)-1)
 for j,(label,z) in enumerate(zip(['Original','Soft + damping2'],traces)):
  c=z['trajectory'][i,13:16];g=z['goal_pose'][i,:3];gap=1000*(np.linalg.norm(c[:2]-g[:2])-.066)
  text=f'{label}: sim can z {c[2]:.3f}m, gap {gap:.1f}mm, contacts {z["contact_counts"][i].tolist()}'
  cv2.putText(canvas,text,(8,667+28*j),cv2.FONT_HERSHEY_SIMPLEX,.47,(255,255,255),1)
 cv2.imwrite(str(D/f'real_{t:g}s.jpg'),canvas);images.append(cv2.resize(canvas,(360,362)))
cv2.imwrite(str(D/'real_release_slide_montage.jpg'),np.hstack(images))
(D/'real_frame_matches.json').write_text(json.dumps(dict(matches=matches,source_trace_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},scope='Selected timestamp-matched real images and simulated numeric diagnostics; no registered image comparison or calibrated pose claim.'),indent=2))
print('max timestamp error',max(abs(r['error_s']) for r in matches))
