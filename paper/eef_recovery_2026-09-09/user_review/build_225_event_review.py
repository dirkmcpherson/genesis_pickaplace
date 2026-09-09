"""Annotate the verified Day 2 supported slide from its saved-pose render."""
from pathlib import Path
import json,hashlib
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
D=ROOT.parent/'early_yaw_pool/225'
trace=D/'collection/225_eef_delta.npz'
meta=json.loads(trace.with_suffix('.json').read_text());seq=meta['sequence']
cap=cv2.VideoCapture(str(D/'metric_visuals/225_timestamp.mp4'));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
out=ROOT/'225_slide_event_annotated.mp4';assert not out.exists()
writer=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'mp4v'),fps,(640,760));assert writer.isOpened()
selected=[seq['release_start']-9,seq['release_start'],seq['push_start'],seq['contact_start'],seq['contact_start']+9,990]
selected_video={min(round(i/3),n-1) for i in selected};pictures=[]
with np.load(trace) as z:
 for j in range(n):
  ok,im=cap.read();assert ok
  i=min(j*3,len(z['trajectory'])-1);row=z['trajectory'][i];c=z['contact_counts'][i]
  canvas=np.zeros((760,640,3),np.uint8);canvas[:640]=im
  status='Before supported release'
  for k,label in [('release_start','Supported release observed'),('push_start','Supported push observed'),('contact_start','Supported goal contact observed')]:
   if i>=seq[k]:status=label
  labels=[f'225 | Day 2 yaw -19.2 deg | exact EEF replay verified',status,
   'Full event: 19.23 mm goalward; 94.1% shelf support',
   'Real camera unavailable in local recording',
   'Saved poses; contact counts are simulation measurements']
  for k,label in enumerate(labels):cv2.putText(canvas,label,(8,661+21*k),0,.46,(255,255,255),1,cv2.LINE_AA)
  writer.write(canvas)
  if j in selected_video:pictures.append(canvas.copy())
cap.release();writer.release();assert len(pictures)==6
cv2.imwrite(str(ROOT/'225_slide_event_phases.jpg'),np.vstack([np.hstack(pictures[:3]),np.hstack(pictures[3:])]))
(ROOT/'225_slide_event_review.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),sequence=seq,selected_trace_frames=[3*i for i in sorted(selected_video)],real_camera='No separate camera files or image/camera topics in local trial bag',scope='Saved-pose video and event-focused six-frame sheet; independent physical replay checked separately'),indent=2))
print('BUILT 225')
