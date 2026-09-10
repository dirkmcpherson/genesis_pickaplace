from pathlib import Path
import cv2,json,hashlib,numpy as np
D=Path(__file__).resolve().parent;p=D/'176_soft2_real_sim.mp4';j=p.with_suffix('.json');m=json.loads(j.read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(p)==m['video_sha256'] and sha(m['trace'])==m['trace_sha256']
cap=cv2.VideoCapture(str(p));n=0;images={};wanted=[25,27.4,35,40,60,65,74.43]
ids={min(range(len(m['matches'])),key=lambda i:abs(m['matches'][i]['time_s']-t)):t for t in wanted}
while True:
 ok,im=cap.read()
 if not ok:break
 assert im.shape==(840,1440,3)
 if n in ids:images[ids[n]]=im.copy()
 n+=1
cap.release();assert n==m['frames']==len(m['matches'])==414
for t,im in images.items():cv2.imwrite(str(D/f'review_{t:g}s.jpg'),im)
cv2.imwrite(str(D/'review_montage.jpg'),np.vstack([cv2.resize(images[t],(1008,588)) for t in [25,27.4,35,40,65,74.43]]))
m['verification']=dict(decoded_frames=n,dimensions_correct=True,trace_and_video_hashes_match=True,selected_review_times_s=list(images),visual_review='pending')
j.write_text(json.dumps(m,indent=2));print('Verified',n,'frames; selected review images saved')
