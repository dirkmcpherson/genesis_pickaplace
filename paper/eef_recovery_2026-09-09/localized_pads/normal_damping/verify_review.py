from pathlib import Path
import subprocess,json,hashlib,cv2,numpy as np
D=Path('/home/james/workspace/genesis_pickaplace/paper/eef_recovery_2026-09-09/localized_pads/normal_damping')
p=D/'233_normal_damping_real_sim.mp4';m=json.loads(p.with_suffix('.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(p)==m['video_sha256'] and sha(m['trace'])==m['trace_sha256']
label='SOFT PADS | normal damping 2x | proximity pass; strict contact test fails'
textfile=D/'233_review_header.txt';textfile.write_text('Trial 233 | '+label+' | recorded motion')
font=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf');assert font.exists()
tmp=D/'233_review_corrected.mp4'
header_raw=D/'233_review_header_raw.mp4'
reader=cv2.VideoCapture(str(p.with_name(p.stem+'_raw.mp4')))
writer=cv2.VideoWriter(str(header_raw),cv2.VideoWriter_fourcc(*'mp4v'),m['fps'],(1440,840));assert writer.isOpened()
header=textfile.read_text();assert cv2.getTextSize(header,cv2.FONT_HERSHEY_SIMPLEX,.57,1)[0][0]<1416
count=0
while True:
 ok,frame=reader.read()
 if not ok:break
 frame[:36,:,:]=22
 cv2.putText(frame,header,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.57,(245,245,245),1,cv2.LINE_AA)
 writer.write(frame);count+=1
reader.release();writer.release();assert count==m['frames']
subprocess.run(['ffmpeg','-v','error','-i',str(header_raw),'-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp)],check=True)
p.rename(D/'233_review_initial_header_unreviewed.mp4');tmp.rename(p)
cap=cv2.VideoCapture(str(p));n=0;stills=[]
indices=[int(np.argmin([abs(x['time_s']-t) for x in m['matches']])) for t in [14,22,25,m['clip_end_s']]]
while True:
 ok,frame=cap.read()
 if not ok:break
 assert frame.shape==(840,1440,3)
 if n in indices:stills.append(cv2.resize(frame,(1008,588)))
 if n==m['frames']-1:cv2.imwrite(str(D/'233_normal_damping_final.jpg'),frame)
 n+=1
cap.release();assert n==m['frames']==161
cv2.imwrite(str(p.with_suffix('.jpg')),np.vstack(stills))
m['initial_header_video_sha256']=m['video_sha256'];m['video_sha256']=sha(p);m['label']=label
m['header_correction']='Use strict contact test fails: saved decision contact states do not prove absence of contact at every physics substep. Re-encoded from original raw frames with only top title replaced.'
m['verification']=dict(all_frames_decoded=n,shape=[840,1440,3],trace_sha256_verified=True,video_sha256_verified=True,script_sha256=sha(__file__))
p.with_suffix('.json').write_text(json.dumps(m,indent=2))
(D/'verify_review.py').write_text(Path(__file__).read_text())
print('Verified161 frames; complete28.86s trial; corrected contact-test wording.')
