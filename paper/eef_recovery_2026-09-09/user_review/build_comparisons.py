"""Review videos only: combine existing real and simulated imagery, no physics edits."""
from pathlib import Path
import json, hashlib
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
POOL=ROOT.parent/'early_yaw_pool'
REPO=ROOT.parents[2]
def fit(im):
    h,w=im.shape[:2]; ratio=min(640/w,640/h)
    out=np.zeros((640,640,3),np.uint8); small=cv2.resize(im,(round(w*ratio),round(h*ratio)))
    y=(640-small.shape[0])//2;x=(640-small.shape[1])//2
    out[y:y+small.shape[0],x:x+small.shape[1]]=small
    return out
for uid in [176,202]:
    d=POOL/str(uid);trace=d/'collection'/f'{uid}_eef_delta.npz'
    meta=json.loads(trace.with_suffix('.json').read_text()); seq=meta['sequence']
    sim_path=d/f'{uid}_timestamp.mp4';sim=cv2.VideoCapture(str(sim_path))
    sfps=sim.get(cv2.CAP_PROP_FPS);sn=int(sim.get(cv2.CAP_PROP_FRAME_COUNT));assert sn>0
    if uid==176:
        real_path=REPO/'inthewild_trials/raw/user_176/cam_dev_video4/output.mp4'
        real=cv2.VideoCapture(str(real_path));rfps=real.get(cv2.CAP_PROP_FPS);rn=int(real.get(cv2.CAP_PROP_FRAME_COUNT));assert rn>0
        timing='Elapsed-time comparison; camera-to-bag offset UNKNOWN (no time stretching)'
    else:
        real_path=d/'202_real_cam4_embedded.npz';camera=np.load(real_path);rt=camera['bag_time_s'];rgb=camera['rgb']
        offset=json.loads((d/'embedded_camera_alignment.json').read_text())['matches'][0]['desired_bag_time_s']
        timing='Bag receipt-time match; acquisition delay UNKNOWN; real camera only 96 x 96'
    output=ROOT/f'{uid}_real_sim_annotated.mp4';assert not output.exists()
    writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),sfps,(1280,780));assert writer.isOpened()
    pics=[];matches=[]
    with np.load(trace) as z:
        for j in range(sn):
            ok,sframe=sim.read();assert ok
            i=min(3*j,len(z['trajectory'])-1);t=(i+1)*.03
            if uid==176:
                ri=min(round(j/sfps*rfps),rn-1);real.set(cv2.CAP_PROP_POS_FRAMES,ri);ok,rframe=real.read();assert ok
                real_time=ri/rfps;error=None
            else:
                desired=offset+i*.03;ri=int(np.argmin(abs(rt-desired)));rframe=rgb[ri][:,:,::-1];real_time=float(rt[ri]);error=real_time-desired
            canvas=np.zeros((780,1280,3),np.uint8);canvas[55:695,:640]=fit(cv2.rotate(rframe,cv2.ROTATE_90_COUNTERCLOCKWISE));canvas[55:695,640:]=fit(sframe)
            row=z['trajectory'][i];contacts=z['contact_counts'][i];dist=np.linalg.norm(row[13:15]-z['goal_pose'][i,:2])*1000
            stage='Before supported release'
            for key,label in [('release_start','Supported release seen'),('push_start','Supported push seen'),('contact_start','Supported goal contact seen')]:
                if seq[key] is not None and i>=seq[key]: stage=label
            labels=[(12,25,f'Trial {uid} | REAL camera 4'),(652,25,'SIM | saved measured poses | exact EEF replay verified'),(12,49,timing),
                (12,718,f'Sim {t:.2f}s | center distance {dist:.1f} mm | tilt {row[20]:.1f} deg | contacts shelf/hand/goal {contacts.tolist()}'),
                (12,742,f'Physical diagnostic: {stage} | full sequence: {seq["reason"]}'),
                (12,768,'Metric passes; release can precede pickup. Distance/contact annotations are SIM measurements only.')]
            for x,y,label in labels:cv2.putText(canvas,label,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.52,(245,245,245),1,cv2.LINE_AA)
            writer.write(canvas);matches.append(dict(sim_trace_frame=i,real_frame=ri,real_time_s=real_time,receipt_error_s=error))
            if j in {round(f*(sn-1)) for f in [0,.3,.55,.72,.88,1]}:pics.append(cv2.resize(canvas,(768,468)))
    writer.release();sim.release()
    if uid==176:real.release()
    cv2.imwrite(str(ROOT/f'{uid}_comparison_phases.jpg'),np.vstack(pics))
    (ROOT/f'{uid}_comparison.json').write_text(json.dumps(dict(uid=uid,trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),real_source=str(real_path),sim_source=str(sim_path),timing=timing,real_display_rotation='90 degrees counterclockwise, no crop',frames=sn,sim_video_fps=sfps,sequence=seq,matches=matches,scope='Full videos assembled from retained imagery; neither new physics nor calibrated real contact measurement.'),indent=2))
    print('BUILT',uid,flush=True)
