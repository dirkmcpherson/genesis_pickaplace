"""Clock-matched real video, real hand crop, and saved simulation views (233)."""
from pathlib import Path
import argparse,os,sys,json,hashlib,subprocess
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2];REF=ROOT.parent/'physical_reference'
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trace',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
a=p.parse_args();assert not a.out.exists()
os.environ.setdefault('TI_CPU_MAX_NUM_THREADS','1');os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('MPLCONFIGDIR','/tmp/eef-recovery-mpl');os.environ.setdefault('NUMBA_CACHE_DIR','/tmp/eef-recovery-numba')
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from eef_delta_control import ArmKinematics,transform
import genesis_can_env
from sim_variant_hook import apply_pre,apply_post
data=np.load(a.trace);meta=json.loads(a.trace.with_suffix('.json').read_text());assert int(meta['uid'])==233
readout=json.loads((a.trace.parent/'readout.json').read_text())
urdf=Path(meta['physics_treatment']['urdf']['candidate'])
assert hashlib.sha256(urdf.read_bytes()).hexdigest()==meta['physics_treatment']['urdf']['candidate_sha256']
manifest=json.loads((REF/'manifest.json').read_text());record=next(r for r in manifest['records'] if r['uid']==233)
camera=next(c for c in record['cameras'] if c['camera']==4)
origin=json.loads((REF/'bag_origins.json').read_text())['233']
times=(np.loadtxt(camera['timestamps'],dtype=np.int64)-origin)/1e9
timed=np.load(meta['timing_reconstruction']['source']);offset=float(timed['t_frame'][0])
pilot=json.loads((REF/'233/rigid_cap_camera_pilot.json').read_text())
fit=next(f for f in pilot['fits'] if f['surface_x_m']==.0152 and f['image_left_local_y_sign']==-1)
fx,fy,cx,cy,k=pilot['intrinsics_prior'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
rv=np.array(fit['rvec']);tv=np.array(fit['tvec']);dist=np.array([k,0,0,0,0.])
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
gripper_tool=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix())
def crop_center(t):
    q=np.array([np.interp(t,timed['t_frame'],timed['q_frame'][:,j]) for j in range(6)])
    pose=fk.tool(q,np.eye(4))@np.linalg.inv(gripper_tool)
    point=pose[:3,:3]@np.array([.0152,0,.070003])+pose[:3,3]
    uv=cv2.projectPoints(point[None],rv,tv,K,dist)[0].reshape(2)
    return np.array([uv[1],1279-uv[0]])
def crop(image,center):
    x,y=np.rint(center-np.array([120,210])).astype(int)
    out=np.zeros((360,240,3),dtype=np.uint8)
    x0,x1=max(0,x),min(image.shape[1],x+240);y0,y1=max(0,y),min(image.shape[0],y+360)
    if x1>x0 and y1>y0:out[y0-y:y1-y,x0-x:x1-x]=image[y0:y1,x0:x1]
    return cv2.resize(out,(480,720))
def letterbox(image,width,height):
    h,w=image.shape[:2];scale=min(width/w,height/h)
    small=cv2.resize(image,(round(w*scale),round(h*scale)))
    out=np.zeros((height,width,3),dtype=np.uint8);y=(height-small.shape[0])//2;x=(width-small.shape[1])//2
    out[y:y+small.shape[0],x:x+small.shape[1]]=small;return out
build=genesis_can_env.build_world
def candidate_world(*args,**kwargs):kwargs['urdf_file']=str(urdf);return build(*args,**kwargs)
genesis_can_env.build_world=candidate_world
apply_pre(meta['variant']);env=genesis_can_env.GenesisCanEnv(backend='cpu',render_size=640,max_steps=10**9)
apply_post(env,meta['variant']);w=env.w;cam=w['cam']
overview=dict(pos=np.array(cam.pos),lookat=np.array(cam.lookat),up=np.array(cam.up))
def render():
    w['scene']._visualizer._t=-1;cam._rasterizer._context._t=-1
    return letterbox(np.asarray(cam.render()[0])[:,:,::-1],480,360)
raw=a.out.with_name(a.out.stem+'_raw.mp4');assert not raw.exists()
writer=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),1/.09,(1440,840));assert writer.isOpened()
cap=cv2.VideoCapture(camera['video']);assert cap.isOpened()
frames=list(range(2,len(data['trajectory']),3))
if frames[-1]!=len(data['trajectory'])-1:frames.append(len(data['trajectory'])-1)
matches=[];stills=[]
try:
    for j,i in enumerate(frames):
        desired=offset+(i+1)*.03;camera_frame=int(np.argmin(abs(times-desired)))
        cap.set(cv2.CAP_PROP_POS_FRAMES,camera_frame);ok,real=cap.read();assert ok
        real=cv2.rotate(real,cv2.ROTATE_90_COUNTERCLOCKWISE)
        center=crop_center(times[camera_frame])
        row=data['trajectory'][i];q=data['finger_joint'][i]
        w['kinova'].set_dofs_position(np.r_[row[:6],q],w['kdofs'])
        w['bottle'].set_pos(row[13:16]);w['bottle'].set_quat(row[16:20])
        goal=data['goal_pose'][i];w['goal'].set_pos(goal[:3]);w['goal'].set_quat(goal[3:])
        cam.set_pose(**overview);wide=render()
        rt=Rotation.from_quat(row[[10,11,12,9]]).as_matrix();rg=rt@Rotation.from_euler('z',-np.pi/2).as_matrix()
        pg=row[6:9]-rg@np.array([0,0,.13])
        cam.set_pose(pos=pg+rg@np.array([.30,0,.125]),lookat=pg+rg@np.array([0,0,.125]),up=rg[:,2]);hand=render()
        canvas=np.full((840,1440,3),22,np.uint8)
        canvas[70:790,:480]=letterbox(real,480,720);canvas[70:790,480:960]=crop(real,center)
        canvas[70:430,960:]=wide;canvas[430:790,960:]=hand
        contact=data['contact_counts'][i];gap=(np.linalg.norm(row[13:15]-goal[:2])-.066)*1000
        labels=[(12,26,'Trial 233 | SOFT-PAD CANDIDATE | recorded motion'),
                (12,56,'Real camera 4'),(492,56,'Real hand crop'),(972,56,'Simulation: overview + hand'),
                (12,818,f'Time {(i+1)*.03:.2f} s | Shelf/hand/goal contacts {contact.tolist()} | Can surface gap {gap:.1f} mm'),
                (900,818,'Clock matched; viewpoints differ')]
        for x,y,label in labels:cv2.putText(canvas,label,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.57,(245,245,245),1,cv2.LINE_AA)
        writer.write(canvas)
        matches.append(dict(trace_frame=i,camera_frame=camera_frame,time_s=(i+1)*.03,match_error_s=float(times[camera_frame]-desired)))
        if any(abs((i+1)*.03-t)<.045 for t in [7,14,20,22.5,25,28.86]):stills.append(cv2.resize(canvas,(1008,588)))
        if j%60==0:print('RENDER',j,'/',len(frames),flush=True)
finally:writer.release();cap.release()
subprocess.run(['ffmpeg','-v','error','-i',str(raw),'-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.out)],check=True)
check=cv2.VideoCapture(str(a.out));assert int(check.get(cv2.CAP_PROP_FRAME_COUNT))==len(frames)
for j in [0,len(frames)//2,len(frames)-1]:
    check.set(cv2.CAP_PROP_POS_FRAMES,j);ok,im=check.read();assert ok and im.shape==(840,1440,3)
check.release()
if stills:cv2.imwrite(str(a.out.with_suffix('.jpg')),np.vstack(stills))
a.out.with_suffix('.json').write_text(json.dumps(dict(trace=str(a.trace),trace_sha256=hashlib.sha256(a.trace.read_bytes()).hexdigest(),
    video=str(a.out),video_sha256=hashlib.sha256(a.out.read_bytes()).hexdigest(),urdf_sha256=meta['physics_treatment']['urdf']['candidate_sha256'],
    real_video=camera['video'],timestamp_sha256=camera['timestamp_sha256'],matches=matches,frames=len(frames),fps=1/.09,
    max_camera_match_error_s=max(abs(m['match_error_s']) for m in matches),
    metric=readout['metric'],strict_sequence=meta['sequence'],
    scope='Saved simulation poses, no physics rerun or added settling. Real crop uses fixed cap-camera hypothesis only to center the view. Different viewpoints; not a registered image overlay.'),indent=2))
print('DONE',a.out,flush=True)
