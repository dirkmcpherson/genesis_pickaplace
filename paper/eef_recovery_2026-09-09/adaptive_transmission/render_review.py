"""Render an existing adaptive-hand trace with overview and moving hand close-up."""
from pathlib import Path
import os,sys,json,hashlib
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
os.environ.setdefault('TI_CPU_MAX_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('MPLCONFIGDIR','/tmp/eef-recovery-mpl')
os.environ.setdefault('NUMBA_CACHE_DIR','/tmp/eef-recovery-numba')
import numpy as np,cv2
from scipy.spatial.transform import Rotation
import genesis_can_env
from sim_variant_hook import apply_pre,apply_post

trace=ROOT/'233_active/233_eef_delta.npz'
meta=json.loads(trace.with_suffix('.json').read_text());data=np.load(trace)
stats=json.loads((trace.parent/'transmission_stats.json').read_text())
assert stats['substep_calls']==8+24*len(data['trajectory'])
urdf=Path(meta['physics_treatment']['urdf']['candidate'])
assert hashlib.sha256(urdf.read_bytes()).hexdigest()==meta['physics_treatment']['urdf']['candidate_sha256']
build=genesis_can_env.build_world
def candidate_world(*args,**kwargs):
 kwargs['urdf_file']=str(urdf);return build(*args,**kwargs)
genesis_can_env.build_world=candidate_world
apply_pre(meta['variant'])
env=genesis_can_env.GenesisCanEnv(backend='cpu',render_size=640,max_steps=10**9)
apply_post(env,meta['variant']);w=env.w;cam=w['cam']
overview=dict(pos=np.array(cam.pos),lookat=np.array(cam.lookat),up=np.array(cam.up))
out=ROOT/'233_adaptive_fingers_overview_closeup_raw.mp4'
assert not out.exists(),out
writer=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'mp4v'),1/.09,(1280,720));assert writer.isOpened()
frames=list(range(0,len(data['trajectory']),3))
phase_targets=[0,66,88,155,211,260,320];phase_images=[]
def render():
 w['scene']._visualizer._t=-1;cam._rasterizer._context._t=-1
 rgb=np.asarray(cam.render()[0]);assert rgb.dtype==np.uint8
 return cv2.resize(rgb[:,:,::-1],(640,560))
try:
 for j,i in enumerate(frames):
  row=data['trajectory'][i];q=data['finger_joint'][i]
  w['kinova'].set_dofs_position(np.r_[row[:6],q],w['kdofs'])
  w['bottle'].set_pos(row[13:16]);w['bottle'].set_quat(row[16:20])
  goal=data['goal_pose'][i];w['goal'].set_pos(goal[:3]);w['goal'].set_quat(goal[3:])
  cam.set_pose(**overview);wide=render()
  # Tool frame to gripper-base frame: inverse of the fixed z=130 mm, Rz=90 joint.
  rt=Rotation.from_quat(row[[10,11,12,9]]).as_matrix()
  rg=rt@Rotation.from_euler('z',-np.pi/2).as_matrix()
  pg=row[6:9]-rg@np.array([0,0,.13])
  cam.set_pose(pos=pg+rg@np.array([.30,0,.125]),lookat=pg+rg@np.array([0,0,.125]),up=rg[:,2])
  close=render()
  canvas=np.full((720,1280,3),22,dtype=np.uint8)
  canvas[80:640,:640]=wide;canvas[80:640,640:]=close
  curl=-np.rad2deg(q[2:]-(.149-.676*q[1]))
  contact=data['contact_counts'][i]
  labels=[(14,29,'Trial 233 | EXPERIMENTAL ADAPTIVE FINGERS'),
          (14,59,'Recorded EEF/grip path; saved simulation poses'),(660,59,'Moving close-up of the gripper'),
          (14,670,f'Time {(i+1)*.03:.2f} s | Motor {data["source_grip"][i]:.1f}% | Inward curl L {curl[0]:.1f} / R {curl[1]:.1f} deg'),
          (14,700,f'Shelf / hand / goal contact counts: {contact.tolist()} | Picks and places; final slide criterion fails')]
  for x,y,text in labels:cv2.putText(canvas,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.62,(245,245,245),1,cv2.LINE_AA)
  writer.write(canvas)
  if j in phase_targets:phase_images.append(cv2.resize(canvas,(896,504)))
  if j%60==0:print('RENDER',j,'/',len(frames),flush=True)
finally:writer.release()
assert len(phase_images)==len(phase_targets)
cv2.imwrite(str(ROOT/'233_adaptive_review_phases.jpg'),np.vstack(phase_images))
check=cv2.VideoCapture(str(out));assert int(check.get(cv2.CAP_PROP_FRAME_COUNT))==len(frames)
for j in [0,len(frames)//2,len(frames)-1]:
 check.set(cv2.CAP_PROP_POS_FRAMES,j);ok,img=check.read();assert ok and img.shape==(720,1280,3)
check.release()
(ROOT/'233_adaptive_video.json').write_text(json.dumps(dict(source_trace=str(trace),trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),frames=len(frames),fps=1/.09,trace_indices=frames,scope='Visualization of saved adaptive simulation state, not a fresh physics run. Overview and moving gripper close-up; original .03 s clock sampled every third frame.',raw_video=str(out),decode_checks=[0,len(frames)//2,len(frames)-1]),indent=2))
print('DONE',out,flush=True)
