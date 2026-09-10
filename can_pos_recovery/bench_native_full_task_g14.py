"""Full-source native benchmark with observed contacts and unchanged task scorers.

Currently requires equal source lengths; duplicated lanes are implementation
controls, not additional recordings. Diagnostic export, not a training tape.
"""
from pathlib import Path
import json,runpy,sys,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(R/'baselines'),str(R/'can_pos_recovery')]
import genesis as gs
import replay_harness as h
from genesis_can_env import PICK_EEF_DIST,PICK_SUSTAIN,GP_CLOSE
from eef_delta_control import ArmKinematics,from_genesis,to_genesis
from eef_task_sequence import score_sequence
from slide_predicate import classify
out=Path(sys.argv[sys.argv.index('--out')+1]);gpu=sys.argv[sys.argv.index('--backend')+1]=='gpu'
paths=[Path(sys.argv[i+1]) for i,arg in enumerate(sys.argv) if arg in ['--trace','--other-trace']]
lengths=[len(np.load(p)['trajectory']) for p in paths]
assert len(set(lengths))==1,'Full-source prototype requires equal lengths; do not truncate or add scored holds'
assert int(sys.argv[sys.argv.index('--steps')+1])>=lengths[0]
original_init=gs.init;original_build=h.build_world;settings={};wrist=[];contacts=[]
def initialize(*args,**kwargs):
 kwargs['use_deterministic_algorithms']=gpu
 return original_init(*args,**kwargs)
gs.init=initialize
def array(x):return x.detach().cpu().numpy()
def build(*args,**kwargs):
 w=original_build(*args,**kwargs);solver=w['scene'].sim.rigid_solver;cfg=solver.rigid_config
 if gpu:assert gs.use_deterministic_algorithms and cfg.prefer_decomposed_solver==1
 settings.update(use_deterministic_algorithms=gpu,prefer_decomposed_solver=int(cfg.prefer_decomposed_solver),para_level=int(cfg.para_level))
 B=solver._B;settings['pick_z']=float(w['pick_z'])
 shelf=next(e for e in w['scene'].entities if e.morph.__class__.__name__=='Box' and np.allclose(e.morph.size,h.BOX_SIZE))
 original_step=w['scene'].step;calls=0
 def observed_step(*args,**kwargs):
  nonlocal calls
  result=original_step(*args,**kwargs);calls+=1
  if calls>1 and (calls-1)%3==0:
   solver.update_forward_pos()
   wrist.append(np.concatenate([array(w['eef'].get_pos()).reshape(B,3),array(w['eef'].get_quat()).reshape(B,4)],axis=1))
   counts=[]
   for entity in [shelf,w['kinova'],w['goal']]:
    c=w['bottle'].get_contacts(entity,is_padded=True)
    counts.append(array(c['valid_mask'].reshape(B,-1).sum(dim=-1)))
   contacts.append(np.stack(counts,axis=-1))
  return result
 w['scene'].step=observed_step
 return w
h.build_world=build
namespace=runpy.run_path(str(R/'can_pos_recovery/bench_native_batch_g14_v2.py'),run_name='__main__')
report=json.loads((out/'report.json').read_text());z=np.load(out/'trace.npz');state=z['state'];N,B,_=state.shape
assert N==lengths[0] and len(wrist)==len(contacts)==N
wrist=np.asarray(wrist);contacts=np.asarray(contacts)
assert contacts.shape==(N,B,3)
fk=ArmKinematics(R/'gen3_lite_2f_robotiq_85.urdf');tool=np.empty_like(wrist,dtype=float)
for t in range(N):
 for b in range(B):
  pos,quat=to_genesis(from_genesis(wrist[t,b,:3],wrist[t,b,3:])@fk.wrist_to_tool)
  tool[t,b]=np.r_[pos,quat]
tilt=np.array([[h.tilt_deg(q) for q in row] for row in state[:,:,13:17]])
trajectory=np.concatenate([state[:,:,:6],tool,state[:,:,10:17],tilt[:,:,None]],axis=-1)
picked=np.zeros((N,B),dtype=bool);run=np.zeros(B,dtype=int);ever=np.zeros(B,dtype=bool)
for t in range(N):
 held=(state[t,:,12]>settings['pick_z']) & (np.clip(z['source_grip'][t],0,100)>GP_CLOSE) & (np.linalg.norm(wrist[t,:,:3]-state[t,:,10:13],axis=-1)<PICK_EEF_DIST)
 run=np.where(held,run+1,0);ever|=run>=PICK_SUSTAIN;picked[t]=ever
results=[];indices=np.arange(3,N,4)
if not len(indices) or indices[-1]!=N-1:indices=np.r_[indices,N-1]
for b in range(B):
 # Only picked is consumed by the strict scorer; do not fabricate other stage flags.
 payload=dict(trajectory=trajectory[:,b],contact_counts=contacts[:,b],goal_pose=state[:,b,17:24],stages=picked[:,b,None])
 sequence=score_sequence(payload)
 # The user metric reads can/goal fields. Unobserved effort is explicitly NaN.
 states=np.full((N,17),np.nan);states[:,:6]=state[:,b,:6];states[:,6]=z['source_grip'][:,b]
 states[:,8:15]=state[:,b,10:17];states[:,15:17]=state[:,b,17:19]
 metric_path=out/f'metric_lane{b}.npz'
 np.savez_compressed(metric_path,uid=int(z['env_uids'][b]),states=states[indices],eef_pos=tool[indices,b,:3],tipped=tilt[indices,b]>60.,sample_time_s=(indices+1)*.03)
 metric=classify(metric_path)
 shift=1000*np.linalg.norm(state[:,b,17:19]-state[0,b,17:19],axis=-1)
 results.append(dict(lane=b,uid=int(z['env_uids'][b]),sequence=sequence,metric=metric,final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max())))
np.savez_compressed(out/'full_task_diagnostic.npz',trajectory=trajectory,contact_counts=contacts,goal_pose=state[:,:,17:24],wrist_pose=wrist,picked=picked,env_uids=z['env_uids'])
report.update(deterministic_dispatch=settings,task_results=results,full_source_length_verified=True,
 diagnostic_sha256=hashlib.sha256((out/'full_task_diagnostic.npz').read_bytes()).hexdigest(),
 qualification='Full equal-length sources, original endpoint, observed contacts and measured tool pose. Diagnostic export only: no independent EEF decoder verification, no complete observation/effort export, no bank admission. Duplicate lanes are not new recordings; timing includes decision-level contact observation.')
(out/'report.json').write_text(json.dumps(report,indent=2))
print('FULL_TASK_RESULT',json.dumps(dict(lanes=B,steps=N,settings=settings,first_lane=results[0])),flush=True)
