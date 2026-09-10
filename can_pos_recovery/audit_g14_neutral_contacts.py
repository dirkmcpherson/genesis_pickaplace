"""Detect the otherwise-filtered fingertip pair at saved poses, without integration."""
from pathlib import Path
import argparse,hashlib,json,os,sys
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'baselines'),str(R/'can_pos_recovery')]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trace',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();assert not a.out.exists()
import genesis as gs
from genesis.utils.misc import qd_to_numpy
import genesis_can_env
from sim_variant_hook import apply_pre,apply_post
z=np.load(a.trace);m=json.loads(a.trace.with_suffix('.json').read_text());urdf=Path(m['physics_treatment']['urdf']['candidate']);assert hashlib.sha256(urdf.read_bytes()).hexdigest()==m['physics_treatment']['urdf']['candidate_sha256']
build=genesis_can_env.build_world
opts=dict(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,ls_iterations=50,ls_tolerance=.01,use_contact_island=False,integrator=gs.integrator.approximate_implicitfast,enable_mujoco_compatibility=True,friction_cone=gs.friction_cone.pyramidal,impratio=1,contact_resolution=gs.contact_resolution.convex,enable_torsional_friction=False,enable_rolling_friction=False,enable_neutral_collision=True)
def world(*args,**kwargs):
 kwargs.update(urdf_file=str(urdf.resolve()),urdf_extra={'default_armature':.1},rigid_extra=opts);return build(*args,**kwargs)
genesis_can_env.build_world=world;os.environ['GENESIS_SIM_VARIANT']=m['variant'];apply_pre(m['variant']);env=genesis_can_env.GenesisCanEnv(backend='cpu',max_steps=10**9);apply_post(env,m['variant']);w=env.w;s=w['scene'].sim.rigid_solver;robot=w['kinova'];c=s.collider;ids=[robot.get_link(n).geoms[0].idx for n in ['right_finger_dist_link','left_finger_dist_link']];assert any(set(pair)==set(ids) for pair in c._valid_collision_pairs)
contacts=[]
for i,row in enumerate(z['trajectory']):
 robot.set_dofs_position(np.r_[row[:6],z['finger_joint'][i]],w['kdofs']);s.update_forward_pos();c.detection();n=int(qd_to_numpy(c.collider_state.n_contacts)[0]);ga=qd_to_numpy(c.collider_state.contact_data.geom_a)[:n,0];gb=qd_to_numpy(c.collider_state.contact_data.geom_b)[:n,0];mask=((ga==ids[0])&(gb==ids[1]))|((ga==ids[1])&(gb==ids[0]))
 if mask.any():
  pen=qd_to_numpy(c.collider_state.contact_data.penetration)[:n,0][mask];contacts.append(dict(frame=i,time_s=(i+1)*.03,count=int(mask.sum()),max_penetration_m=float(pen.max())))
 if (i+1)%1000==0:print('POSES',i+1,flush=True)
a.out.write_text(json.dumps(dict(trace=str(a.trace),trace_sha256=hashlib.sha256(a.trace.read_bytes()).hexdigest(),uid=m['uid'],variant=m['variant'],neutral_pair_enabled=True,geoms=ids,frames=len(z['trajectory']),contact_frames=contacts,qualification='Collision detection only at every saved30ms decision pose; no physics integration or trajectory modification. Unobserved intermediate physics substeps are not covered. This cannot measure the effect of re-enabling the pair on a dynamic replay.'),indent=2));print('DONE',m['uid'],'frames',len(z['trajectory']),'pair_contacts',len(contacts),flush=True)
