"""Cross-version fixed-pad bench; it cannot establish a full-task improvement.

The same normal contact parameters, geometry, timestep and explicit numerical
options are used. New-engine cone/impratio comparisons separate tangential
regularization from globally stiffening normal contact.
"""
from pathlib import Path
import argparse, hashlib, json, os
import numpy as np
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('TI_CPU_MAX_NUM_THREADS','1')
import genesis as gs

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--cone',choices=['legacy','pyramidal','elliptic'],required=True)
p.add_argument('--impratio',type=float,default=1)
a=p.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
gs.init(backend=gs.cpu,seed=0,precision='32',logging_level='warning')
opts=dict(constraint_solver=gs.constraint_solver.CG,iterations=100,tolerance=1e-5,
          ls_iterations=50,ls_tolerance=.01,use_contact_island=False,
          max_collision_pairs=1000,integrator=gs.integrator.approximate_implicitfast)
if a.cone!='legacy':
    opts.update(enable_mujoco_compatibility=True,friction_cone=getattr(gs.friction_cone,a.cone),
                impratio=a.impratio,contact_resolution=gs.contact_resolution.convex,
                enable_torsional_friction=False,enable_rolling_friction=False)
scene=gs.Scene(show_viewer=False,sim_options=gs.options.SimOptions(dt=.01,substeps=8),
               rigid_options=gs.options.RigidOptions(**opts))
scene.add_entity(morph=gs.morphs.Plane())
sets=[];radius=.033;height=.101;pad=(.010,.020,.030)
for tc in [.02,.03]:
    for pen in [.0005,.002]:
        for off in [0,.03]:
            idx=len(sets);cx=.4*(idx%4);cy=.5*(idx//4)
            xp=radius+pad[0]/2-pen
            pads=[scene.add_entity(material=gs.materials.Rigid(rho=1000,friction=1),
                  morph=gs.morphs.Box(size=pad,pos=(cx+sgn*xp,cy+off,.3),fixed=True)) for sgn in [-1,1]]
            can=scene.add_entity(material=gs.materials.Rigid(rho=1000,friction=.2),
                  morph=gs.morphs.Cylinder(radius=radius,height=height,pos=(cx,cy,.3),quat=(2**-.5,2**-.5,0,0)))
            sets.append(dict(pad_tc=tc,penetration_m=pen,offset_m=off,pads=pads,can=can,origin=np.array([cx,cy,.3])))
scene.build();solver=scene.sim.rigid_solver
def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
params=solver.geoms_info.sol_params.to_numpy() if a.cone=='legacy' else arr(solver.get_sol_params())
for s in sets:
    for ent in s['pads']+[s['can']]:
        for g in ent.geoms:params[g.idx]=[s['pad_tc'] if ent is not s['can'] else .02,1,.9,.95,.001,.5,2]
if a.cone=='legacy':
    solver.geoms_info.sol_params.from_numpy(params)
    readback=solver.geoms_info.sol_params.to_numpy()
else:
    solver.set_sol_params(params)
    readback=arr(solver.get_sol_params())
np.testing.assert_array_equal(readback,params)
def arr(x):return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
def axis(q):
    w,x,y,z=q
    return np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
records=[]
for s in sets:
    can=s['can'];mass=float(can.get_mass());assert np.isclose(mass,1000*np.pi*radius**2*height,rtol=.01),mass
    arm=arr(can.get_dofs_armature());assert np.allclose(arm,0),arm
    s['axis0']=axis(arr(can.get_quat()).reshape(4))
    s['trace']=[]
    record={k:s[k] for k in ['pad_tc','penetration_m','offset_m']}
    record.update(mass_kg=mass,armature=arm.tolist(),
        geometry=[dict(vertices=len(g._init_verts),vertices_sha256=hashlib.sha256(np.asarray(g._init_verts).tobytes()).hexdigest(),
                       sol_params=params[g.idx].tolist()) for e in s['pads']+[can] for g in e.geoms])
    records.append(record)
for i in range(200):
    scene.step()
    for s in sets:
        pos=arr(s['can'].get_pos()).reshape(3)
        ax=axis(arr(s['can'].get_quat()).reshape(4))
        droop=float(np.rad2deg(np.arccos(np.clip(abs(ax@s['axis0']),-1,1))))
        c=s['can'].get_contacts();f=arr(c['force_a']).reshape(-1,3)
        s['trace'].append([.01*(i+1),*pos,droop,len(f),float(np.linalg.norm(f,axis=1).sum())])
    if i in [9,49,99,199]:print('STEP',i+1,flush=True)
for j,(s,record) in enumerate(zip(sets,records)):
    tr=np.asarray(s['trace']);record['droop_deg_at_0p1_0p5_1_2_s']=tr[[9,49,99,199],4].tolist()
    record['displacement_mm_at_0p1_0p5_1_2_s']=(1000*np.linalg.norm(tr[[9,49,99,199],1:4]-s['origin'],axis=1)).tolist()
    record['contact_force_sum_N_at_0p1_0p5_1_2_s']=tr[[9,49,99,199],6].tolist()
    np.savez_compressed(a.out/f'case{j}.npz',values=tr,columns=np.array(['t','x','y','z','droop_deg','contacts','sum_contact_force_norm_N']))
report=dict(engine_version=gs.__version__,engine_module=gs.__file__,cone=a.cone,impratio=a.impratio,
            options={k:str(v) for k,v in opts.items()},dt=.01,substeps=8,records=records,
            qualification='Fixed pad boxes. No robot or source replay, no localized layered pad law, no full-task evidence. Compare same-engine cones before attributing cross-engine changes; inspect geometry hashes and actual solver parameters.')
(a.out/'report.json').write_text(json.dumps(report,indent=2));print('DONE',a.out,flush=True)
