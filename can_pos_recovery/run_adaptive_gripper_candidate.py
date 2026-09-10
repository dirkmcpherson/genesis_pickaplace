"""Isolated full EEF replay with the experimental URDF and transmission."""
import argparse,json,runpy,sys,os
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',required=True,type=Path)
p.add_argument('--soft-contact',choices=['none','fingers','can'],default='none')
p.add_argument('--contact-timeconst',type=float,default=.04)
p.add_argument('--record-contact',action='store_true')
p.add_argument('--verify-actions',action='store_true')
p.add_argument('--pad-geometry',choices=['none','hull','split'],default='none')
p.add_argument('--pad-timeconst',type=float,default=.02)
p.add_argument('--pad-depth',type=float,default=.003)
p.add_argument('--distal-lower-limit',type=float,choices=[-.50,-1.03],default=-.50)
a=p.parse_args()
if a.soft_contact!='none' and a.contact_timeconst!=.04:
    raise ValueError('This initial softness comparison declares a single 0.04 s geom time constant')
if a.pad_geometry!='none' and a.soft_contact!='none':
    raise ValueError('Pad experiment cannot also soften whole fingers or can')
if a.pad_geometry!='none' and a.distal_lower_limit!=-.50:
    raise ValueError('Wider travel comparison must retain original collision geometry')
if not .02<=a.pad_timeconst<=.04:raise ValueError('Pad time constant outside declared initial range')
if a.out.exists():raise FileExistsError(a.out)
a.out.mkdir(parents=True)
os.environ['TI_CPU_MAX_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from adaptive_gripper_candidate import make_urdf,install
urdf=a.out/'gen3_lite_2f_adaptive_candidate.urdf'
urdf_info=make_urdf(urdf,distal_lower_limit=a.distal_lower_limit)
if a.pad_geometry!='none':
    from localized_pad_candidate import make_pad_urdf,preserve_pad_meshes
    urdf_info=make_pad_urdf(urdf,a.pad_geometry,a.pad_depth)
    preserve_pad_meshes()
import genesis_can_env,sim_variant_hook
import numpy as np
build=genesis_can_env.build_world
def candidate_world(*args,**kwargs):
    kwargs['urdf_file']=str(urdf.resolve())
    return build(*args,**kwargs)
genesis_can_env.build_world=candidate_world
post=sim_variant_hook.apply_post
state=None;audit=None
contact_rows=[]
pad_contact_rows=[]
def candidate_post(env,variant):
    global state,audit
    result=post(env,variant)
    state,audit=install(env.w)
    w=env.w;solver=w['scene'].sim.rigid_solver
    dofs=np.array(w['kdofs'][-4:])+w['kinova'].dof_start
    limits=solver.dofs_info.limit.to_numpy()[dofs]
    assert np.allclose(limits[2:],[[a.distal_lower_limit,.21]]*2)
    audit['finger_limits_readback']=limits.tolist()
    fingers=[g.idx for link in w['kinova'].links if 'finger' in link.name for g in link.geoms]
    can=list(range(w['bottle'].geom_start,w['bottle'].geom_end))
    assert len(fingers)==(12 if a.pad_geometry=='split' else 4) and len(can)==1
    regions={name:[] for name in ['pad','backing','knuckle']}
    if a.pad_geometry=='split':
        for link in w['kinova'].links:
            if 'finger' not in link.name:continue
            assert len(link.geoms)==3
            for g in link.geoms:
                label=Path(g._metadata['mesh_path']).stem.rsplit('_',1)[-1]
                assert label in regions,label
                regions[label].append(g.idx)
        assert all(len(indices)==4 for indices in regions.values())
    before=solver.geoms_info.sol_params.to_numpy();after=before.copy()
    selected=fingers if a.soft_contact=='fingers' else can if a.soft_contact=='can' else []
    if a.pad_geometry=='split':selected=regions['pad']
    assert np.allclose(before[fingers+can,0],.02), 'Unexpected reference contact softness'
    after[selected,0]=a.pad_timeconst if a.pad_geometry=='split' else a.contact_timeconst
    solver.geoms_info.sol_params.from_numpy(after)
    actual=solver.geoms_info.sol_params.to_numpy();assert np.array_equal(actual,after)
    assert np.array_equal(actual[:,1:],before[:,1:])
    untouched=[i for i in range(len(before)) if i not in selected]
    assert np.array_equal(actual[untouched],before[untouched])
    audit['contact_treatment']=dict(scope=a.soft_contact,selected_geom_indices=selected,
        selected_before=before[selected].tolist(),selected_after=actual[selected].tolist(),
        effective_finger_can_timeconst_s=float((actual[fingers[0],0]+actual[can[0],0])/2),
        interpretation='Soft rigid contact, not mesh deformation. Pair parameters are averaged. Finger treatment also changes finger-other contacts; can treatment changes can-table/shelf/goal contacts. Friction, impedance and damping ratio unchanged.')
    if a.pad_geometry!='none':
        audit['contact_treatment'].update(scope='localized_pads',geometry=a.pad_geometry,
            regions=regions,pad_timeconst_s=a.pad_timeconst,
            effective_finger_can_timeconst_s=None,
            effective_pad_can_timeconst_s=(a.pad_timeconst+.02)/2 if a.pad_geometry=='split' else None,
            friction_readback=solver.geoms_info.friction.to_numpy()[fingers+can].tolist(),
            interpretation='Only designated inner pad cap is softened. Rigid backing, knuckle and manipulated can unchanged. Same link inertia, joint dynamics and exterior hull; decomposition effects require the split rigid control.')
    if a.record_contact:
        from gripper_lab import contact_stats
        original_step=w['scene'].step
        def observed_step(*args,**kwargs):
            result=original_step(*args,**kwargs)
            penetration,force,count=contact_stats(w,set(can),set(fingers))
            contact_rows.append([penetration,force,count])
            if a.pad_geometry=='split':
                pad_contact_rows.append([contact_stats(w,set(can),set(regions[label])) for label in ['pad','backing','knuckle']])
            return result
        w['scene'].step=observed_step
    audit['urdf']=urdf_info;audit['variant']=variant
    audit['source']=str(a.source.resolve())
    (a.out/'transmission_audit.json').write_text(json.dumps(audit,indent=2))
    return result
sim_variant_hook.apply_post=candidate_post
sys.argv=['repair_eef_slide.py',str(a.source),'--out',str(a.out),'--max-extension','0','--polish-ik']
if a.verify_actions:sys.argv.append('--verify-actions')
try:
    runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'),run_name='__main__')
finally:
    if state is not None:
        stats={k:(v.tolist() if hasattr(v,'tolist') else v) for k,v in state.items()}
        (a.out/'transmission_stats.json').write_text(json.dumps(stats,indent=2))
    if a.record_contact and contact_rows:
        np.savez_compressed(a.out/'contact_observations.npz',
            values=np.asarray(contact_rows),columns=np.array(['max_finger_can_penetration_m','max_contact_force_norm_N','finger_can_contacts']),
            scope=np.array('One sample after each original 0.01 s scene step, including reset. Contact solver data at last substep; not material strain.'))
    if pad_contact_rows:
        np.savez_compressed(a.out/'pad_contact_observations.npz',values=np.asarray(pad_contact_rows),
            regions=np.array(['pad','backing','knuckle']),columns=np.array(['max_penetration_m','max_contact_force_norm_N','contacts']))
for path in a.out.glob('*_eef_delta.json'):
    assert state is not None and state['substep_calls']>0
    meta=json.loads(path.read_text());meta['physics_treatment']=audit
    meta['provenance']='Experimental adaptive-transmission replay; not calibrated or bank admitted'
    path.write_text(json.dumps(meta,indent=2))
