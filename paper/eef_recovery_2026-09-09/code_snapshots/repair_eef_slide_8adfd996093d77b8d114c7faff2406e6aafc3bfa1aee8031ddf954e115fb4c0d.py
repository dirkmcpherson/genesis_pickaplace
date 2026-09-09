"""Bounded final-slide repair and independent playback of saved EEF actions.

Produces explicitly repaired real-derived demos, never silently relabels them as
unaltered human recordings. Physics, initial objects and gripper mapping stay fixed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO/'baselines'), str(REPO/'can_pos_recovery')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--max-extension', type=float, default=.01)
    p.add_argument('--insert-after', type=int)
    p.add_argument('--verify-actions', action='store_true')
    p.add_argument('--polish-ik', action='store_true')
    p.add_argument('--hold-contact', action='store_true',
                   help='At extension limit, allow up to 0.3 seconds holding the tool target to settle contact')
    a = p.parse_args()
    if not 0 <= a.max_extension <= .03:
        raise ValueError('Exploratory extension bound is 0 to 30 mm; default remains 10 mm')
    os.environ['TI_CPU_MAX_NUM_THREADS'] = '1'
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', '/tmp/eef-recovery-ti')
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/eef-recovery-mpl')
    import numpy as np
    import torch
    torch.set_num_threads(1)
    from eef_delta_control import ArmKinematics, EEFDeltaController, FKPrecisionRefiner, from_genesis, pose_delta, to_genesis
    from eef_task_sequence import score_sequence, sustained_starts
    from genesis_can_env import GenesisCanEnv
    from replay_harness import STATIC_BOTTLE_POSITION, BOX_SIZE, tilt_deg
    from sim_variant_hook import apply_pre, apply_post

    source = np.load(a.source)
    meta = json.loads(a.source.with_suffix('.json').read_text())
    input_is_plan = meta.get('source_kind') == 'unexecuted_eef_action_plan'
    used_code_sha256 = {str(path.relative_to(REPO)):hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in [Path(__file__),REPO/'baselines/eef_delta_control.py',
                                    REPO/'can_pos_recovery/eef_task_sequence.py']}
    if not meta.get('variant_post_hook', True):
        raise ValueError('Full world configuration required')
    uid, variant = meta['uid'], meta['variant']
    a.out.mkdir(parents=True, exist_ok=True)
    prefix = a.out/f'{uid}_eef_delta'
    if prefix.with_suffix('.json').exists() or prefix.with_suffix('.npz').exists():
        raise FileExistsError(prefix)
    splice = a.insert_after
    if splice is None and not a.verify_actions and a.max_extension:
        r=source['trajectory'];c=source['contact_counts'];g=source['goal_pose']
        supported=(c[:,0]>0)&(np.abs(r[:,15]-.2205)<.004)&(r[:,20]<20)
        release=sustained_starts(supported&(c[:,1]==0)&source['stages'][:,0])
        gap=np.linalg.norm(r[:,13:15]-g[:,:2],axis=1)-.066
        possible=np.flatnonzero(supported&(c[:,1]>0)&(gap>.002)&(gap<.025))
        possible=possible[possible>release[0]+3] if len(release) else []
        if not len(possible):
            raise ValueError('No supported final push following physical release')
        splice=int(possible[-1])
    print('SETUP', uid, 'splice', splice, 'verify_actions', a.verify_actions, flush=True)
    os.environ['GENESIS_SIM_VARIANT']=variant
    apply_pre(variant)
    env=GenesisCanEnv(backend='cpu',max_steps=10**9)
    apply_post(env,variant)
    w=env.w;robot=w['kinova'];indices=w['kdofs'][:6]
    obs=env.reset(can_pos=meta['can_pos'], can_quat=meta.get('can_quat'),
                  goal_pos=(*STATIC_BOTTLE_POSITION[:2],w['goal_start_z']))
    array=EEFDeltaController.array
    fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
    base=robot.get_link(fk.base_link)
    mount=from_genesis(array(base.get_pos()),array(base.get_quat()))
    ctrl=EEFDeltaController(robot,w['eef'],indices,fk.wrist_to_tool,respect_joint_limit=False)
    use_precision=meta.get('precision_ik',False) if a.verify_actions else a.polish_ik
    refiner=FKPrecisionRefiner(fk,mount) if use_precision else None
    planned_targets=(source['target_tool'] if 'target_tool' in source.files else
                     np.asarray([fk.tool(q,mount) for q in source['joint_waypoints']]))
    shelf=next(e for e in w['scene'].entities if e.morph.__class__.__name__=='Box' and np.allclose(e.morph.size,BOX_SIZE))
    observations=[obs['state'].copy()];rows=[];stages=[];contacts=[];goals=[];fingers=[]
    actions=[];commands=[];targets=[];errors=[];origins=[];kinds=[]

    def step(action, origin, kind):
        command=ctrl.command(action[:6])
        if refiner is not None:
            command=refiner.refine(command,ctrl.target)
        ob,_,info=env.step(np.r_[command,action[6]],arm_cmd=command,grip_motor=float(action[6])*100)
        bp=array(w['bottle'].get_pos());bq=array(w['bottle'].get_quat())
        tp,tq=to_genesis(ctrl.measured_tool())
        rows.append(np.r_[array(robot.get_dofs_position(dofs_idx_local=indices)),tp,tq,bp,bq,tilt_deg(bq)])
        goals.append(np.r_[array(w['goal'].get_pos()),array(w['goal'].get_quat())])
        contacts.append([len(array(w['bottle'].get_contacts(e)['position'])) for e in (shelf,robot,w['goal'])])
        fingers.append(array(robot.get_dofs_position(dofs_idx_local=w['kdofs'][-4:])))
        stages.append([info[k] for k in ('picked','placed','contact','contact_push','slide_ok','slide_success')])
        actions.append(np.array(action));commands.append(command);targets.append(ctrl.target.copy())
        errors.append(pose_delta(fk.tool(command,mount),ctrl.target))
        origins.append(origin);kinds.append(kind);observations.append(ob['state'].copy())

    extension=0.;repair_note='not_requested'
    if a.verify_actions:
        for i,action in enumerate(source['actions_eef']):
            step(action,int(source['source_frame_index'][i]),str(source['action_kind'][i]))
            if (i+1)%300==0:print('PROGRESS',i+1,flush=True)
        extension=meta['extension_m'];repair_note='saved_actions_only'
    else:
        for i,target in enumerate(planned_targets):
            grip=float(source['source_grip'][i])/100
            step(np.r_[pose_delta(ctrl.target,target),grip],i,'recorded_path')
            if i==splice and a.max_extension:
                repair_note='guard_rejected'
                # Insert only while the live can is still on the shelf and the
                # robot is touching it. No attachment or pose teleport is used.
                if contacts[-1][0] and contacts[-1][1] and rows[-1][20]<20:
                    repair_note='extension_limit'
                    touch_run=0
                    for _ in range(int(round(a.max_extension/.0005))):
                        direction=np.asarray(goals[-1][:2])-np.asarray(rows[-1][13:15])
                        direction=direction/max(np.linalg.norm(direction),1e-9)
                        delta=np.r_[.0005*direction,0.,0.,0.,0.]
                        step(np.r_[delta,grip],i,'slide_extension')
                        extension+=.0005
                        if rows[-1][20]>=20 or abs(rows[-1][15]-.2205)>.008:
                            repair_note='support_lost';break
                        touch_run=touch_run+1 if contacts[-1][0] and contacts[-1][2] else 0
                        if touch_run>=3:
                            repair_note='contact_reached';break
                    if a.hold_contact and touch_run<3 and repair_note!='support_lost':
                        for _ in range(10):
                            step(np.r_[np.zeros(6),grip],i,'slide_contact_hold')
                            if rows[-1][20]>=20 or abs(rows[-1][15]-.2205)>.008:
                                repair_note='support_lost';break
                            touch_run=touch_run+1 if contacts[-1][0] and contacts[-1][2] else 0
                            if touch_run>=3:
                                repair_note='contact_held';break
            if (i+1)%300==0:print('PROGRESS',i+1,flush=True)
    data=dict(trajectory=np.asarray(rows),contact_counts=np.asarray(contacts),goal_pose=np.asarray(goals),
              stages=np.asarray(stages,dtype=bool),finger_joint=np.asarray(fingers),observations=np.asarray(observations),
              actions_eef=np.asarray(actions),actions_joint=np.asarray(commands),target_tool=np.asarray(targets),
              command_pose_error=np.asarray(errors),source_grip=np.asarray(actions)[:,6]*100,
              source_frame_index=np.asarray(origins),action_kind=np.asarray(kinds),mount=mount)
    score=score_sequence(data)
    errors=np.asarray(errors)
    summary=dict(uid=uid,mode='eef_delta',variant=variant,variant_post_hook=True,can_pos=meta['can_pos'],
                 ik_respect_joint_limit=False,source=str(a.source.resolve()),
                 source_sha256=meta['source_sha256'],parent_trace_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),
                 frames=len(rows),source_frames=meta.get('source_frames',len(planned_targets)),
                 can_quat=meta.get('can_quat',[1,0,0,0]),
                 extension_m=extension,insert_after=(meta.get('insert_after') if a.verify_actions else splice),
                 repair_note=repair_note,sequence=score,
                 extension_bound_m=(meta.get('extension_bound_m',meta.get('extension_m',0.))
                                    if a.verify_actions else a.max_extension),
                 precision_ik=use_precision,
                 hold_contact=(meta.get('hold_contact',False) if a.verify_actions else a.hold_contact),
                 source_kind='executed_trace',action_plan_execution=a.verify_actions and input_is_plan,
                 action_replay_verification=a.verify_actions and not input_is_plan,
                 provenance=(meta.get('provenance','real-derived EEF replay') if a.verify_actions else
                             ('real-derived bounded final-slide repair' if extension else 'real-derived EEF replay')),
                 observation_schema='GenesisCanEnv 17-vector; N+1 observations paired with N actions',
                 action_schema='world dx/dy/dz metres, left-composed world rotvec radians, grip feedback/100',
                 decision_dt_s=.03,
                 command_xyz_m_p99=float(np.quantile(np.linalg.norm(errors[:,:3],axis=1),.99)),
                 command_rotation_rad_p99=float(np.quantile(np.linalg.norm(errors[:,3:],axis=1),.99)),
                 code_sha256=used_code_sha256)
    for key in ('grip_transform','terminal_hold_frames','suffix_offset_m'):
        if key in meta:
            summary[key]=meta[key]
    if not a.verify_actions and meta.get('grip_transform'):
        summary['provenance'] += ' with disclosed grip correction'
    np.savez_compressed(prefix.with_suffix('.npz'),**data)
    prefix.with_suffix('.json').write_text(json.dumps(summary,indent=2))
    print('RESULT',json.dumps(summary),flush=True)


if __name__=='__main__':
    main()
