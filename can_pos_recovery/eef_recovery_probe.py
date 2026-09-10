"""Matched joint/EEF-delta diagnostic. See paper/EEF_GRASP_RECOVERY_2026-09-09.md.

Run one uid/mode per process. Outputs are diagnostics, not a learner dataset.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'baselines'), str(REPO / 'can_pos_recovery')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--uid', type=int, required=True)
    p.add_argument('--mode', choices=('joint', 'eef_delta'), required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--limit', type=int, default=0, help='Diagnostic prefix only; zero means full tape')
    p.add_argument('--threads', type=int, default=1)
    p.add_argument('--ik-unclamped-targets', action='store_true',
                   help='Diagnostic only: match raw joint targets outside URDF limits; physical limits remain active')
    p.add_argument('--archived-pre-only', action='store_true',
                   help='Diagnostic only: omit the post-build hook as the archived early recovery scripts do')
    p.add_argument('--can-offset', nargs=2, type=float, default=[0., 0.], metavar=('DX', 'DY'),
                   help='Declared world-XY placement correction in metres; source position remains unchanged')
    a = p.parse_args()
    # Set before Genesis/Taichi/torch imports. Keep worker caches writable and isolated
    # from installed-engine files; no launch or physics defaults depend on prior runs.
    os.environ['TI_CPU_MAX_NUM_THREADS'] = str(a.threads)
    os.environ['QD_NUM_THREADS'] = str(a.threads)
    os.environ['OMP_NUM_THREADS'] = str(a.threads)
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/eef-recovery-mpl')
    os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', '/tmp/eef-recovery-ti')
    os.environ.setdefault('GS_CACHE_FILE_PATH', '/tmp/eef-recovery-genesis')
    import numpy as np
    from scipy.spatial.transform import Rotation
    import torch
    torch.set_num_threads(a.threads)
    from eef_delta_control import ArmKinematics, EEFDeltaController, from_genesis, pose_delta
    from sim_variant_hook import apply_pre, apply_post
    from replay_harness import load_episode, STATIC_BOTTLE_POSITION, BOX_SIZE, tilt_deg
    from genesis_can_env import GenesisCanEnv

    a.out.mkdir(parents=True, exist_ok=True)
    prefix = a.out / f'{a.uid}_{a.mode}'
    if prefix.with_suffix('.json').exists():
        raise FileExistsError(f'Refusing to replace completed result {prefix}')
    if a.uid < 232:
        winners = json.loads((Path.home() / 'wm_fix_2026-09-03/recover_early/winners.json').read_text())
        record = winners[str(a.uid)]
        variant, can_pos, can_quat = record['sim_variant'], record['can_pos'], [1, 0, 0, 0]
    else:
        record = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials'][str(a.uid)]
        variant, can_pos, can_quat = 'gc_kp4_riser3_shelf6', record['can_pos'], record.get('can_quat')
    source_can_pos = list(can_pos)
    can_pos = [can_pos[0] + a.can_offset[0], can_pos[1] + a.can_offset[1], can_pos[2]]
    os.environ['GENESIS_SIM_VARIANT'] = variant
    apply_pre(variant)
    print('BUILD_START', a.uid, a.mode, variant, flush=True)
    env = GenesisCanEnv(backend='cpu', max_steps=10**9)
    import genesis as gs
    if gs.__version__.startswith('1.'):
        import quadrants as ti
    else:
        import taichi as ti
    effective_threads = ti.lang.impl.get_runtime().prog.config().cpu_max_num_threads
    assert effective_threads == a.threads, (effective_threads, a.threads)
    print('BUILD_DONE', 'cpu_threads', effective_threads, flush=True)
    if not a.archived_pre_only:
        apply_post(env, variant)
    w = env.w
    shelves = [e for e in w['scene'].entities
               if e.morph.__class__.__name__ == 'Box' and np.allclose(e.morph.size, BOX_SIZE)]
    assert len(shelves) == 1
    shelf = shelves[0]
    env.reset(can_pos=can_pos, can_quat=can_quat,
              goal_pos=(*STATIC_BOTTLE_POSITION[:2], w['goal_start_z']))
    robot, wrist, indices = w['kinova'], w['eef'], w['kdofs'][:6]
    array = EEFDeltaController.array
    fk = ArmKinematics(REPO / 'gen3_lite_2f_robotiq_85.urdf')
    base = robot.get_link(fk.base_link)
    mount = from_genesis(array(base.get_pos()), array(base.get_quat()))
    ctrl = EEFDeltaController(robot, wrist, indices, fk.wrist_to_tool,
                              respect_joint_limit=not a.ik_unclamped_targets)
    reset_fk = fk.tool(array(robot.get_dofs_position(dofs_idx_local=indices)), mount)
    fk_error = pose_delta(reset_fk, ctrl.measured_tool())
    print('FK_GATE', fk_error.tolist(), flush=True)
    if np.linalg.norm(fk_error[:3]) > 1e-4 or np.linalg.norm(fk_error[3:]) > np.deg2rad(.01):
        raise AssertionError('URDF FK does not match the built tool frame')
    joints, grip = load_episode(a.uid)
    full_frames = len(joints)
    if a.limit:
        joints, grip = joints[:a.limit], grip[:a.limit]
    previous = ctrl.target.copy()
    rows, targets, actions, ik_errors, qcmds = [], [], [], [], []
    stages, fingers, goal_poses, contact_counts = [], [], [], []
    started = time.monotonic()
    for i, (q, g) in enumerate(zip(joints, grip)):
        desired = fk.tool(q, mount)
        delta = pose_delta(previous, desired)
        previous = desired
        if a.mode == 'eef_delta':
            command = ctrl.command(delta)
            ik_error = ctrl.last_ik_error
        else:
            command = q
            ik_error = np.zeros(6)
        _, _, info = env.step(np.r_[command, np.clip(g / 100., 0, 1)],
                              arm_cmd=command, grip_motor=float(g))
        actual_tool = ctrl.measured_tool()
        bp = array(w['bottle'].get_pos())
        bq = array(w['bottle'].get_quat())
        actual_q = array(robot.get_dofs_position(dofs_idx_local=indices))
        # 6 q, 3 tool xyz, 4 tool wxyz, 3 can xyz, 4 can wxyz, 1 tilt.
        from eef_delta_control import to_genesis
        tp, tq = to_genesis(actual_tool)
        rows.append(np.r_[actual_q, tp, tq, bp, bq, tilt_deg(bq)])
        targets.append(desired)
        actions.append(np.r_[delta, np.clip(g / 100., 0, 1)])
        ik_errors.append(ik_error)
        qcmds.append(command)
        stages.append([info[k] for k in ('picked', 'placed', 'contact', 'contact_push', 'slide_ok', 'slide_success')])
        fingers.append(array(robot.get_dofs_position(dofs_idx_local=w['kdofs'][-4:])))
        goal_poses.append(np.r_[array(w['goal'].get_pos()), array(w['goal'].get_quat())])
        contact_counts.append([len(array(w['bottle'].get_contacts(other)['position']))
                               for other in (shelf, robot, w['goal'])])
        if (i + 1) % 300 == 0:
            print('PROGRESS', a.uid, a.mode, i + 1, '/', len(joints),
                  'picked', info['picked'], 'elapsed', round(time.monotonic() - started, 1), flush=True)
    before_settle = dict(picked=bool(env._picked), contact=bool(env._contact),
                         slide_success=bool(env._slide_success))
    endpoint = env.end_of_episode()
    rows = np.asarray(rows)
    target_array = np.asarray(targets)
    positional = np.linalg.norm(rows[:, 6:9] - target_array[:, :3, 3], axis=1)
    ik_errors = np.asarray(ik_errors)
    qcmds = np.asarray(qcmds)
    # FK of issued joint targets is an independent check on IK's own error report.
    cmd_error = np.asarray([pose_delta(fk.tool(q, mount), t) for q, t in zip(qcmds, targets)])
    np.savez_compressed(prefix.with_suffix('.npz'), trajectory=rows,
                        target_tool=target_array, actions_eef=np.asarray(actions),
                        actions_joint=qcmds, ik_error=ik_errors, command_pose_error=cmd_error,
                        source_joint=joints, source_grip=grip, mount=mount,
                        stages=np.asarray(stages, dtype=bool), finger_joint=np.asarray(fingers),
                        goal_pose=np.asarray(goal_poses), contact_counts=np.asarray(contact_counts))
    summary = dict(uid=a.uid, mode=a.mode, variant=variant, can_pos=can_pos,
                   source_can_pos=source_can_pos, can_offset_xy=a.can_offset,
                   full_tape=len(joints) == full_frames, frames=len(joints), source_frames=full_frames,
                   threads=a.threads, elapsed_s=time.monotonic() - started,
                   ik_respect_joint_limit=not a.ik_unclamped_targets,
                   variant_post_hook=not a.archived_pre_only,
                   actual_arm_kp=array(robot.get_dofs_kp(dofs_idx_local=indices)).tolist(),
                   engine_version=gs.__version__, engine_module=gs.__file__,
                   fk_reset_error=fk_error.tolist(), before_settle=before_settle,
                   endpoint=endpoint,
                   tracking_xyz_m_p50_p95_p99=np.quantile(positional, [.5, .95, .99]).tolist(),
                   command_xyz_m_p99=float(np.quantile(np.linalg.norm(cmd_error[:, :3], axis=1), .99)),
                   command_rotation_rad_p99=float(np.quantile(np.linalg.norm(cmd_error[:, 3:], axis=1), .99)),
                   joint_command_difference_rad_p99=float(np.quantile(np.max(np.abs(qcmds-joints), axis=1), .99)),
                   source_sha256=hashlib.sha256((REPO / f'inthewild_trials/{a.uid}_episodes.npy').read_bytes()).hexdigest())
    summary['stage_columns'] = ['picked', 'placed', 'contact', 'contact_push', 'slide_ok', 'slide_success']
    summary['contact_columns'] = ['can_shelf', 'can_robot', 'can_goal']
    summary['code_sha256'] = {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in [Path(__file__), REPO / 'baselines/eef_delta_control.py',
                                           REPO / 'baselines/genesis_can_env.py',
                                           REPO / 'baselines/sim_variants.py',
                                           REPO / 'can_pos_recovery/replay_harness.py',
                                           REPO / 'gen3_lite_2f_robotiq_85.urdf']}
    prefix.with_suffix('.json').write_text(json.dumps(summary, indent=2))
    print('RESULT', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
