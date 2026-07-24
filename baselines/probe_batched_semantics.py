"""Probe the two unknowns blocking the batched-Genesis adapter (BATCHED_ENV_PLAN §4).

Runs ONE configuration per invocation (genesis allows one scene per process); the
driver in __main__ spawns itself as subprocesses for each config and merges answers.

  probe A (env_separate_rigid=True):
    1. eef.get_pos() batched shape + start-state equality across envs
    2. per-env reset isolation: reset env 1 mid-run; envs 0,2,3 must stay BIT-IDENTICAL
       to an unperturbed repeat run (phase-repeat baseline measured first)
    3. static-camera batched render: one call -> (N,H,W,3)? cost at 64x64
    4. wrist strategy S1: N x (set_pose to env i's wrist, render, take image i) -- cost

  probe B (env_separate_rigid=False, env_spacing=4m):
    5. wrist strategy S2: shared tiled world, camera posed AT env i's cell -> ONE image
       per render call. cost + contamination check (does env i's image contain other
       envs' geometry? judged by comparing against a reference render with only env i
       actively posed... here: pixel-diff between two envs' wrist views of IDENTICAL
       states -- identical states => identical views IF no contamination).

Usage: probe_batched_semantics.py            (driver: runs A then B, prints verdicts)
       probe_batched_semantics.py --config A|B --n 4 --timing-n 16
"""
import os
import argparse, json, subprocess, sys, time, pathlib as pl

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
PY = sys.executable

ap = argparse.ArgumentParser()
ap.add_argument('--config', choices=('A', 'B'), default=None)
ap.add_argument('--n', type=int, default=4)
ap.add_argument('--timing-n', type=int, default=16)
ap.add_argument('--out', default=None)
args = ap.parse_args()


def build_world(n_envs, env_separate_rigid, env_spacing, res=64):
    sys.path.insert(0, str(REPO))
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    import genesis as gs
    from kinova import JOINT_NAMES, EEF_NAME
    from replay_harness import (BOTTLE_RADIUS, BOX_POS, BOX_SIZE, STATIC_BOTTLE_POSITION,
                                joint_dofs)
    gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
    scene = gs.Scene(
        show_viewer=False,
        sim_options=gs.options.SimOptions(dt=0.01, substeps=8),
        vis_options=gs.options.VisOptions(env_separate_rigid=env_separate_rigid,
                                          show_world_frame=False),
    )
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                     morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_POS))
    scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                     morph=gs.morphs.Box(size=(0.419, 1.2, 0.05),
                                         pos=(0.3395, -0.1875, 0.025), fixed=True))
    kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                             fixed=True, pos=(0.0, 0.0, 0.05)))
    bottle = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.2),
                              morph=gs.morphs.Cylinder(pos=(0.4381, 0.1, 0.113),
                                                       radius=0.033, height=0.101))
    goal = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=2.0),
                            morph=gs.morphs.Cylinder(pos=STATIC_BOTTLE_POSITION,
                                                     radius=0.033, height=0.101))
    cam_top = scene.add_camera(res=(res, res), pos=(0.40, -0.08, 1.15),
                               lookat=(0.40, -0.08, 0.10), up=(1, 0, 0), fov=68, GUI=False)
    cam_wrist = scene.add_camera(res=(res, res), pos=(0, 0, 1.0), lookat=(0, 0, 0),
                                 fov=80, GUI=False)
    scene.build(n_envs=n_envs, env_spacing=env_spacing)
    kdofs = [joint_dofs(kinova.get_joint(n)) for n in JOINT_NAMES]
    eef = kinova.get_link(EEF_NAME)
    return dict(gs=gs, scene=scene, kinova=kinova, bottle=bottle, goal=goal,
                kdofs=kdofs, eef=eef, cam_top=cam_top, cam_wrist=cam_wrist)


def np_(x):
    try:
        return x.detach().cpu().numpy()
    except AttributeError:
        return np.asarray(x)


def reset_all(w, n):
    """Deterministic full reset of all envs to identical start state."""
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    from replay_harness import HARDCODED_START
    q = np.tile(np.array(HARDCODED_START), (n, 1))
    w['kinova'].set_dofs_position(q, w['kdofs'])
    # CRITICAL for determinism: reset_all steps the scene once below, and that step
    # runs under whatever control targets LINGER from before the reset. Command the
    # start pose explicitly so the reset step is identical regardless of history.
    # (First probe run skipped this -> 7.5cm "nondeterminism" that was really stale
    # targets from the previous phase acting during the reset step.)
    w['kinova'].control_dofs_position(q, dofs_idx_local=w['kdofs'])
    w['kinova'].zero_all_dofs_velocity()
    w['bottle'].set_pos(np.tile((0.4381, 0.1, 0.113), (n, 1)))
    w['bottle'].set_quat(np.tile((1.0, 0, 0, 0), (n, 1)))
    w['goal'].set_pos(np.tile((0.672, -0.221, 0.19), (n, 1)))
    w['goal'].set_quat(np.tile((1.0, 0, 0, 0), (n, 1)))
    w['bottle'].zero_all_dofs_velocity(); w['goal'].zero_all_dofs_velocity()
    w['scene'].step()


def action_seq(n, t, k):
    """Deterministic wiggle: distinct per env, repeatable across phases."""
    base = np.array([0.33, -1.45, 2.35, -2.4, 0.85, -1.6])
    return base + 0.15 * np.sin(0.05 * t + np.arange(n)[:, None] * 0.7 + np.arange(6) * 0.3)


def run_A(res):
    n = args.n
    w = build_world(n, env_separate_rigid=True, env_spacing=(0.0, 0.0))
    out = {}
    # 1. batched link readout
    reset_all(w, n)
    p = np_(w['eef'].get_pos()); q = np_(w['eef'].get_quat())
    out['eef_pos_shape'] = list(p.shape)
    out['start_state_equal_across_envs'] = bool(np.allclose(p - p[0], 0, atol=0))

    # 2. per-env reset isolation (with phase-repeat baseline)
    def rollout(reset_env1_at=None, steps=50):
        reset_all(w, n)
        traj = []
        for t in range(steps):
            if t == reset_env1_at:
                from replay_harness import HARDCODED_START
                w['kinova'].set_dofs_position(np.array(HARDCODED_START), w['kdofs'],
                                              envs_idx=[1])
                w['bottle'].set_pos(np.array([[0.45, 0.05, 0.113]]), envs_idx=[1])
                w['bottle'].set_quat(np.array([[1.0, 0, 0, 0]]), envs_idx=[1])
                w['kinova'].zero_all_dofs_velocity(envs_idx=[1])
                w['bottle'].zero_all_dofs_velocity(envs_idx=[1])
            w['kinova'].control_dofs_position(action_seq(n, t, 0),
                                              dofs_idx_local=w['kdofs'][:6])
            w['scene'].step()
            traj.append(np.concatenate([np_(w['kinova'].get_dofs_position(
                dofs_idx_local=w['kdofs'])), np_(w['bottle'].get_pos())], axis=-1))
        return np.array(traj)   # (T, n, dofs+3)

    tA = rollout(None)
    tA2 = rollout(None)                    # phase-repeat baseline
    tB = rollout(reset_env1_at=25)
    others = [i for i in range(n) if i != 1]
    out['phase_repeat_bitexact'] = bool(np.array_equal(tA, tA2))
    out['others_bitexact_after_env1_reset'] = bool(
        np.array_equal(tA[:, others], tB[:, others]))
    out['others_maxdiff'] = float(np.abs(tA[:, others] - tB[:, others]).max())
    out['env1_actually_reset'] = bool(np.abs(tA[25:, 1] - tB[25:, 1]).max() > 1e-6)

    # 3. static camera batched render
    r = w['cam_top'].render()[0]
    arr = np.asarray(r)
    out['top_render_shape'] = list(arr.shape)
    t0 = time.time()
    for _ in range(10):
        w['cam_top'].render()
    out['top_render_ms'] = (time.time() - t0) / 10 * 1e3
    out['n_probe'] = args.n

    # 4. wrist strategy S1: per-env set_pose + render + take image i
    def link_T(i):
        pos = np_(w['eef'].get_pos())[i]; quat = np_(w['eef'].get_quat())[i]
        import genesis.utils.geom as gu
        return gu.trans_quat_to_T(pos, quat)
    offT = np.eye(4); offT[:3, 3] = (0.10, 0.0, -0.03)
    t0 = time.time()
    for i in range(n):
        w['cam_wrist'].set_pose(transform=link_T(i) @ offT)
        img = np.asarray(w['cam_wrist'].render()[0])
    out['wrist_S1_percall_ms'] = (time.time() - t0) / n * 1e3
    out['wrist_S1_img_shape'] = list(img.shape)
    return out


def run_B(res):
    n = args.timing_n
    w = build_world(n, env_separate_rigid=False, env_spacing=(4.0, 4.0))
    out = {}
    reset_all(w, n)
    for t in range(5):
        w['kinova'].control_dofs_position(action_seq(n, t, 0), dofs_idx_local=w['kdofs'][:6])
        w['scene'].step()
    # env grid offsets (recomputed as scene lays them out)
    offs = np_(w['scene'].envs_offset)
    out['envs_offset_shape'] = list(offs.shape)
    # physics coords unaffected by spacing?
    p = np_(w['eef'].get_pos())
    out['physics_coords_local'] = bool(np.abs(p[:, :2]).max() < 2.0)  # no 4m offsets in physics

    import genesis.utils.geom as gu
    offT = np.eye(4); offT[:3, 3] = (0.10, 0.0, -0.03)

    def wrist_img(i):
        pos = np_(w['eef'].get_pos())[i] + offs[i]
        quat = np_(w['eef'].get_quat())[i]
        T = gu.trans_quat_to_T(pos, quat) @ offT
        w['cam_wrist'].set_pose(transform=T)
        return np.asarray(w['cam_wrist'].render()[0])

    t0 = time.time()
    imgs = [wrist_img(i) for i in range(n)]
    out['wrist_S2_percall_ms'] = (time.time() - t0) / n * 1e3
    out['wrist_S2_img_shape'] = list(imgs[0].shape)
    # contamination: identical states -> identical views iff each env sees only itself
    d01 = np.abs(imgs[0].astype(int) - imgs[1].astype(int))
    out['wrist_S2_view_maxdiff_env0_vs_env1'] = int(d01.max())
    out['wrist_S2_view_meandiff'] = float(d01.mean())
    out['wrist_S2_img_nontrivial'] = bool(imgs[0].std() > 5)
    # top-down per-env, same trick
    t0 = time.time()
    for i in range(n):
        w['cam_top'].set_pose(pos=tuple(np.array([0.40, -0.08, 1.15]) + offs[i]),
                              lookat=tuple(np.array([0.40, -0.08, 0.10]) + offs[i]),
                              up=(1, 0, 0))
        w['cam_top'].render()
    out['top_S2_percall_ms'] = (time.time() - t0) / n * 1e3
    # physics step cost at this n for reference
    t0 = time.time()
    for _ in range(30):
        w['scene'].step()
    out['scene_step_ms'] = (time.time() - t0) / 30 * 1e3
    return out


if args.config:
    res = run_A(64) if args.config == 'A' else run_B(64)
    print("PROBE_JSON:" + json.dumps(res))
    sys.exit(0)

# driver
results = {}
for cfg in ('A', 'B'):
    r = subprocess.run([PY, __file__, '--config', cfg, '--n', str(args.n),
                        '--timing-n', str(args.timing_n)],
                       capture_output=True, text=True)
    line = [l for l in r.stdout.splitlines() if l.startswith('PROBE_JSON:')]
    if not line:
        print(f'--- config {cfg} FAILED ---')
        print(r.stdout[-1500:]); print(r.stderr[-1500:])
        sys.exit(1)
    results[cfg] = json.loads(line[0][len('PROBE_JSON:'):])
    print(f'--- config {cfg} ---')
    for k, v in results[cfg].items():
        print(f'  {k}: {v}')

if args.out:
    pl.Path(args.out).write_text(json.dumps(results, indent=1))
