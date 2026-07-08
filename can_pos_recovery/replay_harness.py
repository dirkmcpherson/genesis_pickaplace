"""Replay harness that mirrors example.py's sim EXACTLY (scene, gains, gripper mapping,
3 physics steps per 30 Hz command, success test every 30 cmds) but takes per-trial can /
goal-can placements and instruments the three task checkpoints:

  1. picked  - bottle lifted (z > PICK_Z) while the recorded gripper is closed
  2. placed  - bottle resting inside the box (shelf) footprint at shelf height, gripper open
  3. success - example.py's test: bottle contacts goal can AND eef is behind the bottle

Importable (build_world + rollout). Run standalone to time a single trial:
  python can_pos_recovery/replay_harness.py <uid> [canx cany] [goalx goaly]
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, pathlib as pl
import numpy as np
REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import JOINT_NAMES, EEF_NAME

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

BOTTLE_RADIUS = 0.035
BOTTLE_HEIGHT = 0.075
BOX_WIDTH, BOX_HEIGHT = 0.75, 0.12
BOX_POS = (0.75, -BOX_WIDTH / 4, 0.05)
BOX_SIZE = (0.4, BOX_WIDTH, BOX_HEIGHT)
BOX_TOP_Z = BOX_POS[2] + BOX_SIZE[2] / 2                     # 0.11
SHELF_REST_Z = BOX_TOP_Z + BOTTLE_HEIGHT / 2                 # 0.1475
STATIC_BOTTLE_POSITION = (0.6, -0.2, 0.19)
FAR_AWAY = (2.0, 2.0, BOTTLE_HEIGHT / 2)                     # goal can parked out of play

PICK_Z = 0.09           # bottle center height that counts as lifted (validate_grasp.py's value)
GP_CLOSE = 30.0         # recorded gripper motor value above which the gripper is closed

HARDCODED_START = [0.3268500269015339, -1.4471734542578538, 2.3453266624159497,
                   -1.3502152158191212, 2.209384006676201, -1.5125125137062945,
                   -0.96, 0.96, -0.5, -0.5]

def gripper_targets(g_pos, invert=True):
    # identical to example.py
    frac = float(np.clip(g_pos / 100.0, 0.0, 1.0))
    if invert:
        frac = 1.0 - frac
    theta = -0.09 + frac * (0.96 - (-0.09))
    tip = -0.676 * theta + 0.149
    return [-theta, theta, tip, tip]


TABLE_TOP_Z = 0.05   # the robot's mounting surface; bag tool_pose proves the cans sat on it

def build_world(show_viewer=False, backend='gpu', finger_force=None, finger_kp=None,
                can_height=BOTTLE_HEIGHT, can_rho=2000, substeps=1,
                table=False, can_radius=BOTTLE_RADIUS, camera=False):
    """table=True adds the missing table surface (top at z=0.05) under the pick area.
    Trial 232/235 bags: robot-reported tool_pose z at grasp is 0.016-0.039 above the BASE,
    i.e. the humans grasped low on a can standing on the robot's own table -- not on the
    world ground plane 5cm below, where example.py's can rests."""
    """finger_force: None = example.py's current cut (+-10 bottom / +-5 tip);
    a float F = symmetric +-F on all four finger dofs (the handoff doc says the
    current cut regressed pickup).
    finger_kp: None = example.py's (20,20,5,5); a float K = K on all four finger
    dofs (Genesis default is 100). Grip force under position control is kp*error,
    so kp gates squeeze strength as much as the force range does.
    can_height: height of BOTH cylinders. example.py's 0.075 is shorter than the
    real object (FK grasp heights cluster at/above its top); 0.10 matches a soup can."""
    gs.init(backend=gs.gpu if backend == 'gpu' else gs.cpu,
            seed=0, precision="32", logging_level="warning")
    scene = gs.Scene(show_viewer=show_viewer,
                     sim_options=gs.options.SimOptions(dt=0.01, substeps=substeps))
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                     morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_POS))
    if table:
        # pick-area table: top flush with the robot mount (0.05), stops short of the base
        # ends 1mm short of the shelf box front face (x=0.55) -- overlapping the dynamic
        # shelf box ejects it from the scene at build time
        scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                         morph=gs.morphs.Box(size=(0.419, 1.2, 0.05),
                                             pos=(0.3395, -0.1875, 0.025), fixed=True))
    kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / 'gen3_lite_2f_robotiq_85.urdf'),
                                             fixed=True, pos=(0.0, 0.0, 0.05)))
    bottle = scene.add_entity(material=gs.materials.Rigid(rho=can_rho, friction=0.2),
                              morph=gs.morphs.Cylinder(pos=(0.4381, 0.1, 0.05),
                                                       radius=can_radius, height=can_height))
    goal = scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=2.0),
                            morph=gs.morphs.Cylinder(pos=STATIC_BOTTLE_POSITION,
                                                     radius=can_radius, height=can_height))
    kdofs = [kinova.get_joint(n).dof_idx_local for n in JOINT_NAMES]
    eef = kinova.get_link(EEF_NAME)
    cam = None
    if camera:
        cam = scene.add_camera(res=(640, 480), pos=(1.6, 0.9, 0.7),
                               lookat=(0.5, -0.1, 0.15), fov=35, GUI=False)
    scene.build()

    # example.py's gain overrides (arm always; fingers overridable)
    fkp = [20, 20, 5, 5] if finger_kp is None else [finger_kp] * 4
    fkv = [2, 2, 1, 1] if finger_kp is None else [10] * 4   # genesis default kv=10
    kinova.set_dofs_kp(kp=np.array([200, 200, 150, 100, 60, 60] + fkp), dofs_idx_local=kdofs)
    kinova.set_dofs_kv(kv=np.array([20, 20, 15, 10, 6, 6] + fkv), dofs_idx_local=kdofs)
    ff = [10, 10, 5, 5] if finger_force is None else [finger_force] * 4
    kinova.set_dofs_force_range(
        lower=np.array([-50, -50, -50, -20, -20, -20, -ff[0], -ff[1], -ff[2], -ff[3]]),
        upper=np.array([50, 50, 50, 20, 20, 20, ff[0], ff[1], ff[2], ff[3]]),
        dofs_idx_local=kdofs)
    floor_z = TABLE_TOP_Z if table else 0.0
    return dict(scene=scene, kinova=kinova, bottle=bottle, goal=goal, kdofs=kdofs, eef=eef,
                can_h=can_height, can_start_z=floor_z + can_height / 2 + 0.0125,
                goal_start_z=BOX_TOP_Z + can_height / 2 + 0.0425,
                pick_z=floor_z + can_height / 2 + 0.05, cam=cam)


def load_episode(uid):
    d = np.load(REPO / f'inthewild_trials/{uid}_episodes.npy', allow_pickle=True).item()
    return np.asarray(d['vel_cmd']), np.asarray(d['gripper_pos'])[:, 0]


def in_shelf_footprint(p):
    return (BOX_POS[0] - BOX_SIZE[0]/2 < p[0] < BOX_POS[0] + BOX_SIZE[0]/2 and
            BOX_POS[1] - BOX_SIZE[1]/2 < p[1] < BOX_POS[1] + BOX_SIZE[1]/2)


def tilt_deg(quat):
    w_, x, y, z = [float(v) for v in quat]
    zz = 1 - 2 * (x * x + y * y); zx = 2 * (x * z + w_ * y); zy = 2 * (y * z - w_ * x)
    n = (zx * zx + zy * zy + zz * zz) ** 0.5 + 1e-9
    c = max(-1.0, min(1.0, zz / n))
    return float(np.degrees(np.arccos(c)))


def rollout(w, vel, gp, can_pos, goal_pos=None, max_cmds=None, stop_on_success=True,
            settle_steps=100, can_quat=(1, 0, 0, 0)):
    """One open-loop replay. goal_pos=None parks the goal can far away.
    stop_on_success=True mirrors example.py (break at first contact success).
    stop_on_success=False runs the whole episode + settle_steps, then also scores
    NESTED success: both cans upright, in contact, at rest -- the human judge's
    criterion (a grazing touch mid-slide does not count).
    Returns checkpoint dict."""
    scene, kinova, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'],
                                               w['goal'], w['kdofs'], w['eef'])
    gpos = goal_pos if goal_pos is not None else FAR_AWAY
    # reset (mirrors example.py setup(), plus velocity zeroing for run-to-run isolation)
    kinova.set_dofs_position(np.array(HARDCODED_START), kdofs)
    kinova.zero_all_dofs_velocity()
    bottle.set_pos(can_pos); bottle.set_quat(list(can_quat))
    goal.set_pos(gpos); goal.set_quat([1, 0, 0, 0])
    for ent in (bottle, goal):
        try: ent.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()

    n = len(vel) if max_cmds is None else min(max_cmds, len(vel))
    picked_at = placed_at = success_at = -1
    max_z = 0.0
    for i in range(n):
        kinova.control_dofs_position(vel[i], dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.array(gripper_targets(gp[i])), dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):          # steps_per_cmd in example.py
            scene.step()

        bp = np_(bottle.get_pos())
        max_z = max(max_z, float(bp[2]))
        if picked_at < 0 and bp[2] > w.get('pick_z', PICK_Z) and gp[i] > GP_CLOSE:
            picked_at = i
        if placed_at < 0 and picked_at >= 0 and gp[i] < GP_CLOSE and \
           in_shelf_footprint(bp) and BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07:
            placed_at = i
        # contact-success requires the can to have been PICKED first (stricter than
        # example.py's original reward, which counts a can shoved along the table into
        # the goal's base -- see trial 284). picked_at>=0 excludes those false positives.
        if i % 30 == 0 and success_at < 0 and picked_at >= 0:
            c = np_(bottle.get_contacts(goal)['position'])
            if (0 if c.size == 0 else c.shape[0]) and float(np_(eef.get_pos())[0]) < float(bp[0]):
                success_at = i
                if stop_on_success:
                    break
    # Score NESTED at wherever the episode stopped: at first contact-success (the task's
    # semantic end -- the demo's trailing retreat, replayed open-loop, often knocks the
    # nest apart afterward and shouldn't count against it) or at episode end.
    for _ in range(settle_steps):    # let the cans come to rest
        scene.step()
    c = np_(bottle.get_contacts(goal)['position'])
    n_con = 0 if c.size == 0 else c.shape[0]
    b_tilt = tilt_deg(np_(bottle.get_quat()))
    g_tilt = tilt_deg(np_(goal.get_quat()))
    try:
        speed = float(np.linalg.norm(np_(bottle.get_dofs_velocity())[:3]))
    except Exception:
        speed = 0.0
    nested = dict(nested=bool(picked_at >= 0 and n_con > 0 and b_tilt < 20
                              and g_tilt < 20 and speed < 0.05),
                  n_con=int(n_con), bottle_tilt=round(b_tilt, 1),
                  goal_tilt=round(g_tilt, 1), speed=round(speed, 3))
    final_bp = np_(bottle.get_pos())
    return dict(picked=picked_at >= 0, placed=placed_at >= 0, success=success_at >= 0,
                picked_at=picked_at, placed_at=placed_at, success_at=success_at,
                max_z=round(max_z, 4), final_can=[round(float(v), 4) for v in final_bp],
                on_shelf_end=bool(in_shelf_footprint(final_bp) and final_bp[2] > BOX_TOP_Z),
                nested=(nested or {}).get('nested'), nested_detail=nested,
                n_cmds=n)


if __name__ == '__main__':
    import time, json
    uid = int(sys.argv[1]) if len(sys.argv) > 1 else 232
    can = (float(sys.argv[2]), float(sys.argv[3]), 0.05) if len(sys.argv) > 3 else (0.4381, 0.1, 0.05)
    goal = (float(sys.argv[4]), float(sys.argv[5]), 0.19) if len(sys.argv) > 5 else STATIC_BOTTLE_POSITION
    w = build_world()
    vel, gp = load_episode(uid)
    t0 = time.time()
    r = rollout(w, vel, gp, can, goal)
    t1 = time.time()
    print(json.dumps(r))
    print(f"rollout: {t1-t0:.1f}s for {r['n_cmds']} cmds ({3*r['n_cmds']/(t1-t0):.0f} phys steps/s)")
    # second rollout to measure steady-state (first pays JIT warmup)
    t0 = time.time()
    r = rollout(w, vel, gp, can, goal)
    t1 = time.time()
    print(json.dumps(r))
    print(f"rollout2: {t1-t0:.1f}s ({3*r['n_cmds']/(t1-t0):.0f} phys steps/s)")
