import argparse
import genesis as gs
import time

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--vis", action="store_true", default=False)
parser.add_argument("-d", "--debug", action="store_true", default=False)
parser.add_argument("-p", "--position", type=int, default=-1)
parser.add_argument("-t", "--trial", type=int, nargs="+", default=None,
                    help="run only these trial id(s), in order, e.g. -t 232  or  -t 232 245 300")
parser.add_argument("--legacy", action="store_true", default=False,
                    help="old behavior: 3-bucket placement, 0.075 can, weak fingers")
parser.add_argument("--cpu", action="store_true", default=False,
                    help="CPU backend (much faster for this single-env scene)")
args = parser.parse_args()

assert args.position in [-1, 0, 1, 2], "Position must be -1, 0, 1, or 2"

# Per-trial placements recovered by can_pos_recovery/ (see CAN_STARTING_POSITION.md).
# The table also fixes world mismatches found during recovery: the real can was taller
# (0.10 vs 0.075) and lighter (rho 1000 vs 2000) than the original cylinder, the finger
# gain/force cut couldn't grip at all, and substeps=1 dropped the can mid-carry.
import json
import pathlib as pl
_placements_file = pl.Path(__file__).parent / 'can_pos_recovery/trial_placements.json'
PLACEMENTS = None
if not args.legacy and _placements_file.exists():
    _tbl = json.loads(_placements_file.read_text())
    WORLD = _tbl['world']
    # success-labeled only: fail-labeled demos must not be replayed with a goal can
    # relocated to "where the can went" -- that would manufacture success from a demo
    # that failed in the real world.
    # AUDIT QUARANTINE: per-trial relocated goals (goal_moved) are excluded by default;
    # panel-measured ~40-69% false-positive rate on known failures. The can position is
    # still per-trial (trustworthy, tool_pose-validated); only the goal stays static.
    PLACEMENTS = {int(u): r for u, r in _tbl['trials'].items()
                  if r['status'] in ('ok', 'ok_batch') and r.get('label') == 'success'
                  and not r.get('goal_moved')}
    print(f"Loaded {len(PLACEMENTS)} per-trial placements (world: {WORLD}); --legacy to disable")

gs.init(backend=gs.cpu if args.cpu else gs.gpu, seed=0, precision="32", logging_level="warning")


scene = gs.Scene(
    show_viewer=args.vis,
    sim_options=gs.options.SimOptions(
        dt=0.01,   # 100 Hz physics, stated explicitly
        substeps=WORLD['substeps'] if PLACEMENTS else 1,
    ),
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

BOTTLE_RADIUS = WORLD.get('can_radius', 0.035) if PLACEMENTS else 0.035
BOTTLE_HEIGHT = WORLD['can_height'] if PLACEMENTS else 0.075
BOTTLE_RHO = WORLD['can_rho'] if PLACEMENTS else 2000
TABLE_TOP_Z = 0.05 if (PLACEMENTS and WORLD.get('table')) else 0.0
BOX_WIDTH, BOX_HEIGHT = 0.75, 0.12

box = scene.add_entity(
    material=gs.materials.Rigid(rho=1000,
                                friction=0.5),
    morph=gs.morphs.Box(
        size=(0.4, BOX_WIDTH, BOX_HEIGHT),
        pos=(0.75, -BOX_WIDTH / 4, 0.05),
    ),
)

if TABLE_TOP_Z > 0:
    # The missing table: bag tool_pose shows the cans sat on the robot's mounting surface
    # (z=0.05), not the ground plane. Ends 1mm short of the shelf box front face (x=0.55);
    # overlapping the dynamic box ejects it at build time.
    table = scene.add_entity(
        material=gs.materials.Rigid(rho=1000, friction=0.5),
        morph=gs.morphs.Box(size=(0.419, 1.2, 0.05), pos=(0.3395, -BOX_WIDTH / 4, 0.025),
                            fixed=True),
    )

import pathlib as pl
kinova = scene.add_entity(
    # gs.morphs.URDF(
    #     file='urdf/panda_bullet/panda.urdf',
    #     fixed=True,
    # ),
    # gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    gs.morphs.URDF(
        # file='/home/j/catkin_ws/src/ros_kortex/kortex_description/robots/gen3.urdf',
        file=str(pl.Path(__file__).parent / 'gen3_lite_2f_robotiq_85.urdf'),
        fixed=True,
        pos=(0.0, 0.0, 0.05), # raise to account for table mount
    ),

    # gs.morphs.MJCF(file="/home/j/workspace/genesis_pickaplace/005_tomato_soup_can/google_512k/kinbody.xml"),
)

CAN_START_Z = TABLE_TOP_Z + BOTTLE_HEIGHT / 2 + 0.0125   # small settle drop onto the table
GOAL_START_Z = 0.11 + BOTTLE_HEIGHT / 2 + 0.0425         # same settle drop onto the shelf top
STATIC_BOTTLE_POSITION = (0.672, -0.221, GOAL_START_Z)   # human-validated 2026-07-19 (233+242 nest; user picked SOUTH)
POSITION_0 = (0.4381, 0.1, CAN_START_Z)
POSITION_1 = (0.4381, -0.05, CAN_START_Z)
POSITION_2 = (0.4381, -0.2, CAN_START_Z)



bottle = scene.add_entity(
    material=gs.materials.Rigid(rho=BOTTLE_RHO,
                                friction=0.2),
    morph=gs.morphs.Cylinder(
        pos=POSITION_0,
        radius=BOTTLE_RADIUS,
        height=BOTTLE_HEIGHT,
    ),
    visualize_contact=True,
)

goal_bottle = scene.add_entity(
    material=gs.materials.Rigid(rho=1000,
                            friction=2.0),
    morph=gs.morphs.Cylinder(
        pos=STATIC_BOTTLE_POSITION,
        radius=BOTTLE_RADIUS,
        height=BOTTLE_HEIGHT,
    ),
    visualize_contact=True,
)


from kinova import JOINT_NAMES as kinova_joint_names, EEF_NAME as kinova_eef_name, TRIALS_POSITION_0, TRIALS_POSITION_1, TRIALS_POSITION_2
kdofs_idx = [kinova.get_joint(name).dof_idx_local for name in kinova_joint_names]
eef = kinova.get_link(kinova_eef_name)
print(f"Kinova end effector: {eef}")
scene.build()

############ Optional: set control gains ############
import numpy as np
print(f"Control gains for kinova (Genesis defaults, BEFORE override):")
print(" kp ", kinova.get_dofs_kp(dofs_idx_local=kdofs_idx))
print(" kv ", kinova.get_dofs_kv(dofs_idx_local=kdofs_idx))
print(" fr ", kinova.get_dofs_force_range(dofs_idx_local=kdofs_idx))
# Gen3 Lite gains. Order matches kinova.JOINT_NAMES: joint_1..6, then
# left_finger_bottom, right_finger_bottom, left_finger_tip, right_finger_tip.
# These are starting points for a small arm -- tune kp up until it tracks without oscillating.
# Finger gains: the old cut (kp 20/5, force +-10/+-5) cannot hold the can at all (it slips
# out during the carry -- see CAN_STARTING_POSITION.md). With placements active we restore
# Genesis-default finger kp/kv and a +-50 force range, which grips reliably.
if PLACEMENTS:
    _fkp = [WORLD['finger_kp']] * 4; _fkv = [10] * 4; _ff = [WORLD['finger_force']] * 4
else:
    _fkp = [20, 20, 5, 5]; _fkv = [2, 2, 1, 1]; _ff = [10, 10, 5, 5]
kinova.set_dofs_kp(
    kp             = np.array([200, 200, 150, 100, 60, 60] + _fkp),
    dofs_idx_local = kdofs_idx,
)
kinova.set_dofs_kv(
    kv             = np.array([ 20,  20,  15,  10,  6,  6] + _fkv),
    dofs_idx_local = kdofs_idx,
)
kinova.set_dofs_force_range(
    lower          = np.array([-50,-50,-50,-20,-20,-20] + [-f for f in _ff]),
    upper          = np.array([ 50, 50, 50, 20, 20, 20] + _ff),
    dofs_idx_local = kdofs_idx,
)
print(f"Control gains for kinova (AFTER override -- what the sim actually uses):")
print(" kp ", kinova.get_dofs_kp(dofs_idx_local=kdofs_idx))
print(" kv ", kinova.get_dofs_kv(dofs_idx_local=kdofs_idx))
print(" fr ", kinova.get_dofs_force_range(dofs_idx_local=kdofs_idx))

# Genesis does NOT merge the URDF <mimic> finger joints: all four are independent DOFs
# (dof_idx_local left_bottom=8, right_bottom=6, left_tip=9, right_tip=7). So we must set
# all four consistently via gripper_targets() or the gripper geometry fights itself.

print(f"Hard setting kinova joint positions")
harcoded_start = [0.3268500269015339, -1.4471734542578538, 2.3453266624159497, -1.3502152158191212, 2.209384006676201, -1.5125125137062945,  -0.96, 0.96, -0.5, -0.5]  # fingers = open config, matches gripper_targets(0, invert=True)
kinova.set_dofs_position(np.array(harcoded_start), kdofs_idx)


gripper_open = np.array([-1.0, 1.0])
gripper_closed = np.array([0.0, 0.0])

reward_check_period = 30

positions = [POSITION_0, POSITION_1, POSITION_2]
debug_spheres = scene.draw_debug_spheres(poss=positions, radius=0.05, color=(1, 1, 0, 0.5))  # Yellow
# debug_spheres = scene.draw_debug_spheres(poss=[STATIC_BOTTLE_POSITION], radius=0.05, color=(0.5, 1, 0, 1.0))  # Yellow
# if args.debug:
#     n = 10
#     # teleport the bottle between the 3 positions
#     import itertools
#     for position in itertools.cycle(positions):
#         bottle.set_pos(position)
#         scene.step()
#         time.sleep(0.25)
#         n -= 1
#         if n == 0: break


def gripper_targets(g_pos, invert=False):
    # Map recorded gripper motor position (0 open .. 100 closed) to the four finger joint
    # targets using the URDF <mimic> relations. Genesis treats all four as independent DOFs,
    # so they must be set consistently. Driver = right_finger_bottom, [-0.09 open .. 0.96 closed].
    frac = float(np.clip(g_pos / 100.0, 0.0, 1.0))
    if invert:
        frac = 1.0 - frac
    theta = -0.09 + frac * (0.96 - (-0.09))
    tip = -0.676 * theta + 0.149
    # order = kinova.JOINT_NAMES fingers: [left_bottom, right_bottom, left_tip, right_tip]
    return [-theta, theta, tip, tip]


total_reward = 0

# read the trial data
dir = pl.Path('./inthewild_trials/')
if args.trial:
    paths = [dir / f"{tid}_episodes.npy" for tid in args.trial]
    missing = [str(p) for p in paths if not p.exists()]
    if missing:
        raise SystemExit(f"trial file(s) not found: {missing}")
else:
    paths = sorted(dir.glob("*_episodes.npy"), key=lambda p: int(p.stem.split('_')[0]))

for path in paths:
    path = str(path)

    ep_dict = np.load(path, allow_pickle=True).item()
    vel_cmd = ep_dict['vel_cmd']
    gripper_cmds = np.linspace(-1.0, 1.0, len(vel_cmd) + 1)
    gripper_pos = ep_dict['gripper_pos']
    cmd_idx = 0


    def setup():
        kinova.set_dofs_position(np.array(harcoded_start), kdofs_idx)
        goal_bottle.set_pos(STATIC_BOTTLE_POSITION)
        goal_bottle.set_quat([1, 0, 0, 0])

        uid = int(path.split('/')[-1].split('_')[0])
        can_quat = [1, 0, 0, 0]
        if PLACEMENTS and uid in PLACEMENTS:
            r = PLACEMENTS[uid]
            if args.position >= 0 and r.get('pos') != args.position: return False
            bottle.set_pos(r['can_pos'])
            # some demos start with the can knocked over (fallen-can variant)
            can_quat = r.get('can_quat') or [1, 0, 0, 0]
            # goal stays at the single corrected static pos (#24/#27); do NOT use the stale
            # per-trial r['goal_pos'] (computed under the old goal) -- matches render_all_episodes.
        elif uid in TRIALS_POSITION_0:
            if args.position >= 0 and args.position != 0: return False
            bottle.set_pos(POSITION_0)
        elif uid in TRIALS_POSITION_1:
            if args.position >= 0 and args.position != 1: return False
            bottle.set_pos(POSITION_1)
        elif uid in TRIALS_POSITION_2:
            if args.position >= 0 and args.position != 2: return False
            bottle.set_pos(POSITION_2)
        else: return False

        if args.debug and not args.trial and uid != 235: return False

        print(f"Loaded episode {path} with {len(vel_cmd)} steps", end = ' ')

        bottle.set_quat(can_quat)

        scene.step()

        return True

    if not setup(): continue

    import cv2

    SIM_DT  = 0.01
    DATA_HZ = 30        # approx -- timestamps weren't saved; user_232 was ~31.6 Hz (919 frames / 29.06 s)
    steps_per_cmd = max(1, round((1.0 / DATA_HZ) / SIM_DT))   # ~3

    QUIT = False
    step = reward = 0
    stride = 2 if args.debug else 1
    for cmd_idx in range(0, len(vel_cmd), stride):
        if QUIT:
            break

        arm_cmd = vel_cmd[cmd_idx]                       # 6 joint angles (rad) -- POSITION targets
        kinova.control_dofs_position(arm_cmd, dofs_idx_local=kdofs_idx[:len(arm_cmd)])

        # gripper: recorded motor position -> 4 finger joint targets via URDF mimic relations.
        # invert=True: confirmed visually that the driver-joint sign runs opposite to motor pos.
        gripper_cmd = gripper_targets(gripper_pos[cmd_idx][0], invert=True)
        kinova.control_dofs_position(gripper_cmd, dofs_idx_local=np.array(kdofs_idx[-4:]))

        # Hold each recorded waypoint for several physics steps so the PD controller can converge
        # (data ~30 Hz vs sim 100 Hz). Stepping once per waypoint plays ~3x too fast to track.
        for _ in range(steps_per_cmd):
            scene.step()

        if step % reward_check_period == 0:
            # bottles need to be in contact
            if bottle.get_contacts(goal_bottle)['position'].shape[0]:
                # make sure the gripper is to the side of the bottle
                eef_pos = kinova.get_link(kinova_eef_name).get_pos()
                if eef_pos[0] < bottle.get_pos()[0]:
                    reward = 1.
                    total_reward += reward
                    print(f"~~~~~Success at step {step}~~~~~~")
                    break
        step += 1
    if reward == 0:
        print(f"~~~~~Failed~~~~~~")
print(f"Total reward: {total_reward} out of possible {len(paths)}")
        # time.sleep(0.2)
        # from IPython import embed; embed(); exit()