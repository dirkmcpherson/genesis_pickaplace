import argparse
import genesis as gs
import time

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--vis", action="store_true", default=False)
parser.add_argument("-d", "--debug", action="store_true", default=False)
parser.add_argument("-p", "--position", type=int, default=-1)
args = parser.parse_args()

assert args.position in [-1, 0, 1, 2], "Position must be -1, 0, 1, or 2"

gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")


scene = gs.Scene(
    show_viewer=args.vis,
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

BOTTLE_RADIUS = 0.035
BOTTLE_HEIGHT = 0.075
BOX_WIDTH, BOX_HEIGHT = 0.75, 0.12

box = scene.add_entity(
    material=gs.materials.Rigid(rho=1000,
                                friction=0.5),
    morph=gs.morphs.Box(
        size=(0.4, BOX_WIDTH, BOX_HEIGHT),
        pos=(0.75, -BOX_WIDTH / 4, 0.05),
    ),
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
        file='/home/j/workspace/genesis_pickaplace/gen3_lite_2f_robotiq_85.urdf',
        fixed=True,
        pos=(0.0, 0.0, 0.05), # raise to account for table mount
    ),

    # gs.morphs.MJCF(file="/home/j/workspace/genesis_pickaplace/005_tomato_soup_can/google_512k/kinbody.xml"),
)

STATIC_BOTTLE_POSITION = (0.6, -0.2, 0.19)
POSITION_0 = (0.4381, 0.1, 0.05)
POSITION_1 = (0.4381, -0.05, 0.05)
POSITION_2 = (0.4381, -0.2, 0.05)



bottle = scene.add_entity(
    material=gs.materials.Rigid(rho=2000,
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
print(f"Control gains for kinova:")
print(kinova.get_dofs_kp(dofs_idx_local=kdofs_idx))
print(kinova.get_dofs_kv(dofs_idx_local=kdofs_idx))
print(kinova.get_dofs_force_range(dofs_idx_local=kdofs_idx))
# set positional gains
# franka.set_dofs_kp(
#     kp             = np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
#     dofs_idx_local = kdofs_idx,
# )
# # set velocity gains
# franka.set_dofs_kv(
#     kv             = np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
#     dofs_idx_local = kdofs_idx,
# )
# # set force range for safety
# franka.set_dofs_force_range(
#     lower          = np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
#     upper          = np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
#     dofs_idx_local = kdofs_idx,
# )

print(f"Hard setting kinova joint positions")
harcoded_start = [0.3268500269015339, -1.4471734542578538, 2.3453266624159497, -1.3502152158191212, 2.209384006676201, -1.5125125137062945, -1, 1, -0.5 ,0.5]
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


total_reward = 0

# read the trial data
dir = pl.Path('./inthewild_trials/')
for path in dir.iterdir():
    path = str(path)
    if not path.endswith('episodes.npy'): continue

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
        if uid in TRIALS_POSITION_0: 
            if args.position >= 0 and args.position != 0: return False
            bottle.set_pos(POSITION_0)
        elif uid in TRIALS_POSITION_1:
            if args.position >= 0 and args.position != 1: return False
            bottle.set_pos(POSITION_1)
        elif uid in TRIALS_POSITION_2:
            if args.position >= 0 and args.position != 2: return False
            bottle.set_pos(POSITION_2)
        else: return False

        if args.debug and uid != 235: return False

        print(f"Loaded episode {path} with {len(vel_cmd)} steps", end = ' ')

        bottle.set_quat([1, 0, 0, 0])

        scene.step()

        return True

    if not setup(): continue

    import cv2

    QUIT = False
    step = cmd_idx = reward = 0
    while cmd_idx < len(vel_cmd) and not QUIT:
        # QUIT = not listener.running
        if cmd_idx >= len(vel_cmd):
            cmd = np.zeros(6)
            cmd_idx = 0
            setup()
        else:
            cmd = vel_cmd[cmd_idx]
            cmd_idx += 1 if not args.debug else 2
        # cmd[0] = +1.0

        # kinova.control_dofs_velocity(cmd, dofs_idx_local=kdofs_idx[:len(cmd)])

        # gripper_cmd = np.repeat([gripper_cmds[cmd_idx]], 2)
        # print(f"Gripper command: {gripper_cmd}, {gripper_idx}")

        # print the distance from the current position to the commanded position

        kinova.control_dofs_position(cmd, dofs_idx_local=kdofs_idx[:len(cmd)])
        if cmd_idx < len(gripper_pos):
            # map from 0, 100 to -1, 0 for the left finger and 0, 100 to 0, 1 for the right finger
            motor_cmd = (100 - gripper_pos[cmd_idx][0]) / 100
            gripper_cmd = [-motor_cmd, motor_cmd, -0.5, -0.5]
            kinova.control_dofs_position(gripper_cmd, dofs_idx_local=np.array(kdofs_idx[-4:]))
        # print('motor ', ', '.join([f'{c:+1.2f}' for c in cmd]), end=' ')
        # print('gripper ', ', '.join([f'{c:+1.2f}' for c in gripper_cmd]))



        # print the position of the kinova end-effector
        # print(f"End effector position: {pos}")

        # print(goal_bottle.get_contacts())
        scene.step()

        # cv2.waitKey(0)

        pos = kinova.get_dofs_position(dofs_idx_local=kdofs_idx)
        total_diff = sum([abs(jp - cjp) for jp, cjp in zip(pos, cmd)])

        # if total_diff > 0.05: 
        #     print(f"Failed to reach commanded position at step {step} with diff {total_diff}. repeating command")
        #     cmd_idx -= 1

        if step % reward_check_period == 0:
            # bottles need to be in contact
            if bottle.get_contacts(goal_bottle)['position'].shape[0]:
                # this is the simplest but we may want to check that the bottle isn't in the grip of the gripper
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
print(f"Total reward: {total_reward} out of possible {len(dir.iterdir())}")
        # time.sleep(0.2)
        # from IPython import embed; embed(); exit()