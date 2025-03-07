import genesis as gs

gs.init(backend=gs.gpu)

scene = gs.Scene()

plane = scene.add_entity(
    gs.morphs.Plane(),
)

box = scene.add_entity(
    material=gs.materials.Rigid(rho=1000),
    morph=gs.morphs.Box(
        pos=(0.825, 0.0, 0.05),
        size=(0.5, 0.5, 0.05),
    ),
)

# horizontal_scale = 0.25
# vertical_scale = 0.005
# height_field = np.zeros([40, 40])
# heights_range = np.arange(-10, 20, 10)
# height_field[5:35, 5:35] = 200 + np.random.choice(heights_range, (30, 30))
# ########################## entities ##########################
# terrain = scene.add_entity(
#     morph=gs.morphs.Terrain(
#         horizontal_scale=horizontal_scale,
#         vertical_scale=vertical_scale,
#         height_field=height_field,
#         # name="example",
#         # from_stored=True,
#     ),
# )

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

# shelf = scene.add_entity(
#     gs.morphs.URDF(
#         file='/home/j/workspace/genesis_pickaplace/shelf.urdf',
#         fixed=True,
#         pos=(0.7, 0.2125, -0.05),
#         euler=(90, 0, 0),
#         scale=0.85,
#     ),
# )

POSITION_0 = (0.4381, 0.0, 0.1)

bottle = scene.add_entity(
    material=gs.materials.Rigid(rho=300),
    morph=gs.morphs.Cylinder(
        pos=POSITION_0,
        radius=0.05,
        height=0.1,
    ),
    visualize_contact=True,
)

# bottle = scene.add_entity(
#     material=gs.materials.Rigid(rho=300),
#     morph=gs.morphs.URDF(
#         file="./cylinder.urdf",
#         scale=0.5,
#         pos=POSITION_0,
#         euler=(0, 0, 0),
#     ),
#     # morph=gs.morphs.URDF(
#     #     file="urdf/3763/mobility_vhacd.urdf",
#     #     scale=0.09,
#     #     pos=POSITION_0,
#     #     euler=(0, 0, 0),
#     # ),
#     # visualize_contact=True,
# )


from kinova import JOINT_NAMES as kinova_joint_names, EEF_NAME as kinova_eef_name
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


# read the trial data
path = './inthewild_trials/305_episodes.npy'
ep_dict = np.load(path, allow_pickle=True).item()
vel_cmd = ep_dict['vel_cmd']
gripper_pos = ep_dict['gripper_pos']
cmd_idx = 0
print(f"Loaded episode {path} with {len(vel_cmd)} steps")

from pynput import keyboard
import time

# def on_press(key):
#     if key == keyboard.Key.esc:
#         print("Escape key pressed. Exiting...")
#         return False  # Stop the listener
#     # Handle other keys if needed

# # Start the listener in a non-blocking way
# listener = keyboard.Listener(on_press=on_press)
# listener.start()

gripper_cmds = np.linspace(-1.0, 1.0, len(vel_cmd) + 1)

gripper_open = np.array([-1.0, 1.0])
gripper_closed = np.array([0.0, 0.0])


QUIT = False
for _ in range(int(2*len(vel_cmd))):
    # QUIT = not listener.running
    if cmd_idx >= len(vel_cmd):
        cmd = np.zeros(6)
        cmd_idx = 0
        bottle.set_pos(POSITION_0)
        bottle.set_quat([1, 0, 0, 0])
    else:
        cmd = vel_cmd[cmd_idx]
        cmd_idx += 1
    # cmd[0] = +1.0

    # kinova.control_dofs_velocity(cmd, dofs_idx_local=kdofs_idx[:len(cmd)])

    # gripper_cmd = np.repeat([gripper_cmds[cmd_idx]], 2)
    # print(f"Gripper command: {gripper_cmd}, {gripper_idx}")


    kinova.control_dofs_position(cmd, dofs_idx_local=kdofs_idx[:len(cmd)])
    if cmd_idx < len(gripper_pos):
        # map from 0, 100 to -1, 0 for the left finger and 0, 100 to 0, 1 for the right finger
        motor_cmd = (100 - gripper_pos[cmd_idx][0]) / 100
        gripper_cmd = [-motor_cmd, motor_cmd, -0.5, -0.5]
        kinova.control_dofs_position(gripper_cmd, dofs_idx_local=np.array(kdofs_idx[-4:]))
    # print('motor ', ', '.join([f'{c:+1.2f}' for c in cmd]), end=' ')
    # print('gripper ', ', '.join([f'{c:+1.2f}' for c in gripper_cmd]))


    # print the position of the kinova end-effector
    pos = kinova.get_link(kinova_eef_name).get_pos()
    # print(f"End effector position: {pos}")
    # bottle.set_pos(pos)

    scene.step()

    # time.sleep(1.)
    # from IPython import embed; embed(); exit()
try:
    pass
except KeyboardInterrupt:
     print('interrupted!')
except Exception as e:
    print(e)
