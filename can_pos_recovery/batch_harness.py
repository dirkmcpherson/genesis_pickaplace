"""Batched-GPU replay: evaluate B candidate placements of ONE demo simultaneously.
The replay is open-loop (recorded joint commands don't depend on the can), so every env
gets the identical command stream; only the can / goal-can initial poses differ per env.

Success in-batch uses a distance proxy for contact (two upright cylinders touch iff
horizontal center distance <= 2R and vertical overlap), plus example.py's eef-behind-can
test at the same 30-cmd cadence. Winners must be re-confirmed with the exact single-env
contact test (replay_harness.rollout) before being reported.
"""
import os
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import sys, pathlib as pl
import numpy as np
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO))
import genesis as gs
import torch
from kinova import JOINT_NAMES, EEF_NAME
from replay_harness import (BOTTLE_RADIUS, BOTTLE_HEIGHT, BOX_POS, BOX_SIZE, BOX_TOP_Z,
                            STATIC_BOTTLE_POSITION, FAR_AWAY, PICK_Z, GP_CLOSE,
                            HARDCODED_START, TABLE_TOP_Z, gripper_targets, load_episode)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)

CONTACT_D = 2 * BOTTLE_RADIUS + 0.002   # horizontal center distance that counts as touching


def build_batched_world(n_envs, finger_force=50.0, finger_kp=100.0,
                        can_height=0.10, can_rho=1000, substeps=4,
                        table=False, can_radius=BOTTLE_RADIUS):
    gs.init(backend=gs.gpu, seed=0, precision="32", logging_level="warning")
    scene = gs.Scene(show_viewer=False,
                     sim_options=gs.options.SimOptions(dt=0.01, substeps=substeps))
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(material=gs.materials.Rigid(rho=1000, friction=0.5),
                     morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_POS))
    if table:
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
    from replay_harness import joint_dofs
    kdofs = [joint_dofs(kinova.get_joint(n)) for n in JOINT_NAMES]
    eef = kinova.get_link(EEF_NAME)
    scene.build(n_envs=n_envs)

    fkp = [finger_kp] * 4; fkv = [10] * 4
    kinova.set_dofs_kp(kp=np.array([200, 200, 150, 100, 60, 60] + fkp), dofs_idx_local=kdofs)
    kinova.set_dofs_kv(kv=np.array([20, 20, 15, 10, 6, 6] + fkv), dofs_idx_local=kdofs)
    ff = [finger_force] * 4
    kinova.set_dofs_force_range(
        lower=np.array([-50, -50, -50, -20, -20, -20] + [-f for f in ff]),
        upper=np.array([50, 50, 50, 20, 20, 20] + ff), dofs_idx_local=kdofs)
    floor_z = TABLE_TOP_Z if table else 0.0
    return dict(scene=scene, kinova=kinova, bottle=bottle, goal=goal, kdofs=kdofs, eef=eef,
                B=n_envs, can_h=can_height, can_start_z=floor_z + can_height / 2 + 0.0125,
                goal_start_z=BOX_TOP_Z + can_height / 2 + 0.0425,
                pick_z=floor_z + can_height / 2 + 0.05, can_r=can_radius)


def rollout_batch(w, vel, gp, can_xy, goal_xy, max_cmds=None, can_quat=None, can_z=None):
    """can_xy: (B,2) array of can starts; goal_xy: (B,2) of goal starts (NaN row -> parked far).
    can_quat (B,4) / can_z (B,): optional per-env can orientation + start height --
    lying-can demos (fallen can righted by the human) need a sideways spawn.
    Returns per-env dict of checkpoint arrays."""
    scene, kinova, bottle, goal, kdofs, eef, B = (w['scene'], w['kinova'], w['bottle'],
                                                  w['goal'], w['kdofs'], w['eef'], w['B'])
    can_xy = np.asarray(can_xy, dtype=np.float64); goal_xy = np.asarray(goal_xy, dtype=np.float64)
    assert can_xy.shape == (B, 2) and goal_xy.shape == (B, 2)

    kinova.set_dofs_position(np.tile(np.array(HARDCODED_START), (B, 1)), kdofs)
    kinova.zero_all_dofs_velocity()
    zcol = (np.asarray(can_z, dtype=np.float64).reshape(B, 1) if can_z is not None
            else np.full((B, 1), w['can_start_z']))
    cp = np.concatenate([can_xy, zcol], axis=1)
    gp_pos = np.concatenate([goal_xy, np.full((B, 1), w['goal_start_z'])], axis=1)
    far = np.isnan(goal_xy).any(axis=1)
    gp_pos[far] = FAR_AWAY
    quats = (np.asarray(can_quat, dtype=np.float64) if can_quat is not None
             else np.tile([1., 0., 0., 0.], (B, 1)))
    bottle.set_pos(cp); bottle.set_quat(quats)
    goal.set_pos(gp_pos); goal.set_quat(np.tile([1., 0., 0., 0.], (B, 1)))
    for ent in (bottle, goal):
        try: ent.zero_all_dofs_velocity()
        except Exception: pass
    scene.step()

    n = len(vel) if max_cmds is None else min(max_cmds, len(vel))
    picked_at = np.full(B, -1); placed_at = np.full(B, -1); success_at = np.full(B, -1)
    max_z = np.zeros(B)
    xlo, xhi = BOX_POS[0] - BOX_SIZE[0]/2, BOX_POS[0] + BOX_SIZE[0]/2
    ylo, yhi = BOX_POS[1] - BOX_SIZE[1]/2, BOX_POS[1] + BOX_SIZE[1]/2
    for i in range(n):
        kinova.control_dofs_position(np.tile(vel[i], (B, 1)), dofs_idx_local=kdofs[:6])
        kinova.control_dofs_position(np.tile(np.array(gripper_targets(gp[i])), (B, 1)),
                                     dofs_idx_local=np.array(kdofs[-4:]))
        for _ in range(3):
            scene.step()
        bp = np_(bottle.get_pos()).reshape(B, 3)
        max_z = np.maximum(max_z, bp[:, 2])
        newly_picked = (picked_at < 0) & (bp[:, 2] > w.get('pick_z', PICK_Z)) & (gp[i] > GP_CLOSE)
        picked_at[newly_picked] = i
        in_fp = (xlo < bp[:, 0]) & (bp[:, 0] < xhi) & (ylo < bp[:, 1]) & (bp[:, 1] < yhi)
        newly_placed = ((placed_at < 0) & (picked_at >= 0) & (gp[i] < GP_CLOSE) & in_fp &
                        (bp[:, 2] > BOX_TOP_Z + 0.01) & (bp[:, 2] < BOX_TOP_Z + 0.07))
        placed_at[newly_placed] = i
        if i % 30 == 0:
            gpp = np_(goal.get_pos()).reshape(B, 3)
            ep = np_(eef.get_pos()).reshape(B, 3)
            horiz = np.hypot(bp[:, 0] - gpp[:, 0], bp[:, 1] - gpp[:, 1])
            vert = np.abs(bp[:, 2] - gpp[:, 2]) < w['can_h']
            touch = (horiz < 2 * w.get('can_r', BOTTLE_RADIUS) + 0.002) & vert & ~far
            # require picked first (match replay_harness CPU test) -- exclude table-shoves
            newly_succ = (success_at < 0) & touch & (ep[:, 0] < bp[:, 0]) & (picked_at >= 0)
            success_at[newly_succ] = i
    for _ in range(100):   # settle, then score NESTED (upright + touching + at rest)
        scene.step()
    final = np_(bottle.get_pos()).reshape(B, 3)
    gfin = np_(goal.get_pos()).reshape(B, 3)
    bq = np_(bottle.get_quat()).reshape(B, 4); gq = np_(goal.get_quat()).reshape(B, 4)
    def tilts(q):
        w_, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        zz = 1 - 2 * (x * x + y * y); zx = 2 * (x * z + w_ * y); zy = 2 * (y * z - w_ * x)
        return np.degrees(np.arccos(np.clip(zz / (np.sqrt(zx**2 + zy**2 + zz**2) + 1e-9), -1, 1)))
    try:
        spd = np.linalg.norm(np_(bottle.get_dofs_velocity()).reshape(B, -1)[:, :3], axis=1)
    except Exception:
        spd = np.zeros(B)
    horiz = np.hypot(final[:, 0] - gfin[:, 0], final[:, 1] - gfin[:, 1])
    nested = ((horiz < 2 * w.get('can_r', BOTTLE_RADIUS) + 0.004) &
              (np.abs(final[:, 2] - gfin[:, 2]) < w['can_h'] * 0.5) &
              (tilts(bq) < 20) & (tilts(gq) < 20) & (spd < 0.05) & ~far)
    on_shelf = ((xlo < final[:, 0]) & (final[:, 0] < xhi) &
                (ylo < final[:, 1]) & (final[:, 1] < yhi) & (final[:, 2] > BOX_TOP_Z))
    return dict(picked_at=picked_at, placed_at=placed_at, success_at=success_at,
                max_z=max_z.round(4), final=final.round(4), on_shelf_end=on_shelf,
                nested=nested, n_cmds=n)


if __name__ == '__main__':
    # parity smoke test: known-good placement (243) + known no-pick, in one batch
    import time, json
    B = 8
    w = build_batched_world(B)
    vel, gp = load_episode(243)
    seed = (0.442, -0.037)
    can = np.array([[seed[0] + dx, seed[1] + dy]
                    for dx, dy in [(0,0), (-0.01,0), (0.01,0), (0,-0.01), (0,0.01),
                                   (-0.02,0), (0.02,0), (0.1,0.1)]])   # last = obvious miss
    goal = np.tile([STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1]], (B, 1))
    t0 = time.time()
    r = rollout_batch(w, vel, gp, can, goal)
    dt = time.time() - t0
    for b in range(B):
        print(f"env{b} can=({can[b][0]:+.3f},{can[b][1]:+.3f}) picked@{r['picked_at'][b]} "
              f"placed@{r['placed_at'][b]} success@{r['success_at'][b]} max_z={r['max_z'][b]} "
              f"final={r['final'][b].tolist()}")
    print(f"batched rollout: {dt:.1f}s for {r['n_cmds']} cmds x {B} envs "
          f"({3*r['n_cmds']*B/dt:.0f} env-steps/s)")
