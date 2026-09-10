"""Simulator variants for the real2sim lab (paper/real2sim_follower_lab_2026-08-23.md §sim).

The MDP (obs/action semantics incl. cap+leash, horizon, reward/terminal, ICs, pick_z) is
FIXED. A "sim variant" changes only how the Genesis world REALIZES a commanded joint target:
arm PD gains, gravity compensation of the arm entity, arm joint effort (force) limits.
Everything is applied WITHOUT editing the tracked world builder:

  * before the world is built: `install(name)` monkeypatches gs.Scene.add_entity so that the
    Kinova URDF entity gets a gs.materials.Rigid(gravity_compensation=g) material (the builder
    passes none -> Genesis default, gravity_compensation=0);
  * after the world is built: `post_build(env_or_world, name)` sets the ARM dofs' kp/kv and
    force ranges (finger dofs untouched).

Equivalent permanent patch for the cluster (replay_harness.build_world): add kwargs
`arm_kp=None, arm_kv=None, arm_gravity_comp=0.0, arm_effort=None`, pass
`material=gs.materials.Rigid(gravity_compensation=arm_gravity_comp)` to the Kinova
add_entity, and replace the hard-coded set_dofs_kp/kv arrays with the kwargs when given.
world_cfg (can_pos_recovery/trial_placements.json 'world') would carry the same keys.

Baseline values (replay_harness.build_world lines 171-179): kp [200,200,150,100,60,60],
kv [20,20,15,10,6,6], no gravity comp, arm force range +-[50,50,50,20,20,20] N*m (the builder
OVERRIDES the URDF efforts [10,14,10,7,7,7]; Kinova's own soft torque limit is 9 N*m/joint,
user guide Table 28). Joint speed limit of the real arm: 1.0 rad/s (joints 1-5), 1.57 (j6).
"""
import numpy as np

BASE_KP = [200, 200, 150, 100, 60, 60]
BASE_KV = [20, 20, 15, 10, 6, 6]
BASE_EFFORT = [50, 50, 50, 20, 20, 20]     # what build_world sets (NOT the URDF)
URDF_EFFORT = [10, 14, 10, 7, 7, 7]        # kortex_description gen3 lite

VARIANTS = {
    # name: dict(kp_mult, kv_mult, gravity_comp, effort: 'base' | 'urdf')
    'base':         dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=0.0, effort='base'),
    'gc':           dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=1.0, effort='base'),
    'kp2':          dict(kp_mult=2.0, kv_mult=1.41, gravity_comp=0.0, effort='base'),
    'kp4':          dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=0.0, effort='base'),
    'gc_kp2':       dict(kp_mult=2.0, kv_mult=1.41, gravity_comp=1.0, effort='base'),
    'gc_kp4':       dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base'),
    'gc_kp8':       dict(kp_mult=8.0, kv_mult=2.83, gravity_comp=1.0, effort='base'),
    'gc_urdf':      dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=1.0, effort='urdf'),
    'gc_kp4_urdf':  dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='urdf'),
    'kp4_urdf':     dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=0.0, effort='urdf'),
    # kv fixed at base (higher bandwidth: v_ss = (kp e - tau_g)/kv)
    'gc_kp4_kv1':   dict(kp_mult=4.0, kv_mult=1.0, gravity_comp=1.0, effort='base'),
    # robot mounting height: real tool z reaches -0.019..+0.013 (base frame) in the pressing demos
    # while the sim fingertips cannot go below tool z ~ +0.03 (table top == base plane + finger
    # thickness) -> the real work surface is >= 3 cm below the robot's mounting plane. riser = m
    # added to the URDF mount height (table/can/pick_z/ICs untouched).
    'riser3':       dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=0.0, effort='base', riser=0.03),
    'gc_kp4_riser3': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03),
    'gc_kp4_riser2': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.02),
    'gc_kp4_riser4': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.04),
    # gripper variants (full-task validation, §9): finger driver torque cap / remapped finger angle.
    # finger_force: +-N*m on all 4 finger dofs (world_cfg default 50). finger_map: multiply the
    # closing fraction (real reading 0..1) before the URDF-linear map -> the REAL stall reading
    # 0.83 lands on the sim's 66 mm-can stall angle (0.61): 0.61/0.83 = 0.735.
    'gc_kp4_riser3_fcap2': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03, finger_force=2.0),
    'gc_kp4_riser3_fmap':  dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03, finger_map=0.735),
    'fcap2':               dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=0.0, effort='base', finger_force=2.0),
    # shelf geometry (full-task validation, §9): the 12 cm shelf box is half-buried in the current
    # world (centre z 0.05 -> top 0.11 = only 6 cm above the table top 0.05). shelf_dz raises the
    # box (made fixed) so it STANDS ON THE TABLE: top 0.17 world. Real-data evidence: release/slide
    # tool z 0.17-0.21 (base frame) vs grasp tool z 0.02-0.04 -> the human's can bottom at release
    # is >= 0.09 base-frame, i.e. >= 12 cm above the real table. Predicate constants that reference
    # the shelf (BOX_TOP_Z, SHELF_REST_Z, goal_start_z, the placed z-band) MUST move with it.
    'gc_kp4_riser3_shelf6': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03, shelf_dz=0.06),
    # DECLARED world of record on 2026-08-26 (commit 7c8195d) but NEVER RUN as one: every frozen
    # w3 set, harvest and reported arm was built on gc_kp4_riser3_shelf6 WITHOUT this fix (dropped
    # by omission; only matched_w4_pilot/dH + dp_pilotw4 used it) -- paper/TS5_DROPPED_FIX_2026-09-02.md.
    # shelf6 + the CONTACT-SOLVER fix.
    # Every geom is built with MuJoCo's default sol_params timeconst 0.02 s = 8x this engine's own
    # stability floor (2*substep_dt = 0.0025 s at substeps 8); penetration scales as timeconst^2,
    # which is where 10 mm of gripper-into-can clipping came from. Setting the FINGER geoms AND the
    # CAN to 0.005 s (Genesis averages a contact pair's params, so both sides must move) gives:
    # carry penetration 9.9 -> 1.5 mm, free tip-overs 20 -> 12, nested 17 -> 20, pick recreation
    # 57 -> 58/66 (cluster-confirmed), arm-tracking fidelity unchanged, random-teacher control 0/30.
    # Cost, stated: grip force rises 147 -> 245 N (fingers stall at the surface holding full PD
    # error). paper/gripper_lab_2026-08-25.md.
    'gc_kp4_riser3_shelf6_ts5': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                     riser=0.03, shelf_dz=0.06, grasp_timeconst=0.005),
    # ts5 + the CEM-searched physics (paper/WORLD_SEARCH_PREREG_2026-09-02.md, A2.4/A2.5): a
    # `world` block overrides what replay_harness.build_world would otherwise take from
    # trial_placements.json's world block / its kwarg defaults. Realized WITHOUT editing the builder:
    # install() swaps the material handed to add_entity (can: rho+friction; goal / shelf / table:
    # friction), post_build() sets the four finger dofs' kp and force range. Identical to
    # world_search.build_theta_world(theta, 'gc_kp4_riser3_shelf6_ts5') by construction
    # (selftest: `python baselines/sim_variants.py --selftest gc_kp4_riser3_shelf6_ts5_w4`).
    # Search facts (A2.4/A2.5): the lever is finger_kp (>= 55 vs theta0 40); finger_force and the
    # table/goal frictions were inert across the population; can_friction is inert by construction
    # (pair rule = max, gripper_lab §1.3; ablation F: theta0's 0.2 is bit-identical to the champion's
    # 0.068 -> 0.2 kept); can_rho is NOT inert (ablation R: 1010 = the 0.35 kg can costs -12 score,
    # -4 nested on the fit set) -> the champion's 852.5 (a 0.295 kg can) is kept VERBATIM and
    # flagged: weigh the real can before this becomes a world of record.
    'gc_kp4_riser3_shelf6_ts5_w4': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                        riser=0.03, shelf_dz=0.06, grasp_timeconst=0.005,
                                        n_grasp_geoms=5,   # 4 finger collision geoms + 1 can geom
                                        world=dict(finger_kp=64.3438, finger_force=59.2737,
                                                   can_rho=852.5468, can_friction=0.2,
                                                   table_friction=0.4434, goal_friction=2.1183)),
    'shelf6':               dict(kp_mult=1.0, kv_mult=1.0, gravity_comp=0.0, effort='base', shelf_dz=0.06),
    # data-fit alternative: real release tool z p50 0.177 (base) - grasp offset (0.020 + 0.03) = can
    # bottom 0.127 base-frame at release -> shelf top ~0.13 base = 0.21 world (= BOX_TOP_Z + 0.10)
    'gc_kp4_riser3_shelf10': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03, shelf_dz=0.10),
    # 2026-09-03 (sim box, paper/SHELF_HEIGHT_PREREG_2026-09-03.md): the shelf height measured
    # DIFFERENTIALLY from the bags -- tool z at the release minus tool z at the pick closure (same
    # grasp, so the grasp offset and the tool-frame offset cancel), 43 demos with a full release:
    # median 15.8 cm above the table (IQR 14.3-17.0) vs shelf6's 12.0 cm and shelf10's 16.0 cm.
    # In the shelf6 worlds the recorder path releases the can 1.6 cm (w3, after 2.5 cm of in-grasp
    # slip) / 3.5 cm (ts5, slip 0.9 cm) above the shelf -> 'falls from height' is the largest tip
    # class (11/32, 10/26, 13/22 tipped tapes in w3 / ts5 / ts5_w4). shelf10 + ts5 = release ON the
    # shelf with the stiff (non-slipping) grasp. New names only; nothing above is edited.
    'gc_kp4_riser3_shelf10_ts5': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                      riser=0.03, shelf_dz=0.10, grasp_timeconst=0.005, n_grasp_geoms=5),
    'gc_kp4_riser3_shelf8_ts5': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                     riser=0.03, shelf_dz=0.08, grasp_timeconst=0.005, n_grasp_geoms=5),
    # 2026-09-03 (sim box, release lab `can_pos_recovery/release_lab.py`): 20/20 w3 tips re-executed
    # at the release rotate 80-90 deg from the pinch axis while the pads still touch (0.2-8 N); freed
    # in one step (`open_now`) 17/20 land upright; friction level / knuckle masks / ts5 rescue <= 6/20.
    # First hypothesis: a two-point pinch is a free hinge (no torsional friction, one contact per
    # pad at a light pinch) -> can_segments=n: install() swaps BOTH cans' Cylinder morph for a
    # single-link URDF whose collision is n stacked cylinders (same radius, h/n each, flush seams;
    # visual = one full cylinder; explicit <inertial> = the primitive's mass/inertia for the
    # material's rho, friction re-applied in post_build -- the URDF loader ignores both), so a ~2 cm
    # pad always spans a seam (>= 2 contacts per pad). WRONG LEVER: the pinch lab shows the droop is
    # a CREEP of the regularised friction constraint (below), the same with 1, 3 or 6 segments.
    # CREEP FIX candidates (paper/CREEP_PREREG_2026-09-03.md). The engine's contact constraints are
    # regularised by (1 - d) / d with the default impedance d = 0.9..0.95 (sol_params dmin/dmax, =
    # MuJoCo solimp): a pinched can carrying a sub-limit tangential load CREEPS (rotates / slides) at
    # a rate ~ (1 - d) / d x timeconst instead of holding -- the in-grasp slip (2.5 cm over a carry in
    # w3, 0.9 in ts5) and the in-hand / partial-release tips are this. Pinch lab (can_pos_recovery/
    # pinch_lab.py, fixed pads, 0.1 N m gravity torque): droop/s 59 (default) -> 8.5 (d 0.99) -> 0.8
    # (0.999) at timeconst 0.02; the contact also gets stiffer at fixed penetration (x5.6 at 0.99,
    # x55 at 0.999 -- the latter is out; ts5 was x11 and cost grip force). MuJoCo practice for
    # "objects creep on slopes" is exactly solimp d -> 0.99. imp99 = d on EVERY geom (every pair);
    # gimp99 = finger + can geoms only (the mechanism control: if it matches imp99 the slide is
    # impedance-insensitive); ts5_imp99 = the bracket (stiffer + harder).
    'gc_kp4_riser3_shelf6_imp99': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                       riser=0.03, shelf_dz=0.06, impedance=0.99, impedance_scope='all'),
    'gc_kp4_riser3_shelf6_gimp99': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                        riser=0.03, shelf_dz=0.06, impedance=0.99, impedance_scope='grasp',
                                        n_grasp_geoms=5),
    'gc_kp4_riser3_shelf6_ts5_imp99': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                           riser=0.03, shelf_dz=0.06, grasp_timeconst=0.005, n_grasp_geoms=5,
                                           impedance=0.99, impedance_scope='all'),
    # fast-open release (2026-09-03 night, paper/RELEASE_OPEN_PREREG_2026-09-03.md): the impedance
    # census falsified the creep fix (all 3 rows NOT ADOPTED) and the release lab's only working
    # condition is `open_now` (snap the fingers open at the release: 17/20 w3 tips land upright vs
    # 0/20 control; every physics condition <= 10/20; `unload` rule 0/20). Bag check: the bags carry
    # NO independent gripper command topic (g_frame == fb_grip, corr 1.0000), the human's release
    # really is a ~1.7 s ramp and the tape carries it faithfully -- so this is NOT a fidelity fix;
    # it is a per-phase customisation of the recorder (user mandate 09-03 12:40: per-phase worlds /
    # dataset-level phase customisation allowed), disclosed as mechanism-unfaithful, outcome-faithful.
    # `grip_open_gain` is a RECORDER key, not a world key: consumed by record_demos.HumanFollower
    # (causal filter on the tape's grip stream -- drops from the running hold plateau are amplified
    # K-fold (og4) or snapped to full open (ognow, K=inf); closing and holds pass through untouched,
    # regrasps reset the plateau). The WORLD these variants build is bit-identical to
    # gc_kp4_riser3_shelf6 (selftest asserts it); install()/post_build() ignore the key.
    'gc_kp4_riser3_shelf6_ognow': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                       riser=0.03, shelf_dz=0.06, grip_open_gain=float('inf')),
    'gc_kp4_riser3_shelf6_og4': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                     riser=0.03, shelf_dz=0.06, grip_open_gain=4.0),
    # ARM YAW (2026-09-08, paper/EARLY_TRIALS_ROTATION_2026-09-08.md): the 120 never-ingested
    # trials (uids 110-231, recorded 12-15..12-17) were driven with the arm at a different yaw
    # relative to the table/shelf/goal than the 12-18 session every frozen set comes from. Measured
    # from the tapes alone: rotate each day about the robot base so its release cluster (the STATIC
    # goal can) lands on 12-18's, then check the pick cloud, which was NOT used in the fit --
    # 12-16 phi -9.7 deg (goal residual 1.9 cm, pick median 2.8 cm), 12-17 phi -19.2 deg (0.2 cm,
    # 6.3 cm). The scene is authored in base-frame coordinates, so turning the MOUNT by `yaw`
    # (degrees about +z) realizes "the arm is turned relative to the scene". Nothing else moves.
    # These are per-DAY worlds for re-recording those trials; they are NOT candidates for the
    # world of record and no frozen set uses them.
    'gc_kp4_riser3_shelf6_yaw16': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                       riser=0.03, shelf_dz=0.06, yaw=-9.7),    # 2024-12-16 session
    'gc_kp4_riser3_shelf6_yaw17': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                       riser=0.03, shelf_dz=0.06, yaw=-19.2),   # 2024-12-17 session
    # split can (pinch-hinge hypothesis, 2026-09-03 afternoon): NOT the lever -- pinch lab creep with 6
    # segments = 1 segment (13-24 contacts vs 4). Kept as a tested, selftested variant; not proposed.
    'gc_kp4_riser3_shelf6_seg6': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                      riser=0.03, shelf_dz=0.06, can_segments=6),
    'gc_kp4_riser3_shelf6_seg6_ts5': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base',
                                          riser=0.03, shelf_dz=0.06, can_segments=6, grasp_timeconst=0.005,
                                          n_grasp_geoms=10),   # 4 finger geoms + 6 can segments
}
BOX_SIZE = (0.4, 0.75, 0.12); BOX_POS = (0.75, -0.1875, 0.05); BOX_TOP_Z = 0.11
TABLE_SIZE_XY = (0.419, 1.2)                  # replay_harness.build_world(table=True) pick-area table
CAN_COLOR = (0.95, 0.45, 0.10); GOAL_COLOR = (0.10, 0.55, 0.75)   # build_world's surface colors
WORLD_KEYS = ('finger_kp', 'finger_force', 'can_rho', 'can_friction', 'table_friction', 'goal_friction')


def shelf_top(name):
    return BOX_TOP_Z + float(VARIANTS[name].get('shelf_dz', 0.0))


def segmented_can_urdf(radius, height, n, rho):
    """Write (idempotently) and return the absolute path of the split-can URDF: one free link, n stacked
    <cylinder> collision geoms of height h/n (flush seams, same radius), one full-height visual
    cylinder, and an explicit <inertial> equal to what Genesis derives for the primitive can (mass =
    the 32-section polygon cylinder's volume x rho, inertia = trimesh moment_inertia x rho, CoM at the
    origin). Genesis parses <cylinder> with the same trimesh.creation.cylinder call as
    gs.morphs.Cylinder, keeps each convex geom (no merge/decomposition for CYLINDER-typed geoms) and
    has no CYLINDER special case in the collider. Explicit inertial because the loader's own
    inference concatenates the segment meshes, trimesh merges the seam vertices, the result is not
    watertight and the mass silently falls back to 1.0 (caught by --selftest)."""
    import pathlib as pl, trimesh
    n = int(n); r = float(radius); h = float(height); rho = float(rho)
    assert n >= 2 and r > 0 and h > 0 and rho > 0, (n, r, h, rho)
    path = pl.Path(__file__).resolve().parent / 'assets' / f'can_r{r:.4f}_h{h:.4f}_seg{n}_rho{rho:.4f}.urdf'
    tm = trimesh.creation.cylinder(radius=r, height=h)          # == genesis.utils.mesh.create_cylinder
    mass = tm.volume * rho; I = tm.moment_inertia * rho             # trimesh: density 1 -> scale by rho
    assert np.allclose(tm.center_mass, 0, atol=1e-9) and np.allclose(I, np.diag(np.diag(I)), atol=1e-12)
    seg = h / n
    cols = ''.join(f'  <collision name="seg{i}"><origin xyz="0 0 {-h / 2 + (i + 0.5) * seg:.6f}" rpy="0 0 0"/>'
                   f'<geometry><cylinder length="{seg:.6f}" radius="{r:.6f}"/></geometry></collision>\n' for i in range(n))
    txt = (f'<?xml version="1.0"?>\n<!-- generated by sim_variants.segmented_can_urdf(radius={r}, height={h}, n={n}, '
           f'rho={rho}); do not edit -->\n<robot name="can_seg{n}">\n<link name="can">\n'
           f'  <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="{mass:.12g}"/>'
           f'<inertia ixx="{I[0, 0]:.12g}" ixy="0" ixz="0" iyy="{I[1, 1]:.12g}" iyz="0" izz="{I[2, 2]:.12g}"/></inertial>\n'
           f'  <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><cylinder length="{h:.6f}" radius="{r:.6f}"/></geometry></visual>\n'
           f'{cols}</link>\n</robot>\n')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != txt:
        path.write_text(txt)
    return str(path)


def _segmented_morph(gs, morph, n, rho):
    """The gs.morphs.URDF replacing a can's gs.morphs.Cylinder morph (same pose, same fixed flag)."""
    path = segmented_can_urdf(morph.radius, morph.height, n, rho)
    kw = dict(file=path, pos=tuple(float(x) for x in morph.pos), fixed=bool(getattr(morph, 'fixed', False)),
              visualization=bool(morph.visualization), collision=bool(morph.collision))
    if getattr(morph, 'quat', None) is not None:
        kw['quat'] = tuple(float(x) for x in morph.quat)
    return gs.morphs.URDF(**kw)

_installed = {'name': None, 'orig': None, 'subs': {}, 'segs': {}, 'n_cyl': 0}


def _surface_color(surface):
    c = getattr(surface, 'color', None)
    if c is None:
        tex = getattr(surface, 'diffuse_texture', None)
        c = getattr(tex, 'color', None)
    return None if c is None else tuple(float(x) for x in c)


def _classify(gs, morph, surface, is_shelf):
    """Which build_world entity is being added: 'shelf' | 'table' | 'can' | 'goal' | None."""
    if is_shelf:
        return 'shelf'
    if isinstance(morph, gs.morphs.Box) and np.allclose(np.asarray(morph.size, float)[:2], TABLE_SIZE_XY):
        return 'table'
    if isinstance(morph, gs.morphs.Cylinder):
        _installed['n_cyl'] += 1
        c = _surface_color(surface)
        if c is not None and np.allclose(c, CAN_COLOR): return 'can'
        if c is not None and np.allclose(c, GOAL_COLOR): return 'goal'
        # colour-less fallback: build_world adds the can before the goal
        return 'can' if _installed['n_cyl'] == 1 else 'goal'
    return None


def install(name):
    """Monkeypatch gs.Scene.add_entity so the Kinova URDF gets gravity compensation (+ riser),
    the shelf box moves by shelf_dz, and -- for variants with a `world` block -- the can / goal /
    shelf / table materials are replaced by the variant's rho / frictions."""
    v = VARIANTS[name]
    import genesis as gs
    if _installed['orig'] is None:
        _installed['orig'] = gs.Scene.add_entity
    orig = _installed['orig']
    gc = float(v['gravity_comp']); riser = float(v.get('riser', 0.0)); shelf_dz = float(v.get('shelf_dz', 0.0))
    yaw = float(v.get('yaw', 0.0))          # degrees about +z, applied to the robot mount (see add_entity)
    world = v.get('world'); segs = v.get('can_segments')
    _installed['subs'] = {}; _installed['segs'] = {}; _installed['n_cyl'] = 0

    def add_entity(self, morph=None, material=None, surface=None, *a, **kw):
        is_shelf = isinstance(morph, gs.morphs.Box) and \
            np.allclose(np.asarray(morph.size, float), BOX_SIZE) and np.allclose(np.asarray(morph.pos, float), BOX_POS)
        if shelf_dz != 0.0 and is_shelf:
            pos = tuple(float(x) for x in morph.pos)
            try:
                morph.pos = (pos[0], pos[1], pos[2] + shelf_dz); morph.fixed = True
            except Exception:
                object.__setattr__(morph, 'pos', (pos[0], pos[1], pos[2] + shelf_dz)); object.__setattr__(morph, 'fixed', True)
        kind = _classify(gs, morph, surface, is_shelf) if (world is not None or segs is not None) else None
        if world is not None:
            if kind is not None:
                # same constructor calls as replay_harness.build_world, values from the variant
                # (the builder's own material -- from trial_placements' world block / kwarg
                # defaults -- is DROPPED, not merged: every key below is one the builder sets)
                if kind == 'can':
                    material = gs.materials.Rigid(rho=float(world['can_rho']), friction=float(world['can_friction']))
                elif kind == 'goal':
                    material = gs.materials.Rigid(rho=1000, friction=float(world['goal_friction']))
                else:   # shelf / table
                    material = gs.materials.Rigid(rho=1000, friction=float(world['table_friction']))
                _installed['subs'][kind] = _installed['subs'].get(kind, 0) + 1
        if segs is not None and kind in ('can', 'goal'):
            # split can: both cans get the same n-segment URDF (identical objects, as in the rig). The
            # URDF carries the primitive's mass/inertia for THIS material's rho (after any world swap):
            # Genesis would otherwise fall back to mass 1.0 (the concatenated segment mesh is not
            # watertight once trimesh merges the seam vertices). Friction: the URDF loader ignores the
            # material's (hard-coded default 1.0) -> post_build re-applies material.friction.
            assert material is not None and getattr(material, 'rho', None) is not None, (kind, material)
            morph = _segmented_morph(gs, morph, int(segs), float(material.rho))
            _installed['segs'][kind] = _installed['segs'].get(kind, 0) + 1
        if isinstance(morph, gs.morphs.URDF) and 'gen3' in str(getattr(morph, 'file', '')):
            if material is None and gc != 0.0:
                material = gs.materials.Rigid(gravity_compensation=gc)
            if yaw != 0.0:
                # 2026-09-08: the arm sat at a different yaw relative to the table/shelf on the
                # 12-16 and 12-17 recording days (paper/EARLY_TRIALS_ROTATION_2026-09-08.md).
                # The scene (table, shelf, goal) is authored in base-frame coordinates, so rotating
                # the MOUNT by yaw is exactly "the arm is turned by yaw relative to the scene".
                # Set `quat`, NOT `euler`: Genesis morphs derive quat from euler in __init__ and use
                # quat thereafter (genesis/options/morphs.py:69-71, "If specified, `euler` will be
                # ignored"), so assigning euler after construction is silently inert -- the selftest's
                # read-back of the BUILT base orientation caught exactly that.
                import genesis as _gs
                q = tuple(float(x) for x in _gs.utils.geom.xyz_to_quat(
                    np.array([0.0, 0.0, float(yaw)]), rpy=True, degrees=True))
                try:
                    morph.quat = q
                except Exception:
                    object.__setattr__(morph, 'quat', q)
            if riser != 0.0:
                pos = tuple(float(x) for x in morph.pos)
                try:
                    morph.pos = (pos[0], pos[1], pos[2] + riser)
                except Exception:
                    object.__setattr__(morph, 'pos', (pos[0], pos[1], pos[2] + riser))
                assert abs(float(morph.pos[2]) - (pos[2] + riser)) < 1e-9, morph.pos
        return orig(self, morph, material=material, surface=surface, *a, **kw)
    gs.Scene.add_entity = add_entity
    _installed['name'] = name


def post_build(w, name):
    """Set arm gains / effort limits on the built world dict (replay_harness build_world's w)."""
    v = VARIANTS[name]
    kin = w['kinova']; kdofs = w['kdofs']
    arm = np.asarray(kdofs[:6])
    kp = np.array(BASE_KP, float) * v['kp_mult']; kv = np.array(BASE_KV, float) * v['kv_mult']
    kin.set_dofs_kp(kp=kp, dofs_idx_local=arm)
    kin.set_dofs_kv(kv=kv, dofs_idx_local=arm)
    eff = np.array(URDF_EFFORT if v['effort'] == 'urdf' else BASE_EFFORT, float)
    kin.set_dofs_force_range(lower=-eff, upper=eff, dofs_idx_local=arm)
    if v.get('finger_force') is not None:
        ff = float(v['finger_force']); fd = np.asarray(kdofs[-4:])
        kin.set_dofs_force_range(lower=-np.full(4, ff), upper=np.full(4, ff), dofs_idx_local=fd)
    world = v.get('world')
    if world is not None:
        assert set(world) == set(WORLD_KEYS), (name, sorted(world))
        # finger PD gain + driver cap on the four finger dofs (kv untouched: build_world already
        # sets 10 when trial_placements carries finger_kp, which is the search world's kv)
        fd = np.asarray(kdofs[-4:])
        kin.set_dofs_kp(kp=np.full(4, float(world['finger_kp'])), dofs_idx_local=fd)
        ff = float(world['finger_force'])
        kin.set_dofs_force_range(lower=-np.full(4, ff), upper=np.full(4, ff), dofs_idx_local=fd)
        # the install() material swaps must have hit exactly the entities they were written for
        subs = dict(_installed['subs']); _installed['subs'] = {}
        assert _installed['name'] == name, (_installed['name'], name)
        assert subs.get('shelf') == 1 and subs.get('can') == 1 and subs.get('goal') == 1 \
            and subs.get('table', 0) <= 1, f'[sim-variant] {name}: material substitutions {subs}'
        print(f'[sim-variant] {name}: world overlay {world} (materials swapped: {subs})', flush=True)
    if v.get('can_segments') is not None:
        # the install() morph swap must have hit exactly the two cans, and each must have come out of
        # the URDF loader as n separate convex CYLINDER geoms (no merge / decomposition) on ONE free link
        n = int(v['can_segments']); segs = dict(_installed['segs']); _installed['segs'] = {}
        assert _installed['name'] == name, (_installed['name'], name)
        assert segs == {'can': 1, 'goal': 1}, f'[sim-variant] {name}: can morph swaps {segs}'
        import genesis as gs
        for lab, ent in (('can', w['bottle']), ('goal', w['goal'])):
            gl = list(ent.geoms)
            assert len(gl) == n and ent.geom_end - ent.geom_start == n, f'[sim-variant] {name}: {lab} has {len(gl)} geoms, expected {n}'
            assert ent.n_links == 1 and ent.n_dofs == 6, (lab, ent.n_links, ent.n_dofs)
            assert all(g.type == gs.GEOM_TYPE.CYLINDER and g.is_convex for g in gl), [(g.type, g.is_convex) for g in gl]
            zs = sorted(float(g.init_pos[2]) for g in gl)
            assert np.allclose(np.diff(zs), zs[1] - zs[0], atol=1e-6) and abs(sum(zs)) < 1e-6, zs   # evenly stacked, centred on the link (float32 geoms)
            # the URDF loader hard-codes geom friction 1.0 (genesis/utils/urdf.py "TODO: parse friction"):
            # re-apply the material the builder handed to add_entity, read back
            fr = float(ent.material.friction); ent.set_friction(fr)
            solver = w['scene'].sim.rigid_solver
            got = [float(solver.geoms_info.friction[g.idx]) for g in gl]
            assert np.allclose(got, fr, atol=1e-6), (lab, got, fr)
            # mass / inertia must be the primitive can's for this rho (explicit <inertial> in the URDF)
            r = float(gl[0].init_verts[:, :2].max()); h = float(zs[-1] - zs[0] + (zs[1] - zs[0]))
            m_want = float(ent.material.rho) * np.pi * r * r * h
            assert abs(float(ent.get_mass()) - m_want) < 0.01 * m_want, (lab, float(ent.get_mass()), m_want)
        print(f'[sim-variant] {name}: split can x{n} (segment height {zs[1] - zs[0]:.4f} m) on both cans; friction re-applied', flush=True)
    if v.get('grasp_timeconst') is not None:
        # contact softness of the finger<->can pair. Genesis AVERAGES the two geoms' sol_params, so
        # the finger geoms and the can must both be set or the pair only moves halfway. The solver
        # clamps timeconst >= 2*substep_dt; we clamp too so a bad value fails loudly rather than
        # silently reverting. set_global_sol_params() is broken in this build (1-D array into
        # _sanitize_sol_params) -> write the geoms directly. paper/gripper_lab_2026-08-25.md §6.
        solver = w['scene'].sim.rigid_solver
        tmin = 2.0 * float(solver._substep_dt)
        tc = max(float(v['grasp_timeconst']), tmin)
        base = list(solver.geoms_info.sol_params[0])
        vec = [tc] + list(base[1:])
        n_set = 0
        for lk in kin.links:
            if 'finger' in lk.name:
                for gg in lk.geoms:
                    solver.geoms_info.sol_params[gg.idx] = vec; n_set += 1
        for i in range(w['bottle'].geom_start, w['bottle'].geom_end):
            solver.geoms_info.sol_params[i] = vec; n_set += 1
        print(f'[sim-variant] grasp_timeconst {tc} s on {n_set} geoms (floor {tmin})', flush=True)
        if v.get('n_grasp_geoms') is not None:   # variants that declare it assert the count they touched
            assert n_set == int(v['n_grasp_geoms']), f'[sim-variant] {name}: grasp_timeconst touched {n_set} geoms, expected {v["n_grasp_geoms"]}'
    if v.get('impedance') is not None:
        # constraint IMPEDANCE (sol_params dmin, dmax; MuJoCo solimp d). The contact constraint is
        # regularised by R ~ (1 - d) / d (constraint_solver_decomp._func_add_contact: diag *= 2 mu^2
        # (1 - imp) / imp), so a sustained sub-limit tangential load makes the pair CREEP at a rate
        # ~ (1 - d) / d x timeconst instead of holding (pinch lab, 2026-09-03: a 0.1 N m gravity
        # torque on a pinched can droops 59 deg/s at the default 0.9/0.95 vs 8.5 at 0.99 vs 0.8 at
        # 0.999; the can slides 3 mm/s down a centred pinch vs 0.3 / 0.1 -- the census's in-grasp slip
        # and the in-hand / partial-release tips; splitting the can into 6 geoms did nothing). Applied
        # AFTER grasp_timeconst (which rebuilds the whole vector) and by replacing only [dmin, dmax]
        # of each geom's current vector. dmin == dmax removes the penetration dependence (imp is
        # then d at every depth). Genesis clips d to [1e-4, 0.9999]; we require d < 1.
        # impedance_scope: 'all' (every geom: every pair gets d; the sane MuJoCo-style setting) or
        # 'grasp' (finger + can geoms only, as grasp_timeconst: the grasp pair gets d, the can's
        # pairs with shelf / goal get the average of d and the default 0.9/0.95).
        solver = w['scene'].sim.rigid_solver
        d = float(v['impedance']); assert 0.5 <= d < 1.0, d
        scope = v.get('impedance_scope', 'all')
        if scope == 'all':
            idx = list(range(int(solver.n_geoms)))
        elif scope == 'grasp':
            idx = [gg.idx for lk in kin.links if 'finger' in lk.name for gg in lk.geoms] + list(range(w['bottle'].geom_start, w['bottle'].geom_end))
            if v.get('n_grasp_geoms') is not None:
                assert len(idx) == int(v['n_grasp_geoms']), f'[sim-variant] {name}: impedance touched {len(idx)} geoms, expected {v["n_grasp_geoms"]}'
        else:
            raise ValueError(f'[sim-variant] {name}: impedance_scope {scope!r}')
        for i in idx:
            cur = list(solver.geoms_info.sol_params[i]); cur[2] = d; cur[3] = d
            solver.geoms_info.sol_params[i] = cur
        got = [tuple(float(x) for x in list(solver.geoms_info.sol_params[i])[2:4]) for i in idx]
        assert all(abs(a - d) < 1e-6 and abs(b - d) < 1e-6 for a, b in got), got
        print(f'[sim-variant] impedance dmin = dmax = {d} on {len(idx)} geoms (scope {scope}; timeconst untouched)', flush=True)
    if v.get('shelf_dz'):
        # the goal can rests on the shelf: its spawn height follows the shelf (read at every reset)
        w['goal_start_z'] = float(w['goal_start_z']) + float(v['shelf_dz'])
    return dict(name=name, kp=kp.tolist(), kv=kv.tolist(), effort=eff.tolist(), gravity_comp=v['gravity_comp'],
                riser=float(v.get('riser', 0.0)), yaw=float(v.get('yaw', 0.0)),
                finger_force=v.get('finger_force'), finger_map=v.get('finger_map'),
                grasp_timeconst=v.get('grasp_timeconst'), can_segments=v.get('can_segments'),
                impedance=v.get('impedance'), impedance_scope=(v.get('impedance_scope', 'all') if v.get('impedance') is not None else None),
                shelf_dz=float(v.get('shelf_dz', 0.0)), shelf_top=shelf_top(name),
                world=(dict(v['world']) if v.get('world') else None))


def grip_frac(name, frac):
    """Closing fraction actually commanded for a real gripper reading `frac` (0..1) under the variant."""
    m = VARIANTS[name].get('finger_map')
    return float(frac) if m is None else float(frac) * float(m)


def describe(name):
    v = VARIANTS[name]
    return dict(name=name, kp=(np.array(BASE_KP) * v['kp_mult']).tolist(), kv=(np.array(BASE_KV) * v['kv_mult']).tolist(),
                effort=(URDF_EFFORT if v['effort'] == 'urdf' else BASE_EFFORT), gravity_comp=v['gravity_comp'],
                riser=float(v.get('riser', 0.0)), finger_force=v.get('finger_force'), finger_map=v.get('finger_map'),
                grasp_timeconst=v.get('grasp_timeconst'), can_segments=v.get('can_segments'),
                shelf_dz=float(v.get('shelf_dz', 0.0)),
                world=(dict(v['world']) if v.get('world') else None))


def selftest(name, backend='cpu'):
    """Build the world the way GenesisCanEnv does (trial_placements.json world block), apply the
    variant, and READ BACK from the built solver everything the variant claims to set. Exit 1 on
    any mismatch. ~15 s on the cpu backend."""
    import json, math, pathlib as pl, sys
    REPO = pl.Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    from replay_harness import build_world
    v = VARIANTS[name]; world = v.get('world') or {}
    cfg = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['world']
    install(name)
    w = build_world(backend=backend, finger_force=cfg['finger_force'], finger_kp=cfg['finger_kp'],
                    can_height=cfg['can_height'], can_rho=cfg['can_rho'], substeps=cfg.get('substeps', 1),
                    table=cfg.get('table', False), can_radius=cfg.get('can_radius', 0.035))
    info = post_build(w, name)
    kin, kdofs, solver = w['kinova'], w['kdofs'], w['scene'].sim.rigid_solver
    fails = []

    def check(label, got, want, tol=1e-4):
        ok = np.allclose(np.asarray(got, float), np.asarray(want, float), atol=tol, rtol=0)
        print(f'  {"ok " if ok else "FAIL"} {label}: got {np.round(np.asarray(got, float), 5).tolist()} want {want}')
        if not ok: fails.append(label)

    arm = np.asarray(kdofs[:6]); fd = np.asarray(kdofs[-4:])
    check('arm kp', kin.get_dofs_kp(dofs_idx_local=arm), info['kp'])
    check('arm kv', kin.get_dofs_kv(dofs_idx_local=arm), info['kv'])
    lo, hi = kin.get_dofs_force_range(dofs_idx_local=arm)
    check('arm effort', hi, info['effort'])
    want_fkp = world.get('finger_kp', cfg['finger_kp']); want_ff = world.get('finger_force', v.get('finger_force') or cfg['finger_force'])
    check('finger kp', kin.get_dofs_kp(dofs_idx_local=fd), [want_fkp] * 4)
    check('finger kv', kin.get_dofs_kv(dofs_idx_local=fd), [10.0] * 4)
    lo, hi = kin.get_dofs_force_range(dofs_idx_local=fd)
    check('finger force hi', hi, [want_ff] * 4); check('finger force lo', lo, [-want_ff] * 4)
    # geoms: which geom belongs to which entity
    ents = dict(can=w['bottle'], goal=w['goal'])
    for e in w['scene'].entities:
        m = getattr(e, 'morph', None)
        if m is not None and m.__class__.__name__ == 'Box':
            sz = np.asarray(m.size, float)
            if np.allclose(sz, BOX_SIZE): ents['shelf'] = e
            elif np.allclose(sz[:2], TABLE_SIZE_XY): ents['table'] = e
    geoms = {k: list(range(e.geom_start, e.geom_end)) for k, e in ents.items()}
    finger_geoms = [gg.idx for lk in kin.links if 'finger' in lk.name for gg in lk.geoms]
    geoms['finger'] = finger_geoms
    print(f'  geoms: {geoms}')
    fric = {k: [float(solver.geoms_info.friction[i]) for i in idx] for k, idx in geoms.items()}
    tcs = {k: [float(solver.geoms_info.sol_params[i][0]) for i in idx] for k, idx in geoms.items()}
    check('can friction', fric['can'], [world.get('can_friction', 0.2)] * len(geoms['can']))
    check('goal friction', fric['goal'], [world.get('goal_friction', 2.0)] * len(geoms['goal']))
    check('shelf friction', fric['shelf'], [world.get('table_friction', 0.5)] * len(geoms['shelf']))
    if 'table' in geoms:
        check('table friction', fric['table'], [world.get('table_friction', 0.5)] * len(geoms['table']))
    tc = v.get('grasp_timeconst'); tc_default = 0.02
    if tc is not None:
        tc = max(float(tc), 2.0 * float(solver._substep_dt))
        check('finger timeconst', tcs['finger'], [tc] * len(finger_geoms)); check('can timeconst', tcs['can'], [tc] * len(geoms['can']))
        check('n grasp geoms', [len(finger_geoms) + len(geoms['can'])], [v.get('n_grasp_geoms', len(finger_geoms) + len(geoms['can']))])
    check('goal timeconst (untouched)', tcs['goal'], [tc_default] * len(geoms['goal']))
    check('shelf timeconst (untouched)', tcs['shelf'], [tc_default] * len(geoms['shelf']))
    # impedance (dmin, dmax) per geom group: the variant's d where in scope, the engine default elsewhere
    d = v.get('impedance'); scope = v.get('impedance_scope', 'all'); d_def = (0.9, 0.95)
    imps = {k: [tuple(float(x) for x in list(solver.geoms_info.sol_params[i])[2:4]) for i in idx] for k, idx in geoms.items()}
    for k in ('finger', 'can', 'goal', 'shelf', 'table'):
        if k not in geoms: continue
        want = (d, d) if (d is not None and (scope == 'all' or k in ('finger', 'can'))) else d_def
        check(f'{k} impedance (dmin,dmax)', np.asarray(imps[k]).reshape(-1), list(want) * len(geoms[k]), tol=1e-6)
    if d is not None and scope == 'all':
        allimp = [tuple(float(x) for x in list(solver.geoms_info.sol_params[i])[2:4]) for i in range(int(solver.n_geoms))]
        check('ALL geoms impedance', np.asarray(allimp).reshape(-1), [d, d] * int(solver.n_geoms), tol=1e-6)
    # Genesis meshes the cylinder -> mass = rho * polygon volume = 0.64 % under pi r^2 h; check to 1 %
    # and check the can/goal ratio (same mesh) exactly
    rho = world.get('can_rho', cfg['can_rho']); r = cfg.get('can_radius', 0.035); h = cfg['can_height']
    mc, mg = float(w['bottle'].get_mass()), float(w['goal'].get_mass())
    check('can mass (rho*pi*r^2*h, 1%)', [mc], [rho * math.pi * r * r * h], tol=0.01 * rho * math.pi * r * r * h)
    check('goal mass (rho 1000, 1%)', [mg], [1000 * math.pi * r * r * h], tol=0.01 * 1000 * math.pi * r * r * h)
    check('can/goal mass ratio (= rho/1000)', [mc / mg], [rho / 1000.0])
    if v.get('can_segments') is not None:
        # split can: the explicit <inertial> must reproduce the solid cylinder's inertia about its CoM
        # (Ixx = Iyy = m(3r^2+h^2)/12, Izz = m r^2/2; polygon mesh -> 1 %), CoM at the link origin
        for lab, ent, m_want in (('can', w['bottle'], rho * math.pi * r * r * h), ('goal', w['goal'], 1000 * math.pi * r * r * h)):
            lk = ent.links[0]; I = np.asarray(lk.inertial_i, float)
            want = [m_want * (3 * r * r + h * h) / 12] * 2 + [m_want * r * r / 2]
            check(f'{lab} inertia diag (1%)', np.diag(I), want, tol=0.01 * want[0])
            check(f'{lab} inertia off-diag', [I[0, 1], I[0, 2], I[1, 2]], [0, 0, 0], tol=1e-9)
            check(f'{lab} inertial CoM', np.asarray(lk.inertial_pos, float).reshape(-1), [0, 0, 0], tol=1e-6)
            check(f'{lab} n geoms', [len(list(ent.geoms))], [int(v['can_segments'])])
    check('goal_start_z (shelf_dz applied)', [w['goal_start_z']], [BOX_TOP_Z + h / 2 + 0.0425 + float(v.get('shelf_dz', 0.0))])
    if v.get('shelf_dz'):
        check('shelf centre z', [float(np.asarray(ents['shelf'].get_pos()).reshape(-1)[2])], [BOX_POS[2] + float(v['shelf_dz'])], tol=1e-3)
    check('kinova base z (riser)', [float(np.asarray(kin.get_pos()).reshape(-1)[2])], [0.05 + float(v.get('riser', 0.0))], tol=1e-3)
    # yaw: read the BUILT base orientation back, not the morph we set. quat is (w,x,y,z); a pure
    # z-rotation by `yaw` degrees gives w = cos(yaw/2), z = sin(yaw/2) (and x = y = 0).
    q = np.asarray(kin.get_quat()).reshape(-1)
    want_yaw = float(v.get('yaw', 0.0))
    got_yaw = float(np.degrees(2.0 * np.arctan2(q[3], q[0])))
    check('kinova base yaw (deg)', [got_yaw], [want_yaw], tol=1e-2)
    check('kinova base tilt (x,y quat components must stay 0)', [q[1], q[2]], [0.0, 0.0], tol=1e-6)
    if want_yaw:
        # the scene must NOT have moved with the arm -- that is the whole point of the parameter
        check('shelf x,y unmoved by yaw', list(np.asarray(ents['shelf'].get_pos()).reshape(-1)[:2]),
              list(BOX_POS[:2]), tol=1e-3)
    print(f'[selftest] {name}: {"PASS" if not fails else "FAIL " + str(fails)}')
    return not fails


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', metavar='VARIANT')
    ap.add_argument('--backend', default='cpu')
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest(a.selftest, a.backend) else 1)
    for k in VARIANTS: print(k, describe(k))
