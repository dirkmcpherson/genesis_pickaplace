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
}

_installed = {'name': None, 'orig': None}


def install(name):
    """Monkeypatch gs.Scene.add_entity so the Kinova URDF gets gravity compensation."""
    v = VARIANTS[name]
    import genesis as gs
    if _installed['orig'] is None:
        _installed['orig'] = gs.Scene.add_entity
    orig = _installed['orig']
    gc = float(v['gravity_comp']); riser = float(v.get('riser', 0.0))

    def add_entity(self, morph=None, material=None, surface=None, *a, **kw):
        if isinstance(morph, gs.morphs.URDF) and 'gen3' in str(getattr(morph, 'file', '')):
            if material is None and gc != 0.0:
                material = gs.materials.Rigid(gravity_compensation=gc)
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
    return dict(name=name, kp=kp.tolist(), kv=kv.tolist(), effort=eff.tolist(), gravity_comp=v['gravity_comp'],
                riser=float(v.get('riser', 0.0)))


def describe(name):
    v = VARIANTS[name]
    return dict(name=name, kp=(np.array(BASE_KP) * v['kp_mult']).tolist(), kv=(np.array(BASE_KV) * v['kv_mult']).tolist(),
                effort=(URDF_EFFORT if v['effort'] == 'urdf' else BASE_EFFORT), gravity_comp=v['gravity_comp'],
                riser=float(v.get('riser', 0.0)))
