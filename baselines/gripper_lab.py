#!/usr/bin/env python
"""GRIPPER LAB -- joint retune of the simulated Kinova gen3-lite 2F finger model.

Problem (paper/real2sim_follower_lab_2026-08-23.md §5, §9.4): the four finger dofs are
position-driven from the REAL gripper reading mapped LINEARLY onto the URDF joint range.
At a secure grasp the real reading is ~0.83 while the sim fingers stall on the 66 mm can at
~0.61 -> the finger PD (kp 40, force range +-50 N*m) drives ~0.23 rad INTO the can, which the
soft contact model absorbs as 7-10 mm of interpenetration at 80-150 N.  Two single-knob
attempts failed as adoptable fixes (`finger_force=2.0` halves penetration but the can slips;
`finger_map=0.735` halves it but costs picks), so the finger model needs a JOINT retune.

This module is ADDITIVE: it does not modify sim_variants.py, human_follower_lab.py,
sim_fidelity_lab.py, fulltask_fidelity_lab.py or record_demos.py.  `register_all()` injects
gripper configs into `sim_variants.VARIANTS` and wraps `sim_variants.install/post_build/
grip_frac`, so every existing tool can be driven with `--sim/--variant <gripper-config>`.

CONFIG SPACE (GCFG; every default reproduces the base world byte-identically)
  base            sim_variants variant the gripper config sits on  ('gc_kp4_riser3_shelf6')
  map_scale/off   commanded closing fraction f -> clip(off + scale*f, 0, map_max) before the
  map_max         URDF linear angle map (scale 0.735 == the report's `finger_map`; map_max is a
                  SQUEEZE CLAMP: "close until the jaw is this far shut and no further").  The sim
                  jaw is linear in f: distal-pad gap = 104.8 - 103.8*f mm (measured), so
                  map_max 0.45 == a 58 mm jaw, 4 mm inside a 66 mm can.
  map_obs         also invert the map on the observed gripper reading (obs[6]) so tapes stay
                  in REAL-reading units across configs                              (True)
  finger_kp/kv    PD gains on all four finger dofs        (world_cfg: kp 40, kv 10)
  tip_kp/tip_kv   override for the two TIP dofs only (they emulate the real gripper's rigid
                  distal linkage; the URDF mimic IS enforced as an equality constraint in this
                  Genesis build, but the tip dofs are ALSO PD-driven)
  finger_force    +-N*m force range on all four finger dofs           (world_cfg: 50)
  driver_force    +-N*m on the two BOTTOM (driver) dofs only  -- overrides finger_force
  tip_force       +-N*m on the two TIP dofs only              -- overrides finger_force
  pad_friction    friction of the four finger collision geoms      (URDF default 1.0)
  can_friction    friction of the picked can                       (world_cfg: 0.2)
  goal_friction   friction of the goal can                         (build_world: 2.0)
  pad_sol/can_sol/global_sol
                  MuJoCo-style contact solver params as a dict of
                  {timeconst, dampratio, dmin, dmax, width, mid, power}.  Genesis averages the
                  two geoms' sol_params for a contact pair, so a finger-only change moves the
                  finger-can contact HALF way.  Genesis clamps timeconst >= 2*substep_dt
                  (0.0025 s here).  Build default for every geom: timeconst 0.02, dampratio 1,
                  dmin 0.9, dmax 0.95, width 1e-3, mid 0.5, power 2.
  urdf            alternative URDF (e.g. a shrunk pad hull); None = the world's own

SUBCOMMANDS
  bench   fast screening: replay short human pick tapes, measure stall angle, penetration
          (solver AND exact convex-hull geometry), contact force, in-hand slip, pick.
  pick    (a) PICK-SCOPE RECREATION -- human_follower_lab --config arr_either kept /66.
  full    (b)(c) full-task replay -- fulltask_fidelity_lab (penetration/force per phase,
          tips, placed/contact/nested).
  fid     (d) sim_fidelity_lab: sim-vs-REAL arm tracking (a gripper change must not move it).
  negctl  (e) random-teacher negative control (record_demos --teacher random).
  report  tables over everything produced so far.

USAGE
  python baselines/gripper_lab.py list
  python baselines/gripper_lab.py bench --cfg base stiffpad stiffpad_f5 --uids 232 235 242 243
  python baselines/gripper_lab.py pick  --cfg stiffpad --parallel 3 --fresh
  python baselines/gripper_lab.py full  --cfg stiffpad --parallel 3
  python baselines/gripper_lab.py fid   --cfg stiffpad --parallel 3
  python baselines/gripper_lab.py report
Outputs: baselines/demos_v1/_grip/<cfg>/ (bench), plus the host tools' own trees
(_lab/<follower>@<cfg>/, _fulltask/<cfg>/, _simlab/<cfg>/).
"""
import argparse, glob, json, os, pathlib as pl, subprocess, sys, time
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))

OUT_ROOT = 'baselines/demos_v1/_grip'
BASE_SIM = 'gc_kp4_riser3_shelf6'          # world of record (report §9.6)
SOL_KEYS = ('timeconst', 'dampratio', 'dmin', 'dmax', 'width', 'mid', 'power')
SOL_DEFAULT = (0.02, 1.0, 0.9, 0.95, 1e-3, 0.5, 2.0)   # verified at runtime, this Genesis build
BENCH_UIDS = [243, 235, 242, 262, 278, 233]     # short human pick tapes; spread over can buckets and grip depth
SRC_DEFAULT = 'baselines/episodes_pick_phase_dppruned'
PICK_FOLLOWER = 'arr_either'               # the report's adoptable follower (§7.1)

# --------------------------------------------------------------------------- config space
DEF = dict(base=BASE_SIM, map_scale=1.0, map_offset=0.0, map_max=1.0, map_obs=True,
           finger_kp=None, finger_kv=None, tip_kp=None, tip_kv=None,
           finger_force=None, driver_force=None, tip_force=None,
           pad_friction=None, can_friction=None, goal_friction=None,
           pad_sol=None, can_sol=None, global_sol=None, urdf=None, note='')

def C(**kw):
    bad = set(kw) - set(DEF)
    assert not bad, f'unknown gripper-config keys {bad}'
    d = dict(DEF); d.update(kw); return d

# The named grid.  `base` == the world of record with NO gripper change (control).
CFGS = {
    'g_base':        C(note='world of record, gripper untouched (control)'),
    # --- A. contact stiffness only (squeeze force unchanged) -------------------------------
    'g_padstiff':    C(pad_sol=dict(timeconst=0.0025), note='finger geoms at the solver floor -> finger-can timeconst 0.011'),
    'g_padcanstiff': C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025),
                     note='finger AND can at the floor -> finger-can timeconst 0.0025 (8x stiffer)'),
    'g_stiff5':      C(pad_sol=dict(timeconst=0.005), can_sol=dict(timeconst=0.005), note='finger-can timeconst 0.005'),
    'g_dmax99':      C(pad_sol=dict(dmax=0.99), can_sol=dict(dmax=0.99), note='impedance 0.95 -> 0.99 (5x less constraint softness)'),
    'g_padcanstiff_dmax99': C(pad_sol=dict(timeconst=0.0025, dmax=0.99), can_sol=dict(timeconst=0.0025, dmax=0.99),
                            note='stiffest reachable finger-can contact'),
    'g_globalstiff': C(global_sol=dict(timeconst=0.0025), note='EVERY contact at the floor (world-wide)'),
    # --- B. stiffness + a physical squeeze (the joint retune) ------------------------------
    'g_stiff_f10':   C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=10.0),
    'g_stiff_f5':    C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=5.0),
    'g_stiff_f2':    C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=2.0),
    'g_stiff_f1':    C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=1.0),
    'g_stiff_f05':   C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=0.5),
    'g_stiff_f2_mu2': C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=2.0, pad_friction=2.0),
    'g_stiff_f1_mu2': C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=1.0, pad_friction=2.0),
    'g_stiff_f5_mu2': C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), driver_force=5.0, pad_friction=2.0),
    # --- C. the report's two failed single knobs, reproduced on the world of record --------
    'g_fcap2':       C(finger_force=2.0, note="report's finger_force=2.0 (all four dofs)"),
    'g_fmap':        C(map_scale=0.735, note="report's finger_map=0.735"),
    'g_fcap2_drv':   C(driver_force=2.0, note='2 N*m on the DRIVERS ONLY (tips keep the rigid linkage)'),
    'g_fmap_stiff':  C(map_scale=0.735, pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025)),
    # --- D. softer finger PD ---------------------------------------------------------------
    'g_kp10':        C(finger_kp=10.0),
    'g_stiff_kp10':  C(pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025), finger_kp=10.0),
    'g_mu2':         C(pad_friction=2.0, note='pad friction only'),
    # --- E. squeeze CLAMP: close until the jaw is f_max shut, then stop (diagnostic: how much
    #        squeeze does the grasp actually need?).  Object-specific -- see the report.
    'g_clamp45':     C(map_max=0.45, note='jaw clamp at 58 mm (4 mm inside the 66 mm can)'),
    'g_clamp50':     C(map_max=0.50, note='jaw clamp at 53 mm'),
    'g_stiff_clamp45': C(map_max=0.45, pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025)),
    'g_stiff_clamp50': C(map_max=0.50, pad_sol=dict(timeconst=0.0025), can_sol=dict(timeconst=0.0025)),
}


def cfg_of(name):
    if name in CFGS: return dict(CFGS[name])
    sys.exit(f'unknown gripper config {name!r}; known: {sorted(CFGS)}')


def sol_vec(over):
    v = list(SOL_DEFAULT)
    for k, x in (over or {}).items():
        assert k in SOL_KEYS, k
        v[SOL_KEYS.index(k)] = float(x)
    return v


# --------------------------------------------------------------------------- registration
_REG = {'done': False, 'orig': {}}


def register_all():
    """Inject every gripper config into sim_variants and wrap its three entry points.

    After this call `--sim <cfg>` / `--variant <cfg>` works in human_follower_lab.py,
    sim_fidelity_lab.py and fulltask_fidelity_lab.py without editing them.
    """
    if _REG['done']: return
    import sim_variants as SV
    _REG['orig'] = dict(install=SV.install, post_build=SV.post_build, grip_frac=SV.grip_frac)
    for name, g in CFGS.items():
        assert name not in SV.VARIANTS, f'gripper config {name} collides with a sim variant'
        SV.VARIANTS[name] = dict(SV.VARIANTS[g['base']], _grip=name)

    def install(nm):
        g = CFGS.get(nm)
        if g is not None and g['urdf']:
            _patch_urdf(g['urdf'])
        return _REG['orig']['install'](nm)

    def post_build(w, nm):
        info = _REG['orig']['post_build'](w, nm)
        g = CFGS.get(nm)
        if g is not None:
            info['grip'] = apply_gripper(w, g)
            info['grip_cfg'] = nm
        return info

    def grip_frac(nm, frac):
        g = CFGS.get(nm)
        if g is None: return _REG['orig']['grip_frac'](nm, frac)
        return map_frac(g, frac)

    SV.install, SV.post_build, SV.grip_frac = install, post_build, grip_frac
    _REG['done'] = True


def map_frac(g, frac):
    """Commanded closing fraction (real reading 0..1) -> fraction fed to the URDF linear map."""
    return float(np.clip(float(g['map_offset']) + float(g['map_scale']) * float(frac), 0.0, float(g['map_max'])))


def unmap_frac(g, frac):
    s = float(g['map_scale'])
    if s == 0.0: return 0.0
    return float(np.clip((float(frac) - float(g['map_offset'])) / s, 0.0, 1.0))


def _patch_urdf(fname):
    """Swap the Kinova URDF file for the gripper-geometry variants (pre-build)."""
    import genesis as gs
    orig = gs.Scene.add_entity

    def add_entity(self, morph=None, material=None, surface=None, *a, **kw):
        if isinstance(morph, gs.morphs.URDF) and 'gen3' in str(getattr(morph, 'file', '')):
            p = pl.Path(str(morph.file))
            try: morph.file = str(p.parent / fname)
            except Exception: object.__setattr__(morph, 'file', str(p.parent / fname))
        return orig(self, morph, material=material, surface=surface, *a, **kw)
    gs.Scene.add_entity = add_entity


def finger_dofs(w):
    """(driver_dofs, tip_dofs) as LOCAL dof indices; kdofs follows kinova.JOINT_NAMES order:
    [j1..j6, left_finger_bottom, right_finger_bottom, left_finger_tip, right_finger_tip]."""
    kd = list(w['kdofs'])
    return np.asarray(kd[6:8]), np.asarray(kd[8:10])


def apply_gripper(w, g):
    """Apply every gripper knob to a BUILT world dict; returns what was actually set."""
    kin = w['kinova']; sol = w['scene'].sim.rigid_solver
    drv, tip = finger_dofs(w)
    allf = np.concatenate([drv, tip])
    out = {}
    # --- PD gains -------------------------------------------------------------------------
    if g['finger_kp'] is not None:
        kin.set_dofs_kp(kp=np.full(4, float(g['finger_kp'])), dofs_idx_local=allf); out['finger_kp'] = g['finger_kp']
    if g['finger_kv'] is not None:
        kin.set_dofs_kv(kv=np.full(4, float(g['finger_kv'])), dofs_idx_local=allf); out['finger_kv'] = g['finger_kv']
    if g['tip_kp'] is not None:
        kin.set_dofs_kp(kp=np.full(2, float(g['tip_kp'])), dofs_idx_local=tip); out['tip_kp'] = g['tip_kp']
    if g['tip_kv'] is not None:
        kin.set_dofs_kv(kv=np.full(2, float(g['tip_kv'])), dofs_idx_local=tip); out['tip_kv'] = g['tip_kv']
    # --- force ranges (finger_force first, then the per-group overrides) --------------------
    if g['finger_force'] is not None:
        f = float(g['finger_force'])
        kin.set_dofs_force_range(lower=-np.full(4, f), upper=np.full(4, f), dofs_idx_local=allf); out['finger_force'] = f
    if g['driver_force'] is not None:
        f = float(g['driver_force'])
        kin.set_dofs_force_range(lower=-np.full(2, f), upper=np.full(2, f), dofs_idx_local=drv); out['driver_force'] = f
    if g['tip_force'] is not None:
        f = float(g['tip_force'])
        kin.set_dofs_force_range(lower=-np.full(2, f), upper=np.full(2, f), dofs_idx_local=tip); out['tip_force'] = f
    # --- friction ---------------------------------------------------------------------------
    fg = finger_geoms(w)
    if g['pad_friction'] is not None:
        for gi in fg: gi.set_friction(float(g['pad_friction']))
        out['pad_friction'] = g['pad_friction']
    if g['can_friction'] is not None:
        w['bottle'].set_friction(float(g['can_friction'])); out['can_friction'] = g['can_friction']
    if g['goal_friction'] is not None:
        w['goal'].set_friction(float(g['goal_friction'])); out['goal_friction'] = g['goal_friction']
    # --- contact solver params ---------------------------------------------------------------
    tmin = 2.0 * float(sol._substep_dt)
    if g['global_sol'] is not None:
        v = sol_vec(g['global_sol']); v[0] = max(v[0], tmin)
        # NB solver.set_global_sol_params() is broken in this Genesis build (it hands a 1-D
        # array to _sanitize_sol_params, which np.stack(axis=1)s it) -> write every geom.
        for i in range(sol.n_geoms): sol.geoms_info.sol_params[i] = v
        out['global_sol'] = v
    if g['pad_sol'] is not None:
        v = sol_vec(g['pad_sol']); v[0] = max(v[0], tmin)
        for gi in fg: sol.geoms_info.sol_params[gi.idx] = v
        out['pad_sol'] = v
    if g['can_sol'] is not None:
        v = sol_vec(g['can_sol']); v[0] = max(v[0], tmin)
        for i in range(w['bottle'].geom_start, w['bottle'].geom_end): sol.geoms_info.sol_params[i] = v
        out['can_sol'] = v
    # --- the reading -> angle map (patched into every module that owns a copy) ----------------
    # ALWAYS called, so that a bench sweeping several configs in one process cannot leak a map
    # from one config into the next (an identity map restores the untouched functions).
    if _patch_map(g):
        out['map'] = [g['map_offset'], g['map_scale'], g['map_max'], g['map_obs']]
    out['sol_timeconst_min'] = tmin
    return out


def finger_geoms(w):
    return [gg for l in w['kinova'].links if 'finger' in l.name for gg in l.geoms]


def _patch_map(g):
    """Apply the reading->angle map inside the env step (genesis_can_env / full_env own their
    own reference to replay_harness.gripper_targets).  With map_obs, the inverse is applied to
    the observed motor reading so obs[6] stays in REAL-gripper-reading units.  An identity map
    restores the original functions (idempotent: safe to call once per config in one process).
    Returns True iff a non-identity map is now installed."""
    import replay_harness as RH
    if not hasattr(RH, '_gl_orig_gripper_targets'):
        RH._gl_orig_gripper_targets = RH.gripper_targets
    orig = RH._gl_orig_gripper_targets
    identity = (g['map_scale'] == 1.0 and g['map_offset'] == 0.0 and g['map_max'] == 1.0)

    def gt(g_pos, invert=True):
        return orig(map_frac(g, float(np.clip(g_pos, 0.0, 100.0)) / 100.0) * 100.0, invert=invert)
    tgt = orig if identity else gt
    for modname in ('replay_harness', 'genesis_can_env', 'full_env'):
        m = sys.modules.get(modname)
        if m is not None and hasattr(m, 'gripper_targets'):
            m.gripper_targets = tgt
    GCE = sys.modules.get('genesis_can_env')
    if GCE is not None:
        if not hasattr(GCE, '_gl_orig_obs'):
            GCE._gl_orig_obs = GCE.GenesisCanEnv._obs
        if identity or not g['map_obs']:
            GCE.GenesisCanEnv._obs = GCE._gl_orig_obs
        else:
            def _obs(self, _g=dict(g)):
                o = GCE._gl_orig_obs(self)
                s = o['state']; s[6] = np.float32(unmap_frac(_g, float(s[6]))); o['state'] = s
                return o
            GCE.GenesisCanEnv._obs = _obs
    return not identity


# ------------------------------------------------------------------ exact geometric penetration
class PenProbe:
    """Interpenetration measured from CONTACT GEOMETRY, not from the solver's report.

    The can collision geom is a convex 32-gon prism; the four finger collision geoms are the
    convex hulls of the CAD meshes (verified: hull volume == STL convex-hull volume, AABB
    identical to the STL, hull vertices <= 1.3 mm off the STL surface -- so the hull fills the
    mesh's concavities but does NOT inflate the pad envelope).  For a convex can we can compute
    the exact signed distance of any point: sd(p) = max_faces n_f . (p - v_f); sd < 0 == inside.
    Penetration = -min over the finger collision vertices of sd, i.e. the deepest finger
    vertex below the can surface, in metres.  Finger vertices are carried in link-local
    coordinates and transformed with the link pose each frame (validated against
    geom.get_verts() to 1.2e-4 mm)."""

    def __init__(self, w):
        self.w = w
        self.fg = finger_geoms(w)
        self.links = []
        for l in w['kinova'].links:
            if 'finger' not in l.name: continue
            for gg in l.geoms:
                V = np.asarray(gg.get_verts(), np.float64)
                p = np.asarray(_np(l.get_pos()), np.float64); q = np.asarray(_np(l.get_quat()), np.float64)
                self.links.append((l, gg, (V - p) @ _R(q)))
        can = w['bottle']
        cg = list(can.geoms)[0]
        V = np.asarray(cg.get_verts(), np.float64); F = np.asarray(cg.init_faces, np.int32).reshape(-1, 3)
        p = np.asarray(_np(can.get_pos()), np.float64); q = np.asarray(_np(can.get_quat()), np.float64)
        Vl = (V - p) @ _R(q)
        n = np.cross(Vl[F[:, 1]] - Vl[F[:, 0]], Vl[F[:, 2]] - Vl[F[:, 0]])
        nn = np.linalg.norm(n, axis=1, keepdims=True); keep = nn[:, 0] > 1e-12
        self.cn = n[keep] / nn[keep]; self.cv = Vl[F[keep, 0]]
        # dedup planes (a 32-gon prism has 34 distinct faces)
        key = np.round(np.concatenate([self.cn, (self.cn * self.cv).sum(1, keepdims=True)], 1), 6)
        _, idx = np.unique(key, axis=0, return_index=True)
        self.cn, self.cv = self.cn[idx], self.cv[idx]
        self.can = can

    def pad_gap(self):
        """Closest approach between the LEFT and RIGHT distal pad collision hulls (m).
        Frame-free (min pairwise vertex distance), so it stays meaningful when the soft mimic
        equality lets the distal links splay under load."""
        side = {}
        for l, gg, loc in self.links:
            if 'dist' not in l.name: continue
            p = np.asarray(_np(l.get_pos()), np.float64); q = np.asarray(_np(l.get_quat()), np.float64)
            side['L' if l.name.startswith('left') else 'R'] = p + loc @ _R(q).T
        if len(side) != 2: return float('nan')
        L, R = side['L'], side['R']
        d = np.sqrt(((L[:, None, :] - R[None, :, :]) ** 2).sum(2))
        return float(d.min())

    def penetration(self):
        """(max penetration [m], number of penetrating finger vertices)."""
        cp = np.asarray(_np(self.can.get_pos()), np.float64); cq = np.asarray(_np(self.can.get_quat()), np.float64)
        Rc = _R(cq)
        best = 0.0; nin = 0
        for l, gg, loc in self.links:
            p = np.asarray(_np(l.get_pos()), np.float64); q = np.asarray(_np(l.get_quat()), np.float64)
            Vw = p + loc @ _R(q).T
            Vl = (Vw - cp) @ Rc                      # can-local
            sd = ((Vl[:, None, :] - self.cv[None]) * self.cn[None]).sum(2).max(1)
            m = sd.min()
            if m < 0: best = max(best, -float(m)); nin += int((sd < 0).sum())
        return best, nin


def _np(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def _R(q):
    w_, x, y, z = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w_ * z), 2 * (x * z + w_ * y)],
                     [2 * (x * y + w_ * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w_ * x)],
                     [2 * (x * z - w_ * y), 2 * (y * z + w_ * x), 1 - 2 * (x * x + y * y)]])


def contact_stats(w, b_geoms, f_geoms):
    """(solver penetration, contact force, n contacts) for finger-can pairs this step."""
    sol = w['scene'].sim.rigid_solver; col = sol.collider
    nc = int(np.asarray(col.n_contacts.to_numpy()).reshape(-1)[0])
    if not nc: return 0.0, 0.0, 0
    cd = col.contact_data.to_numpy()
    GA = np.asarray(cd['geom_a']).reshape(-1)[:nc]; GB = np.asarray(cd['geom_b']).reshape(-1)[:nc]
    m = np.array([(a in b_geoms and b in f_geoms) or (b in b_geoms and a in f_geoms) for a, b in zip(GA, GB)])
    if not m.any(): return 0.0, 0.0, 0
    pen = float(np.asarray(cd['penetration']).reshape(-1)[:nc][m].max())
    frc = float(np.linalg.norm(np.asarray(cd['force']).reshape(-1, 3)[:nc][m], axis=1).max())
    return pen, frc, int(m.sum())


# --------------------------------------------------------------------------- baseline snapshot
def snapshot(w):
    """Everything apply_gripper can touch, as built (so the bench can run many cfgs in one
    process without carrying knobs over)."""
    sol = w['scene'].sim.rigid_solver
    drv, tip = finger_dofs(w); allf = np.concatenate([drv, tip])
    gi = sol.geoms_info.to_numpy()
    return dict(dofs=allf,
                kp=np.asarray(_np(w['kinova'].get_dofs_kp(dofs_idx_local=allf)), np.float64).copy(),
                kv=np.asarray(_np(w['kinova'].get_dofs_kv(dofs_idx_local=allf)), np.float64).copy(),
                frange=[np.asarray(_np(x), np.float64).copy() for x in w['kinova'].get_dofs_force_range(dofs_idx_local=allf)],
                friction=gi['friction'].copy(), sol_params=gi['sol_params'].copy())


def restore(w, s):
    kin = w['kinova']; sol = w['scene'].sim.rigid_solver
    kin.set_dofs_kp(kp=s['kp'], dofs_idx_local=s['dofs'])
    kin.set_dofs_kv(kv=s['kv'], dofs_idx_local=s['dofs'])
    kin.set_dofs_force_range(lower=s['frange'][0], upper=s['frange'][1], dofs_idx_local=s['dofs'])
    for i in range(sol.n_geoms):
        sol.geoms_info.friction[i] = float(s['friction'][i])
        sol.geoms_info.sol_params[i] = s['sol_params'][i].tolist()


# --------------------------------------------------------------------------- bench (screening)
def bench(cfgs, uids, out_tag='', hold=40, verbose=True):
    """Replay short human pick tapes (the REAL joint stream + the REAL gripper reading) under
    each gripper config in ONE process and measure the grasp.  Screening only -- the numbers
    that go in the report come from the per-episode fresh-process runs (pick/full)."""
    import sim_variants as SV
    from replay_harness import build_world, HARDCODED_START, gripper_targets
    base = CFGS[cfgs[0]]['base']
    assert all(CFGS[c]['base'] == base for c in cfgs), 'bench: all configs must share one base world'
    assert all(CFGS[c]['urdf'] is None for c in cfgs), 'bench: URDF variants need their own process'
    tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text()); wc = tbl['world']
    t0 = time.time()
    SV.install(base)
    w = build_world(backend='cpu', finger_force=wc['finger_force'], finger_kp=wc['finger_kp'],
                    can_height=wc['can_height'], can_rho=wc['can_rho'], substeps=wc.get('substeps', 1),
                    table=True, can_radius=wc.get('can_radius', 0.035))
    SV.post_build(w, base)
    snap = snapshot(w)
    print(f'[bench] world {base} built in {time.time()-t0:.0f}s pick_z={w["pick_z"]:.4f}', flush=True)
    sc, kin, bottle, eef, kdofs = w['scene'], w['kinova'], w['bottle'], w['eef'], w['kdofs']
    probe = PenProbe(w)
    b_geoms = set(range(bottle.geom_start, bottle.geom_end))
    f_geoms = set(gg.idx for gg in finger_geoms(w))
    drv, _tip = finger_dofs(w)
    tapes = {}
    for u in uids:
        d = np.load(REPO / SRC_DEFAULT / f'{u}.npz', allow_pickle=True)
        tapes[u] = (d['states'].astype(np.float64), d['actions'].astype(np.float64))
    rows = []
    for cname in cfgs:
        g = cfg_of(cname)
        restore(w, snap)
        applied = apply_gripper(w, g)
        for u in uids:
            S, A = tapes[u]; n = len(S)
            kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
            bottle.set_pos(S[0, 8:11]); bottle.set_quat(S[0, 11:15])
            try: bottle.zero_all_dofs_velocity()
            except Exception: pass
            sc.step()
            te = time.time()
            frac = np.clip(A[:, 6], 0.0, 1.0)
            closed = frac > 0.3
            pen_s = np.zeros(n + hold); pen_g = np.zeros(n + hold); frc = np.zeros(n + hold)
            th = np.zeros(n + hold); zc = np.zeros(n + hold); tilt = np.zeros(n + hold)
            rel = np.full((n + hold, 3), np.nan); ncon = np.zeros(n + hold, int)
            mim = np.zeros(n + hold); asym = np.zeros(n + hold); gap = np.zeros(n + hold)
            picked = False; picked_at = -1; hard_at = -1; run = 0
            for i in range(n + hold):
                k = min(i, n - 1)
                kin.control_dofs_position(A[k, :6], dofs_idx_local=kdofs[:6])
                kin.control_dofs_position(np.array(gripper_targets(map_frac(g, frac[k]) * 100.0)),
                                          dofs_idx_local=np.array(kdofs[-4:]))
                for _ in range(3): sc.step()
                q = _np(kin.get_dofs_position(dofs_idx_local=kdofs))
                th[i] = float(q[7])                       # right_finger_bottom (JOINT_NAMES order)
                # the URDF mimic is a SOFT joint-equality constraint in this Genesis build:
                # measure how far the real linkage relation is violated under load
                mim[i] = max(abs(float(q[8]) - (-0.676 * th[i] + 0.149)),
                             abs(float(q[9]) - (-0.676 * th[i] + 0.149)))
                asym[i] = abs(float(q[6]) + th[i])
                gap[i] = probe.pad_gap()
                bp = _np(bottle.get_pos()); bq = _np(bottle.get_quat()); ee = _np(eef.get_pos())
                zc[i] = float(bp[2]); tilt[i] = _tilt(bq)
                pen_s[i], frc[i], ncon[i] = contact_stats(w, b_geoms, f_geoms)
                pen_g[i] = probe.penetration()[0]
                rel[i] = _R(_np(eef.get_quat())).T @ (np.asarray(bp, float) - np.asarray(ee, float))
                cl = bool(closed[k])
                if not picked and bp[2] > w['pick_z'] and cl: picked = True; picked_at = i
                held = bp[2] > w['pick_z'] and cl and float(np.linalg.norm(ee - bp)) < 0.20
                run = run + 1 if held else 0
                if hard_at < 0 and run >= 10: hard_at = i
            cmask = np.concatenate([closed, np.repeat(closed[-1], hold)])
            # in-hand slip: drift of the can in the EEF frame while a grasp is held
            hm = cmask & (np.arange(n + hold) >= (picked_at if picked_at >= 0 else 10 ** 9))
            slip = float(np.abs(rel[hm] - rel[hm][0]).max()) if hm.sum() > 2 else float('nan')
            # stall: measured closing fraction vs the commanded one at the deepest command
            i_sq = int(np.argmax(np.where(cmask, frac[np.minimum(np.arange(n + hold), n - 1)], -1)))
            f_cmd = float(frac[min(i_sq, n - 1)]); f_meas = 1.0 - (th[i_sq] - (-0.09)) / (0.96 - (-0.09))
            row = dict(cfg=cname, uid=int(u), n=int(n), seconds=round(time.time() - te, 1),
                       picked=bool(picked), hard=bool(hard_at >= 0), picked_at=int(picked_at),
                       f_cmd=round(f_cmd, 3), f_stall=round(float(f_meas), 3),
                       pen_s_max=round(float(pen_s[cmask].max()) * 1000, 2) if cmask.any() else None,
                       pen_g_max=round(float(pen_g[cmask].max()) * 1000, 2) if cmask.any() else None,
                       pen_g_p50=round(float(np.median(pen_g[cmask & (pen_g > 0)])) * 1000, 2) if (cmask & (pen_g > 0)).any() else 0.0,
                       F_max=round(float(frc[cmask].max()), 1) if cmask.any() else None,
                       F_p50=round(float(np.median(frc[cmask & (frc > 0)])), 1) if (cmask & (frc > 0)).any() else 0.0,
                       ncon_p50=int(np.median(ncon[cmask])) if cmask.any() else 0,
                       slip_mm=round(slip * 1000, 2) if slip == slip else None,
                       tilt_max=round(float(tilt.max()), 1), z_end=round(float(zc[-1]), 4),
                       mimic_max=round(float(mim[cmask].max()), 4) if cmask.any() else None,
                       asym_max=round(float(asym[cmask].max()), 4) if cmask.any() else None,
                       gap_min_mm=round(float(gap[cmask].min()) * 1000, 2) if cmask.any() else None,
                       applied={k: v for k, v in applied.items() if k != 'sol_timeconst_min'})
            rows.append(row)
            if verbose:
                print(f"[bench] {cname:14s} uid={u} pick={int(row['picked'])}/{int(row['hard'])} "
                      f"f_cmd={row['f_cmd']} f_stall={row['f_stall']} pen_geom={row['pen_g_max']}mm "
                      f"pen_solver={row['pen_s_max']}mm F={row['F_max']}N slip={row['slip_mm']}mm "
                      f"mimic={row['mimic_max']} gap={row['gap_min_mm']}mm "
                      f"tilt={row['tilt_max']} ({row['seconds']}s)", flush=True)
    out = REPO / OUT_ROOT / '_bench'; out.mkdir(parents=True, exist_ok=True)
    tag = out_tag or time.strftime('%H%M%S')
    (out / f'bench_{tag}.json').write_text(json.dumps(dict(base=base, uids=list(map(int, uids)), rows=rows), indent=1))
    bench_table(rows)
    return rows


def _tilt(q):
    w_, x, y, z = [float(v) for v in q]
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))


def bench_table(rows):
    import collections
    by = collections.OrderedDict()
    for r in rows: by.setdefault(r['cfg'], []).append(r)
    print(f"\n{'cfg':16s} {'pick':>5s} {'hard':>5s} {'pen_geom':>9s} {'pen_solv':>9s} {'F_max':>7s} "
          f"{'F_p50':>7s} {'slip':>6s} {'f_stall':>7s} {'tilt':>5s} {'mimic':>6s} {'gap':>6s}   (median over uids)")
    for c, rs in by.items():
        A = lambda k: np.array([r[k] for r in rs if r.get(k) is not None], float)
        print(f"{c:16s} {sum(r['picked'] for r in rs):2d}/{len(rs):<2d} {sum(r['hard'] for r in rs):2d}/{len(rs):<2d} "
              f"{np.median(A('pen_g_max')):9.2f} {np.median(A('pen_s_max')):9.2f} {np.median(A('F_max')):7.1f} "
              f"{np.median(A('F_p50')):7.1f} {np.median(A('slip_mm')):6.2f} {np.median(A('f_stall')):7.3f} "
              f"{np.median(A('tilt_max')):5.1f} {np.median(A('mimic_max')):6.3f} {np.median(A('gap_min_mm')):6.2f}")


# --------------------------------------------------------------------------- child bootstrap
def _boot(spec):
    """Command line for a child process that registers the gripper configs FIRST and then runs
    one of the host tools' run_shard.  (The host tools' own --parallel would re-exec themselves
    without the registration, which is why sharding is driven from here.)"""
    return [sys.executable, '-c',
            'import sys; sys.path.insert(0, %r); import gripper_lab as G; G._child(%r)'
            % (str(REPO / 'baselines'), json.dumps(spec))]


def _child(spec_json):
    spec = json.loads(spec_json)
    register_all()
    tool = spec.pop('tool')
    if tool == 'pick':
        import human_follower_lab as H
        H.run_shard(argparse.Namespace(**spec))
    elif tool == 'full':
        import fulltask_fidelity_lab as F
        F.run_shard(argparse.Namespace(**spec))
    elif tool == 'fid':
        import sim_fidelity_lab as S
        S.run_shard(argparse.Namespace(**spec))
    elif tool == 'negctl':
        import record_demos as RD
        sys.argv = ['record_demos.py'] + spec['argv']
        RD.main()
    else:
        raise SystemExit(f'unknown child tool {tool}')


def _spawn(cmds, parallel, logs):
    pend = list(zip(cmds, logs)); run = []
    while pend or run:
        while pend and len(run) < parallel:
            cmd, lg = pend.pop(0)
            fh = open(lg, 'w')
            run.append((subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT), fh, lg))
        time.sleep(1.5)
        still = []
        for p, fh, lg in run:
            if p.poll() is None: still.append((p, fh, lg))
            else:
                fh.close()
                if p.returncode != 0: print(f'[gl] CHILD FAILED rc={p.returncode} log={lg}', flush=True)
        run = still


# --------------------------------------------------------------------------- (a) pick scope
def run_pick(cfg, uids=None, parallel=3, fresh=True, follower=PICK_FOLLOWER, src=SRC_DEFAULT):
    register_all()
    import human_follower_lab as H
    outdir = f'{H.LAB_ROOT}/{follower}@{cfg}'
    out = REPO / outdir; out.mkdir(parents=True, exist_ok=True)
    for old in glob.glob(str(out / 'manifest_*.json')): os.remove(old)
    uids = uids or sorted(int(pl.Path(p).stem) for p in glob.glob(str(REPO / src / '*.npz')))
    base = dict(tool='pick', config=follower, sim=cfg, override=None, src=src, outdir=outdir,
                images=False, trace=False, shard_idx=0, shard_n=1, uids=None,
                rollout_base=H.ROLLOUT_BASE, fresh_tag=None)
    cmds, logs = [], []
    if fresh:                                   # ONE PROCESS PER EPISODE (no solver residue)
        for u in uids:
            s = dict(base, uids=[int(u)], fresh_tag=f'u{u}', rollout_base=H.ROLLOUT_BASE + 10 * (u - 200))
            cmds.append(_boot(s)); logs.append(out / f'u{u}.log')
    else:
        for i in range(parallel):
            s = dict(base, uids=[int(x) for x in uids], shard_idx=i, shard_n=parallel)
            cmds.append(_boot(s)); logs.append(out / f'shard{i}.log')
    t0 = time.time(); _spawn(cmds, parallel, logs)
    s = H.merge_cfg(None, outdir)
    print(f"[gl] PICK {cfg}: kept {s['n_kept']}/{s['n']} in {time.time()-t0:.0f}s  dropped="
          + ' '.join(f'{u}:{o}' for u, o in s['dropped']), flush=True)
    return s


# --------------------------------------------------------------------------- (b)(c) full task
def run_full(cfg, label='success', parallel=3, uids=None):
    register_all()
    import fulltask_fidelity_lab as F
    out = REPO / F.OUT_ROOT / cfg; out.mkdir(parents=True, exist_ok=True)
    for old in out.glob(f'manifest_{label}_shard*.json'): old.unlink()
    cmds, logs = [], []
    for i in range(parallel):
        s = dict(tool='full', variant=cfg, label=label, uids=uids, shard_idx=i, shard_n=parallel)
        cmds.append(_boot(s)); logs.append(out / f'{label}_shard{i}.log')
    t0 = time.time(); _spawn(cmds, parallel, logs)
    recs = F.merge(cfg, label); s = F.summarize(recs)
    print(f'[gl] FULL {cfg} ({label}) in {time.time()-t0:.0f}s: ' + json.dumps(s), flush=True)
    return s


# --------------------------------------------------------------------------- (d) arm fidelity
def run_fid(cfg, parallel=3, uids=None, src=SRC_DEFAULT):
    register_all()
    import sim_fidelity_lab as S
    out = REPO / S.OUT_ROOT / cfg; out.mkdir(parents=True, exist_ok=True)
    for old in out.glob('manifest_shard*.json'): old.unlink()
    cmds, logs = [], []
    for i in range(parallel):
        s = dict(tool='fid', variant=cfg, src=src, uids=uids, shard_idx=i, shard_n=parallel)
        cmds.append(_boot(s)); logs.append(out / f'shard{i}.log')
    t0 = time.time(); _spawn(cmds, parallel, logs)
    m = S.merge(cfg)
    print(f'[gl] FID {cfg} in {time.time()-t0:.0f}s: ' + json.dumps(m), flush=True)
    return m


# --------------------------------------------------------------------------- (e) negative control
def run_negctl(cfg, n=30, parallel=3, seed=0):
    """Random teacher on demo ICs: must keep ~0 (record_demos itself asserts a 1/30 ceiling)."""
    register_all()
    outdir = f'{OUT_ROOT}/negctl_{cfg}'
    (REPO / outdir).mkdir(parents=True, exist_ok=True)
    cmds, logs = [], []
    for i in range(parallel):
        argv = ['--teacher', 'random', '--ic-mode', 'demo', '--n', str(n), '--no-images',
                '--outdir', outdir, '--sim-variant', cfg, '--seed', str(seed),
                '--shard-idx', str(i), '--shard-n', str(parallel)]
        cmds.append(_boot(dict(tool='negctl', argv=argv))); logs.append(REPO / outdir / f'negctl{i}.log')
    _spawn(cmds, parallel, logs)
    kept = len(glob.glob(str(REPO / outdir / '*.npz')))
    print(f'[gl] NEGCTL {cfg}: kept {kept}/{n} (must be ~0)', flush=True)
    return dict(cfg=cfg, n=n, kept=kept)


# --------------------------------------------------------------------------- report
def report():
    register_all()
    import human_follower_lab as H, fulltask_fidelity_lab as F, sim_fidelity_lab as S
    print('== (a) PICK-SCOPE RECREATION  (human_follower_lab --config %s, kept /66)' % PICK_FOLLOWER)
    root = REPO / H.LAB_ROOT
    for d in sorted(root.iterdir()):
        if not d.is_dir() or '@' not in d.name or d.name.endswith('_fails'): continue
        cfg = d.name.split('@', 1)[1]
        s = H.merge_cfg(None, f'{H.LAB_ROOT}/{d.name}')
        if s is None: continue
        print(f"  {d.name:34s} kept {s['n_kept']:2d}/{s['n']:<2d} dil50 {s['dil_p50']:.3f} "
              f"dev50 {s['dev_max_p50']:.4f}  dropped " + ' '.join(f'{u}' for u, _o in s['dropped']))
    print('\n== (b)(c) FULL TASK'); F.report()
    print('\n== (d) ARM FIDELITY vs the REAL joint stream'); S.report()
    print('\n== bench runs')
    for f in sorted((REPO / OUT_ROOT / '_bench').glob('bench_*.json')) if (REPO / OUT_ROOT / '_bench').is_dir() else []:
        b = json.loads(f.read_text()); print(f'--- {f.name} uids={b["uids"]}'); bench_table(b['rows'])


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('list')
    p = sub.add_parser('bench'); p.add_argument('--cfg', nargs='+', required=True)
    p.add_argument('--uids', type=int, nargs='*', default=BENCH_UIDS); p.add_argument('--tag', default='')
    p.add_argument('--hold', type=int, default=40)
    p = sub.add_parser('pick'); p.add_argument('--cfg', required=True); p.add_argument('--uids', type=int, nargs='*', default=None)
    p.add_argument('--parallel', type=int, default=3); p.add_argument('--no-fresh', action='store_true')
    p.add_argument('--follower', default=PICK_FOLLOWER)
    p = sub.add_parser('full'); p.add_argument('--cfg', required=True); p.add_argument('--label', default='success', choices=['success', 'fail'])
    p.add_argument('--parallel', type=int, default=3); p.add_argument('--uids', type=int, nargs='*', default=None)
    p = sub.add_parser('fid'); p.add_argument('--cfg', required=True); p.add_argument('--parallel', type=int, default=3)
    p.add_argument('--uids', type=int, nargs='*', default=None)
    p = sub.add_parser('negctl'); p.add_argument('--cfg', required=True); p.add_argument('--n', type=int, default=30)
    p.add_argument('--parallel', type=int, default=3)
    p = sub.add_parser('report')
    a = ap.parse_args()
    if a.cmd == 'list':
        for k, v in CFGS.items():
            print(f"{k:22s} {v['note'] or ''}\n{'':22s} " + json.dumps({x: y for x, y in v.items() if y != DEF[x] and x != 'note'}))
        return
    if a.cmd == 'bench': register_all(); bench(a.cfg, a.uids, out_tag=a.tag, hold=a.hold); return
    if a.cmd == 'pick': run_pick(a.cfg, a.uids, a.parallel, not a.no_fresh, a.follower); return
    if a.cmd == 'full': run_full(a.cfg, a.label, a.parallel, a.uids); return
    if a.cmd == 'fid': run_fid(a.cfg, a.parallel, a.uids); return
    if a.cmd == 'negctl': run_negctl(a.cfg, a.n, a.parallel); return
    if a.cmd == 'report': report(); return


if __name__ == '__main__':
    main()
