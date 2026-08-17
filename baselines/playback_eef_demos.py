"""CLOSED-LOOP EEF PLAYBACK of the human demos in the cartesian action space.

PLAYBACK ONLY (user directive 2026-08-14): no dataset outputs -- manifests and
census tables only. Confidence gates before any recording is considered.

WHY: the demos were teleoperated in 4-DOF task space (joystick ee-velocity
x,y,z,pitch + grip). A 4-DOF action space is both a large action-space cut and
minimal distribution shift w.r.t. the original demonstrations. The open question:
the real rig's Jacobian controller left roll/yaw FREE (null-space drift, measured
up to 1.03/1.34 rad; cartesian_env.py lines 79-91), so a sim that resolves the
uncontrolled DOFs differently grasps at a different yaw. Hence TWO variants:

  6dof  -- waypoints carry FK position AND orientation; control='delta6' pins the
           wrist to the demonstrated orientation. PRIMARY-fidelity: no IK bet.
  4dof  -- waypoints carry position + pitch only; control='delta' (measured-ref
           ee-delta, self-correcting). MS-parity ablation: roll/yaw resolved by
           the sim's IK, exactly the bet the real rig made with ITS IK.

The per-stage census delta 6dof-vs-4dof IS the number that answers "does 4-DOF
suffice" (gate c).

DESIGN mirrors baselines/rl/rerecord_delta_demos.py (joint-space, recovered
61/55/28/20) and derive_cartesian_realized.py (FK->tool math):

  FK PASS (--fk-pass): for every frame of every demo in episodes_all (GROUND
  TRUTH -- never episodes_cartesian_*, whose _dual source has a documented
  provenance defect), set the arm to the recorded qpos, RE-ISSUE the controller
  targets to that same pose (P2 lesson: set_dofs_position alone leaves stale PD
  targets and the settle step fights the read), zero velocity, one scene.step,
  read the wrist pose. tool = wrist + R_wrist @ offset_local, offset calibrated
  at frame 0 == HARDCODED_START (the pose REF_TOOL_AT_START was measured at;
  per-demo frame-0 calibration error recorded -- must be ~0 for all demos).
  Output: waypoint cache npz per uid (scratchpad, not a deliverable):
  wp_pos(n,3), wp_rv(n,3 rotvec rel. frame-0 orientation), grip(n).

  PITCH CONVENTION GATE (fk pass prints it): the demo pitch envelope is
  documented as [-0.55,+0.30] rad (asymmetric). The FK rel-rotvec Y component
  across demos must fall in that envelope, not its mirror -- confirms dim + sign
  before any census.

  FOLLOWER (one env step = one decision):
    waypoint j = FK tool pose of states[j+1] (the pose the demo ARRIVED at),
    grip_j = actions[j, 6] (recorded 0..1 command).
      a_pos   = wp_pos[j] - tool_meas          (env clips to DCAP=0.01)
      6dof:  a_rot   = wp_rv[j] - rv_meas      (env clips to DROT_CAP=0.06)
      4dof:  a_pitch = wp_rv[j][1] - pitch_meas (env clips to DPITCH_CAP)
    advance j when |tool_meas - wp_pos[j]|_inf < TOL_POS (and, 6dof, rotvec
    within TOL_ROT; 4dof, pitch within TOL_PITCH); dwell >= MAX_DWELL skips the
    waypoint (counted). Time dilation reported. On-track the follower advances
    one waypoint per step = demo speed; off-track it corrects -- which an
    open-loop tape cannot (P1).

  SCORING = collector semantics, mirroring rerecord_delta_demos exactly:
  picked/placed/contact are GenesisCanEnv's own honest predicates snapshotted
  before the settle; nested ONLY settled (hold last waypoint 100 steps, then
  picked AND xy<=NESTED_TOUCH_DIST AND both cans upright). Proxy never reported.

KNOWN TRAPS HANDLED (work order 2026-08-14):
  * control mode passed EXPLICITLY on every construction (silent 'vel' default
    started the control-mode bug family).
  * #26 inner horizon: CartesianCanEnv defaults max_steps=1200 into
    GenesisCanEnv; past it every step runs _nested() = 100 phantom sim steps.
    VCAP playback needs ~1800 steps. We pass max_steps=10**9 and bound episodes
    ourselves.
  * FK calibration at HARDCODED_START only (reset pose), never mid-episode.
  * P2 residue: sequential episodes share solver-cache state even post-fix.
    Census aggregate is the signal; shard split recorded in the manifest;
    single marginal demos prove nothing.

Usage:
  playback_eef_demos.py --fk-pass                     # waypoint cache + pitch gate
  playback_eef_demos.py --variant 6dof --uids 308 325 297 326 265   # gate (a)
  playback_eef_demos.py --variant 6dof --shard-idx 0 --shard-n 8    # census
  playback_eef_demos.py --merge                       # census tables, both variants
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

SCRATCH = pl.Path(os.environ.get(
    'PLAYBACK_SCRATCH',
    '/tmp/claude-1000/-home-j-workspace-genesis-pickaplace/'
    '85fd3758-abf4-4299-a3e2-48a960c5be6c/scratchpad/eef_playback'))

DEMO_DIR = 'baselines/episodes_all'          # ground truth, per work order
STAGE_ORDER = ['no-pick', 'picked', 'placed', 'contact', 'nested']
PITCH_ENVELOPE = (-0.55, 0.30)               # documented demo envelope (cartesian_env.py:60)

# TOL_POS: one DCAP (the per-step drive authority, 3.6x the demo per-frame motion
# of VCAP*DT=2.75mm) -- the finest position the delta space commands in one step.
DEFAULT_TOL_POS = 0.01
DEFAULT_TOL_ROT = 0.06                        # one DROT_CAP (rad, per-axis inf-norm)
DEFAULT_TOL_PITCH = 0.075                     # one DPITCH_CAP
DEFAULT_MAX_DWELL = 8
DEFAULT_DILATION_CAP = 3.0
DEFAULT_SETTLE = 100


def _np(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def _gs_to_xyzw(q):
    q = np.asarray(q, float)
    return np.array([q[1], q[2], q[3], q[0]])


# ---------------------------------------------------------------- FK pass ----
def fk_pass(args):
    """Waypoint cache: FK tool pose per frame per demo, via the sim's own model."""
    from cartesian_env import CartesianCanEnv

    SCRATCH.mkdir(parents=True, exist_ok=True)
    env = CartesianCanEnv(backend='cpu', max_steps=10 ** 9, control='delta6')
    genv = env.env
    w = genv.w
    kin, kdofs, eef = w['kinova'], w['kdofs'], w['eef']
    env.reset(uid=int(args.cal_uid))          # calibrates _offset_local at HARDCODED_START
    off = env._offset_local.copy()

    paths = sorted(glob.glob(str(REPO / DEMO_DIR / '*.npz')),
                   key=lambda p: int(pl.Path(p).stem))
    if args.uids:
        want = set(int(u) for u in args.uids)
        paths = [p for p in paths if int(pl.Path(p).stem) in want]
    if args.shard_n > 1:
        paths = paths[args.shard_idx::args.shard_n]

    pitch_lo, pitch_hi = np.inf, -np.inf
    for p in paths:
        d = np.load(p, allow_pickle=True)
        uid = int(d['uid'])
        S = d['states'].astype(np.float64)
        A = d['actions'].astype(np.float64)
        n = len(S)
        pos = np.empty((n, 3)); quat = np.empty((n, 4))
        for i in range(n):
            q6 = S[i, :6]
            kin.set_dofs_position(np.asarray(q6), kdofs[:6])
            # P2 lesson: align the PD targets with the set pose or the step fights it
            kin.control_dofs_position(np.asarray(q6), dofs_idx_local=kdofs[:6])
            kin.zero_all_dofs_velocity()
            w['scene'].step()
            pos[i] = _np(eef.get_pos()); quat[i] = _np(eef.get_quat())
        rot = R.from_quat(np.stack([quat[:, 1], quat[:, 2], quat[:, 3], quat[:, 0]], axis=1))
        tool = pos + rot.apply(np.broadcast_to(off, (n, 3)))
        rel_rv = (rot * rot[0].inv()).as_rotvec()
        cal_err = float(np.linalg.norm(tool[0] - CartesianCanEnv.REF_TOOL_AT_START))
        grip = np.clip(A[:, 6], 0.0, 1.0)
        np.savez_compressed(SCRATCH / f'wp_{uid}.npz',
                            wp_pos=tool.astype(np.float64),
                            wp_rv=rel_rv.astype(np.float64),
                            grip=grip.astype(np.float64),
                            cal_err=cal_err, uid=uid, n=n)
        lo, hi = float(rel_rv[:, 1].min()), float(rel_rv[:, 1].max())
        pitch_lo, pitch_hi = min(pitch_lo, lo), max(pitch_hi, hi)
        print(f'FK uid={uid} n={n} cal_err={cal_err * 1000:.2f}mm '
              f'pitch=[{lo:+.3f},{hi:+.3f}]', flush=True)
    print(f'\nFK PASS DONE shard={args.shard_idx}/{args.shard_n}')
    print(f'PITCH GATE: observed envelope [{pitch_lo:+.3f},{pitch_hi:+.3f}] '
          f'documented {PITCH_ENVELOPE} '
          f'-> {"OK (asymmetry matches)" if abs(pitch_lo) > abs(pitch_hi) else "SUSPECT SIGN FLIP"}')


# --------------------------------------------------------------- playback ----
def play_one(env, variant, uid, tol_pos, tol_rot, tol_pitch, max_dwell,
             dilation_cap, settle, arrival_mode='legacy',
             tol_tight=None, dwell_tight=None, grip_window=0):
    from replay_harness import tilt_deg, NESTED_TOUCH_DIST

    wp_f = SCRATCH / f'wp_{uid}.npz'
    src = np.load(str(REPO / DEMO_DIR / f'{uid}.npz'), allow_pickle=True)
    rec = dict(uid=uid, label=str(src['label']), src_stage=str(src['stage']),
               variant=variant)
    if not wp_f.exists():
        rec.update(status='NO_WAYPOINTS')
        return rec
    wp = np.load(wp_f, allow_pickle=True)
    P, RV, G = wp['wp_pos'], wp['wp_rv'], wp['grip']
    n_src = int(wp['n'])
    # v3: mark waypoints near a GRIP TRANSITION (command change > 0.02). Within
    # grip_window of one, arrival tightens to tol_tight and dwell patience rises
    # to dwell_tight -- the gripper must close at the demonstrated pose, not
    # 25mm off it. Elsewhere the v2 rules hold (speed where precision is free).
    if grip_window > 0:
        chg = np.where(np.abs(np.diff(G)) > 0.02)[0]
        trans = np.zeros(n_src, bool)
        for c in chg:
            trans[max(0, c - grip_window):min(n_src, c + grip_window + 1)] = True
    else:
        trans = np.zeros(n_src, bool)
    rec.update(n_src_frames=n_src, cal_err_mm=float(wp['cal_err']) * 1000)
    if n_src < 3:
        rec.update(status='TOO_SHORT')
        return rec

    t0 = time.time()
    try:
        env.reset(uid=uid)
    except Exception as e:
        rec.update(status=f'UNRESETTABLE:{type(e).__name__}')
        return rec

    genv = env.env

    def rv_meas():
        rel = (R.from_quat(_gs_to_xyzw(_np(env.eef.get_quat())))
               * R.from_quat(_gs_to_xyzw(env._q0)).inv())
        return rel.as_rotvec()

    def act_toward(j):
        cur = env._tool_pos()
        a_pos = P[j] - cur                     # env clips to DCAP
        if variant == '6dof':
            a_rot = RV[j] - rv_meas()          # env clips to DROT_CAP
            return np.concatenate([a_pos, a_rot, [G[max(j - 1, 0)]]]), cur
        dp = RV[j][1] - env._measured_pitch()  # env clips to DPITCH_CAP
        return np.concatenate([a_pos, [dp], [G[max(j - 1, 0)]]]), cur

    def arrived(j, cur):
        tp = tol_tight if (tol_tight is not None and trans[j]) else tol_pos
        if float(np.max(np.abs(cur - P[j]))) >= tp:
            return False
        if arrival_mode == 'pos':
            # v2 (user-approved 2026-08-14): SYMMETRIC arrival across variants --
            # position-only. v1's per-variant orientation gates made 6dof's advance
            # stricter (full rotvec vs pitch-only), confounding the 4v6 delta with
            # arrival strictness. Orientation is still DRIVEN every step in both
            # variants; it just no longer gates waypoint advance, so the delta
            # isolates orientation CONTROL.
            return True
        if variant == '6dof':
            return float(np.max(np.abs(rv_meas() - RV[j]))) < tol_rot
        return abs(env._measured_pitch() - RV[j][1]) < tol_pitch

    step_cap = int(np.ceil(dilation_cap * n_src))
    j, dwell, stalls, steps = 1, 0, 0, 0
    lags = []
    while j < n_src and steps < step_cap:
        a, _cur = act_toward(j)
        env.step(a)
        steps += 1
        cur = env._tool_pos()
        lags.append(float(np.linalg.norm(cur - P[j])))
        if arrived(j, cur):
            j += 1
            dwell = 0
        else:
            dwell += 1
            dw_cap = dwell_tight if (dwell_tight is not None and trans[j]) else max_dwell
            if dwell >= dw_cap:
                j += 1
                dwell = 0
                stalls += 1
    truncated = j < n_src

    picked = bool(genv._picked)
    placed = bool(genv._placed)
    contact = bool(genv._contact)
    for _ in range(settle):                    # closed-loop hold of the last waypoint
        a, _ = act_toward(n_src - 1)
        env.step(a)
    wv = genv.w
    bp_, gp_ = _np(wv['bottle'].get_pos()), _np(wv['goal'].get_pos())
    touch = float(np.hypot(bp_[0] - gp_[0], bp_[1] - gp_[1])) <= NESTED_TOUCH_DIST
    nested = bool(picked and touch and tilt_deg(_np(wv['bottle'].get_quat())) < 20
                  and tilt_deg(_np(wv['goal'].get_quat())) < 20)
    stage = ('nested' if nested else 'contact' if contact else 'placed' if placed
             else 'picked' if picked else 'no-pick')
    lags = np.asarray(lags)
    rec.update(status='OK', stage=stage, picked=picked, placed=placed,
               contact=contact, settled_nested=nested,
               n_steps=int(steps), dilation=float(steps) / float(n_src),
               dwell_stalls=int(stalls), truncated=bool(truncated),
               lag_median_mm=float(np.median(lags)) * 1000,
               lag_p90_mm=float(np.percentile(lags, 90)) * 1000,
               wall_s=round(time.time() - t0, 1))
    return rec


def run(args):
    from cartesian_env import CartesianCanEnv

    assert args.variant in ('6dof', '4dof')
    control = 'delta6' if args.variant == '6dof' else 'delta'
    outdir = SCRATCH / (f'census_{args.variant}' + (f'_{args.tag}' if args.tag else ''))
    outdir.mkdir(parents=True, exist_ok=True)

    paths = sorted(glob.glob(str(REPO / DEMO_DIR / '*.npz')),
                   key=lambda p: int(pl.Path(p).stem))
    uids = [int(pl.Path(p).stem) for p in paths]
    if args.uids:
        want = set(int(u) for u in args.uids)
        uids = [u for u in uids if u in want]
    if args.shard_n > 1:
        uids = uids[args.shard_idx::args.shard_n]
    assert uids, 'no demos selected'

    # control mode EXPLICIT; max_steps huge (#26: never let the inner env's own
    # horizon fire mid-playback -- our loop bounds the episode).
    env = CartesianCanEnv(backend='cpu', max_steps=10 ** 9, control=control)
    assert env.control == control, (env.control, control)
    if args.leash is not None:
        # instance-attribute override (shadows the class constant for THIS env
        # only): more feed-forward lead -- the joint follower's 5x-cap leash is
        # what made it track; cartesian v1/v2 ran 2.5x.
        env.LEASH = float(args.leash)
    print(f'[play] variant={args.variant} control={env.control} '
          f'DCAP={env.DCAP} DROT_CAP={env.DROT_CAP} LEASH={env.LEASH} '
          f'tol_pos={args.tol_pos} shard={args.shard_idx}/{args.shard_n} '
          f'n={len(uids)}', flush=True)

    recs = []
    for u in uids:
        rec = play_one(env, args.variant, u, args.tol_pos, args.tol_rot,
                       args.tol_pitch, args.max_dwell, args.dilation_cap,
                       args.settle, arrival_mode=args.arrival_mode,
                       tol_tight=args.tol_tight, dwell_tight=args.dwell_tight,
                       grip_window=args.grip_window)
        recs.append(rec)
        print('PLAY ' + ' '.join(f'{k}={v}' for k, v in rec.items()), flush=True)
    man = outdir / f'_manifest_shard{args.shard_idx}of{args.shard_n}.json'
    man.write_text(json.dumps(dict(
        config=dict(variant=args.variant, control=control, demo_dir=DEMO_DIR,
                    tol_pos=args.tol_pos, tol_rot=args.tol_rot,
                    tol_pitch=args.tol_pitch, max_dwell=args.max_dwell,
                    dilation_cap=args.dilation_cap, settle=args.settle,
                    arrival_mode=args.arrival_mode, tag=args.tag,
                    tol_tight=args.tol_tight, dwell_tight=args.dwell_tight,
                    grip_window=args.grip_window, leash=args.leash,
                    shard_idx=args.shard_idx, shard_n=args.shard_n,
                    git=__import__('subprocess').run(
                        ['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO),
                        capture_output=True, text=True).stdout.strip()),
        records=recs), indent=1))
    print(f'[play] DONE shard {args.shard_idx}/{args.shard_n} -> {man}', flush=True)


def merge(args):
    rank = {s: i for i, s in enumerate(STAGE_ORDER)}

    def census(variant):
        recs = {}
        d = SCRATCH / (f'census_{variant}' + (f'_{args.tag}' if args.tag else ''))
        for f in sorted(d.glob('_manifest_shard*.json')):
            for r in json.loads(f.read_text())['records']:
                recs[int(r['uid'])] = r
        return recs

    def tbl(rs, key):
        return {s: sum(1 for r in rs if rank[r[key]] >= i)
                for i, s in enumerate(STAGE_ORDER)}

    have = {v: census(v) for v in ('6dof', '4dof') if census(v)}
    assert have, f'no census manifests under {SCRATCH}'
    print(f'{"":12s} {">=picked":>9s} {">=placed":>9s} {">=contact":>9s} {">=nested":>9s}')
    print(f'{"labels":12s} {65:9d} {62:9d} {25:9d} {16:9d}')
    print(f'{"joint-rerec":12s} {61:9d} {55:9d} {28:9d} {20:9d}')
    for v, recs in have.items():
        ok = [r for r in recs.values() if r.get('status') == 'OK']
        unres = [r for r in recs.values()
                 if str(r.get('status', '')).startswith('UNRESETTABLE')]
        c = tbl(ok, 'stage')
        print(f'{"eef-" + v:12s} {c["picked"]:9d} {c["placed"]:9d} '
              f'{c["contact"]:9d} {c["nested"]:9d}   '
              f'(ok={len(ok)} unresettable={len(unres)})')
        if ok:
            dil = np.array([r['dilation'] for r in ok])
            lag = np.array([r['lag_median_mm'] for r in ok])
            print(f'{"":12s} dilation med={np.median(dil):.2f}x p90='
                  f'{np.percentile(dil, 90):.2f}x  lag med={np.median(lag):.1f}mm '
                  f'trunc={sum(1 for r in ok if r["truncated"])} '
                  f'stalls_med={np.median([r["dwell_stalls"] for r in ok]):.0f}')
    if len(have) == 2:
        a, b = have['6dof'], have['4dof']
        both = sorted(set(a) & set(b))
        print('\nper-uid 6dof vs 4dof (differing only):')
        for u in both:
            if a[u].get('stage') != b[u].get('stage'):
                print(f'  {u}: 6dof={a[u].get("stage"):8s} 4dof={b[u].get("stage")}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fk-pass', action='store_true')
    ap.add_argument('--cal-uid', type=int, default=232)
    ap.add_argument('--variant', choices=['6dof', '4dof'], default=None)
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--shard-idx', type=int, default=0)
    ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--tol-pos', type=float, default=DEFAULT_TOL_POS)
    ap.add_argument('--tol-rot', type=float, default=DEFAULT_TOL_ROT)
    ap.add_argument('--tol-pitch', type=float, default=DEFAULT_TOL_PITCH)
    ap.add_argument('--max-dwell', type=int, default=DEFAULT_MAX_DWELL)
    ap.add_argument('--dilation-cap', type=float, default=DEFAULT_DILATION_CAP)
    ap.add_argument('--settle', type=int, default=DEFAULT_SETTLE)
    ap.add_argument('--merge', action='store_true')
    ap.add_argument('--arrival-mode', choices=['legacy', 'pos'], default='legacy')
    ap.add_argument('--tol-tight', type=float, default=None,
                    help='v3: arrival tol near grip transitions (m)')
    ap.add_argument('--dwell-tight', type=int, default=None,
                    help='v3: dwell patience near grip transitions')
    ap.add_argument('--grip-window', type=int, default=0,
                    help='v3: waypoints around a grip change that use tight rules')
    ap.add_argument('--leash', type=float, default=None,
                    help='v3: override env LEASH (m); None = class default 0.025')
    ap.add_argument('--tag', default=None, help='suffix for census outdir (e.g. v2)')
    args = ap.parse_args()
    if args.merge:
        merge(args)
    elif args.fk_pass:
        fk_pass(args)
    else:
        assert args.variant, '--variant required for playback'
        run(args)


if __name__ == '__main__':
    main()
