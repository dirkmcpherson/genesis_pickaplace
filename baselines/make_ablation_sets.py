#!/usr/bin/env python3
"""A28 burstiness-ablation WAYPOINT SCHEDULES from the frozen corrected-world sets.

PREREG: paper/PREREG_final_round_robin_2026-08-23.md A28 (2026-09-01). Metric screen
that motivated it: paper/WM_METRIC_2026-09-01.md (screening/hypothesis-generating).
The w3-specific human/machine separator is TEMPORAL BURSTINESS (pause_frac, HF action
power, jerk-per-path); every coverage metric separates only the old world. Three
intervention arms test causality (r2dreamer, 4 seeds each, vs the frozen anchors):

  dDPretimed   dDP geometry, human clock: each w3 dDP tape's absolute-target path is
               re-timed to the SAME-IC dH tape's arc-length progress profile (human
               pauses become holds, human bursts become merged deltas), with a
               closure-dwell guard (see below) so the grasp itself keeps its original
               number of decisions. Raises pause_frac/act_hf_frac/jerk toward human.
  dHsmoothed   converse: dH path on a constant-arc-speed clock (same decision count),
               same closure-dwell guard. Lowers the burstiness metrics.
  dDPnoised    DART-style control: i.i.d. Gaussian noise on the dDP WAYPOINTS (arm
               joints only, grip untouched, closure segment untouched), sigma
               calibrated so the predicted action stream's HF power fraction matches
               the measured dH level. Waypoint (not raw-delta) noise is the disclosed
               design choice: with delta_ref='target' open-loop, i.i.d. DELTA noise
               integrates into an unbounded target random walk; waypoint jitter is the
               bounded i.i.d. perturbation that raises bandwidth WITHOUT stop-go
               structure -- exactly the registered intent of the control arm.

DATA (read-only): baselines/matched_w3/{dH,dDP} contract-v1 npz. The schedule is built
from the tape's RECORDED absolute command stream: actions[:, :6] = the env's
_dj_target at each decision-window end (so leash effects during the original
recording are already baked in), actions[:, 6] = absolute grip command in [0,1];
q0 = states[0, :6] (the reset target). Images are never loaded (np.load is lazy).

CLOSURE-DWELL GUARD: warping by arc length compresses the source tape's own pauses to
zero decisions -- including the grasp (grip closes while the arm is still). At the
closure arc position the guard splices in hold decisions replaying the source tape's
own closure-segment grip ramp, so every schedule gives the gripper the same number of
decisions to close as its source tape did. Without it the pick fails for timing
reasons unrelated to the hypothesis; inserted-hold counts are in the manifest.

OUTPUT: <out-root>/sched_{dDPretimed,dHsmoothed,dDPnoised}/<ic_uid>.npz with
  target_q (T,6) f64, grip (T,) f64 in [0,1], ic_uid, src_uid, src_set, transform,
  seed, can_pos/can_quat (frozen tape frame-0, for pose reset of non-resettable ICs)
plus transform_manifest.json (sigma calibration, per-set PREDICTED perfect-tracking
metrics = the pre-flight manipulation check, per-IC bookkeeping). Execution + verify +
contract-v1 recording happen in record_demos.py --teacher sched (open-loop clock);
the REAL manipulation check runs baselines/diagnostics/tape_dynamics_metrics.py on
the recorded sets (A28: reported before any training).

Run (cluster, CPU, seconds):
  python baselines/make_ablation_sets.py --dh baselines/matched_w3/dH \
      --ddp baselines/matched_w3/dDP --out-root baselines/demos_v1/_ablation --seed 0
"""
import argparse
import json
import os
import pathlib as pl
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
REPEAT, DELTA_CAP = 4, 0.025          # contract v1 (record_demos.py asserts these on execution)
GRIP_CLOSED = 0.5                     # absolute grip command threshold (range 0..1)
EPS_MOVE = 2e-3                       # rad: "arm holds still" per-decision displacement
NOISE_CLIP = 0.05                     # rad: per-joint bound on waypoint jitter
SIGMA_GRID = (0.002, 0.003, 0.004, 0.005, 0.006, 0.008, 0.010, 0.014, 0.020)


def load_tape(f):
    z = np.load(f, allow_pickle=True)
    A = np.asarray(z['actions'], np.float64)
    S = np.asarray(z['states'], np.float64)
    E = np.asarray(z['eef_pos'], np.float64)
    return dict(uid=int(z['uid']), ic_uid=int(z['ic_uid']), P=A[:, :6],
                g=np.clip(A[:, 6], 0.0, 1.0), q0=S[0, :6],
                eef_speed=np.linalg.norm(np.diff(E, axis=0), axis=1),  # measured, per decision
                can_pos=S[0, 8:11].copy(), can_quat=S[0, 11:15].copy())


def arc(t):
    """Arc-length knots along the target path: s (T+1,), Q (T+1,6), s[0]=0 at q0."""
    Q = np.vstack([t['q0'][None], t['P']])
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))]), Q


def path_at(s_knots, Q, s_query):
    return np.stack([np.interp(s_query, s_knots, Q[:, i]) for i in range(6)], axis=1)


def closure_seg(t):
    """(tc0, tc1): decisions of the grasp dwell -- from first closed-commanded grip
    while the arm holds still, extended while the arm stays still (cap 40)."""
    idx = np.flatnonzero(t['g'] > GRIP_CLOSED)
    if len(idx) == 0:
        return None
    tc0 = int(idx[0])
    d = np.diff(arc(t)[0])
    tc1 = tc0
    while tc1 + 1 < len(t['g']) and d[tc1 + 1] <= EPS_MOVE and tc1 - tc0 < 40:
        tc1 += 1
    return tc0, tc1


def schedule_from_arcs(src, arcs):
    """Re-time `src` (its own path) to the given per-decision cumulative arc positions
    (monotone, in [0, L_src]). Grip follows path progress; closure-dwell guard splices
    the source's own grasp ramp at the closure arc position."""
    s, Q = arc(src)
    L = s[-1]
    arcs = np.clip(np.asarray(arcs, np.float64), 0.0, L)
    wp = path_at(s, Q, arcs)
    tstar = np.minimum(np.searchsorted(s[1:], arcs, side='left'), len(src['g']) - 1)
    g = src['g'][tstar].copy()
    meta = dict(inserted_hold_decisions=0)
    cs = closure_seg(src)
    if cs is not None:
        tc0, tc1 = cs
        s_c = s[1:][tc0]
        u_c = int(np.searchsorted(arcs, s_c, side='left'))
        need = tc1 - tc0 + 1
        have = int(np.sum(np.abs(arcs[u_c:u_c + 3 * need] - s_c) < 1e-9))
        ins = max(0, need - have)
        if ins:
            wp_c = path_at(s, Q, np.full(ins, s_c))
            ramp = src['g'][tc0:tc0 + ins]
            if len(ramp) < ins:
                ramp = np.concatenate([ramp, np.full(ins - len(ramp), src['g'][tc1])])
            wp = np.concatenate([wp[:u_c], wp_c, wp[u_c:]])
            g = np.concatenate([g[:u_c], ramp, g[u_c:]])
        # everything strictly before the closure point carries the pre-closure command
        pre = src['g'][max(tc0 - 1, 0)] if tc0 > 0 else 0.0
        g[:u_c] = np.minimum(g[:u_c], pre)
        meta['inserted_hold_decisions'] = int(ins)
    return wp, g, meta


def make_retimed(dp, h):
    """dDP path, human clock: human per-decision arc-progress fractions applied to the
    dDP path's total arc length. Human pauses (flat progress) -> repeated waypoints."""
    s_dp, _ = arc(dp)
    s_h, _ = arc(h)
    frac = s_h[1:] / (s_h[-1] or 1.0)
    return schedule_from_arcs(dp, frac * s_dp[-1])


def make_smoothed(h):
    """dH path, constant-speed clock (same decision count as the source tape)."""
    s_h, _ = arc(h)
    T = len(h['g'])
    return schedule_from_arcs(h, np.linspace(s_h[-1] / T, s_h[-1], T))


def make_noised(dp, rng, sigma):
    """dDP schedule + bounded i.i.d. waypoint jitter. Arm joints only; the closure
    segment AND the source's stop/pause decisions are untouched -- jittering a still
    waypoint would DESTROY the tape's pauses (pre-flight v1: pause_frac 0.20 -> 0.07)
    and the control arm must change bandwidth alone, not stop-go structure.
    v2 (manipulation-check regression fix, 2026-09-01): the v1 commanded-arc pause rule
    did NOT protect MEASURED stops -- the recorded v1 arm's strict_stop_frac fell
    0.221 -> 0.127 (jitter settling bleeds into stills; sub-rule creep segments got
    noised). v2 protects decisions where the FROZEN tape's measured EEF speed is below
    2x the strict-stop threshold, unions the commanded rule, and dilates the mask by
    +/-1 decision so the arm enters/leaves stops unjittered."""
    eta = np.clip(rng.normal(0.0, sigma, dp['P'].shape), -NOISE_CLIP, NOISE_CLIP)
    d = np.diff(arc(dp)[0])
    still = (d <= max(EPS_MOVE, 0.2 * float(np.median(d)))) | (dp['eef_speed'] < 0.001)
    still = still | np.roll(still, 1) | np.roll(still, -1)     # +/-1 dilation
    eta[still] = 0.0
    cs = closure_seg(dp)
    if cs is not None:
        eta[cs[0]:cs[1] + 1] = 0.0
    return dp['P'] + eta, dp['g'].copy(), dict(sigma=float(sigma),
                                               noised_frac=round(float(1 - still.mean()), 3))


# ---- predicted (perfect-tracking) metrics: definitions mirror
# baselines/diagnostics/tape_dynamics_metrics.py (hf_frac, pause_frac) but on the
# DESIGNED stream -- used for sigma calibration + pre-flight check only.
def hf_frac(x):
    p = np.abs(np.fft.rfft(x - x.mean())) ** 2
    p = p[1:]
    return float(p[len(p) // 2:].sum() / p.sum()) if p.sum() > 1e-12 else np.nan


def pred_metrics(q0, wp, g):
    dq = np.diff(np.vstack([q0[None], wp]), axis=0)
    a = dq / (REPEAT * DELTA_CAP)
    clip_frac = float((np.abs(a) > 1.0).mean())
    a = np.clip(a, -1.0, 1.0)
    d = np.linalg.norm(dq, axis=1)
    med = np.median(d)
    return dict(
        n=len(wp), clip_frac=round(clip_frac, 4),
        pause_frac=round(float((d < 0.2 * med).mean()), 4) if med > 1e-9 else np.nan,
        act_hf_frac=round(float(np.nanmean([hf_frac(a[:, i]) for i in range(6)])), 4),
        joint_jerk_mean=round(float(np.linalg.norm(np.diff(dq, 2, axis=0), axis=1).mean()), 5)
        if len(wp) > 3 else np.nan)


def measured_hf(tapes_dir):
    """Measured act_hf_frac on a frozen set's RECORDED actions_delta (calibration target)."""
    vals = []
    for f in sorted(pl.Path(tapes_dir).glob('*.npz')):
        a = np.asarray(np.load(f, allow_pickle=True)['actions_delta'], np.float64)
        vals.append(np.nanmean([hf_frac(a[:, i]) for i in range(6)]))
    return float(np.mean(vals))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dh', default='baselines/matched_w3/dH')
    ap.add_argument('--ddp', default='baselines/matched_w3/dDP')
    ap.add_argument('--out-root', dest='out_root', default='baselines/demos_v1/_ablation')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--sigma', type=float, default=None,
                    help='noised: skip calibration, use this waypoint sigma (rad)')
    ap.add_argument('--only', choices=['dDPretimed', 'dHsmoothed', 'dDPnoised'], default=None,
                    help='rebuild just this schedule set (others untouched on disk)')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()

    dh = {t['ic_uid']: t for t in (load_tape(f) for f in sorted((REPO / args.dh).glob('*.npz')))}
    ddp = {t['ic_uid']: t for t in (load_tape(f) for f in sorted((REPO / args.ddp).glob('*.npz')))}
    ics = sorted(set(dh) & set(ddp))
    only = sorted(set(dh) ^ set(ddp))
    print(f'[abl] dH {len(dh)} tapes, dDP {len(ddp)} tapes, common ICs {len(ics)}'
          + (f' (unpaired, skipped: {only})' if only else ''))

    # ---- sigma calibration: predicted noised HF vs measured dH HF
    rng = np.random.default_rng(args.seed)
    target = measured_hf(REPO / args.dh)
    if args.sigma is not None:
        sigma, cal = float(args.sigma), {'mode': 'explicit --sigma'}
    else:
        cal = {'target_dH_act_hf_frac': round(target, 4), 'grid': {}}
        best = None
        for s in SIGMA_GRID:
            r = np.random.default_rng(args.seed + 1)      # same draws across grid points
            hfs = [pred_metrics(ddp[ic]['q0'], *make_noised(ddp[ic], r, s)[:2])['act_hf_frac']
                   for ic in ics]
            m = float(np.mean(hfs))
            cal['grid'][str(s)] = round(m, 4)
            if best is None or abs(m - target) < abs(best[1] - target):
                best = (s, m)
        sigma, cal['chosen'] = best[0], {'sigma': best[0], 'pred_hf': round(best[1], 4)}
        print(f'[abl] sigma calibration: target dH hf {target:.4f} -> sigma {sigma} '
              f'(pred {best[1]:.4f}); grid {cal["grid"]}')

    out = REPO / args.out_root
    sets = {'dDPretimed': [], 'dHsmoothed': [], 'dDPnoised': []}
    if args.only:
        sets = {args.only: []}
    per_ic = []
    for name in sets:
        d = out / f'sched_{name}'
        if d.exists() and not args.force:
            sys.exit(f'FATAL: {d} exists; --force to rebuild')
        d.mkdir(parents=True, exist_ok=True)
        for old in d.glob('*.npz'):
            old.unlink()

    for ic in ics:
        dp, h = ddp[ic], dh[ic]
        built = {'dDPretimed': (dp, *make_retimed(dp, h)),
                 'dHsmoothed': (h, *make_smoothed(h)),
                 'dDPnoised': (dp, *make_noised(dp, rng, sigma))}
        built = {k: v for k, v in built.items() if k in sets}
        row = dict(ic_uid=ic, src_dp_uid=dp['uid'], src_dh_uid=h['uid'],
                   n_dp=len(dp['g']), n_dh=len(h['g']))
        for name, (src, wp, g, meta) in built.items():
            pm = pred_metrics(src['q0'], wp, g)
            sets[name].append(pm)
            row[name] = dict(meta, **pm)
            np.savez_compressed(out / f'sched_{name}' / f'{ic}.npz',
                                target_q=wp, grip=g, ic_uid=ic, src_uid=src['uid'],
                                src_set=('dH' if src is h else 'dDP'), transform=name,
                                seed=args.seed, can_pos=src['can_pos'], can_quat=src['can_quat'])
        per_ic.append(row)

    summary = {name: {k: round(float(np.nanmean([r[k] for r in rows])), 4)
                      for k in ('pause_frac', 'act_hf_frac', 'clip_frac', 'joint_jerk_mean', 'n')}
               for name, rows in sets.items()}
    # frozen-source predicted baselines for the same quantities (apples-to-apples pre-flight)
    for label, pool in (('dH_source_pred', dh), ('dDP_source_pred', ddp)):
        summary[label] = {k: round(float(np.nanmean(
            [pred_metrics(pool[ic]['q0'], pool[ic]['P'], pool[ic]['g'])[k] for ic in ics])), 4)
            for k in ('pause_frac', 'act_hf_frac', 'clip_frac', 'joint_jerk_mean', 'n')}
    man = dict(built=time.strftime('%Y-%m-%dT%H:%M:%S'), seed=args.seed, sigma=sigma,
               sigma_calibration=cal, dh=args.dh, ddp=args.ddp, n_ics=len(ics),
               unpaired_ics=only, prereg='A28', only=args.only,
               predicted_set_means=summary, per_ic=per_ic)
    man_name = f'transform_manifest_{args.only}.json' if args.only else 'transform_manifest.json'
    (out / man_name).write_text(json.dumps(man, indent=1))
    print('[abl] predicted set means (perfect tracking):')
    for k, v in summary.items():
        print(f'   {k}: {v}')
    print(f'[abl] wrote {sum(len(v) for v in sets.values())} schedules -> {out}/sched_* '
          f'+ transform_manifest.json')


if __name__ == '__main__':
    main()
