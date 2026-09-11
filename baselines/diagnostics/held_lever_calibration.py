#!/usr/bin/env python3
"""Calibrate HELD_LEVER_M (baselines/stage_predicates.py) on human demonstration tapes.

LADDER_UNIFY_BRIEF_2026-09-10 D3: `in_hand` is |tool_xy - can_xy| < HELD_LEVER, with NO gripper
term. The brief starts it at 2.5 cm and asks Lane 1 for the measured separation between

  HELD frames      -- the can is in the gripper (SLIDE_ANATOMY: ~1.5 cm grasp lever), and
  PUSH-CONTACT     -- a fist pushing the can, which touches its SURFACE, so the tool point
  frames              cannot be closer to the axis than the can radius, 3.3 cm.

The point of the exercise is that BOTH populations are labelled WITHOUT using the lever and
WITHOUT using the gripper command, so the measurement is not circular:

  held        the can is more than LIFT_MARGIN above whatever it could be resting on. Nothing
              in this world holds a can in the air except the gripper. The two resting heights
              (table, shelf) are read off the data, not taken from replay_harness's constants --
              BOTTLE_HEIGHT there is 0.075 and stale (the world can is ~0.101).
  push        the can is ON a support (within REST_TOL of its resting height), it moved
              goalward by at least MIN_STEP_M on this step, and the tool is on the far side of
              the can along the can->goal line. A can on a surface that advances toward the goal
              with the tool behind it is being pushed, whatever the fingers are doing.

usage:
  python3 baselines/diagnostics/held_lever_calibration.py [--cohort dec18_timestamp] [--json out.json]
"""
import argparse
import glob
import json
import math
import os
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
from stage_predicates import HELD_LEVER_M, tilt_deg  # noqa: E402

ROOT = REPO / 'paper' / 'eef_recovery_2026-09-09' / 'slide_metric_of_record'

# world geometry (can_pos_recovery/replay_harness.py + baselines/sim_variants.py)
BOX_POS, BOX_SIZE = (0.75, -0.1875, 0.05), (0.4, 0.75, 0.12)
SHELF_X = (BOX_POS[0] - BOX_SIZE[0] / 2, BOX_POS[0] + BOX_SIZE[0] / 2)      # 0.55 .. 0.95
SHELF_Y = (BOX_POS[1] - BOX_SIZE[1] / 2, BOX_POS[1] + BOX_SIZE[1] / 2)
CAN_RADIUS = 0.033          # can diameter 0.066 (NESTED_TOUCH_DIST = 0.066 + 0.015)

LIFT_MARGIN = 0.030         # m above a support before the can is certainly airborne
REST_TOL = 0.010            # m: "on" a support
MIN_STEP_M = 0.0005         # m of goalward progress in one 120 ms sample = a real push
UPRIGHT_DEG = 20.0


def load_cohort(name):
    d = ROOT / name / 'adapted'
    if not d.is_dir():
        d = ROOT / name
    fs = [f for f in sorted(glob.glob(str(d / '*.npz'))) if 'legacy_tip_proxy' not in f]
    out = []
    for f in fs:
        z = np.load(f, allow_pickle=True)
        s = z['states'].astype(np.float64)
        out.append(dict(uid=int(z['uid']) if 'uid' in z.files else int(pl.Path(f).stem),
                        can=s[:, 8:11], quat=s[:, 11:15], goal=s[:, 15:17],
                        tool=z['eef_pos'].astype(np.float64)[:len(s)],
                        grip=s[:, 6], path=f))
    return out


def rest_heights(tapes):
    """The two resting can-centre heights, read off the data (bimodal histogram)."""
    zs = np.concatenate([t['can'][:, 2] for t in tapes])
    tilts = np.concatenate([np.array([tilt_deg(q) for q in t['quat']]) for t in tapes])
    up = tilts < UPRIGHT_DEG
    table = float(np.median(zs[up & (zs > 0.07) & (zs < 0.15)]))
    shelf = float(np.median(zs[up & (zs > 0.18) & (zs < 0.26)]))
    return table, shelf


def label_frames(t, table_z, shelf_z):
    can, tool, goal = t['can'], t['tool'], t['goal']
    n = len(can)
    tilt = np.array([tilt_deg(q) for q in t['quat']])
    in_fp = ((can[:, 0] > SHELF_X[0]) & (can[:, 0] < SHELF_X[1])
             & (can[:, 1] > SHELF_Y[0]) & (can[:, 1] < SHELF_Y[1]))
    support = np.where(in_fp, shelf_z, table_z)
    lever = np.hypot(tool[:, 0] - can[:, 0], tool[:, 1] - can[:, 1])
    lever3 = np.linalg.norm(tool[:, :3] - can[:, :3], axis=1)
    dist = np.hypot(can[:, 0] - goal[:, 0], can[:, 1] - goal[:, 1])

    # amendment (x)'s release test, verbatim in spirit and entirely LEVER-FREE and GRIPPER-FREE:
    # the can holds still (< STILL_M) over HOLD_N samples while the tool moves away (> AWAY_M).
    # A can that does not follow a moving tool is not attached to it.
    STILL_M, AWAY_M, HOLD_N = 0.002, 0.010, 10
    released_at = None
    for t0 in range(max(n - HOLD_N, 0)):
        w = slice(t0, t0 + HOLD_N)
        if np.max(np.linalg.norm(can[w, :2] - can[t0, :2], axis=1)) > STILL_M:
            continue
        if np.max(np.linalg.norm(tool[w, :3] - tool[t0, :3], axis=1)) < AWAY_M:
            continue
        released_at = t0
        break
    held = (can[:, 2] > support + LIFT_MARGIN) & (tilt < 60.0)
    # post_release: at or after the release, with no airborne frame since (a re-lift would mean
    # the can went back into the gripper). Uses only can z and can-vs-tool motion -- no lever.
    post_release = np.zeros(n, bool)
    if released_at is not None:
        post_release[released_at:] = True
        relift = np.where(held[released_at:])[0]
        if len(relift):
            post_release[released_at + relift[0]:] = False
    on_support = np.abs(can[:, 2] - support) <= REST_TOL
    step_gain = np.zeros(n)
    step_gain[1:] = dist[:-1] - dist[1:]
    v = goal - can[:, :2]
    w = tool[:, :2] - can[:, :2]
    dot = v[:, 0] * w[:, 0] + v[:, 1] * w[:, 1]
    push = on_support & (step_gain >= MIN_STEP_M) & (dot < 0.0) & (tilt < 60.0)
    return dict(lever=lever, lever3=lever3, held=held, push=push, tilt=tilt,
                dist=dist, dot=dot, on_support=on_support, step_gain=step_gain,
                post_release=post_release, released_at=released_at,
                push_clean=push & post_release)


def pct(a, ps=(1, 5, 25, 50, 75, 95, 99)):
    return {f'p{p}': (float(np.percentile(a, p)) if len(a) else math.nan) for p in ps}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cohort', default='dec18_timestamp')
    ap.add_argument('--json', default=None)
    a = ap.parse_args()

    tapes = load_cohort(a.cohort)
    assert tapes, f'no tapes under {ROOT / a.cohort}'
    table_z, shelf_z = rest_heights(tapes)
    print(f'cohort {a.cohort}: {len(tapes)} tapes, '
          f'{sum(len(t["can"]) for t in tapes)} frames (120 ms decisions)')
    print(f'  resting can-centre height, measured: table {table_z * 100:.2f} cm, '
          f'shelf {shelf_z * 100:.2f} cm  (difference {(shelf_z - table_z) * 100:.2f} cm '
          f'= the shelf riser)')
    print(f'  held  = can z > support + {LIFT_MARGIN * 100:.0f} cm')
    print(f'  push  = can on a support (+-{REST_TOL * 100:.0f} cm), goalward step '
          f'>= {MIN_STEP_M * 1000:.1f} mm, tool on the far side')

    H, P, C = [], [], []
    per_uid = []
    for t in tapes:
        lab = label_frames(t, table_z, shelf_z)
        h, p = lab['lever'][lab['held']], lab['lever'][lab['push']]
        c = lab['lever'][lab['push_clean']]
        H.append(h); P.append(p); C.append(c)
        per_uid.append(dict(uid=t['uid'], n=int(len(t['can'])), n_held=int(lab['held'].sum()),
                            n_push=int(lab['push'].sum()), n_push_clean=int(len(c)),
                            released_at=(None if lab['released_at'] is None
                                         else int(lab['released_at'])),
                            held_med=(float(np.median(h)) if len(h) else None),
                            push_med=(float(np.median(p)) if len(p) else None),
                            push_clean_med=(float(np.median(c)) if len(c) else None)))
    H = np.concatenate(H) if H else np.array([])
    P = np.concatenate(P) if P else np.array([])
    C = np.concatenate(C) if C else np.array([])
    print(f'\nHELD frames      n = {len(H):6d}  over {sum(1 for r in per_uid if r["n_held"]):2d}/'
          f'{len(tapes)} tapes')
    print('   xy lever cm  ' + '  '.join(f'{k} {v * 100:6.2f}' for k, v in pct(H).items()))
    print(f'PUSH-CONTACT     n = {len(P):6d}  over {sum(1 for r in per_uid if r["n_push"]):2d}/'
          f'{len(tapes)} tapes')
    print('   xy lever cm  ' + '  '.join(f'{k} {v * 100:6.2f}' for k, v in pct(P).items()))

    print(f'PUSH, RELEASED   n = {len(C):6d}  over {sum(1 for r in per_uid if r["n_push_clean"]):2d}/'
          f'{len(tapes)} tapes   (push frames that are also post-release and not re-lifted;'
          f' the release test is amendment (x)\'s -- lever-free and gripper-free)')
    print('   xy lever cm  ' + '  '.join(f'{k} {v * 100:6.2f}' for k, v in pct(C).items()))

    print('\nSEPARATION at thresholds (held wrongly called free / push wrongly called in_hand).'
          '\n  "push(all)" is the contaminated set; "push(rel)" is the post-release set.')
    rows = []
    for thr in (0.015, 0.020, 0.022, 0.025, 0.028, 0.030, 0.031, CAN_RADIUS, 0.035, 0.040):
        fn = float(np.mean(H >= thr)) if len(H) else math.nan     # held missed
        fp = float(np.mean(P < thr)) if len(P) else math.nan      # push captured as held
        fc = float(np.mean(C < thr)) if len(C) else math.nan
        mark = '  <-- HELD_LEVER_M default' if abs(thr - HELD_LEVER_M) < 1e-9 else (
            '  <-- can radius' if abs(thr - CAN_RADIUS) < 1e-9 else '')
        print(f'  {thr * 100:5.1f} cm   held->free {fn:7.4f} ({int(round(fn * len(H))):5d}/{len(H)})'
              f'   push(all)->in_hand {fp:7.4f} ({int(round(fp * len(P))):4d}/{len(P)})'
              f'   push(rel)->in_hand {fc:7.4f} ({int(round(fc * len(C))):4d}/{len(C)})'
              f'   err(H,rel) {fn + fc:7.4f}{mark}')
        rows.append(dict(thr_m=thr, held_as_free=fn, push_as_held=fp, push_rel_as_held=fc))

    ov = float(np.mean(P <= np.percentile(H, 99))) if len(H) and len(P) else math.nan
    print(f'\n  overlap: {ov:.4f} of push frames sit below the 99th percentile of held levers')
    if len(H) and len(P):
        print(f'  held p99 {np.percentile(H, 99) * 100:.2f} cm  vs  push p1 '
              f'{np.percentile(P, 1) * 100:.2f} cm')

    # ---- the push population is BIMODAL. Test whether its low-lever half is gripped drag. ----
    # A frame is inside a CARRY BRACKET if the lever stays below `brk` continuously from (or to)
    # a frame at which the can is airborne. An airborne can is in the gripper by physics; if the
    # lever never left the grasp band in between, the gripper never let go. The lever is used
    # only as a continuity test here -- the airborne anchor is what establishes "gripped".
    brk = 0.028      # the measured ceiling of the held population (p99 2.64 cm, 9/5575 above 2.8)
    lo_n = lo_in = hi_n = hi_in = 0
    tapes_with_fist = 0
    fist_lev = []
    for t in tapes:
        lab = label_frames(t, table_z, shelf_z)
        lev, air, push = lab['lever'], lab['held'], lab['push']
        band = lev < brk
        # flood-fill the carry bracket out of every airborne frame along contiguous `band` runs
        carry = np.zeros(len(lev), bool)
        i = 0
        while i < len(lev):
            if band[i]:
                j = i
                while j + 1 < len(lev) and band[j + 1]:
                    j += 1
                if air[i:j + 1].any():
                    carry[i:j + 1] = True
                i = j + 1
            else:
                i += 1
        lowp = push & (lev < brk)
        hip = push & (lev >= CAN_RADIUS)
        lo_n += int(lowp.sum()); lo_in += int((lowp & carry).sum())
        hi_n += int(hip.sum()); hi_in += int((hip & carry).sum())
        if hip.any():
            tapes_with_fist += 1
            fist_lev.append(lev[hip])
    print(f'\n  push frames with lever < {brk * 100:.1f} cm: {lo_n} of which '
          f'{lo_in} ({lo_in / max(lo_n, 1):.3f}) lie inside a CARRY BRACKET '
          f'(a lever-continuous run containing an airborne frame) -> gripped drag, not a fist push')
    print(f'  push frames with lever >= {CAN_RADIUS * 100:.1f} cm (geometrically impossible for a '
          f'tool point inside the can): {hi_n}, of which {hi_in} '
          f'({hi_in / max(hi_n, 1):.3f}) inside a carry bracket; present in '
          f'{tapes_with_fist}/{len(tapes)} tapes')
    if fist_lev:
        F = np.concatenate(fist_lev)
        print('   fist-contact lever cm  ' + '  '.join(f'{k} {v * 100:6.2f}'
                                                       for k, v in pct(F).items()))

    if a.json:
        pl.Path(a.json).write_text(json.dumps(dict(
            cohort=a.cohort, tapes=len(tapes), table_rest_z=table_z, shelf_rest_z=shelf_z,
            lift_margin=LIFT_MARGIN, rest_tol=REST_TOL, min_step_m=MIN_STEP_M,
            held_n=int(len(H)), push_n=int(len(P)),
            held_pct=pct(H), push_pct=pct(P), push_rel_pct=pct(C), push_rel_n=int(len(C)), thresholds=rows, per_uid=per_uid), indent=1))
        print(f'wrote {a.json}')


if __name__ == '__main__':
    main()
