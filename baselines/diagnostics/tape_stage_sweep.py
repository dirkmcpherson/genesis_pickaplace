#!/usr/bin/env python3
"""Run `baselines/stage_predicates.StageTracker` over the human demonstration tapes and sweep
HELD_LEVER_M. Lane 1, LADDER_UNIFY_BRIEF_2026-09-10.

The question this answers is the one the lever calibration raises: on the human tapes, the
post-release goalward-push frames sit at a MEDIAN lever of about 1.5 cm (people release the can
and then shove it home with the fingers back around it -- amendment (p)). `pushed` accumulates
only on frames where the can is NOT in hand, so a HELD_LEVER of 2.5 cm could refuse to count a
genuine human push. This measures whether it does, at the TAPE level, which is what matters.

THREE SUBSTITUTIONS, all forced by what the adapted tapes store, all disclosed:
  * `picked`     derived: the can has been more than 3 cm above its support at some earlier
                 frame. (The tapes carry no stage flags.)
  * `placed_v2`  derived with the env's own clauses but the MEASURED gripper motor in place of
                 the grip COMMAND (the tapes carry `states[:,6]`, the measured motor, not the
                 command), sustained 3 samples = the env's 10 frames rounded up.
  * contacts     absent from the tapes, so `contact_push` is NOT evaluable here and is reported
                 as unavailable rather than as zero.
Sampling is 120 ms (one decision), so AT_REST_FRAMES is 3 samples, not 12 env frames.

usage: python3 baselines/diagnostics/tape_stage_sweep.py [--cohort dec18_timestamp] [--json out]
"""
import argparse
import json
import os
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'diagnostics'))
from held_lever_calibration import (  # noqa: E402
    LIFT_MARGIN, SHELF_X, SHELF_Y, load_cohort, rest_heights,
)
from stage_predicates import HELD_LEVER_M, StageTracker, tilt_deg  # noqa: E402

PLACE_RELEASE, PLACE_TILT_DEG = 0.45, 20.0        # full_env.FullTaskEnv constants
PLACE_SUSTAIN_SAMPLES = 3                          # 10 env frames at 4 frames/sample, rounded up
AT_REST_SAMPLES = 3                                # AT_REST_FRAMES 12 / 4
BAND_LO, BAND_HI = 0.01, 0.07                      # stage_predicates band, above the shelf TOP
TABLE_TOP_Z = 0.05                                 # replay_harness.TABLE_TOP_Z


def tape_inputs(t, table_z, shelf_z, shelf_top):
    can, tool, goal, grip = t['can'], t['tool'], t['goal'], t['grip']
    n = len(can)
    tilt = np.array([tilt_deg(q) for q in t['quat']])
    in_fp = ((can[:, 0] > SHELF_X[0]) & (can[:, 0] < SHELF_X[1])
             & (can[:, 1] > SHELF_Y[0]) & (can[:, 1] < SHELF_Y[1]))
    support = np.where(in_fp, shelf_z, table_z)
    air = can[:, 2] > support + LIFT_MARGIN
    picked = np.maximum.accumulate(air.astype(np.int8)).astype(bool)
    # the env's own placed_v2 band, above the shelf TOP (not the resting can-centre height)
    ok = (grip < PLACE_RELEASE) & in_fp & (can[:, 2] > shelf_top + BAND_LO) \
        & (can[:, 2] < shelf_top + BAND_HI) & (tilt < PLACE_TILT_DEG) & picked
    run, pv2 = 0, np.zeros(n, bool)
    for i in range(n):
        run = run + 1 if ok[i] else 0
        pv2[i] = run >= PLACE_SUSTAIN_SAMPLES
    pv2 = np.maximum.accumulate(pv2.astype(np.int8)).astype(bool)
    return dict(can=can, quat=t['quat'], goal=goal, tool=tool, grip=grip, tilt=tilt,
                picked=picked, placed_v2=pv2, shelf_top=shelf_top, shelf_z=shelf_z)


def run_tape(inp, lever):
    tr = StageTracker(inp['goal'][0], inp['shelf_top'], held_lever_m=lever,
                      at_rest_frames=AT_REST_SAMPLES)
    nv, per = [], []
    for i in range(len(inp['can'])):
        r = tr.update(can_pos=inp['can'][i], can_quat=inp['quat'][i],
                      goal_pos=np.append(inp['goal'][i], inp['shelf_z']),
                      goal_quat=(1.0, 0.0, 0.0, 0.0),
                      tool_xy=inp['tool'][i, :2], grip_cmd=float(inp['grip'][i]),
                      picked=bool(inp['picked'][i]), placed_v2=bool(inp['placed_v2'][i]),
                      can_goal_contact=False, gripper_goal_contact=False)
        nv.append(r['nested_v2']); per.append(r)
    return tr, np.array(nv, bool), per


def clause_report(inp, per, tr):
    """Which clause of nested_v2 fails on the LAST frame. An absent value is not a zero."""
    i, r = len(per) - 1, per[-1]
    return dict(picked=bool(inp['picked'][i]), released=bool(tr.released),
                within=bool(r['dist_xy_m'] <= 0.081), can_up=bool(r['can_tilt_deg'] < 20.0),
                goal_up=bool(r['goal_tilt_deg'] < 20.0), in_band=bool(r['in_band']),
                free=bool(not r['in_hand']), at_rest=bool(r['at_rest']),
                dist_mm=round(r['dist_xy_m'] * 1000, 1),
                z_cm=round(float(inp['can'][i, 2]) * 100, 2),
                band_cm=(round((inp['shelf_top'] + BAND_LO) * 100, 2),
                         round((inp['shelf_top'] + BAND_HI) * 100, 2)),
                lever_mm=round(r['lever_m'] * 1000, 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cohort', default='dec18_timestamp')
    ap.add_argument('--json', default=None)
    a = ap.parse_args()

    tapes = load_cohort(a.cohort)
    table_z, shelf_z = rest_heights(tapes)
    # the shelf TOP, derived from the two measured resting heights and the known table top:
    # can half-height = table resting centre - table top; shelf top = shelf centre - half-height.
    shelf_top = shelf_z - (table_z - TABLE_TOP_Z)
    ins = [tape_inputs(t, table_z, shelf_z, shelf_top) for t in tapes]
    print(f'  measured rest heights: table {table_z * 100:.2f} cm, shelf {shelf_z * 100:.2f} cm'
          f' -> can half-height {(table_z - TABLE_TOP_Z) * 100:.2f} cm, shelf TOP '
          f'{shelf_top * 100:.2f} cm (the env prints 17.0 cm for gc_kp4_riser3_shelf6)')
    ref = json.loads((REPO / 'paper' / 'eef_recovery_2026-09-09' / 'slide_metric_of_record'
                      / a.cohort / 'results.json').read_text())
    xm = {int(r['uid']): r['metric'] for r in ref['records']}
    print(f'cohort {a.cohort}: {len(tapes)} tapes, 120 ms samples')
    print(f'  amendment (x) TAPE classifier on this cohort: released '
          f'{sum(xm[t["uid"]]["released"] for t in tapes)}  pushed '
          f'{sum(xm[t["uid"]]["pushed"] for t in tapes)}  arrived '
          f'{sum(xm[t["uid"]]["arrived"] for t in tapes)}  slide_success '
          f'{sum(xm[t["uid"]]["slide_success"] for t in tapes)}')
    print(f'  derived placed_v2 (measured motor < {PLACE_RELEASE}, footprint, shelf band, '
          f'tilt < {PLACE_TILT_DEG} deg, {PLACE_SUSTAIN_SAMPLES} samples): '
          f'{sum(int(i["placed_v2"].any()) for i in ins)}/{len(ins)} tapes')
    print(f'  derived picked (can > support + {LIFT_MARGIN * 100:.0f} cm): '
          f'{sum(int(i["picked"].any()) for i in ins)}/{len(ins)} tapes')

    print('\n  HELD_LEVER sweep, TAPE counts (D3 predicates; contact_push NOT evaluable here):')
    rows = []
    for lv in (0.015, 0.020, 0.025, 0.028, 0.030, 0.033, 0.040, 0.060, 1e9):
        rel = psh = nvE = nvL = sl = 0
        gains = []
        for inp in ins:
            tr, nv, _ = run_tape(inp, lv)
            rel += int(tr.released); psh += int(tr.pushed)
            nvE += int(nv[-1]); nvL += int(nv[-AT_REST_SAMPLES:].any())
            sl += int(tr.slide_success)
            gains.append(tr.goalward_gain_m * 1000)
        tag = '  <-- default' if abs(lv - HELD_LEVER_M) < 1e-9 else (
            '  (in_hand disabled)' if lv > 1 else '')
        print(f'    lever {("inf" if lv > 1 else f"{lv * 100:4.1f} cm"):>7s}  released {rel:2d}  '
              f'pushed {psh:2d}  nested_v2 end {nvE:2d} / lastK {nvL:2d}  slide(x) {sl:2d}  '
              f'median gain {np.median(gains):6.1f} mm{tag}')
        rows.append(dict(lever_m=lv, released=rel, pushed=psh, nested_v2_end=nvE,
                         nested_v2_lastk=nvL, slide_x=sl,
                         median_gain_mm=float(np.median(gains))))

    # which tapes the (x) classifier calls slide_success and D3 does not, at the default
    tr_default, clauses = {}, {}
    for t, inp in zip(tapes, ins):
        tr, nv, per = run_tape(inp, HELD_LEVER_M)
        tr_default[t['uid']] = dict(released=tr.released, pushed=tr.pushed,
                                    slide=tr.slide_success,
                                    nested_v2=bool(nv[-AT_REST_SAMPLES:].any()),
                                    gain_mm=round(tr.goalward_gain_m * 1000, 1))
        clauses[t['uid']] = clause_report(inp, per, tr)
    xs = {u for u, m in xm.items() if m['slide_success']}
    ds = {u for u, r in tr_default.items() if r['slide']}
    print(f'\n  at the default lever: (x) tape classifier {sorted(xs)}')
    print(f'                        D3 StageTracker      {sorted(ds)}')
    print(f'    both {len(xs & ds)}   (x) only {sorted(xs - ds)}   D3 only {sorted(ds - xs)}')
    for u in sorted(xs - ds):
        r = tr_default[u]
        c = clauses[u]
        fails = [k for k in ('picked', 'released', 'within', 'can_up', 'goal_up', 'in_band',
                             'free', 'at_rest') if not c[k]]
        print(f'      uid {u}: released {int(r["released"])} pushed {int(r["pushed"])} '
              f'(gain {r["gain_mm"]} mm) nested_v2 {int(r["nested_v2"])}  '
              f'final dist {c["dist_mm"]} mm  z {c["z_cm"]} cm (band {c["band_cm"]})  '
              f'lever {c["lever_mm"]} mm  FAILING CLAUSES: {fails or "none"}')
    if a.json:
        pl.Path(a.json).write_text(json.dumps(dict(cohort=a.cohort, sweep=rows,
                                                   default=tr_default, clauses=clauses),
                                              indent=1, default=str))
        print(f'wrote {a.json}')


if __name__ == '__main__':
    main()
