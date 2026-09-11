#!/usr/bin/env python3
"""Calibrate the SLIDE EVENT on the human demonstrations (Lane 7, 2026-09-11).

User, 2026-09-11: "characterize this so that as many human demos do it as possible; the
important part is that they put the can down, move to the opposite side of the can as the
goal, and slide it."

So the slide event is three clauses, and their constants are a MEASUREMENT, not a choice:

    slide_event = released on the shelf
                  AND the tool is on the far side of the can along the can->goal line
                  AND the can travels goalward while pushed from there

This script grids the constants of all three over the stage records and scores every cell by
what it is FOR (human tapes that complete a slide in sim) and what it must not admit (the
non-slide classes -- carry-in, nested-by-drop, placed-only). It then adds the arrival clause
two ways: `nested_v2` (readable in training) and `nested_v2 or the end-of-episode settle`
(readable only on a TAPE, because the settle simulates -- so it can build a demonstration set
but can never gate an online reward).

    python baselines/diagnostics/ladder_n_slide_calibration.py \
        --records /home/j/data/genesis_pickaplace/stage_records \
        --census <lane5>/can_pos_recovery/videos_ladder_2026-09-11

One pass per tape serves the whole grid. The credit a frame carries -- the amount by which it
lowers the running minimum of dist(can, goal) since the release -- does not depend on the
geometry; only WHICH frames are eligible does. So the per-frame credit is computed once and
each grid cell is a masked sum, which is why 160 cells cost one re-read of the records.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.environ.get('GENESIS_PICKAPLACE_ROOT',
                      os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, 'baselines'))

from stage_predicates import HELD_LEVER_M, StageTracker, tilt_deg   # noqa: E402

ARMS = [('dHfull_all', 'human', 'census_human.json'),
        ('dDPfull_first', 'machine', 'census_machine.json')]
# The classes a slide predicate must NOT admit (user: the can was put down and not slid).
FALSE_POSITIVE_CLASSES = ('carry_in', 'nested_drop', 'placed_only')
REACHES = (0.06, 0.08, 0.10, 0.12)
CONES = (90.0, 60.0)          # 90 deg == the brief's dot < 0; 60 deg is the tighter cone
GAINS = (0.005, 0.01, 0.02, 0.03, 0.05)
RELEASES = (0.0, 0.05, 0.08, 0.10)


def tape_features(rec):
    """Everything the grid needs from one tape, computed once.

    Returns None when the tape never releases (no pick or no `placed_v2`): no setting of the
    constants can make such a tape a slide, and counting it in a denominator would hide that.
    """
    n = int(rec['dec'].shape[0])
    can = np.asarray(rec['can_pos'], float)[:, :2]
    goal = np.asarray(rec['goal_pos'], float)[:, :2]
    tool = np.asarray(rec['tool_xy'], float)
    picked = np.asarray(rec['picked'], bool)
    pv2 = np.asarray(rec['placed_v2'], bool)

    rel_mask = pv2 & picked
    if not rel_mask.any():
        return None
    rel = int(np.argmax(rel_mask))

    dist = np.hypot(*(can - goal).T)
    v_tool = tool - can
    v_goal = goal - can
    lever = np.hypot(*v_tool.T)
    in_hand = lever < HELD_LEVER_M
    with np.errstate(invalid='ignore', divide='ignore'):
        cos = ((v_tool * v_goal).sum(1) / (lever * np.hypot(*v_goal.T) + 1e-12)).clip(-1, 1)
    angle_off_axis = np.degrees(np.arccos(cos))          # 0 = tool between can and goal
    # the tool is on the FAR side when it is more than 90 deg off the can->goal direction;
    # `cone` below is the half-angle measured from the OPPOSITE ray, so cone 90 == dot < 0.
    far_angle = 180.0 - angle_off_axis

    # per-frame credit: how much this frame lowers the running minimum of dist since release.
    # Geometry-independent, which is what makes the grid cheap.
    delta = np.zeros(n)
    m = dist[rel]
    for i in range(rel + 1, n):
        if dist[i] < m:
            delta[i] = m - dist[i]
            m = dist[i]
    after = np.zeros(n, bool)
    after[rel + 1:] = True

    # arrival, both readings -- and `at_rest`, which the set-down latch needs
    tr = StageTracker(goal[0], float(rec['shelf_top_z']))
    granted = False
    nv2 = np.zeros(n, bool)
    rest = np.zeros(n, bool)
    for i in range(n):
        pg = bool(pv2[i]) or granted
        granted = granted or bool(pv2[i])
        f = tr.update(can_pos=rec['can_pos'][i], can_quat=rec['can_quat'][i],
                      goal_pos=rec['goal_pos'][i], goal_quat=rec['goal_quat'][i],
                      tool_xy=rec['tool_xy'][i], grip_cmd=float(rec['grip_cmd'][i]),
                      picked=bool(picked[i]), can_goal_contact=bool(rec['can_goal_contact'][i]),
                      gripper_goal_contact=bool(rec['gripper_goal_contact'][i]), placed_v2=pg)
        nv2[i] = f['nested_v2']
        rest[i] = f['at_rest']
    settle = bool(np.asarray(rec['nested_honest']).item())

    # THE SET-DOWN LATCH (Lane 9, 315 sampled {RLPD} rollouts): `pushed` is mostly the
    # release TRANSIENT -- 20 of 23 slide_success positives fired it within 3 decisions of
    # the release, 5 inside the release decision itself, with the can rolling out of the
    # opening hand while the tool lever GREW. "Put the can down" is a clause, not a preamble:
    # credit starts only once the can has been at rest at least one frame after the release.
    after_rel = np.zeros(n, bool)
    after_rel[rel + 1:] = True
    rest_after = np.where(after_rel & rest)[0]
    settled_frame = int(rest_after[0]) if len(rest_after) else None
    after_settled = np.zeros(n, bool)
    if settled_frame is not None:
        after_settled[settled_frame + 1:] = True

    return dict(n=n, rel=rel, release_dist=float(dist[rel]), delta=delta, after=after,
                after_settled=after_settled, settled_frame=settled_frame,
                lever=lever, far_angle=far_angle, in_hand=in_hand, nv2=nv2, at_rest=rest,
                nv2_final=bool(nv2[-1]), nv2_ever=bool(nv2.any()), settle=settle)


def cell(feat, reach, cone, gain_thr, rel_min, latch=True):
    """-> (slide_event, home_nested_v2, home_or_settle, gain_m) for one tape and one cell.

    `latch` is the set-down clause: credit only after the can has been at rest once since the
    release. With it off, the release transient counts, which is what Lane 9 measured the old
    `pushed` to be paying for."""
    if feat is None or feat['release_dist'] < rel_min:
        return False, False, False, 0.0
    window = feat['after_settled'] if latch else feat['after']
    mask = (window & (~feat['in_hand']) & (feat['far_angle'] <= cone)
            & (feat['lever'] <= reach))
    cum = np.cumsum(np.where(mask, feat['delta'], 0.0))
    gain = float(cum[-1])
    if gain < gain_thr:
        return False, False, False, gain
    first = int(np.searchsorted(cum, gain_thr, side='left'))
    arrived = bool(feat['nv2'][first:].any())
    return True, arrived, bool(arrived or feat['settle']), gain


def gain_during_release(feat, reach, cone):
    """The credit the set-down latch EXCLUDES -- the diagnostic that makes it visible."""
    if feat is None or feat['settled_frame'] is None:
        return 0.0
    pre = feat['after'] & (~feat['after_settled'])
    mask = pre & (~feat['in_hand']) & (feat['far_angle'] <= cone) & (feat['lever'] <= reach)
    return float(np.where(mask, feat['delta'], 0.0).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--records', required=True)
    ap.add_argument('--census', required=True)
    ap.add_argument('--top', type=int, default=12)
    a = ap.parse_args()

    data = {}
    for arm, name, fn in ARMS:
        cens = {r['file']: r for r in json.load(open(os.path.join(a.census, fn)))['rows']}
        feats, klass, uid = {}, {}, {}
        for f in sorted(glob.glob(os.path.join(a.records, arm, '*.npz'))):
            b = os.path.basename(f)
            if b == 'manifest.json':
                continue
            feats[b] = tape_features(np.load(f, allow_pickle=True))
            klass[b] = cens.get(b, {}).get('klass', '?')
            uid[b] = cens.get(b, {}).get('ic_uid', -1)
        data[name] = dict(feats=feats, klass=klass, uid=uid)
        n_slide = sum(1 for k in klass.values() if k == 'slide')
        n_fp = sum(1 for k in klass.values() if k in FALSE_POSITIVE_CLASSES)
        print(f'# {name}: {len(feats)} tapes, {n_slide} sim-slides, {n_fp} in the '
              f'non-slide classes {FALSE_POSITIVE_CLASSES}', flush=True)
    print()

    rows = []
    for latch in (True, False):
        for reach in REACHES:
            for cone in CONES:
                for gt in GAINS:
                    for rm in RELEASES:
                        rec = dict(reach=reach, cone=cone, gain=gt, rel=rm, latch=latch)
                        for name in ('human', 'machine'):
                            d = data[name]
                            se = ar = st = 0
                            fp = fp_ar = 0
                            hit = []
                            for b, ft in d['feats'].items():
                                s, h, hs, _g = cell(ft, reach, cone, gt, rm, latch)
                                if s:
                                    se += 1
                                    if d['klass'][b] == 'slide':
                                        hit.append(d['uid'][b])
                                    if d['klass'][b] in FALSE_POSITIVE_CLASSES:
                                        fp += 1
                                ar += int(h)
                                st += int(hs)
                                if h and d['klass'][b] in FALSE_POSITIVE_CLASSES:
                                    fp_ar += 1
                            rec[name] = dict(slide_event=se, home=ar, home_settle=st,
                                             fp=fp, fp_home=fp_ar, slides_hit=sorted(hit))
                        rows.append(rec)
    LATCHED = [r for r in rows if r['latch']]
    UNLATCHED = [r for r in rows if not r['latch']]

    H = data['human']
    n_h_slide = sum(1 for k in H['klass'].values() if k == 'slide')

    print('## A. Coverage grid -- `slide_event` only (release + far side + goalward gain)\n')
    print('Cells are: human sim-slides admitted / human false positives '
          f'({"+".join(FALSE_POSITIVE_CLASSES)}) -- human tapes total, machine tapes total.\n')
    for cone in CONES:
        print(f'### far-side cone <= {cone:g} deg off the can->goal axis'
              + ('  (== the brief\'s dot < 0)' if cone == 90 else '') + '\n')
        for rm in RELEASES:
            print(f'**release clause: '
                  + (f'>= {rm * 100:g} cm from the goal**' if rm else 'none**') + '\n')
            print('| reach \\ gain | ' + ' | '.join(f'>= {g * 100:g} cm' for g in GAINS) + ' |')
            print('|' + '---|' * (len(GAINS) + 1))
            for reach in REACHES:
                cells = []
                for gt in GAINS:
                    r = next(x for x in LATCHED if x['reach'] == reach and x['cone'] == cone
                             and x['gain'] == gt and x['rel'] == rm)
                    h = r['human']
                    cells.append(f"**{len(h['slides_hit'])}**/{h['fp']} — {h['slide_event']}, "
                                 f"{r['machine']['slide_event']}")
                print(f'| {reach * 100:g} cm | ' + ' | '.join(cells) + ' |')
            print()

    print('## B. Ranking -- most human sim-slides, then fewest false positives\n')
    def key(r):
        h = r['human']
        return (-len(h['slides_hit']), h['fp'], -h['slide_event'])
    best = sorted(LATCHED, key=key)[:a.top]
    print('| reach | cone | gain | release | human slides | human FP | human `slide_event` | '
          'machine `slide_event` | human `home` | human `home`+settle | machine `home` |')
    print('|' + '---|' * 11)
    for r in best:
        h, m = r['human'], r['machine']
        print(f"| {r['reach'] * 100:g} cm | {r['cone']:g}° | {r['gain'] * 100:g} cm | "
              f"{(str(r['rel'] * 100) + ' cm') if r['rel'] else 'none'} | "
              f"**{len(h['slides_hit'])}/{n_h_slide}** | {h['fp']} | {h['slide_event']} | "
              f"{m['slide_event']} | {h['home']} | {h['home_settle']} | {m['home']} |")
    print()

    top = best[0]
    print(f"### the leading cell: reach {top['reach'] * 100:g} cm, cone {top['cone']:g}°, "
          f"gain >= {top['gain'] * 100:g} cm, release "
          + (f">= {top['rel'] * 100:g} cm" if top['rel'] else 'unconstrained') + '\n')
    for name in ('human', 'machine'):
        d = data[name]
        got, miss = [], []
        for b, ft in d['feats'].items():
            s, h, hs, g = cell(ft, top['reach'], top['cone'], top['gain'], top['rel'])
            if d['klass'][b] == 'slide':
                (got if s else miss).append((d['uid'][b], g, ft))
            elif s and d['klass'][b] in FALSE_POSITIVE_CLASSES:
                miss.append((d['uid'][b], g, ft))
        print(f"- {name} sim-slides admitted: {sorted(u for u, _, _ in got)}")
        print(f"- {name} sim-slides MISSED: "
              + (', '.join(f'{u} (gain {g * 100:.2f} cm)' for u, g, _ in sorted(miss)) or 'none'))
    print()

    print('## C. What the arrival clause costs\n')
    print('| reach | cone | gain | release | human `slide_event` | + `nested_v2` | '
          '+ `nested_v2` or settle | machine `slide_event` | + `nested_v2` | + settle |')
    print('|' + '---|' * 10)
    for r in best[:6]:
        h, m = r['human'], r['machine']
        print(f"| {r['reach'] * 100:g} cm | {r['cone']:g}° | {r['gain'] * 100:g} cm | "
              f"{(str(r['rel'] * 100) + ' cm') if r['rel'] else 'none'} | {h['slide_event']} | "
              f"{h['home']} | {h['home_settle']} | {m['slide_event']} | {m['home']} | "
              f"{m['home_settle']} |")
    print()

    print('## C2. What the SET-DOWN LATCH costs (Lane 9: the old `pushed` was the release '
          'transient)\n')
    print('The latch requires the can to have been at rest once after the release before any '
          'goalward millimetre counts. Without it, motion inside the release decision pays.\n')
    print('| reach | cone | gain | release | human slides LATCHED | UNLATCHED | human FP latched '
          '| unlatched | machine `slide_event` latched | unlatched |')
    print('|' + '---|' * 10)
    for r in best[:8]:
        u = next(x for x in UNLATCHED if all(x[k] == r[k] for k in ('reach', 'cone', 'gain', 'rel')))
        print(f"| {r['reach'] * 100:g} cm | {r['cone']:g}\u00b0 | {r['gain'] * 100:g} cm | "
              f"{(str(r['rel'] * 100) + ' cm') if r['rel'] else 'none'} | "
              f"**{len(r['human']['slides_hit'])}** | {len(u['human']['slides_hit'])} | "
              f"{r['human']['fp']} | {u['human']['fp']} | {r['machine']['slide_event']} | "
              f"{u['machine']['slide_event']} |")
    print()
    print('### the transient the latch excludes, at the leading cell\n')
    print('| arm | tapes with a set-down | gain excluded p50 (mm) | p90 | max | '
          'tapes whose WHOLE gain was the transient |')
    print('|' + '---|' * 6)
    for name in ('human', 'machine'):
        d = data[name]
        ex, whole, nset = [], 0, 0
        for b, ft in d['feats'].items():
            if ft is None:
                continue
            if ft['settled_frame'] is not None:
                nset += 1
            g = gain_during_release(ft, top['reach'], top['cone'])
            ex.append(g * 1000)
            s_l = cell(ft, top['reach'], top['cone'], top['gain'], top['rel'], True)[0]
            s_u = cell(ft, top['reach'], top['cone'], top['gain'], top['rel'], False)[0]
            whole += int(s_u and not s_l)
        ex = np.asarray(ex)
        print(f'| {name} | {nset} | {np.percentile(ex, 50):.2f} | {np.percentile(ex, 90):.2f} | '
              f'{ex.max():.1f} | {whole} |')
    print()

    print('## D. Release distance by behaviour class (justifies or adjusts the release clause)\n')
    print('| arm | class | tapes | releases | release p10 | p50 | p90 | >= 5 cm | >= 8 cm | >= 10 cm |')
    print('|' + '---|' * 10)
    for name in ('human', 'machine'):
        d = data[name]
        byk = {}
        for b, ft in d['feats'].items():
            byk.setdefault(d['klass'][b], []).append(ft)
        for k in sorted(byk, key=lambda x: -len(byk[x])):
            v = np.array([f['release_dist'] for f in byk[k] if f is not None])
            if not len(v):
                print(f'| {name} | {k} | {len(byk[k])} | 0 | - | - | - | - | - | - |')
                continue
            print(f'| {name} | {k} | {len(byk[k])} | {len(v)} | '
                  + ' | '.join(f'{np.percentile(v, q) * 100:.1f}' for q in (10, 50, 90))
                  + ' | ' + ' | '.join(f'{int((v >= t).sum())}' for t in (0.05, 0.08, 0.10)) + ' |')
    print()


if __name__ == '__main__':
    main()
