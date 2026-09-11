#!/usr/bin/env python3
"""Validate `nested_v2` and the in-episode (x) `slide_success` against the reference predicates.

LADDER_UNIFY_BRIEF_2026-09-10, Lane 1. Consumes the per-ENV-FRAME logs written by
`baselines/eval_e2e_stagerec.py` (one `<out>/frames/ep<k>.npz` per episode) and re-computes the
new predicates OFFLINE with `baselines/stage_predicates.StageTracker`. Nothing is re-simulated,
so the comparison is on the SAME episodes -- the reference columns
(`nested_honest`, `nested_proxy`, amendment-(l) `slide_success`) come from the evaluator's own
`end_of_episode()` call on that episode and are carried in the npz as `ref_*`.

Episode-level readings of `nested_v2`, all reported (they are not the same question):
  end     the predicate on the LAST env frame of the episode.
  ever    sticky: it held on at least one frame.
  lastK   it held on at least one of the final K frames (default 12 = the at_rest window).
`nested_honest` is measured AFTER a 100-step settle, so a pre-settle read can legitimately
differ; that is the thing being quantified, not a bug.

usage:
  python3 baselines/diagnostics/nested_v2_validate.py --roll <dir-of-arms> [--sweep] [--json out]
     <dir-of-arms>/<arm>/ep<k>/frames/ep<k>.npz
"""
import argparse
import glob
import json
import os
import pathlib as pl
import sys

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
from stage_predicates import HELD_LEVER_M, StageTracker  # noqa: E402


def load_episodes(arm_dir):
    """Accepts both layouts: `<arm>/ep<k>/frames/ep<k>.npz` (eval_e2e_stagerec.py, one output dir
    per episode) and `<arm>/frames/ep<k>.npz` (replay_tape_stagerec.py, one dir per tape set)."""
    fs = (glob.glob(os.path.join(arm_dir, 'ep*', 'frames', 'ep*.npz'))
          or glob.glob(os.path.join(arm_dir, 'frames', 'ep*.npz')))
    eps = []
    for f in sorted(fs, key=lambda p: int(pl.Path(p).stem[2:])):
        z = np.load(f, allow_pickle=True)
        eps.append({k: z[k] for k in z.files} | {'path': f})
    return eps


def run_tracker(ep, *, held_lever=HELD_LEVER_M, at_rest_frames=12, **kw):
    tr = StageTracker(np.asarray(ep['goal_pos'])[0, :2], float(ep['shelf_top_z']),
                      held_lever_m=held_lever, at_rest_frames=at_rest_frames, **kw)
    n = int(ep['n_frames'])
    per = []
    for i in range(n):
        per.append(tr.update(
            can_pos=ep['can_pos'][i], can_quat=ep['can_quat'][i],
            goal_pos=ep['goal_pos'][i], goal_quat=ep['goal_quat'][i],
            tool_xy=ep['tool'][i, :2], grip_cmd=float(ep['grip_cmd'][i]),
            picked=bool(ep['picked'][i]), placed_v2=bool(ep['placed_v2'][i]),
            can_goal_contact=bool(ep['can_goal_contact'][i]),
            gripper_goal_contact=bool(ep['gripper_goal_contact'][i])))
    nv = np.array([p['nested_v2'] for p in per], bool)
    return tr, per, nv


def confusion(pred, ref):
    pred, ref = np.asarray(pred, bool), np.asarray(ref, bool)
    tp = int((pred & ref).sum()); fp = int((pred & ~ref).sum())
    fn = int((~pred & ref).sum()); tn = int((~pred & ~ref).sum())
    prec = tp / (tp + fp) if tp + fp else float('nan')
    rec = tp / (tp + fn) if tp + fn else float('nan')
    return dict(tp=tp, fp=fp, fn=fn, tn=tn, n=tp + fp + fn + tn,
                precision=prec, recall=rec,
                agree=(tp + tn) / max(tp + fp + fn + tn, 1))


def fmt(name, c):
    return (f'  {name:26s} TP {c["tp"]:3d}  FP {c["fp"]:3d}  FN {c["fn"]:3d}  TN {c["tn"]:3d}  '
            f'| precision {c["precision"]:.3f}  recall {c["recall"]:.3f}  '
            f'agreement {c["agree"]:.3f}  (n={c["n"]})')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--roll', required=True, help='dir holding one subdir per arm')
    ap.add_argument('--lastk', type=int, default=12)
    ap.add_argument('--sweep', action='store_true', help='sweep HELD_LEVER_M')
    ap.add_argument('--json', default=None)
    a = ap.parse_args()

    arms = [d for d in sorted(os.listdir(a.roll)) if os.path.isdir(os.path.join(a.roll, d))]
    out = {}
    for arm in arms:
        eps = load_episodes(os.path.join(a.roll, arm))
        if not eps:
            print(f'{arm}: NO EPISODES'); continue
        rows = []
        for ep in eps:
            tr, per, nv = run_tracker(ep)
            _, _, nv_nr = run_tracker(ep, at_rest_frames=1)   # at_rest trivially satisfied? no:
            # at_rest_frames=1 means the window is one sample, spread 0 -> at_rest always True.
            k = min(a.lastk, len(nv))
            rows.append(dict(
                ep=int(ep['ep']), n=int(ep['n_frames']), decisions=int(ep['decisions']),
                nested_v2_end=bool(nv[-1]), nested_v2_ever=bool(nv.any()),
                nested_v2_lastk=bool(nv[-k:].any()),
                nested_v2_norest_end=bool(nv_nr[-1]), nested_v2_norest_lastk=bool(nv_nr[-k:].any()),
                slide_x=bool(tr.slide_success), pushed=bool(tr.pushed),
                released=bool(tr.released), contact_push_x=bool(tr.contact_push),
                gain_mm=round(float(tr.goalward_gain_m) * 1000, 2),
                min_lever_post_release_mm=(
                    round(float(np.min([p['lever_m'] for p in per[tr.released_frame:]])) * 1000, 2)
                    if tr.released_frame is not None else None),
                final_dist_mm=round(float(per[-1]['dist_xy_m']) * 1000, 2),
                ref_nested_honest=bool(ep['ref_nested_honest']),
                ref_nested_proxy=bool(ep['ref_nested_proxy']),
                ref_slide_l=bool(ep['ref_slide_l']), ref_slide_route=str(ep['ref_slide_route']),
                ref_picked=bool(ep['ref_picked']), ref_placed_v2=bool(ep['ref_placed_v2']),
                ref_contact=bool(ep['ref_contact']),
                ref_contact_push=bool(ep['ref_contact_push']),
                ref_tipped=bool(ep['ref_tipped'])))
        g = lambda k: [r[k] for r in rows]                                    # noqa: E731
        H = g('ref_nested_honest')
        print(f'\n{"=" * 92}\n{{RLPD}} arm {arm}: {len(rows)} episodes  '
              f'(rnd30, mode, max_steps 1200, one process per episode)')
        print(f'  reference counts: nested_honest {sum(H)}  nested_proxy {sum(g("ref_nested_proxy"))}  '
              f'slide(l) {sum(g("ref_slide_l"))}  picked {sum(g("ref_picked"))}  '
              f'placed_v2 {sum(g("ref_placed_v2"))}  contact {sum(g("ref_contact"))}  '
              f'contact_push(legacy) {sum(g("ref_contact_push"))}  tipped {sum(g("ref_tipped"))}')
        print(f'  new counts:       nested_v2 end {sum(g("nested_v2_end"))} / '
              f'lastK {sum(g("nested_v2_lastk"))} / ever {sum(g("nested_v2_ever"))}   '
              f'slide(x) {sum(g("slide_x"))}  pushed {sum(g("pushed"))}  '
              f'released {sum(g("released"))}  contact_push(x) {sum(g("contact_push_x"))}')
        print('\n  vs nested_honest (the settled reference):')
        cm = {}
        for key in ('nested_v2_end', 'nested_v2_lastk', 'nested_v2_ever',
                    'nested_v2_norest_lastk', 'ref_nested_proxy'):
            cm[key] = confusion(g(key), H)
            print(fmt(key, cm[key]))
        print('\n  slide: (x) in-episode vs (l) end_of_episode:')
        cm['slide_x_vs_l'] = confusion(g('slide_x'), g('ref_slide_l'))
        print(fmt('slide_x vs slide_l', cm['slide_x_vs_l']))
        cm['slide_x_vs_honest'] = confusion(g('slide_x'), H)
        print(fmt('slide_x vs nested_honest', cm['slide_x_vs_honest']))
        cm['contact_push_x_vs_legacy'] = confusion(g('contact_push_x'), g('ref_contact_push'))
        print(fmt('contact_push(x) vs legacy', cm['contact_push_x_vs_legacy']))

        dis = [r for r in rows if r['nested_v2_lastk'] != r['ref_nested_honest']]
        print(f'\n  DISAGREEMENTS nested_v2(lastK) vs nested_honest: {len(dis)}')
        for r in dis:
            why = []
            if r['ref_nested_honest'] and not r['nested_v2_lastk']:
                why.append('settle moved it IN' if r['final_dist_mm'] > 81 else
                           ('not at rest / in hand at the horizon' if r['ref_placed_v2']
                            else 'placed_v2 never granted'))
            else:
                why.append('settle moved it OUT (final_dist %.1f mm pre-settle)' % r['final_dist_mm'])
            print(f'    ep{r["ep"]:2d}  nested_v2 {int(r["nested_v2_lastk"])} honest '
                  f'{int(r["ref_nested_honest"])}  proxy {int(r["ref_nested_proxy"])}  '
                  f'final_dist {r["final_dist_mm"]:6.1f} mm  gain {r["gain_mm"]:6.1f} mm  '
                  f'pv2 {int(r["ref_placed_v2"])} tip {int(r["ref_tipped"])}  -> {why[0]}')
        dsl = [r for r in rows if r['slide_x'] != r['ref_slide_l']]
        print(f'  DISAGREEMENTS slide(x) vs slide(l): {len(dsl)}')
        for r in dsl:
            print(f'    ep{r["ep"]:2d}  slide_x {int(r["slide_x"])} slide_l {int(r["ref_slide_l"])} '
                  f'({r["ref_slide_route"]})  pushed {int(r["pushed"])} gain {r["gain_mm"]:.1f} mm  '
                  f'nested_v2 {int(r["nested_v2_lastk"])} honest {int(r["ref_nested_honest"])}')
        out[arm] = dict(rows=rows, confusion=cm)

        if a.sweep:
            print('\n  HELD_LEVER sweep (episode counts; agreement with nested_honest):')
            sw = []
            for lv in (0.015, 0.020, 0.025, 0.028, 0.030, 0.033, 0.040, 1e9):
                nn = ss = 0
                pred = []
                for ep in eps:
                    tr, _, nv = run_tracker(ep, held_lever=lv)
                    k = min(a.lastk, len(nv))
                    p = bool(nv[-k:].any()); pred.append(p)
                    nn += int(p); ss += int(tr.slide_success)
                c = confusion(pred, H)
                tag = '  <-- default' if abs(lv - HELD_LEVER_M) < 1e-9 else (
                    '  (in_hand disabled)' if lv > 1 else '')
                print(f'    lever {("inf" if lv > 1 else f"{lv * 100:4.1f} cm"):>7s}  '
                      f'nested_v2(lastK) {nn:2d}  slide(x) {ss:2d}  '
                      f'FP {c["fp"]:2d} FN {c["fn"]:2d} precision {c["precision"]:.3f} '
                      f'recall {c["recall"]:.3f}{tag}')
                sw.append(dict(lever_m=lv, nested_v2=nn, slide_x=ss, **c))
            out[arm]['sweep'] = sw

    if a.json:
        pl.Path(a.json).write_text(json.dumps(out, indent=1, default=str))
        print(f'\nwrote {a.json}')


if __name__ == '__main__':
    main()
