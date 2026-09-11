#!/usr/bin/env python3
"""Episode-for-episode reproducibility of two {RLPD} rollout batches (Lane 11, 2026-09-11).

    python baselines/diagnostics/pilot_repro_check.py --a <batch A root> --b <batch B root>

Both roots hold `<ckpt>_<icset>_s<seed>/metrics.json`. Cells present in BOTH are compared
episode by episode on the outcome and on every stage column the two share. Cells present in only
one are listed, never silently dropped.

Written to ask one question: does re-running Lane 9's cells (same checkpoint, same start set,
same seed, same box, same shared-process order) reproduce their 23 `slide_success` positives?
A disagreement is a fact about the protocol, not a failure of either run -- the two batches
differ in the evaluator (`eval_e2e_annot.py --video` vs `eval_e2e.py --records-out`), so a
disagreement localises to the camera render, the recorder wrapper, or genuine non-determinism,
and this script says WHICH episodes so the next test can be targeted.
"""
import argparse
import glob
import json
import os


def cells(root):
    out = {}
    for p in sorted(glob.glob(os.path.join(root, '*', 'metrics.json'))):
        out[os.path.basename(os.path.dirname(p))] = json.load(open(p))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--a', required=True, help='batch A root (e.g. Lane 9 slide_smoke)')
    ap.add_argument('--b', required=True, help='batch B root (e.g. Lane 11 roll)')
    ap.add_argument('--label-a', default='A')
    ap.add_argument('--label-b', default='B')
    ap.add_argument('--stage', default='slide_success', help='the stage the headline counts')
    g = ap.parse_args()

    A, B = cells(g.a), cells(g.b)
    only_a, only_b = sorted(set(A) - set(B)), sorted(set(B) - set(A))
    common = sorted(set(A) & set(B))
    print(f'cells in {g.label_a}: {len(A)}   in {g.label_b}: {len(B)}   common: {len(common)}')
    if only_a:
        print(f'  only in {g.label_a}: {only_a}')
    if only_b:
        print(f'  only in {g.label_b}: {only_b}')
    print()
    print(f'| cell | n | outcome agree | `{g.stage}` {g.label_a} | `{g.stage}` {g.label_b} | '
          f'`{g.stage}` agree | all-stage disagreements |')
    print('|' + '---|' * 7)
    tot = dict(n=0, out_ok=0, st_ok=0, sa=0, sb=0, stage_dis=0)
    detail = []
    for c in common:
        ea, eb = A[c]['per_episode'], B[c]['per_episode']
        n = min(len(ea), len(eb))
        shared = sorted(set(ea[0]['stages']) & set(eb[0]['stages'])) if n else []
        out_ok = sum(1 for i in range(n) if ea[i]['outcome'] == eb[i]['outcome'])
        sa = sum(1 for i in range(n) if ea[i]['stages'].get(g.stage))
        sb = sum(1 for i in range(n) if eb[i]['stages'].get(g.stage))
        st_ok = sum(1 for i in range(n)
                    if bool(ea[i]['stages'].get(g.stage)) == bool(eb[i]['stages'].get(g.stage)))
        dis = 0
        for i in range(n):
            for k in shared:
                if bool(ea[i]['stages'][k]) != bool(eb[i]['stages'][k]):
                    dis += 1
                    detail.append((c, i, ea[i].get('uid'), k,
                                   bool(ea[i]['stages'][k]), bool(eb[i]['stages'][k])))
        print(f'| `{c}` | {n} | {out_ok}/{n} | {sa} | {sb} | {st_ok}/{n} | {dis} |')
        tot['n'] += n; tot['out_ok'] += out_ok; tot['st_ok'] += st_ok
        tot['sa'] += sa; tot['sb'] += sb; tot['stage_dis'] += dis
    print(f'| **total** | {tot["n"]} | {tot["out_ok"]}/{tot["n"]} | {tot["sa"]} | {tot["sb"]} | '
          f'{tot["st_ok"]}/{tot["n"]} | {tot["stage_dis"]} |')
    print()
    if detail:
        print(f'### per-flag disagreements ({len(detail)})\n')
        print(f'| cell | ep | uid | flag | {g.label_a} | {g.label_b} |')
        print('|' + '---|' * 6)
        for row in detail[:200]:
            print(f'| `{row[0]}` | {row[1]} | {row[2]} | `{row[3]}` | {int(row[4])} | {int(row[5])} |')
        if len(detail) > 200:
            print(f'\n_{len(detail) - 200} further rows not printed._')
    else:
        print('**No per-flag disagreement on any shared stage column in any common cell.**')


if __name__ == '__main__':
    main()
