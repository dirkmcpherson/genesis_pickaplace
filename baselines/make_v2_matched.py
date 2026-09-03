#!/usr/bin/env python3
"""Build a PREREG-A21 v2 matched set from ONE contract-v1 recording.

WHY: the frozen sets (matched_v2/{dH,dDP,...}, matched_w3/{dH,dDP}) were built by
make_matched_sets.py across three sources at once. The A21 v2 block adds sets one at
a time (dHv2 first; dDPv2 only after the v2 teacher is trained and harvested), so this
builder writes a single named set from a single recording, with the SAME write_set
machinery (contract validation, hard links, episode_list, content sha, sim_variant
uniformity FATAL, manifest) as the frozen pipeline. Two modes:

  no --base   take ALL success tapes from --src (the dHv2 case: the human set IS the
              IC universe; N = kept count).
  --base D    per-IC match to an existing set D (the dDPv2 case: same per-ic_uid
              multiset as dHv2, drawn rng(--seed) from --src successes; FATAL on any
              shortfall unless --allow-short, which then reports it in the manifest --
              mirrors make_unpruned_matched.py's matching, but SHORTFALLS ARE LOUD).

Never writes into an existing set dir unless --force (frozen sets stay frozen).

Usage:
  python baselines/make_v2_matched.py --src baselines/demos_v2/dHv2 \
      --name dHv2 --out-root baselines/matched_v2
  python baselines/make_v2_matched.py --src baselines/demos_v2/dDPv2 \
      --name dDPv2 --out-root baselines/matched_v2 --base baselines/matched_v2/dHv2
"""
import argparse
import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from make_matched_sets import read_dir, write_set, git_sha  # noqa: E402

FROZEN = {'dH', 'dDP', 'dR2D', 'dHunpruned', 'dHsucc_dup', 'dDPsucc_dup', 'dDPfails',
          'dR2DDPfails', 'dHHfails', 'dR2DR2Dfails', 'dH_A', 'dH_B', 'dDP_A', 'dDP_B',
          'dHallpruned_1e3', 'dHallpruned_1e2', 'dDPallpruned_1e3', 'dDPallpruned_1e2'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='recorder success dir (contract v1)')
    ap.add_argument('--name', required=True, help='set name, e.g. dHv2 / dDPv2')
    ap.add_argument('--out-root', dest='out_root', required=True)
    ap.add_argument('--base', default=None, help='existing set to per-IC match (dDPv2 mode)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--allow-short', action='store_true',
                    help='with --base: tolerate per-IC shortfalls (reported), fill from leftovers to match N')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()

    if args.name in FROZEN:
        sys.exit(f'FATAL: {args.name} is a frozen set name (results of record); v2 sets use new names')
    dst = os.path.join(args.out_root, args.name)
    if os.path.exists(dst) and not args.force:
        sys.exit(f'FATAL: {dst} exists; refusing to overwrite without --force')

    src = [t for t in read_dir(args.src, None, True) if t['label'] == 'success']
    if not src:
        sys.exit(f'FATAL: no success tapes in {args.src}')

    notes = dict(role=f'PREREG A21 v2 set from {args.src}', rows=None)
    if args.base is None:
        chosen = src
        notes['matching'] = 'none: single-source set = ALL kept successes (the IC universe)'
    else:
        base = read_dir(args.base, 'success', True)
        want = collections.Counter(t['ic_uid'] for t in base)
        have = collections.defaultdict(list)
        for t in src:
            have[t['ic_uid']].append(t)
        rng = np.random.default_rng(args.seed)
        chosen, short = [], {}
        for ic, k in sorted(want.items(), key=lambda x: str(x[0])):
            pool = list(have.get(ic, []))
            rng.shuffle(pool)
            if len(pool) < k:
                short[str(ic)] = (k, len(pool))
            chosen += pool[:k]
        if short and not args.allow_short:
            sys.exit(f'FATAL: per-IC shortfall vs {args.base}: {short} '
                     f'(harvest more at these ICs, or pass --allow-short to fill from leftovers)')
        if len(chosen) < len(base):
            used = {t['name'] for t in chosen}
            rest = [t for t in src if t['name'] not in used]
            rng.shuffle(rest)
            chosen += rest[:len(base) - len(chosen)]
        notes['matching'] = f'per-IC multiset matched to {args.base} (rng seed {args.seed})'
        notes['base_set'] = args.base
        notes['short_ics'] = short or None
        notes['ic_multiset_identical'] = not short
    notes['rows'] = int(sum(t['n'] for t in chosen))

    print(f'[v2] {args.name}: {len(src)} src successes -> N={len(chosen)} '
          f'rows={notes["rows"]} ICs={len(set(t["ic_uid"] for t in chosen))} '
          f'len p50 {np.median([t["n"] for t in chosen]):.0f}')
    if os.path.exists(dst):
        import shutil
        shutil.rmtree(dst)   # --force path; write_set also rmtrees, this keeps intent explicit
    sha, man = write_set(args.name, chosen, args.out_root, notes, git_sha(), args.seed,
                         [args.src] + ([args.base] if args.base else []))
    print(f'  wrote {dst}: N={man["N"]} sim_variant={man["sim_variant"]} sha {sha[:16]}')


if __name__ == '__main__':
    main()
