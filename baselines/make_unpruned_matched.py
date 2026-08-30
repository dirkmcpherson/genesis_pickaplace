#!/usr/bin/env python3
"""Build dHunpruned as a matched set: same N and the same per-IC counts as matched_v2/dH, drawn from the
UNPRUNED human recording (baselines/demos_v1/dHunpruned, raw episodes_pick_phase_all, no leading-idle pruning).
User 2026-08-30: 'DP reacts the same to both sources' must be checked against un-cleaned human data.
Usage: python baselines/make_unpruned_matched.py --base baselines/matched_v2/dH --src baselines/demos_v1/dHunpruned --name dHunpruned"""
import argparse, collections, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from make_matched_sets import read_dir, write_set, git_sha

ap = argparse.ArgumentParser()
ap.add_argument('--base', default='baselines/matched_v2/dH'); ap.add_argument('--src', default='baselines/demos_v1/dHunpruned')
ap.add_argument('--name', default='dHunpruned'); ap.add_argument('--seed', type=int, default=0)
a = ap.parse_args()
base = read_dir(a.base, 'success', True); src = [t for t in read_dir(a.src, None, True) if t['label'] == 'success']
want = collections.Counter(t['ic_uid'] for t in base)
have = collections.defaultdict(list)
for t in src: have[t['ic_uid']].append(t)
rng = np.random.default_rng(a.seed); chosen = []; short = {}
for ic, k in sorted(want.items(), key=lambda x: str(x[0])):
    pool = have.get(ic, []); rng.shuffle(pool)
    if len(pool) < k: short[ic] = (k, len(pool))
    chosen += pool[:k]
# top up from unused ICs if some ICs are short, so N matches
if len(chosen) < len(base):
    used = {t['name'] for t in chosen}; rest = [t for t in src if t['name'] not in used]; rng.shuffle(rest)
    chosen += rest[:len(base) - len(chosen)]
print(f'[unpruned] base N={len(base)} ICs={len(want)}; src success={len(src)}; short ICs={short}; chosen N={len(chosen)} '
      f'rows={sum(t["n"] for t in chosen)} (base rows {sum(t["n"] for t in base)}) p50 {np.median([t["n"] for t in chosen]):.0f}')
sha, man = write_set(a.name, chosen, os.path.dirname(os.path.abspath(a.base).rstrip('/')),
                     dict(role='UNPRUNED human control: same N and per-IC counts as matched_v2/dH but from the raw recording '
                              '(no leading-idle pruning); user 2026-08-30', base_set=os.path.basename(a.base), short_ics={str(k): v for k, v in short.items()},
                          rows=int(sum(t['n'] for t in chosen))), git_sha(), a.seed, [a.base, a.src])
print(f'  wrote {a.name}: N={man["N"]} sha {sha[:16]}')
