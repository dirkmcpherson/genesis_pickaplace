#!/usr/bin/env python3
"""PHASE_PLAN (v): add --one-per-ic-first to to_dreamer_native.py. Exact anchors, idempotent."""
import sys, io
p = sys.argv[1]
s = io.open(p, encoding='utf-8').read()
if '--one-per-ic-first' in s:
    print(f'[first_flag] {p}: already patched'); sys.exit(0)

A1 = "    ap.add_argument('--one-per-ic-best', action='store_true', help='END-TO-END arm: keep ONE tape per ic_uid -- the highest recorded reward sum (nested > contact > picked > none), ties -> shortest')\n"
B1 = A1 + "    ap.add_argument('--one-per-ic-first', action='store_true', help=\"PHASE_PLAN (v) DE-CONFOUNDED arm: keep ONE tape per ic_uid -- the FIRST attempt (lowest rollout uid). No reward term: this is the human arm's protocol (one attempt per start, keep whatever happened).\")\n"

A2 = """    if args.one_per_ic_best:
        best = {}
        for f in files:
            z = np.load(f); u = int(z['ic_uid']) if 'ic_uid' in z.files else int(z['uid'])
            key = (float(np.asarray(z['rewards'], np.float32).sum()), -int(z['n']))
            if u not in best or key > best[u][0]:
                best[u] = (key, f)
        best_keep = {v[1] for v in best.values()}
        print(f'[to_dreamer_native] --one-per-ic-best: {len(files)} tapes over {len(best)} ICs -> keeping {len(best_keep)}')
"""
B2 = A2 + """    if args.one_per_ic_first:
        if args.one_per_ic_best:
            raise SystemExit('[to_dreamer_native] --one-per-ic-best and --one-per-ic-first are mutually exclusive')
        first = {}
        for f in files:
            z = np.load(f)
            u = int(z['ic_uid']) if 'ic_uid' in z.files else int(z['uid'])
            if 'uid' not in z.files:
                raise SystemExit(f'[to_dreamer_native] --one-per-ic-first needs the rollout uid to order attempts; {f} has none')
            key = int(z['uid'])                      # attempt order == the sequential rollout uid
            if u not in first or key < first[u][0]:
                first[u] = (key, f)
        best_keep = {v[1] for v in first.values()}
        nrew = sum(float(np.asarray(np.load(f)['rewards'], np.float32).sum()) for f in sorted(best_keep))
        print(f'[to_dreamer_native] --one-per-ic-first: {len(files)} tapes over {len(first)} ICs -> keeping {len(best_keep)} (sum reward {nrew:.0f})')
"""

A3 = "one_per_ic_best=bool(args.one_per_ic_best),"
B3 = "one_per_ic_best=bool(args.one_per_ic_best), one_per_ic_first=bool(args.one_per_ic_first),"

for a, b in ((A1, B1), (A2, B2), (A3, B3)):
    if s.count(a) != 1:
        raise SystemExit(f'[first_flag] {p}: anchor count {s.count(a)} != 1 for {a[:60]!r}')
    s = s.replace(a, b)
io.open(p, 'w', encoding='utf-8').write(s)
print(f'[first_flag] {p}: PATCHED')
