#!/usr/bin/env python
"""Regenerate THE canonical results table from job artifacts. Never quote results from memory.

Reads SWEEP-HEADLINE (RLPD) and HEADLINE.txt (DP) lines out of an artifacts tree and prints the
per-cell table. Excludes smoke runs (seed 0, WAVE=smoke) explicitly -- including them silently
dragged the dH rows down in a hand re-derivation on 08-25.

  python analysis/results_table.py --artifacts ~/workspace/final_rr_artifacts_2026-08-24 [--md out.md]

World is inferred from the seed block (the project's convention): seeds 10-14 = old-world
matched_v2, seeds 20-29 = corrected-world matched_w3. Any seed outside those blocks is printed
under world='?' rather than guessed.
"""
import argparse, collections, glob, os, re, statistics as st

def world_of(seed, path=''):
    """World = demo-set world. Seed blocks: 10-14 old (matched_v2), 20-29 corrected (matched_w3),
    40-42 split halves (matched_w3), 80-93/100-107 r2d old-world matched_v2. The wave directory
    overrides the seed block where it is unambiguous (dp_pilotw4 = ts5 world)."""
    if 'pilotw4' in path or '_w4' in path: return 'corrected+ts5'
    if 10 <= seed <= 19: return 'old'      # 14-19 = 08-28 RLPD old-world top-up
    if 20 <= seed <= 29 or 40 <= seed <= 42: return 'corrected'
    if 80 <= seed <= 93 or 100 <= seed <= 107: return 'old'
    return '?'

def collect(root):
    rows = collections.defaultdict(dict)          # key -> {seed: (seed, hold, rnd)}; dedup by seed
    def put(key, seed, h, r):
        if seed == 0: return                        # smoke
        rows[key].setdefault(seed, (seed, h, r))    # first sighting wins (mirror copies are identical)
    for f in sorted(glob.glob(os.path.join(root, '**', '*.out'), recursive=True)):
        for line in open(f, errors='ignore'):
            m = re.search(r'SWEEP-HEADLINE arm=(\S+) seed=(\d+) reward=(\S+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
            if m:
                arm, seed, rw, h, hn, r, rn = m.groups(); seed = int(seed)
                put(('RLPD', world_of(seed, f), rw, arm), seed, int(h)/int(hn), int(r)/int(rn)); continue
            m = re.search(r'R2D-RESULT arm=(\S+) seed=(\d+) .*?picked=([0-9.]+)', line)
            if m:                                   # sel readout only: selection set, 14/15 ceiling
                arm, seed, p = m.groups(); seed = int(seed)
                put(('r2d(SEL-ONLY, not a headline)', world_of(seed, f), 'dense', arm), seed, float(p), float('nan'))
    for f in sorted(glob.glob(os.path.join(root, '**', 'HEADLINE.txt'), recursive=True)):
        m = re.search(r'arm=(\S+) seed=(\d+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', open(f).read())
        if not m: continue
        arm, seed, h, hn, r, rn = m.groups(); seed = int(seed)
        put(('DP', world_of(seed, f), '-', arm), seed, int(h)/int(hn), int(r)/int(rn))
    resc = collections.defaultdict(dict)          # (arm, seed) -> {set: frac}
    for f in sorted(glob.glob(os.path.join(root, '**', '*.log'), recursive=True)):
        for line in open(f, errors='ignore'):
            m = re.search(r'RESCORE-RESULT tag=\S*?_(dR2DDPfails|dR2D|dH|dDP)_s(\d+) set=(hold|rnd) picked=(\d+)/(\d+) expected=(\d+)', line)
            if not m: continue
            arm, seed, st_, k, n, exp = m.groups()
            if int(n) != int(exp): continue          # asserted denominator failed
            resc[(arm, int(seed))][st_] = int(k)/int(n)
    for (arm, seed), d in resc.items():
        if 'hold' in d and 'rnd' in d:
            put(('r2dreamer', world_of(seed), 'dense', arm), seed, d['hold'], d['rnd'])
    return {k: list(v.values()) for k, v in rows.items()}

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--artifacts', required=True); ap.add_argument('--md')
    a = ap.parse_args(); rows = collect(os.path.expanduser(a.artifacts))
    L = ['| learner | world | reward | arm | n | hold | rnd | per-seed hold |', '|---|---|---|---|---|---|---|---|']
    for k in sorted(rows):
        v = sorted(rows[k]); h = [x[1] for x in v]
        rnd = f'{st.mean([x[2] for x in v]):.2f}' if v[0][2] == v[0][2] else 'n/a'
        L.append(f'| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {len(v)} | {st.mean(h):.2f} | {rnd} | '
                 + ' '.join(f'{x:.2f}' for x in h) + ' |')
    out = '\n'.join(L)
    print(out)
    if a.md:
        open(a.md, 'w').write('# Canonical results table (regenerate: analysis/results_table.py)\n\n'
                              'Smoke runs (seed 0) excluded. World from the seed block: 10-14 old (matched_v2), '
                              '20-29 corrected (matched_w3), 40-42 split halves; r2dreamer rows are RESCORE (hold/rnd) readouts; '
                              'r2d(SEL-ONLY) rows are the selection-set readout and are NOT headlines. Dedup by (arm, seed).\n\n' + out + '\n')

if __name__ == '__main__':
    main()
