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

def collect(root):
    rows = collections.defaultdict(list)
    def world(seed): return 'old' if 10 <= seed <= 14 else ('corrected' if 20 <= seed <= 29 else '?')
    for f in glob.glob(os.path.join(root, 'outs', '*.out')):
        for line in open(f, errors='ignore'):
            m = re.search(r'SWEEP-HEADLINE arm=(\S+) seed=(\d+) reward=(\S+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
            if not m: continue
            arm, seed, rw, h, hn, r, rn = m.groups(); seed = int(seed)
            if seed == 0: continue                      # smoke
            rows[('RLPD', world(seed), rw, arm)].append((seed, int(h)/int(hn), int(r)/int(rn)))
    for f in glob.glob(os.path.join(root, '**', 'HEADLINE.txt'), recursive=True):
        m = re.search(r'arm=(\S+) seed=(\d+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', open(f).read())
        if not m: continue
        arm, seed, h, hn, r, rn = m.groups(); seed = int(seed)
        if seed == 0: continue
        rows[('DP', world(seed), '-', arm)].append((seed, int(h)/int(hn), int(r)/int(rn)))
    return rows

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--artifacts', required=True); ap.add_argument('--md')
    a = ap.parse_args(); rows = collect(os.path.expanduser(a.artifacts))
    L = ['| learner | world | reward | arm | n | hold | rnd | per-seed hold |', '|---|---|---|---|---|---|---|---|']
    for k in sorted(rows):
        v = sorted(rows[k]); h = [x[1] for x in v]
        L.append(f'| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {len(v)} | {st.mean(h):.2f} | '
                 f'{st.mean([x[2] for x in v]):.2f} | ' + ' '.join(f'{x:.2f}' for x in h) + ' |')
    out = '\n'.join(L)
    print(out)
    if a.md:
        open(a.md, 'w').write('# Canonical results table (regenerate: analysis/results_table.py)\n\n'
                              'Smoke runs (seed 0) excluded. World from the seed block: 10-14 old (matched_v2), '
                              '20-29 corrected (matched_w3).\n\n' + out + '\n')

if __name__ == '__main__':
    main()
