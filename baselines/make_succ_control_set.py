#!/usr/bin/env python3
"""Build the N15 success-content control arm: dR2D successes + the K LONGEST dDP SUCCESS tapes.

WHY (paper/AUDIT_results_2026-08-28.md §1, item 5): the dR2DDPfails arm differs from dR2D in three
things at once -- fail content, total demo rows (2.7x) and tape length (301 vs <=25 rows). Fails are
long BY NATURE in an episodic task, so the whole-treatment observation stands; this arm asks whether
the world model is moved by long, voluminous same-teacher tapes that SUCCEED. If it drops as much as
the fails arm, the reading is "volume/length", not "failure".

Honest limits, written into the manifest: the 8 longest dDP successes total ~1521 rows vs the fails'
2400 (63%), and they are success-terminated (reward 1 on the last row) where fails are truncated.

Usage:
  python baselines/make_succ_control_set.py --root baselines/matched_v2 --k 8 [--name dR2DDPsucc]
Writes <root>/<name>/ (flat npz + manifest.json + episode_list.txt), same layout as make_matched_sets.
The added tapes get 6xxxxx stems (fails use 5xxxxx) so nothing collides.
"""
import argparse, json, os, shutil, subprocess, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from make_matched_sets import content_sha  # same hash convention as every other set


def tape_len(p):
    with np.load(p, allow_pickle=True) as z:
        return int(np.asarray(z['actions_delta']).shape[0]), z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='baselines/matched_v2')
    ap.add_argument('--base', default='dR2D', help='success set to extend')
    ap.add_argument('--donor', default='dDP', help='success set to draw the long tapes from')
    ap.add_argument('--k', type=int, default=8)
    ap.add_argument('--name', default='dR2DDPsucc')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    base, donor, out = (os.path.join(a.root, x) for x in (a.base, a.donor, a.name))
    ref = os.path.join(a.root, 'dR2DDPfails')
    if os.path.exists(out):
        if not a.force: sys.exit(f'FATAL: {out} exists (use --force)')
        shutil.rmtree(out)
    bman, dman = (json.load(open(os.path.join(x, 'manifest.json'))) for x in (base, donor))
    for m, x in ((bman, base), (dman, donor)):
        assert m.get('n_fail', 0) == 0, f'{x} is not success-only'
    assert bman.get('sim_variant', 'base') == dman.get('sim_variant', 'base'), 'world mismatch'
    donors = sorted(((tape_len(os.path.join(donor, f))[0], f) for f in os.listdir(donor) if f.endswith('.npz')), reverse=True)[:a.k]
    os.makedirs(out)
    for f in sorted(os.listdir(base)):
        if f.endswith('.npz'): shutil.copy2(os.path.join(base, f), os.path.join(out, f))
    renamed = {}
    for n, f in donors:
        stem = 600000 + int(os.path.splitext(f)[0]) % 100000
        while os.path.exists(os.path.join(out, f'{stem:06d}.npz')): stem += 1
        renamed[f] = f'{stem:06d}.npz'
        shutil.copy2(os.path.join(donor, f), os.path.join(out, renamed[f]))
    names = sorted(f for f in os.listdir(out) if f.endswith('.npz'))
    open(os.path.join(out, 'episode_list.txt'), 'w').write('\n'.join(names) + '\n')
    lens = {f: tape_len(os.path.join(out, f))[0] for f in names}
    added_rows = sum(n for n, _ in donors)
    fails_rows = None
    if os.path.exists(os.path.join(ref, 'manifest.json')):
        rm = json.load(open(os.path.join(ref, 'manifest.json')))
        fails_rows = int(rm['decisions_total']) - int(bman['decisions_total'])
    hist = {}
    for f in names:
        with np.load(os.path.join(out, f), allow_pickle=True) as z:
            u = str(int(z['ic_uid'])) if 'ic_uid' in z else '?'
        hist[u] = hist.get(u, 0) + 1
    git = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    man = dict(set=a.name, built=time.strftime('%Y-%m-%dT%H:%M:%S'), N=len(names), sim_variant=bman.get('sim_variant', 'base'),
               n_success=len(names), n_fail=0, decisions_total=int(sum(lens.values())), decisions_p50=float(np.median(list(lens.values()))),
               n_kept=len(names), chosen=names, renamed_added_tapes=renamed, added_rows=int(added_rows),
               fails_arm_added_rows=fails_rows, length_match_fraction=(added_rows / fails_rows if fails_rows else None),
               ic_uid_histogram=dict(sorted(hist.items())), content_sha256=content_sha([os.path.join(out, n) for n in names]),
               sources=[base, donor], source_manifests={base: bman, donor: dman}, git=git,
               builder='baselines/make_succ_control_set.py', contract='v1',
               role=f'N15 success-content control: {a.base} successes + the {a.k} longest {a.donor} SUCCESS tapes (6xxxxx stems). '
                    f'Matches the fails arm in tape count and same-teacher provenance; rows {added_rows} vs fails {fails_rows}; '
                    f'success-terminated where fails are truncated. Audit 2026-08-28 §1 item 5.')
    json.dump(man, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'[succ-control] {out}: N={len(names)} rows={man["decisions_total"]} added={added_rows} '
          f'(fails arm added {fails_rows}, match {man["length_match_fraction"]}) sha={man["content_sha256"][:16]}')
    for n, f in donors: print(f'  + {f} -> {renamed[f]} ({n} rows)')


if __name__ == '__main__':
    main()
