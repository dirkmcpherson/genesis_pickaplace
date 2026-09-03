#!/usr/bin/env python3
"""Build a SAME-SOURCE fails arm: an existing matched success set + that source's own fail tapes.

USER RULE (2026-08-28): never mix demo sources within an arm. The fails contrast is
    dH  vs dH+Hfails      dDP vs dDP+DPfails      dR2D vs dR2D+R2Dfails
(dR2D+DPfails, the N15 arm, is mixed-source and is now a mechanism cell only.)

Reuses make_matched_sets' reader, lying-can exclusion (audit W5), fail-share cap, 5xxxxx stems,
manifest layout and content sha, so the arm is interchangeable with the existing fails arms.

Usage:
  python baselines/make_samesource_fails_arm.py --base baselines/matched_v2/dR2D \
      --fails baselines/demos_v1/dR2Dprov_fails --name dR2DR2Dfails [--fail-share 0.30] [--seed 0]
"""
import argparse, json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from make_matched_sets import read_dir, write_set, git_sha


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True, help='existing matched success set dir (e.g. baselines/matched_v2/dR2D)')
    ap.add_argument('--fails', required=True, help='recorder fails dir of the SAME source')
    ap.add_argument('--name', required=True)
    ap.add_argument('--fail-share', dest='fail_share', type=float, default=0.30)
    ap.add_argument('--keep-lying-fails', dest='keep_lying_fails', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--no-validate', dest='validate', action='store_false')
    a = ap.parse_args()
    out_root = os.path.dirname(os.path.abspath(a.base).rstrip('/'))
    bman = json.load(open(os.path.join(a.base, 'manifest.json')))
    assert bman.get('n_fail', 0) == 0, f'{a.base} is not success-only'
    base = read_dir(a.base, 'success', a.validate)
    raw = read_dir(a.fails, None, a.validate)
    fails = [t for t in raw if t['label'] != 'success']
    dropped = [t['name'] for t in raw if t['label'] == 'success']
    if dropped:
        print(f'[samesource] {a.fails}: dropped {len(dropped)} success-labelled tape(s) found in the fails dir: {dropped}')
    svs = {t.get('sim_variant', 'base') for t in base + fails}
    assert len(svs) == 1, f'world mismatch across base/fails: {svs}'
    if not a.keep_lying_fails:
        lying = [t for t in fails if t['n'] <= 1 and t.get('tipped')]
        fails = [t for t in fails if not (t['n'] <= 1 and t.get('tipped'))]
        print(f'[samesource] excluded {len(lying)} lying-can-IC fail tapes; {len(fails)} remain')
    N = len(base)
    n_f = min(len(fails), int(np.floor(a.fail_share * N / (1.0 - a.fail_share))))
    rng = np.random.default_rng(a.seed); rng.shuffle(fails); use = fails[:n_f]
    lens = [t['n'] for t in use]
    print(f'[samesource] {a.name}: {N} successes + {n_f}/{len(fails)} fails (share {n_f/(N+n_f):.3f}); '
          f'fail rows {sum(lens)} (mean {np.mean(lens):.0f}, cap-truncated {sum(not t["tipped"] for t in use)}, tipped {sum(bool(t["tipped"]) for t in use)}); '
          f'success rows {sum(t["n"] for t in base)}')
    sha, man = write_set(a.name, base + use, out_root,
                         dict(role=f'same-source fails arm: {os.path.basename(a.base)} successes + {n_f} {os.path.basename(a.fails)} fails '
                                   f'(share {n_f/(N+n_f):.3f}); user rule 2026-08-28: never mix sources',
                              matching=bman.get('matching'), base_set=os.path.basename(a.base), base_sha=bman.get('content_sha256'),
                              fail_rows=int(sum(lens)), success_rows=int(sum(t['n'] for t in base)), dropped_success_tapes_in_fails_dir=dropped),
                         git_sha(), a.seed, [a.base, a.fails])
    print(f'  wrote {os.path.join(out_root, a.name)}: N={man["N"]} sha {sha[:16]}')


if __name__ == '__main__':
    main()
