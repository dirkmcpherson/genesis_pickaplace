#!/usr/bin/env python
"""Split a contract-v1 demo set into two disjoint IC-stratified halves.

WHY (user, 2026-08-26): "all that matters is the result is robust to the human demo version,
otherwise this is bad science." Changing the WORLD necessarily changes the demos (they are
recordings made in it), so world-vs-version cannot be separated by re-recording. A split-half
CAN separate draw-from-everything-else: same world, same recorder, same follower rule, same
teacher -- only WHICH demos you happen to hold differs. If a source ordering survives both
halves it is not an artifact of the particular draw; if it flips, the finding is draw-specific
and must be reported as such.

Halves are stratified by ic_uid so both cover the IC space (an unstratified split could hand one
half all the easy ICs). Odd counts put the extra tape in half A. BC-safe and RL-safe: whole tapes
are moved, nothing is truncated, so the (s,a,s') chain is intact in both halves.

  python baselines/make_split_halves.py --src baselines/matched_w4/dH --dst-a ..._A --dst-b ..._B --seed 0
"""
import argparse, glob, hashlib, json, os, shutil, sys, time
import numpy as np

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', required=True); ap.add_argument('--dst-a', required=True); ap.add_argument('--dst-b', required=True)
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--dry-run', action='store_true'); ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.src, '*.npz')))
    if not files: sys.exit(f'FATAL: no npz in {a.src}')
    by_ic = {}
    for f in files:
        z = np.load(f, allow_pickle=True)
        by_ic.setdefault(int(z['ic_uid']), []).append((os.path.basename(f), f))
    rng = np.random.default_rng(a.seed)
    ics = sorted(by_ic); rng.shuffle(ics)
    A, B = [], []
    for i, u in enumerate(ics):                     # alternate whole ICs -> both halves span the IC space
        (A if i % 2 == 0 else B).extend(by_ic[u])
    print(f'[split] {a.src}: {len(files)} tapes over {len(ics)} ICs -> A {len(A)} / B {len(B)}')
    if a.dry_run: return
    src_man = os.path.join(a.src, 'manifest.json')
    base = json.load(open(src_man)) if os.path.exists(src_man) else {}
    for dst, part, tag in ((a.dst_a, A, 'A'), (a.dst_b, B, 'B')):
        if os.path.exists(dst):
            if not a.force: sys.exit(f'FATAL: {dst} exists (--force)')
            shutil.rmtree(dst)
        os.makedirs(dst); h = hashlib.sha256()
        for name, path in sorted(part):
            shutil.copy2(path, os.path.join(dst, name))
            with open(os.path.join(dst, name), 'rb') as fh: h.update(name.encode()); h.update(fh.read())
        json.dump(dict(set=os.path.basename(dst), built=time.strftime('%Y-%m-%dT%H:%M:%S'),
                       N=len(part), n_kept=len(part), contract='v1', split_half=tag, split_seed=a.seed,
                       sim_variant=base.get('sim_variant'), source=os.path.normpath(a.src),
                       ic_uids=sorted({int(np.load(p, allow_pickle=True)['ic_uid']) for _, p in part}),
                       content_sha256=h.hexdigest(), builder='baselines/make_split_halves.py',
                       PURPOSE='demo-draw robustness: same world/rule/recorder, disjoint tapes'),
                  open(os.path.join(dst, 'manifest.json'), 'w'), indent=1)
        print(f'  wrote {dst}  N={len(part)}  sha {h.hexdigest()[:16]}')

if __name__ == '__main__':
    main()
