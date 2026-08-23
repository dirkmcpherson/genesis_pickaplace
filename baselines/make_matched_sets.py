#!/usr/bin/env python
"""Build the MATCHED demo sets of the final round robin from three contract-v1 recordings.

WHY (paper/PREREG_final_round_robin_2026-08-23.md §3.2): dH / dDP / dR2D must differ in
the demonstrator's BEHAVIOUR and in nothing the builder controls. Every set is recorded by
the same recorder through the same env (baselines/record_demos.py, contract v1); here we
match N (= min over sources of success tapes, capped), match IC coverage (same ic_uids
with the same multiplicity, as far as the recordings allow), keep successes only in the
primary sets, and fingerprint everything so each sbatch can assert what it trained on.

Frames are NOT matched (intrinsic: humans are slow); the census reports them.

Inputs: three dirs of contract-v1 npz (successes, as the recorder writes them) and
optionally the dDP fails dir. Outputs under --out-root:

  <out-root>/dH/*.npz                 chosen tapes (hard links when possible, else copies)
  <out-root>/dH/episode_list.txt      sorted filenames
  <out-root>/dH/manifest.json         {source, N, chosen, ic_uid histogram, content_sha256,
                                       source manifest, seed, git, unmatched notes}
  ... same for dDP, dR2D
  <out-root>/dDPfails/...                 (when --fails-dDP) the N dDP successes + dDP fails,
                                       fail share <= --fail-share of episodes (PREREG §3.3)
  <out-root>/dR2DDPfails/...           the N dR2D successes + the same dDP fails
  <out-root>/MATCHED_SETS.json        shas of every set + the census command + stats

Matching rule (deterministic, rng(--seed)):
  per ic_uid u: common capacity c(u) = min over sources of #success tapes with ic_uid u.
  If sum_u c(u) >= N: draw N slots round-robin over uids (random uid order), never more
  than c(u) per uid -> the three sets have IDENTICAL ic_uid multisets. Otherwise take
  all common capacity and fill each source's remainder from its own leftovers
  (reported under manifest['unmatched']). Within a uid, tapes are drawn uniformly.

Usage (numpy only):
  python baselines/make_matched_sets.py --dH baselines/rec_dH --dDP baselines/rec_dDP \
      --dR2D baselines/rec_dR2D --fails-dDP baselines/rec_dDP_fails \
      --out-root baselines/matched_sets --seed 0 --cap-n 66 --census
"""
import argparse, glob, hashlib, json, os, shutil, subprocess, sys, time
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

V1_KEYS = ('states', 'actions_delta', 'actions', 'rewards', 'terminated', 'truncated', 'picked')
try:   # the recorder's own contract checker (numpy-only module); the builder refuses tapes it rejects
    sys.path.insert(0, os.path.join(REPO, 'baselines'))
    from record_demos import validate_tape   # noqa: E402
except Exception as _e:   # pragma: no cover
    validate_tape = None
    print(f'WARN: record_demos.validate_tape unavailable ({_e}); only the builder\'s minimal checks run', file=sys.stderr)


def git_sha():
    try:
        return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()
    except Exception:
        return None


def read_dir(path, want_label=None, validate=True):
    """-> list of dict(name, path, label, ic_uid, n, tipped)."""
    out = []
    for f in sorted(glob.glob(os.path.join(path, '*.npz'))):
        z = np.load(f, allow_pickle=True)
        miss = [k for k in V1_KEYS if k not in z.files]
        if miss:
            sys.exit(f'FATAL: {f}: not a contract-v1 tape (missing {miss}); matched sets are built from recorder output only')
        if validate and validate_tape is not None:
            errs = validate_tape(z)
            if errs:
                sys.exit(f'FATAL: {f}: contract violations {errs[:5]} -- re-record; the builder never repairs tapes')
        picked = np.asarray(z['picked'], bool).reshape(-1)
        term = np.asarray(z['terminated'], bool).reshape(-1)
        trunc = np.asarray(z['truncated'], bool).reshape(-1)
        label = str(z['label'].item()) if 'label' in z.files else ('success' if picked[-1] else 'fail')
        if (label == 'success') != bool(picked[-1]):
            sys.exit(f'FATAL: {f}: label={label} but picked[-1]={bool(picked[-1])} (contract rule: success iff last row picked)')
        if not (term[-1] or trunc[-1]):
            sys.exit(f'FATAL: {f}: last row is neither terminated nor truncated')
        ic = z['ic_uid'].item() if 'ic_uid' in z.files else None
        out.append(dict(name=os.path.basename(f), path=f, label=label,
                        ic_uid=(int(ic) if ic is not None and str(ic) not in ('None', '') else None),
                        n=int(len(picked)), tipped=bool(np.asarray(z['tipped'], bool).reshape(-1)[-1]) if 'tipped' in z.files else None))
    if want_label:
        bad = [t['name'] for t in out if t['label'] != want_label]
        if bad:
            sys.exit(f'FATAL: {path}: expected only {want_label} tapes, found {len(bad)} others (e.g. {bad[:3]}); '
                     f'the recorder splits successes and fails into separate dirs -- point at the right one')
    if not out:
        sys.exit(f'FATAL: no npz in {path}')
    return out


def content_sha(paths):
    h = hashlib.sha256()
    for p in sorted(paths, key=os.path.basename):
        h.update(os.path.basename(p).encode())
        with open(p, 'rb') as fh:
            h.update(fh.read())
    return h.hexdigest()


def stratified_pick(sources, N, rng):
    """sources: {name: [tape dicts]} (successes). Returns ({name: [tapes]}, notes)."""
    by_uid = {s: {} for s in sources}
    for s, tapes in sources.items():
        for t in tapes:
            by_uid[s].setdefault(t['ic_uid'], []).append(t)
    uids = sorted(set().union(*[set(d) for d in by_uid.values()]), key=lambda u: (u is None, u))
    cap = {u: min(len(by_uid[s].get(u, [])) for s in sources) for u in uids}
    common = int(sum(cap.values()))
    notes = dict(common_capacity=common, N=N, uids_total=len(uids))
    # shuffle within uid, per source
    for s in sources:
        for u in by_uid[s]:
            rng.shuffle(by_uid[s][u])
    order = list(uids); rng.shuffle(order)
    slots = {u: 0 for u in uids}
    take = min(N, common); k = 0
    while k < take:
        progressed = False
        for u in order:
            if k >= take: break
            if slots[u] < cap[u]:
                slots[u] += 1; k += 1; progressed = True
        if not progressed: break
    chosen = {s: [] for s in sources}
    for s in sources:
        for u in uids:
            chosen[s].extend(by_uid[s].get(u, [])[:slots[u]])
    unmatched = {}
    if take < N:
        for s in sources:
            left = [t for u in uids for t in by_uid[s].get(u, [])[slots[u]:]]
            rng.shuffle(left)
            extra = left[:N - take]
            chosen[s].extend(extra)
            unmatched[s] = dict(n_filled_from_own_leftovers=len(extra),
                                ic_uids=sorted(set(t['ic_uid'] for t in extra), key=lambda u: (u is None, u)))
    notes['unmatched'] = unmatched or None
    notes['ic_multiset_identical'] = (take == N)
    return chosen, notes


def write_set(name, tapes, out_root, extra, git, seed, source_paths):
    d = os.path.join(out_root, name); ep = d   # FLAT: npz directly under <out-root>/<set>/ (sbatch_rlpd/sbatch_dp read $DEMO_ROOT/<ARM>/*.npz + manifest.json)
    if os.path.exists(d):
        shutil.rmtree(d)
    os.makedirs(ep)
    renamed = {}
    for t in sorted(tapes, key=lambda t: (t['label'] != 'success', t['name'])):   # successes first keep their names
        out_name = t['name']
        if t['label'] != 'success':
            # fail tapes in a fails arm: project convention = 5xxxxx stems (m1all_harvest); avoids the
            # collision between two recorders that both number rollouts from 100000
            base = 500000 + int(os.path.splitext(t['name'])[0]) % 100000
            while os.path.exists(os.path.join(ep, f'{base:06d}.npz')):
                base += 1
            out_name = f'{base:06d}.npz'
            if out_name != t['name']:
                renamed[t['name']] = out_name
        dst = os.path.join(ep, out_name)
        if os.path.exists(dst):
            sys.exit(f'FATAL: filename collision in {name}: {out_name} (two sources share a rollout index?)')
        t = dict(t, name=out_name)
        try:
            os.link(t['path'], dst)
        except OSError:
            shutil.copy2(t['path'], dst)
    names = sorted(f for f in os.listdir(ep) if f.endswith('.npz'))
    open(os.path.join(d, 'episode_list.txt'), 'w').write('\n'.join(names) + '\n')
    sha = content_sha([os.path.join(ep, n) for n in names])
    hist = {}
    for t in tapes:
        hist[str(t['ic_uid'])] = hist.get(str(t['ic_uid']), 0) + 1
    srcman = {}
    for sp in source_paths:
        mp = os.path.join(sp, 'manifest.json')
        if os.path.exists(mp):
            try: srcman[sp] = json.load(open(mp))
            except Exception as e: srcman[sp] = f'unreadable: {e}'
    man = dict(set=name, built=time.strftime('%Y-%m-%dT%H:%M:%S'), N=len(tapes),
               n_success=sum(t['label'] == 'success' for t in tapes), n_fail=sum(t['label'] != 'success' for t in tapes),
               decisions_total=int(sum(t['n'] for t in tapes)), decisions_p50=float(np.median([t['n'] for t in tapes])),
               n_kept=len(tapes), chosen=names, renamed_fail_tapes=renamed, ic_uid_histogram=dict(sorted(hist.items())), content_sha256=sha,
               sources=source_paths, source_manifests=srcman, seed=seed, git=git,
               builder='baselines/make_matched_sets.py', contract='v1', **extra)
    json.dump(man, open(os.path.join(d, 'manifest.json'), 'w'), indent=1)
    return sha, man


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dH', required=True); ap.add_argument('--dDP', required=True); ap.add_argument('--dR2D', required=True)
    ap.add_argument('--keep-lying-fails', dest='keep_lying_fails', action='store_true', help='keep lying-can-IC 1-decision fail tapes in the fails arms (default: excluded, audit W5)')
    ap.add_argument('--fails-dDP', dest='fails_dDP', default=None, help='dDP fails dir (recorder <outdir>_fails) -> builds the two fails arms')
    ap.add_argument('--out-root', dest='out_root', required=True)
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--cap-n', dest='cap_n', type=int, default=66)
    ap.add_argument('--fail-share', dest='fail_share', type=float, default=0.30, help='max fail share of EPISODES in the fails arms (PREREG: the human no-pick share)')
    ap.add_argument('--census', action='store_true', help='run analysis/characterize_demo_sets.py on the final sets')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--no-validate', dest='validate', action='store_false', help='skip record_demos.validate_tape (tests on synthetic tapes only)')
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed); git = git_sha()

    src = {'dH': read_dir(args.dH, 'success', args.validate), 'dDP': read_dir(args.dDP, 'success', args.validate), 'dR2D': read_dir(args.dR2D, 'success', args.validate)}
    counts = {s: len(v) for s, v in src.items()}
    N = min(min(counts.values()), args.cap_n)
    print(f'[matched] success tapes: {counts} -> N = {N} (cap {args.cap_n})')
    for s, v in src.items():
        nn = sum(t['ic_uid'] is None for t in v)
        if nn: print(f'  WARN {s}: {nn} tapes carry no ic_uid (matched as their own stratum)')
    chosen, notes = stratified_pick(src, N, rng)
    print(f'[matched] common IC capacity {notes["common_capacity"]} vs N {N}; ic multiset identical across sources: {notes["ic_multiset_identical"]}')
    if notes['unmatched']:
        for s, u in notes['unmatched'].items():
            print(f'  {s}: {u["n_filled_from_own_leftovers"]} tapes filled from own leftovers (uids {u["ic_uids"]})')
    fails = read_dir(args.fails_dDP, 'fail', args.validate) if args.fails_dDP else None
    if fails is not None and not args.keep_lying_fails:
        # audit W5: a fail tape of ONE decision whose last row is tipped = the lying-can IC (tip rule at
        # decision 1) -- an IC artifact, not a teacher failure; excluded from the fails arms (disclosed)
        lying = [t for t in fails if t['n'] <= 1 and t.get('tipped')]
        fails = [t for t in fails if not (t['n'] <= 1 and t.get('tipped'))]
        print(f'[matched] fails: excluded {len(lying)} lying-can-IC tapes (n<=1, tipped at t0); {len(fails)} teacher fails remain')
    n_f = 0
    if fails is not None:
        # fail share s = F/(N+F) <= fail_share  ->  F <= fail_share*N/(1-fail_share)
        n_f = min(len(fails), int(np.floor(args.fail_share * N / (1.0 - args.fail_share))))
        rng.shuffle(fails); fails_use = fails[:n_f]
        print(f'[matched] fails arms: {n_f}/{len(fails)} dDP fails (share {n_f/(N+n_f):.2f} <= {args.fail_share}); '
              f'tipped-terminal {sum(bool(t["tipped"]) for t in fails_use)} / cap-truncated {sum(not t["tipped"] for t in fails_use)}')
    if args.dry_run:
        for s in src: print(f'  {s}: {len(chosen[s])} tapes, uids {sorted(set(t["ic_uid"] for t in chosen[s]), key=lambda u: (u is None, u))[:12]}...')
        print('[dry-run] nothing written'); return

    os.makedirs(args.out_root, exist_ok=True)
    shas = {}; mans = {}
    for s, path in (('dH', args.dH), ('dDP', args.dDP), ('dR2D', args.dR2D)):
        shas[s], mans[s] = write_set(s, chosen[s], args.out_root, dict(matching=notes, role='primary success-only'), git, args.seed, [path])
        print(f'  wrote {s}: N={len(chosen[s])} sha {shas[s][:16]}')
    if fails is not None and n_f:
        for s, base in (('dDPfails', 'dDP'), ('dR2DDPfails', 'dR2D')):
            shas[s], mans[s] = write_set(s, chosen[base] + fails_use, args.out_root,
                                         dict(role=f'secondary: {base} successes + {n_f} dDP fails (share {n_f/(N+n_f):.3f})', matching=notes),
                                         git, args.seed, [args.dDP if base == 'dDP' else args.dR2D, args.fails_dDP])
            print(f'  wrote {s}: N={len(chosen[base]) + n_f} sha {shas[s][:16]}')
    census_cmd = ['python', 'analysis/characterize_demo_sets.py'] + [f'{s}={os.path.join(args.out_root, s)}' for s in shas] + \
                 ['--out', os.path.join(args.out_root, 'census.md'), '--json', os.path.join(args.out_root, 'census.json')]
    top = dict(built=time.strftime('%Y-%m-%dT%H:%M:%S'), N=N, cap_n=args.cap_n, seed=args.seed, git=git,
               source_counts=counts, matching=notes, fails_arm_n_fails=n_f,
               sets={s: dict(dir=os.path.join(args.out_root, s), content_sha256=shas[s], N=mans[s]['N']) for s in shas},
               census_command=' '.join(census_cmd))
    json.dump(top, open(os.path.join(args.out_root, 'MATCHED_SETS.json'), 'w'), indent=1)
    print(f'wrote {args.out_root}/MATCHED_SETS.json')
    if args.census:
        print('[matched] running census: ' + ' '.join(census_cmd))
        subprocess.run([sys.executable] + census_cmd[1:], cwd=REPO, check=True)


if __name__ == '__main__':
    main()
