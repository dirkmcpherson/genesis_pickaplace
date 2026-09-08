#!/usr/bin/env python
"""Place-phase demonstrations for RLPD and DP (PHASE_PLAN_2026-09-04 amendment (h), 2026-09-07).

One file, three jobs, so the RLPD demo half and the DP dataset are cut from the SAME phase boundaries
(`<prefix>_phases.json` from make_phase_banks.py: k_pick, k_placed_v2 per FULL-scope contract-v1 tape):

  (1) segment_transitions(seg_dir, expect)  -- RLPD demo half from an r2dreamer-native SEGMENT dir
      (to_dreamer_native.py --phase place --with-state --state-only; demos_state/{dH_place,dDP_place_n39}):
      per tape  state (T,17)  action (T,7) backward-shifted (action[t] led INTO state[t]; action[0]=0)
                reward (T,)   +1 on the last row only          is_terminal (T,)  [-1] True
      -> transitions t = 0..T-2: (state[t], action[t+1], reward[t+1], state[t+1], is_terminal[t+1]).
      The SAME rows r2dreamer trained on, no re-encoding; repeat.json stamps are asserted against the run.

  (2) cut  -- DP dataset: the FULL contract-v1 tapes cut to rows [k_pick, k_placed_v2] (inclusive; row k = the
      obs BEFORE decision k, the bank convention) and written as contract-v1-shaped npz (states / actions =
      ABSOLUTE window-end joint targets + grip 0..1 / actions_delta / rewards / terminated / truncated /
      final_state + every recorder stamp) into <out>/<6-digit rollout>.npz + manifest.json (contract 'v1',
      sim_variant, n_kept, content_sha256, cuts), so baselines/convert_to_lerobot.py (contract-v1 aware,
      fps 30/action_repeat) and the sbatch_dp.sh-style provenance gates accept it unchanged.
      --keep-from <repeat.json>: restrict to the tapes of a matched-N dreamer set (its subsample_kept names
      carry (ic_uid, T); with --one-per-ic the FIRST sorted tape per IC that reaches the phase is the one
      to_dreamer_native.py cut, cross-checked by T == rows + 1).

  (3) check -- cross-check (2) against (1): same tape count, per tape rows == T-1, first state equal,
      actions_delta == action[1:], exactly one +1 per tape.

Usage:
  python baselines/rl/place_demos.py cut --src baselines/demos_v2/dHfull_w3_all --phases $W/phase_banks/human_phases.json \
      --out baselines/matched_w3/dH_place
  python baselines/rl/place_demos.py cut --src baselines/demos_v2/dDPfull_w3_all --phases $W/phase_banks/machine_phases.json \
      --out baselines/matched_w3/dDP_place_n39 --one-per-ic --keep-from $W/demos_state/dDP_place_n39/repeat.json
  python baselines/rl/place_demos.py check --raw baselines/matched_w3/dH_place --segments $W/demos_state/dH_place
  python baselines/rl/place_demos.py census --segments $W/demos_state/dH_place
"""
import argparse, glob, hashlib, json, os, sys, time
import numpy as np

REPO = os.environ.get('GENESIS_PICKAPLACE_ROOT', os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


# ----------------------------------------------------------------------------------------------- (1) RLPD
def segment_meta(seg_dir):
    m = json.load(open(os.path.join(seg_dir, 'repeat.json')))
    files = sorted(glob.glob(os.path.join(seg_dir, '*.npz')))
    assert files, f'no npz in {seg_dir}'
    assert int(m['n_written']) == len(files), (seg_dir, m['n_written'], len(files))
    return m, files


def segment_transitions(seg_dir, expect):
    """-> (transitions [(obs, a, r, next_obs, done)], census). expect: dict(sim_variant, action_repeat, delta_cap,
    phase='place', terminal_reward=1.0); every repeat.json stamp must equal it (silent-default rule)."""
    m, files = segment_meta(seg_dir)
    for k, want in expect.items():
        got = m.get(k)
        ok = (abs(float(got) - float(want)) < 1e-9) if isinstance(want, (int, float)) and not isinstance(want, bool) else (str(got) == str(want))
        assert ok, f'{seg_dir}/repeat.json {k}={got!r} but this run expects {want!r}'
    assert m.get('with_state') is True and int(m.get('state_dim') or 0) == 17, m
    out = []; census = dict(n_tapes=0, n_transitions=0, n_rewarded=0, n_terminal=0, lens=[], uids=[])
    for f in files:
        z = np.load(f)
        st = np.asarray(z['state'], np.float32); ac = np.asarray(z['action'], np.float32)
        rw = np.asarray(z['reward'], np.float32).reshape(-1); term = np.asarray(z['is_terminal'], bool).reshape(-1)
        T = st.shape[0]
        assert st.shape == (T, 17) and ac.shape == (T, 7) and rw.shape == (T,) and term.shape == (T,), (f, st.shape, ac.shape)
        assert T >= 2 and np.isfinite(st).all(), f
        assert float(rw.sum()) == 1.0 and rw[-1] == 1.0 and not rw[:-1].any(), f'{f}: one +1 on the last row expected'
        assert bool(term[-1]) and not term[:-1].any(), f'{f}: terminal on the last row only'
        assert np.abs(ac).max() <= 1.0 + 1e-6 and not np.abs(ac[0]).any(), f'{f}: action[0] must be 0 (backward-shifted layout)'
        for t in range(T - 1):
            out.append((st[t], ac[t + 1], float(rw[t + 1]), st[t + 1], bool(term[t + 1])))
        census['n_tapes'] += 1; census['n_transitions'] += T - 1; census['n_rewarded'] += 1; census['n_terminal'] += 1
        census['lens'].append(T - 1); census['uids'].append(int(os.path.basename(f).split('-')[1]))
    return out, census


def print_segment_census(c, tag=''):
    print(f'[segments] {tag}: {c["n_tapes"]} tapes -> {c["n_transitions"]} transitions, {c["n_rewarded"]} rewarded (+1 each), '
          f'{c["n_terminal"]} terminal; decisions p50 {int(np.median(c["lens"]))} min {min(c["lens"])} max {max(c["lens"])}; '
          f'{len(set(c["uids"]))} distinct ICs', flush=True)


# ----------------------------------------------------------------------------------------------- (2) DP
def _scalar(x):
    x = np.asarray(x)
    return x.item() if x.shape == () else x


def content_sha(paths):
    h = hashlib.sha256()
    for p in sorted(paths, key=os.path.basename):
        h.update(os.path.basename(p).encode())
        with open(p, 'rb') as fh:
            h.update(fh.read())
    return h.hexdigest()


def cut_one(z, k0, k1, phase_name='place'):
    """FULL contract-v1 tape (np.load'ed) -> dict of the [k0, k1] phase segment in contract-v1 layout."""
    n_tape = int(z['n']); rep = int(z['action_repeat'])
    assert 0 <= k0 < k1 < n_tape, (k0, k1, n_tape)
    n = k1 - k0 + 1
    S = np.asarray(z['states'], np.float32); A = np.asarray(z['actions'], np.float32); AD = np.asarray(z['actions_delta'], np.float32)
    fs = S[k1 + 1] if k1 + 1 < n_tape else np.asarray(z['final_state'], np.float32)
    out = dict(states=S[k0:k1 + 1], actions=A[k0:k1 + 1], actions_delta=AD[k0:k1 + 1], final_state=fs.astype(np.float32),
               rewards=np.zeros(n, np.float32), terminated=np.zeros(n, bool), truncated=np.zeros(n, bool), n=np.int64(n))
    out['rewards'][-1] = 1.0; out['terminated'][-1] = True
    for k in ('picked', 'placed', 'contact', 'nested', 'tipped'):
        if k in z.files:
            out[k] = np.asarray(z[k], bool)[k0:k1 + 1]
    if 'eef_pos' in z.files:
        out['eef_pos'] = np.asarray(z['eef_pos'], np.float32)[k0:k1 + 2]
    if 'sim_states' in z.files and 'sim_actions' in z.files:
        out['sim_states'] = np.asarray(z['sim_states'], np.float32)[k0 * rep:(k1 + 1) * rep]
        out['sim_actions'] = np.asarray(z['sim_actions'], np.float32)[k0 * rep:(k1 + 1) * rep]
    for k in z.files:   # every scalar stamp travels (act_mode, action_repeat, delta_cap, delta_leash, delta_ref, sim_variant, ...)
        if k not in out and np.asarray(z[k]).shape == ():
            out[k] = z[k]
    out['label'] = np.str_('success'); out['stage'] = np.str_(phase_name); out['end_reason'] = np.str_('terminated')
    out['scope'] = np.str_(phase_name); out['phase'] = np.str_(phase_name); out['k_entry'] = np.int64(k0); out['k_grant'] = np.int64(k1)
    assert bool(out['picked'][0]) if 'picked' in out else True, 'the entry row must be past the pick (picked True)'
    return out


def cmd_cut(args):
    phases = json.load(open(args.phases))
    files = sorted(glob.glob(os.path.join(args.src, '*.npz')))
    assert files, f'no npz in {args.src}'
    keep_T = None
    if args.keep_from:
        r = json.load(open(args.keep_from)); kept = r.get('subsample_kept') or []
        assert kept, f'{args.keep_from} has no subsample_kept list'
        keep_T = {}
        for nm in kept:   # genesis-{uid:06d}-{k:03d}-{T}.npz
            parts = os.path.basename(nm).replace('.npz', '').split('-')
            keep_T[int(parts[1])] = int(parts[3])
        print(f'[cut] --keep-from {args.keep_from}: {len(keep_T)} ICs (matched-N set of record)')
    plan = []; seen_ic = set(); n_no_phase = 0; n_dup = 0; n_not_kept = 0
    for f in files:
        base = os.path.basename(f)
        ph = phases.get(base)
        if ph is None:
            z = np.load(f, allow_pickle=True); u = int(z['ic_uid']) if 'ic_uid' in z.files else int(z['uid']); ph = phases.get(str(u))
        k0key, k1key = (('k_pick', 'k_placed_v2') if args.phase == 'place' else ('k_placed_v2', 'k_contact'))
        if not ph or ph.get(k0key) is None or ph.get(k1key) is None or not (ph[k1key] > ph[k0key]):
            n_no_phase += 1; continue
        uid = int(ph['uid'])
        if args.one_per_ic:
            if uid in seen_ic:
                n_dup += 1; continue
            seen_ic.add(uid)
        if keep_T is not None:
            if uid not in keep_T:
                n_not_kept += 1; continue
            assert keep_T[uid] == ph[k1key] - ph[k0key] + 2, (base, uid, keep_T[uid], ph)
        plan.append((f, uid, int(ph[k0key]), int(ph[k1key])))
    print(f'[cut] phase={args.phase}: {len(files)} tapes in {args.src}: {n_no_phase} never reached the phase boundary, {n_dup} duplicate-IC skipped, '
          f'{n_not_kept} not in the matched set, {len(plan)} cut')
    if keep_T is not None:
        assert len(plan) == len(keep_T), (len(plan), len(keep_T))
    assert plan, 'nothing to cut'
    if os.path.exists(args.out) and os.listdir(args.out) and not args.force:
        sys.exit(f'FATAL: {args.out} exists and is not empty (use --force)')
    if args.dry_run:
        for f, uid, k0, k1 in plan[:5]: print('  ', os.path.basename(f), uid, k0, k1, k1 - k0 + 1)
        print('[dry-run] nothing written'); return
    os.makedirs(args.out, exist_ok=True)
    written = []; cuts = {}; svs = set(); reps = set(); caps = set()
    for f, uid, k0, k1 in plan:
        z = np.load(f, allow_pickle=True)
        assert str(_scalar(z['scope'])) == 'full' and str(_scalar(z['contract'])) == 'v1', f
        svs.add(str(_scalar(z['sim_variant']))); reps.add(int(z['action_repeat'])); caps.add(round(float(z['delta_cap']), 6))
        ep = cut_one(z, k0, k1, phase_name=args.phase)
        stem = os.path.basename(f).replace('.npz', '')[-6:]   # the recorder's 6-digit rollout index (merged dirs prefix the source dir)
        assert stem.isdigit() and len(stem) == 6, (f, stem)
        dst = os.path.join(args.out, f'{stem}.npz')
        assert not os.path.exists(dst), f'rollout-index collision {dst} (two source dirs share an index?)'
        np.savez_compressed(dst, **ep); written.append(dst)
        cuts[f'{stem}.npz'] = dict(src=os.path.basename(f), ic_uid=uid, k_pick=k0, k_placed_v2=k1, rows=k1 - k0 + 1)
    assert len(svs) == 1 and len(reps) == 1 and len(caps) == 1, (svs, reps, caps)
    sv = svs.pop()
    if args.sim_variant and sv != args.sim_variant:
        sys.exit(f'FATAL: tapes are stamped sim_variant={sv}, expected {args.sim_variant}')
    rows = [c['rows'] for c in cuts.values()]
    man = dict(set=os.path.basename(os.path.normpath(args.out)), built=time.strftime('%Y-%m-%dT%H:%M:%S'), contract='v1', sim_variant=sv,
               action_repeat=reps.pop(), delta_cap=caps.pop(), scope=args.phase, phase=args.phase,
               role=f'{args.phase}-phase DP set (PHASE_PLAN amendment {"(h)" if args.phase == "place" else "(m)"})',
               N=len(written), n_kept=len(written), n_success=len(written), n_fail=0, decisions_total=int(sum(rows)), decisions_p50=float(np.median(rows)),
               decisions_min=int(min(rows)), decisions_max=int(max(rows)), one_per_ic=bool(args.one_per_ic), keep_from=(os.path.abspath(args.keep_from) if args.keep_from else None),
               src=os.path.abspath(args.src), phases_json=os.path.abspath(args.phases), cuts=cuts, chosen=sorted(os.path.basename(p) for p in written),
               content_sha256=content_sha(written), builder='baselines/rl/place_demos.py cut')
    json.dump(man, open(os.path.join(args.out, 'manifest.json'), 'w'), indent=1)
    open(os.path.join(args.out, 'episode_list.txt'), 'w').write('\n'.join(man['chosen']) + '\n')
    print(f'[cut] wrote {len(written)} {args.phase} tapes -> {args.out} (rows total {sum(rows)}, p50 {int(np.median(rows))}, '
          f'min {min(rows)}, max {max(rows)}; sha {man["content_sha256"][:16]})')


# ----------------------------------------------------------------------------------------------- (3) check
def cmd_check(args):
    m, seg_files = segment_meta(args.segments)
    raw_files = sorted(glob.glob(os.path.join(args.raw, '*.npz')))
    man = json.load(open(os.path.join(args.raw, 'manifest.json')))
    assert len(raw_files) == len(seg_files) == int(man['n_kept']), (len(raw_files), len(seg_files), man['n_kept'])
    segs = {}
    for f in seg_files:
        z = np.load(f); segs.setdefault(int(os.path.basename(f).split('-')[1]), []).append((f, z))
    n_ok = 0
    for f in raw_files:
        z = np.load(f, allow_pickle=True); uid = int(z['ic_uid']); n = int(z['n'])
        cands = [(sf, sz) for sf, sz in segs.get(uid, []) if sz['state'].shape[0] == n + 1]
        assert len(cands) == 1, f'{f}: {len(cands)} segment(s) with uid {uid} and T={n + 1}'
        sf, sz = cands[0]
        assert np.allclose(sz['state'][0], z['states'][0], atol=1e-6), f'{f} vs {sf}: first state differs'
        assert np.allclose(sz['state'][-1], z['final_state'], atol=1e-6), f'{f} vs {sf}: final state differs'
        assert np.allclose(sz['action'][1:], z['actions_delta'], atol=1e-6), f'{f} vs {sf}: actions_delta differ'
        assert float(sz['reward'].sum()) == 1.0 == float(z['rewards'].sum()), f'{f}: reward'
        n_ok += 1
    print(f'CHECK-OK {args.raw} == {args.segments}: {n_ok} tapes, rows {sum(int(np.load(f)["n"]) for f in raw_files)} == '
          f'{sum(int(np.load(f)["state"].shape[0]) - 1 for f in seg_files)} segment transitions; manifest sha {man["content_sha256"][:16]}')


def cmd_census(args):
    tr, c = segment_transitions(args.segments, {})
    print_segment_census(c, args.segments)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('cut'); c.add_argument('--src', required=True); c.add_argument('--phases', required=True); c.add_argument('--out', required=True)
    c.add_argument('--phase', choices=['place', 'contact'], default='place',
                   help="place = [k_pick, k_placed_v2] (default, unchanged); contact = [k_placed_v2, k_contact], the SLIDE "
                        "phase of PHASE_PLAN amendment (m): entry = the released placed-on-shelf state")
    c.add_argument('--keep-from', default=None); c.add_argument('--one-per-ic', action='store_true'); c.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
    c.add_argument('--force', action='store_true'); c.add_argument('--dry-run', action='store_true'); c.set_defaults(fn=cmd_cut)
    k = sub.add_parser('check'); k.add_argument('--raw', required=True); k.add_argument('--segments', required=True); k.set_defaults(fn=cmd_check)
    s = sub.add_parser('census'); s.add_argument('--segments', required=True); s.set_defaults(fn=cmd_census)
    args = ap.parse_args(); args.fn(args)


if __name__ == '__main__':
    main()
