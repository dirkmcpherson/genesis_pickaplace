#!/usr/bin/env python
"""END-TO-END (full-task) demonstrations for RLPD and DP (PHASE_PLAN_2026-09-04 amendment (n), 2026-09-07).

The world model's end-to-end arm (amendment (d), PHASE_RESULTS §5/§5.1) trained on two r2dreamer-native
FULL-scope sets: human `dHfull_all` (74 tapes, every full-task tape of the 74 success uids, incl. 10 no-picks)
and machine `dDPfull` (72 tapes = the 195-tape dDP harvest reduced to one tape per IC by highest recorded reward,
`to_dreamer_native.py --one-per-ic-best`). This file lets the OTHER TWO learners train on exactly those tapes:

  (1) segment_transitions_full(seg_dir, expect) -- the RLPD demo half, read from the r2dreamer-native segment dir
      VERBATIM (the same rows the world model trained on, no re-encoding):
        per tape  state (T,17)   action (T,7) backward-shifted (action[t] led INTO state[t]; action[0] == 0)
                  reward (T,)    the STAGED grants as recorded (picked 1 / placed 1 / contact 2 / nested 4,
                                 each once; several rows may be non-zero and a row may carry 6.0 = contact+nested)
                  is_terminal (T,)  True only where the recorded episode actually terminated (nested / tip);
                                 a cap-truncated tape carries no terminal row at all.
      -> transitions t = 0..T-2: (state[t], action[t+1], reward[t+1], state[t+1], is_terminal[t+1]).
      This differs from place_demos.segment_transitions ONLY in the reward/terminal shape: the place phase pays a
      single +1 on the last row, the full task pays the staged ladder wherever the recording earned it. Every other
      invariant (shapes, |a| <= 1, action[0] == 0, finite states) is asserted identically.

  (2) select -- the DP dataset: the FULL contract-v1 tapes those segments were made from, copied unchanged (no
      cut, no pruning) into <out>/<6-digit rollout>.npz + manifest.json (contract 'v1', scope 'full', sim_variant,
      n_kept, content_sha256), so baselines/convert_to_lerobot.py (contract-v1 aware, fps 30/action_repeat = 7.5)
      and the sbatch_dp.sh-style provenance gates accept it unchanged. The segment -> source mapping is by the
      recorded rollout `uid` (the segment file name is genesis-{uid:06d}-{k:03d}-{T}.npz) and is verified twice:
      exactly one source tape per uid, and T == n + 1 (the segment carries one extra state row).

  (3) check -- cross-check (2) against (1) tape by tape: same count, T == n + 1, first state equal, final state
      equal, actions_delta == action[1:], reward sums equal.

  (4) census -- counts, rows, staged-reward sums, stage yields and the IDLE FRACTION of both sets (the
      prune_full_v1 rule: decision t+1 is idle iff max |actions[t+1] - actions[t]| < 1e-3 on the absolute joint
      targets). Amendment (n) disconfirm branch (iii) registers the idle asymmetry as a DP confound, so the number
      is measured and reported rather than assumed.

Usage:
  python baselines/rl/full_demos.py select --segments $W/demos_state_full/dHfull_all \
      --src $W/demos_state_full/src_dHfull_all --out .../matched_w3/dHfull_all
  python baselines/rl/full_demos.py check  --raw .../matched_w3/dHfull_all --segments $W/demos_state_full/dHfull_all
  python baselines/rl/full_demos.py census --segments $W/demos_state_full/dHfull_all [--raw .../dHfull_all]
"""
import argparse, glob, hashlib, json, os, shutil, sys, time
import numpy as np

STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)
IDLE_EPS = 1e-3   # prune_full_v1.py's rule, on the ABSOLUTE window-end joint targets


# ----------------------------------------------------------------------------------------------- (1) RLPD
def segment_meta(seg_dir):
    m = json.load(open(os.path.join(seg_dir, 'repeat.json')))
    files = sorted(glob.glob(os.path.join(seg_dir, '*.npz')))
    assert files, f'no npz in {seg_dir}'
    assert int(m['n_written']) == len(files), (seg_dir, m['n_written'], len(files))
    return m, files


def seg_uid(path):
    """genesis-{uid:06d}-{k:03d}-{T}.npz -> (uid, T); uid is the recorded ROLLOUT id, not the IC uid."""
    parts = os.path.basename(path).replace('.npz', '').split('-')
    assert len(parts) == 4 and parts[0] == 'genesis', path
    return int(parts[1]), int(parts[3])


def segment_transitions_full(seg_dir, expect):
    """-> (transitions [(obs, a, r, next_obs, done)], census) for a FULL-scope r2dreamer-native segment dir.

    expect: dict of repeat.json stamps that MUST match this run (silent-default rule); an empty dict skips the
    check (census use only). Rewards are the recorded staged grants; terminals are the recorded terminals."""
    m, files = segment_meta(seg_dir)
    for k, want in expect.items():
        got = m.get(k)
        ok = ((abs(float(got) - float(want)) < 1e-9)
              if isinstance(want, (int, float)) and not isinstance(want, bool) else (str(got) == str(want)))
        assert ok, f'{seg_dir}/repeat.json {k}={got!r} but this run expects {want!r}'
    assert m.get('with_state') is True and int(m.get('state_dim') or 0) == 17, m
    assert str(m.get('scope')) == 'full', f'{seg_dir}: scope={m.get("scope")!r}, expected full'
    assert m.get('reward_from_tape') is True, f'{seg_dir}: the staged rewards must come from the tape'
    ladder = {round(v, 6) for v in _reachable_reward_sums()}
    out = []
    census = dict(n_tapes=0, n_transitions=0, n_rewarded=0, n_terminal=0, lens=[], uids=[],
                  reward_total=0.0, reward_values={}, n_multi_reward=0)
    for f in files:
        z = np.load(f)
        st = np.asarray(z['state'], np.float32); ac = np.asarray(z['action'], np.float32)
        rw = np.asarray(z['reward'], np.float32).reshape(-1); term = np.asarray(z['is_terminal'], bool).reshape(-1)
        T = st.shape[0]
        assert st.shape == (T, 17) and ac.shape == (T, 7) and rw.shape == (T,) and term.shape == (T,), (f, st.shape, ac.shape)
        assert T >= 2 and np.isfinite(st).all(), f
        assert np.abs(ac).max() <= 1.0 + 1e-6 and not np.abs(ac[0]).any(), f'{f}: action[0] must be 0 (backward-shifted layout)'
        assert float(rw.min()) >= 0.0, f'{f}: negative reward in a staged-sparse full-task tape'
        for v in np.unique(rw[rw != 0]):
            assert round(float(v), 6) in ladder, f'{f}: reward {v} is not a sum of the staged ladder {sorted(ladder)}'
            census['reward_values'][float(v)] = census['reward_values'].get(float(v), 0) + int((rw == v).sum())
        assert int(term.sum()) <= 1 and (not term.any() or bool(term[-1])), \
            f'{f}: a terminal may only appear on the LAST row (got {np.nonzero(term)[0][:5]} of {T})'
        for t in range(T - 1):
            out.append((st[t], ac[t + 1], float(rw[t + 1]), st[t + 1], bool(term[t + 1])))
        census['n_tapes'] += 1; census['n_transitions'] += T - 1
        census['n_rewarded'] += int((rw[1:] != 0).sum()); census['n_terminal'] += int(term[1:].sum())
        census['reward_total'] += float(rw.sum()); census['n_multi_reward'] += int((rw != 0).sum() > 1)
        census['lens'].append(T - 1); census['uids'].append(seg_uid(f)[0])
    return out, census


def _reachable_reward_sums():
    """Every value a single decision can pay under the staged ladder (any subset of the four stages)."""
    vals = {0.0}
    for r in STAGE_REWARD.values():
        vals |= {v + r for v in vals}
    return sorted(vals)


def print_segment_census(c, tag=''):
    print(f'[segments] {tag}: {c["n_tapes"]} tapes -> {c["n_transitions"]} transitions, Sigma reward '
          f'{c["reward_total"]:.0f} over {c["n_rewarded"]} rewarded transitions ({c["n_multi_reward"]} tapes with '
          f'>1 grant), {c["n_terminal"]} terminal; decisions p50 {int(np.median(c["lens"]))} min {min(c["lens"])} '
          f'max {max(c["lens"])}; {len(set(c["uids"]))} distinct rollouts; reward values '
          f'{ {k: v for k, v in sorted(c["reward_values"].items())} }', flush=True)


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


def idle_stats(z):
    """(n_idle, n_decisions, n_idle_prepick, n_prepick) under the prune_full_v1 rule on absolute targets."""
    a = np.asarray(z['actions'], np.float64); n = int(z['n'])
    a = a[:n]
    if n < 2:
        return 0, n, 0, 0
    idle = np.max(np.abs(a[1:] - a[:-1]), axis=1) < IDLE_EPS   # idle[t] -> decision t+1 repeats t
    picked = np.asarray(z['picked'], bool)[:n] if 'picked' in z.files else np.zeros(n, bool)
    j = int(np.argmax(picked)) if picked.any() else n          # first picked decision, else the whole tape
    return int(idle.sum()), n - 1, int(idle[:max(j - 1, 0)].sum()), max(j - 1, 0)


def cmd_select(args):
    """Copy the FULL contract-v1 source tapes of a segment set into a DP raw set (no cut, no pruning)."""
    m, seg_files = segment_meta(args.segments)
    assert str(m.get('scope')) == 'full', m.get('scope')
    src_files = sorted(glob.glob(os.path.join(args.src, '*.npz')))
    assert src_files, f'no npz in {args.src}'
    by_uid = {}
    for f in src_files:
        z = np.load(f, allow_pickle=True)
        assert 'uid' in z.files, f'{f}: no uid stamp (not a recorder tape?)'
        u = int(z['uid'])
        assert u not in by_uid, f'duplicate rollout uid {u}: {by_uid[u][0]} and {f}'
        by_uid[u] = (f, int(z['n']), int(z['ic_uid']))
    plan = []
    for sf in seg_files:
        u, T = seg_uid(sf)
        assert u in by_uid, f'{os.path.basename(sf)}: no source tape with uid {u} in {args.src}'
        f, n, ic = by_uid[u]
        assert n + 1 == T, f'{os.path.basename(sf)}: segment T={T} but source n={n} (expected T == n + 1)'
        plan.append((f, u, n, ic, T))
    assert len(plan) == len(seg_files) == int(m['n_written']), (len(plan), len(seg_files), m['n_written'])
    print(f'[select] {len(plan)} of {len(src_files)} source tapes matched to {args.segments} '
          f'({len(set(p[3] for p in plan))} distinct ICs)')
    if os.path.exists(args.out) and [x for x in os.listdir(args.out) if x.endswith('.npz')] and not args.force:
        sys.exit(f'FATAL: {args.out} already holds npz (use --force)')
    if args.dry_run:
        for f, u, n, ic, T in plan[:5]:
            print('  ', os.path.basename(f), 'uid', u, 'ic', ic, 'n', n)
        print('[dry-run] nothing written'); return
    os.makedirs(args.out, exist_ok=True)
    written = []; chosen = {}; svs = set(); reps = set(); caps = set(); rows = []
    n_idle = n_dec = n_idle_pp = n_pp = 0
    stages = dict(picked=0, placed=0, contact=0, nested=0, tipped=0)
    r_total = 0.0
    for f, u, n, ic, T in plan:
        z = np.load(f, allow_pickle=True)
        assert str(_scalar(z['scope'])) == 'full' and str(_scalar(z['contract'])) == 'v1', f
        svs.add(str(_scalar(z['sim_variant']))); reps.add(int(z['action_repeat'])); caps.add(round(float(z['delta_cap']), 6))
        stem = f'{u:06d}'
        dst = os.path.join(args.out, f'{stem}.npz')
        assert not os.path.exists(dst) or args.force, f'rollout-index collision {dst}'
        shutil.copyfile(f, dst)                     # the tape travels UNCHANGED (no cut, no re-stamp)
        written.append(dst); rows.append(n)
        r_total += float(np.asarray(z['rewards']).sum()) if 'rewards' in z.files else 0.0
        for k in stages:
            if k in z.files:
                stages[k] += int(bool(np.asarray(z[k], bool)[:n].any()))
        a, b, c, d = idle_stats(z)
        n_idle += a; n_dec += b; n_idle_pp += c; n_pp += d
        chosen[f'{stem}.npz'] = dict(src=os.path.basename(f), rollout_uid=u, ic_uid=ic, n=n, segment_T=T)
    assert len(svs) == 1 and len(reps) == 1 and len(caps) == 1, (svs, reps, caps)
    sv = svs.pop()
    if args.sim_variant and sv != args.sim_variant:
        sys.exit(f'FATAL: tapes are stamped sim_variant={sv}, expected {args.sim_variant}')
    man = dict(set=os.path.basename(os.path.normpath(args.out)), built=time.strftime('%Y-%m-%dT%H:%M:%S'),
               contract='v1', sim_variant=sv, action_repeat=reps.pop(), delta_cap=caps.pop(), scope='full', phase=None,
               role='end-to-end (full task) DP set (PHASE_PLAN amendment (n)); the SAME tapes the world model and '
                    'RLPD train on, unpruned and uncut',
               N=len(written), n_kept=len(written), n_success=len(written), n_fail=0,
               decisions_total=int(sum(rows)), decisions_p50=float(np.median(rows)),
               decisions_min=int(min(rows)), decisions_max=int(max(rows)),
               tape_reward_total=r_total, stage_yields=stages,
               idle_frac=(n_idle / n_dec if n_dec else 0.0), idle_frac_prepick=(n_idle_pp / n_pp if n_pp else 0.0),
               idle_eps=IDLE_EPS, one_per_ic_best=bool(m.get('one_per_ic_best')),
               segments=os.path.abspath(args.segments), segments_sha=m.get('src_sha'), src=os.path.abspath(args.src),
               chosen=chosen, content_sha256=content_sha(written), builder='baselines/rl/full_demos.py select')
    json.dump(man, open(os.path.join(args.out, 'manifest.json'), 'w'), indent=1)
    open(os.path.join(args.out, 'episode_list.txt'), 'w').write('\n'.join(sorted(chosen)) + '\n')
    print(f'[select] wrote {len(written)} full-task tapes -> {args.out} (decisions {sum(rows)}, p50 '
          f'{int(np.median(rows))}, min {min(rows)}, max {max(rows)}; tape reward {r_total:.0f}; stages {stages}; '
          f'idle {man["idle_frac"]:.3f} overall / {man["idle_frac_prepick"]:.3f} pre-pick; sha '
          f'{man["content_sha256"][:16]})')


# ----------------------------------------------------------------------------------------------- (3) check
def cmd_check(args):
    m, seg_files = segment_meta(args.segments)
    raw_files = sorted(glob.glob(os.path.join(args.raw, '*.npz')))
    man = json.load(open(os.path.join(args.raw, 'manifest.json')))
    assert len(raw_files) == len(seg_files) == int(man['n_kept']) == int(m['n_written']), \
        (len(raw_files), len(seg_files), man['n_kept'], m['n_written'])
    segs = {seg_uid(f)[0]: f for f in seg_files}
    n_ok = 0; r_seg = 0.0; r_raw = 0.0
    for f in raw_files:
        z = np.load(f, allow_pickle=True); u = int(z['uid']); n = int(z['n'])
        assert u in segs, f'{f}: no segment for rollout uid {u}'
        sz = np.load(segs[u])
        assert sz['state'].shape[0] == n + 1, (f, sz['state'].shape[0], n)
        assert np.allclose(sz['state'][0], z['states'][0], atol=1e-6), f'{f} vs {segs[u]}: first state differs'
        assert np.allclose(sz['state'][-1], z['final_state'], atol=1e-6), f'{f} vs {segs[u]}: final state differs'
        assert np.allclose(sz['action'][1:], np.asarray(z['actions_delta'])[:n], atol=1e-6), \
            f'{f} vs {segs[u]}: actions_delta differ'
        rs = float(np.asarray(sz['reward']).sum()); rr = float(np.asarray(z['rewards'])[:n].sum())
        assert abs(rs - rr) < 1e-6, f'{f}: segment reward {rs} != tape reward {rr}'
        r_seg += rs; r_raw += rr; n_ok += 1
    print(f'CHECK-OK {args.raw} == {args.segments}: {n_ok} tapes, decisions '
          f'{sum(int(np.load(f, allow_pickle=True)["n"]) for f in raw_files)} == '
          f'{sum(int(np.load(f)["state"].shape[0]) - 1 for f in seg_files)} segment transitions; Sigma reward '
          f'{r_seg:.0f} (tape {r_raw:.0f}); manifest sha {man["content_sha256"][:16]}')


# ----------------------------------------------------------------------------------------------- (4) census
def cmd_census(args):
    _, c = segment_transitions_full(args.segments, {})
    print_segment_census(c, args.segments)
    if args.raw:
        man = json.load(open(os.path.join(args.raw, 'manifest.json')))
        print(f'[raw] {args.raw}: N={man["n_kept"]} decisions={man["decisions_total"]} '
              f'tape_reward={man["tape_reward_total"]:.0f} stages={man["stage_yields"]} '
              f'idle={man["idle_frac"]:.3f} (pre-pick {man["idle_frac_prepick"]:.3f}) sha={man["content_sha256"][:16]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('select'); s.add_argument('--segments', required=True); s.add_argument('--src', required=True)
    s.add_argument('--out', required=True); s.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
    s.add_argument('--force', action='store_true'); s.add_argument('--dry-run', action='store_true')
    s.set_defaults(fn=cmd_select)
    k = sub.add_parser('check'); k.add_argument('--raw', required=True); k.add_argument('--segments', required=True)
    k.set_defaults(fn=cmd_check)
    c = sub.add_parser('census'); c.add_argument('--segments', required=True); c.add_argument('--raw', default=None)
    c.set_defaults(fn=cmd_census)
    args = ap.parse_args(); args.fn(args)


if __name__ == '__main__':
    main()
