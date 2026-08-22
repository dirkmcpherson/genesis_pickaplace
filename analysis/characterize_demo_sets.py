#!/usr/bin/env python
"""Characterize demonstration sets every way we know how (2026-08-22).

Runs anywhere with numpy (cluster login node is fine; no torch/genesis needed). Reads
episodes-format npz sets (keys: states (T,17) f32, actions (T,7) f32, label, stage, uid,
optional images) — the format every RLPD/DP tape in this project uses — and reports, per set
and across sets, the properties that have mattered so far:

  composition   episodes by stage/label, transitions, tape length p50/mean/max, fail share,
                unique ICs (can xy at t0), IC grid occupancy (4x4 over the union of all sets),
                goal xy, image presence
  reward        rewarded transitions (first lift = can z > PICK_Z), reward density, first-lift
                index p50/mean, frames after lift (PICK_TAIL), demo-batch expected rewards
  fidelity      under the RLPD delta_joint encoder (delta_ref=target, cap 0.025, leash 0.125):
                frac frames with any arm dim over cap, leash-exceeded (|cmd - qmeas| > leash),
                one-step label error (|clip(d,cap)-d|, p99/max), zero-delta frames, |delta| p50/p99
  env-consistency  post-termination frames: tilt > TIP_DEG with grip cmd < GRIP_OPEN (the pick-
                scope tip rule) — share of buffer, tapes affected, longest chain, frames where the
                can is already tipped at t0 (lying-can ICs); can max displacement from IC; can z min;
                tapes that are exactly at a cap length (timed-out fails)
  behaviour     gripper closed fraction, grip effort stats, joint-space path length, mean |qdot|
  OOD-ness      nearest-neighbour distance (standardized by each set's success-frame std) of
                FAIL frames to SUCCESS frames within the set (tipped vs not), and a cross-set
                matrix: NN distance of set A's frames (all / fails / successes) to set B's
                success frames (standardized by the pooled success std) — "how far is A from
                what B's learner sees as success"
  quick stats   tilt distribution at t0 / overall; can z at t0 (table height sanity)

Usage (repo root; names are free-form labels, paths are dirs of npz):
  python analysis/characterize_demo_sets.py \
      dH=baselines/episodes_pick_phase_all dDP=baselines/m1all_harvest \
      dR2D=baselines/episodes_champion_pick dDPsucc=baselines/m1all_harvest_succ \
      dHpruned=baselines/episodes_pick_phase_dppruned \
      --out paper/demo_set_census_$(date +%F).md --json paper/demo_set_census_$(date +%F).json
  Options: --nn-samples N (default 6000; NN work is O(N*M)), --no-cross (skip cross-set NN),
           --pick-z 0.1505 --cap 0.025 --leash 0.125 --tip-deg 60 --grip-open 0.3

Dreamer-format sets (the r2dreamer / dv3 demo dirs: npz with image, action, reward, discount,
is_first/is_terminal; no state) are detected automatically and get a COMPOSITION-ONLY table:
episodes, steps, rewarded vs zero-reward (fail) episodes, fail share of steps, lengths, terminal
reward magnitude, stride/repeat stamp (repeat.json), image shape — enough to answer "is this WM
set success-only or does it carry failure tapes?" (design-audit issue 2, 2026-08-22).

Caveats printed into the report: state layout is asserted (17 dims; 0:6 joints, 6 grip, 7 grip
effort, 8:11 can xyz, 11:15 can quat wxyz, 15:17 goal xy); sets in other layouts (cartesian
abs6/delta6 tapes) are reported as 'layout mismatch' and skipped.
"""
import argparse, glob, json, os, sys, time
import numpy as np

STATE_DIM = 17
J = slice(0, 6); GRIP = 6; GRIP_EFF = 7; CAN = slice(8, 11); QUAT = slice(11, 15); GOAL = slice(15, 17)


def tilt_deg(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    zz = 1.0 - 2.0 * (x * x + y * y)
    zx = 2.0 * (x * z + w * y)
    zy = 2.0 * (y * z - w * x)
    return np.degrees(np.arccos(np.clip(zz / (np.sqrt(zx * zx + zy * zy + zz * zz) + 1e-9), -1.0, 1.0)))


def pct(x, p):
    x = np.asarray(x, float)
    return float(np.percentile(x, p)) if x.size else float('nan')


def load_set(path):
    tapes = []
    for f in sorted(glob.glob(os.path.join(path, '*.npz'))):
        z = np.load(f, allow_pickle=True)
        if 'states' not in z.files or 'actions' not in z.files:
            return None, f'{os.path.basename(f)}: keys {list(z.files)} (not episodes-format)'
        s = np.asarray(z['states'], np.float32); a = np.asarray(z['actions'], np.float32)
        if s.ndim != 2 or s.shape[1] != STATE_DIM or a.ndim != 2 or a.shape[1] != 7:
            return None, f'{os.path.basename(f)}: states {s.shape} actions {a.shape} (expected (T,17),(T,7))'
        n = min(len(s), len(a))
        tapes.append(dict(name=os.path.basename(f), s=s[:n], a=a[:n],
                          label=str(z['label'].item()) if 'label' in z.files else '?',
                          stage=str(z['stage'].item()) if 'stage' in z.files else '?',
                          uid=int(z['uid'].item()) if 'uid' in z.files and z['uid'].shape == () else None,
                          images=('images' in z.files)))
    if not tapes:
        return None, 'no npz'
    return tapes, None


def load_dreamer_set(path):
    """Dreamer-format dir -> list of dicts (len, reward array, is_terminal, image shape) or None."""
    eps = []
    for f in sorted(glob.glob(os.path.join(path, '*.npz'))):
        z = np.load(f, allow_pickle=True)
        if 'reward' not in z.files or 'action' not in z.files:
            return None
        r = np.asarray(z['reward'], np.float64).reshape(-1)
        eps.append(dict(name=os.path.basename(f), n=int(len(r)), reward=r,
                        is_terminal=(np.asarray(z['is_terminal']).reshape(-1) if 'is_terminal' in z.files else None),
                        image_shape=(tuple(z['image'].shape[1:]) if 'image' in z.files else None),
                        action_dim=int(np.asarray(z['action']).shape[-1]),
                        has_state=('state' in z.files)))
    if not eps:
        return None
    stamp = None
    rp = os.path.join(path, 'repeat.json')
    if os.path.exists(rp):
        try: stamp = json.load(open(rp))
        except Exception: stamp = {'unreadable': True}
    return eps, stamp


def summarize_dreamer(name, eps, stamp):
    rew_eps = [e for e in eps if (e['reward'] > 0).any()]
    fail = [e for e in eps if not (e['reward'] > 0).any()]
    tot = sum(e['n'] for e in eps); ftot = sum(e['n'] for e in fail)
    term_vals = sorted(set(float(v) for e in rew_eps for v in e['reward'][e['reward'] > 0]))
    first_rew = [int(np.argmax(e['reward'] > 0)) for e in rew_eps]
    return dict(name=name, format='dreamer', n_episodes=len(eps), n_rewarded=len(rew_eps), n_zero_reward=len(fail),
                steps=int(tot), fail_share=(ftot / tot) if tot else float('nan'),
                len_p50=pct([e['n'] for e in eps], 50), len_mean=float(np.mean([e['n'] for e in eps])), len_max=int(max(e['n'] for e in eps)),
                len_rewarded_p50=pct([e['n'] for e in rew_eps], 50), len_fail_p50=pct([e['n'] for e in fail], 50),
                first_reward_idx_p50=pct(first_rew, 50),
                reward_values=term_vals[:6], rewards_per_episode_max=int(max((e['reward'] > 0).sum() for e in eps)),
                terminal_flag_eps=int(sum(bool(e['is_terminal'] is not None and e['is_terminal'].any()) for e in eps)),
                image_shape=str(eps[0]['image_shape']), action_dim=eps[0]['action_dim'], has_state=int(sum(e['has_state'] for e in eps)),
                stamp={k: stamp[k] for k in list(stamp)[:12]} if isinstance(stamp, dict) else None)


def per_tape(t, P):
    s, a = t['s'], t['a']; n = len(s)
    can = s[:, CAN]; tilt = tilt_deg(s[:, QUAT]); grip_cmd = a[:, GRIP]
    lifted = np.where(can[:, 2] > P.pick_z)[0]
    pick = int(lifted[0]) if len(lifted) else -1
    d = np.diff(a[:, J], axis=0); dmax = np.abs(d).max(1) if len(d) else np.zeros(0)
    lab_err = (np.abs(d) - P.cap).clip(min=0).max(1) if len(d) else np.zeros(0)  # one-step unrepresentable part
    lead = np.abs(a[:, J] - s[:, J]).max(1)
    post_term = (tilt > P.tip_deg) & (grip_cmd < P.grip_open)
    runs, c = [], 0
    for v in post_term:
        if v: c += 1
        else:
            if c: runs.append(c); c = 0
    if c: runs.append(c)
    disp = np.linalg.norm(can[:, :2] - can[0, :2], axis=1)
    q = s[:, J]; path = float(np.abs(np.diff(q, axis=0)).sum()) if n > 1 else 0.0
    return dict(
        name=t['name'], uid=t['uid'], label=t['label'], stage=t['stage'], n=n, pick=pick,
        frames_after_pick=(n - pick) if pick >= 0 else None,
        over_cap=float((dmax > P.cap + 1e-6).mean()) if len(dmax) else 0.0,
        zero_delta=float((dmax < 1e-6).mean()) if len(dmax) else 0.0,
        dmax_p50=pct(dmax, 50), dmax_p99=pct(dmax, 99), lab_err_p99=pct(lab_err, 99), lab_err_max=float(lab_err.max()) if len(lab_err) else 0.0,
        lab_err_frac=float((lab_err > 1e-3).mean()) if len(lab_err) else 0.0,
        over_leash=float((lead > P.leash).mean()), lead_p99=pct(lead, 99),
        tilt0=float(tilt[0]), tilt_max=float(tilt.max()), tipped_any=bool((tilt > P.tip_deg).any()),
        post_term_frac=float(post_term.mean()), post_term_n=int(post_term.sum()), post_term_longest=int(max(runs) if runs else 0),
        tipped_at_t0=bool(post_term[0]),
        can_disp_max=float(disp.max()), can_z0=float(can[0, 2]), can_z_min=float(can[:, 2].min()),
        ic=(round(float(can[0, 0]), 4), round(float(can[0, 1]), 4)), goal=(round(float(s[0, GOAL][0]), 3), round(float(s[0, GOAL][1]), 3)),
        grip_closed=float((grip_cmd < 0.5).mean()), grip_eff_mean=float(s[:, GRIP_EFF].mean()),
        path_len=path, mean_qdot=float(np.abs(np.diff(q, axis=0)).mean()) if n > 1 else 0.0,
        images=t['images'])


def nn_dist(Q, R, scale, chunk=1500):
    Qs = Q / scale; Rs = R / scale; out = np.empty(len(Qs))
    for i in range(0, len(Qs), chunk):
        d = ((Qs[i:i + chunk, None, :] - Rs[None, :, :]) ** 2).sum(-1)
        out[i:i + chunk] = np.sqrt(d.min(1))
    return out


def summarize(name, tapes, rows, P, rng):
    S = [r for r in rows if r['pick'] >= 0]; F = [r for r in rows if r['pick'] < 0]
    tot = sum(r['n'] for r in rows); ftot = sum(r['n'] for r in F)
    cap_len = max(r['n'] for r in rows)
    out = dict(
        name=name, n_tapes=len(rows), n_success=len(S), n_fail=len(F),
        by_stage={k: sum(r['stage'] == k for r in rows) for k in sorted(set(r['stage'] for r in rows))},
        by_label={k: sum(r['label'] == k for r in rows) for k in sorted(set(r['label'] for r in rows))},
        transitions=int(tot), fail_share=ftot / tot,
        len_p50=pct([r['n'] for r in rows], 50), len_mean=float(np.mean([r['n'] for r in rows])), len_max=int(cap_len),
        len_succ_p50=pct([r['n'] for r in S], 50), len_fail_p50=pct([r['n'] for r in F], 50),
        tapes_at_max_len=int(sum(r['n'] == cap_len for r in rows)),
        rewarded=len(S), reward_density=len(S) / tot, expected_rewards_per_128=128 * len(S) / tot,
        first_lift_p50=pct([r['pick'] for r in S], 50), first_lift_mean=float(np.mean([r['pick'] for r in S])) if S else float('nan'),
        frames_after_pick_p50=pct([r['frames_after_pick'] for r in S], 50),
        over_cap=float(np.average([r['over_cap'] for r in rows], weights=[r['n'] for r in rows])),
        over_cap_succ=float(np.average([r['over_cap'] for r in S], weights=[r['n'] for r in S])) if S else float('nan'),
        over_cap_fail=float(np.average([r['over_cap'] for r in F], weights=[r['n'] for r in F])) if F else float('nan'),
        zero_delta=float(np.average([r['zero_delta'] for r in rows], weights=[r['n'] for r in rows])),
        lab_err_frac=float(np.average([r['lab_err_frac'] for r in rows], weights=[r['n'] for r in rows])),
        lab_err_p99=pct([r['lab_err_p99'] for r in rows], 50), lab_err_max=max(r['lab_err_max'] for r in rows),
        dmax_p99=pct([r['dmax_p99'] for r in rows], 50),
        over_leash=float(np.average([r['over_leash'] for r in rows], weights=[r['n'] for r in rows])),
        lead_p99=pct([r['lead_p99'] for r in rows], 50),
        tipped_tapes=int(sum(r['tipped_any'] for r in rows)), tipped_tapes_fail=int(sum(r['tipped_any'] for r in F)),
        post_term_share=sum(r['post_term_n'] for r in rows) / tot,
        post_term_share_fail=(sum(r['post_term_n'] for r in F) / ftot) if ftot else 0.0,
        post_term_longest=int(max(r['post_term_longest'] for r in rows)),
        tipped_at_t0=int(sum(r['tipped_at_t0'] for r in rows)),
        tilt0_p50=pct([r['tilt0'] for r in rows], 50), tilt0_max=max(r['tilt0'] for r in rows),
        can_disp_max_p50_fail=pct([r['can_disp_max'] for r in F], 50), can_disp_max_max=max(r['can_disp_max'] for r in rows),
        can_z0_p50=pct([r['can_z0'] for r in rows], 50), can_z_min=min(r['can_z_min'] for r in rows),
        unique_ics=len(set(r['ic'] for r in rows)), goals=sorted(set(r['goal'] for r in rows)),
        grip_closed=float(np.average([r['grip_closed'] for r in rows], weights=[r['n'] for r in rows])),
        grip_eff_mean=float(np.mean([r['grip_eff_mean'] for r in rows])),
        path_len_succ_p50=pct([r['path_len'] for r in S], 50), mean_qdot=float(np.mean([r['mean_qdot'] for r in rows])),
        images=int(sum(r['images'] for r in rows)),
    )
    # within-set OOD-ness: fail frames vs success frames
    if S and F:
        succ = np.concatenate([t['s'][:, :15] for t, r in zip(tapes, rows) if r['pick'] >= 0])
        fail = np.concatenate([t['s'][:, :15] for t, r in zip(tapes, rows) if r['pick'] < 0])
        ftilt = np.concatenate([tilt_deg(t['s'][:, QUAT]) for t, r in zip(tapes, rows) if r['pick'] < 0])
        scale = succ.std(0) + 1e-6
        ref = succ[rng.choice(len(succ), min(len(succ), P.nn_samples), replace=False)]
        idx = rng.choice(len(fail), min(len(fail), P.nn_samples), replace=False)
        d = nn_dist(fail[idx], ref, scale); ft = ftilt[idx]
        d_self = nn_dist(succ[rng.choice(len(succ), min(len(succ), P.nn_samples // 2), replace=False)], ref, scale)
        thr = pct(d_self, 99)
        out.update(nn_fail_to_succ_p50=pct(d, 50), nn_fail_to_succ_p90=pct(d, 90), nn_fail_to_succ_p99=pct(d, 99),
                   nn_succ_self_p50=pct(d_self, 50), nn_succ_self_p99=thr, frac_fail_beyond_succ_p99=float((d > thr).mean()),
                   nn_fail_tipped_p50=pct(d[ft > P.tip_deg], 50) if (ft > P.tip_deg).any() else None,
                   nn_fail_nottipped_p50=pct(d[ft <= P.tip_deg], 50) if (ft <= P.tip_deg).any() else None)
    return out


def emit_dreamer(L, dsets):
    names = list(dsets); Sm = {n: dsets[n]['summary'] for n in names}
    L.append('\n### Dreamer-format (world-model) sets — composition only (no state in these npz)')
    L.append('| metric | ' + ' | '.join(names) + ' |'); L.append('|---|' + '---|' * len(names))
    for key, label, nd in [('n_episodes', 'episodes', 0), ('n_rewarded', 'rewarded (success) episodes', 0), ('n_zero_reward', 'ZERO-reward (fail) episodes', 0),
                           ('steps', 'decision steps', 0), ('fail_share', 'fail share of steps', 3), ('len_p50', 'episode len p50', 0), ('len_mean', 'episode len mean', 0), ('len_max', 'episode len max', 0),
                           ('len_rewarded_p50', 'rewarded len p50', 0), ('len_fail_p50', 'fail len p50', 0), ('first_reward_idx_p50', 'first-reward idx p50', 0),
                           ('reward_values', 'reward values seen', 0), ('rewards_per_episode_max', 'max rewards per episode', 0), ('terminal_flag_eps', 'episodes with is_terminal', 0),
                           ('image_shape', 'image shape', 0), ('action_dim', 'action dim', 0), ('has_state', 'episodes with state key', 0), ('stamp', 'repeat.json stamp (first keys)', 0)]:
        L.append(f'| {label} | ' + ' | '.join(fmt(Sm[n].get(key), nd) for n in names) + ' |')
    L.append('- A WM set with ZERO-reward episodes carries failure tapes; one with none is success-only. Compare across the dH / dDP / dR2D dirs of the same learner before any human-vs-model WM claim (design audit 2026-08-22, issue 2).')


def fmt(v, nd=3):
    if v is None: return '—'
    if isinstance(v, float):
        if np.isnan(v): return '—'
        return f'{v:.{nd}f}' if abs(v) < 1e4 else f'{v:.3g}'
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sets', nargs='+', help='name=path pairs')
    ap.add_argument('--out', default=None); ap.add_argument('--json', default=None)
    ap.add_argument('--nn-samples', type=int, default=6000); ap.add_argument('--no-cross', action='store_true')
    ap.add_argument('--pick-z', dest='pick_z', type=float, default=0.1505)
    ap.add_argument('--cap', type=float, default=0.025); ap.add_argument('--leash', type=float, default=0.125)
    ap.add_argument('--tip-deg', dest='tip_deg', type=float, default=60.0); ap.add_argument('--grip-open', dest='grip_open', type=float, default=0.3)
    ap.add_argument('--seed', type=int, default=0)
    P = ap.parse_args(); rng = np.random.default_rng(P.seed)

    sets = {}; skipped = {}; dsets = {}
    for spec in P.sets:
        name, path = spec.split('=', 1)
        tapes, err = load_set(path)
        if tapes is None:
            dr = load_dreamer_set(path)
            if dr is not None:
                dsets[name] = dict(path=path, summary=summarize_dreamer(name, *dr))
                print(f'[{name}] dreamer-format, {dr[0].__len__()} episodes from {path}', file=sys.stderr)
                continue
            skipped[name] = f'{path}: {err}'; continue
        rows = [per_tape(t, P) for t in tapes]
        sets[name] = dict(path=path, tapes=tapes, rows=rows, summary=summarize(name, tapes, rows, P, rng))
        print(f'[{name}] {len(tapes)} tapes from {path}', file=sys.stderr)
    if not sets and not dsets: sys.exit('no usable sets')
    if not sets:  # dreamer-only run: skip the state-based report body
        L = [f'# Demo-set census — {time.strftime("%Y-%m-%d %H:%M")}']
        cross = {}; names = []; Sm = {}
        if skipped: L.append('Skipped: ' + '; '.join(f'{k}: {v}' for k, v in skipped.items()))
        emit_dreamer(L, dsets)
        rep = '\n'.join(L)
        if P.out: os.makedirs(os.path.dirname(P.out) or '.', exist_ok=True); open(P.out, 'w').write(rep + '\n'); print(f'wrote {P.out}', file=sys.stderr)
        else: print(rep)
        if P.json: json.dump({n: d['summary'] for n, d in dsets.items()}, open(P.json, 'w'), indent=1, default=str)
        return

    # IC grid over the union of ICs (4x4, like the harvest manifest's ic_coverage)
    allic = np.array([r['ic'] for s in sets.values() for r in s['rows']])
    lo, hi = allic.min(0), allic.max(0) + 1e-6
    for s in sets.values():
        ic = np.array([r['ic'] for r in s['rows']]); b = np.floor((ic - lo) / (hi - lo) * 4).astype(int).clip(0, 3)
        grid = np.zeros((4, 4), int)
        for x, y in b: grid[x, y] += 1
        s['summary']['ic_grid_occupied'] = int((grid > 0).sum()); s['summary']['ic_grid'] = grid.tolist()

    # cross-set NN matrix: frames of A (all / fail / success) -> success frames of B
    cross = {}
    if not P.no_cross and len(sets) > 1:
        succ_frames = {n: np.concatenate([t['s'][:, :15] for t, r in zip(s['tapes'], s['rows']) if r['pick'] >= 0]) for n, s in sets.items()}
        succ_frames = {n: v for n, v in succ_frames.items() if len(v)}
        pooled_scale = np.concatenate(list(succ_frames.values())).std(0) + 1e-6
        refs = {n: v[rng.choice(len(v), min(len(v), P.nn_samples), replace=False)] for n, v in succ_frames.items()}
        for a, sa in sets.items():
            for part in ('all', 'fail', 'success'):
                sel = [t['s'][:, :15] for t, r in zip(sa['tapes'], sa['rows'])
                       if part == 'all' or (part == 'fail') == (r['pick'] < 0)]
                if not sel: continue
                A = np.concatenate(sel); A = A[rng.choice(len(A), min(len(A), P.nn_samples // 2), replace=False)]
                for b, R in refs.items():
                    d = nn_dist(A, R, pooled_scale)
                    cross[(a, part, b)] = dict(p50=pct(d, 50), p90=pct(d, 90))
        print('[cross] done', file=sys.stderr)

    # ---- report ----
    L = []
    L.append(f'# Demo-set census — {time.strftime("%Y-%m-%d %H:%M")}')
    L.append(f'Constants: pick_z {P.pick_z}, delta cap {P.cap} rad/step, leash {P.leash} rad, tip rule tilt>{P.tip_deg} deg & grip cmd<{P.grip_open}. '
             f'NN samples {P.nn_samples}. Sets: ' + ', '.join(f'{n}={s["path"]}' for n, s in sets.items()) + '.')
    if skipped: L.append('Skipped (layout mismatch / unreadable): ' + '; '.join(f'{k}: {v}' for k, v in skipped.items()))
    names = list(sets); Sm = {n: sets[n]['summary'] for n in names}
    def table(title, fields):
        L.append(f'\n### {title}'); L.append('| metric | ' + ' | '.join(names) + ' |'); L.append('|---|' + '---|' * len(names))
        for key, label, nd in fields:
            L.append(f'| {label} | ' + ' | '.join(fmt(Sm[n].get(key), nd) for n in names) + ' |')
    table('Composition', [('n_tapes', 'tapes', 0), ('by_stage', 'by stage', 0), ('by_label', 'by label', 0), ('n_success', 'success (lifted)', 0), ('n_fail', 'fail (never lifted)', 0),
                          ('transitions', 'transitions', 0), ('fail_share', 'fail share of buffer', 3), ('len_p50', 'tape len p50', 0), ('len_mean', 'tape len mean', 0), ('len_max', 'tape len max', 0),
                          ('len_succ_p50', 'success len p50', 0), ('len_fail_p50', 'fail len p50', 0), ('tapes_at_max_len', 'tapes at max len (timed out)', 0),
                          ('unique_ics', 'unique ICs (can xy @t0)', 0), ('ic_grid_occupied', 'IC grid cells occupied /16', 0), ('goals', 'goal xy', 0), ('images', 'tapes with images', 0)])
    table('Reward (pick scope)', [('rewarded', 'rewarded transitions', 0), ('reward_density', 'reward density', 5), ('expected_rewards_per_128', 'expected rewards per 128-demo batch', 3),
                                  ('first_lift_p50', 'first-lift idx p50', 0), ('first_lift_mean', 'first-lift idx mean', 0), ('frames_after_pick_p50', 'frames kept after lift p50', 0)])
    table('Fidelity under delta_joint encoding (cap/leash)', [('over_cap', 'frac frames over cap', 3), ('over_cap_succ', '  successes', 3), ('over_cap_fail', '  fails', 3),
                                                              ('lab_err_frac', 'frac frames one-step label err >1e-3', 3), ('lab_err_p99', 'label err p99 (rad, median tape)', 4), ('lab_err_max', 'label err max (rad)', 3),
                                                              ('dmax_p99', '|delta cmd| p99 (median tape)', 4), ('zero_delta', 'frac zero-delta frames', 3), ('over_leash', 'frac frames |cmd-qmeas|>leash', 3), ('lead_p99', 'lead p99 (median tape)', 3)])
    table('Env-consistency (pick-scope termination) & can state', [('tipped_tapes', 'tapes where can tilts >tip_deg', 0), ('tipped_tapes_fail', '  of which fails', 0),
                                                                   ('post_term_share', 'POST-TERMINATION frames / buffer', 3), ('post_term_share_fail', '  / fail frames', 3), ('post_term_longest', 'longest post-term chain (frames)', 0),
                                                                   ('tipped_at_t0', 'tapes tipped at t0 (lying-can IC)', 0), ('tilt0_p50', 'tilt @t0 p50 (deg)', 1), ('tilt0_max', 'tilt @t0 max', 1),
                                                                   ('can_disp_max_p50_fail', 'can max displacement, fails p50 (m)', 3), ('can_disp_max_max', 'can max displacement, max (m)', 3), ('can_z0_p50', 'can z @t0 p50', 3), ('can_z_min', 'can z min', 3)])
    table('Behaviour', [('grip_closed', 'grip cmd closed (<0.5) frac', 3), ('grip_eff_mean', 'grip effort mean', 3), ('path_len_succ_p50', 'joint path length, successes p50 (rad)', 2), ('mean_qdot', 'mean |dq| per frame (rad)', 5)])
    table('OOD-ness within set (fail frames vs own success frames; std units of the success frames)',
          [('nn_fail_to_succ_p50', 'NN dist fail->succ p50', 2), ('nn_fail_to_succ_p90', '  p90', 2), ('nn_fail_to_succ_p99', '  p99', 2), ('nn_succ_self_p50', 'succ->succ p50', 3), ('nn_succ_self_p99', 'succ->succ p99', 2),
           ('frac_fail_beyond_succ_p99', 'frac fail frames beyond succ p99', 2), ('nn_fail_tipped_p50', 'NN dist, tipped fail frames p50', 2), ('nn_fail_nottipped_p50', 'NN dist, non-tipped fail frames p50', 2)])
    if cross:
        L.append('\n### Cross-set OOD matrix: NN distance (p50 / p90, pooled-success std units) of ROW set frames to COLUMN set SUCCESS frames')
        cols = [n for n in names if any(k[2] == n for k in cross)]
        L.append('| row (set:part) | ' + ' | '.join(cols) + ' |'); L.append('|---|' + '---|' * len(cols))
        for a in names:
            for part in ('all', 'fail', 'success'):
                if (a, part, cols[0]) not in cross: continue
                L.append(f'| {a}:{part} | ' + ' | '.join(f"{cross[(a,part,b)]['p50']:.2f} / {cross[(a,part,b)]['p90']:.2f}" for b in cols) + ' |')
    if dsets: emit_dreamer(L, dsets)
    L.append('\n### How to read')
    L.append('- fail share / post-termination share / longest chain: what the RL critic bootstraps through that the online MDP never produces (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD").')
    L.append('- over cap / label err: frames the delta_joint encoder cannot represent (AUDIT_normalization_2026-08-17 C1); leash: PD lead beyond the env leash.')
    L.append('- NN distances: large = states nothing grounded covers; tipped-can frames land ~40 std away because success frames never vary in can orientation.')
    L.append('- reward density is informative but was shown NOT to move RLPD ignition (hold-reward arm, 25x density, FABLE_HANDOFF §20).')
    rep = '\n'.join(L)
    if P.out:
        os.makedirs(os.path.dirname(P.out) or '.', exist_ok=True); open(P.out, 'w').write(rep + '\n'); print(f'wrote {P.out}', file=sys.stderr)
    else:
        print(rep)
    if P.json:
        J = {n: dict(path=s['path'], summary=s['summary'], per_tape=s['rows']) for n, s in sets.items()}
        J.update({n: dict(path=d['path'], summary=d['summary']) for n, d in dsets.items()})
        J['_cross'] = {f'{a}|{p}|{b}': v for (a, p, b), v in cross.items()}; J['_params'] = vars(P); J['_skipped'] = skipped
        json.dump(J, open(P.json, 'w'), indent=1, default=str); print(f'wrote {P.json}', file=sys.stderr)


if __name__ == '__main__':
    main()
