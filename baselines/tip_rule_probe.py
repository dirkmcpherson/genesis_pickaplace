#!/usr/bin/env python
"""Measure the TIP rule (Lane 6, 2026-09-11). ANALYSIS ONLY -- the env class is never changed.

    # every tape of a set, tip rule OFF, one npz of per-env-frame diagnostics per tape
    python baselines/tip_rule_probe.py run --in <segment-set-dir> --census <census.json> \
        --out-dir <dir> --set-name human [--procs 8] [--tip-deg 1e9]

    # tables 2 and 3 from the npz files two `run` calls wrote
    python baselines/tip_rule_probe.py report --probe-dir <dir> --census-human <j> \
        --census-machine <j> --out <report.json>

WHAT THE RULE IS (baselines/rl/full_env.py ~1260)
-------------------------------------------------
    terminate, info['tipped']=True   iff   a_phys[6] < GRIP_OPEN (0.3)
                                      and  tilt_deg(can quat) > TIP_DEG (60.0)
checked once per ENV FRAME (action_repeat=4 frames per decision), full scope pays
TIP_PENALTY = 0.0, so the rule costs nothing but the rest of the episode.

HOW THIS FILE DISABLES IT
-------------------------
`env.TIP_DEG = <big>` sets an INSTANCE attribute that shadows the class attribute for this
one object. `FullTaskEnv.TIP_DEG` is untouched -- import this module, or run it, and the class
default is still 60.0 (asserted at the end of `build`). The same mechanism re-thresholds it
(`--tip-deg 80`) without editing the class.

`env.terminal_stages = ()` is the second analysis-only override: under ladder='staged' the env
also ends an episode on `slide_success`, and a tape that ends there would stop the action
stream before this file can ask "what happens next". Neither override touches the physics --
termination is a label on a frame, not a force -- so every frame BEFORE the first would-fire
frame is bit-identical to the normal census. That is checked, not assumed: `report` compares
the reconstructed end decision against the census `end_decision` for all 146 tapes.

WHAT IS LOGGED, PER ENV FRAME (not per decision)
-----------------------------------------------
tilt of the can (deg), COMMANDED grip a_phys[6] (0..1), can z, can xy distance to the goal,
the tool-to-can lever, and the tracker's flags (in_hand, at_rest, released, pushed,
contact_push, nested_v2, slide_success) plus the env's own picked / placed_v2 / contact and the
per-frame can<->goal contact boolean. Everything is READ from `env._track` / `info` -- this file
computes no predicate of its own, which is what makes the numbers about the ladder.

At the end of the stream ONE `genv.end_of_episode()` settle runs (100 scene steps) and the
settled tilt is recorded: "where did the can actually finish".
"""
import argparse
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

from relabel_reward import tape_layout, tape_stream, reset_to_tape_ic, set_meta  # noqa: E402

TIP_OFF = 1.0e9            # a tilt no quaternion can reach: the rule can never fire
# the flag columns, in the order they are packed into the uint8 array
FLAGS = ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push', 'nested_v2',
         'slide_success', 'picked', 'placed_v2', 'contact', 'can_goal_contact', 'in_band')


# ----------------------------------------------------------------- env
def build(sim_variant, max_sim_steps, ladder, tip_deg):
    """The annotate_demos env (same asserts, same contract), with the two analysis overrides
    applied to the INSTANCE. Imported rather than re-written so there is one build path."""
    import annotate_demos
    env = annotate_demos.build_env(sim_variant, max_sim_steps, ladder, with_video=False)
    from full_env import FullTaskEnv
    cls_default = float(FullTaskEnv.TIP_DEG)
    env.TIP_DEG = float(tip_deg)
    env.terminal_stages = ()
    assert float(FullTaskEnv.TIP_DEG) == 60.0 == cls_default, (
        'the class default moved: this script must never change it')
    print(f'[tip] instance TIP_DEG {env.TIP_DEG:g} (class default {cls_default:g} untouched), '
          f'terminal_stages {env.terminal_stages}', flush=True)
    return env


# ----------------------------------------------------------------- one tape
def probe_one(env, path, ic_uid=None, set_name='', ic_tol=0.002):
    z = np.load(path, allow_pickle=True)
    layout = tape_layout(z)
    _s0, acts, old_rew, _want = tape_stream(z, layout)
    n = int(acts.shape[0])
    how, d_can, d_goal = reset_to_tape_ic(env, z)
    assert d_can <= ic_tol and d_goal <= ic_tol, (
        f'{os.path.basename(path)}: restored IC differs ({how}): can {d_can * 1000:.1f} mm, '
        f'goal {d_goal * 1000:.1f} mm')

    rec = {k: [] for k in ('dec', 'tilt', 'grip', 'can_z', 'dist_xy', 'lever', 'goal_tilt')}
    flg = {k: [] for k in FLAGS}
    state = dict(dec=0)

    inner = env._step_once

    def wrapped(action):
        out = inner(action)
        info = out[4]
        tr = env._track                       # written by _full_scope_predicates this frame
        bp = np.asarray(env.genv.w['bottle'].get_pos(), dtype=np.float64).reshape(-1)
        rec['dec'].append(state['dec'])
        rec['tilt'].append(float(tr['can_tilt_deg']))
        rec['grip'].append(float(tr['grip_cmd']))
        rec['can_z'].append(float(bp[2]))
        rec['dist_xy'].append(float(tr['dist_xy_m']))
        rec['lever'].append(float(tr['lever_m']))
        rec['goal_tilt'].append(float(tr['goal_tilt_deg']))
        for k in FLAGS:
            v = tr[k] if k in tr else info.get(k)
            flg[k].append(1 if v else 0)
        return out

    env._step_once = wrapped                  # instance attribute; the class is untouched
    t0 = time.time()
    end_reason, end_dec = 'stream_exhausted', n
    try:
        for t in range(n):
            state['dec'] = t
            obs, r, term, trunc, info = env.step(acts[t])
            if term or trunc:
                end_reason = ('tipped' if info.get('tipped') else 'truncated' if trunc
                              else 'terminated')
                end_dec = t + 1
                break
    finally:
        del env._step_once                    # drop the shadow, restore the bound method
    settle = env.genv.end_of_episode()
    final_tilt = _tilt(env.genv.w['bottle'].get_quat())
    row = dict(file=os.path.basename(path), set=set_name, ic_uid=ic_uid, decisions=n,
               layout=layout, ic=how, ic_can_mm=round(d_can * 1000, 2),
               ic_goal_mm=round(d_goal * 1000, 2), tip_deg=float(env.TIP_DEG),
               probe_end_reason=end_reason, probe_end_decision=int(end_dec),
               frames=len(rec['dec']), settle_nested=bool(settle['nested']),
               settle_slide=bool(settle['slide_success']),
               settle_slide_route=str(settle.get('slide_route')),
               final_tilt_deg=round(final_tilt, 3),
               seconds=round(time.time() - t0, 1))
    arrs = {k: np.asarray(v, np.float32) for k, v in rec.items() if k != 'dec'}
    arrs['dec'] = np.asarray(rec['dec'], np.int32)
    for k in FLAGS:
        arrs['f_' + k] = np.asarray(flg[k], np.uint8)
    return row, arrs


def _tilt(quat):
    from stage_predicates import tilt_deg
    return tilt_deg(np.asarray(quat).reshape(-1)[:4])


# ----------------------------------------------------------------- driver
def files_of(set_dir, names=None):
    fs = sorted(pl.Path(set_dir).glob('genesis-*.npz'))
    if names:
        want = {os.path.basename(x) for x in names}
        fs = [f for f in fs if f.name in want]
        missing = want - {f.name for f in fs}
        assert not missing, f'not in {set_dir}: {sorted(missing)}'
    assert fs, f'no tapes in {set_dir}'
    return fs


def ic_map(census_json):
    """{segment basename: ic_uid} from the census of record -- the census resolved them with
    --src-dir, so this file inherits that resolution rather than repeating it."""
    d = json.load(open(census_json))
    return {r['file']: r['ic_uid'] for r in d['rows']}


def run_shard(args, files, meta, icm):
    env = build(args.sim_variant or meta['sim_variant'], meta['max_sim_steps'],
                args.ladder, args.tip_deg)
    out = pl.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for i, f in enumerate(files):
        row, arrs = probe_one(env, str(f), ic_uid=icm.get(f.name), set_name=args.set_name)
        np.savez_compressed(out / f'{args.set_name}_{f.stem}.npz', **arrs)
        rows.append(row)
        print(f'[{i + 1}/{len(files)}] {f.name} ic={row["ic_uid"]} {row["frames"]} frames, '
              f'end {row["probe_end_reason"]}@{row["probe_end_decision"]}/{row["decisions"]}, '
              f'final tilt {row["final_tilt_deg"]:.1f} deg [{row["seconds"]:.0f}s]', flush=True)
    return rows


def cmd_run(args):
    files = files_of(args.inp, args.tapes)
    if args.limit:
        files = files[:int(args.limit)]
    meta = set_meta(args.inp, [str(f) for f in files], args)
    icm = ic_map(args.census) if args.census else {}
    if args.shard is not None:
        mine = [f for i, f in enumerate(files) if i % args.nshards == args.shard]
        rows = run_shard(args, mine, meta, icm)
        json.dump(rows, open(f'{args.out_dir}/_shard{args.shard}.json', 'w'), indent=1)
        return
    n_proc = max(1, min(int(args.procs), 8, len(files)))
    if n_proc == 1:
        rows = run_shard(args, files, meta, icm)
    else:
        base = [sys.executable, os.path.abspath(__file__), 'run', '--in', args.inp,
                '--out-dir', args.out_dir, '--set-name', args.set_name,
                '--ladder', args.ladder, '--tip-deg', repr(args.tip_deg),
                '--max-sim-steps', str(meta['max_sim_steps'])]
        if args.census:
            base += ['--census', args.census]
        if args.limit:
            base += ['--limit', str(args.limit)]
        if args.sim_variant:
            base += ['--sim-variant', args.sim_variant]
        if args.tapes:
            base += ['--tapes'] + list(args.tapes)
        pl.Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        procs = [subprocess.Popen(base + ['--shard', str(i), '--nshards', str(n_proc)])
                 for i in range(n_proc)]
        rcs = [p.wait() for p in procs]
        assert all(rc == 0 for rc in rcs), f'shard failures: {rcs}'
        rows = []
        for i in range(n_proc):
            p = f'{args.out_dir}/_shard{i}.json'
            rows += json.load(open(p))
            os.remove(p)
    rows.sort(key=lambda r: r['file'])
    json.dump(dict(set=args.set_name, source=os.path.abspath(args.inp), ladder=args.ladder,
                   tip_deg=args.tip_deg, sim_variant=args.sim_variant or meta['sim_variant'],
                   n_tapes=len(rows), rows=rows),
              open(f'{args.out_dir}/_probe_{args.set_name}.json', 'w'), indent=1)
    print(f'wrote {args.out_dir}/_probe_{args.set_name}.json  ({len(rows)} tapes)')


# ----------------------------------------------------------------- analysis
def load_tape(probe_dir, set_name, stem):
    return np.load(pl.Path(probe_dir) / f'{set_name}_{stem}.npz')


def first_true(mask):
    idx = np.flatnonzero(mask)
    return int(idx[0]) if idx.size else None


def fire_frame(z, tip_deg, guard):
    """The first ENV FRAME at which the rule with this threshold and guard would fire.
    guard: 'grip' = the rule of record (a_phys[6] < 0.3); 'hand' = tracker `not in_hand`;
    'none' = bare tilt."""
    tilt = z['tilt']
    if guard == 'grip':
        g = z['grip'] < 0.3
    elif guard == 'hand':
        g = z['f_in_hand'] == 0
    elif guard == 'none':
        g = np.ones(tilt.shape, bool)
    else:
        raise ValueError(guard)
    return first_true((tilt > tip_deg) & g)


def analyse(z, row, census_row, tip_deg=60.0):
    """Everything task 2 asks, for one tape."""
    tilt, grip, dec = z['tilt'], z['grip'], z['dec']
    free = (grip < 0.3) | (z['f_in_hand'] == 0)          # the task's `released` for table 3
    f0 = fire_frame(z, tip_deg, 'grip')
    out = dict(file=row['file'], ic_uid=row['ic_uid'], set=row['set'],
               decisions=int(row['decisions']), frames=int(row['frames']),
               census_end_reason=(census_row or {}).get('end_reason'),
               census_end_decision=(census_row or {}).get('end_decision'),
               fire_frame=f0,
               fire_decision=(None if f0 is None else int(dec[f0]) + 1),
               tilt_at_fire=(None if f0 is None else round(float(tilt[f0]), 2)),
               grip_at_fire=(None if f0 is None else round(float(grip[f0]), 3)),
               in_hand_at_fire=(None if f0 is None else bool(z['f_in_hand'][f0])),
               max_tilt_all=round(float(tilt.max()), 2),
               max_tilt_free=(round(float(tilt[free].max()), 2) if free.any() else None),
               final_tilt_deg=row['final_tilt_deg'],
               settle_nested=row['settle_nested'], settle_slide=row['settle_slide'])
    if f0 is not None:
        post = slice(f0 + 1, None)
        tpost = tilt[post]
        out.update(
            max_tilt_after=(round(float(tpost.max()), 2) if tpost.size else None),
            horizontal_after=(bool((tpost >= 80).any()) if tpost.size else False),
            frames_after=int(tilt.size - f0 - 1),
            decisions_after=int(dec[-1] - dec[f0]) if tilt.size else 0,
            # recovery, two readings. FREE = the can stands back up while NOT in the hand
            # (the July claim's sense: an unheld can past 60 deg is a dead end).
            recovered_free=bool(((tilt[post] < 20) & (z['f_in_hand'][post] == 0)).any()),
            recovered_any=bool((tilt[post] < 20).any()),
            stage_after={k: bool((z['f_' + k][post] == 1).any())
                         for k in ('placed_v2', 'nested_v2', 'slide_success', 'contact_push')},
            stage_at_fire={k: bool(z['f_' + k][f0]) for k in
                           ('placed_v2', 'nested_v2', 'slide_success', 'contact_push',
                            'picked', 'in_hand')},
        )
    out['horizontal_ever'] = bool((tilt >= 80).any())
    return out


def cmd_report(args):
    probe = {}
    for s in ('human', 'machine'):
        p = pl.Path(args.probe_dir) / f'_probe_{s}.json'
        probe[s] = json.load(open(p))
    cens = {'human': json.load(open(args.census_human)),
            'machine': json.load(open(args.census_machine))}
    cmap = {s: {r['file']: r for r in cens[s]['rows']} for s in cens}

    per_tape, checks = [], dict(end_match=0, end_mismatch=[], n=0)
    for s in ('human', 'machine'):
        for row in probe[s]['rows']:
            stem = pl.Path(row['file']).stem
            z = load_tape(args.probe_dir, s, stem)
            cr = cmap[s].get(row['file'])
            a = analyse(z, row, cr, tip_deg=args.tip_deg)
            # ---- equivalence check: reconstruct what the NORMAL census would have done.
            # The census ends at the FIRST of (tip fires, a terminal stage fires, truncation,
            # stream exhausted); before that frame this run is bit-identical.
            f_tip = a['fire_frame']
            f_sl = first_true(z['f_slide_success'] == 1)
            cand = [(f, k) for f, k in ((f_tip, 'tipped'), (f_sl, 'slide_success'))
                    if f is not None]
            if cand:
                f, k = min(cand)
                pred_reason, pred_dec = k, int(z['dec'][f]) + 1
            else:
                nd = int(row['decisions'])
                # max_sim_steps truncation: 2400 env frames = 600 decisions
                pred_reason = 'truncated' if z['dec'].size >= args.max_frames else 'stream_exhausted'
                pred_dec = min(nd, int(z['dec'][-1]) + 1)
            a['predicted_census_end'] = [pred_reason, pred_dec]
            ok = (cr is not None and cr['end_reason'] == pred_reason
                  and cr['end_decision'] == pred_dec)
            checks['n'] += 1
            if ok:
                checks['end_match'] += 1
            else:
                checks['end_mismatch'].append(
                    dict(set=s, file=row['file'], ic=row['ic_uid'],
                         census=[cr['end_reason'], cr['end_decision']] if cr else None,
                         predicted=[pred_reason, pred_dec]))
            per_tape.append(a)

    # ---- task 2: the tipped tapes
    tipped = [a for a in per_tape if a['census_end_reason'] == 'tipped']
    # ---- task 2b: the threshold sweep, over the 25 tipped tapes
    sweep = []
    for T in args.sweep:
        fires, recov, rec_any = [], 0, 0
        for a in tipped:
            z = load_tape(args.probe_dir, a['set'], pl.Path(a['file']).stem)
            f = fire_frame(z, T, 'grip')
            if f is None:
                continue
            fires.append(a['ic_uid'])
            post = slice(f + 1, None)
            if ((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any():
                recov += 1
            if (z['tilt'][post] < 20).any():
                rec_any += 1
        sweep.append(dict(tip_deg=T, n_terminate=len(fires), recovered_free=recov,
                          recovered_any=rec_any, ics=sorted(x for x in fires if x is not None)))
    # the same sweep over ALL tapes (a threshold change touches every episode, not only
    # the 25 the current rule caught)
    sweep_all = []
    for T in args.sweep:
        n_f, n_rec, n_held = 0, 0, 0
        for a in per_tape:
            z = load_tape(args.probe_dir, a['set'], pl.Path(a['file']).stem)
            f = fire_frame(z, T, 'grip')
            if f is None:
                continue
            n_f += 1
            post = slice(f + 1, None)
            if ((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any():
                n_rec += 1
            if z['f_in_hand'][f]:
                n_held += 1
        sweep_all.append(dict(tip_deg=T, n_terminate=n_f, recovered_free=n_rec,
                              fired_while_in_hand=n_held))
    # ---- guard variants at the threshold of record
    guards = []
    for guard in ('grip', 'hand', 'none'):
        n_f, n_held, n_rec, ics = 0, 0, 0, []
        for a in per_tape:
            z = load_tape(args.probe_dir, a['set'], pl.Path(a['file']).stem)
            f = fire_frame(z, args.tip_deg, guard)
            if f is None:
                continue
            n_f += 1
            ics.append((a['set'], a['ic_uid']))
            # a "false termination of a held can": the can is IN HAND at the firing frame
            if z['f_in_hand'][f]:
                n_held += 1
            post = slice(f + 1, None)
            if ((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any():
                n_rec += 1
        guards.append(dict(guard=guard, tip_deg=args.tip_deg, n_terminate=n_f,
                           fired_while_in_hand=n_held, recovered_free=n_rec))

    # ---- task 3: max tilt while released, over ALL tapes
    dist = []
    for T in (30, 45, 60, 70, 80):
        rows = []
        for s in ('human', 'machine'):
            n_reach, n_rec, n_rec_any = 0, 0, 0
            for a in [x for x in per_tape if x['set'] == s]:
                z = load_tape(args.probe_dir, s, pl.Path(a['file']).stem)
                free = (z['grip'] < 0.3) | (z['f_in_hand'] == 0)
                f = first_true((z['tilt'] > T) & free)
                if f is None:
                    continue
                n_reach += 1
                post = slice(f + 1, None)
                if ((z['tilt'][post] < 20) & free[post]).any():
                    n_rec += 1
                if (z['tilt'][post] < 20).any():
                    n_rec_any += 1
            rows.append(dict(set=s, n_reach=n_reach, recovered_free=n_rec,
                             recovered_any=n_rec_any))
        dist.append(dict(deg=T, by_set=rows))

    out = dict(tip_deg_of_record=args.tip_deg, checks=checks, sweep_tipped=sweep,
               sweep_all=sweep_all, guards=guards, tilt_distribution=dist,
               tipped_tapes=tipped, per_tape=per_tape)
    json.dump(out, open(args.out, 'w'), indent=1)
    print(json.dumps(dict(checks={k: v if k != 'end_mismatch' else len(v)
                                  for k, v in checks.items()},
                          sweep_tipped=sweep, guards=guards), indent=1))
    print(f'wrote {args.out}')


# ----------------------------------------------------------------- markdown tables
PAY = dict(picked=1.0, placed_v2=1.0, contact_push=2.0, slide_success=4.0)


def _first_sustained(m, k):
    """First index at which `m` has been true for k consecutive frames."""
    if k <= 1:
        return first_true(m)
    c = 0
    for i, v in enumerate(m):
        c = c + 1 if v else 0
        if c >= k:
            return i
    return None


def _guard_mask(z, guard):
    return {'grip': z['grip'] < 0.3,
            'hand': z['f_in_hand'] == 0,
            'union': (z['grip'] < 0.3) | (z['f_in_hand'] == 0),
            'none': np.ones(z['tilt'].shape, bool)}[guard]


def cmd_tables(args):
    """Every table in paper/TIP_RULE_2026-09-11.md, from the report + the per-frame npz."""
    rep = json.load(open(args.report))
    per = rep['per_tape']
    Z = {a['set'] + a['file']: load_tape(args.probe_dir, a['set'], pl.Path(a['file']).stem)
         for a in per}

    print('## check: the rule-off re-execution reproduces the census\n')
    c = rep['checks']
    print(f"{c['end_match']} / {c['n']} tapes end on the same decision with the same reason; "
          f"{len(c['end_mismatch'])} mismatches.\n")

    print('## table 2 -- the 25 tapes whose census `end_reason` is `tipped`\n')
    print('| set | ic | fires @dec / decisions | tilt at fire | grip | in_hand | max tilt after '
          '| final tilt (settle) | horizontal (>=80) | recovers | new stages after |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for a in sorted([x for x in per if x['census_end_reason'] == 'tipped'],
                    key=lambda x: (x['set'], x['ic_uid'])):
        z = Z[a['set'] + a['file']]
        f = a['fire_frame']
        new = [k for k in PAY if (z['f_' + k][f + 1:] == 1).any()
               and not (z['f_' + k][:f + 1] == 1).any()]
        print('| %s | %s | %d / %d | %.1f | %.2f | %d | %s | %.1f | %s | %s | %s |'
              % (a['set'], a['ic_uid'], a['fire_decision'], a['decisions'], a['tilt_at_fire'],
                 a['grip_at_fire'], int(a['in_hand_at_fire']),
                 '%.1f' % a['max_tilt_after'] if a['max_tilt_after'] is not None else '--',
                 a['final_tilt_deg'], 'yes' if a['horizontal_ever'] else 'no',
                 'yes' if a['recovered_free'] else 'no', ', '.join(new) or 'none'))

    print('\n## threshold sweep, grip guard unchanged (the 25 tipped tapes)\n')
    print('| TIP_DEG | tapes that still terminate | of those, recover later |')
    print('|---|---|---|')
    for r in rep['sweep_tipped']:
        print('| %g | %d | %d |' % (r['tip_deg'], r['n_terminate'], r['recovered_free']))
    print('\n## the same sweep over all %d tapes\n' % len(per))
    print('| TIP_DEG | terminate | recover later | fired while the can is in hand |')
    print('|---|---|---|---|')
    for r in rep['sweep_all']:
        print('| %g | %d | %d | %d |' % (r['tip_deg'], r['n_terminate'], r['recovered_free'],
                                         r['fired_while_in_hand']))

    print('\n## guard variants x threshold x sustain, all %d tapes\n' % len(per))
    print('| guard | TIP_DEG | sustain (env frames) | terminates | fires while in hand | '
          'recovers later | free horizontal cans MISSED | median decisions left at the fire |')
    print('|---|---|---|---|---|---|---|---|')
    for guard in ('grip', 'hand', 'union', 'none'):
        for T in args.sweep2:
            for k in args.sustain:
                n = nh = nr = miss = 0
                left = []
                for a in per:
                    z = Z[a['set'] + a['file']]
                    f = _first_sustained((z['tilt'] > T) & _guard_mask(z, guard), k)
                    horiz = bool(((z['tilt'] >= 80) & (z['f_in_hand'] == 0)).any())
                    if f is None:
                        miss += int(horiz)
                        continue
                    n += 1
                    nh += int(bool(z['f_in_hand'][f]))
                    post = slice(f + 1, None)
                    nr += int(bool(((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any()))
                    left.append(int(z['dec'][-1]) - int(z['dec'][f]))
                print('| %s | %g | %d | %d | %d | %d | %d | %d |'
                      % (guard, T, k, n, nh, nr, miss,
                         int(np.median(left)) if left else -1))

    print('\n## reward a guard change would forfeit (stages first granted after the fire)\n')
    print('| guard | sustain | tapes losing a rung | reward lost | which |')
    print('|---|---|---|---|---|')
    for guard, k in (('grip', 1), ('hand', 1), ('hand', 4)):
        tot, det = 0.0, []
        for a in per:
            z = Z[a['set'] + a['file']]
            f = _first_sustained((z['tilt'] > 60) & _guard_mask(z, guard), k)
            if f is None:
                continue
            lost = [x for x in PAY if (z['f_' + x][f + 1:] == 1).any()
                    and not (z['f_' + x][:f + 1] == 1).any()]
            if lost:
                tot += sum(PAY[x] for x in lost)
                det.append('%s %s: %s' % (a['set'], a['ic_uid'], '+'.join(lost)))
        print('| %s | %d | %d | %.0f | %s |' % (guard, k, len(det), tot, '; '.join(det) or '--'))

    print('\n## table 3 -- max can tilt while released, all %d tapes\n' % len(per))
    print('| tilt reached while released | {human tapes} reach | recover | '
          '{machine tapes} reach | recover |')
    print('|---|---|---|---|---|')
    for r in rep['tilt_distribution']:
        h, m = r['by_set']
        print('| >= %d deg | %d / 74 | %d | %d / 72 | %d |'
              % (r['deg'], h['n_reach'], h['recovered_free'], m['n_reach'], m['recovered_free']))

    print('\n## free horizontal cans the rule never labels (>=80 deg, not in hand, rule silent)\n')
    print('| set | ic | horizontal at dec / decisions | commanded grip there | census end |')
    print('|---|---|---|---|---|')
    for a in sorted(per, key=lambda x: (x['set'], x['ic_uid'] or -1)):
        z = Z[a['set'] + a['file']]
        h = first_true((z['tilt'] >= 80) & (z['f_in_hand'] == 0))
        if h is None or fire_frame(z, 60.0, 'grip') is not None:
            continue
        print('| %s | %s | %d / %d | %.2f | %s |'
              % (a['set'], a['ic_uid'], int(z['dec'][h]) + 1, a['decisions'], z['grip'][h],
                 a['census_end_reason']))


def cmd_rollout(args):
    """The same questions asked of POLICY rollouts produced by baselines/eval_e2e_tipoff.py
    (tip rule OFF). Its npz schema is this file's schema, so one analysis serves both."""
    d = pl.Path(args.probe_dir)
    runs = sorted(d.glob('_rollout_*.json'))
    assert runs, f'no _rollout_*.json in {d}'
    print('## {RLPD} e2e rollouts with the tip rule OFF -- where the rule would have fired\n')
    print('| run | eps | rule fires | before pick | picked, in hand | picked, free | '
          'after release | median decision of the fire | median decisions it would cut |')
    print('|---|---|---|---|---|---|---|---|---|')
    allrows = []
    for rj in runs:
        meta = json.load(open(rj))
        n = fires = a = b = c = e = 0
        at, cut = [], []
        for r in meta['rows']:
            z = np.load(d / f"{meta['tag']}_ep{r['ep']:03d}.npz")
            n += 1
            f = fire_frame(z, args.tip_deg, 'grip')
            r['_z'] = str(d / f"{meta['tag']}_ep{r['ep']:03d}.npz")
            r['_fire'] = f
            allrows.append((meta['tag'], r, z))
            if f is None:
                continue
            fires += 1
            at.append(int(z['dec'][f]) + 1)
            cut.append(int(z['dec'][-1]) - int(z['dec'][f]))
            if not z['f_picked'][f]:
                a += 1
            elif z['f_released'][f]:
                e += 1
            elif z['f_in_hand'][f]:
                b += 1
            else:
                c += 1
        print('| %s | %d | %d | %d | %d | %d | %d | %d | %d |'
              % (meta['tag'], n, fires, a, b, c, e,
                 int(np.median(at)) if at else -1, int(np.median(cut)) if cut else -1))

    print('\n### the counterfactual: what the can does after the frame the rule would fire\n')
    print('| run | fires | goes horizontal (>=80) after | RECOVERS (<20 free) after | '
          'flat at the settle | any rung granted after |')
    print('|---|---|---|---|---|---|')
    for rj in runs:
        meta = json.load(open(rj))
        fires = hz = rec = flat = rung = 0
        for r in meta['rows']:
            z = np.load(d / f"{meta['tag']}_ep{r['ep']:03d}.npz")
            f = fire_frame(z, args.tip_deg, 'grip')
            if f is None:
                continue
            fires += 1
            post = slice(f + 1, None)
            hz += int(bool((z['tilt'][post] >= 80).any()))
            rec += int(bool(((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any()))
            flat += int(r['final_tilt_deg'] >= 80)
            rung += int(any((z['f_' + k][post] == 1).any() and not (z['f_' + k][:f + 1] == 1).any()
                            for k in PAY))
        print('| %s | %d | %d | %d | %d | %d |' % (meta['tag'], fires, hz, rec, flat, rung))

    print('\n### guard variants on the same rollouts (what each would terminate)\n')
    print('| run | guard | sustain | terminates of %d | fires while in hand | recovers after | '
          'free-flat episodes MISSED | median decisions cut |' % len(allrows))
    print('|---|---|---|---|---|---|---|---|')
    for rj in runs:
        meta = json.load(open(rj))
        for guard, k in (('grip', 1), ('hand', 1), ('hand', 4), ('hand', 12)):
            nn = nh = nr = miss = 0
            cut = []
            for r in meta['rows']:
                z = np.load(d / f"{meta['tag']}_ep{r['ep']:03d}.npz")
                f = _first_sustained((z['tilt'] > args.tip_deg) & _guard_mask(z, guard), k)
                horiz = bool(((z['tilt'] >= 80) & (z['f_in_hand'] == 0)).any())
                if f is None:
                    miss += int(horiz)
                    continue
                nn += 1
                nh += int(bool(z['f_in_hand'][f]))
                post = slice(f + 1, None)
                nr += int(bool(((z['tilt'][post] < 20) & (z['f_in_hand'][post] == 0)).any()))
                cut.append(int(z['dec'][-1]) - int(z['dec'][f]))
            print('| %s | %s | %d | %d | %d | %d | %d | %d |'
                  % (meta['tag'], guard, k, nn, nh, nr, miss,
                     int(np.median(cut)) if cut else -1))

    print('\n### horizon accounting (tip rule OFF, so every episode ran the full horizon)\n')
    print('| run | episodes | decisions run | decisions the rule of record would have cut | '
          'the `hand` guard (sustain 4) would cut |')
    print('|---|---|---|---|---|')
    for rj in runs:
        meta = json.load(open(rj))
        tot = c1 = c2 = 0
        for r in meta['rows']:
            z = np.load(d / f"{meta['tag']}_ep{r['ep']:03d}.npz")
            tot += int(z['dec'][-1]) + 1
            for guard, k, acc in (('grip', 1, 'c1'), ('hand', 4, 'c2')):
                f = _first_sustained((z['tilt'] > args.tip_deg) & _guard_mask(z, guard), k)
                if f is not None:
                    if acc == 'c1':
                        c1 += int(z['dec'][-1]) - int(z['dec'][f])
                    else:
                        c2 += int(z['dec'][-1]) - int(z['dec'][f])
        print('| %s | %d | %d | %d (%.0f %%) | %d (%.0f %%) |'
              % (meta['tag'], len(meta['rows']), tot, c1, 100 * c1 / max(tot, 1),
                 c2, 100 * c2 / max(tot, 1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('run', 'report', 'tables', 'rollout'))
    ap.add_argument('--in', dest='inp', default=None)
    ap.add_argument('--census', default=None, help='run: census json, for the ic_uid map')
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--set-name', default='set')
    ap.add_argument('--tapes', nargs='*', default=None)
    ap.add_argument('--ladder', choices=('staged', 'sparse'), default='staged')
    ap.add_argument('--sim-variant', default=None)
    ap.add_argument('--max-sim-steps', type=int, default=None)
    ap.add_argument('--tip-deg', type=float, default=TIP_OFF,
                    help='INSTANCE override. Default 1e9 = the rule can never fire.')
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--shard', type=int, default=None)
    ap.add_argument('--nshards', type=int, default=None)
    # report
    ap.add_argument('--probe-dir', default=None)
    ap.add_argument('--census-human', default=None)
    ap.add_argument('--census-machine', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--sweep', type=float, nargs='*', default=[45, 60, 70, 80, 85, 89])
    ap.add_argument('--max-frames', type=int, default=2400)
    # tables
    ap.add_argument('--report', default=None, help='tables: the report.json to read')
    ap.add_argument('--sweep2', type=float, nargs='*', default=[60, 80],
                    help='tables: thresholds for the guard x threshold x sustain grid')
    ap.add_argument('--sustain', type=int, nargs='*', default=[1, 4, 12],
                    help='tables: consecutive ENV FRAMES the condition must hold (4 = 1 decision)')
    args = ap.parse_args()
    if args.cmd == 'run':
        assert args.inp and args.out_dir, 'run needs --in and --out-dir'
        cmd_run(args)
    elif args.cmd == 'report':
        assert args.probe_dir and args.out, 'report needs --probe-dir and --out'
        args.tip_deg = 60.0 if args.tip_deg == TIP_OFF else args.tip_deg
        cmd_report(args)
    elif args.cmd == 'tables':
        assert args.probe_dir and args.report, 'tables needs --probe-dir and --report'
        cmd_tables(args)
    else:
        assert args.probe_dir, 'rollout needs --probe-dir'
        args.tip_deg = 60.0 if args.tip_deg == TIP_OFF else args.tip_deg
        cmd_rollout(args)


if __name__ == '__main__':
    main()
