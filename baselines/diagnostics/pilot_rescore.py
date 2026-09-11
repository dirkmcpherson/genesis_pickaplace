#!/usr/bin/env python3
"""Score POLICY stage records offline under every candidate ladder (Lane 11, 2026-09-11).

    python baselines/diagnostics/pilot_rescore.py --records <root> [--out <json>]

`<root>` holds one directory per evaluation cell (`<ckpt>_<icset>_s<seed>/`), each written by

    baselines/eval_e2e.py --records-out <root>/<cell> ...

i.e. one `ep<N>_uid<U>.npz` per episode in the format `baselines/rl/relabel_reward.py
--records-out` writes for demonstration tapes, plus a `manifest.json`. Every episode is replayed
through `relabel_reward.offline_episode` -- the SAME `StageTracker` + `LadderAccountant` the env
runs -- under `staged`, `sparse`, `nested_sparse` and `nested_ramp`.

WHAT IS AND IS NOT MEASURABLE FROM THESE RECORDS
------------------------------------------------
The rollout ran under ONE ladder (`record_ladder`, normally `staged`) and TERMINATED where that
ladder terminates. Termination was not suppressed, because a policy is closed-loop: extending a
recorded trajectory past its terminal is not possible offline, and re-rolling with termination
suppressed produces a DIFFERENT trajectory, not a longer version of this one.

Consequences, both reported rather than assumed:
  * the predicate view over the WHOLE record is the `staged` replay's grants, which must equal
    the live run's `live_stages` (that is the verification below, and it is a check on the
    recorder, not an assumption about it);
  * a ladder whose terminal comes LATER than the record's is RIGHT-CENSORED on the episodes
    that terminated. Under `staged` the terminal is `slide_success`; `sparse`'s (`nested_v2`)
    can never come later, but the nested ladders' (`home`) can. The episodes at risk are
    exactly those with `end_reason == <the record ladder's terminal>` and no `home` grant, and
    they are counted in the `censored` column. An absent `home` on a censored episode is
    UNKNOWN, not a zero.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.environ.get('GENESIS_PICKAPLACE_ROOT',
                      os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, 'baselines'))
sys.path.insert(0, os.path.join(REPO, 'baselines', 'rl'))

LADDERS = ('staged', 'sparse', 'nested_sparse', 'nested_ramp')
# every column asked for, in the order the doc prints them
COLS = ('picked', 'placed_v2', 'contact_push', 'pushed', 'farside', 'slide_event', 'home',
        'nested_v2', 'nested_honest', 'tipped', 'slide_success')


def scalar(z, k, default=None):
    """Read one stamp out of a record. Accepts an NpzFile or the materialised dict."""
    keys = z.files if hasattr(z, 'files') else z
    if k not in keys:
        return default
    a = np.asarray(z[k])
    return a.item() if a.shape == () else a


def score_episode(path):
    """One record -> its four ladder scorings plus the diagnostics the confusion analysis needs."""
    from relabel_reward import offline_episode
    # MATERIALISE the record before replaying it. `np.load` on a COMPRESSED npz returns a lazy
    # NpzFile, and `offline_episode` indexes `rec['can_pos'][i]` once per frame -- every one of
    # those re-decompresses the whole array. Measured on this batch: 3.35 s per replay lazily
    # against 0.13 s materialised, a 25x difference and the reason a first run of this script
    # was projected at 134 minutes. `offline_episode` only ever does `rec[key]`, so a plain dict
    # is a drop-in and nothing about the scoring changes.
    z = {k: np.asarray(v) for k, v in np.load(path, allow_pickle=True).items()}
    rec_ladder = str(scalar(z, 'record_ladder', 'staged'))
    live = json.loads(str(scalar(z, 'live_stages', '{}')))
    out = dict(file=os.path.basename(path), ep=int(scalar(z, 'ep', -1)),
               uid=int(scalar(z, 'uid', -1)), n_decisions=int(scalar(z, 'n_decisions', 0)),
               frames=int(np.asarray(z['dec']).shape[0]),
               record_ladder=rec_ladder,
               live_end_reason=str(scalar(z, 'end_reason', '')),
               live_reward=float(scalar(z, 'reward_recorded_total', float('nan'))),
               nested_honest=bool(scalar(z, 'nested_honest', False)),
               slide_success_settle=bool(scalar(z, 'slide_success_settle', False)),
               live_stages=live, ladders={})
    for lad in LADDERS:
        r = offline_episode(z, lad, far_release=False)
        ep = r['episode']
        out['ladders'][lad] = dict(
            reward=float(np.asarray(r['rewards'], np.float64).sum()),
            ramp_paid=float(r['ramp_paid']), end_reason=r['end_reason'],
            end_decision=int(r['end_decision']), grants={k: int(v) for k, v in r['grants'].items()},
            slide_gain_m=float(ep['slide_gain_m']),
            gain_during_release_m=float(ep['gain_during_release_m']),
            release_dist_m=float(ep['release_dist_m']),
            settled_after_release=bool(ep['settled_after_release']),
            farside=bool(ep['farside']), slide_event=bool(ep['slide_event']), home=bool(ep['home']),
            pushed=bool(ep['pushed']), nested_v2=bool(ep['nested_v2']),
            released=bool(ep['released']))
    # The predicate view over the WHOLE record is the replay under the ladder that RAN: its
    # terminal is where the record ends, so its grants span every frame that exists.
    base = out['ladders'][rec_ladder]
    out['flags'] = {
        'picked': 'picked' in base['grants'], 'placed_v2': 'placed_v2' in base['grants'],
        'contact_push': 'contact_push' in base['grants'], 'pushed': base['pushed'],
        'farside': base['farside'], 'slide_event': base['slide_event'], 'home': base['home'],
        'nested_v2': base['nested_v2'], 'nested_honest': out['nested_honest'],
        'tipped': base['end_reason'] == 'tipped',
        'slide_success': 'slide_success' in base['grants'],
    }
    out['diag'] = dict(slide_gain_m=base['slide_gain_m'],
                       gain_during_release_m=base['gain_during_release_m'],
                       release_dist_m=base['release_dist_m'],
                       settled_after_release=base['settled_after_release'])
    # A ladder terminal LATER than the record's cannot be observed on an episode that stopped at
    # the record ladder's PAID terminal. `truncated` (the horizon) and `tipped` (the universal
    # tip rule) end every ladder alike, and `stream_exhausted` means no terminal was reached at
    # all -- none of the three censors anything.
    out['censored_for_home'] = bool(
        base['end_reason'] not in ('truncated', 'tipped', 'stream_exhausted') and not base['home'])
    return out


def verify(eps):
    """The offline replay under the recorded ladder must reproduce the LIVE run, flag for flag
    and reward for reward. Reported as a count of disagreements, never assumed."""
    keys = ('picked', 'placed_v2', 'contact_push', 'slide_success', 'nested_v2',
            'farside', 'slide_event', 'home')
    bad_flags, bad_reward, checked = [], [], 0
    for e in eps:
        live = e['live_stages']
        if not live:
            continue
        checked += 1
        for k in keys:
            if k in live and bool(live[k]) != bool(e['flags'][k]):
                bad_flags.append((e['file'], k, bool(live[k]), bool(e['flags'][k])))
        got = e['ladders'][e['record_ladder']]['reward']
        if not np.isnan(e['live_reward']) and abs(got - e['live_reward']) > 1e-4:
            bad_reward.append((e['file'], e['live_reward'], got))
    return dict(checked=checked, flag_disagreements=bad_flags, reward_disagreements=bad_reward)


def cell_rows(root):
    cells = []
    for d in sorted(glob.glob(os.path.join(root, '*'))):
        if not os.path.isdir(d):
            continue
        files = sorted(glob.glob(os.path.join(d, 'ep*.npz')))
        if not files:
            continue
        man = {}
        mp = os.path.join(d, 'manifest.json')
        if os.path.exists(mp):
            man = json.load(open(mp))
        cells.append(dict(cell=os.path.basename(d), dir=d, manifest=man,
                          episodes=[score_episode(f) for f in files]))
    return cells


def group(cells, key):
    out = {}
    for c in cells:
        out.setdefault(key(c), []).append(c)
    return out


def rate(eps, k):
    return sum(1 for e in eps if e['flags'][k]), len(eps)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--records', required=True, help='root holding one directory per cell')
    ap.add_argument('--out', default=None, help='write the full per-episode scoring here (json)')
    a = ap.parse_args()

    cells = cell_rows(a.records)
    assert cells, f'no cells with ep*.npz under {a.records}'
    all_eps = [e for c in cells for e in c['episodes']]

    v = verify(all_eps)
    print('## 0. Verification: offline replay vs the live run (same ladder)\n')
    print(f'- episodes with a live stamp : {v["checked"]} of {len(all_eps)}')
    print(f'- flag disagreements         : {len(v["flag_disagreements"])}')
    print(f'- reward disagreements       : {len(v["reward_disagreements"])}')
    for row in v['flag_disagreements'][:20]:
        print(f'  FLAG   {row[0]} {row[1]}: live={row[2]} offline={row[3]}')
    for row in v['reward_disagreements'][:20]:
        print(f'  REWARD {row[0]}: live={row[1]} offline={row[2]}')
    print()

    # ---- per checkpoint x start set -----------------------------------------------------
    def ck_set(c):
        m = c['manifest']
        return (str(m.get('tag') or '?'), str(m.get('ic_set') or '?'))

    print('## 1. Stage rates per checkpoint x start set (all flags from the record)\n')
    print('| checkpoint | set | n | ' + ' | '.join(f'`{k}`' for k in COLS) + ' |')
    print('|' + '---|' * (len(COLS) + 3))
    per_group = {}
    for (tag, ics), cs in sorted(group(cells, ck_set).items()):
        eps = [e for c in cs for e in c['episodes']]
        per_group[(tag, ics)] = eps
        cnts = [rate(eps, k)[0] for k in COLS]
        print(f'| `{tag}` | `{ics}` | {len(eps)} | ' + ' | '.join(str(x) for x in cnts) + ' |')
    print()

    print('## 2. What each ladder would have paid (Σ reward; the record ladder is the one that ran)\n')
    print('| checkpoint | set | n | record ladder | '
          + ' | '.join(f'Σ `{l}`' for l in LADDERS) + ' | censored for `home` |')
    print('|' + '---|' * (len(LADDERS) + 5))
    for (tag, ics), eps in sorted(per_group.items()):
        rl = eps[0]['record_ladder'] if eps else '?'
        sums = [sum(e['ladders'][l]['reward'] for e in eps) for l in LADDERS]
        cens = sum(1 for e in eps if e['censored_for_home'])
        print(f'| `{tag}` | `{ics}` | {len(eps)} | `{rl}` | '
              + ' | '.join(f'{s:.1f}' for s in sums) + f' | {cens} |')
    print()

    print('## 3. Rungs each ladder would have GRANTED (its own terminal applies)\n')
    print('| checkpoint | set | ladder | n | picked | placed_v2 | farside | slide_event | home | '
          'nested_v2 | slide_success | end=terminal |')
    print('|' + '---|' * 12)
    for (tag, ics), eps in sorted(per_group.items()):
        for lad in LADDERS:
            g = [e['ladders'][lad] for e in eps]

            def n_of(k, _g=g):
                return sum(1 for x in _g if k in x['grants'])
            term = sum(1 for x in g if x['end_reason'] not in ('truncated', 'tipped', 'stream_exhausted'))
            print(f'| `{tag}` | `{ics}` | `{lad}` | {len(g)} | '
                  f'{n_of("picked")} | {n_of("placed_v2")} | {sum(1 for x in g if x["farside"])} | '
                  f'{sum(1 for x in g if x["slide_event"])} | {sum(1 for x in g if x["home"])} | '
                  f'{sum(1 for x in g if x["nested_v2"])} | {n_of("slide_success")} | {term} |')
    print()

    # ---- confusion: the OLD slide_success against the calibrated predicates --------------
    print('## 4. Confusion: old `slide_success` vs calibrated `slide_event` / `home`\n')
    pos = [e for e in all_eps if e['flags']['slide_success']]
    ev = [e for e in all_eps if e['flags']['slide_event']]
    hm = [e for e in all_eps if e['flags']['home']]
    both_ev = [e for e in pos if e['flags']['slide_event']]
    both_hm = [e for e in pos if e['flags']['home']]
    print(f'- episodes                           : {len(all_eps)}')
    print(f'- old `slide_success` positives      : {len(pos)}')
    print(f'- calibrated `slide_event`           : {len(ev)}')
    print(f'- calibrated `home`                  : {len(hm)}')
    print(f'- old positives that survive as `slide_event` : {len(both_ev)} / {len(pos)}')
    print(f'- old positives that survive as `home`        : {len(both_hm)} / {len(pos)}')
    print(f'- `home` without an old positive              : {len(hm) - len(both_hm)}')
    print(f'- `slide_event` without an old positive       : {len(ev) - len(both_ev)}')
    print()
    if pos:
        print('| episode | uid | `slide_event` | `home` | `settled_after_release` | '
              'slide_gain (mm) | gain during release (mm) | release dist (cm) | end |')
        print('|' + '---|' * 9)
        for e in pos:
            d = e['diag']
            print(f'| `{e["file"]}` | {e["uid"] if e["uid"] >= 0 else "rnd"} | '
                  f'{int(e["flags"]["slide_event"])} | {int(e["flags"]["home"])} | '
                  f'{int(d["settled_after_release"])} | {d["slide_gain_m"] * 1000:.1f} | '
                  f'{d["gain_during_release_m"] * 1000:.1f} | {d["release_dist_m"] * 100:.1f} | '
                  f'{e["live_end_reason"]} |')
        print()
        # WHY each old positive fails, decomposed against the two clauses that can kill it. The
        # two are reported separately because they say different things: a missing `farside` is
        # "the tool was never behind the can", a zero `slide_gain` with `farside` granted is
        # "the tool was behind it but the can did not travel from there".
        dead = [e for e in pos if not e['flags']['slide_event']]
        no_far = [e for e in dead if not e['ladders'][e['record_ladder']]['farside']]
        far_nogain = [e for e in dead if e['ladders'][e['record_ladder']]['farside']]
        nolatch = [e for e in pos if not e['diag']['settled_after_release']]
        print(f'**Why the {len(dead)} non-surviving old positives fail** '
              f'(`settled_after_release` is not the discriminator here: {len(nolatch)} of '
              f'{len(pos)} failed to reach it):\n')
        print(f'- `farside` NEVER granted -- the tool was never on the opposite side of the can '
              f'within the reach: **{len(no_far)}**')
        print(f'- `farside` granted but less than the minimum goalward travel credited from '
              f'there: **{len(far_nogain)}**')
        gz = [e['diag']['slide_gain_m'] * 1000 for e in dead]
        if gz:
            print(f'- credited `slide_gain` on those {len(dead)}: max {max(gz):.1f} mm, '
                  f'{sum(1 for x in gz if x == 0)} are exactly 0.0 mm')
        exc = [e['diag']['gain_during_release_m'] * 1000 for e in dead]
        if exc:
            print(f'- goalward drift the SET-DOWN LATCH excluded on those {len(dead)} (mm): '
                  f'p50 {np.median(exc):.1f}, p90 {np.percentile(exc, 90):.1f}, max {max(exc):.1f}; '
                  f'{sum(1 for x in exc if x >= 10)} of {len(dead)} clear the old 10 mm `pushed` '
                  f'threshold on that drift alone')
        print()

    cen = [e for e in all_eps if e['censored_for_home']]
    print('## 4b. Right-censoring for `home`\n')
    print(f'- episodes that ended at the record ladder\'s own PAID terminal without `home`: '
          f'**{len(cen)}** of {len(all_eps)}. On these, `home` under a nested ladder is UNKNOWN, '
          f'not 0: that ladder would not have stopped the episode there.')
    if cen:
        nf = sum(1 for e in cen if not e['ladders'][e['record_ladder']]['farside'])
        gz = [e['diag']['slide_gain_m'] * 1000 for e in cen]
        print(f'- of those, `farside` was never granted in the recorded frames: {nf}')
        print(f'- credited `slide_gain` at the cut (mm): max {max(gz):.1f}, '
              f'{sum(1 for x in gz if x == 0)} are exactly 0.0 -- i.e. how far from `home` the '
              f'episode was when the record stopped')
        print(f'- all of them already satisfy `nested_v2`: '
              f'{sum(1 for e in cen if e["flags"]["nested_v2"])} of {len(cen)}')
    print()

    print('## 5. `slide_event` / `home` episodes in full\n')
    if ev:
        print('| episode | uid | checkpoint | set | old `slide_success` | `home` | '
              'slide_gain (mm) | `nested_v2` | `nested_honest` |')
        print('|' + '---|' * 9)
        by_file = {}
        for (tag, ics), eps in per_group.items():
            for e in eps:
                by_file[id(e)] = (tag, ics)
        for e in ev:
            tag, ics = by_file.get(id(e), ('?', '?'))
            print(f'| `{e["file"]}` | {e["uid"] if e["uid"] >= 0 else "rnd"} | `{tag}` | `{ics}` | '
                  f'{int(e["flags"]["slide_success"])} | {int(e["flags"]["home"])} | '
                  f'{e["diag"]["slide_gain_m"] * 1000:.1f} | {int(e["flags"]["nested_v2"])} | '
                  f'{int(e["flags"]["nested_honest"])} |')
    else:
        print('_No episode in this set satisfies `slide_event`._')
    print()

    if a.out:
        json.dump(dict(records=os.path.abspath(a.records), ladders=list(LADDERS),
                       verification=dict(checked=v['checked'],
                                         flag_disagreements=v['flag_disagreements'],
                                         reward_disagreements=v['reward_disagreements']),
                       cells=[dict(cell=c['cell'], manifest=c['manifest'],
                                   episodes=c['episodes']) for c in cells]),
                  open(a.out, 'w'), indent=1, default=str)
        print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
