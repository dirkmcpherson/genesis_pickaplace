#!/usr/bin/env python
"""D5 -- relabel a demonstration set's reward column BY RE-EXECUTION through the training env.

    python baselines/rl/relabel_reward.py --in <set> --out <set>_rz [--procs 8] [--limit N]

WHY THIS IS A RE-EXECUTION AND NOT A CLASSIFIER (LADDER_UNIFY_BRIEF D5)
----------------------------------------------------------------------
The previous version of this file scored tapes with its OWN offline predicates (a release
detector, a proximity test, a settle test) and wrote the resulting reward into the demo set.
That is the defect it was meant to fix, one level up: the demo buffer then pays a ladder the
environment cannot. Measured consequence -- the `_rx` prefill pays +4 on 13 of 74 human
tapes while the online env, under the ladder those runs trained on, could never pay it at
all (E2E_AUDIT_BRIEF defect 5). Buffer and env disagreed about the objective.

Re-executing the tape through `FullTaskEnv(scope='full')` -- the SAME code path training
uses, same world, same action semantics -- makes `tape reward == env reward` true BY
CONSTRUCTION rather than by two implementations agreeing. There is no predicate in this
file. It runs the env and writes down what the env paid.

WHAT CHANGES AND WHAT DOES NOT
------------------------------
Changed:   `rewards` (per decision, the unified ladder as the env paid it).
Added:     `rz_*` diagnostics (stage-grant decisions, the re-execution's end reason, the
           agreement between recorded and re-executed stage flags) and `reward_ladder`.
Unchanged: EVERYTHING else, and the action stream in particular -- the sha256 of
           `actions_delta` is asserted identical to the source before anything is written,
           per tape and over the whole set. A set whose actions moved is not a relabel.

WHERE THE REWARD CAN LEGITIMATELY DIFFER FROM THE RECORDING
-----------------------------------------------------------
The recorded tapes were produced under the OLD ladder, which terminated on the `nested`
proxy. Under the unified ladder the episode instead terminates on `slide_success` or a tip,
so a re-execution can run PAST the decision the recording stopped at, or stop before it.
Both are reported per tape (`rz_end_reason`, `rz_end_decision`) and counted in the manifest.
Decisions after the re-execution's terminal are paid 0 -- the env is in an absorbing state
and a demo row there would teach a transition that cannot occur.

ONE GENESIS WORLD PER PROCESS is a hard constraint, so `--procs N` (N <= 8) re-launches this
same script as N subprocesses with `--shard i --nshards N`; each builds one world and takes
every N-th tape. The driver merges the shard manifests.

The sim variant defaults to the tapes' own `sim_variant` stamp and is asserted, not assumed.
"""
import argparse
import glob
import hashlib
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))

# The stages whose first grant is recorded per tape. `slide_success` is the paid terminal;
# `nested_v2` / the legacy names are logged so a reader can see WHY a tape paid what it did.
REPORT_STAGES = ('picked', 'placed_v2', 'contact_push', 'slide_success',
                 'nested_v2', 'released', 'pushed', 'placed', 'contact', 'nested')
MAX_PROCS = 8


def sha_actions(a):
    return hashlib.sha256(np.ascontiguousarray(np.asarray(a, np.float32)).tobytes()).hexdigest()


def scalar(z, k, default=None):
    if k not in z.files:
        return default
    a = np.asarray(z[k])
    return a.item() if a.shape == () else a


# ------------------------------------------------------------------------------- worker
def build_env(sim_variant, max_sim_steps):
    """FullTaskEnv in the END-TO-END contract MDP -- the same knobs baselines/record_demos.py
    build_env and cluster/sbatch_rlpd_e2e.sh use, every one asserted (silent-default rule)."""
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    os.environ['GENESIS_SIM_VARIANT'] = sim_variant
    from sim_variant_hook import apply_pre, apply_post
    from full_env import FullTaskEnv, refuse_legacy_gates
    refuse_legacy_gates()
    apply_pre(sim_variant)
    t0 = time.time()
    env = FullTaskEnv(backend='cpu', max_steps=int(max_sim_steps), scope='full',
                      action_mode='delta_joint', delta_cap=0.025, delta_leash_mult=5.0,
                      action_repeat=4, delta_ref='target', camera_rig=False)
    apply_post(env, sim_variant)
    assert env.scope == 'full' and env.action_mode == 'delta_joint' and env.delta_ref == 'target'
    assert env.action_repeat == 4 and abs(env.delta_cap - 0.025) < 1e-12
    assert abs(env.delta_leash - 0.125) < 1e-12, env.delta_leash
    assert env.genv.max_steps >= 10 ** 8, 'inner env must never truncate (#26)'
    assert not env.goalward_shaping, 'a relabel must use the unshaped ladder'
    print(f'[env] built in {time.time() - t0:.1f}s | variant {sim_variant} | max_sim {env.max_steps} | '
          f'shelf_top_z {env.shelf_top_z:.3f}', flush=True)
    return env


def reset_to_tape_ic(env, z):
    """Restore the tape's initial condition and PROVE it matched.

    uid ICs go through the env's own `reset(options={'uid': ...})`, so the placement table,
    the corrected static goal and every piece of FullTaskEnv bookkeeping are the env's code,
    not a copy of it. The restored can pose is then checked against the tape's own first
    state -- an IC that silently differs is the whole failure mode of a re-execution."""
    ic_uid = scalar(z, 'ic_uid')
    s0 = np.asarray(z['states'], np.float64)[0]
    if ic_uid is not None and int(ic_uid) in env.genv.placements:
        obs, _ = env.reset(options={'uid': int(ic_uid)})
        how = f'uid {int(ic_uid)}'
    else:
        from replay_harness import STATIC_BOTTLE_POSITION
        obs, _ = env.reset_to(dict(can_pos=[float(v) for v in s0[8:11]],
                                   can_quat=[float(v) for v in s0[11:15]],
                                   goal_pos=(float(STATIC_BOTTLE_POSITION[0]),
                                             float(STATIC_BOTTLE_POSITION[1]),
                                             float(env.genv.w['goal_start_z']))))
        how = 'pose'
    env.genv._calib_tool_offset()
    got = np.asarray(obs, np.float64)
    d_can = float(np.linalg.norm(got[8:11] - s0[8:11]))
    d_goal = float(np.linalg.norm(got[15:17] - s0[15:17]))
    return how, d_can, d_goal


def relabel_one(env, path, out_dir, ic_tol):
    z = np.load(path, allow_pickle=True)
    assert str(scalar(z, 'scope')) == 'full', f'{path}: scope={scalar(z, "scope")!r}, expected full'
    assert str(scalar(z, 'contract')) == 'v1', f'{path}: not a contract-v1 tape'
    n = int(scalar(z, 'n'))
    acts = np.asarray(z['actions_delta'], np.float32)[:n]
    src_sha = sha_actions(acts)

    how, d_can, d_goal = reset_to_tape_ic(env, z)
    assert d_can <= ic_tol and d_goal <= ic_tol, (
        f'{os.path.basename(path)}: restored IC differs from the tape ({how}): can {d_can * 1000:.1f} mm, '
        f'goal {d_goal * 1000:.1f} mm > {ic_tol * 1000:.1f} mm. Refusing to relabel a tape against a '
        f'different start.')

    rew = np.zeros(n, np.float32)
    grants, end_reason, end_dec = {}, 'stream_exhausted', n
    seen = set()
    tipped = False
    t0 = time.time()
    for t in range(n):
        _, r, term, trunc, info = env.step(acts[t])
        rew[t] = float(r)
        tipped = tipped or bool(info.get('tipped'))
        gr = set(env._granted) | {k for k in REPORT_STAGES if info.get(k)}
        for k in gr - seen:
            if k in REPORT_STAGES:
                grants[k] = int(t)
        seen |= gr
        if term or trunc:
            end_reason = ('tipped' if info.get('tipped') else
                          'slide_success' if info.get('slide_success') else
                          'truncated' if trunc else 'terminated')
            end_dec = t + 1
            break

    d = {k: z[k] for k in z.files}
    assert sha_actions(np.asarray(d['actions_delta'], np.float32)[:n]) == src_sha, 'action stream moved'
    old_sum = float(np.asarray(d['rewards'], np.float64)[:n].sum()) if 'rewards' in d else 0.0
    d['rewards'] = rew
    d['reward_ladder'] = json.dumps(dict(__import__('full_env').STAGE_REWARD))
    d['reward_relabelled'] = 'reexecution_2026-09-10'
    d['rz_grants'] = json.dumps(grants)
    d['rz_end_reason'] = end_reason
    d['rz_end_decision'] = np.int64(end_dec)
    d['rz_decisions'] = np.int64(n)
    d['rz_actions_sha256'] = src_sha
    # agreement between what the RECORDING said and what the re-execution did. Reported,
    # never asserted: the recording ran under the old ladder, whose terminal differed, so a
    # disagreement is information about the two MDPs rather than a failure of this script.
    agree = {}
    for k in ('picked', 'contact', 'nested'):
        if k in z.files:
            agree[k] = [bool(np.asarray(z[k], bool)[:n].any()), bool(k in seen)]
    if 'tipped' in z.files:
        # `tipped` is NOT a stage and never enters `_granted`, so it has to be tracked from
        # info -- reading it out of `seen` reported False for every tipped re-execution.
        agree['tipped'] = [bool(np.asarray(z['tipped'], bool)[:n].any()), bool(tipped)]
    d['rz_recorded_vs_reexecuted'] = json.dumps(agree)
    out = os.path.join(out_dir, os.path.basename(path))
    np.savez_compressed(out, **d)
    return dict(file=os.path.basename(path), n=n, ic=how, ic_can_mm=round(d_can * 1000, 2),
                ic_goal_mm=round(d_goal * 1000, 2), old_reward=old_sum,
                new_reward=float(rew.sum()), grants=grants, end_reason=end_reason,
                end_decision=end_dec, actions_sha256=src_sha,
                agree=agree, seconds=round(time.time() - t0, 1))


def run_shard(args, files):
    z0 = np.load(files[0], allow_pickle=True)
    sv = args.sim_variant or str(scalar(z0, 'sim_variant'))
    max_sim = int(args.max_sim_steps or scalar(z0, 'max_sim_steps', 2400))
    env = build_env(sv, max_sim)
    rows = []
    for i, f in enumerate(files):
        z = np.load(f, allow_pickle=True)
        assert str(scalar(z, 'sim_variant')) == sv, (f, scalar(z, 'sim_variant'), sv)
        assert int(scalar(z, 'action_repeat')) == 4 and abs(float(scalar(z, 'delta_cap')) - 0.025) < 1e-9, f
        assert str(scalar(z, 'delta_ref')) == 'target', f
        r = relabel_one(env, f, args.out, args.ic_tol)
        rows.append(r)
        print(f'[{i + 1}/{len(files)}] {r["file"]}: {r["n"]} decisions, reward {r["old_reward"]:.1f} -> '
              f'{r["new_reward"]:.1f}, end {r["end_reason"]}@{r["end_decision"]}, grants '
              f'{ {k: v for k, v in sorted(r["grants"].items())} } [{r["seconds"]:.0f}s]', flush=True)
    return rows, sv


# -------------------------------------------------------------------------------- driver
def summarize(rows, files, in_dir, out_dir, sv):
    sys.path.insert(0, str(REPO / 'baselines'))
    sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
    import full_env
    per_rung = {k: sum(1 for r in rows if k in r['grants']) for k in REPORT_STAGES}
    ends = {}
    for r in rows:
        ends[r['end_reason']] = ends.get(r['end_reason'], 0) + 1
    man = dict(
        set=os.path.basename(os.path.normpath(out_dir)), source=os.path.abspath(in_dir),
        built=time.strftime('%Y-%m-%dT%H:%M:%S'), builder='baselines/rl/relabel_reward.py (D5 re-execution)',
        method=('re-executed through FullTaskEnv(scope=full) on the training code path; the reward column is '
                'what the env paid, not what a classifier scored'),
        sim_variant=sv, scope='full', contract='v1', action_repeat=4, delta_cap=0.025, delta_ref='target',
        n_tapes=len(rows), decisions_total=int(sum(r['n'] for r in rows)),
        reward_total_old=float(sum(r['old_reward'] for r in rows)),
        reward_total_new=float(sum(r['new_reward'] for r in rows)),
        tapes_granting=per_rung, end_reasons=ends,
        actions_sha256=hashlib.sha256(''.join(r['actions_sha256'] for r in
                                              sorted(rows, key=lambda x: x['file'])).encode()).hexdigest(),
        ladder_provenance=full_env.ladder_provenance(), ladder_stamp=full_env.ladder_stamp(),
        per_tape=sorted(rows, key=lambda r: r['file']))
    return man


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in', dest='inp', required=True, help='source set of contract-v1 FULL-scope tapes')
    ap.add_argument('--out', required=True, help="destination; the convention is <set>_rz")
    ap.add_argument('--sim-variant', default=None, help="default: the tapes' own stamp (asserted equal across the set)")
    ap.add_argument('--max-sim-steps', type=int, default=None, help="default: the tapes' own max_sim_steps stamp")
    ap.add_argument('--ic-tol', type=float, default=0.002,
                    help='m; the restored can/goal must match the tape first state within this (default 2 mm)')
    ap.add_argument('--procs', type=int, default=1, help=f'parallel worker processes, <= {MAX_PROCS} (one world each)')
    ap.add_argument('--limit', type=int, default=None, help='first N tapes only (dry runs)')
    ap.add_argument('--shard', type=int, default=None, help='worker mode: this shard index')
    ap.add_argument('--nshards', type=int, default=None, help='worker mode: total shards')
    ap.add_argument('--dry-run', action='store_true', help='print the plan and exit')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.inp, '*.npz')))
    assert files, f'no npz in {args.inp}'
    if args.limit:
        files = files[:int(args.limit)]
    os.makedirs(args.out, exist_ok=True)

    if args.dry_run:
        print(f'[dry] {len(files)} tapes {args.inp} -> {args.out}, procs {args.procs}')
        for f in files[:5]:
            z = np.load(f, allow_pickle=True)
            print('   ', os.path.basename(f), 'n', scalar(z, 'n'), 'ic_uid', scalar(z, 'ic_uid'),
                  'variant', scalar(z, 'sim_variant'), 'recorded reward',
                  float(np.asarray(z['rewards'])[:int(scalar(z, 'n'))].sum()))
        return

    # ---- worker ----
    if args.shard is not None:
        assert args.nshards and 0 <= args.shard < args.nshards
        mine = [f for i, f in enumerate(files) if i % args.nshards == args.shard]
        print(f'[shard {args.shard}/{args.nshards}] {len(mine)} tapes', flush=True)
        rows, sv = run_shard(args, mine)
        json.dump(dict(rows=rows, sim_variant=sv),
                  open(os.path.join(args.out, f'_shard{args.shard}.json'), 'w'), indent=1)
        return

    # ---- driver ----
    n_proc = max(1, min(int(args.procs), MAX_PROCS, len(files)))
    if n_proc == 1:
        rows, sv = run_shard(args, files)
    else:
        base = [sys.executable, os.path.abspath(__file__), '--in', args.inp, '--out', args.out,
                '--ic-tol', str(args.ic_tol)]
        if args.limit:
            base += ['--limit', str(args.limit)]
        if args.sim_variant:
            base += ['--sim-variant', args.sim_variant]
        if args.max_sim_steps:
            base += ['--max-sim-steps', str(args.max_sim_steps)]
        procs = [subprocess.Popen(base + ['--shard', str(i), '--nshards', str(n_proc)])
                 for i in range(n_proc)]
        rcs = [p.wait() for p in procs]
        assert all(rc == 0 for rc in rcs), f'shard failures: {rcs}'
        rows, svs = [], set()
        for i in range(n_proc):
            d = json.load(open(os.path.join(args.out, f'_shard{i}.json')))
            rows += d['rows']; svs.add(d['sim_variant'])
        assert len(svs) == 1, svs
        sv = svs.pop()

    assert len(rows) == len(files), (len(rows), len(files))
    man = summarize(rows, files, args.inp, args.out, sv)
    json.dump(man, open(os.path.join(args.out, 'manifest.json'), 'w'), indent=1)
    print('\n' + '=' * 78)
    print(f'tapes                : {man["n_tapes"]}  ({man["decisions_total"]} decisions)')
    print(f'reward sum  recorded : {man["reward_total_old"]:.1f}')
    print(f'reward sum  RE-EXEC  : {man["reward_total_new"]:.1f}')
    print(f'tapes granting       : { {k: v for k, v in man["tapes_granting"].items() if v} }')
    print(f'episode end reasons  : {man["end_reasons"]}')
    print(f'ladder               : {man["ladder_stamp"]}')
    print(f'wrote                : {args.out}')


if __name__ == '__main__':
    main()
