"""Closed-loop RE-RECORD of the human demos in the delta_joint (measured-ref) env.

WHY (P1, paper/p1_delta_divergence_2026-08-13.md; FABLE_HANDOFF_2026-08-13.md §8):
the delta representation cannot be RE-ENCODED from the recorded tape without losing
the demonstrated downstream phases.

  * `delta_ref='target'` (running-target integration): one cap-clipped frame leaves a
    PERMANENT offset in the integrated target and the open-loop replay never
    re-converges (frozen drift). Census: >=picked 42 / placed 40 / contact 5 / nested 1
    over the 72 resettable demos, against recorded labels 65/62/25/16.
  * `delta_ref='measured'` re-encoded from the tape (2026-08-14, commit 97e9526):
    cap-scaled = 5x under-drive (pick gate 0/5 -- the encoding IS the PD lead, whose
    p99 ~ the leash, not the cap); leash-scaled = correct drive, gate 3/5. But a TAPE
    in a self-referenced space reproduces DRIVE, not POSITION: once the arm is a few
    mm off the demonstrated path nothing in the tape pulls it back, because every
    action is measured relative to wherever the arm already is.

THE FIX IMPLEMENTED HERE: do not re-encode -- RE-RECORD. Drive the delta env with a
CLOSED-LOOP waypoint follower whose waypoints are the demo's own recorded absolute
joint commands, and write down what the env actually executed. Positions are then
tracked (not merely differentiated), and the produced (state, action) tape is
self-consistent in the delta space by construction -- the same thing
`derive_cartesian_realized` did for the cartesian arm.

FOLLOWER (one env step = one sim decision, action_repeat=1):

    waypoint j carries   cmd_j  = src actions[j, :6]   (absolute joint target, rad)
                         grip_j = src actions[j, 6]    (0..1)
                         ref_j  = src states[j+1, :6]  (the pose the demo ARRIVED at
                                                        after issuing cmd_j)

    each step:  a_arm = clip((cmd_j - q_meas_now) / LEASH, -1, 1)
                a_grip = grip_j * 2 - 1
    the env (delta_ref='measured') then commands
                target = clip(q_meas_now + a_arm*LEASH, ARM_LO, ARM_HI)
    i.e. exactly cmd_j whenever the demo's PD lead fits inside the leash, and a
    leash-limited step toward it otherwise. ON-TRACK THE FOLLOWER IS BIT-FAITHFUL TO
    THE DEMO'S OWN COMMAND; off-track it corrects, which the tape could not do.

    advance j when  ||q_meas - ref_j||_inf < TOL          (arrived)
             or     dwell >= MAX_DWELL                    (unreachable -- counted)

    Progress is measured against the demo's recorded MEASURED pose, not against the
    commanded target: the demo's command LEADS its measured pose by a median 0.045 rad
    (p99 0.23), so "arrive at the command" is a condition the demonstrator himself
    never met and would stall on every waypoint. This makes the follower a strict
    generalisation of the tape: when tracking is perfect it advances every step and
    reproduces the tape exactly; when it falls behind it spends extra steps (TIME
    DILATION) and pulls back onto the demonstrated path.

OUTPUT (default baselines/episodes_delta_rerecord/<uid>.npz), schema-compatible with
baselines/episodes_all:
    states      (n,17) float32   obs BEFORE each step (same convention as the collector)
    actions     (n,7)  float32   the ABSOLUTE command the env actually issued
                                 [6 joint targets rad, grip 0..1]
    actions_delta (n,7) float32  the normalized [-1,1]^7 action commanded to the env
    rewards     (n,)   float32   FullTaskEnv staged reward per step
    dones       (n,)   bool      env terminated flag per step (recording continues)
    uid, n, label, stage
`actions` is stored in the ABSOLUTE convention on purpose. It round-trips EXACTLY
through train_sacfd_full.delta_encode_transitions_measured (leash-scaled):
    (target - q_meas)/LEASH == a_arm  and  grip*2-1 == a_grip,
so the existing encoders and every loader that reads episodes_all keep working
unchanged, and `actions_delta` is carried alongside only as a verification copy.
(Verified per episode at write time; max round-trip error is in the manifest.)

`stage` uses the COLLECTOR's semantics (collect_all_classified.py), NOT the env's
proxy: picked/placed/contact from the env's own honest predicates, and nested only
from a settled score -- hold the last waypoint 100 sim steps, then
picked AND xy-proximity <= NESTED_TOUCH_DIST AND both cans upright. P1 §8.6: never
report proxy-derived nested. The env's proxy grants are kept separately in the
manifest as `granted_proxy` so the two are never confused.

CAVEAT (P2, handoff §4a-0): episodes replayed sequentially in one process carry
residual cross-episode solver state. The controller-target fix (e9f6e24) closed the
dominant channel but solver-cache residue remains; borderline episodes flip. Read the
CENSUS AGGREGATE, never a single marginal demo. Shard with one env per PROCESS and
record the split (--shard-idx/--shard-n) so the schedule is reproducible.

Usage:
    ./.venv-eval/bin/python baselines/rl/rerecord_delta_demos.py --uids 308 325 297 326 265
    ./.venv-eval/bin/python baselines/rl/rerecord_delta_demos.py --shard-idx 0 --shard-n 12
    ./.venv-eval/bin/python baselines/rl/rerecord_delta_demos.py --merge      # no sim
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

# TOL: one delta_cap. The cap is the demos' p99 per-frame commanded joint step, so a
# tolerance of one cap means "within one demonstrated frame of motion of where the
# demo arm was" -- the finest resolution the delta action space can express.
DEFAULT_TOL = 0.025
# MAX_DWELL: give up on a waypoint the arm cannot reach. Bounds the worst-case time
# dilation at MAX_DWELL x and stops one unreachable pose from consuming the budget.
DEFAULT_MAX_DWELL = 8
# Total step budget per demo, as a multiple of the source frame count.
DEFAULT_DILATION_CAP = 3.0
DEFAULT_SETTLE = 100          # mirrors collect_all_classified's settle before scoring

STAGE_ORDER = ['no-pick', 'picked', 'placed', 'contact', 'nested']


def _np(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def rerecord_one(env, src_path, tol, max_dwell, dilation_cap, settle, repeat=1):
    """Closed-loop follow one demo. Returns (arrays dict, record dict) or (None, rec)
    if the uid is not resettable."""
    from replay_harness import tilt_deg, NESTED_TOUCH_DIST
    from train_sacfd_full import STAGE_RANK

    d = np.load(src_path, allow_pickle=True)
    uid = int(d['uid'])
    label = str(d['label']) if 'label' in d.files else '?'
    src_stage = str(d['stage']) if 'stage' in d.files else '?'
    S = d['states'].astype(np.float64)
    A = d['actions'].astype(np.float64)
    n_src = len(S)
    rec = dict(uid=uid, label=label, src_stage=src_stage, n_src_frames=int(n_src))
    if n_src < 3:
        rec.update(status='TOO_SHORT')
        return None, rec

    cmd = A[:, :6]
    grip = np.clip(A[:, 6], 0.0, 1.0)
    # ref_j = the measured pose the demo reached AFTER issuing cmd_j. states[i] is the
    # obs BEFORE actions[i] (collector convention), so that is states[j+1]; the final
    # waypoint has no successor frame and reuses the last recorded pose.
    ref = np.concatenate([S[1:, :6], S[-1:, :6]], axis=0)

    # step_cap counts DECISIONS. At repeat>1 each decision spans `repeat` sim steps
    # and may advance up to `repeat` waypoints, so the decision budget divides by it.
    step_cap = int(np.ceil(dilation_cap * n_src / repeat))
    env.max_steps = (step_cap + settle) * repeat + 10   # env truncation in SIM steps
    t0 = time.time()
    try:
        obs, _info = env.reset(options={'uid': uid})
    except Exception as e:
        rec.update(status=f'UNRESETTABLE:{type(e).__name__}')
        return None, rec

    leash = float(env.delta_leash)
    states, acts_abs, acts_dn, rews, dones = [], [], [], [], []
    j = dwell = stalls = 0
    term_step = -1
    tipped = False
    steps = 0
    while j < n_src and steps < step_cap:
        q = np.asarray(obs[:6], dtype=np.float64)
        # decision-level window (skip-N): aim at the window-END waypoint and take its
        # grip -- the same window rules as delta_encode_transitions_*_repeat (grip =
        # last frame in window). At repeat==1 this is exactly the original follower.
        jt = min(j + repeat - 1, n_src - 1)
        a_arm = np.clip((cmd[jt] - q) / leash, -1.0, 1.0)
        a = np.concatenate([a_arm, [grip[jt] * 2.0 - 1.0]]).astype(np.float32)
        states.append(np.asarray(obs, dtype=np.float32))
        obs, r, term, trunc, info = env.step(a)
        steps += 1
        # the absolute command the env actually issued this step
        acts_abs.append(np.concatenate([
            np.asarray(env._dj_target, dtype=np.float64),
            [(float(np.clip(a[6], -1.0, 1.0)) + 1.0) / 2.0]]).astype(np.float32))
        acts_dn.append(a)
        rews.append(float(r))
        dones.append(bool(term))
        if term and term_step < 0:
            term_step = steps
            tipped = bool(info.get('tipped'))
        qn = np.asarray(obs[:6], dtype=np.float64)
        # advance to the FURTHEST arrived waypoint in the window (mid-window points
        # are passed in transit during the repeat sim steps; requiring each at the
        # decision boundary would spuriously dwell). repeat==1 degenerates to the
        # original single-waypoint check.
        adv = -1
        for k in range(jt, j - 1, -1):
            if float(np.max(np.abs(qn - ref[k]))) < tol:
                adv = k
                break
        if adv >= 0:
            j = adv + 1
            dwell = 0
        else:
            dwell += 1
            if dwell >= max_dwell:
                j = jt + 1
                dwell = 0
                stalls += 1
    truncated = j < n_src

    # --- stage scoring, mirroring collect_all_classified exactly -------------------
    # picked/placed/contact are the env's OWN honest predicates, snapshotted BEFORE
    # the settle (the collector likewise only accumulates them in its command loop).
    granted = set(env._granted)
    picked = bool(env.genv._picked)
    placed = bool(env.genv._placed)
    contact = bool(env.genv._contact)
    # settle: hold the LAST waypoint (closed-loop position hold), then score nested
    for _ in range(max(1, settle // repeat)):
        q = np.asarray(obs[:6], dtype=np.float64)
        a_arm = np.clip((cmd[-1] - q) / leash, -1.0, 1.0)
        a = np.concatenate([a_arm, [grip[-1] * 2.0 - 1.0]]).astype(np.float32)
        obs, _r, _term, _trunc, _info = env.step(a)
    w = env.genv.w
    bp_, gp_ = _np(w['bottle'].get_pos()), _np(w['goal'].get_pos())
    touch = float(np.hypot(bp_[0] - gp_[0], bp_[1] - gp_[1])) <= NESTED_TOUCH_DIST
    nested = bool(picked and touch and tilt_deg(_np(w['bottle'].get_quat())) < 20
                  and tilt_deg(_np(w['goal'].get_quat())) < 20)
    stage = ('nested' if nested else 'contact' if contact else 'placed' if placed
             else 'picked' if picked else 'no-pick')

    states = np.stack(states).astype(np.float32)
    acts_abs = np.stack(acts_abs).astype(np.float32)
    acts_dn = np.stack(acts_dn).astype(np.float32)
    # round-trip check: the stored ABSOLUTE actions must re-encode to the normalized
    # actions actually commanded (train_sacfd_full.delta_encode_transitions_measured)
    rt = np.clip((acts_abs[:, :6].astype(np.float64)
                  - states[:, :6].astype(np.float64)) / leash, -1.0, 1.0)
    rt_err = float(np.max(np.abs(rt - acts_dn[:, :6].astype(np.float64))))

    arrays = dict(states=states, actions=acts_abs, actions_delta=acts_dn,
                  rewards=np.asarray(rews, np.float32),
                  dones=np.asarray(dones, bool),
                  uid=uid, n=len(states), label=label, stage=stage,
                  # ENCODING STAMP (newbox_supp audit 08-14): actions_delta is
                  # LEASH-referenced (a = (target - qmeas)/0.125), NOT cap-referenced.
                  # Re-deriving with cap yields a clean 5x error that looks like
                  # corruption. Same class as the action_mode sidecar; the tape
                  # carries its own semantics so a loader can assert, not assume.
                  delta_ref='measured', delta_scale=float(env.delta_leash),
                  action_repeat=int(repeat))
    rec.update(status='OK', stage=stage, n_rerecord_steps=int(steps),
               dwell_stalls=int(stalls), truncated=bool(truncated),
               dilation=float(steps) / float(n_src),
               granted_proxy=sorted(granted, key=lambda s: STAGE_RANK.get(s, 0)),
               proxy_nested=bool('nested' in granted), settled_nested=bool(nested),
               term_step=int(term_step), tipped=bool(tipped),
               roundtrip_err=rt_err, wall_s=round(time.time() - t0, 1))
    return arrays, rec


def run(args):
    from full_env import FullTaskEnv

    outdir = REPO / args.outdir
    outdir.mkdir(parents=True, exist_ok=True)
    paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')),
                   key=lambda p: int(pl.Path(p).stem))
    if args.uids:
        want = set(int(u) for u in args.uids)
        paths = [p for p in paths if int(pl.Path(p).stem) in want]
    if args.shard_n > 1:
        paths = paths[args.shard_idx::args.shard_n]   # interleaved: balances lengths
    assert paths, 'no demos selected'

    # ONE env per PROCESS (P2): shard across processes, never threads.
    env = FullTaskEnv(backend='cpu', max_steps=4000, scope='full',
                      action_repeat=args.action_repeat,
                      action_mode='delta_joint',
                      delta_ref='measured')
    print(f'[rerec] env built | delta_ref=measured cap={env.delta_cap} '
          f'leash={env.delta_leash} pick_z={env.pick_z:.4f} | tol={args.tol} '
          f'max_dwell={args.max_dwell} dilation_cap={args.dilation_cap} '
          f'shard={args.shard_idx}/{args.shard_n} n_demos={len(paths)}', flush=True)

    recs = []
    for p in paths:
        arrays, rec = rerecord_one(env, p, args.tol, args.max_dwell,
                                   args.dilation_cap, args.settle,
                                   repeat=args.action_repeat)
        if arrays is not None and not args.no_write:
            np.savez_compressed(outdir / f"{rec['uid']}.npz", **arrays)
        recs.append(rec)
        print('RERECORD ' + ' '.join(f'{k}={v}' for k, v in rec.items()), flush=True)
    man = outdir / f'_manifest_shard{args.shard_idx}of{args.shard_n}.json'
    man.write_text(json.dumps(dict(
        config=dict(demo_dir=args.demo_dir, action_repeat=args.action_repeat,
                    tol=args.tol, max_dwell=args.max_dwell,
                    dilation_cap=args.dilation_cap, settle=args.settle,
                    delta_ref='measured', delta_cap=env.delta_cap,
                    delta_leash=env.delta_leash,
                    shard_idx=args.shard_idx, shard_n=args.shard_n),
        records=recs), indent=1))
    print(f'[rerec] DONE shard {args.shard_idx}/{args.shard_n} -> {man}', flush=True)


def merge(args):
    """No sim: union the shard manifests and print the census table."""
    outdir = REPO / args.outdir
    recs, cfgs = {}, []
    for f in sorted(outdir.glob('_manifest_shard*.json')):
        blob = json.loads(f.read_text())
        cfgs.append(blob['config'])
        for r in blob['records']:
            recs[int(r['uid'])] = r
    assert recs, f'no shard manifests in {outdir}'
    (outdir / '_manifest.json').write_text(
        json.dumps(dict(configs=cfgs, records=[recs[u] for u in sorted(recs)]), indent=1))

    ok = [r for r in recs.values() if r.get('status') == 'OK']
    unres = [r for r in recs.values() if r.get('status', '').startswith('UNRESETTABLE')]
    rank = {s: i for i, s in enumerate(STAGE_ORDER)}

    def tbl(rs, key):
        c = {s: sum(1 for r in rs if rank[r[key]] >= i) for i, s in enumerate(STAGE_ORDER)}
        return c

    print(f'demos={len(recs)}  rerecorded={len(ok)}  unresettable={len(unres)} '
          f'{sorted(r["uid"] for r in unres)}')
    re_c = tbl(ok, 'stage')
    src_c = tbl(ok, 'src_stage')
    print(f'{"":10s} {">=picked":>9s} {">=placed":>9s} {">=contact":>9s} {">=nested":>9s}')
    for nm, c in (('recorded', src_c), ('re-record', re_c)):
        print(f'{nm:10s} {c["picked"]:9d} {c["placed"]:9d} {c["contact"]:9d} '
              f'{c["nested"]:9d}')
    dil = np.array([r['dilation'] for r in ok])
    st = np.array([r['dwell_stalls'] for r in ok])
    print(f'dilation  median={np.median(dil):.2f}x  mean={dil.mean():.2f}x  '
          f'p90={np.percentile(dil, 90):.2f}x  max={dil.max():.2f}x')
    print(f'truncated={sum(1 for r in ok if r["truncated"])}  '
          f'dwell_stalls median={np.median(st):.0f} max={st.max():.0f}  '
          f'proxy_nested={sum(1 for r in ok if r["proxy_nested"])} '
          f'settled_nested={sum(1 for r in ok if r["settled_nested"])}  '
          f'max_roundtrip_err={max(r["roundtrip_err"] for r in ok):.2e}')
    # confusion: recorded label vs re-recorded stage
    print('recorded -> re-recorded:')
    for s in STAGE_ORDER:
        row = [sum(1 for r in ok if r['src_stage'] == s and r['stage'] == t)
               for t in STAGE_ORDER]
        if sum(row):
            print(f'  {s:8s} n={sum(row):3d}  ' +
                  '  '.join(f'{t}={v}' for t, v in zip(STAGE_ORDER, row) if v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demo-dir', default='baselines/episodes_all')
    ap.add_argument('--outdir', default='baselines/episodes_delta_rerecord')
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--shard-idx', type=int, default=0)
    ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--tol', type=float, default=DEFAULT_TOL)
    ap.add_argument('--max-dwell', type=int, default=DEFAULT_MAX_DWELL)
    ap.add_argument('--dilation-cap', type=float, default=DEFAULT_DILATION_CAP)
    ap.add_argument('--settle', type=int, default=DEFAULT_SETTLE)
    ap.add_argument('--action-repeat', type=int, default=1,
                    help='decision-level skip-N: one follower decision spans N sim '
                         'steps and may advance up to N waypoints (window-end '
                         'target + window-end grip, matching the repeat encoders). '
                         'OUTDIR should encode N; the manifest records it.')
    ap.add_argument('--no-write', action='store_true', help='pilot: sim but do not save npz')
    ap.add_argument('--merge', action='store_true', help='no sim: union shard manifests')
    args = ap.parse_args()
    if args.merge:
        merge(args)
    else:
        run(args)


if __name__ == '__main__':
    main()
