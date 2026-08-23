#!/usr/bin/env python
"""Sim-fidelity replay: how well does the Genesis arm reproduce the REAL robot's measured
joint trajectory when driven with it as its command stream?

The human source tapes (baselines/episodes_pick_phase_dppruned/<uid>.npz) were recorded by
replaying the REAL measured joint positions (inthewild_trials/<uid>_episodes.npy 'vel_cmd'
== /joint_states positions at ~30 Hz) as absolute PD targets, 3 physics steps (dt 0.01) per
frame. So tape `actions[:, :6]` == the real arm's trajectory, tape `states[:, :6]` == the
sim arm's trajectory under the 'base' simulator. Tracking error e_j = cmd_j - q_sim_{j+1} IS
the real2sim discrepancy of the arm. This tool re-runs that replay under a sim variant
(baselines/sim_variants.py: PD gains, gravity compensation, effort limits) and reports:

  per joint: RMS, p95, max |e|; static sag (command still >= 15 frames); dynamic lag
  per tape: frac frames |e|_inf > leash (0.125), > cap (0.025); hardened pick re-earned?
            first sim pick frame vs the tape's length; can displacement before the grip closes
  --offline: the same metrics straight from the tapes (== 'base' on the collecting machine)

USAGE
  python baselines/sim_fidelity_lab.py --offline
  python baselines/sim_fidelity_lab.py --variant gc_kp4 --parallel 3
  python baselines/sim_fidelity_lab.py --report
Outputs: baselines/demos_v1/_simlab/<variant>/manifest*.json (no tapes).
"""
import argparse
import glob
import json
import os
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
import sim_variants  # noqa: E402

OUT_ROOT = 'baselines/demos_v1/_simlab'
SRC_DEFAULT = 'baselines/episodes_pick_phase_dppruned'
LEASH = 0.125; CAP = 0.025; GRIP_CLOSED = 0.5; PICK_Z = 0.1505


def track_metrics(cmd, q_after, grip, can_xy, picked_frame):
    """cmd (n,6) real measured = command; q_after (n,6) sim measured after applying cmd_j."""
    e = cmd - q_after
    ae = np.abs(e)
    dc = np.abs(np.diff(cmd, axis=0, prepend=cmd[:1])).max(1)
    still = dc < 1e-3
    run = 0; sag_idx = []; dyn_idx = []
    for j in range(len(dc)):
        run = run + 1 if still[j] else 0
        if run >= 15: sag_idx.append(j)
        elif not still[j]: dyn_idx.append(j)
    gc = np.where(grip > GRIP_CLOSED)[0]; gci = int(gc[0]) if len(gc) else len(grip) - 1
    m = dict(n=int(len(cmd)),
             rms=np.sqrt((e ** 2).mean(0)).round(4).tolist(),
             p95=np.percentile(ae, 95, axis=0).round(4).tolist(),
             max=ae.max(0).round(4).tolist(),
             inf_p50=round(float(np.median(ae.max(1))), 4), inf_p95=round(float(np.percentile(ae.max(1), 95)), 4),
             inf_max=round(float(ae.max()), 4),
             frac_over_leash=round(float((ae.max(1) > LEASH).mean()), 4),
             frac_over_cap=round(float((ae.max(1) > CAP).mean()), 4),
             sag_p50=(np.median(ae[sag_idx], 0).round(4).tolist() if sag_idx else None),
             sag_p95=(np.percentile(ae[sag_idx], 95, axis=0).round(4).tolist() if sag_idx else None),
             sag_signed_mean=(e[sag_idx].mean(0).round(4).tolist() if sag_idx else None),
             dyn_p50=(np.median(ae[dyn_idx], 0).round(4).tolist() if dyn_idx else None),
             dyn_p95=(np.percentile(ae[dyn_idx], 95, axis=0).round(4).tolist() if dyn_idx else None),
             err_at_close=ae[max(0, gci - 1)].round(4).tolist(),
             can_move_before_close=round(float(np.linalg.norm(can_xy[gci] - can_xy[0])), 4),
             picked_frame=int(picked_frame))
    return m


def offline(src):
    files = sorted(glob.glob(str(REPO / src / '*.npz')), key=lambda p: int(pl.Path(p).stem))
    recs = []
    for f in files:
        d = np.load(f, allow_pickle=True); S = d['states'].astype(np.float64); A = d['actions'].astype(np.float64)
        cmd = A[:-1, :6]; q_after = S[1:, :6]; grip = np.clip(A[:-1, 6], 0, 1)
        zl = np.where(S[:, 10] > PICK_Z)[0]
        m = track_metrics(cmd, q_after, grip, S[:-1, 8:10], int(zl[0]) if len(zl) else -1)
        m['uid'] = int(d['uid']); recs.append(m)
    out = REPO / OUT_ROOT / 'offline'; out.mkdir(parents=True, exist_ok=True)
    (out / 'manifest.json').write_text(json.dumps(dict(variant='offline(tapes as recorded)', records=recs), indent=1))
    return recs


def summarize(recs):
    A = lambda k: np.array([r[k] for r in recs if r.get(k) is not None], float)
    rms = A('rms'); p95 = A('p95'); mx = A('max')
    sag = A('sag_p95'); dyn = A('dyn_p95')
    return dict(n=len(recs),
                rms_mean=rms.mean(0).round(4).tolist(), p95_med=np.median(p95, 0).round(4).tolist(),
                max_med=np.median(mx, 0).round(4).tolist(),
                inf_p50_med=round(float(np.median(A('inf_p50'))), 4), inf_p95_med=round(float(np.median(A('inf_p95'))), 4),
                frac_over_leash_mean=round(float(A('frac_over_leash').mean()), 4),
                frac_over_cap_mean=round(float(A('frac_over_cap').mean()), 4),
                sag_p95_med=(np.median(sag, 0).round(4).tolist() if len(sag) else None),
                dyn_p95_med=(np.median(dyn, 0).round(4).tolist() if len(dyn) else None),
                picked=int(sum(1 for r in recs if r.get('picked_frame', -1) >= 0)),
                err_at_close_inf_med=round(float(np.median(A('err_at_close').max(1))), 4))


def run_shard(args):
    sim_variants.install(args.variant)
    from record_demos import build_env, Recorder
    env = build_env(argparse.Namespace(no_images=True))
    applied = sim_variants.post_build(env.genv.w, args.variant)
    print(f'[simlab] variant {applied}', flush=True)
    rec = Recorder(env, images=False, sim_tape=False)
    files = sorted(glob.glob(str(REPO / args.src / '*.npz')), key=lambda p: int(pl.Path(p).stem))
    if args.uids:
        want = set(int(u) for u in args.uids); files = [f for f in files if int(pl.Path(f).stem) in want]
    files = files[args.shard_idx::args.shard_n]
    solved = set(env.success_uids)
    recs = []
    for f in files:
        d = np.load(f, allow_pickle=True); S = d['states'].astype(np.float64); A = d['actions'].astype(np.float64)
        uid = int(d['uid']); n = len(S)
        ic = dict(uid=uid) if uid in solved else dict(uid=None, ic_uid=uid, can_pos=[float(v) for v in S[0, 8:11]],
                                                      can_quat=[float(v) for v in S[0, 11:15]], goal_pos=None)
        t0 = time.time()
        obs = rec._reset(ic)
        genv = env.genv
        q_after = np.zeros((n, 6)); can_xy = np.zeros((n, 2)); picked_frame = -1
        for j in range(n):
            a = A[j].astype(np.float32)
            o, _done, info = genv.step(a, grip_motor=float(A[j, 6]) * 100.0, arm_cmd=A[j, :6])
            st = np.asarray(o['state'], np.float64)
            q_after[j] = st[:6]; can_xy[j] = st[8:10]
            if picked_frame < 0 and info.get('picked'):
                picked_frame = j
        # the tape ends at the geometric lift; give the hardened pick its PICK_SUSTAIN frames
        if picked_frame < 0:
            for k in range(30):
                o, _done, info = genv.step(A[-1].astype(np.float32), grip_motor=float(A[-1, 6]) * 100.0, arm_cmd=A[-1, :6])
                if info.get('picked'):
                    picked_frame = n + k; break
        m = track_metrics(A[:, :6], q_after, np.clip(A[:, 6], 0, 1), can_xy, picked_frame)
        # also: sim-vs-TAPE (the collector's own sim) deviation, to see how far the variant moved the arm
        m['dev_vs_tape_inf_p95'] = round(float(np.percentile(np.abs(q_after[:-1] - S[1:, :6]).max(1), 95)), 4)
        m['uid'] = uid; m['seconds'] = round(time.time() - t0, 1); m['variant'] = args.variant
        recs.append(m)
        print(f"[simlab] uid={uid} n={n} picked_frame={picked_frame} inf_p50={m['inf_p50']} inf_p95={m['inf_p95']} "
              f"over_leash={m['frac_over_leash']} sag95={m['sag_p95']} err_close={max(m['err_at_close']):.3f} ({m['seconds']}s)", flush=True)
    out = REPO / OUT_ROOT / args.variant; out.mkdir(parents=True, exist_ok=True)
    (out / f'manifest_shard{args.shard_idx}of{args.shard_n}.json').write_text(
        json.dumps(dict(variant=args.variant, applied=applied, records=recs), indent=1))


def merge(variant):
    out = REPO / OUT_ROOT / variant
    recs = {}
    for m in sorted(out.glob('manifest_shard*.json')):
        for r in json.loads(m.read_text())['records']:
            recs[r['uid']] = r
    if not recs:
        return None
    recs = [recs[u] for u in sorted(recs)]
    s = summarize(recs); s['variant'] = variant
    (out / 'manifest.json').write_text(json.dumps(dict(summary=s, records=recs), indent=1))
    return s


def report():
    root = REPO / OUT_ROOT
    rows = []
    for d in sorted(root.iterdir()):
        if not d.is_dir(): continue
        if d.name == 'offline':
            recs = json.loads((d / 'manifest.json').read_text())['records']; s = summarize(recs); s['variant'] = 'offline'
        else:
            s = merge(d.name)
        if s: rows.append(s)
    print(f"{'variant':14s} {'n':>3s} {'inf50':>6s} {'inf95':>6s} {'>leash':>7s} {'>cap':>6s} {'picked':>6s} {'errclose':>8s}  rms/joint  |  sag95/joint  |  dyn95/joint")
    for s in rows:
        print(f"{s['variant']:14s} {s['n']:3d} {s['inf_p50_med']:6.3f} {s['inf_p95_med']:6.3f} {s['frac_over_leash_mean']:7.3f} {s['frac_over_cap_mean']:6.3f} "
              f"{s['picked']:6d} {s['err_at_close_inf_med']:8.3f}  {s['rms_mean']}  |  {s['sag_p95_med']}  |  {s['dyn_p95_med']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--variant', default='base', choices=sorted(sim_variants.VARIANTS))
    ap.add_argument('--src', default=SRC_DEFAULT)
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--shard-idx', type=int, default=0); ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--parallel', type=int, default=0)
    ap.add_argument('--offline', action='store_true'); ap.add_argument('--report', action='store_true'); ap.add_argument('--merge', action='store_true')
    args = ap.parse_args()
    if args.offline:
        recs = offline(args.src); print(json.dumps(summarize(recs), indent=1)); return
    if args.report:
        report(); return
    if args.merge:
        print(json.dumps(merge(args.variant), indent=1)); return
    if args.parallel and args.parallel > 1:
        out = REPO / OUT_ROOT / args.variant; out.mkdir(parents=True, exist_ok=True)
        for old in out.glob('manifest_shard*.json'): old.unlink()
        procs = []
        for i in range(args.parallel):
            cmd = [sys.executable, __file__, '--variant', args.variant, '--shard-idx', str(i), '--shard-n', str(args.parallel)]
            if args.uids: cmd += ['--uids'] + [str(u) for u in args.uids]
            log = open(out / f'shard{i}.log', 'w')
            procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT))
        for p in procs: p.wait()
        print(json.dumps(merge(args.variant), indent=1)); return
    run_shard(args)


if __name__ == '__main__':
    main()
