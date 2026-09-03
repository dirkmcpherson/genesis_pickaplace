#!/usr/bin/env python
"""Full pick-and-place validation of a simulator variant: replay the human's REAL joint +
gripper command streams (inthewild_trials/<uid>_episodes.npy, the same streams
can_pos_recovery/remeasure_contact.py replays) through the world (replay_harness.build_world
+ per-trial placements, goal at the human-validated south position) and score every
downstream stage with the repo's own predicates, plus tip-overs, gripper contact and the
whole-episode tracking error vs the real measured joints.

Predicates (identical to remeasure_contact.py / replay_harness.rollout / full_env where noted):
  picked       can z > pick_z while the recorded gripper is closed (gp > GP_CLOSE=30)  [replay]
  picked_hard  genesis_can_env hardened: + |eef-can| < 0.20 sustained 10 frames        [env]
  placed       remeasure: max z > pick_z and final can z > 0.11                          [replay]
  placed_v2    full_env: grip cmd open (<0.3) & in shelf footprint & BOX_TOP_Z+0.01<z<+0.07
               & tilt < 20 deg, sustained 10 frames                                       [env]
  contact      picked & bottle-goal contacts & eef x < can x (every 30 frames)           [replay]
  nested       settled 100 steps: picked & xy-dist(can, goal) <= NESTED_TOUCH_DIST & both upright [remeasure]
  tipped       max can tilt > 60 deg at any frame; tipped_free = tilt > 60 while grip open (tip rule)
Tracking: e_j = cmd_j - q_sim after frame j, per phase (pre-close / closed / post-release).
Gripper: max finger-can penetration + max contact force per phase (closed = carry, post = release).

USAGE
  python baselines/fulltask_fidelity_lab.py --variant base --parallel 3
  python baselines/fulltask_fidelity_lab.py --variant gc_kp4_riser3 --parallel 3
  python baselines/fulltask_fidelity_lab.py --report
Outputs: baselines/demos_v1/_fulltask/<variant>/manifest*.json
"""
import argparse, glob, json, os, pathlib as pl, subprocess, sys, time
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'can_pos_recovery'))
import sim_variants  # noqa: E402

OUT_ROOT = 'baselines/demos_v1/_fulltask'
GOAL = (0.672, -0.221)          # human-validated south goal (replay_harness.STATIC_BOTTLE_POSITION)
GP_CLOSE = 30.0; PICK_EEF_DIST = 0.20; PICK_SUSTAIN = 10; TIP_DEG = 60.0
LEASH = 0.125; CAP = 0.025
BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}


def _np(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def tilt_deg(q):
    w_, x, y, z = [float(v) for v in q]
    zz = 1 - 2 * (x * x + y * y); zx = 2 * (x * z + w_ * y); zy = 2 * (y * z - w_ * x)
    n = (zx * zx + zy * zy + zz * zz) ** 0.5 + 1e-9
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, zz / n)))))


def run_shard(args):
    sim_variants.install(args.variant)
    import torch  # noqa
    from replay_harness import (build_world, load_episode, gripper_targets, HARDCODED_START,
                                in_shelf_footprint, BOX_TOP_Z, NESTED_TOUCH_DIST)
    tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
    wcfg = tbl['world']; trials = tbl['trials']
    fk = {int(k): v for k, v in json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
    t0 = time.time()
    w = build_world(backend='cpu', finger_force=wcfg['finger_force'], finger_kp=wcfg['finger_kp'],
                    can_height=wcfg['can_height'], can_rho=wcfg['can_rho'], substeps=wcfg.get('substeps', 1),
                    table=True, can_radius=wcfg.get('can_radius', 0.035))
    applied = sim_variants.post_build(w, args.variant)
    print(f'[fulltask] world built {time.time()-t0:.0f}s variant={applied}', flush=True)
    scene, kin, bottle, goal, kdofs, eef = (w['scene'], w['kinova'], w['bottle'], w['goal'], w['kdofs'], w['eef'])
    CANZ, GOALZ, PICKZ = w['can_start_z'], w['goal_start_z'], w['pick_z']   # goal_start_z already shifted by post_build for shelf variants
    SHELF_TOP = sim_variants.shelf_top(args.variant)
    print(f'[fulltask] shelf top {SHELF_TOP} goal z {GOALZ} pick_z {PICKZ}', flush=True)
    sol = scene.sim.rigid_solver; col = sol.collider
    b_geoms = set(range(bottle.geom_start, bottle.geom_end))
    finger_geoms = set()
    for l in kin.links:
        if 'finger' in l.name:
            for g in l.geoms: finger_geoms.add(g.idx)
    uids = sorted(int(u) for u, r in trials.items() if r.get('label') == args.label and u not in ('290', '322'))
    if args.uids: uids = [u for u in uids if u in set(args.uids)]
    uids = uids[args.shard_idx::args.shard_n]
    recs = []
    for uid in uids:
        r = trials[str(uid)]
        solved = r['status'] in ('ok', 'ok_batch')
        can_quat = list(r.get('can_quat') or [1, 0, 0, 0]) if solved else [1, 0, 0, 0]
        if solved:
            can_pos = tuple(r['can_pos'])
        else:
            f = fk.get(uid, {}); seed = f.get('close_xy') or (f.get('can_xy') if f.get('conf') in ('HIGH', 'MED') else BUCKET[f.get('pos')])
            can_pos = (seed[0], seed[1], CANZ)
        vel, gp = load_episode(uid)
        te = time.time()
        kin.set_dofs_position(np.array(HARDCODED_START), kdofs); kin.zero_all_dofs_velocity()
        bottle.set_pos(can_pos); bottle.set_quat(can_quat)
        goal.set_pos((GOAL[0], GOAL[1], GOALZ)); goal.set_quat([1, 0, 0, 0])
        for e in (bottle, goal):
            try: e.zero_all_dofs_velocity()
            except Exception: pass
        scene.step()
        n = len(vel)
        picked = contact = False; picked_at = contact_at = -1; hard_at = -1; run = 0
        pv2_run = 0; placed_v2_at = -1
        maxz = 0.0; max_tilt = 0.0; max_tilt_free = 0.0; tip_free_at = -1
        err = np.zeros((n, 6)); phase = np.zeros(n, np.int8)   # 0 pre-close, 1 closed, 2 post-release
        pen = np.zeros(n); force = np.zeros(n)
        closed_seen = False
        can_first_move = -1; can0 = np.asarray(can_pos[:2], float)
        for i in range(n):
            frac = sim_variants.grip_frac(args.variant, float(np.clip(gp[i] / 100.0, 0.0, 1.0)))
            kin.control_dofs_position(np.asarray(vel[i], np.float64), dofs_idx_local=kdofs[:6])
            kin.control_dofs_position(np.array(gripper_targets(frac * 100.0)), dofs_idx_local=np.array(kdofs[-4:]))
            for _ in range(3): scene.step()
            q = _np(kin.get_dofs_position(dofs_idx_local=kdofs[:6]))
            err[i] = np.asarray(vel[i], float) - q
            bp = _np(bottle.get_pos()); bq = _np(bottle.get_quat()); ee = _np(eef.get_pos())
            closed = gp[i] > GP_CLOSE
            if closed: closed_seen = True
            phase[i] = 1 if closed else (2 if closed_seen else 0)
            maxz = max(maxz, float(bp[2]))
            tl = tilt_deg(bq); max_tilt = max(max_tilt, tl)
            if not closed:
                max_tilt_free = max(max_tilt_free, tl)
                if tl > TIP_DEG and tip_free_at < 0: tip_free_at = i
            if can_first_move < 0 and np.linalg.norm(bp[:2] - can0) > 0.005: can_first_move = i
            if not picked and bp[2] > PICKZ and closed: picked = True; picked_at = i
            held = bp[2] > PICKZ and closed and float(np.linalg.norm(ee - bp)) < PICK_EEF_DIST
            run = run + 1 if held else 0
            if hard_at < 0 and run >= PICK_SUSTAIN: hard_at = i
            okv2 = (gp[i] / 100.0 < 0.3) and in_shelf_footprint(bp) and (SHELF_TOP + 0.01 < bp[2] < SHELF_TOP + 0.07) and tl < 20
            pv2_run = pv2_run + 1 if okv2 else 0
            if placed_v2_at < 0 and picked and pv2_run >= 10: placed_v2_at = i
            if not contact and i % 30 == 0 and picked:
                c = _np(bottle.get_contacts(goal)['position'])
                if (c.size and c.shape[0]) and float(ee[0]) < float(bp[0]): contact = True; contact_at = i
            # finger-can contact
            nc = int(np.asarray(col.n_contacts.to_numpy()).reshape(-1)[0])
            if nc:
                cd = col.contact_data.to_numpy()
                GA = np.asarray(cd['geom_a']).reshape(-1)[:nc]; GB = np.asarray(cd['geom_b']).reshape(-1)[:nc]
                m = np.array([(a in b_geoms and b in finger_geoms) or (b in b_geoms and a in finger_geoms) for a, b in zip(GA, GB)])
                if m.any():
                    pen[i] = float(np.asarray(cd['penetration']).reshape(-1)[:nc][m].max())
                    force[i] = float(np.linalg.norm(np.asarray(cd['force']).reshape(-1, 3)[:nc][m], axis=1).max())
        for _ in range(100): scene.step()
        bp = _np(bottle.get_pos()); gpos = _np(goal.get_pos())
        touch = float(np.hypot(bp[0] - gpos[0], bp[1] - gpos[1])) <= NESTED_TOUCH_DIST
        bt = tilt_deg(_np(bottle.get_quat())); gt = tilt_deg(_np(goal.get_quat()))
        nested = bool(picked and touch and bt < 20 and gt < 20)
        placed = bool(maxz > PICKZ and float(bp[2]) > SHELF_TOP)   # remeasure's 'final z > 0.11' moved with the shelf
        ae = np.abs(err); inf = ae.max(1)
        def ph(k, arr, fn):
            sel = arr[phase == k]; return (None if sel.size == 0 else round(float(fn(sel)), 4))
        rec = dict(uid=uid, label=args.label, status=r['status'], n=int(n), seconds=round(time.time() - te, 1),
                   picked=bool(picked), picked_at=int(picked_at), picked_hard_at=int(hard_at),
                   placed=placed, placed_v2_at=int(placed_v2_at), contact=bool(contact), contact_at=int(contact_at),
                   nested=nested, final_can=[round(float(v), 4) for v in bp], final_tilt=round(bt, 1), goal_tilt=round(gt, 1),
                   max_tilt=round(max_tilt, 1), tipped=bool(max_tilt > TIP_DEG), tipped_free=bool(tip_free_at >= 0), tip_free_at=int(tip_free_at),
                   can_first_move=int(can_first_move),
                   err_inf_p50=round(float(np.median(inf)), 4), err_inf_p95=round(float(np.percentile(inf, 95)), 4), err_inf_max=round(float(inf.max()), 4),
                   err_over_leash=round(float((inf > LEASH).mean()), 4), err_over_cap=round(float((inf > CAP).mean()), 4),
                   err_rms=np.sqrt((err ** 2).mean(0)).round(4).tolist(),
                   err_p95_pre=ph(0, inf, lambda a: np.percentile(a, 95)), err_p95_closed=ph(1, inf, lambda a: np.percentile(a, 95)),
                   err_p95_post=ph(2, inf, lambda a: np.percentile(a, 95)),
                   pen_max_closed=ph(1, pen, np.max), pen_p95_closed=ph(1, pen, lambda a: np.percentile(a, 95)),
                   force_max_closed=ph(1, force, np.max), force_p95_closed=ph(1, force, lambda a: np.percentile(a, 95)),
                   pen_max_post=ph(2, pen, np.max), force_max_post=ph(2, force, np.max),
                   pen_max_pre=ph(0, pen, np.max), force_max_pre=ph(0, force, np.max),
                   frames_closed=int((phase == 1).sum()), frames_post=int((phase == 2).sum()))
        recs.append(rec)
        print(f"[fulltask] {uid}: picked={picked}(hard@{hard_at}) placed={placed} v2@{placed_v2_at} contact={contact} nested={nested} "
              f"tilt_max={max_tilt:.0f} tipfree={tip_free_at} err50={rec['err_inf_p50']} err95={rec['err_inf_p95']} >leash={rec['err_over_leash']} "
              f"pen_closed={rec['pen_max_closed']} F_closed={rec['force_max_closed']} ({rec['seconds']}s)", flush=True)
    out = REPO / OUT_ROOT / args.variant; out.mkdir(parents=True, exist_ok=True)
    (out / f"manifest_{args.label}_shard{args.shard_idx}of{args.shard_n}.json").write_text(
        json.dumps(dict(variant=args.variant, applied=applied, label=args.label, goal=GOAL, records=recs), indent=1))


def merge(variant, label='success'):
    out = REPO / OUT_ROOT / variant; recs = {}
    for m in sorted(out.glob(f'manifest_{label}_shard*.json')):
        for r in json.loads(m.read_text())['records']: recs[r['uid']] = r
    if not recs: return None
    recs = [recs[u] for u in sorted(recs)]
    (out / f'manifest_{label}.json').write_text(json.dumps(dict(variant=variant, label=label, records=recs), indent=1))
    return recs


def summarize(recs):
    n = len(recs); A = lambda k: np.array([r[k] for r in recs if r.get(k) is not None], float)
    return dict(n=n, picked=sum(r['picked'] for r in recs), picked_hard=sum(r['picked_hard_at'] >= 0 for r in recs),
                placed=sum(r['placed'] for r in recs), placed_v2=sum(r['placed_v2_at'] >= 0 for r in recs),
                contact=sum(r['contact'] for r in recs), nested=sum(r['nested'] for r in recs),
                tipped=sum(r['tipped'] for r in recs), tipped_free=sum(r['tipped_free'] for r in recs),
                err50=round(float(np.median(A('err_inf_p50'))), 4), err95=round(float(np.median(A('err_inf_p95'))), 4),
                over_leash=round(float(A('err_over_leash').mean()), 4), over_cap=round(float(A('err_over_cap').mean()), 4),
                e95_pre=round(float(np.median(A('err_p95_pre'))), 4) if len(A('err_p95_pre')) else None,
                e95_closed=round(float(np.median(A('err_p95_closed'))), 4) if len(A('err_p95_closed')) else None,
                e95_post=round(float(np.median(A('err_p95_post'))), 4) if len(A('err_p95_post')) else None,
                pen_closed_med=round(float(np.median(A('pen_max_closed'))), 4) if len(A('pen_max_closed')) else None,
                pen_closed_max=round(float(A('pen_max_closed').max()), 4) if len(A('pen_max_closed')) else None,
                F_closed_med=round(float(np.median(A('force_max_closed'))), 1) if len(A('force_max_closed')) else None,
                pen_post_med=round(float(np.median(A('pen_max_post'))), 4) if len(A('pen_max_post')) else None,
                F_post_med=round(float(np.median(A('force_max_post'))), 1) if len(A('force_max_post')) else None)


def report():
    root = REPO / OUT_ROOT
    for label in ('success', 'fail'):
        rows = []
        for d in sorted(root.iterdir()):
            if d.is_dir():
                recs = merge(d.name, label)
                if recs: rows.append((d.name, summarize(recs)))
        if not rows: continue
        print(f'== label={label}')
        print(f"{'variant':22s} {'n':>3s} {'pick':>4s} {'hard':>4s} {'plcd':>4s} {'pv2':>4s} {'cont':>4s} {'nest':>4s} {'tip':>4s} {'tipF':>4s} {'e50':>6s} {'e95':>6s} {'>leash':>6s} {'e95pre':>6s} {'e95cls':>6s} {'e95pst':>6s} {'pen_c':>6s} {'F_c':>6s} {'pen_p':>6s} {'F_p':>6s}")
        for name, s in rows:
            print(f"{name:22s} {s['n']:3d} {s['picked']:4d} {s['picked_hard']:4d} {s['placed']:4d} {s['placed_v2']:4d} {s['contact']:4d} {s['nested']:4d} {s['tipped']:4d} {s['tipped_free']:4d} "
                  f"{s['err50']:6.3f} {s['err95']:6.3f} {s['over_leash']:6.3f} {s['e95_pre'] or 0:6.3f} {s['e95_closed'] or 0:6.3f} {s['e95_post'] or 0:6.3f} "
                  f"{s['pen_closed_med'] or 0:6.4f} {s['F_closed_med'] or 0:6.1f} {s['pen_post_med'] or 0:6.4f} {s['F_post_med'] or 0:6.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--variant', default='base', choices=sorted(sim_variants.VARIANTS))
    ap.add_argument('--label', default='success', choices=['success', 'fail'])
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    ap.add_argument('--shard-idx', type=int, default=0); ap.add_argument('--shard-n', type=int, default=1)
    ap.add_argument('--parallel', type=int, default=0)
    ap.add_argument('--report', action='store_true'); ap.add_argument('--merge', action='store_true')
    args = ap.parse_args()
    if args.report: report(); return
    if args.merge: print(json.dumps(summarize(merge(args.variant, args.label)), indent=1)); return
    if args.parallel and args.parallel > 1:
        out = REPO / OUT_ROOT / args.variant; out.mkdir(parents=True, exist_ok=True)
        for old in out.glob(f'manifest_{args.label}_shard*.json'): old.unlink()
        procs = []
        for i in range(args.parallel):
            cmd = [sys.executable, __file__, '--variant', args.variant, '--label', args.label, '--shard-idx', str(i), '--shard-n', str(args.parallel)]
            if args.uids: cmd += ['--uids'] + [str(u) for u in args.uids]
            log = open(out / f'{args.label}_shard{i}.log', 'w')
            procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT))
        for p in procs: p.wait()
        print(json.dumps(summarize(merge(args.variant, args.label)), indent=1)); return
    run_shard(args)


if __name__ == '__main__':
    main()
