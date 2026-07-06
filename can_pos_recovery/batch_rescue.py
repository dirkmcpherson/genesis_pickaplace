"""Batched-GPU rescue search for trials the sequential CPU search couldn't solve.

Per trial (B=32 envs, one scene, episode replayed at most 3x):
  A. 32 can-start candidates around the FK seed, goal can at STATIC   -> direct wins
  B. same candidates, goal parked far                                  -> slide rest points
  C. best on-shelf candidates x goal placements derived from their rest points -> wins
Winners are recorded with the in-batch (distance-proxy) outcome; the exact contact-test
confirmation happens later via merge_and_validate.py validate (CPU, single env).

Usage: batch_rescue.py --uids ... --out rescue.json
"""
import argparse, json, time, pathlib as pl
import numpy as np
from batch_harness import build_batched_world, rollout_batch, REPO, STATIC_BOTTLE_POSITION
from replay_harness import load_episode

BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}
B = 32


def can_grid(seed, bucket, close=None, wide=False):
    """Candidates around the FK grasp seed AND the closure-start seed (bags show humans
    often drag the can several cm between first contact and lift -- the can must start
    where the fingers first close). wide=True trades density for reach (+-6cm) for
    trials where both seeds appear to be off."""
    pts = [seed, bucket]
    if close is not None:
        pts.insert(1, close)
    anchors = [seed] if close is None or np.hypot(close[0]-seed[0], close[1]-seed[1]) < 0.015 \
        else [close, seed]   # close-seed first: it's the physically correct start
    steps = (-0.02, 0.0, 0.02) if wide else (-0.01, 0.0, 0.01)
    ring = ((-0.04, 0), (0.04, 0), (0, -0.04), (0, 0.04), (-0.06, 0), (0.06, 0),
            (0, -0.06), (0, 0.06), (-0.04, -0.04), (0.04, 0.04), (-0.04, 0.04),
            (0.04, -0.04)) if wide else ((-0.02, 0), (0.02, 0), (0, -0.02), (0, 0.02))
    for ax, ay in anchors:
        for dx in steps:
            for dy in steps:
                if (dx, dy) != (0.0, 0.0):
                    pts.append((ax + dx, ay + dy))
        for dx, dy in ring:
            pts.append((ax + dx, ay + dy))
    seen = set(); out = []
    for p in pts:
        k = (round(p[0], 3), round(p[1], 3))
        if k not in seen:
            seen.add(k); out.append(p)
    out = out[:B]
    while len(out) < B:
        out.append(out[0])
    return np.array(out)


def summarize(r, b):
    return dict(picked=int(r['picked_at'][b]), placed=int(r['placed_at'][b]),
                success=int(r['success_at'][b]), max_z=float(r['max_z'][b]),
                final=[float(v) for v in r['final'][b]], on_shelf=bool(r['on_shelf_end'][b]),
                nested=bool(r['nested'][b]))


def pick_winner(r, wins):
    """Prefer envs that end NESTED (upright, touching, at rest); tie-break by latest
    contact-success (most end-of-demo-like)."""
    nested = [b for b in wins if r['nested'][b]]
    pool = nested if nested else wins
    return max(pool, key=lambda i: r['success_at'][i]), len(nested)


def lying_grid(close, seed):
    """Fallen-can candidates: positions near the closure-start seed x 8 axis yaws.
    quat = rotate pi/2 about horizontal axis (cos t, sin t, 0) -> cylinder lies on side."""
    anchor = close or seed
    pos = [anchor, seed, (anchor[0] - 0.015, anchor[1]), (anchor[0] + 0.015, anchor[1])]
    s = np.sin(np.pi / 4); c = np.cos(np.pi / 4)
    quats = [(c, s * np.cos(t), s * np.sin(t), 0.0)
             for t in np.linspace(0, np.pi, 8, endpoint=False)]
    cans, qs = [], []
    for p in pos:
        for q in quats:
            cans.append(p); qs.append(q)
    return np.array(cans[:B]), np.array(qs[:B])


def rescue_trial(w, uid, fk, log):
    vel, gp = load_episode(uid)
    seed = tuple(fk['can_xy']) if fk['conf'] in ('HIGH', 'MED') else BUCKET[fk['pos']]
    close = tuple(fk['close_xy']) if fk.get('close_xy') else None
    static = np.tile([STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1]], (B, 1))

    if LYING:
        cans, quats = lying_grid(close, seed)
        lz = np.full(B, 0.05 + 0.033 + 0.0125)   # table + radius + settle drop
        rA = rollout_batch(w, vel, gp, cans, static, can_quat=quats, can_z=lz)
        winsA = [b for b in range(B) if rA['success_at'][b] >= 0]
        picked = [b for b in range(B) if rA['picked_at'][b] >= 0]
        log(f"  LYING A: {len(winsA)} wins, {len(picked)} picked")
        if winsA:
            b, n_nested = pick_winner(rA, winsA)
            return dict(status='ok_batch', stage='A-lying', can_pos=[round(float(cans[b][0]), 4),
                        round(float(cans[b][1]), 4), float(lz[b])],
                        can_quat=[round(float(v), 4) for v in quats[b]],
                        goal_pos=[STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1],
                                  w['goal_start_z']],
                        goal_moved=False, n_batch_wins=len(winsA), batch=summarize(rA, b))
        # fall through to stage B/C with the best lying candidates
        shelf = [b for b in range(B) if rA['on_shelf_end'][b]]
        if picked and not shelf:
            return dict(status='lying_picked_no_shelf', picked_envs=len(picked),
                        best_max_z=float(rA['max_z'].max()))
        if not picked:
            return dict(status='lying_no_pick', best_max_z=float(rA['max_z'].max()))
        rB = rA  # reuse: goal was static, but rest points still informative
        pairs = []
        for b in sorted(shelf, key=lambda i: -rA['max_z'][i])[:8]:
            fx, fy = float(rA['final'][b][0]), float(rA['final'][b][1])
            for gx, gy in ((fx, fy), (fx + 0.03, fy)):
                pairs.append((b, (gx, gy)))
        pairs = pairs[:B]
        canC = np.array([cans[b] for b, _ in pairs])
        qC = np.array([quats[b] for b, _ in pairs])
        zC = np.array([lz[b] for b, _ in pairs])
        goalC = np.array([g for _, g in pairs])
        pad = B - len(pairs)
        if pad:
            canC = np.vstack([canC, np.tile(canC[-1], (pad, 1))])
            qC = np.vstack([qC, np.tile(qC[-1], (pad, 1))])
            zC = np.concatenate([zC, np.tile(zC[-1], pad)])
            goalC = np.vstack([goalC, np.tile(goalC[-1], (pad, 1))])
        rC = rollout_batch(w, vel, gp, canC, goalC, can_quat=qC, can_z=zC)
        winsC = [b for b in range(len(pairs)) if rC['success_at'][b] >= 0]
        log(f"  LYING C: {len(winsC)}/{len(pairs)} pairs succeed")
        if winsC:
            b, n_nested = pick_winner(rC, winsC)
            return dict(status='ok_batch', stage='C-lying',
                        can_pos=[round(float(canC[b][0]), 4), round(float(canC[b][1]), 4),
                                 float(zC[b])],
                        can_quat=[round(float(v), 4) for v in qC[b]],
                        goal_pos=[round(float(goalC[b][0]), 4), round(float(goalC[b][1]), 4),
                                  w['goal_start_z']],
                        goal_moved=True, n_batch_wins=len(winsC), batch=summarize(rC, b))
        return dict(status='lying_shelf_no_slide')

    cans = can_grid(seed, BUCKET[fk['pos']], close, wide=WIDE)

    # stage A: static goal
    rA = rollout_batch(w, vel, gp, cans, static)
    winsA = [b for b in range(B) if rA['success_at'][b] >= 0]
    if winsA:
        b, n_nested = pick_winner(rA, winsA)
        log(f"  A: {len(winsA)} direct wins ({n_nested} nested); "
            f"picking can=({cans[b][0]:+.3f},{cans[b][1]:+.3f})")
        return dict(status='ok_batch', stage='A', can_pos=[round(float(cans[b][0]), 4),
                    round(float(cans[b][1]), 4), w['can_start_z']],
                    goal_pos=[STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1],
                              w['goal_start_z']],
                    goal_moved=False, n_batch_wins=len(winsA), batch=summarize(rA, b))

    # stage B: goal parked far -> rest points
    farg = np.full((B, 2), np.nan)
    rB = rollout_batch(w, vel, gp, cans, farg)
    shelf = [b for b in range(B) if rB['on_shelf_end'][b]]
    log(f"  A: 0 wins. B: {len(shelf)}/{B} candidates rest on shelf")
    if not shelf:
        picked_any = int((rB['picked_at'] >= 0).sum())
        return dict(status='no_shelf_batch', picked_envs=int(picked_any),
                    best_max_z=float(rB['max_z'].max()))

    # stage C: pair on-shelf candidates with goals derived from their rest points
    shelf.sort(key=lambda b: -rB['max_z'][b])
    pairs = []
    for b in shelf[:8]:
        fx, fy = float(rB['final'][b][0]), float(rB['final'][b][1])
        d = np.array([fx - fk['release_xy'][0], fy - fk['release_xy'][1]]) if fk.get('release_xy') else np.array([1.0, 0.0])
        nrm = np.linalg.norm(d); d = d / nrm if nrm > 1e-3 else np.array([1.0, 0.0])
        for gx, gy in ((fx, fy), (fx - 0.04 * d[0], fy - 0.04 * d[1]),
                       (fx + 0.03 * d[0], fy + 0.03 * d[1]), (fx + 0.03, fy)):
            pairs.append((cans[b], (gx, gy)))
    pairs = pairs[:B]
    canC = np.array([p[0] for p in pairs])
    goalC = np.array([p[1] for p in pairs])
    if len(pairs) < B:
        canC = np.vstack([canC, np.tile(canC[-1], (B - len(pairs), 1))])
        goalC = np.vstack([goalC, np.tile(goalC[-1], (B - len(pairs), 1))])
    rC = rollout_batch(w, vel, gp, canC, goalC)
    winsC = [b for b in range(len(pairs)) if rC['success_at'][b] >= 0]
    log(f"  C: {len(winsC)}/{len(pairs)} pairs succeed")
    if winsC:
        b, n_nested = pick_winner(rC, winsC)
        log(f"  C: {n_nested} nested; picking pair {b}")
        return dict(status='ok_batch', stage='C',
                    can_pos=[round(float(canC[b][0]), 4), round(float(canC[b][1]), 4),
                             w['can_start_z']],
                    goal_pos=[round(float(goalC[b][0]), 4), round(float(goalC[b][1]), 4),
                              w['goal_start_z']],
                    goal_moved=True, n_batch_wins=len(winsC), batch=summarize(rC, b))
    return dict(status='shelf_but_no_slide',
                rest_points=[[round(float(v), 4) for v in rB['final'][b][:2]] for b in shelf[:8]])


WIDE = False
LYING = False

def main():
    global WIDE, LYING
    ap = argparse.ArgumentParser()
    ap.add_argument('--uids', type=int, nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--substeps', type=int, default=4)
    ap.add_argument('--wide', action='store_true')
    ap.add_argument('--lying', action='store_true',
                    help='fallen-can candidate grid (positions x axis yaws)')
    args = ap.parse_args()
    WIDE = args.wide
    LYING = args.lying
    fk_all = {int(k): v for k, v in
              json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
    outp = pl.Path(args.out)
    results = json.loads(outp.read_text()) if outp.exists() else {}
    # world v2: table under the pick area + true soup-can dims (see bag/tool_pose evidence)
    w = build_batched_world(B, can_height=0.101, can_radius=0.033, can_rho=1000,
                            table=True, substeps=args.substeps)
    for uid in args.uids:
        if str(uid) in results:
            print(f"[{uid}] done, skip"); continue
        fk = fk_all[uid]
        t0 = time.time()
        print(f"[{uid}] conf={fk['conf']} seed={fk['can_xy']}", flush=True)
        res = rescue_trial(w, uid, fk, lambda m: print(m, flush=True))
        res.update(label=fk['label'], conf=fk['conf'], pos=fk['pos'],
                   elapsed_s=round(time.time() - t0, 1))
        results[str(uid)] = res
        outp.write_text(json.dumps(results, indent=1))
        print(f"[{uid}] -> {res['status']} ({res['elapsed_s']}s)", flush=True)
    from collections import Counter
    print(Counter(v['status'] for v in results.values()))


if __name__ == '__main__':
    main()
