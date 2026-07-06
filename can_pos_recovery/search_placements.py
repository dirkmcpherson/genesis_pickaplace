"""Per-trial placement search: find a can start position (and, when needed, a goal-can
position) that makes example.py-style open-loop replay succeed.

Stages per trial:
  1. PICK   - seed the can at the FK-recovered grasp point (bucket if LOW confidence);
              if the truncated pick-gate rollout doesn't lift the can, spiral-search a
              small xy grid around the seed.
  2. SLIDE  - full rollout with the default goal can (STATIC). Success -> verify.
  3. GOAL   - if the can gets placed on the shelf but never hits the goal can, replay
              once with the goal parked far away, observe where the can comes to rest,
              and put the goal can there (and one variant nudged along the slide dir).
  4. VERIFY - repeat the winning config; report success rate (stochastic contacts).

Writes/updates --out incrementally after every trial (crash-safe, resumable).

Usage:
  search_placements.py --uids 232 235 ... --out placements_shard0.json
                       [--finger-force F] [--backend gpu|cpu] [--verify N]
"""
import argparse, json, time, pathlib as pl
import numpy as np
from replay_harness import (build_world, load_episode, rollout,
                            STATIC_BOTTLE_POSITION, SHELF_REST_Z, REPO)

BUCKET = {0: (0.4381, 0.1, 0.05), 1: (0.4381, -0.05, 0.05), 2: (0.4381, -0.2, 0.05)}

# spiral of xy offsets tried around the seed when the seed itself doesn't pick
PICK_OFFSETS = [(0, 0), (-0.01, 0), (0.01, 0), (0, -0.01), (0, 0.01),
                (-0.02, 0), (0.02, 0), (0, -0.02), (0, 0.02),
                (-0.02, -0.02), (-0.02, 0.02), (0.02, -0.02), (0.02, 0.02),
                (-0.03, 0), (0.03, 0), (0, -0.03), (0, 0.03)]
MAX_PICK_WINNERS = 2     # how many distinct picking positions to try through stages 2-3


def search_trial(w, uid, fk, verify_n, log, max_winners=MAX_PICK_WINNERS):
    CAN_Z = w['can_start_z']
    GOAL_Z = w['goal_start_z']
    STATIC_GOAL = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], GOAL_Z)
    vel, gp = load_episode(uid)
    n_steps = fk['n_steps']
    pick_gate = min(n_steps, fk['grasp_idx'] + 150)
    if fk['conf'] in ('HIGH', 'MED'):
        seed = (fk['can_xy'][0], fk['can_xy'][1])
    else:
        b = fk['bucket'] or BUCKET[0]
        seed = (b[0], b[1])
    # anchor the spiral on the closure-start seed too (drag demos: the can must start
    # where the fingers first close, not at the FK grasp point mid-drag)
    anchors = [seed]
    if fk.get('close_xy'):
        c = (fk['close_xy'][0], fk['close_xy'][1])
        if np.hypot(c[0] - seed[0], c[1] - seed[1]) > 0.015:
            anchors.insert(0, c)

    tried = []
    def run(can_xy, goal_pos, max_cmds=None, tag=''):
        r = rollout(w, vel, gp, (can_xy[0], can_xy[1], CAN_Z), goal_pos, max_cmds=max_cmds)
        tried.append({'can': [round(can_xy[0], 4), round(can_xy[1], 4)],
                      'goal': None if goal_pos is None else [round(v, 4) for v in goal_pos],
                      'max_cmds': max_cmds, 'tag': tag,
                      'picked': r['picked'], 'placed': r['placed'], 'success': r['success']})
        return r

    # ---- stage 1: find positions that pick ----
    pick_winners = []
    for ax, ay in anchors:
        for dx, dy in PICK_OFFSETS:
            xy = (ax + dx, ay + dy)
            r = run(xy, STATIC_GOAL, max_cmds=pick_gate, tag="pick-gate")
            log(f"  pick-gate can=({xy[0]:+.3f},{xy[1]:+.3f}) picked={r['picked']} max_z={r['max_z']}")
            if r['picked']:
                pick_winners.append(xy)
                if len(pick_winners) >= max_winners:
                    break
        if len(pick_winners) >= max_winners:
            break
    if not pick_winners:
        return {'status': 'no_pick', 'tried': tried}

    # ---- stages 2-3: for each picking position, find a goal placement that succeeds ----
    for xy in pick_winners:
        r = run(xy, STATIC_GOAL, tag="full-static")
        log(f"  full/static can=({xy[0]:+.3f},{xy[1]:+.3f}) picked={r['picked']} "
            f"placed={r['placed']} success={r['success']} final={r['final_can']}")
        best = None
        if r['success']:
            best = {"can": xy, "goal": STATIC_GOAL}
        elif r['picked']:
            # observe unobstructed slide: where does the can come to rest?
            pr = run(xy, None, tag='goal-probe')
            log(f"  goal-probe final={pr['final_can']} on_shelf={pr['on_shelf_end']}")
            if pr['on_shelf_end']:
                fx, fy = pr['final_can'][0], pr['final_can'][1]
                goal_cands = [(fx, fy, GOAL_Z)]
                # variants along the slide direction (FK release point -> rest point):
                # +0.03 = just past the rest, -0.04 = early block so contact happens
                # while the eef is still behind the can
                if fk.get('release_xy'):
                    d = np.array([fx - fk['release_xy'][0], fy - fk['release_xy'][1]])
                    nrm = float(np.linalg.norm(d))
                    if nrm > 1e-3:
                        d = d / nrm
                        goal_cands.append((fx + 0.03 * float(d[0]), fy + 0.03 * float(d[1]), GOAL_Z))
                        goal_cands.append((fx - 0.04 * float(d[0]), fy - 0.04 * float(d[1]), GOAL_Z))
                goal_cands.append((fx + 0.03, fy, GOAL_Z))   # default nudge: away from robot
                for gc in goal_cands:
                    r2 = run(xy, gc, tag='full-movedgoal')
                    log(f"  full/goal=({gc[0]:+.3f},{gc[1]:+.3f}) success={r2['success']}")
                    if r2['success']:
                        best = {'can': xy, 'goal': gc}
                        break
        if best is None:
            continue
        # ---- stage 4: verify ----
        succ = 1; picked = 1  # the run that found it counts
        for _ in range(verify_n - 1):
            rv = run(best['can'], best['goal'], tag='verify')
            succ += int(rv['success']); picked += int(rv['picked'])
        log(f"  VERIFY can=({best['can'][0]:+.3f},{best['can'][1]:+.3f}) "
            f"goal=({best['goal'][0]:+.3f},{best['goal'][1]:+.3f}) success {succ}/{verify_n}")
        return {'status': 'ok',
                'can_pos': [round(best['can'][0], 4), round(best['can'][1], 4), CAN_Z],
                'goal_pos': [round(v, 4) for v in best['goal']],
                'goal_moved': list(best["goal"]) != list(STATIC_GOAL),
                'success_rate': succ / verify_n, 'picked_rate': picked / verify_n,
                'verify_n': verify_n, 'tried': tried}
    return {'status': 'pick_but_no_slide', 'pick_winners': [list(x) for x in pick_winners],
            'tried': tried}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uids', type=int, nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--finger-force', type=float, default=None)
    ap.add_argument('--finger-kp', type=float, default=None)
    ap.add_argument('--can-height', type=float, default=0.075)
    ap.add_argument('--can-radius', type=float, default=0.035)
    ap.add_argument('--can-rho', type=float, default=2000)
    ap.add_argument('--table', action='store_true')
    ap.add_argument('--substeps', type=int, default=1)
    ap.add_argument('--backend', default='gpu')
    ap.add_argument('--verify', type=int, default=3)
    ap.add_argument('--max-winners', type=int, default=MAX_PICK_WINNERS)
    args = ap.parse_args()

    fk_all = {int(k): v for k, v in
              json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
    outp = pl.Path(args.out)
    results = json.loads(outp.read_text()) if outp.exists() else {}

    w = build_world(backend=args.backend, finger_force=args.finger_force,
                    finger_kp=args.finger_kp, can_height=args.can_height, can_rho=args.can_rho,
                    substeps=args.substeps, table=args.table, can_radius=args.can_radius)
    for uid in args.uids:
        if str(uid) in results:
            print(f"[{uid}] already done, skipping"); continue
        fk = fk_all[uid]
        if fk['conf'] == 'LOW' and fk['label'] != 'success':
            results[str(uid)] = {'status': 'skipped_low_conf_fail'}
            outp.write_text(json.dumps(results, indent=1)); continue
        t0 = time.time()
        print(f"[{uid}] label={fk['label']} conf={fk['conf']} seed={fk['can_xy']}")
        log = lambda m: print(m, flush=True)
        res = search_trial(w, uid, fk, args.verify, log, max_winners=args.max_winners)
        res['label'] = fk['label']; res['conf'] = fk['conf']; res['pos'] = fk['pos']
        res['elapsed_s'] = round(time.time() - t0, 1)
        results[str(uid)] = res
        outp.write_text(json.dumps(results, indent=1))
        print(f"[{uid}] -> {res['status']} ({res['elapsed_s']}s)", flush=True)

    done = [r for r in results.values() if r.get('status') == 'ok']
    print(f"\n{len(done)}/{len(results)} trials have a working placement")


if __name__ == '__main__':
    main()
