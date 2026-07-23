"""Job #2 (new box): CPU-parallel re-search of residual no-pick demos under the CURRENT world.

Why CPU, not batch_rescue (GPU): the GPU batch search has a known GPU->CPU winner
transfer cliff AND builds a stale world (finger_kp=100 vs current 40). Here every
candidate is scored with the exact single-env CPU replay_harness.rollout under the
trial_placements world config + south goal -- a winner is CPU-verified ('ok') by
construction. 32 cores make this tractable (~25s/rollout, early-stop on success).

Subcommands:
  gen    --uids U... --out jobs.json [--wide] [--lying]     generate candidate jobs
  run    --jobs jobs.json --slice K N --out part_K.json     score one shard (worker)
  pick   --parts part_*.json --out rescue_cpu.json          pick winners per uid

Driver (16-way):
  python can_pos_recovery/cpu_research.py gen --uids ... --out J.json --lying
  for k in $(seq 0 15); do python ... run --jobs J.json --slice $k 16 --out P_$k.json & done; wait
  python ... pick --parts P_*.json --out rescue_cpu.json
"""
import os
import argparse, json, sys, time, pathlib as pl
import numpy as np

sys.path.insert(0, str(pl.Path(__file__).parent))
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))

BUCKET = {0: (0.4381, 0.1), 1: (0.4381, -0.05), 2: (0.4381, -0.2), None: (0.4381, -0.05)}


def can_grid(seed, bucket, close=None, wide=False, cap=32):
    """batch_rescue.can_grid, minus the B-padding (no need to pad on CPU)."""
    pts = [seed, bucket]
    if close is not None:
        pts.insert(1, close)
    anchors = [seed] if close is None or np.hypot(close[0]-seed[0], close[1]-seed[1]) < 0.015 \
        else [close, seed]
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
            seen.add(k); out.append((float(p[0]), float(p[1])))
    return out[:cap]


def lying_quats():
    s = np.sin(np.pi / 4); c = np.cos(np.pi / 4)
    return [(float(c), float(s * np.cos(t)), float(s * np.sin(t)), 0.0)
            for t in np.linspace(0, np.pi, 8, endpoint=False)]


def gen(args):
    fk_all = {int(k): v for k, v in
              json.loads((REPO / 'can_pos_recovery/fk_recovered.json').read_text()).items()}
    jobs = []
    for uid in args.uids:
        fk = fk_all[uid]
        seed = tuple(fk['can_xy']); close = fk.get('close_xy')
        close = tuple(close) if close else None
        bucket = BUCKET[fk.get('pos')]
        for xy in can_grid(seed, bucket, close, wide=args.wide):
            jobs.append(dict(uid=uid, xy=list(xy), quat=[1, 0, 0, 0], lying=False))
        if args.lying:
            anchors = [close or seed, seed,
                       ((close or seed)[0] - 0.015, (close or seed)[1]),
                       ((close or seed)[0] + 0.015, (close or seed)[1])]
            seen = set()
            for p in anchors:
                k = (round(p[0], 3), round(p[1], 3))
                if k in seen: continue
                seen.add(k)
                for q in lying_quats():
                    jobs.append(dict(uid=uid, xy=[float(p[0]), float(p[1])],
                                     quat=list(q), lying=True))
    pl.Path(args.out).write_text(json.dumps(jobs))
    from collections import Counter
    print(f"{len(jobs)} jobs -> {args.out}", Counter(j['uid'] for j in jobs))


def envpath_rollout(env, vel, gp, can_pos, can_quat, goal_pos):
    """Collector-faithful scoring: identical env.step calls (raw commands, per-step
    obs reads) + the collector's settle/proximity-nested scoring."""
    from replay_harness import tilt_deg, NESTED_TOUCH_DIST
    import torch
    def _np(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)
    obs = env.reset(can_pos=can_pos, can_quat=list(can_quat), goal_pos=goal_pos)
    picked = placed = contact = False
    for i in range(len(vel) - 1):
        a = np.concatenate([vel[i], [np.clip(gp[i] / 100.0, 0, 1)]]).astype(np.float32)
        obs, done, info = env.step(a, grip_motor=gp[i], arm_cmd=vel[i])
        picked |= bool(info['picked']); placed |= bool(info['placed'])
        contact |= bool(info['contact'])
    hold = np.concatenate([vel[-1], [np.clip(gp[-1] / 100.0, 0, 1)]]).astype(np.float32)
    for _ in range(100):
        obs, done, info = env.step(hold, grip_motor=gp[-1], arm_cmd=vel[-1])
    w = env.w
    bp_, gp_ = _np(w['bottle'].get_pos()), _np(w['goal'].get_pos())
    prox = float(np.hypot(bp_[0] - gp_[0], bp_[1] - gp_[1]))
    nested = bool(picked and prox <= NESTED_TOUCH_DIST
                  and tilt_deg(_np(w['bottle'].get_quat())) < 20
                  and tilt_deg(_np(w['goal'].get_quat())) < 20)
    return dict(picked=picked, placed=placed, success=contact, nested=nested,
                prox_nested=nested, final_can=[round(float(v), 4) for v in bp_],
                max_z=None, nested_detail={})


def run(args):
    jobs = json.loads(pl.Path(args.jobs).read_text())
    k, n = args.slice
    mine = [j for i, j in enumerate(jobs) if i % n == k]
    # group by uid so episode loads amortize; world built ONCE per process
    mine.sort(key=lambda j: j['uid'])
    from replay_harness import build_world, load_episode, rollout, STATIC_BOTTLE_POSITION, \
        NESTED_TOUCH_DIST, TABLE_TOP_Z
    tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
    wcfg = tbl['world']
    if args.env_path:
        sys.path.insert(0, str(REPO / 'baselines'))
        from genesis_can_env import GenesisCanEnv
        env = GenesisCanEnv(backend='cpu')
        env.max_steps = 10 ** 9   # eval horizon must NOT fire _nested mid-tape
        w = env.w
    else:
        w = build_world(backend='cpu', finger_force=wcfg['finger_force'],
                        finger_kp=wcfg['finger_kp'], can_height=wcfg['can_height'],
                        can_rho=wcfg['can_rho'], substeps=wcfg.get('substeps', 1),
                        table=True, can_radius=wcfg.get('can_radius', 0.035))
    goal_pos = (STATIC_BOTTLE_POSITION[0], STATIC_BOTTLE_POSITION[1], w['goal_start_z'])
    out = []
    cur_uid, vel, gp = None, None, None
    for j in mine:
        if j['uid'] != cur_uid:
            cur_uid = j['uid']; vel, gp = load_episode(cur_uid)
        z = (TABLE_TOP_Z + wcfg.get('can_radius', 0.035) + 0.002) if j['lying'] \
            else w['can_start_z']
        t0 = time.time()
        cp = (j['xy'][0], j['xy'][1], z)
        if args.env_path:
            r = envpath_rollout(env, vel, gp, cp, j['quat'], goal_pos)
            prox_nested = r['prox_nested']
        else:
            r = rollout(w, vel, gp, cp, goal_pos=goal_pos,
                        can_quat=tuple(j['quat']), stop_on_success=True)
            prox = float(np.hypot(r['final_can'][0] - goal_pos[0],
                                  r['final_can'][1] - goal_pos[1]))
            prox_nested = bool(r['picked'] and prox <= NESTED_TOUCH_DIST
                               and r['nested_detail']['bottle_tilt'] < 20
                               and r['nested_detail']['goal_tilt'] < 20)
        out.append(dict(**j, picked=bool(r['picked']), placed=bool(r['placed']),
                        success=bool(r['success']), nested=bool(r['nested']),
                        prox_nested=prox_nested,
                        final_can=r['final_can'], max_z=r['max_z'],
                        elapsed=round(time.time() - t0, 1)))
        pl.Path(args.out).write_text(json.dumps(out))   # checkpoint after every rollout
        print(f"[{k}] uid={j['uid']} xy=({j['xy'][0]:.3f},{j['xy'][1]:.3f}) "
              f"lying={j['lying']} picked={r['picked']} success={r['success']} "
              f"({out[-1]['elapsed']}s)", flush=True)
    print(f"[{k}] done: {len(out)} rollouts")


def pick(args):
    rows = []
    for p in args.parts:
        rows.extend(json.loads(pl.Path(p).read_text()))
    tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
    rank = lambda r: (r['prox_nested'], r['success'], r['placed'], r['picked'])
    winners = {}
    for r in rows:
        u = r['uid']
        if u not in winners or rank(r) > rank(winners[u]):
            winners[u] = r
    shard = {}
    for u, r in sorted(winners.items()):
        if not r['picked']:
            print(f"[{u}] best candidate still no-pick -- leaving unsolved")
            continue
        stage = 'nested' if r['prox_nested'] else ('contact' if r['success'] else
                ('placed' if r['placed'] else 'picked'))
        # z matches run(): can_start_z = floor_z + h/2 + 0.0125 = 0.113 (current world)
        z = (0.05 + 0.033 + 0.002) if r['lying'] else 0.113
        old = tbl['trials'].get(str(u), {})
        shard[str(u)] = dict(old, status='ok', label='success',
                             can_pos=[r['xy'][0], r['xy'][1], z], can_quat=r['quat'],
                             search='cpu_research_bb', stage=stage,
                             success_rate=None)
        print(f"[{u}] WINNER stage={stage} xy=({r['xy'][0]:.3f},{r['xy'][1]:.3f}) "
              f"lying={r['lying']}")
    pl.Path(args.out).write_text(json.dumps(shard, indent=1))
    print(f"{len(shard)} winners -> {args.out}")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('gen'); g.add_argument('--uids', type=int, nargs='+', required=True)
    g.add_argument('--out', required=True); g.add_argument('--wide', action='store_true')
    g.add_argument('--lying', action='store_true')
    r = sub.add_parser('run'); r.add_argument('--jobs', required=True)
    r.add_argument('--slice', type=int, nargs=2, required=True, metavar=('K', 'N'))
    r.add_argument('--out', required=True)
    r.add_argument('--env-path', action='store_true',
                   help='score candidates with the COLLECTOR stepping (GenesisCanEnv, '
                        'raw commands, per-step obs reads) instead of replay_harness.'
                        'rollout. Genesis 0.2.1 getter call-count shifts the solver '
                        'FP stream -> the two paths are different dynamical basins; '
                        'a winner must be validated in the basin it will be USED in.')
    p = sub.add_parser('pick'); p.add_argument('--parts', nargs='+', required=True)
    p.add_argument('--out', required=True)
    a = ap.parse_args()
    dict(gen=gen, run=run, pick=pick)[a.cmd](a)
