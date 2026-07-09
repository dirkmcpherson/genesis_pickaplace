"""Merge placement-search shards into trial_placements.json, then validate the merged
table end-to-end: replay every solved trial in one process (example.py order) N times
and report per-trial and aggregate success, per checkpoint.

Usage:
  merge_and_validate.py merge <shard.json ...>          -> writes trial_placements.json
  merge_and_validate.py validate [N]                    -> replays solved trials N times
"""
import sys, json, pathlib as pl
import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
OUT = REPO / 'can_pos_recovery/trial_placements.json'

# world corrections the search ran under (see CAN_STARTING_POSITION.md follow-up notes):
# v2 = table under the pick area (bag tool_pose proves cans sat at z=0.05, not the ground
# plane) + true Campbell's-can dimensions confirmed from the trial videos.
WORLD = dict(can_height=0.101, can_radius=0.033, can_rho=1000,
             finger_kp=100.0, finger_force=50.0, substeps=8, table=True)


def merge(paths):
    """Later files override earlier ones only with a strictly better solution class:
    'ok' (CPU-verified, transfers to validation by construction) > 'ok_batch' (GPU
    batch winner, may not transfer to CPU) > anything else."""
    RANK = {'ok': 2, 'ok_batch': 1}
    merged = {'world': WORLD, 'trials': {}}
    for p in paths:
        d = json.loads(pl.Path(p).read_text())
        for uid, r in d.items():
            if uid == 'world':
                continue
            keep = {k: r.get(k) for k in ('status', 'can_pos', 'can_quat', 'goal_pos',
                                          'goal_moved', 'success_rate', 'picked_rate',
                                          'label', 'conf', 'pos')}
            # data-collection protocol: cans ALWAYS start upright. Lying-spawn solutions
            # are degenerate (the rotated camera feed misled an earlier analysis) --
            # reject them regardless of sim success.
            if keep.get('can_quat') and abs(keep['can_quat'][0] - 1.0) > 1e-3:
                keep['status'] = 'rejected_lying_start'
            prev = merged['trials'].get(uid)
            if prev is None or RANK.get(keep['status'], 0) > RANK.get(prev['status'], 0):
                merged['trials'][uid] = keep
    ok = [u for u, r in merged['trials'].items() if r['status'] in RANK]
    OUT.write_text(json.dumps(merged, indent=1))
    print(f"wrote {OUT}: {len(merged['trials'])} trials, {len(ok)} with working placements")


def validate(n_rep):
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    from replay_harness import build_world, load_episode, rollout
    table = json.loads(OUT.read_text())
    wcfg = table['world']
    w = build_world(backend='cpu', finger_force=wcfg['finger_force'],
                    finger_kp=wcfg['finger_kp'], can_height=wcfg['can_height'],
                    can_rho=wcfg['can_rho'], substeps=wcfg.get('substeps', 1),
                    table=wcfg.get('table', False),
                    can_radius=wcfg.get('can_radius', 0.035))
    solved = {int(u): r for u, r in table['trials'].items() if r['status'] in ('ok', 'ok_batch')}
    agg = {'picked': 0, 'placed': 0, 'success': 0, 'nested': 0, 'n': 0}
    print(f"{'uid':>4} {'label':>8}  contact(S)/nested(N) x{n_rep}   c-rate n-rate")
    for uid in sorted(solved):
        r = solved[uid]
        vel, gp = load_episode(uid)
        outcomes = []
        for _ in range(n_rep):
            # stop at first contact (example.py semantics); nested is scored there
            # after a settle -- the task's semantic end
            res = rollout(w, vel, gp, tuple(r['can_pos']), tuple(r['goal_pos']),
                          can_quat=tuple(r.get('can_quat') or (1, 0, 0, 0)))
            outcomes.append(res)
            agg['picked'] += res['picked']; agg['placed'] += res['placed']
            agg['success'] += res['success']; agg['nested'] += bool(res['nested'])
            agg['n'] += 1
        marks = " ".join((('S' if o['success'] else '.') + ('N' if o['nested'] else '.'))
                         for o in outcomes)
        crate = sum(o['success'] for o in outcomes) / n_rep
        nrate = sum(bool(o['nested']) for o in outcomes) / n_rep
        print(f"{uid:>4} {r['label']:>8}  {marks}   {crate:.2f}  {nrate:.2f}", flush=True)
    n = agg['n']
    print(f"\naggregate over {n} rollouts: picked {agg['picked']/n:.2f} "
          f"placed {agg['placed']/n:.2f} contact-success {agg['success']/n:.2f} "
          f"nested-success {agg['nested']/n:.2f}")


if __name__ == '__main__':
    if sys.argv[1] == 'merge':
        merge(sys.argv[2:])
    else:
        validate(int(sys.argv[2]) if len(sys.argv) > 2 else 3)
