#!/usr/bin/env python3
"""Merge the per-episode metrics.json files of an ISOLATED end-to-end cell into one cell-level metrics.json
(PHASE_PLAN amendment (n); coordinator finding 2026-09-07 "full-scope episodes are order-dependent").

An isolated cell is produced by cluster/e2e_eval_cells.sh as <cell>/ep<k>/metrics.json, one FRESH PROCESS per start
(baselines/eval_e2e.py --ic-index k). Genesis allows one world per process, so a fresh process is the only available
form of episode independence; this script glues the single-episode results back into the layout every reader
(baselines/e2e_table_all.py, cluster/e2e_eval_cells.sh's headline, paper tables) already understands.

Rules:
  * every expected index 0..n-1 must be present, or the merge FAILS (a missing episode is never silently dropped --
    it would change the denominator and therefore the rate);
  * the merged summary carries isolation='fresh_process', the set of nodes and pids that produced it, and per-episode
    node/pid/order stamps as written by eval_e2e.py;
  * scalar protocol fields (mode, seed, ic_file, ic_set, max_steps, sim_variant, action_repeat, delta_cap/leash,
    act_selection) must AGREE across every episode, or the merge fails: a cell assembled from processes that ran
    different protocols is not a cell.

usage: merge_e2e_iso.py --cell <dir> --n <expected episodes> [--out <dir>/metrics.json]
"""
import argparse, json, os, sys

STAGES = ('picked', 'placed', 'placed_v2', 'contact', 'contact_push', 'slide_success', 'nested_proxy', 'nested_honest')
OUTCOMES = ('nested_proxy', 'tipped', 'timeout')
MUST_AGREE = ('kind', 'mode', 'seed', 'max_steps', 'ic_file', 'ic_set', 'scope', 'sim_variant', 'action_repeat',
              'act_selection', 'delta_cap', 'delta_leash', 'checkpoint', 'amendment', 'eval_fixes')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cell', required=True)
    ap.add_argument('--n', type=int, required=True)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    parts = []
    missing = []
    for k in range(a.n):
        f = os.path.join(a.cell, f'ep{k}', 'metrics.json')
        if not os.path.exists(f):
            missing.append(k); continue
        d = json.load(open(f))
        if int(d['episodes']) != 1 or len(d['per_episode']) != 1:
            sys.exit(f'FATAL: {f} holds {d["episodes"]} episodes; an isolated shard must hold exactly 1')
        if d.get('isolation') != 'fresh_process':
            sys.exit(f'FATAL: {f} is stamped isolation={d.get("isolation")!r}, not fresh_process')
        if int(d['per_episode'][0]['ep']) != k:
            sys.exit(f'FATAL: {f} holds episode {d["per_episode"][0]["ep"]}, expected {k}')
        parts.append((k, d))
    if missing:
        sys.exit(f'FATAL: isolated cell {a.cell} is missing episode(s) {missing} of {a.n} -- refusing to merge a '
                 f'short cell (it would change the denominator)')
    base = parts[0][1]
    for k, d in parts[1:]:
        bad = [f for f in MUST_AGREE if d.get(f) != base.get(f)]
        if bad:
            sys.exit(f'FATAL: episode {k} disagrees with episode 0 on {bad} -- not one cell')
    eps = [d['per_episode'][0] for _, d in parts]
    n = len(eps)
    sc = {s: sum(int(bool(e['stages'].get(s))) for e in eps) for s in STAGES}
    oc = {o: sum(1 for e in eps if e['outcome'] == o) for o in OUTCOMES}
    routes = {}
    for e in eps:
        routes[str(e.get('slide_route'))] = routes.get(str(e.get('slide_route')), 0) + 1
    devs = [d.get('sample_dev_mean') for _, d in parts if d.get('sample_dev_mean') is not None]
    out = dict(base)
    out.update(episodes=n, isolation='fresh_process', ic_index=None, ic_offset=0,
               nodes=sorted({e['node'] for e in eps}), pids=sorted({e['pid'] for e in eps}),
               slide_success=sc['slide_success'] / n,
               stages={s: sc[s] / n for s in STAGES}, stage_counts=sc,
               outcomes={o: oc[o] / n for o in OUTCOMES}, slide_routes=routes,
               mean_steps=sum(e['steps'] for e in eps) / n, mean_reward=sum(e['reward'] for e in eps) / n,
               sample_dev_mean=(sum(devs) / len(devs) if devs else None),
               sample_dev_max=(max(d.get('sample_dev_max') or 0.0 for _, d in parts) if devs else None),
               seconds=round(sum(float(d.get('seconds') or 0.0) for _, d in parts), 1),
               merged_from=f'{a.cell}/ep*/metrics.json', merger='baselines/merge_e2e_iso.py',
               per_episode=eps)
    dst = a.out or os.path.join(a.cell, 'metrics.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print(f'MERGE-OK {dst}: {n} isolated episodes, nodes {out["nodes"]}, {len(out["pids"])} pids, '
          f'slide_success {sc["slide_success"]}/{n}, picked {sc["picked"]}/{n}, '
          f'nested_honest {sc["nested_honest"]}/{n} [{out["seconds"]:.0f} s of process time]')


if __name__ == '__main__':
    main()
