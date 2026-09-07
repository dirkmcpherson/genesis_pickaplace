"""Build the machine-first set `dRL` (paper/MACHINE_FIRST_PLAN_2026-09-07.md §2): ONE success tape per
IC from the reward-only teacher's harvest -- the first success by attempt order (rollout id order
within an IC = attempt order, as the recorder numbers rollouts) -- written with make_matched_sets.py's
own writer so the manifest carries every field the RLPD / DP launchers gate on (contract v1,
sim_variant, n_kept, content_sha256, ic_uid_histogram). N = the teacher's yield on the 66 starts;
missing ICs are listed in the manifest (no subsampling of the human arm; asymmetry disclosed).

usage: python baselines/mf_build_drl.py --src baselines/demos_v2/dRL_w3 --out-root baselines/matched_w3 \
           --name dRL --base baselines/matched_w3/dHv2raw
"""
import argparse
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'baselines'))
from make_matched_sets import read_dir, write_set, git_sha  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', required=True, help='recorder success dir (the harvest)')
    ap.add_argument('--out-root', required=True)
    ap.add_argument('--name', default='dRL')
    ap.add_argument('--base', default='baselines/matched_w3/dHv2raw', help='the IC universe (ic_uid_histogram of its manifest)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    base_man = json.load(open(os.path.join(REPO, args.base, 'manifest.json')))
    universe = sorted(int(k) for k in base_man['ic_uid_histogram'])
    tapes = read_dir(os.path.join(REPO, args.src), want_label='success')
    by_ic = {}
    for t in sorted(tapes, key=lambda t: int(os.path.splitext(t['name'])[0])):   # rollout id = attempt order
        if t['ic_uid'] in universe and t['ic_uid'] not in by_ic:
            by_ic[t['ic_uid']] = t
    chosen = [by_ic[u] for u in universe if u in by_ic]
    missing = [u for u in universe if u not in by_ic]
    outside = sorted(set(t['ic_uid'] for t in tapes) - set(universe))
    print(f'[dRL] harvest {args.src}: {len(tapes)} success tapes over {len(set(t["ic_uid"] for t in tapes))} ICs; '
          f'universe {len(universe)} ({args.base}); kept {len(chosen)} (one per IC, first by attempt); '
          f'missing ICs {len(missing)}: {missing}; ICs outside the universe (dropped): {outside}')
    if args.dry_run:
        return
    extra = dict(role='machine-first set (reward-only teacher harvest), MACHINE_FIRST_PLAN_2026-09-07 §2',
                 matching='one success per IC, the first by attempt order (rollout id); no subsampling of the base',
                 base_set=args.base, base_content_sha256=base_man.get('content_sha256'),
                 ic_universe_n=len(universe), missing_ics=missing, ics_outside_universe=outside,
                 mf_builder='baselines/mf_build_drl.py', rows=int(sum(t['n'] for t in chosen)))
    sha, man = write_set(args.name, chosen, os.path.join(REPO, args.out_root), extra, git_sha(), 0,
                         [os.path.join(REPO, args.src)])
    print(f'[dRL] wrote {os.path.join(args.out_root, args.name)}: N={man["N"]} rows={extra["rows"]} '
          f'decisions_p50={man["decisions_p50"]} sha={sha[:16]}')


if __name__ == '__main__':
    main()
