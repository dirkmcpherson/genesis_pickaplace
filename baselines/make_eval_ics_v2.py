#!/usr/bin/env python3
"""Generate the PREREG-A23 v2 evaluation IC file for one world from its dHv2 matched set.

A23 (withdrew A22): v2 learners train on ALL kept demo ICs; checkpoint selection uses a
fixed 15-IC training subset (mirroring how the frozen sel was a sorted-success-uid
prefix, make_eval_ics.py); the final/selected checkpoint is scored ONCE on (a) ALL
training ICs -- reported strictly as IN-DISTRIBUTION -- and (b) the FIXED rnd-30 copied
verbatim from baselines/eval_ics.json (the headline statistic, comparable to A16/A20).

Set names keep the launcher contract (sel/hold/rnd; eval_sweep.sh + dp_select_confirm.sh
+ sbatch_rlpd.sh consume them unchanged via IC_FILE=<this file>):
  sel   first 15 uids of the sorted kept training-IC list (selection; TRAINING subset)
  hold  ALL kept training ICs (in-distribution readout -- NOT a holdout; A23 renames
        the semantics, the key stays 'hold' so no launcher changes)
  rnd   the fixed 30 support ICs, byte-copied from --base

Usage:
  python baselines/make_eval_ics_v2.py --set baselines/matched_v2/dHv2 \
      --out baselines/eval_ics_v2.json
  python baselines/make_eval_ics_v2.py --set baselines/matched_w3/dHv2 \
      --out baselines/eval_ics_v2_w3.json
"""
import argparse
import json
import os
import pathlib as pl
import subprocess

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', dest='set_dir', required=True, help='dHv2 matched set (manifest.json holds the IC histogram)')
    ap.add_argument('--base', default='baselines/eval_ics.json', help='frozen eval IC file (rnd source)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--n-sel', type=int, default=15)
    args = ap.parse_args()

    man = json.loads((REPO / args.set_dir / 'manifest.json').read_text())
    base = json.loads((REPO / args.base).read_text())
    ics = sorted(int(u) for u in man['ic_uid_histogram'])
    assert len(ics) >= args.n_sel, f'only {len(ics)} training ICs'
    try:
        git = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO, text=True).strip()
    except Exception:
        git = None
    out = dict(
        version=1,
        generator='baselines/make_eval_ics_v2.py',
        prereg='A21/A23',
        source_set=args.set_dir,
        source_set_sha=man.get('content_sha256'),
        sim_variant=man.get('sim_variant'),
        git=git,
        sel=ics[:args.n_sel],
        hold=ics,
        rnd=base['rnd'],
        sel_note=f'first {args.n_sel} of the sorted kept training ICs (selection subset; A23 mirror of the '
                 'frozen sorted-success-uid-prefix sel)',
        hold_note='ALL kept training ICs -- IN-DISTRIBUTION readout, not a holdout (A23); key kept as '
                  "'hold' for launcher compatibility",
        rnd_note=f'byte-copy of {args.base} rnd (the fixed rnd-30 headline statistic, comparable to A16/A20)',
        base_git=base.get('git'),
    )
    (REPO / args.out).write_text(json.dumps(out, indent=1))
    print(f'[eval_ics_v2] {args.out}: sel {len(out["sel"])} hold {len(out["hold"])} rnd {len(out["rnd"])} '
          f'sim_variant={out["sim_variant"]} from {args.set_dir} (sha {str(out["source_set_sha"])[:16]})')


if __name__ == '__main__':
    main()
