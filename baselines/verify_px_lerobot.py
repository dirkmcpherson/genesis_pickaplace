#!/usr/bin/env python
"""Gate a PIXEL lerobot dataset (PHASE_PLAN amendment (ag), 2026-09-14) against the STATE dataset
of record it must share an action column with.

The (ag) arm trains a Diffusion Policy on the SAME tapes as the end-to-end DP arm of record
(amendments (n)/(ab)) with the observation swapped for proprioception + two 64x64 cameras. Two
things therefore have to be true of the dataset, and neither is safe to assume:

  1. the ACTION column is the (n)/(ab) column, byte for byte, episode for episode -- same action
     space (absolute window-end joint targets rad + grip 0..1), same rows, same order. This is what
     makes the two rows of the three-learner table differ in OBSERVATION only.
  2. the observation is proprio + pixels and NOTHING else: no observation.environment_state, i.e.
     the ground-truth can pose and goal xy never reach the policy (the (af) `state_slice 8` cut).

Usage:
  python baselines/verify_px_lerobot.py --px <px dataset root> --ref <state dataset root> [--raw <recorder dir>]

Exits non-zero with a named failure; prints PX-DATASET-OK with the counts on success.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import pyarrow.parquet as pq

WANT_IMG = ('observation.images.top', 'observation.images.wrist')


def load(root):
    info = json.loads(open(os.path.join(root, 'meta', 'info.json')).read())
    src = json.loads(open(os.path.join(root, 'genesis_source.json')).read())
    files = sorted(glob.glob(os.path.join(root, 'data', '**', '*.parquet'), recursive=True))
    assert files, f'{root}: no data parquet'
    import pandas as pd
    df = pd.concat([pq.read_table(f).to_pandas() for f in files], ignore_index=True)
    return info, src, df


def actions_by_episode(df, src):
    """{source npz filename: action bytes} -- episode_index is the position in genesis_source.json."""
    out = {}
    for ep, name in enumerate(src['episodes']):
        sub = df[df['episode_index'] == ep].sort_values('frame_index')
        assert len(sub), f'episode {ep} ({name}) has no rows'
        out[name] = np.stack(sub['action'].to_numpy()).astype(np.float32).tobytes()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--px', required=True, help='the pixel dataset to gate')
    ap.add_argument('--ref', required=True, help='the state dataset of record (same raw tapes)')
    ap.add_argument('--raw', default=None, help='optional: the recorder tape dir, checked as a third witness')
    a = ap.parse_args()

    pinfo, psrc, pdf = load(a.px)
    rinfo, rsrc, rdf = load(a.ref)

    # ---- 2. features: proprio + two cameras + action, and NOTHING else ------------------------
    feats = {k: v for k, v in pinfo['features'].items()
             if k.startswith('observation.') or k == 'action'}
    want = {'observation.state', 'action', *WANT_IMG}
    if set(feats) != want:
        sys.exit(f'FATAL: px features {sorted(feats)} != {sorted(want)} '
                 f'(extra: {sorted(set(feats) - want)}, missing: {sorted(want - set(feats))})')
    if 'observation.environment_state' in pinfo['features']:
        sys.exit('FATAL: the pixel dataset still carries observation.environment_state (can pose + goal xy)')
    if tuple(feats['observation.state']['shape']) != (8,):
        sys.exit(f"FATAL: observation.state shape {feats['observation.state']['shape']} != (8,)")
    for k in WANT_IMG:
        if tuple(feats[k]['shape']) != (64, 64, 3):
            sys.exit(f'FATAL: {k} shape {feats[k]["shape"]} != (64, 64, 3)')
    if tuple(feats['action']['shape']) != tuple(rinfo['features']['action']['shape']):
        sys.exit('FATAL: action shape differs from the reference dataset')
    if abs(float(pinfo['fps']) - float(rinfo['fps'])) > 1e-9:
        sys.exit(f"FATAL: fps {pinfo['fps']} != reference {rinfo['fps']}")

    # ---- 1. the action column IS the reference column ----------------------------------------
    if psrc['episodes'] != rsrc['episodes']:
        only_px = sorted(set(psrc['episodes']) - set(rsrc['episodes']))
        only_ref = sorted(set(rsrc['episodes']) - set(psrc['episodes']))
        sys.exit(f'FATAL: episode lists differ (px-only {only_px[:5]}, ref-only {only_ref[:5]}, '
                 f'or a different order)')
    pa, ra = actions_by_episode(pdf, psrc), actions_by_episode(rdf, rsrc)
    bad = [n for n in psrc['episodes'] if pa[n] != ra[n]]
    if bad:
        sys.exit(f'FATAL: action column differs from the reference on {len(bad)} episodes: {bad[:5]}')

    # ---- 3. optional third witness: the recorder tapes themselves ----------------------------
    n_raw = 0
    if a.raw:
        for name in psrc['episodes']:
            z = np.load(os.path.join(a.raw, name), allow_pickle=True)
            if pa[name] != np.asarray(z['actions'], np.float32).tobytes():
                sys.exit(f'FATAL: action column differs from the recorder tape {name}')
            n_raw += 1

    print(f'PX-DATASET-OK {a.px}: episodes {pinfo["total_episodes"]} frames {pinfo["total_frames"]} '
          f'fps {pinfo["fps"]} | features {sorted(feats)} | action bytes == {a.ref} on '
          f'{len(psrc["episodes"])}/{len(psrc["episodes"])} episodes'
          + (f' and == the recorder tapes on {n_raw}/{n_raw}' if a.raw else '')
          + f' | images_from {psrc.get("images_from")} no_env_state {psrc.get("no_env_state")}')


if __name__ == '__main__':
    main()
