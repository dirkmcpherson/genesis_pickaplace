#!/bin/bash
# Review reels for the DP / RLPD place arms (PHASE_PLAN amendment (h)), the review_sweep.sh + tile_review.py conventions:
# per condition, K episodes sampled ACROSS SEEDS from the in-distribution cell (holdE sample) and the out-of-distribution
# cell (polE sample), seed-tagged file names, manifest.json, then one tiled mp4 per condition with the LEARNER named in
# the header (tile_review.py's learner_of reads 'rlpd' / '_dp_' from the run names).
# usage (on the dev box): bash cluster/place_review.sh   [K=4] [SEED=0]
#   cluster side: samples into $LAB/gp_place/review/<label>/{in,out}; local side: rsync to ~/wm_fix_2026-09-03/review/
#   and tile (the WM agent's tile_review.py, unchanged).
set -eo pipefail
K=${K:-4}; SEED=${SEED:-0}
LAB=/cluster/tufts/shortlab/jstale02; L=~/wm_fix_2026-09-03; PY=~/workspace/genesis_sim2real/venv/bin/python
ssh pax "cd $LAB/gp_place && python3 - $K $SEED <<'PY'
import glob, json, os, random, re, shutil, sys
K, seed = int(sys.argv[1]), int(sys.argv[2])
conds = [('place_rlpd_human_n39', 'baselines/rl/checkpoints/place/pl_rlpd_dH_s[0-7]'),
         ('place_rlpd_machine_n39', 'baselines/rl/checkpoints/place/pl_rlpd_dDP_s[0-7]'),
         ('place_dp_human_n39', 'baselines/outputs/dp_place/pl_dp_dH_s[0-7]'),
         ('place_dp_machine_n39', 'baselines/outputs/dp_place/pl_dp_dDP_s[0-7]')]
for label, rglob in conds:
    runs = sorted(glob.glob(rglob)); out = f'review/{label}'
    man = {'label': label, 'runs': [os.path.basename(r) for r in runs], 'in': [], 'out': []}
    for kind, cell in (('in', 'holdE_sample'), ('out', 'polE_sample')):
        pool = []
        for r in runs:
            m = re.search(r'_s(\d+)$', r); s = m.group(1) if m else '?'
            pool += [(s, f) for f in sorted(glob.glob(f'{r}/fresh_eval_{cell}/ep*.mp4'))]
        if not pool:
            print(f'[review] {label} {kind}: no videos in cell {cell}'); continue
        rng = random.Random(seed); pick = rng.sample(pool, min(K, len(pool)))
        os.makedirs(f'{out}/{kind}', exist_ok=True)
        for s, f in sorted(pick):
            dst = f'{out}/{kind}/s{s}_{os.path.basename(f)}'; shutil.copy(f, dst)
            man[kind].append(dict(seed=s, src=f, file=os.path.basename(dst), outcome=os.path.basename(f).rsplit('_', 1)[-1][:-4]))
        print(f'[review] {label} {kind} ({cell}): {len(pick)} of {len(pool)} episodes across {len({s for s, _ in pool})} seeds -> {out}/{kind}')
    if runs:
        os.makedirs(out, exist_ok=True); json.dump(man, open(f'{out}/manifest.json', 'w'), indent=1)
PY" 2>&1 | grep -E "^\[review\]"
mkdir -p $L/review
rsync -aq "pax:$LAB/gp_place/review/place_*" $L/review/
for d in $L/review/place_rlpd_* $L/review/place_dp_*; do [ -f "$d/manifest.json" ] || continue; $PY $L/tile_review.py "$d" 1 4 120 2>&1 | grep -v Warning || true; done
