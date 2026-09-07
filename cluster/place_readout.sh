#!/bin/bash
# One-command readout for the DP / RLPD place arms (PHASE_PLAN amendment (h)) + the spots60/sampled pick re-evals.
# Run on the cluster login node from the code clone ($LAB/gp_place). Steps:
#   1. for every finished place run (rlpd_final.zip / 100000 checkpoint present) run any MISSING eval cell -- in
#      particular the polE cells, which the in-job stage DEFERS until $W/phase_banks/polE_place.json carries
#      `bank_version` (adversarial review S2-4). Cells are resume-safe (existing metrics.json kept). CPU, sequential.
#   2. print the learner x source place table (baselines/place_table_all.py) with exact permutation tests;
#   3. print the RLPD sampled/deterministic/spots60 table and the DP spots60 table.
# usage: bash cluster/place_readout.sh [--no-eval]   (--no-eval: tables only)
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=${W:-$LAB/wm_fix_2026-09-03}
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
if [ "${1:-}" != "--no-eval" ]; then
  module load anaconda/2025.06.0 2>/dev/null; conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}" 2>/dev/null
  for A in dH dDP; do for S in $(seq 0 7); do
    R=baselines/rl/checkpoints/place/pl_rlpd_${A}_s$S
    [ -f "$R/rlpd_final.zip" ] && KIND=sac CKPT=$R/rlpd_final.zip OUT=$R ARM=$A SEED=$S PAR=${PAR:-4} bash cluster/place_eval_cells.sh 2>&1 | grep -E "PLACE-HEADLINE|POLE-|# cell|FATAL"
    D=baselines/outputs/dp_place/pl_dp_${A}_s$S
    [ -d "$D/checkpoints/100000/pretrained_model" ] && KIND=dp CKPT=$D/checkpoints/100000/pretrained_model OUT=$D ARM=$A SEED=$S PAR=${PAR:-2} bash cluster/place_eval_cells.sh 2>&1 | grep -E "PLACE-HEADLINE|POLE-|# cell|FATAL"
  done; done
fi
echo; echo "## place table (learner x source), polE tag ${POLE_TAG:-polE}"
python3 baselines/place_table_all.py --wm-runs "$W/runs" --polE-tag "${POLE_TAG:-polE}"
echo; echo "## RLPD pick checkpoints of record: deterministic vs sampled, hold15/rnd30/spots60"
python3 analysis/rlpd_sampled_table.py --root "$LAB/genesis_pickaplace/baselines/rl/checkpoints"
echo; echo "## DP pick checkpoints of record on spots60"
python3 analysis/dp_spots60_table.py --root "$LAB/genesis_pickaplace/baselines/outputs"
