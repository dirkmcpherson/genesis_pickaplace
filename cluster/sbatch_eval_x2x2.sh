#!/bin/bash
# POST-HOC eval of the obs x action 2x2 checkpoints (4 cells x 3 seeds).
#
# The overnight 2x2 trainings completed but every eval step died: eval builds a
# genesis world, which the poisoned conda env segfaulted (fixed 07-31). Worse, the
# original launcher's eval mapping keyed the env off the OBS half of the cell name;
# the env must follow the ACTION half (it executes what the policy emits), and the
# policy's obs layout is handled by wandb_eval --obs. Mapping:
#
#   cell        env (follows action)          --obs (follows obs)
#   jobs_jact   joint                          env
#   jobs_eact   cartesian abs6                 joint
#   eobs_jact   joint                          ee
#   eobs_eact   cartesian abs6                 env
#
# Without this, jobs_eact runs abs6 pose targets through the joint env (dimension-
# compatible, semantically garbage) and reads ~0.00 no matter what the policy
# learned -- a confident wrong answer to the 2x2's central question (audit B1).
#
# Submit from the genesis_pickaplace root:  sbatch cluster/sbatch_eval_x2x2.sh

#SBATCH -J eval-x2x2
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-12:00:00
#SBATCH --output=eval_x2x2_%j.out
#SBATCH --error=eval_x2x2_%j.out

set -o pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl PYTHONUNBUFFERED=1
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

FAIL=0; DONE=0
# ~85 min per eval x 12 outruns the 12h limit around checkpoint 8; resubmit the
# remainder with e.g.:  CELLS="eobs_jact eobs_eact" sbatch --export=ALL,CELLS ...
for CELL in ${CELLS:-jobs_jact jobs_eact eobs_jact eobs_eact}; do
  case $CELL in
    jobs_jact) FLAGS="";                              OBS=env;;
    jobs_eact) FLAGS="--cartesian --control abs6";    OBS=joint;;
    eobs_jact) FLAGS="";                              OBS=ee;;
    eobs_eact) FLAGS="--cartesian --control abs6";    OBS=env;;
  esac
  for S in ${SEEDS:-0 1 2}; do
    CKPT=baselines/outputs/x2x2_${CELL}_s${S}/checkpoints/last/pretrained_model
    [ -d "$CKPT" ] || { echo "== SKIP $CELL s$S: no checkpoint at $CKPT"; continue; }
    echo "== eval $CELL s$S ($FLAGS --obs $OBS) $(date)"
    # one wandb_eval per PROCESS: genesis allows one world per process
    python baselines/wandb_eval.py --kind dp $FLAGS --obs "$OBS" \
      --ic-mode both --random "${EVAL_EPS:-15}" --seed 0 \
      --checkpoint "$CKPT" \
      --project genesis_x2x2 --group "x2x2_$CELL" --name "x2x2_${CELL}_s${S}-eval" \
      && DONE=$((DONE+1)) || { echo "== EVAL FAILED $CELL s$S"; FAIL=$((FAIL+1)); }
  done
done
echo "== x2x2 post-hoc eval: $DONE ok, $FAIL failed  $(date)"
[ "$FAIL" -eq 0 ]
