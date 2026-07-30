#!/bin/bash
# Submit a QUEUE of ouroboros lineages, one sbatch each.
#
# Each lineage is self-perpetuating: job A (train+eval) submits job B (harvest +
# convert), which submits the next generation's job A, up to MAXGEN. So the
# generational dependency needs NO slurm --dependency graph; only the lineages are
# submitted here, and they run concurrently and independently.
#
#   bash cluster/launch_ouroboros.sh CONDITIONS.txt
#
# CONDITIONS.txt: one lineage per line, KEY=VAL pairs, '#' comments ignored:
#   TAG=dp_joint   ALGO=dp  ACTIONS=joint  DATASET=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace
#   TAG=act_joint  ALGO=act ACTIONS=joint  DATASET=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace
#
# Applies to every lineage unless overridden per line:
#   MAXGEN (3) SCOPE (pick) HARVEST_N (900) TARGET_KEPT (67) IC_MODE (demo)
#   MIN_KEPT (10) CAMS (none) CTRL (vel) TRAIN_STEPS (100000) EVAL_EPS (15)
#
# TARGET_KEPT defaults to 67 = the human gen-0 demo count, so every generation and
# every algorithm trains on the SAME amount of data. HARVEST_N is the rollout budget
# to reach it (67 kept at a 0.13-rate teacher needs ~515 rollouts; 900 leaves slack).
# A lineage whose teacher cannot reach the target logs short_of_target in its
# manifest -- check that before comparing conditions.
set -eo pipefail
CONDITIONS=${1:?usage: launch_ouroboros.sh CONDITIONS.txt}
cd "${GENESIS_PICKAPLACE_ROOT:-$PWD}"

: "${MAXGEN:=3}"; : "${SCOPE:=pick}"; : "${HARVEST_N:=900}"; : "${TARGET_KEPT:=67}"
: "${IC_MODE:=demo}"; : "${MIN_KEPT:=10}"; : "${CAMS:=none}"; : "${CTRL:=vel}"
: "${TRAIN_STEPS:=100000}"; : "${EVAL_EPS:=15}"; : "${ACTIONS:=joint}"

n=0
while IFS= read -r line; do
  line="${line%%#*}"; [ -z "${line// }" ] && continue
  # per-line KEY=VAL override the defaults above, in a subshell so lines are independent
  (
    for kv in $line; do export "${kv?}"; done
    : "${TAG:?each line needs TAG=}"; : "${DATASET:?each line needs DATASET=}"
    if [ ! -d "$DATASET" ]; then
      echo "SKIP $TAG: gen-0 dataset missing: $DATASET" >&2
      exit 0
    fi
    echo "submit $TAG: ALGO=$ALGO ACTIONS=$ACTIONS CTRL=$CTRL scope=$SCOPE " \
         "maxgen=$MAXGEN target_kept=$TARGET_KEPT ic_mode=$IC_MODE"
    sbatch --job-name="ouro-$TAG" \
      --export=ALL,TAG=$TAG,ALGO=${ALGO:-dp},ACTIONS=$ACTIONS,CTRL=$CTRL,CAMS=$CAMS,\
DATASET=$DATASET,MAXGEN=$MAXGEN,SCOPE=$SCOPE,HARVEST_N=$HARVEST_N,\
TARGET_KEPT=$TARGET_KEPT,IC_MODE=$IC_MODE,MIN_KEPT=$MIN_KEPT,\
TRAIN_STEPS=$TRAIN_STEPS,EVAL_EPS=$EVAL_EPS,\
GENESIS_PICKAPLACE_ROOT=$PWD,CONDA_ENV=${CONDA_ENV:-} \
      cluster/sbatch_ouro_train.sh
  )
  n=$((n+1))
done < "$CONDITIONS"
echo "== submitted $n lineages; each self-chains through gen$((MAXGEN-1))"
echo "== watch: squeue -u \$USER | grep ouro   |   wandb project genesis_pickaplace_ouro"
