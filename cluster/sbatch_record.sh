#!/bin/bash
# Record contract-v1 demo tapes as CPU batch jobs (one shard per job).
#
#   TEACHER=dp|r2d|human|random  CKPT=<path>  OUTDIR=baselines/demos_v1/<set>  SHARD_N=16 [GRES=gpu:1 PARTITION=gpu] \
#     EXTRA="--ic-mode demo --ic-from-tape --attempts 3 --mode sample --verify --target-kept 5" \
#     bash cluster/sbatch_record.sh            # submits SHARD_N jobs (array)
#
# Env (all optional unless stated): TEACHER (required), CKPT (dp/r2d), SRC (human: stride-1 dir),
# OUTDIR (required), SHARD_N (default 8), EXTRA (passed verbatim to record_demos.py), VENV_PY
# (python to use; default: conda env python; r2d needs $LAB/r2d_venv/bin/python), SEED (0),
# ROLLOUT_BASE (100000), PARTITION (batch), TIME (2:00:00), CPUS (2), MEM (8g), DRYRUN=1.
# Logs: $OUTDIR/../logs/<set>_shard<K>_<jobid>.log. Afterwards: record_demos.py --teacher T --merge --outdir OUTDIR.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace}
CONDA_ENV=${CONDA_ENV:-$LAB/condaenv/genesis}
: "${TEACHER:?TEACHER required}"; : "${OUTDIR:?OUTDIR required}"
SHARD_N=${SHARD_N:-8}; SEED=${SEED:-0}; ROLLOUT_BASE=${ROLLOUT_BASE:-100000}
PARTITION=${PARTITION:-batch}; TIME=${TIME:-2:00:00}; CPUS=${CPUS:-2}; MEM=${MEM:-8g}
GRES=${GRES:-}   # e.g. GRES=gpu:1 PARTITION=gpu for the dp adapter (policy on GPU, sim on CPU)
GRES_FLAG=(); [ -n "$GRES" ] && GRES_FLAG=(--gres "$GRES")
EXTRA=${EXTRA:-}
case "$TEACHER" in
  r2d) VENV_PY=${VENV_PY:-$LAB/r2d_venv/bin/python}; ARGS="--checkpoint ${CKPT:?CKPT required for r2d}" ;;
  dp)  VENV_PY=${VENV_PY:-}; ARGS="--checkpoint ${CKPT:?CKPT required for dp}" ;;
  human) VENV_PY=${VENV_PY:-}; ARGS="--src ${SRC:-baselines/episodes_pick_phase_dppruned}" ;;
  sched) VENV_PY=${VENV_PY:-}; ARGS="--src ${SRC:?SRC required for sched (make_ablation_sets.py schedule dir)}" ;;
  random) VENV_PY=${VENV_PY:-}; ARGS="" ;;
  *) echo "FATAL: TEACHER=$TEACHER"; exit 1 ;;
esac
SET=$(basename "$OUTDIR"); LOGDIR=$(dirname "$OUTDIR")/logs; mkdir -p "$LOGDIR"
echo "[record] teacher=$TEACHER set=$SET shards=$SHARD_N partition=$PARTITION extra='$EXTRA' py=${VENV_PY:-conda:$CONDA_ENV}"
for K in $(seq 0 $((SHARD_N - 1))); do
  # sim is CPU regardless (FullTaskEnv backend='cpu'); only blank CUDA when no GPU was requested,
  # so a GRES=gpu:1 dp job keeps its GPU for the policy
  if [ -n "$GRES" ]; then CMD="cd $ROOT && export GENESIS_PICKAPLACE_ROOT=$ROOT"; else CMD="cd $ROOT && export CUDA_VISIBLE_DEVICES='' GENESIS_PICKAPLACE_ROOT=$ROOT"; fi
  if [ -z "$VENV_PY" ]; then
    CMD="$CMD && source ~/.bashrc >/dev/null 2>&1; conda activate $CONDA_ENV && PY=python"
  else
    CMD="$CMD && PY=$VENV_PY"
  fi
  CMD="$CMD && \$PY baselines/record_demos.py --teacher $TEACHER $ARGS --outdir $OUTDIR --seed $SEED --shard-idx $K --shard-n $SHARD_N --rollout-base $ROLLOUT_BASE $EXTRA"
  if [ "${DRYRUN:-0}" = "1" ]; then echo "[dry] sbatch -p $PARTITION ${GRES_FLAG[*]} -c $CPUS --mem $MEM --time $TIME -J rec_${SET}_$K --wrap \"$CMD\""; continue; fi
  sbatch -p "$PARTITION" "${GRES_FLAG[@]}" -c "$CPUS" --mem "$MEM" --time "$TIME" -J "rec_${SET}_$K" \
         --output "$LOGDIR/${SET}_shard${K}_%j.log" --wrap "$CMD"
done
