#!/bin/bash
# Pack PACK r2dreamer training runs onto ONE GPU (2026-08-28).
#
# WHY: r2dreamer on this task is CPU-bound Genesis + a small model: measured 0% GPU utilisation,
# ~3 GB GPU memory, ~4.7 cores average, 33 GB RSS, ~15 h wall (sacct 2970588). The QOS caps us at
# 10 GPUs, so one run per GPU wastes the GPU and the cap. Batched Genesis envs are NOT an option
# (probe 2970418: per-env reset perturbs the other envs). Packing changes nothing about a run --
# same launcher, same config, same code -- only which process shares the GPU. Runs get their own
# .out file so nothing interleaves; the RUN_REGISTRY rows share one job id (both stamped).
#
# Threads: unpacked jobs were oversubscribing (pax049: 64 cores, load 138 from 8 jobs), so each
# packed run gets OMP/MKL_NUM_THREADS = 8 = its share of the allocation. Logged in SESSION_LOG.
#
# Usage (all launcher knobs pass through the environment unchanged):
#   ARM=dR2Ddup13 PACK_SEEDS="202 203" DEMOSET=v2 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400 \
#   GENESIS_PICKAPLACE_ROOT=$PWD sbatch -p gpu --time=24:00:00 cluster/sbatch_r2dreamer_pack.sh
#
#SBATCH -J r2d-pack
#SBATCH -p gpu
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 16
#SBATCH --mem=96g
#SBATCH --time=1-00:00:00
#SBATCH --output=r2d_pack_%j.out
#SBATCH --error=r2d_pack_%j.out
set -uo pipefail
: "${PACK_SEEDS:?PACK_SEEDS required, e.g. \"202 203\"}"
LABEL=${ARM:-${CONFIG:-r2d}}   # ARM is optional (no-demo probes such as touchgoal have none)
GPR=${GENESIS_PICKAPLACE_ROOT:-/cluster/tufts/shortlab/jstale02/genesis_pickaplace}
cd "$GPR"
N=$(wc -w <<<"$PACK_SEEDS"); THR=$(( ${SLURM_CPUS_ON_NODE:-16} / N )); [ "$THR" -ge 2 ] || THR=2
echo "== r2d-pack job=$SLURM_JOB_ID node=$(hostname) arm=$LABEL seeds=[$PACK_SEEDS] threads/run=$THR gpu=${CUDA_VISIBLE_DEVICES:-?} start=$(date)"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -1
PIDS=(); OUTS=()
for s in $PACK_SEEDS; do
  out="r2d_train_${SLURM_JOB_ID}_s${s}.out"; OUTS+=("$out")
  ( export SEED=$s OMP_NUM_THREADS=$THR MKL_NUM_THREADS=$THR OPENBLAS_NUM_THREADS=$THR
    [ -n "${LOGDIR_BASE:-}" ] && export LOGDIR="${LOGDIR_BASE}_s${s}"   # per-seed logdir for arms that need a custom name
    unset PACK_SEEDS LOGDIR_BASE
    bash cluster/sbatch_r2dreamer.sh ) > "$out" 2>&1 &
  PIDS+=($!); echo "   launched seed $s pid $! -> $out"
  sleep 20      # stagger Genesis/Taichi init and the registry write
done
RC=0
for i in "${!PIDS[@]}"; do
  wait "${PIDS[$i]}"; r=$?; echo "== seed $(sed -n "$((i+1))p" <<<"${PACK_SEEDS// /$'\n'}") rc=$r ($(date))"
  [ "$r" -eq 0 ] || RC=$r
done
for o in "${OUTS[@]}"; do grep -a "R2D-RESULT" "$o" | sed "s|^|[$o] |"; done
echo "== r2d-pack done rc=$RC $(date)"
exit $RC
