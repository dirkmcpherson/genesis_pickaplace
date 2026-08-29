#!/bin/bash
# Pack several dv3 (dreamerv3-torch) genesis runs onto ONE GPU (2026-08-29), mirroring
# genesis_pickaplace/cluster/sbatch_r2dreamer_pack.sh: dv3 on this task is CPU-bound Genesis + a
# small model (no diffusion), so one run per GPU wastes the 10-GPU QOS cap. Each packed run is the
# UNCHANGED launcher (sbatch_genesis_final_rr.sh) with its own .out, threads = its share of the CPUs.
#
# Usage (launcher knobs pass through the environment):
#   export PY=$LAB/condaenv/genesis/bin/python MUJOCO_GL=egl GENESIS_PICKAPLACE_ROOT=$LAB/genesis_pickaplace
#   PACK_SEEDS="23 24 25" ARM=dH REWARD=dense STEPS=3000000 EXTRA_CONFIGS=genesis_dv3std TAGSUF=-std \
#   DEMODIR=... sbatch -p gpu -n 24 --mem 96g -t 2-00:00:00 sbatch_genesis_pack.sh
#SBATCH -J dv3-pack
#SBATCH -p gpu
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 24
#SBATCH --mem=96g
#SBATCH -t 2-00:00:00
#SBATCH --output=dv3_pack_%j.out
#SBATCH --error=dv3_pack_%j.out
set -uo pipefail
: "${PACK_SEEDS:?PACK_SEEDS required}"
cd "${DV3_ROOT:-/cluster/tufts/shortlab/jstale02/dreamerv3-torch}"
N=$(wc -w <<<"$PACK_SEEDS"); THR=$(( ${SLURM_CPUS_ON_NODE:-24} / N )); [ "$THR" -ge 2 ] || THR=2
echo "== dv3-pack job=$SLURM_JOB_ID node=$(hostname) arm=${ARM:-?}${TAGSUF:-} seeds=[$PACK_SEEDS] threads/run=$THR gpu=${CUDA_VISIBLE_DEVICES:-?} start=$(date)"
PIDS=(); OUTS=()
for s in $PACK_SEEDS; do
  out="dv3_train_${SLURM_JOB_ID}_s${s}.out"; OUTS+=("$out")
  ( export SEED=$s OMP_NUM_THREADS=$THR MKL_NUM_THREADS=$THR OPENBLAS_NUM_THREADS=$THR SLURM_CPUS_PER_TASK=$THR
    unset PACK_SEEDS
    bash sbatch_genesis_final_rr.sh ) > "$out" 2>&1 &
  PIDS+=($!); echo "   launched seed $s pid $! -> $out"
  sleep 30
done
RC=0
for i in "${!PIDS[@]}"; do wait "${PIDS[$i]}"; r=$?; echo "== pack member $i rc=$r ($(date))"; [ "$r" -eq 0 ] || RC=$r; done
for o in "${OUTS[@]}"; do grep -a "DV3-HEADLINE\|== training done\|FATAL" "$o" | sed "s|^|[$o] |"; done
echo "== dv3-pack done rc=$RC $(date)"; exit $RC
