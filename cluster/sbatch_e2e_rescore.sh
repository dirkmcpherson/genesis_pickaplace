#!/bin/bash
# PINNED, CPU-ONLY evaluation pass over finished end-to-end checkpoints -- the CELLS OF RECORD for PHASE_PLAN
# amendment (n) (coordinator 2026-09-07, amendments (s)/(t)).
#
# Why a separate pass. Long-horizon full-scope outcomes diverge between AVX2 and AVX-512 parts, and hardware class is
# already partially confounded with arm in the published 8v8 (amendment (t)). Pinning the TRAINING jobs is the wrong
# lever: it would constrain them to whatever AVX2 GPU nodes exist, and the checkpoint -- not the training node -- is
# the artefact. So training runs anywhere, its in-job evaluation is stamped `role: preview` and is never a number for
# a table, and every cell of record comes from THIS pass: one instruction-set class by construction, CPU only, both
# the shared-process cell (statistic of record, order-matched to PHASE_RESULTS 5.1) and the isolated cell (the
# correctness check of amendment (s)), written under <run>/rec/ so a preview and a record cell never share a path.
#
# Submit (pin with --nodelist from cluster/isa_probe.sh -- NEVER --constraint: Slurm's labels mislabel at least one
# Cascade Lake node as `broadwell`, so a constraint cannot pin the class):
#   NODELIST=$(python3 -c "import json;print(','.join(sorted(n for n,d in json.load(open('isa_map.json')).items() if d['isa']=='avx2')))")
#   for A in dH dDP; do for S in $(seq 0 7); do
#     LEARNER=rlpd ARM=$A SEED=$S sbatch --nodelist="$NODELIST" -J e2erec_rlpd_${A}_s$S cluster/sbatch_e2e_rescore.sh
#   done; done
# Env vars:
#   LEARNER  rlpd | dp        ARM dH | dDP        SEED required
#   RUN_ROOT default per learner (baselines/rl/checkpoints/e2e | baselines/outputs/dp_e2e)
#   REQUIRE_ISA avx2 (default)   SETS/MODES/ISO/ISO_SETS as e2e_eval_cells.sh   PAR (default = the job's cpus)
#   DRYRUN=1 prints the plan
# The pass is CPU only on purpose: RLPD evaluation never needed a GPU, and DP evaluation runs on CPU too (slower),
# so node availability stops being a constraint on which class we can pin to.
#SBATCH -J e2erec
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=9000
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=32g
#SBATCH --time=1-00:00:00
#SBATCH --output=e2erec_%j.out
#SBATCH --error=e2erec_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
LEARNER=${LEARNER:?set LEARNER (rlpd | dp)}; ARM=${ARM:?set ARM (dH | dDP)}; SEED=${SEED:?set SEED}
REQUIRE_ISA=${REQUIRE_ISA:-avx2}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
PAR=${PAR:-${SLURM_CPUS_ON_NODE:-8}}
case "$LEARNER" in
  rlpd) RUN_ROOT=${RUN_ROOT:-baselines/rl/checkpoints/e2e}; OUT=$RUN_ROOT/e2e_rlpd_${ARM}_s${SEED}
        KIND=sac; CKPT=$OUT/rlpd_final.zip; DEF_MODES="sample mode" ;;
  dp)   RUN_ROOT=${RUN_ROOT:-baselines/outputs/dp_e2e};     OUT=$RUN_ROOT/e2e_dp_${ARM}_s${SEED}
        KIND=dp;  CKPT=$(ls -d "$OUT"/checkpoints/[0-9]*/pretrained_model 2>/dev/null | sort -V | tail -1); DEF_MODES="sample" ;;
  *) echo "FATAL: LEARNER must be rlpd | dp (got $LEARNER)"; exit 1 ;;
esac
MODES=${MODES:-$DEF_MODES}
CPU_MODEL=$(awk -F': ' '/^model name/{print $2; exit}' /proc/cpuinfo 2>/dev/null)
CPU_ISA=$(awk '/^flags/{if ($0 ~ /avx512f/) print "avx512"; else if ($0 ~ /avx2/) print "avx2"; else print "pre-avx2"; exit}' /proc/cpuinfo 2>/dev/null)
echo "== E2E-RESCORE learner=$LEARNER arm=$ARM seed=$SEED node=$(hostname) isa=${CPU_ISA} cpu='${CPU_MODEL}' require=$REQUIRE_ISA out=$OUT $(date)"
# fail loudly rather than trust the label that put us here (pax001 advertises `broadwell` and is Cascade Lake)
[ "$CPU_ISA" = "$REQUIRE_ISA" ] || { echo "FATAL: landed on $CPU_ISA ($CPU_MODEL) but this pass requires $REQUIRE_ISA -- Slurm feature labels are unreliable; pin with --nodelist from cluster/isa_probe.sh"; exit 1; }
[ -n "$CKPT" ] && { [ -f "$CKPT" ] || [ -d "$CKPT" ]; } || { echo "FATAL: no finished checkpoint for $OUT (has the training run landed?)"; exit 1; }
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] KIND=$KIND CKPT=$CKPT CELL_DIR=rec ROLE=record REQUIRE_ISA=$REQUIRE_ISA MODES='$MODES' PAR=$PAR"
  exit 0
fi
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
KIND=$KIND CKPT="$CKPT" OUT="$OUT" ARM="$ARM" SEED="$SEED" SIM_VARIANT="$SIM_VARIANT" \
  SETS="${SETS:-hold15 rnd30 spots60}" MODES="$MODES" ISO="${ISO:-1}" ISO_SETS="${ISO_SETS:-rnd30 spots60}" \
  VIDEO_SETS="${VIDEO_SETS:-none}" REQUIRE_ISA="$REQUIRE_ISA" ROLE=record CELL_DIR=rec PAR="$PAR" \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_rescore.log"
echo "JOB DONE $(date)"
