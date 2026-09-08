#!/bin/bash
# PINNED, CPU-ONLY evaluation pass over finished end-to-end checkpoints -- the CELLS OF RECORD for PHASE_PLAN
# amendment (n) (coordinator 2026-09-07, amendments (s)/(t)).
#
# Why a separate pass. Long-horizon full-scope outcomes diverge with HARDWARE, and hardware is already partially
# confounded with arm in the published 8v8 (amendment (t)). Pinning the TRAINING jobs is the wrong lever: the
# checkpoint -- not the training node -- is the artefact. So training runs anywhere, its in-job evaluation is stamped
# `role: preview` and is never a number for a table, and every cell of record comes from THIS pass: ONE hardware
# configuration by construction, CPU only, both the shared-process cell (statistic of record, order-matched to
# PHASE_RESULTS 5.1) and the isolated cell (the correctness check of amendment (s)), written under <run>/rec/ so a
# preview cell and a cell of record never share a path.
#
# THE AXIS IS MACHINE SIZE, AND THE VERDICT IS IN: `cores` (coordinator, 2026-09-07). It rests only on processor
# counts, which are reliable: 53 of 53 same-core-count comparisons bit-identical across nodes, labels and code
# versions, and every one of the 19 disagreements had a 36-core machine on exactly one side. The instruction-set
# question is UNRESOLVED rather than ruled out -- the CPU-family labels behind both the original AVX claim and its
# withdrawal come from Slurm's AvailableFeatures, which are wrong on this cluster -- so isa/avx512f are stamped as
# diagnostics and are never the guard. Good news for scheduling: a core pin is satisfiable on ANY machine of the
# right size, so this pass needs no --nodelist and the CPU-only Diffusion Policy cost becomes ordinary parallel work.
#
# The launcher still REFUSES to run until SWEEP_VERDICT is stated explicitly, so nobody pins on a guess. Pass
# SWEEP_VERDICT=cores with REQUIRE_CORES=<n> (and THREADS=<n> alongside it, which costs nothing and removes one more
# free variable). SWEEP_VERDICT=threads is accepted only with an explicit written override: the audit that
# established the core rule never set a thread variable -- thread count simply WAS the core count in all 128
# comparisons -- so nothing on record shows a thread pin alone is sufficient.
#
# Pick <n> with cluster/hw_probe.sh, which groups machines by size and flags 36-core machines (the size on one side
# of every recorded disagreement). Submit:
#   for A in dH dDP; do for S in $(seq 0 7); do
#     SWEEP_VERDICT=cores REQUIRE_CORES=$C THREADS=$T LEARNER=rlpd ARM=$A SEED=$S \
#       sbatch -J e2erec_rlpd_${A}_s$S cluster/sbatch_e2e_rescore.sh
#   done; done
# Env vars:
#   LEARNER  rlpd | dp        ARM dH | dDP        SEED required
#   RUN_ROOT default per learner (baselines/rl/checkpoints/e2e | baselines/outputs/dp_e2e)
#   SWEEP_VERDICT cores  REQUIRED, and gates the launch. `threads` needs THREADS_ONLY_OK=<reason> as well.
#   REQUIRE_CORES machine physical-core count to demand (required when SWEEP_VERDICT=cores) -- the guard of record
#   THREADS       per-task thread count to pin alongside it (recommended; removes one more free variable)
#   SETS/MODES/ISO/ISO_SETS as e2e_eval_cells.sh   PAR (default = the job's cpus)
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
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
SWEEP_VERDICT=${SWEEP_VERDICT:-}; THREADS=${THREADS:-}; REQUIRE_CORES=${REQUIRE_CORES:-}
case "$SWEEP_VERDICT" in
  cores)
    [ -n "$REQUIRE_CORES" ] || { echo "FATAL: SWEEP_VERDICT=cores needs REQUIRE_CORES=<n> (see cluster/hw_probe.sh)"; exit 1; }
    [ "$REQUIRE_CORES" = 36 ] && echo "WARNING: pinning to 36 physical cores -- every recorded disagreement had a 36-core machine on exactly one side."
    [ -n "$THREADS" ] || echo "NOTE: THREADS unset; the core guard is the one of record, but pinning threads too removes a free variable."
    ;;
  threads)
    [ -n "$THREADS" ] || { echo "FATAL: SWEEP_VERDICT=threads needs THREADS=<n>"; exit 1; }
    [ -n "${THREADS_ONLY_OK:-}" ] || {
      echo "FATAL: SWEEP_VERDICT=threads is not supported by any evidence on record. The 128-comparison audit that"
      echo "       established the core-count rule never set a thread variable -- thread count simply WAS the"
      echo "       physical core count in every one of those comparisons -- so it cannot show that a thread pin"
      echo "       alone suffices. The verdict of record is \`cores\`. To override anyway, set"
      echo "       THREADS_ONLY_OK=<written reason>."; exit 1; }
    echo "OVERRIDE: threads-only pin, reason: ${THREADS_ONLY_OK}"
    ;;
  *) echo "HOLD: state the verdict you are acting on. The verdict of record is \`cores\` (coordinator 2026-09-07):"
     echo "      SWEEP_VERDICT=cores REQUIRE_CORES=<n> [THREADS=<n>]   (pick <n> with cluster/hw_probe.sh)"
     exit 1 ;;
esac
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
CPU_CORES=$(python3 -c "
import re
t=open('/proc/cpuinfo').read()
c=re.search(r'^cpu cores\s*:\s*(\d+)', t, re.M)
s=len(set(re.findall(r'^physical id\s*:\s*(\d+)', t, re.M))) or 1
print((int(c.group(1))*s) if c else t.count('processor\t'))" 2>/dev/null)
echo "== E2E-RESCORE learner=$LEARNER arm=$ARM seed=$SEED node=$(hostname) cores=${CPU_CORES} cpu='${CPU_MODEL}' verdict=$SWEEP_VERDICT require_cores='${REQUIRE_CORES}' threads='${THREADS}' out=$OUT $(date)"
# fail loudly rather than trust the label that put us here (Slurm's features mislabel a Cascade Lake node as broadwell)
if [ -n "$REQUIRE_CORES" ] && [ "$CPU_CORES" != "$REQUIRE_CORES" ]; then
  echo "FATAL: landed on a ${CPU_CORES}-core machine ($CPU_MODEL) but this pass requires ${REQUIRE_CORES} physical cores."; exit 1
fi
[ -n "$CKPT" ] && { [ -f "$CKPT" ] || [ -d "$CKPT" ]; } || { echo "FATAL: no finished checkpoint for $OUT (has the training run landed?)"; exit 1; }
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] KIND=$KIND CKPT=$CKPT CELL_DIR=rec ROLE=record verdict=$SWEEP_VERDICT REQUIRE_CORES='$REQUIRE_CORES' THREADS='$THREADS' MODES='$MODES' PAR=$PAR"
  exit 0
fi
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
KIND=$KIND CKPT="$CKPT" OUT="$OUT" ARM="$ARM" SEED="$SEED" SIM_VARIANT="$SIM_VARIANT" \
  SETS="${SETS:-hold15 rnd30 spots60}" MODES="$MODES" ISO="${ISO:-1}" ISO_SETS="${ISO_SETS:-rnd30 spots60}" \
  VIDEO_SETS="${VIDEO_SETS:-none}" REQUIRE_CORES="$REQUIRE_CORES" THREADS="$THREADS" ROLE=record CELL_DIR=rec PAR="$PAR" \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_rescore.log"
echo "JOB DONE $(date)"
