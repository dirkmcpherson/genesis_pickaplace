#!/usr/bin/env bash
# Pinned re-score of the PILOT's four staged {RLPD} checkpoints, with per-episode STAGE RECORDS,
# so the pilot cells can be read under the calibrated `slide_event` / `home` predicates instead of
# the withdrawn `slide_success` (HANDOFF_2026-09-11 §5.6; the pilot's slide_success column is a
# set-down-transient detector on policies, paper/PILOT_RESCORE_2026-09-11.md).
#
#   sbatch --array=0-3 cluster/ln_pilot_rescore.sbatch
#
# WHAT IS AND IS NOT CHANGED. The rollout runs under the ladder the policy TRAINED on -- `staged`,
# from each checkpoint's own sidecar -- and under the tip guard it trained on. The sidecars record
# `tip_guard: None`, i.e. they predate the argument, so the evaluator takes the class default
# `grip`, which IS the rule of record those runs ran under; nothing here passes --tip-guard, and
# passing `not_in_hand` would silently terminate episodes somewhere else and make the cells
# incomparable with the pilot's own. What the records add is the ability to score ANY predicate
# offline afterwards, with no second simulation.
#
# `$LAB/gp_unified` is READ ONLY here: checkpoints are read from it, and every byte written goes to
# $OUT_ROOT. The code comes from THIS tree (GENESIS_PICKAPLACE_ROOT), not from gp_unified, so a
# running lz_* job cannot be disturbed.
#
# HARDWARE. eval_e2e.py's --require-cores counts PHYSICAL cores, which is the project's guard of
# record for cells (the axis is machine size); note this is NOT the same quantity as
# cluster/ladderN_sets.sbatch's REQUIRE_CORES, which counts LOGICAL processors from /proc/cpuinfo.
# Same name, same number, two different measurements -- worth knowing before comparing guards.
#SBATCH -J ln_rescore
#SBATCH -p batch
#SBATCH --qos=normal
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 0-06:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%A_%a.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
PILOT=${PILOT:-$LAB/gp_unified/baselines/rl/checkpoints/e2e}
OUT_ROOT=${OUT_ROOT:-$W/pilot_rescore_2026-09-11}
SETS=${SETS:-rnd}          # rnd = the rnd30 box, the pilot's cell of record
MODE=${MODE:-mode}
REQ_CORES=${REQ_CORES:-64}

: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this re-score must use}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/eval_e2e.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 2; }
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G)"; exit 2; fi
done
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6

RUNS=(e2e_rlpd_dH_s940 e2e_rlpd_dH_s941 e2e_rlpd_dDPfirst_s960 e2e_rlpd_dDPfirst_s961)
R=${RUNS[${SLURM_ARRAY_TASK_ID:-0}]}
CKPT=$PILOT/$R/rlpd_final.zip
[ -f "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT"; exit 2; }

FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | run $R | tree $(git describe --always --dirty)"

OUT=$OUT_ROOT/$R/${SETS}_${MODE}
REC=$OUT_ROOT/$R/records_${SETS}_${MODE}
mkdir -p "$OUT" "$REC"
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"

python baselines/eval_e2e.py --kind sac --checkpoint "$CKPT" \
  --ic-file baselines/eval_ics.json --ic-set "$SETS" --mode "$MODE" --seed 0 \
  --out "$OUT" --records-out "$REC" --role record --require-cores "$REQ_CORES" --device cpu
echo "# DONE $R $(date -Is)"
