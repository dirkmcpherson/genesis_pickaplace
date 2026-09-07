#!/usr/bin/env bash
# amendment (j) RE-SCORE (2026-09-07): every world-model cell affected by the eval/env review fixes, re-run with the
# patched evaluator (pinned bank entries, physical-grip banks, nested_honest / placed_v2 in scope=full) into NEW dirs
# fresh_eval_<tag>_<mode>_v2. The cells of record (fresh_eval_<tag>_<mode>) are never touched. CPU only. Lane L of NLANES
# runs every cell line i with i % NLANES == L from the cell file, 4 concurrent evals (4 cores each, like the launcher of
# record's -n 4); 4 lanes x 4 = 16 concurrent evals, the brief's cap.
# usage: sbatch rs2_rescore.sbatch <cells.txt> <lane> <nlanes>      (re-runnable: cells with a metrics.json are skipped)
#SBATCH -J rs2_rescore
#SBATCH -p batch,preempt
#SBATCH --exclude=pax077
#SBATCH --requeue
#SBATCH -N 1
#SBATCH -n 16
#SBATCH --mem=60g
#SBATCH -t 1-12:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
CELLS=$1; LANE=$2; NLANES=$3
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID}
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
mkdir -p $W/rs2_logs
echo "# $(date -Is) host=$(hostname) lane=$LANE/$NLANES cells=$CELLS evaluator md5 $(md5sum $W/r2dreamer_fix/eval_genesis.py | cut -c1-12) adapter $(md5sum $W/r2dreamer_fix/envs/genesis.py | cut -c1-12) full_env $(md5sum $W/gp_root/baselines/rl/full_env.py | cut -c1-12)"
cd $W/r2dreamer_fix
run_cell() {   # <run> <kind: phase|full> <bank-or-icset> <tag> <mode>
  local RUN=$1 KIND=$2 SRC=$3 TAG=$4 MODE=$5
  local LOGDIR=$W/runs/$RUN OUT=$W/runs/$RUN/fresh_eval_${TAG}_${MODE}_v2 LOG=$W/rs2_logs/${RUN}__${TAG}_${MODE}.log
  [ -f $LOGDIR/latest.pt ] || { echo "FATAL no checkpoint $LOGDIR/latest.pt"; return 1; }
  [ -f $LOGDIR/fresh_eval_${TAG}_${MODE}/metrics.json ] || { echo "FATAL no cell of record $LOGDIR/fresh_eval_${TAG}_${MODE}/metrics.json"; return 1; }
  [ -f $OUT/metrics.json ] && { echo "skip (done) $RUN $TAG $MODE"; return 0; }
  rm -rf $OUT
  echo "# $(date -Is) START $RUN $TAG $MODE src=$SRC"
  if [ $KIND = phase ]; then
    [ -s "$SRC" ] || { echo "FATAL bank missing $SRC"; return 1; }
    "$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes 1 --mode $MODE --max-steps 600 --ic-set all --entry-bank $SRC \
        --seed 0 --out $OUT --device cpu > $LOG 2>&1
  else
    local N=15; [ $SRC = rnd ] && N=30
    "$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes $N --mode $MODE --max-steps 1200 \
        --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $SRC --seed 0 --out $OUT --device cpu > $LOG 2>&1
  fi
  local rc=$?
  echo "# $(date -Is) END rc=$rc $RUN $TAG $MODE $(grep -E '^\[eval\] [0-9]+ episodes|pinned entries' $LOG | tail -2 | tr '\n' ' ')"
  return $rc
}
export -f run_cell; export W LAB PY
awk -v L=$LANE -v N=$NLANES 'NR % N == L' $CELLS | xargs -P 4 -L 1 bash -c 'run_cell "$@"' _
echo "# $(date -Is) LANE-DONE $LANE"
