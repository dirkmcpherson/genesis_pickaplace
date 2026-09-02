#!/bin/bash
# Run ONCE on the login node: submits the whole A31 chain with --dependency=afterok links and logs every job id.
set -e; source "$(dirname "$0")/common.sh"; STAGE=launch; cd "$GPR" || exit 1; ALLT=()
mkdir -p "$CHAIN_LOGDIR" baselines/demos_v2/logs
export GENESIS_PICKAPLACE_ROOT=$GPR; unset DEMO_DIR RAW DATASET
sub() { local id; id=$(sbatch --parsable "$@") || { clog "FATAL: sbatch failed: $*"; exit 1; }; echo "${id%%;*}"; }
clog "=== A31 chain launch (git $(git rev-parse --short HEAD)) ==="
declare -A MERGE
for W in old w3; do world_cfg $W
  T=()
  for S in 0 1; do
    id=$(env ARM=dHv2 SEED=$S WAVE=$WAVE_T DEMO_ROOT=$DEMO_ROOT SIM_VARIANT=$SIM_VARIANT IC_FILE=$IC_FILE sbatch --parsable --exclude=pax007 -p gpu cluster/sbatch_dp.sh) || { clog "FATAL teacher sbatch"; exit 1; }
    T+=("${id%%;*}"); ALLT+=("${id%%;*}"); clog "teacher $W s$S (ARM=dHv2 WAVE=$WAVE_T): job ${id%%;*}"
  done
  SEL=$(sub -p batch -c 1 --mem 2g -t 0:20:00 -J a31sel_$W --dependency=afterok:${T[0]}:${T[1]} -o $CHAIN_LOGDIR/select_${W}_%j.out cluster/a31_chain/select.sh $W)
  clog "select $W: job $SEL (afterok ${T[0]}:${T[1]})"
  SH=(); for K in $(seq 0 $((SHARD_N-1))); do
    id=$(sub -p gpu --gres gpu:1 -c 4 --mem 16g --time 4:00:00 --exclude=pax007 -J rec_${SET}_$K --dependency=afterok:$SEL -o baselines/demos_v2/logs/${SET}_shard${K}_%j.log cluster/a31_chain/shard.sh $W $K)
    SH+=("$id")
  done
  clog "harvest shards $W ($SET): jobs ${SH[*]} (afterok $SEL)"
  DEP=$(IFS=:; echo "${SH[*]}")
  M=$(sub -p batch -c 4 --mem 16g -t 1:00:00 -J a31merge_$W --dependency=afterok:$DEP -o $CHAIN_LOGDIR/merge_${W}_%j.out cluster/a31_chain/merge.sh $W)
  MERGE[$W]=$M; clog "merge $W: job $M (afterok $DEP)"
done
B=$(sub -p batch -c 4 --mem 24g -t 3:00:00 -J a31build --dependency=afterok:${MERGE[old]}:${MERGE[w3]} -o $CHAIN_LOGDIR/build_%j.out cluster/a31_chain/build.sh)
clog "build: job $B (afterok ${MERGE[old]}:${MERGE[w3]})"
U=$(sub -p batch -c 1 --mem 2g -t 0:40:00 -J a31submit --dependency=afterok:$B -o $CHAIN_LOGDIR/submit_%j.out cluster/a31_chain/submit_learners.sh)
clog "submit: job $U (afterok $B)"
clog "=== chain launched; teachers ${ALLT[*]} run first ==="
