#!/bin/bash
# Repair sweep cells whose per-episode jsons are complete but whose sweep.json is missing (observed 2026-09-07: a
# sweep's parent shell can be killed between `wait` and the aggregation, losing the cell while every episode survived).
# eval_sweep.sh is resume-safe, so re-invoking it skips the episodes and only aggregates -- seconds per cell.
# usage: bash cluster/sweep_repair.sh <run-dir-glob>...    e.g. bash cluster/sweep_repair.sh 'baselines/rl/checkpoints/rlpd_g99v2fullw3_*'
set -uo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT
CELLS=(final_sampled:hold,rnd:sampled final_det15:hold:det final_sampled_spots60:spots60:sampled final_det_spots60:spots60:det)
n_ok=0; n_fix=0; n_fail=0
for RUN in "$@"; do
  for D in $RUN; do
    SW=$D/sweep; [ -d "$SW" ] || continue
    ARM=$(basename "$D" | sed -E 's/^rlpd_[^_]+_(.*)_s[0-9]+$/\1/'); SEED=$(basename "$D" | grep -oE 's[0-9]+$' | tr -d s)
    CK=$D/ckpt_100/rlpd_ckpt.zip; [ -f "$CK" ] || continue
    for C in "${CELLS[@]}"; do
      CELL=${C%%:*}; REST=${C#*:}; SETS=${REST%%:*}; MODE=${REST##*:}
      OUT_D=$SW/$CELL; [ -d "$OUT_D" ] || continue
      NJ=$(ls "$OUT_D"/*.json 2>/dev/null | grep -cv sweep.json)
      if [ -f "$OUT_D/sweep.json" ]; then n_ok=$((n_ok+1)); continue; fi
      [ "$NJ" -eq 0 ] && continue
      ICF=baselines/eval_ics.json; [ "$SETS" = spots60 ] && ICF=baselines/eval_ics_spots60.json
      SMP=(); [ "$MODE" = sampled ] && SMP=(--sample-actions)
      echo "REPAIR $OUT_D ($NJ episode jsons, no sweep.json)"
      bash cluster/eval_sweep.sh sac "$CK" "$OUT_D" --sets "$SETS" "${SMP[@]}" --no-video --tag "$CELL" \
        --ic-file "$ICF" --max-steps 1200 --arm "$ARM" --seed "$SEED" --ckpt-step ckpt_100 --reward sparse > "$OUT_D/../${CELL}_repair.log" 2>&1
      if [ -f "$OUT_D/sweep.json" ]; then echo "  REPAIRED"; n_fix=$((n_fix+1)); else echo "  FAILED (see ${CELL}_repair.log)"; n_fail=$((n_fail+1)); fi
    done
  done
done
echo "SWEEP-REPAIR done: $n_ok already complete, $n_fix repaired, $n_fail failed"
