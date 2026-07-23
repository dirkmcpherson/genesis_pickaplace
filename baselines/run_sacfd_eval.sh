#!/bin/bash
# Post-train eval for SACfD-on-everything, WITH a trained-enough verdict:
#   A. checkpoint CURVE: eval every 25k checkpoint on the same 10 random ICs (short
#      400-step horizon -- pick settles early) -> pick-rate vs steps -> plateau check.
#   B. final numbers: 3 seeds x 15 random ICs at the FULL 1200-step horizon on the last
#      checkpoint -- directly comparable to DP-pick (same eval_core/ICs/horizon).
# Detached; waits for training to finish first.
cd /home/j/workspace/genesis_pickaplace
LOG=baselines/sacfd_eval.log
echo "[$(date)] SACfD EVAL: waiting for training to finish..." > $LOG
while [ "$(ps aux | grep train_sacfd | grep -v grep | wc -l)" -gt 0 ]; do sleep 300; done
echo "[$(date)] SACfD training finished" >> $LOG

CKDIR=baselines/rl/checkpoints/sacfd_all
FINAL=$CKDIR/sacfd_final.zip
[ -f "$FINAL" ] || FINAL=$(ls $CKDIR/sacfd_*_steps.zip 2>/dev/null | sort -t_ -k2 -n | tail -1)
if [ -z "$FINAL" ] || [ ! -f "$FINAL" ]; then echo "[$(date)] NO CHECKPOINT -> abort" >> $LOG; exit 1; fi
echo "[$(date)] final checkpoint: $FINAL" >> $LOG

# A. checkpoint curve (parallel batches of 3; short horizon; fixed seed -> same ICs each point)
echo "[$(date)] A: checkpoint curve (pick @ 400-step horizon, 10 ICs, seed 0)" >> $LOG
i=0
for ck in $(ls $CKDIR/sacfd_*_steps.zip 2>/dev/null | sort -t_ -k2 -n) "$FINAL"; do
  tag=$(basename "$ck" .zip)
  [ -f "baselines/curve_${tag}.log" ] && continue
  .venv-eval/bin/python -u baselines/rl/eval_sac.py "$ck" --random 10 --seed 0 --max-steps 400 \
      > baselines/curve_${tag}.log 2>&1 &
  i=$((i+1)); [ $((i % 3)) -eq 0 ] && wait
done
wait
echo "--- curve (steps -> picked rate) ---" >> $LOG
for f in baselines/curve_sacfd_*.log; do
  tag=$(basename "$f" .log | sed 's/curve_sacfd_//; s/_steps//')
  rate=$(grep -oE "picked [0-9.]+" "$f" | tail -1 | awk '{print $2}')
  echo "$tag $rate" >> $LOG
done

# B. final comparable numbers (full horizon, 3 seeds parallel)
echo "[$(date)] B: final eval (1200-step horizon, 3 seeds x 15 ICs)" >> $LOG
for s in 0 1 2; do
  .venv-eval/bin/python -u baselines/rl/eval_sac.py "$FINAL" --random 15 --seed $s \
      > baselines/sacfd_eval_s${s}.log 2>&1 &
done
wait
echo "--- final per-seed ---" >> $LOG
grep -h -E "rollouts:|funnel" baselines/sacfd_eval_s*.log >> $LOG 2>/dev/null
echo "[$(date)] SACFD EVAL DONE (curve above answers trained-enough)" >> $LOG
