#!/bin/bash
# Post-train eval + video for the DP-successful-picks policy. Detached; waits for training,
# then evals 3 seeds IN PARALLEL (random ICs, corrected goal, standard 1200-step horizon so
# it's comparable to prior/other-policy numbers) and renders rollout videos. #21 throughput:
# parallel seeds give ~3x wall-clock without changing the metric.
cd /home/james/workspace/genesis_pickaplace
LOG=baselines/pick_eval.log
echo "[$(date)] PICK EVAL: waiting for training to finish..." > $LOG
while [ "$(ps aux | grep lerobot-train | grep -v grep | wc -l)" -gt 0 ]; do sleep 300; done
echo "[$(date)] training finished" >> $LOG

# locate the final checkpoint (lerobot: outputs/dp_pick/checkpoints/{last,NNNNNN}/pretrained_model)
CK=baselines/outputs/dp_pick/checkpoints/last/pretrained_model
[ -d "$CK" ] || CK=$(ls -d baselines/outputs/dp_pick/checkpoints/*/pretrained_model 2>/dev/null | sort | tail -1)
echo "[$(date)] checkpoint: $CK" >> $LOG
if [ -z "$CK" ] || [ ! -d "$CK" ]; then echo "[$(date)] NO CHECKPOINT -> abort" >> $LOG; exit 1; fi

# 3 seeds in parallel (each loads its own DP on the now-free GPU; env on CPU)
for s in 0 1 2; do
  .venv-eval/bin/python -u baselines/eval_policy.py "$CK" --random 15 --seed $s --max-steps 1200 \
      > baselines/pick_eval_s${s}.log 2>&1 &
done
wait
echo "[$(date)] eval seeds done" >> $LOG
echo "--- per-seed summaries ---" >> $LOG
grep -h "\[dp\] .* rollouts:" baselines/pick_eval_s*.log >> $LOG 2>/dev/null

# rollout videos (deterministic demo ICs -> clean visual) for the standing directive
.venv-eval/bin/python -u baselines/render_policy_rollout.py "$CK" 232 235 251 265 \
    >> baselines/pick_eval.log 2>&1
echo "[$(date)] PICK EVAL DONE" >> $LOG
