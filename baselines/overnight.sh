#!/bin/bash
# OVERNIGHT (user: "run alone all night"):
#   1. resume pick-SACfD +150k from sacfd_all/sacfd_final (GPU ~1.5h)
#   2. eval resumed: checkpoint curve + 3-seed final @1200 (CPU)
#   3. train FULL-TASK SACfD 400k on all 91 relabeled demos (GPU ~4h)
#   4. eval full-task: 3-seed final @1200 + 6 rollout videos
#   5. write morning summary to baselines/MORNING_REPORT.txt
cd /home/james/workspace/genesis_pickaplace
PY=.venv-eval/bin/python
LOG=baselines/overnight.log
R=baselines/MORNING_REPORT.txt
echo "[$(date)] OVERNIGHT START" > $LOG

# ---- 1. resume pick-SACfD ----
$PY -u baselines/rl/resume_sacfd.py --ckpt baselines/rl/checkpoints/sacfd_all/sacfd_final.zip \
    --steps 150000 --demo-dir baselines/episodes_all \
    --out-dir baselines/rl/checkpoints/sacfd_all_resume --device cuda \
    > baselines/resume.log 2>&1
echo "[$(date)] 1. resume done (exit $?)" >> $LOG

# ---- 2. eval resumed ----
CK=baselines/rl/checkpoints/sacfd_all_resume/sacfd_final.zip
i=0
for ck in $(ls baselines/rl/checkpoints/sacfd_all_resume/sacfd_*_steps.zip 2>/dev/null | sort -t_ -k2 -n); do
  tag=$(basename "$ck" .zip)
  $PY -u baselines/rl/eval_sac.py "$ck" --random 10 --seed 0 --max-steps 400 \
      > baselines/curve_resume_${tag}.log 2>&1 &
  i=$((i+1)); [ $((i % 3)) -eq 0 ] && wait
done
wait
for s in 0 1 2; do
  $PY -u baselines/rl/eval_sac.py "$CK" --random 15 --seed $s \
      > baselines/resume_eval_s${s}.log 2>&1 &
done
wait
echo "[$(date)] 2. resumed eval done" >> $LOG

# ---- 3. full-task SACfD ----
$PY -u baselines/rl/train_sacfd_full.py --steps 400000 --demo-dir baselines/episodes_all \
    --out-dir baselines/rl/checkpoints/sacfd_full --device cuda \
    > baselines/sacfd_full.log 2>&1
echo "[$(date)] 3. full-task train done (exit $?)" >> $LOG

# ---- 4. eval full-task ----
CKF=baselines/rl/checkpoints/sacfd_full/sacfd_final.zip
for s in 0 1 2; do
  $PY -u baselines/rl/eval_sac.py "$CKF" --random 15 --seed $s \
      > baselines/full_eval_s${s}.log 2>&1 &
done
wait
$PY -u baselines/rl/eval_sac.py "$CKF" --random 6 --seed 1 --max-steps 900 \
    --record-dir baselines/policy_videos/sacfd_full > baselines/full_videos.log 2>&1
echo "[$(date)] 4. full-task eval + videos done" >> $LOG

# ---- 5. morning report ----
{
echo "===== MORNING REPORT $(date) ====="
echo ""
echo "--- resumed pick-SACfD (200k->350k) curve (10 ICs, 400-step) ---"
for f in baselines/curve_resume_*.log; do
  tag=$(basename "$f" .log | sed 's/curve_resume_sacfd_//; s/_steps//')
  echo "$tag $(grep -oE 'picked [0-9.]+' "$f" | tail -1 | awk '{print $2}')"
done
echo ""
echo "--- resumed pick-SACfD final (3 seeds x 15 ICs @1200) ---"
grep -h -E "rollouts:|funnel" baselines/resume_eval_s*.log
echo "(prior 200k: picked 0.40/0.53/0.47 mean 0.47 | DP-pick: 0.18)"
echo ""
echo "--- FULL-TASK SACfD final (3 seeds x 15 ICs @1200) ---"
grep -h -E "rollouts:|funnel" baselines/full_eval_s*.log
echo "(DP-pick full funnel for reference: picked 0.18 placed 0.11 contact 0.09 nested 0.02)"
echo ""
echo "--- full-task training reward tail ---"
grep -E "ep_rew_mean" baselines/sacfd_full.log | tail -5
echo ""
echo "videos: baselines/policy_videos/sacfd_full/"
} > $R
echo "[$(date)] OVERNIGHT DONE -> $R" >> $LOG
