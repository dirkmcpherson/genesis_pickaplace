#!/bin/bash
# DAY RUN ("go"): two GPU branches in parallel + BC eval, all with post-evals.
#   A: warm-start full-task SACfD (from the 0.47 pick policy) 300k -> eval + videos
#   B: v6 contact-stage DP (33 corrected-goal contact demos) 100k -> eval + videos
#   C: honest eval of the existing BC checkpoint (v4-era; old-goal obs caveat)
# -> baselines/DAY_REPORT.txt
cd /home/j/workspace/genesis_pickaplace
PY=.venv-eval/bin/python
LOG=baselines/day.log
echo "[$(date)] DAY RUN START" > $LOG

# ---- branch A: warm-start full-task SACfD ----
(
  $PY -u baselines/rl/train_sacfd_full.py --steps 300000 --demo-dir baselines/episodes_all \
      --out-dir baselines/rl/checkpoints/sacfd_full_ws --device cuda \
      --warm-start baselines/rl/checkpoints/sacfd_all_resume/sacfd_325000_steps.zip \
      > baselines/sacfd_full_ws.log 2>&1
  echo "[$(date)] A: warm-start train done (exit $?)" >> $LOG
  CK=baselines/rl/checkpoints/sacfd_full_ws/sacfd_final.zip
  for s in 0 1 2; do
    $PY -u baselines/rl/eval_sac.py "$CK" --random 15 --seed $s \
        > baselines/ws_eval_s${s}.log 2>&1 &
  done
  wait
  $PY -u baselines/rl/eval_sac.py "$CK" --random 6 --seed 1 --max-steps 900 \
      --record-dir baselines/policy_videos/sacfd_full_ws > /dev/null 2>&1
  echo "[$(date)] A: eval + videos done" >> $LOG
) &
A_PID=$!

# ---- branch B: v6 contact-stage DP ----
(
  bash baselines/run_v6_train.sh
  echo "[$(date)] B: v6 DP train done" >> $LOG
  CK=baselines/outputs/dp_v6/checkpoints/last/pretrained_model
  [ -d "$CK" ] || CK=$(ls -d baselines/outputs/dp_v6/checkpoints/*/pretrained_model 2>/dev/null | sort | tail -1)
  for s in 0 1 2; do
    $PY -u baselines/eval_policy.py "$CK" --random 15 --seed $s \
        > baselines/v6_eval_s${s}.log 2>&1 &
  done
  wait
  $PY -u baselines/render_policy_rollout.py "$CK" 232 235 251 265 > /dev/null 2>&1
  echo "[$(date)] B: eval + videos done" >> $LOG
) &
B_PID=$!

# ---- branch C: BC honest eval (fast; v4-era ckpt -> old-goal obs caveat) ----
(
  for s in 0 1 2; do
    $PY -u baselines/bc/eval_bc.py baselines/bc/bc_human_v4_s0.pt --random 15 --seed $s \
        > baselines/bc_eval_s${s}.log 2>&1
  done
  echo "[$(date)] C: BC eval done" >> $LOG
) &
C_PID=$!

wait $A_PID $B_PID $C_PID

# ---- report ----
{
echo "===== DAY REPORT $(date) ====="
echo ""
echo "--- A: WARM-START full-task SACfD (from 0.47 pick policy, 300k) ---"
grep -h -E "rollouts:|funnel" baselines/ws_eval_s*.log 2>/dev/null
echo "(from-scratch full-task was: picked 0.09 contact 0.00 | pick-only champion: 0.47)"
echo ""
echo "--- B: v6 contact-stage DP (33 corrected-goal contact demos, 100k) ---"
grep -h -E "\[dp\].*rollouts:|\[dp\] funnel" baselines/v6_eval_s*.log 2>/dev/null
echo "(DP-pick was: picked 0.18 placed 0.11 contact 0.09 nested 0.02)"
echo ""
echo "--- C: BC v4 checkpoint (CAVEAT: trained w/ old-goal obs) ---"
grep -h -E "rollouts:|funnel" baselines/bc_eval_s*.log 2>/dev/null
echo ""
echo "--- training tails ---"
echo "[A tail]"; grep -E "ep_rew_mean" baselines/sacfd_full_ws.log 2>/dev/null | tail -3
echo "[B tail]"; tail -c 200 baselines/train_v6.log 2>/dev/null | tr '\r' '\n' | grep -E "Training:" | tail -1
echo ""
echo "videos: policy_videos/sacfd_full_ws/ + policy_videos/ (dp_v6 uids 232/235/251/265)"
} > baselines/DAY_REPORT.txt
echo "[$(date)] DAY RUN DONE -> baselines/DAY_REPORT.txt" >> $LOG
