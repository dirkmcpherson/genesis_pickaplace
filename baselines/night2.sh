#!/bin/bash
# NIGHT2 (user: "rerun all demos with that position, then report back. If you don't get a
# response from me just continue and train a DP and a SACfD on the new dataset").
# Goal = human-validated SOUTH (0.672,-0.221); nested = proximity-touch (NESTED_TOUCH_DIST).
#   1. re-collect episodes_all (91, keep-everything + classification) at the new goal
#   2. finalize success-only pick subset -> episodes_pick
#   3. GPU (sequential, shares with dreamer4): SACfD 200k on everything -> eval;
#      then DP 100k on successful picks -> eval; videos for both
#   4. CPU (parallel with GPU): honest remeasure (success + fail neg-control) + re-render
#      gallery & real|sim side-by-sides
#   5. NIGHT2_REPORT.txt
cd /home/j/workspace/genesis_pickaplace
PY=.venv-eval/bin/python
LOG=baselines/night2.log
SP=/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad
echo "[$(date)] NIGHT2 START (goal 0.672,-0.221)" > $LOG

# ---- 0. archive old-goal datasets ----
[ -d baselines/episodes_all ] && mv baselines/episodes_all baselines/episodes_all_goal662
[ -d baselines/episodes_pick ] && mv baselines/episodes_pick baselines/episodes_pick_goal662
mkdir -p baselines/episodes_all

# ---- 1. re-collect all 91 (4 parallel procs) ----
$PY - <<'PYEOF'
import json, pathlib
t = json.loads(pathlib.Path("can_pos_recovery/trial_placements.json").read_text())['trials']
uids = sorted(int(u) for u in t if u not in ('290', '322'))
SP = "/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad"
ch = [[] for _ in range(4)]
for i, u in enumerate(uids): ch[i % 4].append(u)
for i, c in enumerate(ch):
    pathlib.Path(f"{SP}/n2_chunk{i}.txt").write_text(" ".join(map(str, c)))
PYEOF
for i in 0 1 2 3; do
  $PY -u baselines/collect_all_classified.py --outdir baselines/episodes_all \
      --uids $(cat $SP/n2_chunk${i}.txt) > $SP/n2_collect_p${i}.log 2>&1 &
done
wait
echo "[$(date)] 1. collection done: $(ls baselines/episodes_all/*.npz | wc -l)/91" >> $LOG

# ---- 2. pick subset (success-only, solved) ----
$PY baselines/finalize_pick_dataset.py >> $LOG 2>&1

# ---- 4a. CPU branch in background (parallel with GPU trainings) ----
(
  timeout 5000 $PY -u can_pos_recovery/remeasure_contact.py --label success \
      > $SP/n2_remeasure_succ.log 2>&1
  timeout 3000 $PY -u can_pos_recovery/remeasure_contact.py --label fail \
      > $SP/n2_remeasure_fail.log 2>&1
  echo "[$(date)] 4a. remeasure done" >> $LOG
  bash $SP/rebuild_videos.sh > /dev/null 2>&1
  echo "[$(date)] 4b. gallery + side-by-sides re-rendered" >> $LOG
) &
CPU_PID=$!

# ---- 3a. SACfD 200k on everything (lesson: stop at 200k, more collapses) ----
$PY -u baselines/rl/train_sacfd.py --full --steps 200000 --demo-dir baselines/episodes_all \
    --device cuda --out-dir baselines/rl/checkpoints/sacfd_all_v2 \
    > $SP/n2_sacfd.log 2>&1
echo "[$(date)] 3a. SACfD train done (exit $?)" >> $LOG
CK=baselines/rl/checkpoints/sacfd_all_v2/sacfd_final.zip
for s in 0 1 2; do
  $PY -u baselines/rl/eval_sac.py "$CK" --random 15 --seed $s > baselines/n2_sac_eval_s${s}.log 2>&1 &
done
wait
$PY -u baselines/rl/eval_sac.py "$CK" --random 6 --seed 1 --max-steps 600 \
    --record-dir baselines/policy_videos/sacfd_all_v2 > /dev/null 2>&1
echo "[$(date)] 3a. SACfD eval + videos done" >> $LOG

# ---- 3b. DP 100k on successful picks ----
LV=/home/j/workspace/genesis_pickaplace/.venv-eval/bin
$LV/python baselines/convert_to_lerobot.py baselines/episodes_pick \
    baselines/lerobot_dataset_pick_v2/genesis_pickaplace 8 4 > baselines/convert_pick_v2.log 2>&1
rm -rf baselines/outputs/dp_pick_v2
$LV/lerobot-train --dataset.repo_id=local/genesis_pickaplace_pick_v2 \
    --dataset.root=baselines/lerobot_dataset_pick_v2/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_pick_v2 --batch_size=64 --steps=100000 \
    --wandb.enable=false > baselines/train_pick_v2.log 2>&1
echo "[$(date)] 3b. DP train done (exit $?)" >> $LOG
DCK=baselines/outputs/dp_pick_v2/checkpoints/last/pretrained_model
[ -d "$DCK" ] || DCK=$(ls -d baselines/outputs/dp_pick_v2/checkpoints/*/pretrained_model 2>/dev/null | sort | tail -1)
for s in 0 1 2; do
  $PY -u baselines/eval_policy.py "$DCK" --random 15 --seed $s > baselines/n2_dp_eval_s${s}.log 2>&1 &
done
wait
$PY -u baselines/render_policy_rollout.py "$DCK" 232 235 251 265 > /dev/null 2>&1
echo "[$(date)] 3b. DP eval + videos done" >> $LOG

wait $CPU_PID

# ---- 5. report ----
{
echo "===== NIGHT2 REPORT $(date) — goal (0.672,-0.221), proximity nested ====="
echo ""
echo "--- replay stage stats (all 91, new goal) ---"
$PY - <<'PYEOF'
import json, glob, pathlib
from collections import Counter
man = {}
for sh in glob.glob('baselines/episodes_all/_stage_*.json'):
    man.update(json.loads(pathlib.Path(sh).read_text()))
t = json.loads(pathlib.Path('can_pos_recovery/trial_placements.json').read_text())['trials']
for lab in ('success', 'fail'):
    rows = [v for k, v in man.items() if t[str(k)].get('label') == lab]
    print(f"{lab.upper()} ({len(rows)}): {dict(Counter(r['stage'] for r in rows))}")
PYEOF
echo ""
echo "--- honest remeasure funnel (settle + proximity nested) ---"
grep -A2 "===" /tmp/claude-1000/-home-james-workspace-genesis-pickaplace/5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad/n2_remeasure_succ.log 2>/dev/null | tail -3
echo "fail neg-control:"
grep -A2 "===" /tmp/claude-1000/-home-james-workspace-genesis-pickaplace/5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad/n2_remeasure_fail.log 2>/dev/null | tail -3
echo ""
echo "--- SACfD-v2 (200k, everything) random ICs ---"
grep -h -E "rollouts:|funnel" baselines/n2_sac_eval_s*.log 2>/dev/null
echo "(old-goal SACfD: picked 0.47 @200k, 0.67 @325k-peak)"
echo ""
echo "--- DP-pick-v2 (100k, successful picks) random ICs ---"
grep -h -E "\[dp\].*rollouts:|\[dp\] funnel" baselines/n2_dp_eval_s*.log 2>/dev/null
echo "(old-goal DP-pick: picked 0.18)"
echo ""
echo "videos: policy_videos/sacfd_all_v2/, policy_videos/ (dp uids), videos_realsim/ (replays)"
} > baselines/NIGHT2_REPORT.txt
echo "[$(date)] NIGHT2 DONE -> baselines/NIGHT2_REPORT.txt" >> $LOG
