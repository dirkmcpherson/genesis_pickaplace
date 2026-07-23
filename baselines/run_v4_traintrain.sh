#!/bin/bash
# Detached v4 convert+train (collection is complete: 34 eps; the 4 remaining are chronic
# skips that also failed in v3). setsid+nohup so it survives agent-session teardown.
cd /home/j/workspace/genesis_pickaplace
LEROBOT=/home/j/workspace/genesis_pickaplace/.venv-eval
echo "[$(date)] v4 convert start" >> baselines/run_v4_chain.log
rm -rf baselines/lerobot_dataset_v4
$LEROBOT/bin/python baselines/convert_to_lerobot.py baselines/episodes_raw_v4 \
    baselines/lerobot_dataset_v4/genesis_pickaplace 8 > baselines/convert_v4.log 2>&1
if [ ! -f baselines/lerobot_dataset_v4/genesis_pickaplace/meta/info.json ]; then
    echo "[$(date)] v4 CONVERT FAILED" >> baselines/run_v4_chain.log
    exit 1
fi
echo "[$(date)] v4 convert done" >> baselines/run_v4_chain.log
$LEROBOT/bin/lerobot-train --dataset.repo_id=local/genesis_pickaplace_v4 \
    --dataset.root=baselines/lerobot_dataset_v4/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_v4 --batch_size=64 --steps=100000 \
    --wandb.enable=false > baselines/train_v4.log 2>&1
echo "[$(date)] v4 TRAIN DONE (exit $?)" >> baselines/run_v4_chain.log
