#!/bin/bash
# v6 = corrected-goal (#27) human demo dataset. Mirrors run_v4_train.sh but adds the
# convert step and points at episodes_raw_v6. Run AFTER v6 collection completes.
cd /home/j/workspace/genesis_pickaplace
LV=/home/j/workspace/genesis_pickaplace/.venv-eval/bin
CHAIN=baselines/run_v6_chain.log
echo "[$(date)] v6 pipeline START" > $CHAIN

# Stage 2a: pack npz -> LeRobotDataset (lerobot venv). PROPRIO=8 (17-dim state - 9 world dims).
# MIN_FRAMES=4 keeps everything (demos are contact-truncated but long).
$LV/python baselines/convert_to_lerobot.py \
    baselines/episodes_raw_v6 baselines/lerobot_dataset_v6/genesis_pickaplace 8 4 \
    > baselines/convert_v6.log 2>&1
echo "[$(date)] v6 CONVERT exit $?" >> $CHAIN
tail -3 baselines/convert_v6.log >> $CHAIN

# Stage 2b: train state-only Diffusion Policy (lerobot venv, GPU).
rm -rf baselines/outputs/dp_v6
$LV/lerobot-train \
    --dataset.repo_id=local/genesis_pickaplace_v6 \
    --dataset.root=baselines/lerobot_dataset_v6/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_v6 --batch_size=64 --steps=100000 \
    --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true > baselines/train_v6.log 2>&1
echo "[$(date)] v6 TRAIN exit $?" >> $CHAIN
