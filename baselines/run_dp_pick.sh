#!/bin/bash
# DP on the SUCCESSFUL-PICKS subset (demos that reached >= picked, corrected goal).
# Prereq: collect_all_classified.py done + finalize_pick_dataset.py built episodes_pick/.
# GPU-HELD: do NOT run while dreamer4_hansen owns the GPU. Fire when the GPU is free.
cd /home/james/workspace/genesis_pickaplace
LV=/home/james/workspace/lerobot/.venv/bin
CHAIN=baselines/run_dp_pick_chain.log
echo "[$(date)] DP-pick pipeline START" > $CHAIN

# Stage 2a: pack the successful-pick npz -> LeRobotDataset (PROPRIO=8, keep all lengths).
$LV/python baselines/convert_to_lerobot.py \
    baselines/episodes_pick baselines/lerobot_dataset_pick/genesis_pickaplace 8 4 \
    > baselines/convert_pick.log 2>&1
echo "[$(date)] pick CONVERT exit $?" >> $CHAIN
tail -3 baselines/convert_pick.log >> $CHAIN

# Stage 2b: train state-only Diffusion Policy (GPU).
rm -rf baselines/outputs/dp_pick
$LV/lerobot-train \
    --dataset.repo_id=local/genesis_pickaplace_pick \
    --dataset.root=baselines/lerobot_dataset_pick/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_pick --batch_size=64 --steps=100000 \
    --wandb.enable=false > baselines/train_pick.log 2>&1
echo "[$(date)] pick TRAIN exit $?" >> $CHAIN
