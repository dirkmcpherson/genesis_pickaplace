#!/bin/bash
cd /home/j/workspace/genesis_pickaplace
rm -rf baselines/outputs/dp_v4
/home/j/workspace/genesis_pickaplace/.venv-eval/bin/lerobot-train \
    --dataset.repo_id=local/genesis_pickaplace_v4 \
    --dataset.root=baselines/lerobot_dataset_v4/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_v4 --batch_size=64 --steps=100000 \
    --wandb.enable=false > baselines/train_v4.log 2>&1
echo "[$(date)] v4 TRAIN DONE (exit $?)" >> baselines/run_v4_chain.log
