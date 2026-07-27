#!/bin/bash
# ACT leg on the cartesian gen-0 dataset (state-only, proprio 9, action 5).
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/act_cart_delta
$LV/lerobot-train \
  --dataset.repo_id=local/cart_delta \
  --dataset.root=baselines/lerobot_dataset_cart_delta/genesis_pickaplace \
  --policy.type=act --policy.push_to_hub=false \
  --output_dir=baselines/outputs/act_cart_delta --batch_size=64 --steps=100000 \
  --job_name=act_cart_delta_100k \
  --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true \
  > baselines/train_act_cart_delta.log 2>&1
echo "TRAIN exit $?"
