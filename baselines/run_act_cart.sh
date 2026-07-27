#!/bin/bash
# ACT leg on the cartesian gen-0 dataset (state-only, proprio 9, action 5).
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/act_cart
$LV/lerobot-train \
  --dataset.repo_id=local/cart_gen0 \
  --dataset.root=baselines/lerobot_dataset_cart_pruned/genesis_pickaplace \
  --policy.type=act --policy.push_to_hub=false \
  --output_dir=baselines/outputs/act_cart --batch_size=64 --steps=100000 \
  --job_name=act_cart_100k \
  --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true \
  > baselines/train_act_cart.log 2>&1
echo "TRAIN exit $?"
