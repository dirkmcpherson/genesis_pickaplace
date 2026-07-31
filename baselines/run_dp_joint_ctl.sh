#!/bin/bash
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/dp_joint_ctl
$LV/lerobot-train \
  --dataset.repo_id=local/joint_pruned \
  --dataset.root=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace \
  --policy.type=diffusion --policy.push_to_hub=false \
  --output_dir=baselines/outputs/dp_joint_ctl --batch_size=64 --steps=100000 \
  --job_name=dp_joint_ctl_100k \
  --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true \
  > baselines/train_dp_joint_ctl.log 2>&1
echo "TRAIN exit $?"
$LV/python baselines/wandb_eval.py --kind dp \
  --checkpoint baselines/outputs/dp_joint_ctl/checkpoints/last/pretrained_model \
  --random 15 --seed 0 --group dp_cart --name dp_joint_ctl-eval \
  > baselines/eval_dp_joint_ctl.log 2>&1
echo "EVAL exit $?"
