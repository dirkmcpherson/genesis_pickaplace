#!/bin/bash
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/audit_joint_rebuilt
$LV/lerobot-train \
  --dataset.repo_id=local/audit_rebuilt \
  --dataset.root=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace \
  --policy.type=diffusion --policy.push_to_hub=false --seed=0 \
  --output_dir=baselines/outputs/audit_joint_rebuilt --batch_size=64 --steps=100000 \
  --job_name=audit_joint_rebuilt \
  --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true \
  > baselines/train_audit_a1.log 2>&1
echo "TRAIN exit $?"
$LV/python baselines/wandb_eval.py --kind dp --ic-mode both \
  --checkpoint baselines/outputs/audit_joint_rebuilt/checkpoints/last/pretrained_model \
  --random 15 --seed 0 --group audit --name audit_joint_rebuilt-eval \
  > baselines/eval_audit_a1.log 2>&1
echo "EVAL exit $?"
