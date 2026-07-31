#!/bin/bash
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/dp_cart_delta_s1
$LV/lerobot-train   --dataset.repo_id=local/cart_delta   --dataset.root=baselines/lerobot_dataset_cart_delta/genesis_pickaplace   --policy.type=diffusion --policy.push_to_hub=false --seed=1   --output_dir=baselines/outputs/dp_cart_delta_s1 --batch_size=64 --steps=100000   --job_name=dp_cart_delta_s1   --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true   > baselines/train_dp_delta_s1.log 2>&1
echo "TRAIN exit $?"
$LV/python baselines/wandb_eval.py --kind dp --cartesian --control delta   --checkpoint baselines/outputs/dp_cart_delta_s1/checkpoints/last/pretrained_model   --random 15 --seed 0 --group dp_cart --name dp_cart_delta_s1-eval   > baselines/eval_dp_delta_s1.log 2>&1
echo "EVAL exit $?"
