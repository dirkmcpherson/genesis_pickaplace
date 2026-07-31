#!/bin/bash
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
rm -rf baselines/outputs/hdp_s1
$LV/lerobot-train   --dataset.repo_id=local/hdp_s1   --dataset.root=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace   --policy.type=diffusion --policy.push_to_hub=false --seed=1   --output_dir=baselines/outputs/hdp_s1 --batch_size=64 --steps=100000   --job_name=hdp_s1   --wandb.enable=true --wandb.project=genesis_paper --wandb.disable_artifact=true   > baselines/train_hdp_s1.log 2>&1
echo "TRAIN exit $?"
$LV/python baselines/wandb_eval.py --kind dp --ic-mode both   --checkpoint baselines/outputs/hdp_s1/checkpoints/last/pretrained_model   --random 15 --seed 0 --group paper_H-DP --name hdp_s1-eval   > baselines/eval_hdp_s1.log 2>&1
echo "EVAL exit $?"
