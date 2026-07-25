#!/bin/bash
# Image-DP legs on the pruned pick set (67 demos w/ (64,64,6) rig obs).
#   run_dp_img.sh 2   -> top+wrist (dataset lerobot_dataset_img2, already converted)
#   run_dp_img.sh 1   -> top only  (converts lerobot_dataset_img1 first)
# 100k steps for comparability with the state-only legs; champion gets extended later.
set -e
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
N=${1:-2}
if [ "$N" = "1" ]; then DS=baselines/lerobot_dataset_img1; CAMS=top; else DS=baselines/lerobot_dataset_img2; CAMS=top,wrist; fi
RUN=dp_img${N}_$(date +%m%d_%H%M)
CHAIN=baselines/run_dp_img${N}_chain.log
echo "[$(date)] dp_img${N} START" > $CHAIN

if [ ! -d "$DS/genesis_pickaplace" ]; then
  $LV/python baselines/convert_to_lerobot.py \
    baselines/episodes_pick_pruned_img $DS/genesis_pickaplace 8 4 $CAMS video \
    > baselines/convert_img${N}.log 2>&1
  echo "[$(date)] CONVERT exit $?" >> $CHAIN
fi

rm -rf baselines/outputs/dp_img${N}
$LV/lerobot-train \
  --dataset.repo_id=local/genesis_pickaplace_img${N} \
  --dataset.root=$DS/genesis_pickaplace \
  --policy.type=diffusion --policy.push_to_hub=false \
  --output_dir=baselines/outputs/dp_img${N} --batch_size=64 --steps=100000 \
  --job_name=$RUN \
  --wandb.enable=true --wandb.project=genesis_pickaplace --wandb.disable_artifact=true \
  > baselines/train_img${N}.log 2>&1
echo "[$(date)] TRAIN exit $?" >> $CHAIN

bash baselines/dp_ckpt_curve.sh baselines/outputs/dp_img${N} $RUN 8 1200 >> baselines/ckpt_curve_chain.log 2>&1
echo "[$(date)] CKPT-CURVE exit $?" >> $CHAIN

$LV/python baselines/wandb_eval.py --kind dp \
  --checkpoint baselines/outputs/dp_img${N}/checkpoints/last/pretrained_model \
  --random 15 --seed 0 --group $RUN --name $RUN-eval \
  > baselines/eval_img${N}_wandb.log 2>&1
echo "[$(date)] EVAL exit $?" >> $CHAIN
