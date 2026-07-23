#!/bin/bash
# DP on the SUCCESSFUL-PICKS subset (demos that reached >= picked, corrected goal).
# Prereq: collect_all_classified.py done + finalize_pick_dataset.py built episodes_pick/.
# GPU-HELD: do NOT run while dreamer4_hansen owns the GPU. Fire when the GPU is free.
cd /home/j/workspace/genesis_pickaplace
LV=/home/j/workspace/genesis_pickaplace/.venv-eval/bin
CHAIN=baselines/run_dp_pruned_chain.log
echo "[$(date)] DP-pick pipeline START" > $CHAIN

# Stage 2a: pack the successful-pick npz -> LeRobotDataset (PROPRIO=8, keep all lengths).
$LV/python baselines/convert_to_lerobot.py \
    baselines/episodes_pick_pruned baselines/lerobot_dataset_pick_pruned/genesis_pickaplace 8 4 \
    > baselines/convert_pruned.log 2>&1
echo "[$(date)] pick CONVERT exit $?" >> $CHAIN
tail -3 baselines/convert_pruned.log >> $CHAIN

# Stage 2b: train state-only Diffusion Policy (GPU).
# wandb: scalars only (disable_artifact keeps checkpoints local-only, per user).
RUN=dp_pick_pruned_$(date +%m%d_%H%M)
rm -rf baselines/outputs/dp_pick_pruned
$LV/lerobot-train \
    --dataset.repo_id=local/genesis_pickaplace_pickpr \
    --dataset.root=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_pick_pruned --batch_size=64 --steps=100000 \
    --job_name=$RUN \
    --wandb.enable=true --wandb.project=genesis_pickaplace \
    --wandb.disable_artifact=true > baselines/train_pruned.log 2>&1
echo "[$(date)] pick TRAIN exit $?" >> $CHAIN

# Stage 2b': checkpoint overfitting curve (short-horizon eval of every 20k ckpt).
# Small-data DP memorizes (loss ~0.003 by 100k); the best-generalizing checkpoint is
# often NOT the last one -- pick champions off this curve, not off 'last'.
bash baselines/dp_ckpt_curve.sh baselines/outputs/dp_pick_pruned $RUN >> baselines/ckpt_curve_chain.log 2>&1
echo "[$(date)] pick CKPT-CURVE exit $?" >> $CHAIN

# Stage 2c: random-IC video eval -> same wandb project (group ties it to the train run).
$LV/python baselines/wandb_eval.py --kind dp \
    --checkpoint baselines/outputs/dp_pick_pruned/checkpoints/last/pretrained_model \
    --random 15 --seed 0 --group $RUN --name $RUN-eval \
    > baselines/eval_pruned_wandb.log 2>&1
echo "[$(date)] pick EVAL exit $?" >> $CHAIN
tail -4 baselines/eval_pruned_wandb.log >> $CHAIN
