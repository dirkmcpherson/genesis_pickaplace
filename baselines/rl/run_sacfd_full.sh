#!/bin/bash
cd /home/james/workspace/genesis_pickaplace
CUDA_VISIBLE_DEVICES="" .venv-eval/bin/python -u baselines/rl/train_sacfd.py --full > baselines/rl/train_sacfd.log 2>&1
echo "[$(date)] SACfD full done (exit $?)" >> baselines/rl/train_sacfd.log
