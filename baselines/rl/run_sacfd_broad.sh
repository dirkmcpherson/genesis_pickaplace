#!/bin/bash
cd /home/james/workspace/genesis_pickaplace
# 1. collect ALL demos (success + fail + dropped) for the RL buffer -- keep everything
.venv-eval/bin/python -u baselines/rl/collect_all_rl.py > baselines/rl/collect_all_rl.log 2>&1
echo "[$(date)] collect_all_rl done" >> baselines/rl/run_sacfd_broad.log
# 2. SACfD on the broad set (demo_buffer now prefers episodes_raw_rl)
CUDA_VISIBLE_DEVICES="" .venv-eval/bin/python -u baselines/rl/train_sacfd.py --full > baselines/rl/train_sacfd.log 2>&1
echo "[$(date)] SACfD full done (exit $?)" >> baselines/rl/run_sacfd_broad.log
