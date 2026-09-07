#!/bin/bash
# Robomimic leg: install the r2dreamer-side files into the COPY $LAB/robomimic_r2d (never the Genesis tree):
# envs/robosuite.py, configs/env/robosuite_can_state.yaml, eval_robosuite.py, and the two guarded edits
# (demo_prefill.py dims + robosuite log keys; envs/__init__.py suite branch) via apply_patches.py. Idempotent.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
R2D=${ROBO_R2D:-$LAB/robomimic_r2d}
SRC=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace}/baselines/robomimic/r2d
[ -d "$R2D/envs" ] || { echo "FATAL: $R2D is not an r2dreamer tree (run copy_r2d_tree.sh first)"; exit 1; }
cp "$SRC/robosuite.py" "$R2D/envs/robosuite.py"
cp "$SRC/robosuite_can_state.yaml" "$R2D/configs/env/robosuite_can_state.yaml"
cp "$SRC/eval_robosuite.py" "$R2D/eval_robosuite.py"
$LAB/r2d_venv_robo/bin/python "$SRC/apply_patches.py" "$R2D"
echo "== installed into $R2D: $(ls $R2D/envs/robosuite.py $R2D/configs/env/robosuite_can_state.yaml $R2D/eval_robosuite.py | tr '\n' ' ')"
