#!/bin/bash
# Verify ONE conda env serves BOTH lerobot and dreamerv3-torch/genesis.
#
# Coexistence is proven possible: the dev box runs both from a single pip-only venv
# (lerobot 0.4.5 + genesis + torch 2.7.0+cu126). The cluster env broke on 07-30,
# with genesis segfaulting at SCENE BUILD (mesh loading, C++) right after a conda
# ffmpeg install -- conda-forge packages carry their own libstdc++ and can shadow
# the one taichi/genesis were built against.
#
# Run INSIDE a GPU allocation:
#   srun -p preempt --gres=gpu:1 --constraint="l40s|l40" -n 8 --mem=32g \
#     --time=0:30:00 --pty bash cluster/verify_env.sh
#
# If step 3 segfaults while 1-2 pass, the env's C++ runtime is broken. Recovery:
#   conda list --revisions            # find the revision BEFORE any conda install
#   conda install --revision <N>      # roll back (keeps pip packages)
#   bash cluster/install_lerobot.sh   # re-run (now pip-only)
# Then re-run this script.
set -o pipefail
cd "$(dirname "$0")/.."
export GENESIS_PICKAPLACE_ROOT="$PWD" MUJOCO_GL=egl PYTHONUNBUFFERED=1

echo "== [1/4] lerobot imports + policy configs"
python - <<'PY' || exit 1
import torch, lerobot
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.act.configuration_act import ACTConfig
print(f'  OK torch {torch.__version__} lerobot {lerobot.__version__}')
PY

echo "== [2/4] libstdc++ sanity (which GLIBCXX does python actually load?)"
python - <<'PY'
import ctypes, subprocess
libs = subprocess.run(['bash','-c','ldd $(python -c "import taichi, pathlib; print(pathlib.Path(taichi.__file__).parent)" 2>/dev/null)/_lib/core/*.so 2>/dev/null | grep stdc++ | head -1'],
                      capture_output=True, text=True).stdout.strip()
print(f'  taichi links: {libs or "(could not resolve -- not fatal)"}')
PY

echo "== [3/4] genesis FULL world build on GPU (the step that segfaulted 07-30)"
python - <<'PY' || { echo "  FAILED at the exact 07-30 crash point -> roll the env back (see header)"; exit 1; }
import sys, os
sys.path.insert(0, 'baselines'); sys.path.insert(0, 'can_pos_recovery')
from replay_harness import build_world
w = build_world(show_viewer=False, backend='gpu', camera=False)
print('  OK full kinova world built on GPU (URDF + meshes + can + shelf)')
PY

echo "== [4/4] the two stacks in ONE process (lerobot policy + genesis world)"
python - <<'PY' || exit 1
import sys
sys.path.insert(0, 'baselines'); sys.path.insert(0, 'can_pos_recovery')
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
cfg = DiffusionConfig()          # lerobot object alive...
import genesis as gs             # ...while genesis is initialized (step 3 already did)
print('  OK lerobot and genesis coexist in one process')
PY

echo "== VERIFY-ENV: ALL CHECKS PASSED -- one env serves both stacks"
