#!/bin/bash
# Build the venv for the r2dreamer MANISKILL POSITIVE CONTROL. Idempotent.
#
#   bash cluster/install_r2dreamer_ms.sh [venv_path]      # default ~/r2d_ms_venv
#
# WHY A SEPARATE VENV (do not "simplify" this into ~/r2d_venv):
#   mani_skill 3.0.0b21 pins gymnasium==0.29.1 and numpy<2.0.0. The genesis
#   r2dreamer env (~/r2d_venv) is pinned at gymnasium 1.2.0 + numpy 2.4.6 --
#   the latter deliberately (genesis-world upgraded past r2dreamer's 1.26 pin
#   and tools.py carries the numpy-2 tensorboard shim). Installing mani_skill
#   into ~/r2d_venv would downgrade BOTH under the live genesis arm. The two
#   venvs are cheap; a poisoned genesis env is not (cf. the 07-30 conda-ffmpeg
#   incident). Same reasoning was applied on the dev box: r2dreamer/.venv-ms.
#
# PIP ONLY. Never conda-install into this env (conda-forge libstdc++ poisoning,
# 07-30). Conda may supply the python 3.11 INTERPRETER if none is on PATH.
#
# Two checkouts must be RSYNCED from the dev box (git clone is NOT enough):
#   * r2dreamer   -- the port (envs/maniskill.py, envs/genesis.py, demo_prefill,
#                    configs/env/*) is uncommitted upstream.
#   * ManiSkill   -- the dev box runs a FORK (3 local commits past 3.0.0b21;
#                    the shipped PickCube demos were recorded against it).
# And one data dir (rsync ONLY, never git):
#   * dreamerv3-torch/demonstrations/maniskill_teleop_sparse100  (10 episodes,
#     ~40 MB; built by ms_control.py demos on the dev box)
set -eo pipefail
cd "$(dirname "$0")/.."

VENV=${1:-$HOME/r2d_ms_venv}
_LAB=/cluster/tufts/shortlab/$USER
R2D_DIR=${R2D_DIR:-$([ -d "$_LAB/r2dreamer" ] && echo "$_LAB/r2dreamer" || echo "$HOME/r2dreamer")}
MS_DIR=${MS_DIR:-$([ -d "$_LAB/ManiSkill" ] && echo "$_LAB/ManiSkill" || echo "$HOME/ManiSkill")}

[ -f "$R2D_DIR/envs/maniskill.py" ] || {
  echo "FATAL: no r2dreamer ManiSkill port at $R2D_DIR/envs/maniskill.py"
  echo "  rsync -av --exclude runs/ --exclude '.venv*' --exclude wandb/ \\"
  echo "    <devbox>:~/workspace/r2dreamer/ $R2D_DIR/"
  exit 1; }
[ -f "$MS_DIR/setup.py" ] || {
  echo "FATAL: ManiSkill checkout missing at $MS_DIR"
  echo "  rsync -av --exclude '.git/' <devbox>:~/workspace/ManiSkill/ $MS_DIR/"
  echo "  (the dev box runs a FORK; PyPI mani_skill==3.0.0b21 is NOT identical)"
  exit 1; }

# --- python 3.11 interpreter -> venv (idempotent) ---------------------------------
if [ -x "$VENV/bin/python" ]; then
  echo "== reusing existing env at $VENV"
elif command -v python3.11 >/dev/null 2>&1; then
  python3.11 -m venv "$VENV"
else
  echo "== no python3.11 on PATH -> conda for the INTERPRETER ONLY (pip-only inside)"
  module load anaconda/2025.06.0
  conda create -y -p "$VENV" python=3.11
fi
PY="$VENV/bin/python"
PYV=$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
[ "$PYV" = "3.11" ] || { echo "FATAL: $VENV has python $PYV, need 3.11"; exit 1; }

# --- pip-only installs (order matters) --------------------------------------------
# 1. r2dreamer's own pinned stack, torch FIRST from the cu126 index (dev-box parity).
"$PY" -m pip install -q --upgrade pip
"$PY" -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cu126
"$PY" -m pip install torchrl==0.9.2 tensordict==0.9.1 ruamel.yaml==0.17.4 \
    moviepy==1.0.3 einops==0.3.0 hydra-core==1.3.2 "tensorboard>=2.20,<3" wandb
# 2. ManiSkill's pins. These are the ONLY two deviations from r2dreamer's
#    pyproject (which asks for gymnasium 1.2.0 / numpy 1.26.0): gymnasium
#    0.29.1 and numpy 1.26.4. r2dreamer touches gymnasium only for Env/Wrapper/
#    spaces, all of which are identical in 0.29 -- verified by the gates.
"$PY" -m pip install numpy==1.26.4 gymnasium==0.29.1 sapien==3.0.2 scipy dacite h5py \
    pyyaml tqdm GitPython tabulate transforms3d trimesh "imageio[ffmpeg]" mplib==0.1.1 \
    fast_kinematics==0.2.2 IPython pytorch_kinematics==0.7.5 pynvml "tyro>=0.8.5" huggingface_hub
# 3. sapien 3.0.2 imports pkg_resources -> setuptools must stay <81.
#    (r2dreamer's own pin; setuptools 84 removed pkg_resources and sapien
#    fails to import. Cost 10 min on the dev box.)
"$PY" -m pip install "setuptools==77.0.3"
# 4. mani_skill from the LOCAL FORK, --no-deps so nothing above is re-resolved.
"$PY" -m pip install -e "$MS_DIR" --no-deps --no-build-isolation

# --- verify -----------------------------------------------------------------------
echo "== [1/2] imports + pins"
( cd "$R2D_DIR" && "$PY" - <<'PYEOF' ) || exit 1
import gymnasium, numpy, torch

import mani_skill
import demo_prefill, dreamer, envs, tools, trainer  # the port
import envs.maniskill
assert torch.__version__.startswith("2.8.0"), torch.__version__
assert numpy.__version__.startswith("1.26"), numpy.__version__
assert gymnasium.__version__ == "0.29.1", gymnasium.__version__
print(f"  OK torch {torch.__version__} cuda={torch.cuda.is_available()} "
      f"numpy {numpy.__version__} gymnasium {gymnasium.__version__} "
      f"mani_skill {mani_skill.__version__}")
PYEOF

echo "== [2/2] ManiSkill RENDER probe (sapien needs a working Vulkan ICD -- this is"
echo "         the most likely cluster-only failure; genesis never exercised it)"
if "$PY" -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)'; then
  ( cd "$R2D_DIR" && MUJOCO_GL=egl "$PY" -m envs.maniskill --steps 120 ) || {
    echo "  FAILED. Checklist: nvidia Vulkan ICD present (/usr/share/vulkan/icd.d/"
    echo "  nvidia_icd.json), try VK_ICD_FILENAMES=<that path>; ensure the job has a"
    echo "  GPU; sapien needs no X server but does need the driver's ICD."
    exit 1; }
  echo "== INSTALL VERIFIED: $VENV serves r2dreamer + mani_skill + GPU rendering"
else
  echo "== no GPU visible: SKIPPED the render probe. Re-run inside an allocation:"
  echo "   srun -p preempt --gres=gpu:1 --constraint=\"l40s|a100|l40|h200\" -n 8 \\"
  echo "     --mem=32g --time=0:30:00 --pty bash cluster/install_r2dreamer_ms.sh $VENV"
fi
