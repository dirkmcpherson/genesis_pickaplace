#!/bin/bash
# Build the r2dreamer environment on the cluster. Idempotent -- safe to re-run;
# every step skips or fast-forwards if already done.
#
#   bash cluster/install_r2dreamer.sh [venv_path]     # default ~/r2d_venv
#   R2D_DIR=... GENESIS_DIR=... to point at the rsynced checkouts.
#
# r2dreamer needs python 3.11 (pyproject: >=3.11,<3.12) -- the shared genesis
# conda env (/cluster/tufts/shortlab/jstale02/condaenv/genesis) is py3.10 and
# must NOT be touched; this script builds a SEPARATE env and never activates
# that one.
#
# Interpreter resolution: python3.11 on PATH (try `module avail python` if the
# cluster ships one) -> plain venv. Fallback: conda create -p <path> python=3.11
# -- the INTERPRETER ONLY. Either way, everything after creation is pip-only.
# NEVER conda-install libraries into it: conda-forge packages carry their own
# libstdc++/GLIBCXX which genesis/taichi's C++ (mesh loading) was not built
# against -- a conda ffmpeg install poisoned the py3.10 env this way on 07-30
# (segfaults at scene build that looked like hardware for a full day).
#
# Two checkouts must be RSYNCED from the dev box (git clone is NOT enough):
#   * r2dreamer: the genesis port (envs/genesis.py, demo_prefill.py,
#     eval_genesis.py, configs/env/genesis_*.yaml + edits to train/trainer/
#     buffer/dreamer/tools) is UNCOMMITTED in the local clone -- a clone of the
#     upstream repo is missing the entire port.
#   * Genesis: dev box runs 0.2.1 + 269 upstream commits + a local headless-
#     render patch (f41427d) -- NOT pip-reproducible from any index or tag.
#
# Pins that matter (mirrors the dev box .venv exactly):
#   * torch 2.8.0+cu126 from the pytorch cu126 index, installed FIRST so
#     `pip -e r2dreamer` (which pins torch==2.8.0) resolves against it instead
#     of pulling the default-index wheel.
#   * install order r2dreamer THEN Genesis: genesis-world upgrades numpy past
#     r2dreamer's 1.26 pin; final numpy pinned 2.4.6 = dev-box value (the
#     np.reshape/newshape tensorboard breakage is shimmed in-tree in tools.py).
#   * moviepy==1.0.3: genesis declares >=2.0.0 but 2.x drops moviepy.editor ->
#     torch's make_video silently writes 0-byte GIFs. Expect (and ignore) a pip
#     resolver warning for this pin.
set -eo pipefail
cd "$(dirname "$0")/.."
PICKAPLACE_ROOT="$PWD"

VENV=${1:-$HOME/r2d_venv}
R2D_DIR=${R2D_DIR:-$HOME/r2dreamer}
GENESIS_DIR=${GENESIS_DIR:-$HOME/Genesis}

# --- the two rsynced checkouts: fail early with the remedy ------------------------
[ -f "$R2D_DIR/train.py" ] || {
  echo "FATAL: r2dreamer checkout missing at $R2D_DIR"
  echo "  rsync -av --exclude runs/ --exclude .venv/ --exclude wandb/ \\"
  echo "    <devbox>:~/workspace/r2dreamer/ $R2D_DIR/"
  exit 1; }
[ -f "$R2D_DIR/envs/genesis.py" ] && [ -f "$R2D_DIR/demo_prefill.py" ] || {
  echo "FATAL: $R2D_DIR has no genesis port (envs/genesis.py / demo_prefill.py)."
  echo "  The port is UNCOMMITTED on the dev box -- git clone cannot deliver it."
  echo "  rsync the working tree from <devbox>:~/workspace/r2dreamer/ (see header)."
  exit 1; }
[ -f "$GENESIS_DIR/pyproject.toml" ] || [ -f "$GENESIS_DIR/setup.py" ] || {
  echo "FATAL: Genesis checkout missing at $GENESIS_DIR"
  echo "  Genesis 0.2.1+patches is NOT pip-reproducible (269 commits past the tag"
  echo "  + local headless patch). rsync -av <devbox>:~/workspace/Genesis/ $GENESIS_DIR/"
  exit 1; }

# --- python 3.11 interpreter -> venv (idempotent) ---------------------------------
if [ -x "$VENV/bin/python" ]; then
  echo "== reusing existing env at $VENV"
elif command -v python3.11 >/dev/null 2>&1; then
  echo "== python3.11 found on PATH ($(command -v python3.11)) -> venv"
  python3.11 -m venv "$VENV"
else
  echo "== no python3.11 on PATH (also try: module avail python 2>&1 | grep 3.11)"
  echo "== falling back to conda for the INTERPRETER ONLY (pip-only inside)"
  module load anaconda/2025.06.0
  conda create -y -p "$VENV" python=3.11
fi
PY="$VENV/bin/python"
PYV=$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
[ "$PYV" = "3.11" ] || { echo "FATAL: $VENV has python $PYV, need 3.11"; exit 1; }

# --- pip-only installs (order matters, see header) --------------------------------
"$PY" -m pip install -q --upgrade pip
"$PY" -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cu126
"$PY" -m pip install -e "$R2D_DIR"
"$PY" -m pip install -e "$GENESIS_DIR"
"$PY" -m pip install pyvista "moviepy==1.0.3" wandb imageio imageio-ffmpeg
"$PY" -m pip install "numpy==2.4.6"   # dev-box parity; last so nothing re-resolves it

# --- verify -----------------------------------------------------------------------
echo "== [1/2] imports: torch/cuda + the r2dreamer port modules"
( cd "$R2D_DIR" && "$PY" - <<'PYEOF' ) || exit 1
import torch, numpy, genesis, moviepy
import tools, buffer, dreamer, trainer, demo_prefill, envs   # the port itself
import envs.genesis
print(f"  OK torch {torch.__version__} (cuda available: {torch.cuda.is_available()})")
print(f"  OK numpy {numpy.__version__} genesis {genesis.__version__} + r2dreamer port modules")
assert torch.__version__.startswith("2.8.0"), "torch pin drifted"
assert numpy.__version__ == "2.4.6", "numpy pin drifted"
PYEOF

echo "== [2/2] genesis FULL kinova world build on GPU (verify_env.sh stage-3 mirror)"
if "$PY" -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)'; then
  export GENESIS_PICKAPLACE_ROOT="$PICKAPLACE_ROOT" MUJOCO_GL=egl PYTHONUNBUFFERED=1
  if ! "$PY" - <<'PYEOF'
import sys, os
root = os.environ["GENESIS_PICKAPLACE_ROOT"]
sys.path.insert(0, os.path.join(root, "baselines")); sys.path.insert(0, os.path.join(root, "can_pos_recovery"))
from replay_harness import build_world
w = build_world(show_viewer=False, backend="gpu", camera=False)
print("  OK full kinova world built on GPU (URDF + meshes + can + shelf)")
PYEOF
  then
    echo "  FAILED at the 07-30 crash point (mesh load, C++). If this env ever saw"
    echo "  a conda lib install, rebuild it pip-only."
    exit 1
  fi
  echo "== INSTALL VERIFIED: $VENV serves torch 2.8.0+cu126 + r2dreamer + genesis"
else
  echo "== no GPU visible: SKIPPED the world build. Re-run the verify inside an allocation:"
  echo "   srun -p preempt --gres=gpu:1 --constraint=\"l40s|a100|l40|h200\" -n 8 --mem=32g \\"
  echo "     --time=0:30:00 --pty bash cluster/install_r2dreamer.sh $VENV"
  echo "   (idempotent: installs no-op, only the verify runs)"
fi
