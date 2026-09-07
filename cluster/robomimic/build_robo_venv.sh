#!/bin/bash
# Robomimic leg (paper/ROBOMIMIC_PLAN_2026-09-05.md §4; brief 2026-09-06): build the NEW
# $LAB/robo_venv for RLPD + DP + BC-RNN on robosuite Can. Never touches condaenv/genesis,
# r2d_venv, r2d_venv_dmc, dv3_venv. Idempotent: re-running skips finished stages (stamp files).
#
#   ssh pax 'nohup bash ~/genesis_pickaplace/cluster/robomimic/build_robo_venv.sh > $LAB/robomimic_build_robo_venv.log 2>&1 &'
#
# Pins (plan §2/§4): python 3.10 (base = condaenv/three_ten 3.10.14, venv WITHOUT system site
# packages), torch 2.7.0+cu126 (+ torchvision 0.22.0, the 2.7 pairing), sb3 2.8.0 (rlpd_sac.py
# asserts 2.8.x), gymnasium, robosuite==1.5.1 (the datasets' env_version), mujoco==3.3.7 (the
# version already on r2d_venv, so both venvs simulate with the same MuJoCo), robomimic v0.5.0 from
# the GitHub tag (PyPI stops at 0.3.0), h5py, lerobot 0.4.5 = the user's fork branch genesis-fixes
# (cluster/install_lerobot.sh pins: editable, --no-deps + explicit deps, torchcodec 0.3.* for torch 2.7).
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
V=${ROBO_VENV:-$LAB/robo_venv}
BASEPY=${BASEPY:-$LAB/condaenv/three_ten/bin/python}
LEROBOT_DIR=${LEROBOT_DIR:-/cluster/home/jstale02/lerobot}   # the fork checkout condaenv/genesis already uses (branch genesis-fixes)
ROBOMIMIC_REF=${ROBOMIMIC_REF:-v0.5.0}
STAMP=$V/.build_stamps; 
step() { local name=$1; shift; if [ -f "$STAMP/$name" ]; then echo "== [$name] done earlier, skip"; return 0; fi
         echo "== [$name] $(date -Is)"; if "$@"; then mkdir -p "$STAMP"; touch "$STAMP/$name"; else echo "!! [$name] FAILED rc=$?"; return 1; fi; }
echo "# build_robo_venv $(date -Is) host=$(hostname) V=$V BASEPY=$BASEPY"
"$BASEPY" --version || { echo "FATAL: base python missing"; exit 1; }
[ -x "$V/bin/python" ] || "$BASEPY" -m venv "$V" || exit 1
PIP="$V/bin/pip"
step pip_upgrade "$PIP" install --upgrade pip wheel setuptools || exit 1
step torch "$PIP" install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu126 || exit 1
step core "$PIP" install "stable-baselines3==2.8.0" gymnasium h5py "mujoco==3.3.7" numpy scipy wandb tensorboard || exit 1
# robosuite 1.5.1 with its own deps (mink, numba, ...); mujoco stays pinned by the explicit ==3.3.7 above
step robosuite "$PIP" install "robosuite==1.5.1" "mujoco==3.3.7" || exit 1
# robomimic v0.5.0 from the GitHub tag. --no-deps: its setup.py would otherwise re-resolve torch/torchvision
# from PyPI (a different CUDA build). Runtime deps it actually imports are listed explicitly.
step robomimic "$PIP" install --no-deps "git+https://github.com/ARISE-Initiative/robomimic.git@${ROBOMIMIC_REF}" || exit 1
step robomimic_deps "$PIP" install termcolor tqdm imageio imageio-ffmpeg psutil tensorboardX matplotlib "huggingface_hub>=0.27" diffusers || exit 1
# robomimic v0.5.0 imports transformers at MODULE level (utils/lang_utils.py <- file_utils, env_robosuite); its setup.py
# pins transformers==4.41.2 / diffusers==0.11.1 / huggingface_hub==0.23.4, which lerobot 0.4.5 cannot live with -- we
# install a current transformers (only AutoModel/pipeline/AutoTokenizer/CLIPTextModelWithProjection names are imported;
# no language model is ever loaded for lang=None). Verified by the import loop below.
step robomimic_deps2 "$PIP" install transformers || exit 1
# transformers 5.x makes PretrainedConfig a dataclass, which breaks lerobot 0.4.5's groot config at import
# (`non-default argument backbone_cfg follows default argument`, seen 23:10); 4.57.x imports cleanly for both.
step transformers_lt5 "$PIP" install "transformers<5" || exit 1
robomimic_import_loop() {
  for i in 1 2 3 4 5 6; do
    MISSING=$("$V/bin/python" -c 'import robomimic.utils.file_utils, robomimic.envs.env_robosuite, robomimic.algo' 2>&1 | grep -oE "No module named '[^']+'" | grep -oE "'[^']+'" | tr -d "'" | cut -d. -f1)
    [ -z "$MISSING" ] && { echo "   robomimic import chain OK after $((i-1)) fix(es)"; return 0; }
    case "$MISSING" in cv2) PKG=opencv-python ;; yaml) PKG=pyyaml ;; PIL) PKG=pillow ;; *) PKG=$MISSING ;; esac
    echo "   missing module $MISSING -> pip install $PKG"; "$PIP" install "$PKG" || return 1
  done
  echo "!! robomimic import chain still failing"; return 1
}
step robomimic_imports robomimic_import_loop || exit 1
# lerobot (DP arm): the fork, editable, --no-deps + the explicit dep list of cluster/install_lerobot.sh
if [ ! -f "$LEROBOT_DIR/pyproject.toml" ]; then echo "FATAL: $LEROBOT_DIR has no pyproject.toml"; exit 1; fi
step lerobot "$PIP" install -e "$LEROBOT_DIR" --no-deps || exit 1
step lerobot_deps "$PIP" install "torchcodec==0.3.*" diffusers einops datasets "huggingface_hub>=0.27" jsonlines draccus av imageio deepdiff termcolor pyarrow || exit 1
# lerobot 0.4.5 also imports accelerate at package import (utils/utils.py) -- not in install_lerobot.sh's list because the
# genesis conda env already had it; first verify run here failed on it (2026-09-06 22:59).
step lerobot_deps2 "$PIP" install accelerate omegaconf hydra-core || exit 1
# lerobot's import chain pulls in hardware modules (motors_bus -> pyserial, ...). Resolve remaining import-time
# ModuleNotFoundErrors mechanically (module -> pip name map), at most 8 rounds, logging each.
lerobot_import_loop() {
  for i in 1 2 3 4 5 6 7 8; do
    MISSING=$("$V/bin/python" -c 'import lerobot.policies.diffusion.configuration_diffusion' 2>&1 | grep -oE "No module named '[^']+'" | grep -oE "'[^']+'" | tr -d "'" | cut -d. -f1)
    [ -z "$MISSING" ] && { echo "   lerobot import chain OK after $((i-1)) fix(es)"; return 0; }
    case "$MISSING" in serial) PKG=pyserial ;; cv2) PKG=opencv-python ;; yaml) PKG=pyyaml ;; PIL) PKG=pillow ;; sklearn) PKG=scikit-learn ;; *) PKG=$MISSING ;; esac
    echo "   missing module $MISSING -> pip install $PKG"; "$PIP" install "$PKG" || return 1
  done
  echo "!! lerobot import chain still failing after 8 rounds"; return 1
}
step lerobot_imports lerobot_import_loop || exit 1
"$PIP" freeze > "$V/freeze.txt"
echo "== verify"
"$V/bin/python" - <<'PY'
import torch, stable_baselines3 as sb3, gymnasium, h5py, numpy, mujoco, robosuite, robomimic
print('torch', torch.__version__, 'cuda_build', torch.version.cuda, 'sb3', sb3.__version__, 'gymnasium', gymnasium.__version__,
      'h5py', h5py.__version__, 'numpy', numpy.__version__, 'mujoco', mujoco.__version__, 'robosuite', robosuite.__version__, 'robomimic', robomimic.__version__)
assert robosuite.__version__ == '1.5.1', robosuite.__version__
assert sb3.__version__.startswith('2.8'), sb3.__version__
import lerobot
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
print('lerobot', lerobot.__version__, 'DiffusionConfig ok')
import robomimic.utils.env_utils, robomimic.utils.file_utils, robomimic.envs.env_robosuite, robomimic.algo
print('robomimic env_utils/file_utils/env_robosuite/algo import ok')
print('BUILD-OK')
PY
echo "# build_robo_venv end $(date -Is)"
