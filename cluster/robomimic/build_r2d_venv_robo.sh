#!/bin/bash
# Robomimic leg: overlay venv for the r2dreamer arm, built EXACTLY the way the WM-fix session built
# $LAB/wm_fix_2026-09-03/r2d_venv_dmc (pyvenv.cfg there: `r2d_venv/bin/python -m venv --system-site-packages ...`):
# python 3.11.15 + torch 2.8.0+cu126 + numpy 2.4.6 + mujoco 3.3.7 come from r2d_venv's site-packages (read-only);
# robosuite 1.5.1 / robomimic v0.5.0 / h5py are added INSIDE the overlay. r2d_venv itself is never modified.
#   ssh pax 'nohup bash ~/genesis_pickaplace/cluster/robomimic/build_r2d_venv_robo.sh > $LAB/robomimic_build_r2d_venv_robo.log 2>&1 &'
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
V=${R2D_VENV_ROBO:-$LAB/r2d_venv_robo}
BASE=$LAB/r2d_venv
ROBOMIMIC_REF=${ROBOMIMIC_REF:-v0.5.0}
STAMP=$V/.build_stamps
step() { local name=$1; shift; if [ -f "$STAMP/$name" ]; then echo "== [$name] done earlier, skip"; return 0; fi
         echo "== [$name] $(date -Is)"; if "$@"; then mkdir -p "$STAMP"; touch "$STAMP/$name"; else echo "!! [$name] FAILED rc=$?"; return 1; fi; }
echo "# build_r2d_venv_robo $(date -Is) host=$(hostname) V=$V base=$BASE"
[ -x "$V/bin/python" ] || "$BASE/bin/python" -m venv --system-site-packages "$V" || exit 1
PIP="$V/bin/pip"
# mujoco pinned to the base's 3.3.7 so no second mujoco shadows it; torch/numpy must NOT be re-resolved (robomimic --no-deps)
step robosuite "$PIP" install "robosuite==1.5.1" "mujoco==3.3.7" h5py || exit 1
step robomimic "$PIP" install --no-deps "git+https://github.com/ARISE-Initiative/robomimic.git@${ROBOMIMIC_REF}" || exit 1
step robomimic_deps "$PIP" install termcolor tqdm psutil tensorboardX || exit 1
# robomimic v0.5.0 imports transformers at module level (utils/lang_utils.py <- env_robosuite); torch is already satisfied
# by the base site-packages, so this pulls only tokenizers/safetensors/regex/etc. into the overlay.
step robomimic_deps2 "$PIP" install transformers imageio matplotlib || exit 1
# robomimic also imports torchvision (models/obs_core.py). A plain `pip install torchvision` re-resolves torch from PyPI
# INTO the overlay (seen 23:07: a 555 MB torch wheel shadowing r2d_venv's 2.8.0+cu126) -- so: the cu126 build that pairs
# with torch 2.8.0, --no-deps, and any shadow torch removed first.
# NEVER `pip uninstall` here: with --system-site-packages pip can reach into r2d_venv. Delete overlay dirs only.
# The aborted `pip install torchvision` (23:07) left torch's WHOLE dependency set in the overlay -- torch, functorch,
# triton 3.8 (base: 3.4), nvidia-*/cuda-* cu13 libs, setuptools 79 (base pin 77.0.3): triton shadowed torch 2.8's
# inductor (`triton_key` ImportError, r2d smoke 3337813). All of them go.
fix_torch_shadow() { ( cd "$V/lib/python3.11/site-packages" && rm -rf torch torch-*.dist-info torchvision torchvision-*.dist-info functorch triton triton-*.dist-info nvidia nvidia_*.dist-info cuda cuda_*.dist-info setuptools setuptools-*.dist-info pkg_resources _distutils_hack distutils-precedence.pth ); return 0; }
step torchvision_clean fix_torch_shadow || exit 1
step torchvision "$PIP" install --no-deps "torchvision==0.23.0" --index-url https://download.pytorch.org/whl/cu126 || exit 1
robomimic_import_loop() {
  for i in 1 2 3 4 5 6; do
    MISSING=$("$V/bin/python" -c 'import robomimic.utils.file_utils, robomimic.envs.env_robosuite' 2>&1 | grep -oE "No module named '[^']+'" | grep -oE "'[^']+'" | tr -d "'" | cut -d. -f1)
    [ -z "$MISSING" ] && { echo "   robomimic import chain OK after $((i-1)) fix(es)"; return 0; }
    case "$MISSING" in cv2) PKG=opencv-python ;; yaml) PKG=pyyaml ;; PIL) PKG=pillow ;; torch|torchvision) echo "!! torch stack missing in overlay ($MISSING) -- refuse to pip it (would shadow r2d_venv)"; return 1 ;; *) PKG=$MISSING ;; esac
    echo "   missing module $MISSING -> pip install --no-deps $PKG"; "$PIP" install --no-deps "$PKG" || return 1
  done
  echo "!! robomimic import chain still failing"; return 1
}
step robomimic_imports robomimic_import_loop || exit 1
"$PIP" freeze > "$V/freeze.txt"
echo "== overlay site-packages (anything here shadows r2d_venv):"; ls "$V/lib/python3.11/site-packages" | grep -v -E "dist-info|__pycache__" | tr '\n' ' '; echo
echo "== verify"
"$V/bin/python" - <<'PY'
import sys, torch, numpy, mujoco, robosuite, robomimic, h5py, gymnasium, tensordict, torchrl
print('python', sys.version.split()[0], 'torch', torch.__version__, 'numpy', numpy.__version__, 'mujoco', mujoco.__version__,
      'robosuite', robosuite.__version__, 'robomimic', robomimic.__version__, 'h5py', h5py.__version__, 'gymnasium', gymnasium.__version__,
      'tensordict', tensordict.__version__, 'torchrl', torchrl.__version__)
print('torch from', torch.__file__); print('numpy from', numpy.__file__)
assert robosuite.__version__ == '1.5.1'
assert torch.__version__.startswith('2.8.0'), torch.__version__   # the base r2d_venv build, not a PyPI re-resolve
import robomimic.utils.env_utils, robomimic.envs.env_robosuite
import triton, setuptools; assert triton.__version__.startswith('3.4'), triton.__version__   # the base's triton (torch 2.8 inductor)
import torch._inductor.codecache; from triton.compiler.compiler import triton_key
print('inductor+triton', triton.__version__, 'setuptools', setuptools.__version__)
print('BUILD-OK')
PY
echo "# build_r2d_venv_robo end $(date -Is)"
