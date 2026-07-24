#!/bin/bash
# Make a GENESIS environment able to run DreamerV3 (branch: genesis).
# Rationale: genesis-world 0.2.1 + taichi + the physics patch are fragile and
# NOT pip-reproducible; dreamer is torch-tolerant (pins 2.4.1 but runs on 2.7).
# So we add dreamer's deps INTO the genesis env, never the reverse.
#
# Run with the cluster's GENESIS python/venv active (or pass it as $1).
#   ./cluster_dv3_setup.sh [/path/to/genesis/venv/bin/python]
set -e
PY=${1:-python}
echo "== target interpreter: $($PY -c 'import sys;print(sys.executable)')"

echo "== 1. genesis must already import here (this env's whole reason to exist)"
if ! $PY -c "import genesis" 2>/dev/null; then
  echo "  genesis MISSING -> run: ./third_party/install_genesis.sh  (upstream@31951c3f + patch)"
  echo "  then pin torch==2.7.0+cu126 taichi==1.7.4 numpy==2.2.6 for physics parity"
else
  $PY -c "import genesis; print('  genesis OK:', genesis.__file__)"
fi
$PY -c "import torch; print('  torch', torch.__version__, 'cuda', torch.cuda.is_available())"

echo "== 2. dreamer's hard deps -- report present/MISSING (only tensorboard was missing locally)"
for m in tensorboard gym gymnasium einops "ruamel.yaml" moviepy wandb matplotlib cv2 tqdm; do
  if $PY -c "import ${m}" 2>/dev/null; then echo "  $m: present"; else echo "  $m: MISSING"; fi
done

echo "== 3. install ONLY what's missing (edit this list from step 2 output)"
echo "   e.g.: $PY -m pip install tensorboard "gym==0.26.2"   # gym: dreamer needs old gym AND gymnasium"
echo "   NB: do NOT let pip touch torch/taichi/numpy -- pin-lock or --no-deps if it tries."

echo "== 4. runtime env vars (genesis forces PYOPENGL_PLATFORM=egl; mujoco must agree)"
echo "   export MUJOCO_GL=egl        # dreamer.py hardcodes osmesa; our patch uses setdefault"

echo "== 5. smoke test: build ONE genesis env through the dreamer adapter"
echo "   cd <dreamer repo> && MUJOCO_GL=egl $PY -c \"import dreamer,envs.genesis\""
echo ""
echo "If steps 1-2 pass and only tensorboard needed adding, you're done."
