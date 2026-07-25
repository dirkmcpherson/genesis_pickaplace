#!/bin/bash
# Add lerobot (DP training) to the cluster genesis conda env. Run ONCE, after
# cluster_bootstrap.sh, inside the activated env:
#   conda activate <genesis-env> && bash cluster/install_lerobot.sh
#
# Pins that matter (learned the hard way on the dev box):
#   * torchcodec MUST match torch: 0.3.x <-> torch 2.7. The 0.10 that pip resolves
#     by default against modern indexes has a different C++ ABI -> undefined symbol
#     at import, which only surfaces when a video dataset is LOADED (train start).
#   * lerobot is the user's fork (has the image_writer mkdir fix), editable install
#     so cluster-side patches are possible.
set -eo pipefail

LEROBOT_DIR=${LEROBOT_DIR:-$HOME/lerobot}
if [ ! -d "$LEROBOT_DIR" ]; then
  git clone git@github.com:dirkmcpherson/lerobot.git "$LEROBOT_DIR"
fi

TORCH_V=$(python -c 'import torch; print(torch.__version__.split("+")[0])')
case "$TORCH_V" in
  2.7.*) TC="torchcodec==0.3.*" ;;
  2.6.*) TC="torchcodec==0.2.*" ;;
  *) echo "WARNING: torch $TORCH_V has no known torchcodec pairing here; trying 0.3.*"; TC="torchcodec==0.3.*" ;;
esac

pip install -e "$LEROBOT_DIR" --no-deps
# lerobot deps minus the heavy/conflicting ones the genesis env already provides
pip install "$TC" diffusers einops datasets "huggingface_hub>=0.27" \
    jsonlines draccus av wandb imageio deepdiff termcolor

python - <<'PY'
import torch, torchcodec, lerobot
from torchcodec.decoders import VideoDecoder
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
print(f'OK: torch {torch.__version__} + torchcodec {torchcodec.__version__} + lerobot {lerobot.__version__}')
PY
echo "lerobot install verified"
