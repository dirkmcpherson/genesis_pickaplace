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
# Pin to the VALIDATED ref, not the fork's default branch. lerobot main has moved to
# requires-python >=3.12 while the cluster env (and the dev box) run 3.10; the
# genesis-fixes branch is lerobot 0.4.5 (requires-python >=3.10) plus the
# image_writer mkdir fix -- exactly what every local result was produced with.
LEROBOT_REF=${LEROBOT_REF:-genesis-fixes}
# The repo carries git-lfs pointers (test assets/media -- not needed to train).
# Clusters rarely have git-lfs, and without it the CHECKOUT fails after a successful
# clone, leaving a source tree with missing files. Skip the smudge filter entirely.
export GIT_LFS_SKIP_SMUDGE=1
GITNOLFS="git -c filter.lfs.smudge= -c filter.lfs.process= -c filter.lfs.required=false"
if [ ! -d "$LEROBOT_DIR/.git" ]; then
  $GITNOLFS clone git@github.com:dirkmcpherson/lerobot.git "$LEROBOT_DIR"
fi
( cd "$LEROBOT_DIR" && $GITNOLFS fetch origin "$LEROBOT_REF" \
  && $GITNOLFS checkout -f "$LEROBOT_REF" ) \
  || { echo "FATAL: cannot check out lerobot ref '$LEROBOT_REF'"; exit 1; }
# repair a clone whose checkout died mid-way (empty working tree, HEAD intact)
if [ ! -f "$LEROBOT_DIR/pyproject.toml" ]; then
  echo "== repairing incomplete checkout in $LEROBOT_DIR"
  ( cd "$LEROBOT_DIR" && $GITNOLFS checkout -f HEAD )
fi
[ -f "$LEROBOT_DIR/pyproject.toml" ] || {
  echo "FATAL: $LEROBOT_DIR has no pyproject.toml -- checkout still incomplete"; exit 1; }
PYV=$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])')
REQ=$(grep -m1 requires-python "$LEROBOT_DIR/pyproject.toml" || true)
echo "== lerobot ref $LEROBOT_REF ($REQ) against python $PYV"
case "$REQ" in
  *3.12*) echo "FATAL: this lerobot ref needs python >=3.12 but the env has $PYV."
          echo "  Use LEROBOT_REF=genesis-fixes (0.4.5, python>=3.10) -- the ref every"
          echo "  local result was produced with."; exit 1 ;;
esac

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

# torchcodec needs FFmpeg shared libs (libavutil et al); conda envs often lack them.
# Only IMAGE/VIDEO datasets decode -- state-only training (CAMS=none, which every
# current ouroboros condition uses) never does. Try to supply them, do not require.
# NEVER conda-install into this env: conda-forge packages carry their own
# libstdc++/GLIBCXX which genesis/taichi's C++ (mesh loading) was not built against
# -- prime suspect for the 07-30 segfaults-at-scene-build that appeared right after
# an ffmpeg conda install here. The dev box proves pip-only coexistence works.
# torchcodec/FFmpeg stays OPTIONAL: state-only datasets never decode video, and
# image datasets fall back to PNG frames automatically.
python -c 'from torchcodec.decoders import VideoDecoder' 2>/dev/null ||   echo "== note: torchcodec/FFmpeg unavailable (fine: state-only + PNG paths in use)"

python - <<'VERIFY'
import torch, lerobot
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from lerobot.policies.act.configuration_act import ACTConfig
print(f'OK: torch {torch.__version__} + lerobot {lerobot.__version__} (dp+act configs load)')
try:
    import torchcodec
    from torchcodec.decoders import VideoDecoder
    print(f'OK: torchcodec {torchcodec.__version__} -- image/video datasets available')
except Exception as e:
    print('WARN: torchcodec/FFmpeg unavailable. STATE-ONLY datasets (CAMS=none) are')
    print('      unaffected; image/video datasets would fail to load.')
    print('      detail:', str(e).splitlines()[0][:100])
VERIFY
echo "lerobot install verified (state-only path is a hard requirement; see any WARN)"
