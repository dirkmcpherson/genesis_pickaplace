#!/bin/bash
# One-shot: build the whole genesis+dreamer stack in the ACTIVE conda/venv, in the
# order that actually works (torch -> base deps -> Genesis editable). Run from the
# genesis_pickaplace repo root.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"

echo "== 1/4 torch trio (cu126 wheels live only on the pytorch index)"
pip install --index-url https://download.pytorch.org/whl/cu126 \
    torch==2.7.0+cu126 torchvision==0.22.0+cu126 torchcodec==0.10.0+cu126

echo "== 2/4 base stack from the validated freeze (numpy, taichi 1.7.4, ...)"
grep -vE "^(-e|genesis|lerobot|torch==|torchvision==|torchcodec==)" \
    "$REPO/migration_requirements.txt" | pip install -r /dev/stdin

echo "== 3/4 Genesis (upstream@31951c3f + headless patch)"
"$REPO/third_party/install_genesis.sh"

echo "== 4/4 dreamer needs only tensorboard on top"
pip install tensorboard

python -c "import torch,genesis,taichi,numpy; \
print('torch',torch.__version__,'cuda',torch.cuda.is_available()); \
print('genesis',genesis.__version__,'taichi',taichi.__version__,'numpy',numpy.__version__)"
echo "== done. Next: MIGRATION.md Gate 1 to confirm physics parity."
