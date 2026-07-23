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
# The freeze is a pip-list from a py3.10.12 machine; a few pins have no wheel for
# other platforms/py builds (e.g. pymeshlab 2025.7.post1 needs >=3.11). Try the exact
# freeze first; if it fails, relax the NON-numerics-critical pins and let pip pick
# compatible versions. numpy/taichi stay hard-pinned -- they affect physics parity.
BASE=$(grep -vE "^(-e|genesis|lerobot|torch==|torchvision==|torchcodec==)" \
    "$REPO/migration_requirements.txt")
if ! echo "$BASE" | pip install -r /dev/stdin 2>/tmp/pipbase.err; then
  echo "  exact freeze failed; retrying with non-critical pins relaxed"
  echo "  (numpy==2.2.6 taichi==1.7.4 stay pinned for physics parity)"
  # keep the two numerics pins exact; strip == from everything else
  echo "$BASE" | sed -E 's/^(numpy|taichi)==.*/&/; t; s/([A-Za-z0-9_.-]+)==.*/\1/' \
      | pip install -r /dev/stdin
fi

echo "== 3/4 Genesis (upstream@31951c3f + headless patch)"
"$REPO/third_party/install_genesis.sh"

echo "== 4/4 dreamer needs only tensorboard on top"
pip install tensorboard

python -c "import torch,genesis,taichi,numpy; \
print('torch',torch.__version__,'cuda',torch.cuda.is_available()); \
print('genesis',genesis.__version__,'taichi',taichi.__version__,'numpy',numpy.__version__)"
echo "== done. Next: MIGRATION.md Gate 1 to confirm physics parity."
