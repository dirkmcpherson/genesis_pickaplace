#!/bin/bash
# Install the EXACT Genesis our physics validated against, on a fresh machine
# (e.g. the cluster). = upstream Genesis-Embodied-AI @ 31951c3f (270 commits past
# v0.2.1) + one 6-line local patch (headless render, no X11 popups).
#
# Run with the target env active (conda or venv). Needs git + pip.
#   ./third_party/install_genesis.sh [/dest/dir]   (default: ./Genesis)
set -e
DEST=${1:-$PWD/Genesis}
PATCH="$(cd "$(dirname "$0")" && pwd)/genesis-headless-render.patch"

echo "== clone upstream Genesis @ the validated commit"
if [ ! -d "$DEST/.git" ]; then
  git clone https://github.com/Genesis-Embodied-AI/Genesis.git "$DEST"
fi
cd "$DEST"
git fetch --all --quiet
git checkout -q 31951c3f7349a409e8ca729187abe7ae3fe63ec7

echo "== apply the headless-render patch"
if git apply --check "$PATCH" 2>/dev/null; then
  git apply "$PATCH" && echo "  patch applied"
else
  git apply --reverse --check "$PATCH" 2>/dev/null && echo "  patch already applied, skipping" \
    || { echo "  ERROR: patch does not apply cleanly"; exit 1; }
fi

echo "== editable install (does NOT pull torch/taichi -- pin those yourself)"
pip install -e . --no-build-isolation
echo "== CRITICAL pins for physics parity: torch==2.7.0+cu126  taichi==1.7.4  numpy==2.2.6"
python -c "import genesis; print('genesis', genesis.__version__, 'OK')"
