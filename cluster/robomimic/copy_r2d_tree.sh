#!/bin/bash
# Robomimic leg: make $LAB/robomimic_r2d = a COPY of the WM-fix r2dreamer tree ($W/r2dreamer_fix, the recipe of record)
# so the robosuite adapter + the configurable state-dim assert never touch the Genesis tree. Code only (no runs/caches).
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
SRC=$LAB/wm_fix_2026-09-03/r2dreamer_fix; DST=${ROBO_R2D:-$LAB/robomimic_r2d}
[ -d "$SRC" ] || { echo "FATAL: $SRC missing"; exit 1; }
mkdir -p "$DST"
rsync -a --exclude '__pycache__' --exclude 'runs' --exclude 'logdir' --exclude 'outputs' --exclude '.git' --exclude 'wandb' --exclude '*.pt' "$SRC/" "$DST/"
( cd "$SRC" && git rev-parse --short HEAD 2>/dev/null || echo no-git ) > "$DST/COPIED_FROM.txt"
echo "src=$SRC copied $(date -Is)" >> "$DST/COPIED_FROM.txt"
du -sh "$DST"; echo "# copy_r2d_tree done $(date -Is)"
