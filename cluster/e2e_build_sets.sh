#!/bin/bash
# Build the END-TO-END (full task) DP sets of record (PHASE_PLAN amendment (n), 2026-09-07): take the FULL
# contract-v1 tapes the world model's amendment-(d) segments were made from -- UNCUT and UNPRUNED -- cross-check them
# row for row against those segments (which RLPD trains on verbatim), and convert to lerobot (fps 7.5).
# Data only (rsync-only dirs), never git. Idempotent: refuses to overwrite a non-empty set unless REDO=1.
#   DEMO_ROOT  target root (default /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3)
#   W          WM-fix dir holding demos_state_full/{dHfull_all,dDPfull,src_dHfull_all,src_dDPfull}
# Usage (conda env with lerobot active, from the code checkout root): bash cluster/e2e_build_sets.sh
#SBATCH -J e2e_build
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --nice=9000
#SBATCH -N 1
#SBATCH -c 4
#SBATCH --mem=16g
#SBATCH --time=1:00:00
#SBATCH --output=e2e_build_%j.out
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1
LAB=/cluster/tufts/shortlab/jstale02
W=${W:-$LAB/wm_fix_2026-09-03}; DEMO_ROOT=${DEMO_ROOT:-$LAB/genesis_pickaplace/baselines/matched_w3}
if [ -n "${SLURM_JOB_ID:-}" ]; then module load anaconda/2025.06.0; conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"; fi
FORCE=(); [ "${REDO:-0}" = 1 ] && FORCE=(--force)
echo "== e2e_build_sets $(date) host=$(hostname) git=$(git rev-parse --short HEAD) DEMO_ROOT=$DEMO_ROOT W=$W"
build_one() {   # $1 = set name (dHfull_all | dDPfull), $2 = source tape dir under demos_state_full, $3 = expected N
  local SET=$1 SRC=$2 N=$3
  if [ -f "$DEMO_ROOT/$SET/manifest.json" ] && [ "${REDO:-0}" != 1 ]; then
    echo "# $SET already selected (manifest present), kept"
  else
    python baselines/rl/full_demos.py select --segments "$W/demos_state_full/$SET" --src "$W/demos_state_full/$SRC" \
        --out "$DEMO_ROOT/$SET" "${FORCE[@]}"
  fi
  python baselines/rl/full_demos.py check  --raw "$DEMO_ROOT/$SET" --segments "$W/demos_state_full/$SET"
  python baselines/rl/full_demos.py census --segments "$W/demos_state_full/$SET" --raw "$DEMO_ROOT/$SET"
  local DS=$DEMO_ROOT/$SET/lerobot
  if [ -d "$DS" ] && [ "${REDO:-0}" = 1 ]; then rm -rf "$DS"; fi
  [ -d "$DS" ] || python baselines/convert_to_lerobot.py "$DEMO_ROOT/$SET" "$DS" 8 4 none
  python3 - "$DS" "$DEMO_ROOT/$SET/manifest.json" "$N" <<'PY'
import json, sys
i = json.load(open(sys.argv[1] + '/meta/info.json')); m = json.load(open(sys.argv[2]))
# the RAW set is the full tape count; the lerobot dataset can hold FEWER (convert_to_lerobot drops episodes below
# MIN_FRAMES -- recorded as n_lerobot/decisions_lerobot/short_tapes by full_demos.py select, never silent)
assert m['n_kept'] == int(sys.argv[3]), (m['n_kept'], sys.argv[3])
assert i['total_episodes'] == m['n_lerobot'], (i['total_episodes'], m['n_lerobot'])
assert abs(i['fps'] - 7.5) < 1e-9 and i['total_frames'] == m['decisions_lerobot'], (i['fps'], i['total_frames'], m['decisions_lerobot'])
print('LEROBOT-OK %s: episodes %d/%d frames %d/%d fps %s short_tapes %s sha %s' % (
    sys.argv[1], i['total_episodes'], m['n_kept'], i['total_frames'], m['decisions_total'], i['fps'], m['short_tapes'], m['content_sha256'][:16]))
PY
}
build_one dHfull_all src_dHfull_all 74
build_one dDPfull    src_dDPfull    72
echo "== e2e_build_sets done $(date)"
