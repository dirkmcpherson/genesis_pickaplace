#!/bin/bash
# Build the PIXEL lerobot datasets for PHASE_PLAN amendment (ag) (2026-09-14).
#
# Same tapes as the end-to-end DP arm of record (amendments (n)/(ab)) -- the RECORDER npz under
# $DEMO_ROOT/{dHfull_all,dDPfull_first} keep supplying `states` and `actions`, so the action column
# of the new dataset is the (n)/(ab) column byte for byte -- with the PIXELS taken from the
# r2dreamer-native `_img` re-executions ($W/demos_state_full/*_rns10h_img, real renders 74/74 and
# 72/72, action sha identical to the sources). The observation is proprio (8) + top + wrist and
# carries NO observation.environment_state: the ground-truth can pose and goal xy are removed
# exactly as `env.state_slice 8` removes them for the (af) pixel world models.
#
# convert_to_lerobot.py does the work under two opt-in env vars (LEROBOT_IMAGES_FROM,
# LEROBOT_NO_ENV_STATE); it gates every tape pairing itself (native T == n+1, native delta stream
# == the tape's actions_delta byte for byte, native state rows == the tape's states byte for byte).
# baselines/verify_px_lerobot.py then gates the finished dataset against the state dataset of record.
#
# IMG_DTYPE is `image` (PNG frames), NOT `video`: torchcodec cannot load its shared libraries in
# either the cluster conda env or the local venv (FFmpeg ABI), so an mp4 dataset would not decode.
# 64x64 PNGs cost ~5 KB per frame-pair, ~330 MB for both arms.
#
#   DEMO_ROOT  default /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3
#   W          default $LAB/wm_fix_2026-09-03      REDO=1 rebuilds an existing dataset
# Usage (conda env with lerobot active, from the code checkout root):
#   GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/ag_build_pixel_sets.sh      # or: bash cluster/ag_build_pixel_sets.sh
#SBATCH -J ag_px_build
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --nice=9000
#SBATCH -N 1
#SBATCH -c 8
#SBATCH --mem=32g
#SBATCH --time=4:00:00
#SBATCH --output=ag_px_build_%j.out
set -eo pipefail
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this build must use}"
cd "$GENESIS_PICKAPLACE_ROOT"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}; DEMO_ROOT=${DEMO_ROOT:-$LAB/genesis_pickaplace/baselines/matched_w3}
if [ -n "${SLURM_JOB_ID:-}" ]; then module load anaconda/2025.06.0; conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"; fi
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9')
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "== ag_build_pixel_sets $(date) host=$(hostname) git=$(git rev-parse --short HEAD) DEMO_ROOT=$DEMO_ROOT W=$W free=${FREE_GB}G"

build_one() {   # $1 = raw set name, $2 = native _img set name, $3 = expected tape count
  local SET=$1 IMG=$2 N=$3
  local RAW=$DEMO_ROOT/$SET IMGDIR=$W/demos_state_full/$IMG DS=$DEMO_ROOT/$SET/lerobot_px REF=$DEMO_ROOT/$SET/lerobot
  [ -d "$RAW" ] || { echo "FATAL: raw set $RAW missing (cluster/e2e_build_sets.sh)"; exit 1; }
  [ -d "$IMGDIR" ] || { echo "FATAL: native image set $IMGDIR missing"; exit 1; }
  [ -d "$REF" ] || { echo "FATAL: state dataset of record $REF missing -- the action gate has nothing to compare to"; exit 1; }
  python3 - "$RAW" "$IMGDIR" "$N" <<'PY'
import json, os, sys
raw, img, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
m = json.load(open(os.path.join(raw, 'manifest.json')))
assert m.get('contract') == 'v1' and m.get('scope') == 'full', m
assert int(m['n_kept']) == n, (m.get('n_kept'), n)
r = json.load(open(os.path.join(img, 'repeat.json')))
assert r.get('images') == 'rendered', ('the _img set carries no rendered image column', r.get('images'))
rel = r.get('relabel', {})
assert rel.get('images', {}).get('column') == 'rendered', rel.get('images')
assert float(rel['images']['nonzero_frac_min']) > 0.0, rel['images']
print(f"SRC-OK raw={raw} n_kept={m['n_kept']} sha={m['content_sha256'][:16]} | img={img} "
      f"ladder={rel.get('ladder')} tip_guard={rel.get('tip_guard')} renders={rel['images']['column']} "
      f"nonzero_frac_min={rel['images']['nonzero_frac_min']} mean_min={rel['images']['mean_min']}")
PY
  if [ -d "$DS" ] && [ "${REDO:-0}" = 1 ]; then echo "== REDO=1: removing $DS"; rm -rf "$DS"; fi
  if [ -d "$DS" ]; then echo "# $DS exists, kept"; else
    LEROBOT_IMAGES_FROM="$IMGDIR" LEROBOT_NO_ENV_STATE=1 \
      python baselines/convert_to_lerobot.py "$RAW" "$DS" 8 4 top,wrist image
  fi
  python3 baselines/verify_px_lerobot.py --px "$DS" --ref "$REF" --raw "$RAW"
  du -sh "$DS"
}

build_one dHfull_all       dHfull_all_rns10h_img       74
build_one dDPfull_first    dDPfull_first_rns10h_img    72
echo "== ag_build_pixel_sets done $(date)"
