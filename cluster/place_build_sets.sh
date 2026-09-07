#!/bin/bash
# Build the PLACE-phase DP sets of record (PHASE_PLAN amendment (h), 2026-09-07): cut the FULL contract-v1 tapes at the
# phase manifests' [k_pick, k_placed_v2], cross-check against the r2dreamer segments RLPD trains on, convert to lerobot
# (fps 7.5). Data only (rsync-only dirs), never git. Idempotent: refuses to overwrite a non-empty set unless REDO=1.
#   DEMO_ROOT  target root (default /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3)
#   TAPES      demos_v2 root holding dHfull_w3_all / dDPfull_w3_all (default .../genesis_pickaplace/baselines/demos_v2)
#   W          WM-fix dir with phase_banks/ and demos_state/
# Usage (conda env with lerobot active, from the code checkout root): bash cluster/place_build_sets.sh
#SBATCH -J pl_build
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH -N 1
#SBATCH -c 4
#SBATCH --mem=16g
#SBATCH --time=1:00:00
#SBATCH --output=pl_build_%j.out
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1
LAB=/cluster/tufts/shortlab/jstale02
W=${W:-$LAB/wm_fix_2026-09-03}; DEMO_ROOT=${DEMO_ROOT:-$LAB/genesis_pickaplace/baselines/matched_w3}; TAPES=${TAPES:-$LAB/genesis_pickaplace/baselines/demos_v2}
if [ -n "${SLURM_JOB_ID:-}" ]; then module load anaconda/2025.06.0; conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"; fi
FORCE=(); [ "${REDO:-0}" = 1 ] && FORCE=(--force)
echo "== place_build_sets $(date) host=$(hostname) git=$(git rev-parse --short HEAD) DEMO_ROOT=$DEMO_ROOT"
if [ -f "$DEMO_ROOT/dH_place/manifest.json" ] && [ "${REDO:-0}" != 1 ]; then echo "# dH_place already cut (manifest present), kept"; else
python baselines/rl/place_demos.py cut --src "$TAPES/dHfull_w3_all"  --phases "$W/phase_banks/human_phases.json"   --out "$DEMO_ROOT/dH_place" "${FORCE[@]}"; fi
if [ -f "$DEMO_ROOT/dDP_place_n39/manifest.json" ] && [ "${REDO:-0}" != 1 ]; then echo "# dDP_place_n39 already cut (manifest present), kept"; else
python baselines/rl/place_demos.py cut --src "$TAPES/dDPfull_w3_all" --phases "$W/phase_banks/machine_phases.json" --out "$DEMO_ROOT/dDP_place_n39" --one-per-ic --keep-from "$W/demos_state/dDP_place_n39/repeat.json" "${FORCE[@]}"; fi
python baselines/rl/place_demos.py check --raw "$DEMO_ROOT/dH_place"       --segments "$W/demos_state/dH_place"
python baselines/rl/place_demos.py check --raw "$DEMO_ROOT/dDP_place_n39"  --segments "$W/demos_state/dDP_place_n39"
python baselines/rl/place_demos.py census --segments "$W/demos_state/dH_place"
python baselines/rl/place_demos.py census --segments "$W/demos_state/dDP_place_n39"
for SET in dH_place dDP_place_n39; do
  DS=$DEMO_ROOT/$SET/lerobot
  if [ -d "$DS" ] && [ "${REDO:-0}" = 1 ]; then rm -rf "$DS"; fi
  [ -d "$DS" ] || python baselines/convert_to_lerobot.py "$DEMO_ROOT/$SET" "$DS" 8 4 none
  python3 -c 'import json,sys; i=json.load(open(sys.argv[1]+"/meta/info.json")); m=json.load(open(sys.argv[2])); assert i["total_episodes"]==m["n_kept"]==39 and abs(i["fps"]-7.5)<1e-9 and i["total_frames"]==m["decisions_total"], (i["total_episodes"], i["fps"], i["total_frames"], m["decisions_total"]); print(f"LEROBOT-OK {sys.argv[1]}: episodes {i[\"total_episodes\"]} frames {i[\"total_frames\"]} fps {i[\"fps\"]} sha {m[\"content_sha256\"][:16]}")' "$DS" "$DEMO_ROOT/$SET/manifest.json"
done
echo "== place_build_sets done $(date)"
