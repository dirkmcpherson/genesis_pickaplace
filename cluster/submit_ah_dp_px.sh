#!/usr/bin/env bash
# PHASE_PLAN amendment (ah): {Diffusion Policy} on PIXEL observations, human v machine, 4 v 4.
# Registered 2026-09-14 BEFORE any build. Mirrors submit_aa_rev5_sparse_packs.sh's conventions.
#
#   GP=$LAB/gp_ah bash cluster/submit_ah_dp_px.sh          # submits 8 jobs
#   GP=$LAB/gp_ah DRYRUN=1 bash cluster/submit_ah_dp_px.sh # prints what each job would do, submits nothing
#
# GP must be a FRESH clone (gp_ah). It is refused if it names any of the pinned trees that in-flight jobs
# import at run time.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
: "${GP:?set GP to the fresh (ah) clone (gp_ah); never a pinned tree}"
case "$GP" in
  *gp_ladderN*|*gp_ac|*gp_ac/*|*gp_aa4*|*gp_e2e*|*gp_root*|*gp_unified*|*r2dreamer_*)
    echo "FATAL: refusing pinned/in-flight tree $GP"; exit 1;;
esac
[ -f "$GP/cluster/sbatch_dp_px.sh" ] || { echo "FATAL: $GP has no cluster/sbatch_dp_px.sh (clone is too old)"; exit 1; }
[ -f "$GP/baselines/verify_px_lerobot.py" ] || { echo "FATAL: $GP has no baselines/verify_px_lerobot.py"; exit 1; }
grep -q -- '--camera-rig' "$GP/baselines/eval_e2e.py" || { echo "FATAL: $GP/baselines/eval_e2e.py has no --camera-rig"; exit 1; }
DEMO_ROOT=${DEMO_ROOT:-$LAB/genesis_pickaplace/baselines/matched_w3}
STEPS=${STEPS:-100000}; SEEDS=${SEEDS:-"0 1 2 3"}
LADDER=${LADDER:-nested_sparse10}; TIP_GUARD=${TIP_GUARD:-not_in_hand}

# ---- the datasets must exist and must still pass the action gate (cluster/ah_build_pixel_sets.sh) ----
# The gate needs numpy + pyarrow + pandas, which live in the conda env, not in the login node's python3.
# Refuse with an instruction rather than a traceback -- and never skip the gate silently.
PY=${PY:-python3}
$PY -c 'import numpy, pyarrow, pandas' 2>/dev/null || {
  echo "FATAL: $PY cannot import numpy/pyarrow/pandas, so the dataset gate cannot run. Activate the env first:"
  echo "         module load anaconda/2025.06.0 && conda activate $LAB/condaenv/genesis"
  echo "       (or set PY=<interpreter>). The gate is not optional -- it is registration gate 2."; exit 1; }
for SET in dHfull_all dDPfull_first; do
  [ -d "$DEMO_ROOT/$SET/lerobot_px" ] || { echo "FATAL: $DEMO_ROOT/$SET/lerobot_px missing -- run cluster/ah_build_pixel_sets.sh first"; exit 1; }
  $PY "$GP/baselines/verify_px_lerobot.py" --px "$DEMO_ROOT/$SET/lerobot_px" \
      --ref "$DEMO_ROOT/$SET/lerobot" --raw "$DEMO_ROOT/$SET" || { echo "FATAL: $SET pixel dataset failed its gate"; exit 1; }
done

FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 100 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 100 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($(git -C "$GP" describe --always --dirty)) | steps $STEPS | ladder $LADDER | tip_guard $TIP_GUARD"

Q=(-p gpu --qos=normal)
one () {  # arm seed
  local arm=$1 s=$2 name="ah_dp_px_${1}_s${2}"
  [ -e "$GP/baselines/outputs/dp_px/$name" ] && { echo "FATAL: run dir for $name already exists"; exit 1; }
  if [ -n "${DRYRUN:-}" ]; then
    env GENESIS_PICKAPLACE_ROOT="$GP" ARM="$arm" SEED="$s" STEPS="$STEPS" DEMO_ROOT="$DEMO_ROOT" \
        LADDER="$LADDER" TIP_GUARD="$TIP_GUARD" DRYRUN=1 bash "$GP/cluster/sbatch_dp_px.sh"
    return
  fi
  env GENESIS_PICKAPLACE_ROOT="$GP" ARM="$arm" SEED="$s" STEPS="$STEPS" DEMO_ROOT="$DEMO_ROOT" \
      LADDER="$LADDER" TIP_GUARD="$TIP_GUARD" \
      sbatch -J "$name" "${Q[@]}" "$GP/cluster/sbatch_dp_px.sh" | sed "s/$/  # $name/"
}
for S in $SEEDS; do one dH "$S"; done
for S in $SEEDS; do one dM "$S"; done
