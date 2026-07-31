#!/bin/bash
# OUROBOROS generation-k HARVEST+CONVERT (job B; submitted by sbatch_ouro_train.sh).
# Rolls the gen-k DP out over randomized ICs (harvest_ai_demos.py, closed loop),
# keeps success trajectories, VERIFIES each by open-loop replay (anti-manufacturing
# guard), runs the random-teacher NEGATIVE CONTROL, converts the kept demos to the
# gen-(k+1) LeRobotDataset, and submits the next train job. Not usually submitted by
# hand -- but it can be (same env vars) to redo a harvest.
#
# Honesty protocol baked in: --verify on every kept trajectory; negative control
# (random teacher, 50 ICs) MUST keep ~0 or the manifest flags it; per-gen manifest
# with yield + IC coverage lands in ouroboros/$TAG/gen$GEN/harvest/manifest.json.
#
# GPU node because: image teachers need EGL for the camera rig, and closed-loop DP
# inference (100 denoise steps/chunk) is far faster on GPU even for state policies.

#SBATCH -J ouro-harvest
#SBATCH -p gpu,preempt
#SBATCH --requeue
# 'preempt' has far more L40S but jobs can be KILLED at any moment. --requeue puts
# the job straight back in the queue; the payload below must therefore RESUME rather
# than restart, or preemption silently throws away hours of training.
#SBATCH --gres=gpu:1
# GPU type: lerobot training touches no genesis, and its eval builds the env on the
# CPU backend -- so A100 is fine here. Only dv3 (genesis on the GPU backend every
# step) segfaults at taichi init on sm_80; those sbatches stay pinned to l40s.
#SBATCH --constraint="l40s|a100"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-12:00:00      # 300 pick-scope rollouts ~2-4h + verify + convert
#SBATCH --output=ouro_harvest_%j.out
#SBATCH --error=ouro_harvest_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT
TAG=${TAG:?} ; GEN=${GEN:?} ; CAMS=${CAMS:-none} ; ACTIONS=${ACTIONS:-joint} ; CTRL=${CTRL:-vel} ; ALGO=${ALGO:-dp} ; IC_MODE=${IC_MODE:-demo}
MAXGEN=${MAXGEN:-3} ; SCOPE=${SCOPE:-pick} ; HARVEST_N=${HARVEST_N:-300}
G=ouroboros/$TAG/gen$GEN
NEXT=$((GEN + 1))
CKPT=$G/dp/checkpoints/last/pretrained_model
[ -d "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT (train job incomplete?)"; exit 1; }

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

IMG_FLAG=""
[ "$CAMS" != "none" ] && IMG_FLAG="--images"

python -c 'import lerobot' 2>/dev/null || {
  echo "FATAL: lerobot not importable ($CONDA_PREFIX). Run: bash cluster/install_lerobot.sh"; exit 1; }
echo "== ouroboros $TAG gen$GEN HARVEST start $(date) scope=$SCOPE n=$HARVEST_N cams=$CAMS"

# --- negative control FIRST (cheap; a broken predicate fails fast, before 300 rollouts)
NEG_N=${NEG_N:-50}
python baselines/harvest_ai_demos.py --teacher-type random \
  --action-space "$ACTIONS" --control "$CTRL" --ic-mode "$IC_MODE" \
  --n "$NEG_N" --scope "$SCOPE" --verify --seed 1 \
  --outdir "$G/harvest_negctl"
NEG_KEPT=$(python -c "import json; print(json.load(open('$G/harvest_negctl/manifest.json'))['kept'])")
echo "== negative control kept $NEG_KEPT/50"
# Rate-based gate: an absolute ">2 of 50" is 4%, tighter than the predicate's own
# measured floor. What matters is that a RANDOM teacher stays far below the real
# teacher, not that it is exactly zero.
NEG_MAX=${NEG_MAX_PCT:-10}
NEG_PCT=$(( 100 * NEG_KEPT / NEG_N ))
echo "== negative control: $NEG_KEPT/$NEG_N = ${NEG_PCT}% (gate: <=${NEG_MAX}%)"
[ "$NEG_PCT" -gt "$NEG_MAX" ] && {
  echo "FATAL: random teacher succeeds ${NEG_PCT}% of the time -- the success"
  echo "  predicate is too loose to distinguish a real teacher. Aborting."; exit 1; }

# --- harvest model-demos from the gen-k teacher -----------------------------------
python baselines/harvest_ai_demos.py --teacher-type dp \
  --checkpoint "$CKPT" --action-space "$ACTIONS" \
  --n "$HARVEST_N" ${TARGET_KEPT:+--target-kept $TARGET_KEPT} --ic-mode "$IC_MODE" \
  --scope "$SCOPE" --verify --seed "$GEN" $IMG_FLAG \
  --cap "${CAP:-600}" \
  --outdir "$G/harvest"
KEPT=$(python -c "import json; print(json.load(open('$G/harvest/manifest.json'))['kept'])")
echo "== gen$GEN harvest kept $KEPT/$HARVEST_N"
[ "$KEPT" -lt "${MIN_KEPT:-10}" ] && { echo "FATAL: <10 model-demos harvested -- teacher too weak to continue the chain"; exit 1; }

# --- convert -> gen-(k+1) dataset (same recipe as gen 0: proprio=8, video codec) ---
NEXTG=ouroboros/$TAG/gen$NEXT
mkdir -p "$NEXTG"
rm -rf "$NEXTG/dataset"
# Image datasets: mp4 needs torchcodec+FFmpeg (often absent in conda envs); PNG
# frames decode through PIL and need neither. Pick whichever this env supports so
# image conditions are not blocked on a system library. State-only ignores both.
CODEC=video
if [ "$CAMS" != "none" ]; then
  python -c 'from torchcodec.decoders import VideoDecoder' 2>/dev/null || CODEC=image
  echo "== image codec: $CODEC$( [ "$CODEC" = image ] && echo '  (no FFmpeg -> PNG frames)' )"
fi
python baselines/convert_to_lerobot.py \
  "$G/harvest" "$NEXTG/dataset" $( [ "$ACTIONS" = cartesian ] && echo 9 || echo 8 ) 4 "$CAMS" "$CODEC"
echo "== gen$NEXT dataset ready"

# --- chain: next generation's train (unless done) ---------------------------------
if [ "$NEXT" -ge "$MAXGEN" ]; then
  echo "== ouroboros $TAG complete: $MAXGEN generations trained, final harvest banked"
elif [ -z "${NO_CHAIN:-}" ]; then
  sbatch --export=ALL,TAG=$TAG,GEN=$NEXT,CAMS=$CAMS,ACTIONS=$ACTIONS,CTRL=$CTRL,ALGO=$ALGO,TARGET_KEPT=${TARGET_KEPT:-},MIN_KEPT=${MIN_KEPT:-10},IC_MODE=$IC_MODE,DATASET=$NEXTG/dataset,MAXGEN=$MAXGEN,SCOPE=$SCOPE,HARVEST_N=$HARVEST_N,TRAIN_STEPS=${TRAIN_STEPS:-100000},EVAL_EPS=${EVAL_EPS:-15},GENESIS_PICKAPLACE_ROOT=$GENESIS_PICKAPLACE_ROOT,CONDA_ENV=${CONDA_ENV:-} \
    cluster/sbatch_ouro_train.sh
  echo "== submitted train for gen$NEXT"
fi
