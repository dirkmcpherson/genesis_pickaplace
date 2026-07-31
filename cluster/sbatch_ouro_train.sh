#!/bin/bash
# OUROBOROS generation-k TRAIN+EVAL (job A of the chained pair).
# Trains a Diffusion Policy on this generation's dataset, evals it honestly
# (15 random ICs, videos -> wandb), then submits sbatch_ouro_harvest.sh (job B),
# which harvests model-demos from the trained DP and, if GEN+1 < MAXGEN, submits
# the next generation's train job. Generations therefore alternate A->B->A->B...
# with no babysitting; kill the chain by scancel'ing the pending job or NO_CHAIN=1.
#
# --- Submit gen 0 ----------------------------------------------------------------
#   cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
#   TAG=ouro1 CAMS=none DATASET=baselines/lerobot_dataset_pick_pruned/genesis_pickaplace \
#     sbatch cluster/sbatch_ouro_train.sh
#   CAMS: none | top | top,wrist  -- MUST match how DATASET was converted; controls
#         whether eval/harvest build the camera rig.
#   Gen-0 DATASET is the HUMAN-demo LeRobotDataset (rsync it from the box; datasets
#   are gitignored). Later gens' datasets are produced by job B on the cluster.
#   Overrides: TRAIN_STEPS (100000) EVAL_EPS (15) MAXGEN (3) SCOPE (pick|full, pick)
#              HARVEST_N (300) GEN (0) NO_CHAIN
# All state lives under ouroboros/$TAG/gen$GEN/ (dp/, harvest/, dataset/, logs).

#SBATCH -J ouro-train
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
#SBATCH --time=0-12:00:00      # 100k DP steps ~2.5-3h on L40S + eval ~1h; margin for queue-day variance
#SBATCH --output=ouro_train_%j.out
#SBATCH --error=ouro_train_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT
TAG=${TAG:?set TAG (experiment name, stable across the whole chain)}
GEN=${GEN:-0}
CAMS=${CAMS:-none} ; ACTIONS=${ACTIONS:-joint} ; CTRL=${CTRL:-vel} ; ALGO=${ALGO:-dp} ; IC_MODE=${IC_MODE:-demo}
DATASET=${DATASET:?set DATASET (LeRobotDataset root for this generation)}
G=ouroboros/$TAG/gen$GEN
mkdir -p "$G"

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl

# cuDNN loader guard (see cluster_bootstrap.sh: the shared nvidia/ tree must win)
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

# --- preflight: fail immediately with the remedy, not 50 lines in ---------------
if command -v lerobot-train >/dev/null 2>&1; then
  LEROBOT_TRAIN="lerobot-train"
elif python -c 'import lerobot.scripts.lerobot_train' 2>/dev/null; then
  LEROBOT_TRAIN="python -m lerobot.scripts.lerobot_train"   # console script absent
else
  echo "FATAL: lerobot is not importable in this env ($CONDA_PREFIX)."
  echo "  Run once:  conda activate <env> && bash cluster/install_lerobot.sh"
  exit 1
fi
[ -d "$DATASET" ] || { echo "FATAL: dataset missing: $DATASET (rsync it -- gitignored)"; exit 1; }
if [ "$CAMS" != "none" ] && grep -q '"dtype": "video"' "$DATASET/meta/info.json" 2>/dev/null; then
  # only VIDEO-encoded image datasets need decoding; PNG-frame datasets do not
  python -c 'from torchcodec.decoders import VideoDecoder' 2>/dev/null || {
    echo "FATAL: $DATASET is video-encoded but torchcodec/FFmpeg is unavailable."
    echo "  Rebuild it with the PNG codec, or: conda install -c conda-forge 'ffmpeg<8'"
    exit 1; }
fi
echo "== ouroboros $TAG gen$GEN TRAIN start $(date) dataset=$DATASET cams=$CAMS"
echo "== trainer: $LEROBOT_TRAIN"

# --- train -----------------------------------------------------------------------
# PREEMPTION-SAFE: only wipe the output dir on a FIRST attempt. On a requeue
# (SLURM_RESTART_COUNT>0) keep it and resume from the last checkpoint, otherwise
# every preemption restarts training from zero.
RESUME_ARG=""
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$G/dp/checkpoints/last" ]; then
  echo "== requeued (restart #${SLURM_RESTART_COUNT}); RESUMING from $G/dp/checkpoints/last"
  RESUME_ARG="--resume=true"
else
  rm -rf "$G/dp"
fi
$LEROBOT_TRAIN ${RESUME_ARG} \
  --dataset.repo_id="local/${TAG}_${ALGO}_gen${GEN}" \
  --dataset.root="$DATASET" \
  --policy.type=$( [ "$ALGO" = act ] && echo act || echo diffusion ) \
  --policy.push_to_hub=false \
  --output_dir="$G/dp" --batch_size=64 --steps="${TRAIN_STEPS:-100000}" \
  --job_name="${TAG}_${ALGO}_gen${GEN}" \
  --wandb.enable=true --wandb.project=genesis_pickaplace_ouro \
  --wandb.disable_artifact=true
CKPT=$G/dp/checkpoints/last/pretrained_model
[ -d "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT"; exit 1; }

# --- honest eval: 15 random ICs, stage metrics + tiled video -> wandb -------------
python baselines/wandb_eval.py --kind dp --checkpoint "$CKPT" \
  $( [ "$ACTIONS" = cartesian ] && echo --cartesian --control $CTRL ) \
  --random "${EVAL_EPS:-15}" --seed 0 \
  --group "${TAG}" --name "${TAG}_${ALGO}_gen${GEN}-eval" \
  > "$G/eval.log" 2>&1 \
  || { echo "WARN: eval FAILED (train artifact intact; harvest still submitted)"
       echo "---- last 25 lines of $G/eval.log ----"
       tail -25 "$G/eval.log"
       echo "---- end ----"; }

echo "== gen$GEN train+eval done $(date)"

# --- chain: job B (harvest+convert+next-gen submit) -------------------------------
if [ -z "${NO_CHAIN:-}" ]; then
  sbatch --export=ALL,TAG=$TAG,GEN=$GEN,CAMS=$CAMS,ACTIONS=$ACTIONS,CTRL=$CTRL,ALGO=$ALGO,TARGET_KEPT=${TARGET_KEPT:-},IC_MODE=$IC_MODE,MAXGEN=${MAXGEN:-3},SCOPE=${SCOPE:-pick},HARVEST_N=${HARVEST_N:-300},TRAIN_STEPS=${TRAIN_STEPS:-100000},EVAL_EPS=${EVAL_EPS:-15},GENESIS_PICKAPLACE_ROOT=$GENESIS_PICKAPLACE_ROOT,CONDA_ENV=${CONDA_ENV:-} \
    cluster/sbatch_ouro_harvest.sh
  echo "== submitted harvest for gen$GEN"
fi
