#!/bin/bash
# r2dreamer MANISKILL POSITIVE CONTROL: one seed per GPU, env-var driven.
#
# --- Submit (from the genesis_pickaplace checkout root) ---------------------------
#   for S in 0 1 2; do SEED=$S sbatch cluster/sbatch_r2d_ms.sh; done
#
# Env vars (defaults):
#   SEED      0
#   CONFIG    maniskill_pickcube            (hydra env group)
#   STEPS     3.02e5                        (~1.1k spent by the demo prefill)
#   DEMO_DIR  $DV3_DIR/demonstrations/maniskill_teleop_sparse100  (rsync only)
#   LOGDIR    $R2D_DIR/runs/ms_pickcube_s$SEED
#   VENV      ~/r2d_ms_venv   (cluster/install_r2dreamer_ms.sh -- NOT ~/r2d_venv;
#                              mani_skill needs gymnasium 0.29/numpy<2, the genesis
#                              env is pinned the other way. See that script's header.)
#   ENV_NUM 4  EVAL_EPS 10  BUFFER_MAX 3.2e5  PRETRAIN 100  SAVE_EVERY 5e4
#   EXTRA (raw hydra overrides)   DRYRUN=1 (print the plan, no submit)
#
# WHAT THIS IS: our fork on their benchmark. The genesis arm ignites on 6/24
# seeds; the reference dv3 implementation ignites on ~every seed on this exact
# task (takeoff ~110-137k env steps). Bar and protocol:
# genesis_pickaplace/paper/r2d_ms_control_2026-08-15.md.
#
# NOTE: no eval_genesis stage here -- ManiSkill has no one-world-per-process
# constraint, so eval runs IN-LOOP (eval_episode_num>0, every trainer.eval_every
# steps). eval_ms.py is the fresh-process CONFIRMATION path for a near-bar seed.

#SBATCH -J r2d-ms
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=1-00:00:00
#SBATCH --output=r2d_ms_%j.out
#SBATCH --error=r2d_ms_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export MUJOCO_GL=egl PYTHONUNBUFFERED=1

SEED=${SEED:-0}
CONFIG=${CONFIG:-maniskill_pickcube}
STEPS=${STEPS:-3.02e5}
VENV=${VENV:-$HOME/r2d_ms_venv}
_LAB=/cluster/tufts/shortlab/$USER
R2D_DIR=${R2D_DIR:-$([ -d "$_LAB/r2dreamer" ] && echo "$_LAB/r2dreamer" || echo "$HOME/r2dreamer")}
DV3_DIR=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
DEMO_DIR=${DEMO_DIR:-$DV3_DIR/demonstrations/maniskill_teleop_sparse100}
LOGDIR=${LOGDIR:-$R2D_DIR/runs/ms_pickcube_s$SEED}
RUN_NAME=$(basename "$LOGDIR"); RUNS_DIR=$(dirname "$LOGDIR")
PY="$VENV/bin/python"

RESUME=""
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$LOGDIR/latest.pt" ]; then
  RESUME="+resume=true"   # WARM restart: weights+optims only (buffer/step reset)
  echo "== requeued (restart #${SLURM_RESTART_COUNT}): WARM RESTART from $LOGDIR/latest.pt"
fi

TRAIN_CMD=("$PY" train.py "env=$CONFIG" "seed=$SEED" "env.steps=$STEPS" \
  "env.env_num=${ENV_NUM:-4}" "env.eval_episode_num=${EVAL_EPS:-10}" \
  "env.demo_dir=$DEMO_DIR" "buffer.max_size=${BUFFER_MAX:-3.2e5}" \
  "trainer.pretrain=${PRETRAIN:-100}" "trainer.save_every=${SAVE_EVERY:-5e4}" \
  "logdir=$LOGDIR" $RESUME ${EXTRA:-})

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] cd $R2D_DIR"; echo "[dry] ${TRAIN_CMD[*]}"
  echo "[dry] demo_dir=$DEMO_DIR logdir=$LOGDIR venv=$VENV"; exit 0
fi

# --- preflight: fail immediately with the remedy ----------------------------------
[ -x "$PY" ] || { echo "FATAL: no python at $PY -- run: bash cluster/install_r2dreamer_ms.sh $VENV"; exit 1; }
[ -f "$R2D_DIR/envs/maniskill.py" ] || { echo "FATAL: $R2D_DIR lacks the ManiSkill port (UNCOMMITTED upstream -- rsync the dev-box tree)"; exit 1; }
[ -f "$R2D_DIR/configs/env/$CONFIG.yaml" ] || { echo "FATAL: no configs/env/$CONFIG.yaml in $R2D_DIR"; exit 1; }
[ -d "$DEMO_DIR" ] || { echo "FATAL: demo dir missing: $DEMO_DIR (rsync from <devbox>:~/workspace/dreamerv3-torch/demonstrations/ -- gitignored, rsync ONLY)"; exit 1; }
N_NPZ=$(ls "$DEMO_DIR"/*.npz 2>/dev/null | wc -l)
[ "$N_NPZ" -gt 0 ] || { echo "FATAL: $DEMO_DIR has no *.npz"; exit 1; }
mkdir -p "$LOGDIR"

# Double-submission guard (dDP_s4 dual-writer incident, 08-12).
if [ -f "$LOGDIR/.claim" ]; then
  OTHER=$(cat "$LOGDIR/.claim")
  if [ "$OTHER" != "${SLURM_JOB_ID:-none}" ] && squeue -h -j "$OTHER" -t RUNNING 2>/dev/null | grep -q .; then
    echo "FATAL: job $OTHER is already RUNNING on $LOGDIR (double-submission guard)"; exit 1
  fi
fi
echo "${SLURM_JOB_ID:-none}" > "$LOGDIR/.claim"

echo "== r2dreamer MS control $CONFIG seed $SEED start $(date) demo=$DEMO_DIR ($N_NPZ eps) logdir=$LOGDIR"

# --- background wandb sync (idempotent; safe on the live run) ---------------------
(
  while true; do
    sleep 1800
    ( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
      >> "$LOGDIR/wandb_sync.log" 2>&1 || true
  done
) & SYNC_PID=$!

# --- archive every periodic checkpoint (decision points are 50k multiples) --------
# No in-loop eval of archives is needed here (unlike genesis): the trainer
# evaluates itself every eval_every steps. Archives exist so a near-bar seed can
# be re-measured with eval_ms.py in a fresh process.
(
  T0=0
  while true; do
    sleep 120
    T1=$(stat -c %Y "$LOGDIR/latest.pt" 2>/dev/null || echo 0)
    if [ "$T1" != "$T0" ] && [ "$T1" -gt 0 ]; then
      T0=$T1; sleep 15; cp "$LOGDIR/latest.pt" "$LOGDIR/ckpt_$T1.pt"
    fi
  done
) & ARCH_PID=$!
trap 'kill $SYNC_PID $ARCH_PID 2>/dev/null || true' EXIT

( cd "$R2D_DIR" && "${TRAIN_CMD[@]}" ) 2>&1 | tee -a "$LOGDIR/run.log"

kill $SYNC_PID $ARCH_PID 2>/dev/null || true
( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
  >> "$LOGDIR/wandb_sync.log" 2>&1 || true

# --- fresh-process confirmation on the final checkpoint (n=20, deterministic) -----
if [ -z "${NO_EVAL:-}" ]; then
  ( cd "$R2D_DIR" && "$PY" eval_ms.py --checkpoint "$LOGDIR/latest.pt" --episodes 20 \
      --mode mode --device cuda:0 --append-metrics "$LOGDIR" ) 2>&1 | tee "$LOGDIR/eval.log" \
    || echo "WARN: final eval FAILED"
fi
echo "== done $(date)"
