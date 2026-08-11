#!/bin/bash
# r2dreamer training on genesis: ONE run per GPU, env-var driven.
#
# --- Submit (from the genesis_pickaplace checkout root) ---------------------------
#   cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
#   CONFIG=genesis_pick_v3       SEED=0 sbatch cluster/sbatch_r2dreamer.sh   # msparity (abs joint)
#   CONFIG=genesis_pick_v4_delta SEED=0 sbatch cluster/sbatch_r2dreamer.sh   # delta-joint
#
# Env vars (defaults):
#   CONFIG    genesis_pick_v3 | genesis_pick_v4_delta (hydra env group name)
#   SEED      0
#   STEPS     3e6                 total SIM steps incl. prefill (v3/v4: repeat 4)
#   DEMO_DIR  auto per CONFIG: $DV3_DIR/demonstrations/genesis_pick_pruned for v3,
#             ..._delta for v4_delta (encodings MUST match the config -- override
#             only if you know why). DV3_DIR default: ../dreamerv3-torch.
#   LOGDIR    $R2D_DIR/runs/${CONFIG#genesis_}_s$SEED   (runs/ tree; wandb run name
#             = its basename via sync_runs_to_wandb.py)
#   VENV      ~/r2d_venv          (from cluster/install_r2dreamer.sh)
#   R2D_DIR   ~/r2dreamer         (rsynced -- the port is uncommitted)
#   ENV_NUM 6  BUFFER_MAX 450000  REINJECT 300000  PRETRAIN 1000  EXTRA (raw hydra
#   overrides)  EVAL_EPS 15  EVAL_MAX_STEPS 400 (SIM steps; v3/v4 tl)  NO_EVAL
#   DRYRUN=1 bash cluster/sbatch_r2dreamer.sh   # print the resolved plan, no submit
#
# wandb (entity jambotime, project r2dreamer_genesis -- hardcoded in the repo's
# sync/eval scripts): train.py itself logs to tensorboard/metrics.jsonl only; a
# background loop runs sync_runs_to_wandb.py --only <run> every 30 min (idempotent,
# resume-safe on live runs) + a final sync, then eval_genesis.py --wandb uploads
# the honest post-train eval (videos + scalars). Needs WANDB_API_KEY in the
# environment or a prior `wandb login` on the cluster.
#
# PREEMPT/REQUEUE: --requeue resubmits on preemption. r2dreamer periodic-
# checkpoints latest.pt (trainer.save_every 1e5, atomic). train.py resume is a
# WARM RESTART ONLY (+resume=true, added 2026-08-10): agent weights + optimizer
# state reload from latest.pt, but the replay buffer is NOT persisted -- the
# buffer restarts at demo prefill and the step counter re-runs the full STEPS
# budget. Learned parameters survive preemption; buffer contents and step
# progress do not. Requeues therefore extend wall time rather than losing the
# model; budget --time accordingly.
#
# NOTE: #SBATCH lines are parsed by sbatch, NOT shell-expanded -- no $VARs in
# them; override at submit time (sbatch --time=... --mem=...) if needed.

#SBATCH -J r2d-train
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
# Training worlds are CPU-backend (one Genesis world per spawn worker); the GPU
# runs only the dreamer model -> any verified type is fine (l40s|a100|l40|h200
# all pass cluster/verify_env.sh incl. the world build).
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=2-00:00:00
# 3e6 sim steps ran ~13 h on the dev box (64 sim-steps/s, CPU-bound stepping);
# margin for slower cluster cores + warm-restart budget re-runs after preemption.
#SBATCH --output=r2d_train_%j.out
#SBATCH --error=r2d_train_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT MUJOCO_GL=egl PYTHONUNBUFFERED=1

CONFIG=${CONFIG:-genesis_pick_v3}
SEED=${SEED:-0}
STEPS=${STEPS:-3e6}
VENV=${VENV:-$HOME/r2d_venv}
R2D_DIR=${R2D_DIR:-$HOME/r2dreamer}
DV3_DIR=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
case "$CONFIG" in
  *v5*)    DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta25 ;;  # cap 0.025 sets
  *delta*) DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta ;;
  *)       DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned ;;
esac
DEMO_DIR=${DEMO_DIR:-$DEMO_DEFAULT}
LOGDIR=${LOGDIR:-$R2D_DIR/runs/${CONFIG#genesis_}_s$SEED}
RUN_NAME=$(basename "$LOGDIR"); RUNS_DIR=$(dirname "$LOGDIR")
PY="$VENV/bin/python"

# Warm restart on requeue only (see header): fresh submissions must NOT pick up
# a stale latest.pt from a reused LOGDIR.
RESUME=""
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$LOGDIR/latest.pt" ]; then
  RESUME="+resume=true"
  echo "== requeued (restart #${SLURM_RESTART_COUNT}): WARM RESTART from $LOGDIR/latest.pt"
  echo "   (weights+optims reload; buffer + step counter start over -- by design)"
fi

TRAIN_CMD=("$PY" train.py "env=$CONFIG" "seed=$SEED" "env.steps=$STEPS" \
  "env.env_num=${ENV_NUM:-6}" "env.demo_dir=$DEMO_DIR" \
  ${REINJECT:+"env.demo_reinject_every=$REINJECT"} \
  ${DUPLICATE:+"env.demo_duplicate=$DUPLICATE"} \
  ${SAVE_EVERY:+"trainer.save_every=$SAVE_EVERY"} \
  "buffer.max_size=${BUFFER_MAX:-450000}" \
  "trainer.pretrain=${PRETRAIN:-1000}" "logdir=$LOGDIR" $RESUME ${EXTRA:-})
# REINJECT/DUPLICATE are passed ONLY when set: genesis_pick_v5d4_delta BAKES the
# champion values (150k / 4x); an unconditional sbatch default would silently
# override the config (the silent-defaults bug family).

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] cd $R2D_DIR"
  echo "[dry] ${TRAIN_CMD[*]}"
  echo "[dry] demo_dir=$DEMO_DIR logdir=$LOGDIR venv=$VENV wandb-run=$RUN_NAME"
  exit 0
fi

# --- preflight: fail immediately with the remedy ----------------------------------
[ -x "$PY" ] || { echo "FATAL: no python at $PY -- run: bash cluster/install_r2dreamer.sh $VENV"; exit 1; }
[ -f "$R2D_DIR/envs/genesis.py" ] || { echo "FATAL: $R2D_DIR lacks the genesis port (UNCOMMITTED -- rsync the dev-box working tree, not a clone)"; exit 1; }
[ -f "$R2D_DIR/configs/env/$CONFIG.yaml" ] || { echo "FATAL: no configs/env/$CONFIG.yaml in $R2D_DIR"; exit 1; }
[ -d "$DEMO_DIR" ] || { echo "FATAL: demo dir missing: $DEMO_DIR (rsync from <devbox>:~/workspace/dreamerv3-torch/demonstrations/ -- gitignored, rsync ONLY)"; exit 1; }
N_NPZ=$(ls "$DEMO_DIR"/*.npz 2>/dev/null | wc -l)
[ "$N_NPZ" -gt 0 ] || { echo "FATAL: $DEMO_DIR has no *.npz"; exit 1; }
mkdir -p "$LOGDIR"
echo "== r2dreamer $CONFIG seed $SEED start $(date) demo=$DEMO_DIR ($N_NPZ eps) logdir=$LOGDIR"

# --- background wandb sync (idempotent; safe on the live run) ---------------------
(
  while true; do
    sleep 1800
    ( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
      >> "$LOGDIR/wandb_sync.log" 2>&1 || true
  done
) & SYNC_PID=$!
trap 'kill $SYNC_PID 2>/dev/null || true' EXIT

# --- checkpoint lottery coverage (2026-08-11): training is BISTABLE (diagnosed
# lambda-return explosion) -- final/latest checkpoints are a PHASE LOTTERY.
# Archive every latest.pt write, eval it (15 eps, sample, seed 0), keep only the
# best 2 + the newest (checkpoints are 145MB), record scores to ckpt_scores.tsv.
(
  T0=0
  while true; do
    sleep 120
    T1=$(stat -c %Y "$LOGDIR/latest.pt" 2>/dev/null || echo 0)
    if [ "$T1" != "$T0" ] && [ "$T1" -gt 0 ]; then
      T0=$T1; sleep 15
      CK="$LOGDIR/ckpt_$T1.pt"; cp "$LOGDIR/latest.pt" "$CK"
      # DEVICE CPU, deliberately: a second CUDA process on the training GPU
      # kills the compiled (cudagraph) trainer -- all four dH_R2D jobs died at
      # ~230k steps, exactly when the first concurrent cuda eval ran
      # (2026-08-11). The model is small; CPU eval just takes ~20 min.
      P=$( (cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$CK" \
            --episodes 15 --mode sample --seed 0 --device cpu --torch-threads 2 \
            --append-metrics "$LOGDIR" 2>/dev/null) \
           | grep -oP 'picked \K[0-9.]+' | head -1 )
      echo -e "$CK\t${P:-ERR}" >> "$LOGDIR/ckpt_scores.tsv"
      echo "== ckpt eval: $CK picked=${P:-ERR}"
      # prune: keep best 2 by score + the newest file
      NEWEST=$(ls -t "$LOGDIR"/ckpt_*.pt | head -1)
      KEEP=$( { sort -k2 -rn "$LOGDIR/ckpt_scores.tsv" | head -2 | cut -f1; echo "$NEWEST"; } | sort -u )
      for F in "$LOGDIR"/ckpt_*.pt; do echo "$KEEP" | grep -qxF "$F" || rm -f "$F"; done
    fi
  done
) & ARCH_PID=$!
trap 'kill $SYNC_PID $ARCH_PID 2>/dev/null || true' EXIT

# --- train ------------------------------------------------------------------------
( cd "$R2D_DIR" && "${TRAIN_CMD[@]}" ) 2>&1 | tee -a "$LOGDIR/run.log"

kill $SYNC_PID 2>/dev/null || true
( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
  >> "$LOGDIR/wandb_sync.log" 2>&1 || true

kill $ARCH_PID 2>/dev/null || true

# --- honest post-train eval + BEST-CHECKPOINT CONFIRMATION -------------------------
# Protocol (pre-registered): the in-run loop SELECTS the best checkpoint on its
# seed-0 eval; the headline number is the mean of x3 CONFIRMATION evals on fresh
# seeds 1-3 (+1 mode eval) -- selection and confirmation on independent draws.
if [ -z "${NO_EVAL:-}" ]; then
  ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/latest.pt" \
      --episodes "${EVAL_EPS:-15}" --max-steps "${EVAL_MAX_STEPS:-400}" \
      --mode sample --seed 0 --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee "$LOGDIR/eval.log" \
    || echo "WARN: final eval FAILED"
  BEST=$(sort -k2 -rn "$LOGDIR/ckpt_scores.tsv" 2>/dev/null | head -1 | cut -f1)
  if [ -n "$BEST" ] && [ -f "$BEST" ]; then
    cp "$BEST" "$LOGDIR/BEST_selected.pt"
    echo "== confirming best checkpoint $BEST (selected on seed-0 eval)"
    for ES in 1 2 3; do
      ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/BEST_selected.pt" \
          --episodes 15 --mode sample --seed $ES --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee -a "$LOGDIR/confirm.log"
    done
    ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/BEST_selected.pt" \
        --episodes 15 --mode mode --seed 0 --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee -a "$LOGDIR/confirm.log"
    grep -h "15 episodes" "$LOGDIR/confirm.log" | tail -4
  else
    echo "WARN: no scored checkpoints -- lottery coverage produced nothing (see ckpt_scores.tsv)"
  fi
fi
echo "== r2dreamer $CONFIG seed $SEED done $(date)"
