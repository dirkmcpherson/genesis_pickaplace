#!/bin/bash
# Robomimic leg: EVAL-ONLY for a DP run whose training finished but whose bank evaluation is missing
# (recovery path after the 2026-09-07 filesystem-full incident). Scores the run's LAST checkpoint on bank_can50.
#   RUN=dp_MG200s_s1 sbatch cluster/robomimic/sbatch_dp_eval_robo.sh
# Env: RUN (run dir name under $LAB/robomimic_runs/dp) EVAL_EPISODES (50) DRYRUN=1
#SBATCH -J robo_dp_eval
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --exclude=pax077
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --gres=gpu:1
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=24g
#SBATCH -t 0-02:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/robomimic_runs/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02; V=$LAB/robo_venv; PY=$V/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl
RUN=${RUN:?set RUN}; EVAL_EPISODES=${EVAL_EPISODES:-50}
OUT=$LAB/robomimic_runs/dp/$RUN; CKPT=$OUT/checkpoints/last/pretrained_model
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic
MIN_FREE_GB=${MIN_FREE_GB:-100}
_free=$(df -BG --output=avail /cluster/tufts/shortlab | tail -1 | tr -dc "0-9")
if [ "${_free:-0}" -lt "$MIN_FREE_GB" ]; then echo "FATAL: only ${_free}G free on /cluster/tufts/shortlab (need ${MIN_FREE_GB}G)"; exit 1; fi
echo "# $(date -Is) host=$(hostname) job=${SLURM_JOB_ID:-} run=$RUN ckpt=$CKPT disk=${_free}G"
[ -d "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT"; exit 1; }
STEP=$(readlink -f $OUT/checkpoints/last | sed 's#.*/checkpoints/##')
WANT=$($PY -c 'import json,sys; print(json.load(open(sys.argv[1]))["steps"])' $CKPT/train_config.json)
[ "$STEP" = "$(printf %06d $WANT)" ] || { echo "FATAL: last checkpoint is step $STEP but the run's budget is $WANT -- training did not finish; retrain instead of evaluating"; exit 1; }
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
if [ -n "${DRYRUN:-}" ]; then echo "[dry] $PY $B/eval_dp_robosuite.py --checkpoint $CKPT --episodes $EVAL_EPISODES --out $OUT/eval_bank50_sample"; exit 0; fi
$PY $B/eval_dp_robosuite.py --checkpoint "$CKPT" --episodes "$EVAL_EPISODES" --out $OUT/eval_bank50_sample 2>&1 | { grep -E "^\[eval|Traceback|Error" || true; } | tail -3
$PY - "$OUT" <<'PY'
import json, sys, os
o = sys.argv[1]; v = json.load(open(f"{o}/eval_bank50_sample/metrics.json"))
print(f"DP-RESULT {os.path.basename(o)} sample={v['n_success']}/{v['episodes']}")
PY
echo "# DONE $RUN $(date -Is)"
