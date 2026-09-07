#!/bin/bash
# Robomimic leg: r2dreamer (state-input WM) on robosuite Can (plan §4). One seed per GPU, env-var driven.
#   ARM=PH200 SEED=0 sbatch cluster/robomimic/sbatch_r2d_robo.sh
# Env: ARM SEED ONLINE (online decisions; 500000 = budget of record, the ONE registered extension is 1000000)
#      TAG EVAL_EPISODES (50) DRYRUN=1 [extra hydra overrides as positional args]
# env.steps = demo prefill rows (repeat.json total_rows, spent from the budget by design) + ONLINE; buffer.max_size =
# 2*rows + ONLINE so no demo row is ever evicted and the demo_duplicate bound holds. Tree = $LAB/robomimic_r2d (copy),
# venv = $LAB/r2d_venv_robo. Requeue restarts CLEAN. Evals (mode + sample) on the 50-state bank in fresh processes.
#SBATCH -J robo_r2d
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --exclude=pax077
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --gres=gpu:1
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 0-16:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/robomimic_runs/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02; R2D=$LAB/robomimic_r2d; PY=$LAB/r2d_venv_robo/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl
export TORCHINDUCTOR_CACHE_DIR=$LAB/robomimic_runs/inductor_cache/${SLURM_JOB_ID:-local} PYTHONPATH=$R2D   # PYTHONPATH: r2d_venv carries an editable r2dreamer install pointing at the ORIGINAL tree; the copy must win
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}; ONLINE=${ONLINE:-500000}; TAG=${TAG:-}; EVAL_EPISODES=${EVAL_EPISODES:-50}
case "$ARM" in PH200|MH200|MG200s|MH300|MGall|PH200pb|none) ;; *) echo "FATAL: ARM=$ARM"; exit 1 ;; esac
DEMO=$LAB/robomimic_data/arms/$ARM/r2d
NAME=r2d_${ARM}${TAG:+_$TAG}_s${SEED}; LOGDIR=$LAB/robomimic_runs/r2d/$NAME
if [ "$ARM" = none ]; then ROWS=0; DEMO_OVR=(); else
  [ -f "$DEMO/repeat.json" ] || { echo "FATAL: demo dir missing: $DEMO (convert_arms.py)"; exit 1; }
  ROWS=$(python3 -c 'import json,sys; m=json.load(open(sys.argv[1])); assert m["action_repeat"]==1 and m["state_dim"]==23 and abs(m["terminal_reward"]-1.0)<1e-9, m; print(int(m["total_rows"]))' "$DEMO/repeat.json")
  DEMO_OVR=(env.demo_dir=$DEMO)
fi
STEPS=$((ROWS + ONLINE)); MAXSIZE=$((2 * ROWS + ONLINE))
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
echo "# $(date -Is) host=$(hostname) node=${SLURM_NODELIST:-} job=${SLURM_JOB_ID:-} arm=$ARM seed=$SEED prefill_rows=$ROWS online=$ONLINE env.steps=$STEPS buffer.max_size=$MAXSIZE restart=${SLURM_RESTART_COUNT:-0}"
CMD=("$PY" train.py env=robosuite_can_state seed=$SEED env.steps=$STEPS "${DEMO_OVR[@]}" buffer.max_size=$MAXSIZE logdir=$LOGDIR "$@")
if [ -n "${DRYRUN:-}" ]; then echo "[dry] cd $R2D && ${CMD[*]}"; exit 0; fi
cd $R2D; if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$LOGDIR" ]; then echo "# requeued: clearing partial $LOGDIR, restarting clean"; rm -rf "$LOGDIR"; fi
mkdir -p "$LOGDIR" $LAB/robomimic_runs/slurm
echo "${CMD[*]}"
set +e
"${CMD[@]}" 2>&1 | grep --line-buffered -vE "Warning|warn" | tee $LOGDIR/console.log | grep --line-buffered -E "^\[|Traceback|Error|FATAL|Demo prefill|Encoder|Step accounting" | cut -c1-220
TRAIN_RC=${PIPESTATUS[0]}
set -e
echo "# train rc=$TRAIN_RC $(date -Is)"
[ "$TRAIN_RC" -eq 0 ] || { echo "FATAL: training exited $TRAIN_RC"; exit 1; }
[ -f $LOGDIR/latest.pt ] || { echo "FATAL: no latest.pt"; exit 1; }
LAST=$(tail -c 4000 $LOGDIR/metrics.jsonl 2>/dev/null | grep -oE "\"step\": [0-9]+" | tail -1 | grep -oE "[0-9]+" || echo 0)
if [ "$ONLINE" -gt 10000 ]; then [ "${LAST:-0}" -ge $((STEPS - 5000)) ] || { echo "FATAL: last logged step $LAST < $((STEPS - 5000)) -- training did not reach its budget"; exit 1; }; else echo "# smoke (ONLINE=$ONLINE <= 10k): budget check skipped; last logged step $LAST"; fi
for MODE in mode sample; do
  CUDA_VISIBLE_DEVICES="" "$PY" eval_robosuite.py --checkpoint $LOGDIR/latest.pt --mode $MODE --episodes $EVAL_EPISODES --seed 0 --out $LOGDIR/eval_bank50_$MODE --device cpu 2>&1 | { grep -E "^\[eval|Traceback|Error|FATAL" || true; } | tail -3
done
python3 - "$LOGDIR" <<'PY'
import json, sys, os
o = sys.argv[1]; r = {m: json.load(open(f"{o}/eval_bank50_{m}/metrics.json")) for m in ("mode", "sample") if os.path.exists(f"{o}/eval_bank50_{m}/metrics.json")}
print("R2D-RESULT " + os.path.basename(o) + " " + " ".join(f"{m}={v['n_success']}/{v['episodes']}" for m, v in r.items()))
PY
echo "# DONE $NAME $(date -Is)"
