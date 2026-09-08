#!/bin/bash
# Robomimic leg: BC-RNN positive control (plan §4/§5 G1) -- robomimic's own low-dim BC-RNN config via robomimic's
# own trainer, on the masked hdf5 copy with filter_key = ARM; LAST checkpoint scored on the 50-state bank.
#   ARM=PH200 SEED=0 sbatch cluster/robomimic/sbatch_bcrnn_robo.sh
# Env: ARM SEED EPOCHS (2000 = paper) TAG EVAL_EPISODES DRYRUN=1. robomimic's train.py swallows exceptions and exits 0:
# the run is gated on its "finished run successfully!" line. Requeue restarts clean.
#SBATCH -J robo_bcrnn
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --exclude=pax077
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --gres=gpu:1
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=32g
#SBATCH -t 0-08:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/robomimic_runs/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02; PY=$LAB/robo_venv/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl
# Array mode (the 3-source x 3-seed control under the <= 4-GPU-jobs-in-flight rule): sbatch --array=0-8%N ... maps
# task id -> ARM = (PH200 MH200 MG200s)[id / 3], SEED = id % 3 when ARM/SEED are not given explicitly.
if [ -n "${SLURM_ARRAY_TASK_ID:-}" ] && [ -z "${ARM:-}" ]; then _A=(PH200 MH200 MG200s); ARM=${_A[$((SLURM_ARRAY_TASK_ID / 3))]}; SEED=$((SLURM_ARRAY_TASK_ID % 3)); fi
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}; EPOCHS=${EPOCHS:-2000}; TAG=${TAG:-}; EVAL_EPISODES=${EVAL_EPISODES:-50}
case "$ARM" in PH200|MH200|MG200s|MH300|MGall) ;; *) echo "FATAL: ARM=$ARM"; exit 1 ;; esac
NAME=bcrnn_${ARM}${TAG:+_$TAG}_s${SEED}; OUT=$LAB/robomimic_runs/bcrnn/$NAME
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic
MIN_FREE_GB=${MIN_FREE_GB:-100}
_free=$(df -BG --output=avail /cluster/tufts/shortlab | tail -1 | tr -dc "0-9")
if [ "${_free:-0}" -lt "$MIN_FREE_GB" ]; then echo "FATAL: only ${_free}G free on /cluster/tufts/shortlab (need ${MIN_FREE_GB}G) -- refusing to start (2026-09-07 filesystem-full incident)"; exit 1; fi
echo "# disk: ${_free}G free on /cluster/tufts/shortlab"
echo "# $(date -Is) host=$(hostname) node=${SLURM_NODELIST:-} job=${SLURM_JOB_ID:-} arm=$ARM seed=$SEED epochs=$EPOCHS out=$OUT restart=${SLURM_RESTART_COUNT:-0}"
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
if [ -n "${DRYRUN:-}" ]; then echo "[dry] make_bcrnn_config.py --arm $ARM --seed $SEED --num-epochs $EPOCHS -> $OUT/config.json; robomimic train.py --config $OUT/config.json; eval LAST on bank"; exit 0; fi
if [ -d "$OUT" ]; then echo "# clearing $OUT (restart ${SLURM_RESTART_COUNT:-0})"; rm -rf "$OUT"; fi
mkdir -p "$OUT/trained" $LAB/robomimic_runs/slurm
$PY $B/make_bcrnn_config.py --arm $ARM --seed $SEED --num-epochs $EPOCHS --out $OUT/config.json --output-dir $OUT/trained --name $NAME
TRAIN_PY=$($PY -c 'import robomimic, os; print(os.path.join(os.path.dirname(robomimic.__file__), "scripts", "train.py"))')
$PY $TRAIN_PY --config $OUT/config.json 2>&1 | tee $OUT/train.log | grep --line-buffered -E "Epoch [0-9]+0 |finished run|run failed|Traceback|Error" | cut -c1-200
grep -q "finished run successfully" $OUT/train.log || { echo "FATAL: robomimic train.py did not finish successfully (see $OUT/train.log)"; exit 1; }
# LAST = the highest epoch number, sorted on the BASENAME (sorting the full path on `_` fields picked epoch 950 in the
# first array, 2026-09-07 01:20 -- those evals were re-done at epoch 2000 by bcrnn_reeval_last.sh)
CKPT=$(ls -1 $OUT/trained/$NAME/*/models/model_epoch_*.pth | awk -F'model_epoch_' '{split($2,a,".pth"); print a[1]"\t"$0}' | sort -n | tail -1 | cut -f2)
[ -f "$CKPT" ] || { echo "FATAL: no checkpoint under $OUT/trained"; exit 1; }
echo "# LAST checkpoint: $CKPT"
CUDA_VISIBLE_DEVICES="" $PY $B/eval_bcrnn_robosuite.py --checkpoint "$CKPT" --arm $ARM --episodes $EVAL_EPISODES --out $OUT/eval_bank50 2>&1 | grep -E "^\[eval|Traceback|Error" | tail -3
$PY - "$OUT" <<'PY'
import json, sys, os
o = sys.argv[1]; v = json.load(open(f"{o}/eval_bank50/metrics.json"))
print(f"BCRNN-RESULT {os.path.basename(o)} last={v['n_success']}/{v['episodes']} epoch={v.get('epoch')}")
PY
echo "# DONE $NAME $(date -Is)"
