#!/bin/bash
# Robomimic leg: RLPD on robosuite Can (plan §4). One seed per GPU, env-var driven.
#   ARM=PH200 SEED=0 sbatch cluster/robomimic/sbatch_rlpd_robo.sh
# Env: ARM (PH200|MH200|MG200s|MH300|MGall|PH200pb|none) SEED STEPS (100000 decisions = budget of record; the ONE
#      registered extension is 300000) TAG (run-name suffix) EVAL_EPISODES (50) DRYRUN=1
# Train in $LAB/robo_venv; fresh-process eval on the 50-state bank (mode + sample) writes metrics.json under the run.
# Requeue restarts CLEAN (no partial-checkpoint resume; disclosed in the log).
#SBATCH -J robo_rlpd
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --exclude=pax077
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --gres=gpu:1
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=32g
#SBATCH -t 0-10:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/robomimic_runs/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02; PY=$LAB/robo_venv/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl OMP_NUM_THREADS=4
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}; STEPS=${STEPS:-100000}; TAG=${TAG:-}; EVAL_EPISODES=${EVAL_EPISODES:-50}
case "$ARM" in PH200|MH200|MG200s|MH300|MGall|PH200pb|MG718s|MG200s_re|MG200s_sm|MG200s_smm|MH200_re|MH200_rough|none) ;; *) echo "FATAL: ARM=$ARM"; exit 1 ;; esac
DEMO=$LAB/robomimic_data/arms/$ARM/rlpd/transitions.npz; [ "$ARM" = none ] && DEMO=none
NAME=rlpd_${ARM}${TAG:+_$TAG}_s${SEED}; OUT=$LAB/robomimic_runs/rlpd/$NAME   # ARM=none -> rlpd_none_s<k> = G2b no-demo control (demo_batch 0)
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic
echo "# $(date -Is) host=$(hostname) node=${SLURM_NODELIST:-} job=${SLURM_JOB_ID:-} arm=$ARM seed=$SEED steps=$STEPS demo=$DEMO out=$OUT restart=${SLURM_RESTART_COUNT:-0}"
[ "$DEMO" = none ] || [ -f "$DEMO" ] || { echo "FATAL: demo file missing: $DEMO (convert_arms.py)"; exit 1; }
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
if [ -n "${DRYRUN:-}" ]; then echo "[dry] $PY $B/train_rlpd_robosuite.py --demo $DEMO --arm $ARM --steps $STEPS --seed $SEED --out $OUT --device cuda"; exit 0; fi
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$OUT" ]; then echo "# requeued: clearing partial $OUT and restarting clean"; rm -rf "$OUT"; fi
mkdir -p "$OUT" $LAB/robomimic_runs/slurm
nvidia-smi --query-gpu=name --format=csv,noheader | head -1
$PY $B/train_rlpd_robosuite.py --demo "$DEMO" --arm "$ARM" --steps "$STEPS" --seed "$SEED" --out "$OUT" --device cuda 2>&1 | tee $OUT/train.log | grep --line-buffered -E "^\[|Traceback|Error|Q-WATCHDOG" | cut -c1-220
[ -f $OUT/rlpd_final.zip ] || { echo "FATAL: no rlpd_final.zip"; exit 1; }
for MODE in mode sample; do
  CUDA_VISIBLE_DEVICES="" $PY $B/eval_rlpd_robosuite.py --checkpoint $OUT/rlpd_final.zip --mode $MODE --episodes $EVAL_EPISODES --out $OUT/eval_bank50_$MODE 2>&1 | { grep -E "^\[eval|Traceback|Error|FATAL" || true; } | tail -3
done
$PY - "$OUT" <<'PY'
import json, sys, os
o = sys.argv[1]; r = {m: json.load(open(f"{o}/eval_bank50_{m}/metrics.json")) for m in ("mode", "sample") if os.path.exists(f"{o}/eval_bank50_{m}/metrics.json")}
print("RLPD-RESULT " + os.path.basename(o) + " " + " ".join(f"{m}={v['n_success']}/{v['episodes']}" for m, v in r.items()))
PY
echo "# DONE $NAME $(date -Is)"
