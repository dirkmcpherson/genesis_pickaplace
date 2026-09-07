#!/bin/bash
# Robomimic leg: Diffusion Policy (lerobot, state-only) on robosuite Can -- the DP amendment (plan §A1).
#   ARM=PH200 SEED=0 sbatch cluster/robomimic/sbatch_dp_robo.sh
# Env: ARM SEED STEPS (100000 gradient steps, batch 64 = cluster/sbatch_dp.sh's recipe of record) TAG EVAL_EPISODES DRYRUN=1
# Dataset = $LAB/robomimic_data/arms/<ARM>/lerobot (convert_arms.py, fps 20, observation.state 9 + environment_state 14).
# Requeue resumes via lerobot's own train_config.json (as sbatch_dp.sh does). Eval: LAST checkpoint on the 50-state bank.
#SBATCH -J robo_dp
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
LAB=/cluster/tufts/shortlab/jstale02; V=$LAB/robo_venv; PY=$V/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}; STEPS=${STEPS:-100000}; TAG=${TAG:-}; EVAL_EPISODES=${EVAL_EPISODES:-50}
case "$ARM" in PH200|MH200|MG200s|MH300|MGall|PH200pb) ;; *) echo "FATAL: ARM=$ARM"; exit 1 ;; esac
DATASET=$LAB/robomimic_data/arms/$ARM/lerobot
NAME=dp_${ARM}${TAG:+_$TAG}_s${SEED}; OUT=$LAB/robomimic_runs/dp/$NAME
SAVE_FREQ=$(( STEPS / 5 )); [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic
echo "# $(date -Is) host=$(hostname) node=${SLURM_NODELIST:-} job=${SLURM_JOB_ID:-} arm=$ARM seed=$SEED steps=$STEPS dataset=$DATASET out=$OUT restart=${SLURM_RESTART_COUNT:-0}"
[ -f "$DATASET/meta/info.json" ] || { echo "FATAL: lerobot dataset missing: $DATASET (convert_arms.py --targets lerobot)"; exit 1; }
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
TRAIN="$V/bin/lerobot-train"; [ -x "$TRAIN" ] || TRAIN="$PY -m lerobot.scripts.lerobot_train"
FLAGS=(--dataset.repo_id="local/robomimic_can_${ARM}" --dataset.root="$DATASET" --policy.type=diffusion --policy.push_to_hub=false
       --seed="$SEED" --output_dir="$OUT" --batch_size=64 --steps="$STEPS" --save_freq="$SAVE_FREQ" --job_name="$NAME" --wandb.enable=false)
if [ -n "${DRYRUN:-}" ]; then echo "[dry] $TRAIN ${FLAGS[*]}"; exit 0; fi
mkdir -p $LAB/robomimic_runs/slurm
nvidia-smi --query-gpu=name --format=csv,noheader | head -1
TC="$OUT/checkpoints/last/pretrained_model/train_config.json"
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$TC" ]; then
  echo "# requeued (restart ${SLURM_RESTART_COUNT}): RESUMING via $TC"; $TRAIN --config_path="$TC" --resume=true
else
  rm -rf "$OUT"; $TRAIN "${FLAGS[@]}"
fi
CKPT=$OUT/checkpoints/last/pretrained_model; [ -d "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT"; exit 1; }
for d in "$OUT"/checkpoints/*/; do
  $PY - "$d" "$ARM" "$SEED" "$STEPS" "$DATASET" <<'PY'
import json, pathlib as pl, sys, hashlib
d, arm, seed, steps, ds = sys.argv[1:6]
info = json.load(open(pl.Path(ds) / "meta" / "info.json"))
side = dict(learner="dp", task="robosuite_can", arm=arm, seed=int(seed), steps=int(steps), steps_unit="grad_steps", batch_size=64,
            dataset=ds, dataset_fps=info["fps"], dataset_episodes=info["total_episodes"], dataset_frames=info["total_frames"],
            horizon=400, action_space="osc_pose_delta[-1,1]", proprio_dim=9, hdf5=json.load(open(pl.Path(ds) / "robomimic_source.json"))["hdf5"])
(pl.Path(d) / "dp_sidecar.json").write_text(json.dumps(side))
PY
done
$PY $B/eval_dp_robosuite.py --checkpoint $CKPT --episodes $EVAL_EPISODES --out $OUT/eval_bank50_sample 2>&1 | grep -E "^\[eval|Traceback|Error" | tail -3
$PY - "$OUT" <<'PY'
import json, sys, os
o = sys.argv[1]; v = json.load(open(f"{o}/eval_bank50_sample/metrics.json"))
print(f"DP-RESULT {os.path.basename(o)} sample={v['n_success']}/{v['episodes']}")
PY
echo "# DONE $NAME $(date -Is)"
