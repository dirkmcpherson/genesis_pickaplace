#!/bin/bash
# Machine-first arm, teacher T2 = reward-only SAC (RLPD recipe of record WITHOUT demonstrations
# + pick-scope potential shaping). paper/MACHINE_FIRST_PLAN_2026-09-07.md §1 T2 + amendment (a).
#
# Derived from cluster/sbatch_rlpd.sh (the RLPD launcher of record: train_rlpd.py, K archived
# ckpts, cluster/eval_sweep.sh fresh-process evals) with the demo-dir gates removed and
#   --no-demos --pick-shaping on --demo-shaping off      (train_rlpd.py, amendment (a))
# Everything else is the w3 RLPD recipe: gamma 0.99, UTD 10, E=10 Z=2, LN critics,
# delta_joint/target, action_repeat 4, train horizon 1200 sim steps, eval 1200, IC file
# baselines/eval_ics.json (rnd30 = the registered gate set; hold15 alongside).
#
# usage:  SEED=<s> [STEPS=400000] sbatch -p gpu cluster/mf/mf_sac_teacher.sbatch     (QOS normal)
#         STEPS are DECISIONS (x4 sim steps). Smoke: STEPS=2000.
# gate readout: SWEEP-HEADLINE line (final_rnd = LAST checkpoint, deterministic policy).
#
#SBATCH -J mf_sac_t2
#SBATCH -p gpu
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=2-00:00:00
#SBATCH --output=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
#SBATCH --error=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out

set -eo pipefail
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
cd "${GENESIS_PICKAPLACE_ROOT:=$LAB/mf_gp}"     # the machine-first checkout (branch worktree-machine-first-arm)
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1

SEED=${SEED:?set SEED}
STEPS=${STEPS:-400000}          # decisions (plan §1 T2: 4e5 decisions)
ACTION_REPEAT=4; TRAIN_HORIZON=1200; EVAL_HORIZON=1200
GAMMA=${GAMMA:-0.99}; UTD=${UTD:-10}
SIM_VARIANT=gc_kp4_riser3_shelf6
IC_FILE=${IC_FILE:-baselines/eval_ics.json}
WAVE=${WAVE:-mf_t2}
ARM=dRLteacher
OUT=baselines/rl/checkpoints/mf_sac_t2_s${SEED}${TAG:+_$TAG}
RUN_NAME="mf_sac_t2_s${SEED}${TAG:+_$TAG}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
# EVAL_ONLY=1: skip training, (re)score an existing run's checkpoints (post-hoc path; CPU job is fine)
EVAL_ONLY=${EVAL_ONLY:-0}
if [ "$EVAL_ONLY" != 1 ]; then
  [ -e "$OUT/rlpd_final.zip" ] && { echo "FATAL: $OUT already holds a final checkpoint"; exit 2; }
fi

TRAIN_ARGS=(--steps "$STEPS" --scope pick --action-mode delta_joint --delta-ref target
  --action-repeat "$ACTION_REPEAT" --train-max-steps "$TRAIN_HORIZON" --eval-max-steps "$EVAL_HORIZON"
  --gamma "$GAMMA" --backup-entropy off --per-member-ln off --pick-hold-reward off
  --utd "$UTD" --ensemble-size 10 --subset-size 2
  --no-demos --demo-format native --demo-shaping off --pick-shaping on --pick-shaping-terminal-zero on
  --demo-terminal-guard on --sim-variant "$SIM_VARIANT"
  --ckpt-fracs "${CKPT_FRACS:-0.25,0.5,0.75,1.0}"
  --out-dir "$OUT" --run-name "$RUN_NAME" --project genesis_paper
  --seed "$SEED" --device cuda)
REG_KNOBS=(steps="$STEPS" budget_unit=decisions scope=pick action_mode=delta_joint delta_ref=target
           action_repeat="$ACTION_REPEAT" train_horizon="$TRAIN_HORIZON" eval_horizon="$EVAL_HORIZON"
           gamma="$GAMMA" backup_entropy=off per_member_ln=off pick_hold_reward=off utd="$UTD"
           ensemble_size=10 subset_size=2 demo_batch=0 reward=dense pick_shaping=on
           demo_format=none demo_sha=none wave="$WAVE" no_demos=on
           demo_shaping=off pick_shaping_terminal_zero=on demo_terminal_guard=on sim_variant="$SIM_VARIANT")
SWEEP_COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --reward dense
              --wandb-run "$RUN_NAME" --wandb-project genesis_paper)

if [ "$EVAL_ONLY" = 1 ]; then
  echo "== $RUN_NAME EVAL_ONLY $(date -Is) node=$NODE_CLASS root=$GENESIS_PICKAPLACE_ROOT"
  module load anaconda/2025.06.0; conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"
else
echo "== $RUN_NAME start $(date -Is) node=$NODE_CLASS host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) root=$GENESIS_PICKAPLACE_ROOT git=$(git rev-parse --short HEAD) steps=$STEPS(decisions) gamma=$GAMMA utd=$UTD variant=$SIM_VARIANT"
printf '%s %s ' "$(date -Is)" "$RUN_NAME" >> $W/COMMANDS.log; printf '%q ' python baselines/rl/train_rlpd.py "${TRAIN_ARGS[@]}" >> $W/COMMANDS.log; echo " # job ${SLURM_JOB_ID:-local}" >> $W/COMMANDS.log

if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script mf_sac_teacher.sbatch --arm "$ARM" --seed "$SEED" \
    --demo-dir none --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script mf_sac_teacher.sbatch --arm "$ARM" --seed "$SEED" \
    --demo-dir none --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
else
  echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial out dir and starting clean"; rm -rf "$OUT"
fi

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-$LAB/condaenv/genesis}"
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'

python baselines/rl/train_rlpd.py "${TRAIN_ARGS[@]}"
echo "# train rc=$? $(date -Is)"
SLURM_OUT=$W/slurm/${SLURM_JOB_NAME}_${SLURM_JOB_ID}.out
n=$(grep -c "\[sim-variant\] $SIM_VARIANT" "$SLURM_OUT" || true); echo "# [sim-variant] lines: $n"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] line"; exit 3; }
grep -q -- '--no-demos: NO demonstration buffer' "$SLURM_OUT" || { echo "FATAL: the no-demos line is missing"; exit 3; }
grep -E '^\[cfg\] RLPD' "$SLURM_OUT" | grep -q 'demo_batch=0/256' || { echo "FATAL: demo_batch is not 0"; exit 3; }
grep -E '^\[cfg\] RLPD' "$SLURM_OUT" | grep -q 'pick_shaping=on' || { echo "FATAL: pick_shaping is not on"; exit 3; }
fi   # EVAL_ONLY

FINAL_CK=$OUT/rlpd_final.zip
[ -f "$FINAL_CK" ] || { echo "FATAL: no final checkpoint at $FINAL_CK"; exit 1; }
mapfile -t CK_DIRS < <(ls -d "$OUT"/ckpt_*/ 2>/dev/null | sort -V)
SW=$OUT/sweep; mkdir -p "$SW"
set +e
# THE GATE: LAST checkpoint, fresh process per episode, hold15 + rnd30 (deterministic policy = MODE)
bash cluster/eval_sweep.sh sac "$FINAL_CK" "$SW/final" --sets hold,rnd --ckpt-step final --tag final "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/final.log"
# secondary (disclosed): every archived checkpoint on rnd30 (the 25 % one = the 1e5-decision RLPD budget)
for D in "${CK_DIRS[@]}"; do
  T=$(basename "$D"); Z=$(ls "$D"/*.zip 2>/dev/null | head -1); [ -n "$Z" ] || continue
  bash cluster/eval_sweep.sh sac "$Z" "$SW/$T" --sets rnd --ckpt-step "$T" --tag "$T" "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/$T.log"
done
python3 - "$SW" "$SEED" "$NODE_CLASS" <<'PY'
import json, sys, glob, os
sw, seed, node = sys.argv[1:4]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception:
        return 'MISSING'
fin = os.path.join(sw, 'final', 'sweep.json')
extra = ' '.join(f"{os.path.basename(d)}_rnd={rd(os.path.join(d,'sweep.json'),'rnd')}" for d in sorted(glob.glob(os.path.join(sw, 'ckpt_*'))) if os.path.isdir(d))
print(f"SWEEP-HEADLINE arm=dRLteacher seed={seed} reward=dense no_demos=on final_hold={rd(fin,'hold')} final_rnd={rd(fin,'rnd')} {extra} node={node}")
PY
echo "JOB DONE $RUN_NAME $(date -Is)"
