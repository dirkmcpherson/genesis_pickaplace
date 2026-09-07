#!/bin/bash
# RLPD PLACE-phase launcher (PHASE_PLAN_2026-09-04 amendment (h), 2026-09-07): the RLPD recipe of record
# (cluster/sbatch_rlpd.sh: UTD 10, E10/Z2, LN critics, gamma 0.99, 50/50 demo batches, delta_joint cap 0.025 /
# leash 5x, delta_ref target, action_repeat 4) trained in FullTaskEnv(scope='place', phase_sparse) from the human
# pick-grant bank, demo half = the r2dreamer-native place SEGMENTS (the rows the WM trained on), then the 4 place
# evaluation cells (holdE/polE x sample/mode) of cluster/place_eval_cells.sh on the LAST checkpoint.
#
# Submit (from the code checkout root; W = the WM-fix dir with phase_banks/ and demos_state/):
#   for S in $(seq 0 7); do ARM=dH  SEED=$S sbatch -J pl_rlpd_dH_s$S  cluster/sbatch_rlpd_place.sh; done
#   for S in $(seq 0 7); do ARM=dDP SEED=$S sbatch -J pl_rlpd_dDP_s$S cluster/sbatch_rlpd_place.sh; done
# Env vars:
#   ARM       dH | dDP  (dH -> $W/demos_state/dH_place, 39 human segments; dDP -> $W/demos_state/dDP_place_n39, matched 39)
#   SEED      required
#   STEPS     250000 decisions (= 1e6 sim steps at repeat 4; amendment (h))
#   BANK      $W/phase_banks/human_place.json   (the HUMAN pick-grant bank for BOTH arms; amendment (h))
#   HOLDE / POLE   evaluation banks (defaults $W/phase_banks/{holdE,polE}_place.json; polE deferred until rebuilt)
#   OUT_ROOT  baselines/rl/checkpoints/place    -> $OUT_ROOT/pl_rlpd_${ARM}_s${SEED}
#   WAVE      place   SIM_VARIANT gc_kp4_riser3_shelf6   GAMMA 0.99   DRYRUN=1 prints the plan
# A requeued (preempted) run restarts CLEAN (the trainer does not resume; same as the WM launcher).
#SBATCH -J pl_rlpd
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=2000
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=1-00:00:00
#SBATCH --output=pl_rlpd_%j.out
#SBATCH --error=pl_rlpd_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
W=${W:-/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03}
ARM=${ARM:?set ARM (dH | dDP)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-250000}; WAVE=${WAVE:-place}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; GAMMA=${GAMMA:-0.99}
ACTION_REPEAT=4; TRAIN_HORIZON=600; EVAL_HORIZON=600
case "$ARM" in
  dH)  DEMO=${DEMO:-$W/demos_state/dH_place} ;;
  dDP) DEMO=${DEMO:-$W/demos_state/dDP_place_n39} ;;
  *) echo "FATAL: ARM must be dH | dDP (got $ARM)"; exit 1 ;;
esac
BANK=${BANK:-$W/phase_banks/human_place.json}
HOLDE=${HOLDE:-$W/phase_banks/holdE_place.json}; POLE=${POLE:-$W/phase_banks/polE_place.json}
OUT_ROOT=${OUT_ROOT:-baselines/rl/checkpoints/place}
OUT=$OUT_ROOT/pl_rlpd_${ARM}_s${SEED}
RUN_NAME="pl_rlpd_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
export GENESIS_SIM_VARIANT=$SIM_VARIANT

# ---- provenance gates: segment set (repeat.json) + banks ----
[ -d "$DEMO" ] || { echo "FATAL: segment dir $DEMO missing"; exit 1; }
DEMO_SHA=$(python3 - "$DEMO" "$SIM_VARIANT" "$ARM" <<'PY'
import json, os, sys, glob, hashlib
d, sv, arm = sys.argv[1:4]
m = json.load(open(os.path.join(d, 'repeat.json')))
assert m['sim_variant'] == sv and m.get('phase') == 'place' and m.get('with_state') is True and int(m['action_repeat']) == 4, m
assert abs(float(m['terminal_reward']) - 1.0) < 1e-9 and abs(float(m['delta_cap']) - 0.025) < 1e-9, m
fs = sorted(glob.glob(os.path.join(d, '*.npz'))); assert len(fs) == int(m['n_written']) == 39, (len(fs), m.get('n_written'))
if arm == 'dDP': assert m.get('matched_n_of') == 63 and m.get('matched_n_seed') == 0 and m.get('one_per_ic') is True, m
h = hashlib.sha256()
for f in fs:
    h.update(os.path.basename(f).encode()); h.update(open(f, 'rb').read())
sha = h.hexdigest()
print(f'DEMO-SHA {arm} segment n={len(fs)} sha={sha[:16]} total_reward={m["total_reward"]} decisions_p50={m["decisions_median"]}', file=sys.stderr)
print(sha[:16])
PY
) || exit 1
for b in "$BANK" "$HOLDE"; do [ -s "$b" ] || { echo "FATAL: bank missing/empty: $b"; exit 1; }; done
python3 -c 'import json,sys; b=json.load(open(sys.argv[1])); n=len(b) if isinstance(b,list) else len(b.get("entries", b)); assert n==64, n; print(f"BANK-OK {sys.argv[1]}: {n} human pick-grant entries")' "$BANK" || exit 1

TRAIN_ARGS=(--steps "$STEPS" --scope place --entry-bank "$BANK" --demo-format segment --demo-dir "$DEMO"
  --action-mode delta_joint --delta-ref target --action-repeat "$ACTION_REPEAT"
  --train-max-steps "$TRAIN_HORIZON" --eval-max-steps "$EVAL_HORIZON" --eval-freq 0
  --gamma "$GAMMA" --backup-entropy off --per-member-ln off --pick-hold-reward off --pick-shaping off
  --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128
  --demo-shaping off --pick-shaping-terminal-zero on --demo-terminal-guard on --sim-variant "$SIM_VARIANT"
  --out-dir "$OUT" --run-name "$RUN_NAME" --project genesis_paper --seed "$SEED" --device cuda)
REG_KNOBS=(steps="$STEPS" budget_unit=decisions scope=place action_mode=delta_joint delta_ref=target action_repeat="$ACTION_REPEAT"
           train_horizon="$TRAIN_HORIZON" eval_horizon="$EVAL_HORIZON" gamma="$GAMMA" backup_entropy=off per_member_ln=off utd=10
           ensemble_size=10 subset_size=2 demo_batch=128 reward=sparse demo_format=segment demo_sha="$DEMO_SHA" wave="$WAVE"
           sim_variant="$SIM_VARIANT" entry_bank="$(basename "$BANK")" phase_sparse=on)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS(decisions = $((STEPS * ACTION_REPEAT)) sim steps) DEMO=$DEMO sha=$DEMO_SHA BANK=$BANK OUT=$OUT NODE=$NODE_CLASS"
  echo "[dry] train: python baselines/rl/train_rlpd.py ${TRAIN_ARGS[*]}"
  echo "[dry] eval : KIND=sac CKPT=$OUT/rlpd_final.zip OUT=$OUT ARM=$ARM SEED=$SEED HOLDE=$HOLDE POLE=$POLE bash cluster/place_eval_cells.sh"
  exit 0
fi
echo "== RLPD-PLACE $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) demo=$DEMO sha=$DEMO_SHA bank=$BANK restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_rlpd_place.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_rlpd_place.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
else
  echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial run dir and starting clean"; rm -rf "$OUT"
fi
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'
python baselines/rl/train_rlpd.py "${TRAIN_ARGS[@]}"
FINAL_CK=$OUT/rlpd_final.zip
[ -f "$FINAL_CK" ] && [ -f "${FINAL_CK%.zip}.action_mode.json" ] || { echo "FATAL: no final checkpoint + sidecar at $FINAL_CK -- no evaluation of a partial run"; exit 1; }
python3 - "$OUT" "$STEPS" <<'PY' || exit 1
import json, sys, os
out, steps = sys.argv[1], int(sys.argv[2])
sc = json.load(open(os.path.join(out, 'rlpd_final.action_mode.json')))
assert sc['scope'] == 'place' and int(sc['steps']) == steps and sc.get('phase_sparse') is True and abs(float(sc['delta_cap']) - 0.025) < 1e-9, sc
ck = json.load(open(os.path.join(out, 'ckpt_100', 'rlpd_ckpt.action_mode.json')))
assert int(ck['ckpt_step']) >= steps, ck   # the archive's 100% checkpoint fired at the budget: training reached it
print(f'TRAIN-OK {out}: budget {steps} decisions reached (ckpt_100 at {ck["ckpt_step"]}), sidecar scope=place phase_sparse cap={sc["delta_cap"]} leash={sc["delta_leash"]}')
PY
set +e   # evals never fail an already-trained job
KIND=sac CKPT="$FINAL_CK" OUT="$OUT" ARM="$ARM" SEED="$SEED" HOLDE="$HOLDE" POLE="$POLE" SIM_VARIANT="$SIM_VARIANT" PAR=4 bash cluster/place_eval_cells.sh 2>&1 | tee "$OUT/place_eval.log"
echo "JOB DONE $(date)"
