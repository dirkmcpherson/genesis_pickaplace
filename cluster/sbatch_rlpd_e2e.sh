#!/bin/bash
# RLPD END-TO-END (full task) launcher (PHASE_PLAN_2026-09-04 amendment (n), 2026-09-07): the RLPD recipe of record
# (cluster/sbatch_rlpd.sh: UTD 10, E10/Z2, LN critics, gamma 0.99, 50/50 demo batches, delta_joint cap 0.025 /
# leash 5x, delta_ref target, action_repeat 4) trained in FullTaskEnv(scope='full') from the pick-scope ICs under
# the STAGED sparse ladder (picked 1 / placed 1 / contact 2 / nested 4), demo half = the r2dreamer-native FULL-scope
# segments the world model's amendment-(d) arm trained on, then the end-to-end evaluation cells of
# cluster/e2e_eval_cells.sh on the LAST checkpoint.
#
# Submit (from the code checkout root; W = the WM-fix dir with demos_state_full/):
#   for S in $(seq 0 7); do ARM=dH  SEED=$S sbatch -J e2e_rlpd_dH_s$S  cluster/sbatch_rlpd_e2e.sh; done
#   for S in $(seq 0 7); do ARM=dDP SEED=$S sbatch -J e2e_rlpd_dDP_s$S cluster/sbatch_rlpd_e2e.sh; done
# Env vars:
#   ARM       dH | dDP  (dH -> $W/demos_state_full/dHfull_all, 74 human tapes incl. 10 no-picks;
#                        dDP -> $W/demos_state_full/dDPfull, 72 machine tapes = best per IC)
#   SEED      required
#   STEPS     250000 decisions (= 1e6 sim steps at repeat 4; amendment (n), the (h) place budget)
#   OUT_ROOT  baselines/rl/checkpoints/e2e  -> $OUT_ROOT/e2e_rlpd_${ARM}_s${SEED}
#   CKPT_FRACS 0.4,1.0  -- DISK RULE (2026-09-07 incident): ckpt_100 = LAST (statistic of record), ckpt_040 = 100k
#             decisions (the RLPD pick-budget read); periodic snapshot zips OFF (--ckpt-every 0). 3 x 12 MB per run.
#   SETS/MODES/VIDEO_SETS  passed to e2e_eval_cells.sh   WAVE e2e   SIM_VARIANT gc_kp4_riser3_shelf6   GAMMA 0.99
#   DRYRUN=1  prints the plan and exits before any conda/module/training call
# A requeued (preempted) run restarts CLEAN (the trainer does not resume; same as the place/WM launchers).
#SBATCH -J e2e_rlpd
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=9000
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=1-06:00:00
#SBATCH --output=e2e_rlpd_%j.out
#SBATCH --error=e2e_rlpd_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
W=${W:-/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03}
ARM=${ARM:?set ARM (dH | dDP)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-250000}; WAVE=${WAVE:-e2e}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; GAMMA=${GAMMA:-0.99}
ACTION_REPEAT=4; TRAIN_HORIZON=1200; EVAL_HORIZON=1200
case "$ARM" in
  dH)  DEMO=${DEMO:-$W/demos_state_full/dHfull_all}; N_EXP=74 ;;
  dDP) DEMO=${DEMO:-$W/demos_state_full/dDPfull};    N_EXP=72 ;;
  *) echo "FATAL: ARM must be dH | dDP (got $ARM)"; exit 1 ;;
esac
OUT_ROOT=${OUT_ROOT:-baselines/rl/checkpoints/e2e}
OUT=$OUT_ROOT/e2e_rlpd_${ARM}_s${SEED}
RUN_NAME="e2e_rlpd_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
export GENESIS_SIM_VARIANT=$SIM_VARIANT

# ---- disk guard (2026-09-07 filesystem-full incident; registered floor 150 GB) ----
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9')
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: /cluster/tufts/shortlab free ${FREE_GB:-?} GB < 150 GB floor -- refusing to train"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"

# ---- provenance gate: the FULL-scope segment set (repeat.json), amendment (n) ----
DEMO_SHA=$(python3 - "$DEMO" "$SIM_VARIANT" "$ARM" "$N_EXP" <<'PY'
import json, os, sys, glob, hashlib
d, sv, arm, n_exp = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
m = json.load(open(os.path.join(d, 'repeat.json')))
assert m['sim_variant'] == sv and m.get('scope') == 'full' and m.get('with_state') is True and int(m['action_repeat']) == 4, m
assert m.get('reward_from_tape') is True and abs(float(m['delta_cap']) - 0.025) < 1e-9, m
assert m.get('phase') in (None, 'null'), m
fs = sorted(glob.glob(os.path.join(d, '*.npz')))
assert len(fs) == int(m['n_written']) == n_exp, (len(fs), m.get('n_written'), n_exp)
if arm == 'dDP':
    assert m.get('one_per_ic_best') is True, m
else:
    assert not m.get('one_per_ic_best'), m
h = hashlib.sha256()
for f in fs:
    h.update(os.path.basename(f).encode()); h.update(open(f, 'rb').read())
print(f'DEMO-SHA {arm} full-scope segments n={len(fs)} sha={h.hexdigest()[:16]} total_reward={m["total_reward"]} '
      f'pick={m["n_pick"]} nopick={m["n_nopick"]} decisions_p50={m["decisions_median"]}', file=sys.stderr)
print(h.hexdigest()[:16])
PY
) || exit 1

TRAIN_ARGS=(--steps "$STEPS" --scope full --demo-format segment --demo-dir "$DEMO"
  --action-mode delta_joint --delta-ref target --action-repeat "$ACTION_REPEAT"
  --train-max-steps "$TRAIN_HORIZON" --eval-max-steps "$EVAL_HORIZON" --eval-freq 0
  --gamma "$GAMMA" --backup-entropy off --per-member-ln off --pick-hold-reward off --pick-shaping off
  --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128
  --demo-shaping off --pick-shaping-terminal-zero on --demo-terminal-guard on --sim-variant "$SIM_VARIANT"
  --ckpt-every 0 --ckpt-fracs "${CKPT_FRACS:-0.4,1.0}"
  --out-dir "$OUT" --run-name "$RUN_NAME" --project genesis_paper --seed "$SEED" --device cuda)
REG_KNOBS=(steps="$STEPS" budget_unit=decisions scope=full action_mode=delta_joint delta_ref=target action_repeat="$ACTION_REPEAT"
           train_horizon="$TRAIN_HORIZON" eval_horizon="$EVAL_HORIZON" gamma="$GAMMA" backup_entropy=off per_member_ln=off utd=10
           ensemble_size=10 subset_size=2 demo_batch=128 reward=staged_sparse demo_format=segment demo_sha="$DEMO_SHA" wave="$WAVE"
           sim_variant="$SIM_VARIANT" entry_bank=none phase_sparse=off amendment=n)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS(decisions = $((STEPS * ACTION_REPEAT)) sim steps) DEMO=$DEMO sha=$DEMO_SHA OUT=$OUT NODE=$NODE_CLASS"
  echo "[dry] train: python baselines/rl/train_rlpd.py ${TRAIN_ARGS[*]}"
  echo "[dry] eval : KIND=sac CKPT=$OUT/rlpd_final.zip OUT=$OUT ARM=$ARM SEED=$SEED bash cluster/e2e_eval_cells.sh"
  exit 0
fi
echo "== RLPD-E2E $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) demo=$DEMO sha=$DEMO_SHA restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_rlpd_e2e.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_rlpd_e2e.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
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
assert sc['scope'] == 'full' and int(sc['steps']) == steps and abs(float(sc['delta_cap']) - 0.025) < 1e-9, sc
assert sc.get('phase_sparse') is False and sc.get('entry_bank') is None, sc
ck = json.load(open(os.path.join(out, 'ckpt_100', 'rlpd_ckpt.action_mode.json')))
assert not [f for f in os.listdir(out) if f.endswith('_steps.zip')], 'periodic snapshot zips written despite --ckpt-every 0'
assert int(ck['ckpt_step']) >= steps, ck   # the archive's 100% checkpoint fired at the budget: training reached it
print(f'TRAIN-OK {out}: budget {steps} decisions reached (ckpt_100 at {ck["ckpt_step"]}), sidecar scope=full staged reward cap={sc["delta_cap"]} leash={sc["delta_leash"]}')
PY
set +e   # evals never fail an already-trained job
KIND=sac CKPT="$FINAL_CK" OUT="$OUT" ARM="$ARM" SEED="$SEED" SIM_VARIANT="$SIM_VARIANT" \
  SETS="${SETS:-hold15 rnd30 spots60}" MODES="${MODES:-sample mode}" VIDEO_SETS="${VIDEO_SETS:-rnd30}" PAR=${PAR:-6} \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_eval.log"
echo "JOB DONE $(date)"
