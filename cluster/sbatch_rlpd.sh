#!/bin/bash
# RLPD n=20 wave: one seed per GPU, env-var driven. Post-E3-demo-rng code
# (>= 2fbed2a REQUIRED — the demo buffer ignores --seed before that commit).
#
# --- Submit (from the genesis_pickaplace checkout root, AFTER git pull) -----------
#   for S in $(seq 0 19); do ARM=dH        SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#   for S in $(seq 0 19); do ARM=dR2D      SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#   for S in $(seq 0 19); do ARM=dDP       SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#   for S in $(seq 0 19); do ARM=dHpruned  SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#   # dense-reward lever, any arm:
#   for S in $(seq 0 19); do ARM=dH SEED=$S PICK_SHAPING=on sbatch cluster/sbatch_rlpd.sh; done
#
# Env vars:
#   ARM    dH | dR2D | dDP | dHpruned   (required; picks demo dir + run name; provenance-checked)
#   SEED   0..19       (required)
#   STEPS  100000
#   PICK_SHAPING  on|off (default off; off is BYTE-IDENTICAL to every prior wave --
#                  train_rlpd.py's own --pick-shaping default is 'off', so the flag
#                  is only appended to the command line when this is 'on')
#   DUPLICATE_OK  <reason>  (unset by default; set to bypass a REGISTRY-REFUSE on an
#                  exact (script,arm,seed,git,knobs,demo) repeat -- see run_registry.py)
#   CONDA_ENV  /cluster/tufts/shortlab/jstale02/condaenv/genesis
#   DRYRUN=1   print the plan and exit before any conda/module/training call
#
# ARMS (demo dir / provenance pattern; dDP and dHpruned added 08-18, see
# paper/AUDIT_run_identity_2026-08-17.md and the demo-dir-naming-trap memory --
# m1all_harvest is MODEL-harvested demos, dppruned is PRUNED HUMAN demos, never
# assume from the name alone):
#   dH        baselines/episodes_pick_phase_all       human, full pick-phase set
#   dR2D      baselines/episodes_champion_pick         r2dreamer-harvested (control)
#   dDP       baselines/m1all_harvest                  DP-harvested model demos (rollout-index names)
#   dHpruned  baselines/episodes_pick_phase_dppruned    human, DP-pruned subset
# All four confirmed to carry the layout train_rlpd.py's demo loader expects:
# states (T,17) float32, actions (T,7) float32, a 'stage' field (see 08-18 audit
# note in this file's git history for the check transcript).
#
# DATA (rsync ONLY, never git — run these from the dev box once before submitting):
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_pick_phase_all/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_pick_phase_all/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_champion_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_champion_pick/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/m1all_harvest/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/m1all_harvest/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_pick_phase_dppruned/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_pick_phase_dppruned/
#
# Stages: RUN_REGISTRY check -> train (100k, ~2-3h) -> RUN_REGISTRY register ->
# §4a-2 sweep IN-JOB (15 demo-IC + 15 random-IC, one fresh process per episode,
# CPU) -> per-seed summary line in the .out -> best-effort wandb summary push.
# Same-machine rule: the arm CONTRAST is internal to the cluster/node class;
# do not compare absolute rates against local-box numbers without a caveat.
#
# --- WEDNESDAY SMOKE PLAN (run in this order; each step's proof is stated) --------
# 1. DRYRUN, one per ARM (local or on the login node -- no GPU needed, exits
#    before conda/module/git-gate/training):
#      for A in dH dR2D dDP dHpruned; do
#        GENESIS_PICKAPLACE_ROOT=$PWD ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd.sh
#      done
#    PROOF: each prints "[dry] ARM=<A> ... DEMO=<dir> OUT=<dir> RUN_NAME=<name>"
#    plus the exact planned registry-check and train_rlpd.py command lines, and
#    exits 0 with NO module/conda/pip/git-gate output above or below it.
# 2. One short REAL seed through the full path: ARM=dH SEED=0 STEPS=5000
#    sbatch cluster/sbatch_rlpd.sh (no DRYRUN).
#    PROOF: the .out shows, in order, "REGISTRY-OK ...", the train_rlpd.py
#    "[cfg] RLPD ..." line, "REGISTRY-REGISTERED ...", the 30-episode sweep (15
#    demo-IC + 15 random-IC) collapsing into exactly ONE "SWEEP-RESULT arm=dH
#    seed=0 ..." line, and either "wandb: pushed sweep/..." or a WARN line that
#    does NOT end the job (the .out ends with "JOB DONE", Slurm exit code 0).
#    Also: cluster/RUN_REGISTRY.jsonl gained exactly one new line for this key.
# 3. Re-submit the IDENTICAL seed (ARM=dH SEED=0 STEPS=5000, same git, no
#    DUPLICATE_OK): sbatch cluster/sbatch_rlpd.sh.
#    PROOF: the .out shows "REGISTRY-REFUSE key=... full-key match ..." and the
#    job exits before touching conda/training (no "[cfg] RLPD" line appears,
#    Slurm job state = FAILED, exit code 2). Re-running with
#    DUPLICATE_OK="smoke test" must instead print "REGISTRY-DUPLICATE-OK ..."
#    and proceed through the full path a second time.

#SBATCH -J rlpd
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-08:00:00
#SBATCH --output=rlpd_%j.out
#SBATCH --error=rlpd_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export PYTHONUNBUFFERED=1

ARM=${ARM:?set ARM=dH, dR2D, dDP, or dHpruned}
SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}
PICK_SHAPING=${PICK_SHAPING:-off}
case "$PICK_SHAPING" in
  on|off) ;;
  *) echo "FATAL: PICK_SHAPING must be on|off, got '$PICK_SHAPING'"; exit 1 ;;
esac

case "$ARM" in
  dH)       ARM_DEMO=baselines/episodes_pick_phase_all;       PAT='^[0-9]{3}\.npz$' ;;
  dR2D)     ARM_DEMO=baselines/episodes_champion_pick;        PAT='^1[0-9]{5}\.npz$' ;;
  dDP)      ARM_DEMO=baselines/m1all_harvest;                 PAT='^[0-9]{6}\.npz$' ;;
  dHpruned) ARM_DEMO=baselines/episodes_pick_phase_dppruned;  PAT='^[0-9]{3}\.npz$' ;;
  *) echo "FATAL: ARM=$ARM (must be dH, dR2D, dDP, or dHpruned)"; exit 1 ;;
esac

# stale-env case-guard (mirrors cluster/sbatch_r2d_ms.sh's DEMO_DIR guard, 08-18):
# a stale exported DEMO_DIR/DEMO/DATASET from another submission or shell must not
# silently override the ARM's own demo dir (this is the exact failure mode that fed
# a 4-dim ManiSkill run 7-dim genesis actions in the r2d_ms incident).
for _v in DEMO_DIR DEMO DATASET; do
  _val="${!_v:-}"
  if [ -n "$_val" ] && [ "$_val" != "$ARM_DEMO" ]; then
    echo "FATAL: env $_v=$_val is exported and does not match ARM=$ARM's demo dir ($ARM_DEMO)."
    echo "       A stale 'export $_v=...' from another submission is the usual cause: unset $_v"
    exit 1
  fi
done

# provenance gate (naming-trap rule): filename pattern must match the claimed source
ls "$ARM_DEMO" | grep -E '\.npz$' | head -5 | grep -qE "$PAT" || {
  echo "FATAL: $ARM_DEMO contents do not match $ARM provenance pattern $PAT"; exit 1; }

# (if/then, not `[ cond ] && x=y` or `$([ cond ] && echo y)` -- under `set -e`
# either of those forms exits the whole script SILENTLY when cond is false,
# because bash counts a false-and-short-circuited compound/substitution as the
# simple command's exit status. Bit us during local DRYRUN testing 08-18.)
SHAPE_SUFFIX=""; RUN_SUFFIX=""; PICK_SHAPING_FLAG=()
if [ "$PICK_SHAPING" = on ]; then
  SHAPE_SUFFIX="_shaped"; RUN_SUFFIX="-shaped"; PICK_SHAPING_FLAG=(--pick-shaping on)
fi
OUT=baselines/rl/checkpoints/rlpd_n20_${ARM}_s${SEED}${SHAPE_SUFFIX}
RUN_NAME="${ARM}_RLPD-n20_s${SEED}${RUN_SUFFIX}"

# semantic knobs for the RUN_REGISTRY identity key (audit §5: seed, gamma, utd,
# ensemble_size, subset_size, demo_batch, steps were the fields no sidecar recorded)
REG_KNOBS=(steps="$STEPS" scope=pick action_mode=delta_joint delta_ref=target
           action_repeat=1 gamma=0.998 backup_entropy=off per_member_ln=off
           pick_hold_reward=off utd=10 ensemble_size=10 subset_size=2
           demo_batch=128 pick_shaping="$PICK_SHAPING")

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS PICK_SHAPING=$PICK_SHAPING"
  echo "[dry] ARM_DEMO=$ARM_DEMO OUT=$OUT RUN_NAME=$RUN_NAME"
  echo "[dry] registry check: python cluster/run_registry.py check --script sbatch_rlpd.sh --arm $ARM --seed $SEED --demo-dir $ARM_DEMO --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
  echo "[dry] train: python baselines/rl/train_rlpd.py --steps $STEPS --scope pick --action-mode delta_joint --delta-ref target --action-repeat 1 --gamma 0.998 --backup-entropy off --per-member-ln off --pick-hold-reward off --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128 --demo-dir $ARM_DEMO --out-dir $OUT --run-name $RUN_NAME --project genesis_paper --seed $SEED --device cuda ${PICK_SHAPING_FLAG[*]}"
  echo "[dry] registry register: python cluster/run_registry.py register --script sbatch_rlpd.sh --arm $ARM --seed $SEED --demo-dir $ARM_DEMO --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
  exit 0
fi

# ---- RUN_REGISTRY: refuse an exact repeat, warn on a git-only-diff repeat --------
# Runs BEFORE conda/module so a refusal costs no GPU time; run_registry.py is
# stdlib-only, so the login-node system python3 suffices here.
python3 cluster/run_registry.py check --script sbatch_rlpd.sh --arm "$ARM" --seed "$SEED" \
  --demo-dir "$ARM_DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"

# sb3 is pip-only in this env; install once, guarded. NEVER conda install here.
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'

# git gate: demo-RNG fix must be present
git merge-base --is-ancestor 2fbed2a HEAD || {
  echo 'FATAL: checkout predates the demo-RNG fix (2fbed2a). git pull first.'; exit 1; }

python baselines/rl/train_rlpd.py \
  --steps "$STEPS" --scope pick --action-mode delta_joint --delta-ref target \
  --action-repeat 1 --gamma 0.998 --backup-entropy off --per-member-ln off \
  --pick-hold-reward off --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128 \
  --demo-dir "$ARM_DEMO" --out-dir "$OUT" \
  --run-name "$RUN_NAME" --project genesis_paper \
  --seed "$SEED" --device cuda "${PICK_SHAPING_FLAG[@]}"

python cluster/run_registry.py register --script sbatch_rlpd.sh --arm "$ARM" --seed "$SEED" \
  --demo-dir "$ARM_DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"

# ---- §4a-2 sweep, in-job, CPU, fresh process per episode ------------------------
# rlpd_final.zip (always written by model.save() at the end of training, any STEPS)
# rather than the periodic rlpd_<N>_steps.zip snapshot: SidecarCheckpointCallback's
# save_freq is a fixed 50_000, so STEPS values that are not multiples of 50_000
# (e.g. the STEPS=5000 smoke run) never produce a periodic snapshot at all. Same
# trained weights either way when STEPS IS a 50_000 multiple (final save happens
# immediately after the last training step, no further gradient updates in between).
CK=$OUT/rlpd_final.zip
[ -f "$CK" ] || { echo "FATAL: no final checkpoint at $CK"; exit 1; }
SW=$OUT/sweep; mkdir -p "$SW"
UIDS="232 234 235 236 237 239 242 243 244 245 246 247 248 250 251"
for u in $UIDS; do
  while [ "$(jobs -rp | wc -l)" -ge 5 ]; do sleep 3; done
  ( CUDA_VISIBLE_DEVICES="" python baselines/wandb_eval.py --kind sac --checkpoint "$CK" \
      --action-mode delta_joint --action-repeat 1 --ic-mode demo --random 0 --reps 1 \
      --uids "$u" --max-steps 400 --record-dir "$SW/v_d_$u" \
      --json "$SW/d_$u.json" --no-wandb > "$SW/d_$u.log" 2>&1 ) &
done
for k in $(seq 0 14); do
  while [ "$(jobs -rp | wc -l)" -ge 5 ]; do sleep 3; done
  ( CUDA_VISIBLE_DEVICES="" python baselines/wandb_eval.py --kind sac --checkpoint "$CK" \
      --action-mode delta_joint --action-repeat 1 --ic-mode random --random 1 --seed "$k" \
      --max-steps 400 --record-dir "$SW/v_r_$k" \
      --json "$SW/r_$k.json" --no-wandb > "$SW/r_$k.log" 2>&1 ) &
done
wait

python - "$SW" "$ARM" "$SEED" "$RUN_NAME" <<'PYEOF'
import json, glob, sys
sw, arm, seed, run_name = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
def count(pref):
    n = p = 0
    for f in sorted(glob.glob(f'{sw}/{pref}_*.json')):
        v = json.load(open(f))['metrics']['eval/picked']   # nested key — never top-level
        n += 1; p += (v > 0)
    return p, n
d, dn = count('d'); r, rn = count('r')
print(f'SWEEP-RESULT arm={arm} seed={seed} demoIC={d}/{dn} randomIC={r}/{rn}')

# ---- off-cluster durability (08-18): push the per-seed sweep numbers onto the
# training run's wandb summary, so they exist even if this .out file is lost.
# Best-effort ONLY -- the SWEEP-RESULT line above is the source of truth; a wandb
# hiccup here must never fail an already-completed job. ----
try:
    import wandb
    api = wandb.Api(timeout=60)
    runs = list(api.runs('jambotime/genesis_paper', filters={'display_name': run_name}))
    if not runs:
        print(f'WARN: wandb push skipped -- no run named {run_name!r} in jambotime/genesis_paper')
    else:
        if len(runs) > 1:
            print(f'WARN: {len(runs)} runs named {run_name!r} found -- pushing to the most recent')
        run = sorted(runs, key=lambda x: x.created_at)[-1]
        run.summary['sweep/demoIC'] = f'{d}/{dn}'
        run.summary['sweep/randomIC'] = f'{r}/{rn}'
        run.summary['sweep/n'] = dn + rn
        run.summary.update()
        print(f'wandb: pushed sweep/demoIC={d}/{dn} sweep/randomIC={r}/{rn} '
              f'sweep/n={dn + rn} to run {run.id} ({run_name})')
except Exception as e:
    print(f'WARN: wandb sweep push failed ({type(e).__name__}: {e}) -- '
          'SWEEP-RESULT line above is authoritative')
PYEOF
echo "JOB DONE $(date)"
