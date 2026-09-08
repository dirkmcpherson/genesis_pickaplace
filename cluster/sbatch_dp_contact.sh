#!/bin/bash
# DP CONTACT/SLIDE-phase launcher (PHASE_PLAN_2026-09-04 amendment (m), 2026-09-07): the DP recipe of record
# (cluster/sbatch_dp.sh: lerobot Diffusion Policy, state-only, absolute joint targets at fps 7.5, 100k grad steps,
# batch 64, K=5 checkpoints, preemption-safe resume) on the CONTACT (slide) segments of the full-task
# tapes ([k_placed_v2, k_contact]; place_demos.py cut --phase contact -> convert_to_lerobot.py), then the evaluation
# cells (holdE_contact / polE_contact_physgrip, sampled) of cluster/place_eval_cells.sh PHASE=contact on the LAST
# (100k) checkpoint, executed hold-4 through the env's delta integrator. Statistic of record: slide_success (l').
# Both arms are BELOW the registered 20-demo floor (11 v 11); the pair is reported as sub-floor.
#
# Submit (from the code checkout root):
#   for S in $(seq 0 7); do ARM=dH  SEED=$S sbatch -J sl_dp_dH_s$S  cluster/sbatch_dp_contact.sh; done
#   for S in $(seq 0 7); do ARM=dDP SEED=$S sbatch -J sl_dp_dDP_s$S cluster/sbatch_dp_contact.sh; done
# Env vars:
#   ARM        dH | dDP   (dH -> $DEMO_ROOT/dH_contact, dDP -> $DEMO_ROOT/dDP_contact_n11; RAW npz + manifest + lerobot/)
#   SEED       required     STEPS 100000     DEMO_ROOT /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3
#   HOLDE / POLE  evaluation banks (defaults holdE_contact.json, polE_contact_physgrip.json = the rebuilt bank of record)
#   OUT_ROOT   baselines/outputs/dp_contact -> $OUT_ROOT/sl_dp_${ARM}_s${SEED}    PROJ genesis_paper    DRYRUN=1
#   SAVE_FREQ  STEPS/2 -- DISK RULE (2026-09-07 incident, coordinator: keep only the final checkpoint). ONE lerobot
#             checkpoint is 2.85 GB (949 MB pretrained_model + 1.9 GB training_state), so the launcher-of-record's
#             save_freq=STEPS/5 would hold 5 x 2.85 GB x 16 runs = 228 GB. Here: at most TWO numbered checkpoints
#             exist during a run (one mid-run resume point for the preempt queue), and after training every
#             checkpoint except the final is deleted AND the final's training_state (optimizer state, which no eval
#             reads and no finished run resumes from) is deleted -- 949 MB per run at rest, 15 GB for all 16.
#             A requeue AFTER the budget was reached skips training entirely instead of resuming.
#SBATCH -J sl_dp
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=8000
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-14:00:00
#SBATCH --output=sl_dp_%j.out
#SBATCH --error=sl_dp_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
W=${W:-/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03}
# PHASE_PLAN (p) HOLD (2026-09-07): (l)'s grip<0.3 clause is withdrawn (it passes 2 of 74 demonstrations) and
# amendment (o) was STOPPED before landing, so this phase currently has no correct reward. The 32 Slide jobs are held;
# this launcher refuses to run rather than train against a withdrawn predicate. Release requires (p)'s calibrated
# prior-release predicate AND the lineage question (dHfull_w3 vs census, 24/74 streams disagree) being settled.
if [ -z "${CONTACT_GRANT:-}" ]; then
  echo "SLIDE-ON-HOLD: refusing to train DP contact -- PHASE_PLAN (p) withdrew the grip clause and stopped (o);"
  echo "  no reward predicate is currently registered as correct. Export CONTACT_GRANT=<bare_contact|prior_release> only"
  echo "  once the coordinator releases the phase."
  exit 0
fi
ARM=${ARM:?set ARM (dH | dDP)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}; PROJ=${PROJ:-genesis_paper}; WAVE=${WAVE:-contact}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
ACTION_REPEAT=4; EVAL_HORIZON=600
DEMO_ROOT=${DEMO_ROOT:-/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3}
case "$ARM" in
  dH)  SET=dH_contact ;;
  dDP) SET=dDP_contact_n11 ;;
  *) echo "FATAL: ARM must be dH | dDP (got $ARM)"; exit 1 ;;
esac
RAW=$DEMO_ROOT/$SET; DATASET=$RAW/lerobot
HOLDE=${HOLDE:-$W/phase_banks/holdE_contact.json}; POLE=${POLE:-$W/phase_banks/polE_contact_physgrip.json}
OUT_ROOT=${OUT_ROOT:-baselines/outputs/dp_contact}
OUT=$OUT_ROOT/sl_dp_${ARM}_s${SEED}
RUN_NAME="sl_dp_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
SAVE_FREQ=${SAVE_FREQ:-$(( STEPS / 2 ))}; [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1
export GENESIS_SIM_VARIANT=$SIM_VARIANT SIM_VARIANT_FOR_SIDECAR=$SIM_VARIANT

# ---- provenance gates (the sbatch_dp.sh rules: 6-digit rollout stems, contract-v1 manifest, sim_variant, n_kept, fps) ----
[ -d "$RAW" ] || { echo "FATAL: raw contact set $RAW missing (place_demos.py cut --phase contact)"; exit 1; }
ls "$RAW" | grep -E '\.npz$' | head -5 | grep -qE '^[0-9]{6}\.npz$' || { echo "FATAL: $RAW contents are not 6-digit rollout stems"; exit 1; }
N_SUCCESS=$(ls "$RAW"/*.npz | wc -l)
DEMO_SHA=$(python3 - "$RAW" "$SIM_VARIANT" "$ARM" <<'PY'
import json, os, sys
d, sv, arm = sys.argv[1:4]
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = json.load(open(os.path.join(d, 'manifest.json')))
assert m.get('contract') == 'v1' and m.get('phase') == 'contact' and m.get('sim_variant') == sv, m
assert int(m['n_kept']) == len(files) == 11, (m.get('n_kept'), len(files))
assert m.get('builder') == 'baselines/rl/place_demos.py cut', m.get('builder')
if arm == 'dDP': assert m.get('one_per_ic') is True and m.get('keep_from'), 'machine set must be the matched-11 cut'
print(f'DEMO-SHA {arm} contact n={len(files)} sha={m["content_sha256"][:16]} rows={m["decisions_total"]}', file=sys.stderr)
print(m['content_sha256'][:16])
PY
) || exit 1
[ -d "$DATASET" ] || { echo "FATAL: lerobot dataset $DATASET missing (convert_to_lerobot.py $RAW $DATASET 8 4 none)"; exit 1; }
python3 - "$DATASET" "$N_SUCCESS" "$ACTION_REPEAT" <<'PY' || exit 1
import json, sys, pathlib as pl
ds, n_exp, rep = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
info = json.loads((pl.Path(ds) / 'meta' / 'info.json').read_text())
assert info['total_episodes'] == n_exp, (info['total_episodes'], n_exp)
assert abs(float(info['fps']) - 30.0 / rep) < 1e-6, (info['fps'], rep)
src = json.loads((pl.Path(ds) / 'genesis_source.json').read_text()); assert src.get('contract') == 'v1', src
print(f'PROVENANCE-OK dataset={ds} total_episodes={info["total_episodes"]} total_frames={info["total_frames"]} fps={info["fps"]}')
PY
[ -s "$HOLDE" ] || { echo "FATAL: holdE bank missing: $HOLDE"; exit 1; }
REG_KNOBS=(steps="$STEPS" budget_unit=grad_steps batch_size=64 policy=diffusion dataset_root="$DATASET" action_repeat="$ACTION_REPEAT"
           eval_horizon="$EVAL_HORIZON" demo_format=native_place demo_sha="$DEMO_SHA" save_freq="$SAVE_FREQ" wave="$WAVE" sim_variant="$SIM_VARIANT" scope=contact)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS RAW=$RAW (N=$N_SUCCESS sha=$DEMO_SHA) DATASET=$DATASET OUT=$OUT NODE=$NODE_CLASS SAVE_FREQ=$SAVE_FREQ"
  echo "[dry] train: lerobot-train --dataset.repo_id=local/${RUN_NAME} --dataset.root=$DATASET --policy.type=diffusion --policy.push_to_hub=false --seed=$SEED --output_dir=$OUT --batch_size=64 --steps=$STEPS --save_freq=$SAVE_FREQ --job_name=$RUN_NAME --wandb.enable=true --wandb.project=$PROJ --wandb.disable_artifact=true"
  echo "[dry] prune: keep checkpoints/$(printf %06d $STEPS)/pretrained_model only (drop other numbered ckpts + training_state)"
  echo "[dry] eval : KIND=dp CKPT=$OUT/checkpoints/$(printf %06d $STEPS)/pretrained_model OUT=$OUT ARM=$ARM SEED=$SEED HOLDE=$HOLDE POLE=$POLE bash cluster/place_eval_cells.sh"
  exit 0
fi
echo "== DP-CONTACT $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) raw=$RAW sha=$DEMO_SHA dataset=$DATASET restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_dp_contact.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_dp_contact.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
fi
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"
if command -v lerobot-train >/dev/null 2>&1; then LEROBOT_TRAIN="lerobot-train"
elif python -c 'import lerobot.scripts.lerobot_train' 2>/dev/null; then LEROBOT_TRAIN="python -m lerobot.scripts.lerobot_train"
else echo "FATAL: lerobot is not importable in this env ($CONDA_PREFIX)"; exit 1; fi
# ---- train (the launcher of record's invocation + preemption-safe resume, verbatim) ----
TC="$OUT/checkpoints/last/pretrained_model/train_config.json"
DONE_CK=$OUT/checkpoints/$(printf %06d "$STEPS")/pretrained_model
if [ -d "$DONE_CK" ]; then
  # a requeue AFTER the budget was reached (preempted during the eval stage): never retrain, and never try to
  # resume a finished run whose training_state was pruned below -- go straight to the evaluation cells.
  echo "== TRAIN-DONE-ALREADY: $DONE_CK exists; skipping training (requeue #${SLURM_RESTART_COUNT:-0})"
elif [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$TC" ]; then
  echo "== requeued (restart #${SLURM_RESTART_COUNT}); RESUMING via $TC"
  $LEROBOT_TRAIN --config_path="$TC" --resume=true
else
  rm -rf "$OUT"
  $LEROBOT_TRAIN --dataset.repo_id="local/${RUN_NAME}" --dataset.root="$DATASET" --policy.type=diffusion --policy.push_to_hub=false \
    --seed="$SEED" --output_dir="$OUT" --batch_size=64 --steps="$STEPS" --save_freq="$SAVE_FREQ" --job_name="$RUN_NAME" \
    --wandb.enable=true --wandb.project="$PROJ" --wandb.disable_artifact=true
fi
LAST_D=$(ls -d "$OUT"/checkpoints/[0-9]*/ 2>/dev/null | sort -V | tail -1)
[ -n "$LAST_D" ] && [ -d "$LAST_D/pretrained_model" ] || { echo "FATAL: no numbered checkpoint under $OUT/checkpoints"; exit 1; }
[ "$(basename "$LAST_D")" = "$(printf %06d "$STEPS")" ] || { echo "FATAL: last checkpoint $(basename "$LAST_D") != budget $STEPS -- training did not reach its budget; no evaluation of a partial run"; exit 1; }
# ---- DISK RULE (2026-09-07): keep ONLY the final checkpoint's weights ----------------------
# Every numbered checkpoint except the final goes; the final's training_state (optimizer, 1.9 GB) goes too --
# no evaluation reads it and the requeue guard above means a finished run never resumes. Reported in the .out.
BEFORE_KB=$(du -sk "$OUT" | cut -f1)
FINAL_N=$(basename "$LAST_D")
for D in $(ls -d "$OUT"/checkpoints/[0-9]*/ | sort -V); do
  N=$(basename "$D")
  [ "$N" = "$FINAL_N" ] && continue
  echo "== pruning superseded checkpoint $D"; rm -rf "$D"
done
[ -d "$LAST_D/training_state" ] && { echo "== pruning $LAST_D/training_state (optimizer state; no eval reads it)"; rm -rf "$LAST_D/training_state"; }
AFTER_KB=$(du -sk "$OUT" | cut -f1)
echo "CKPT-PRUNE $OUT: $((BEFORE_KB / 1024)) MB -> $((AFTER_KB / 1024)) MB (kept checkpoints/$FINAL_N/pretrained_model only)"
[ -d "$LAST_D/pretrained_model" ] || { echo "FATAL: prune removed the final checkpoint -- refusing to continue"; exit 1; }

GIT_HASH=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
for D in $(ls -d "$OUT"/checkpoints/[0-9]*/ | sort -V); do
  python3 - "$D" "$ARM" "$SEED" "$RAW" "$DATASET" "$GIT_HASH" "$STEPS" "$PROJ" "$ACTION_REPEAT" "$DEMO_SHA" "$NODE_CLASS" "$SIM_VARIANT" <<'PY'
import json, sys, pathlib as pl, datetime
d, arm, seed, raw, dataset, git, steps, proj, rep, sha, node, sv = sys.argv[1:13]
pl.Path(d, 'dp_sidecar.json').write_text(json.dumps({
    'script': 'sbatch_dp_contact.sh', 'arm': arm, 'seed': int(seed), 'scope': 'contact', 'raw_demo_dir': raw, 'dataset_root': dataset, 'git': git,
    'action_repeat': int(rep), 'delta_cap': 0.025, 'delta_leash': 0.125, 'demo_sha': sha, 'demo_format': 'native_place', 'node': node, 'sim_variant': sv,
    'ckpt_step': pl.Path(d).name, 'config': {'policy': 'diffusion', 'batch_size': 64, 'steps': int(steps), 'project': proj},
    'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')}, indent=1))
print(f'sidecar -> {d}dp_sidecar.json')
PY
done
set +e   # evals never fail an already-trained job
PHASE=contact KIND=dp CKPT="$LAST_D/pretrained_model" OUT="$OUT" ARM="$ARM" SEED="$SEED" HOLDE="$HOLDE" POLE="$POLE" SIM_VARIANT="$SIM_VARIANT" PAR=2 bash cluster/place_eval_cells.sh 2>&1 | tee "$OUT/contact_eval.log"
echo "JOB DONE $(date)"
