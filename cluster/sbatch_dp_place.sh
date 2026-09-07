#!/bin/bash
# DP PLACE-phase launcher (PHASE_PLAN_2026-09-04 amendment (h), 2026-09-07): the DP recipe of record
# (cluster/sbatch_dp.sh: lerobot Diffusion Policy, state-only, absolute joint targets at fps 7.5, 100k grad steps,
# batch 64, K=5 checkpoints, preemption-safe resume) on the PLACE segments of the full-task tapes
# (baselines/rl/place_demos.py cut -> convert_to_lerobot.py), then the place evaluation cells (holdE/polE, sampled)
# of cluster/place_eval_cells.sh on the LAST (100k) checkpoint, executed hold-4 through the env's delta integrator.
#
# Submit (from the code checkout root):
#   for S in $(seq 0 7); do ARM=dH  SEED=$S sbatch -J pl_dp_dH_s$S  cluster/sbatch_dp_place.sh; done
#   for S in $(seq 0 7); do ARM=dDP SEED=$S sbatch -J pl_dp_dDP_s$S cluster/sbatch_dp_place.sh; done
# Env vars:
#   ARM        dH | dDP   (dH -> $DEMO_ROOT/dH_place, dDP -> $DEMO_ROOT/dDP_place_n39; RAW npz + manifest + lerobot/)
#   SEED       required     STEPS 100000     DEMO_ROOT /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3
#   HOLDE / POLE  evaluation banks (defaults $W/phase_banks/{holdE,polE}_place.json; polE deferred until rebuilt)
#   OUT_ROOT   baselines/outputs/dp_place  -> $OUT_ROOT/pl_dp_${ARM}_s${SEED}     PROJ genesis_paper    DRYRUN=1
#SBATCH -J pl_dp
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
#SBATCH --time=0-14:00:00
#SBATCH --output=pl_dp_%j.out
#SBATCH --error=pl_dp_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
W=${W:-/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03}
ARM=${ARM:?set ARM (dH | dDP)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}; PROJ=${PROJ:-genesis_paper}; WAVE=${WAVE:-place}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
ACTION_REPEAT=4; EVAL_HORIZON=600
DEMO_ROOT=${DEMO_ROOT:-/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3}
case "$ARM" in
  dH)  SET=dH_place ;;
  dDP) SET=dDP_place_n39 ;;
  *) echo "FATAL: ARM must be dH | dDP (got $ARM)"; exit 1 ;;
esac
RAW=$DEMO_ROOT/$SET; DATASET=$RAW/lerobot
HOLDE=${HOLDE:-$W/phase_banks/holdE_place.json}; POLE=${POLE:-$W/phase_banks/polE_place.json}
OUT_ROOT=${OUT_ROOT:-baselines/outputs/dp_place}
OUT=$OUT_ROOT/pl_dp_${ARM}_s${SEED}
RUN_NAME="pl_dp_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
SAVE_FREQ=$(( STEPS / 5 )); [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1
export GENESIS_SIM_VARIANT=$SIM_VARIANT SIM_VARIANT_FOR_SIDECAR=$SIM_VARIANT

# ---- provenance gates (the sbatch_dp.sh rules: 6-digit rollout stems, contract-v1 manifest, sim_variant, n_kept, fps) ----
[ -d "$RAW" ] || { echo "FATAL: raw place set $RAW missing (baselines/rl/place_demos.py cut)"; exit 1; }
ls "$RAW" | grep -E '\.npz$' | head -5 | grep -qE '^[0-9]{6}\.npz$' || { echo "FATAL: $RAW contents are not 6-digit rollout stems"; exit 1; }
N_SUCCESS=$(ls "$RAW"/*.npz | wc -l)
DEMO_SHA=$(python3 - "$RAW" "$SIM_VARIANT" "$ARM" <<'PY'
import json, os, sys
d, sv, arm = sys.argv[1:4]
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = json.load(open(os.path.join(d, 'manifest.json')))
assert m.get('contract') == 'v1' and m.get('phase') == 'place' and m.get('sim_variant') == sv, m
assert int(m['n_kept']) == len(files) == 39, (m.get('n_kept'), len(files))
assert m.get('builder') == 'baselines/rl/place_demos.py cut', m.get('builder')
if arm == 'dDP': assert m.get('one_per_ic') is True and m.get('keep_from'), 'machine set must be the matched-39 cut'
print(f'DEMO-SHA {arm} place n={len(files)} sha={m["content_sha256"][:16]} rows={m["decisions_total"]}', file=sys.stderr)
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
           eval_horizon="$EVAL_HORIZON" demo_format=native_place demo_sha="$DEMO_SHA" save_freq="$SAVE_FREQ" wave="$WAVE" sim_variant="$SIM_VARIANT" scope=place)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS RAW=$RAW (N=$N_SUCCESS sha=$DEMO_SHA) DATASET=$DATASET OUT=$OUT NODE=$NODE_CLASS SAVE_FREQ=$SAVE_FREQ"
  echo "[dry] train: lerobot-train --dataset.repo_id=local/${RUN_NAME} --dataset.root=$DATASET --policy.type=diffusion --policy.push_to_hub=false --seed=$SEED --output_dir=$OUT --batch_size=64 --steps=$STEPS --save_freq=$SAVE_FREQ --job_name=$RUN_NAME --wandb.enable=true --wandb.project=$PROJ --wandb.disable_artifact=true"
  echo "[dry] eval : KIND=dp CKPT=$OUT/checkpoints/$(printf %06d $STEPS)/pretrained_model OUT=$OUT ARM=$ARM SEED=$SEED HOLDE=$HOLDE POLE=$POLE bash cluster/place_eval_cells.sh"
  exit 0
fi
echo "== DP-PLACE $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) raw=$RAW sha=$DEMO_SHA dataset=$DATASET restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_dp_place.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_dp_place.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
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
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$TC" ]; then
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
GIT_HASH=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
for D in $(ls -d "$OUT"/checkpoints/[0-9]*/ | sort -V); do
  python3 - "$D" "$ARM" "$SEED" "$RAW" "$DATASET" "$GIT_HASH" "$STEPS" "$PROJ" "$ACTION_REPEAT" "$DEMO_SHA" "$NODE_CLASS" "$SIM_VARIANT" <<'PY'
import json, sys, pathlib as pl, datetime
d, arm, seed, raw, dataset, git, steps, proj, rep, sha, node, sv = sys.argv[1:13]
pl.Path(d, 'dp_sidecar.json').write_text(json.dumps({
    'script': 'sbatch_dp_place.sh', 'arm': arm, 'seed': int(seed), 'scope': 'place', 'raw_demo_dir': raw, 'dataset_root': dataset, 'git': git,
    'action_repeat': int(rep), 'delta_cap': 0.025, 'delta_leash': 0.125, 'demo_sha': sha, 'demo_format': 'native_place', 'node': node, 'sim_variant': sv,
    'ckpt_step': pl.Path(d).name, 'config': {'policy': 'diffusion', 'batch_size': 64, 'steps': int(steps), 'project': proj},
    'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')}, indent=1))
print(f'sidecar -> {d}dp_sidecar.json')
PY
done
set +e   # evals never fail an already-trained job
KIND=dp CKPT="$LAST_D/pretrained_model" OUT="$OUT" ARM="$ARM" SEED="$SEED" HOLDE="$HOLDE" POLE="$POLE" SIM_VARIANT="$SIM_VARIANT" PAR=2 bash cluster/place_eval_cells.sh 2>&1 | tee "$OUT/place_eval.log"
echo "JOB DONE $(date)"
