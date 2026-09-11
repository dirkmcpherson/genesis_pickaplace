#!/bin/bash
# DP END-TO-END (full task) launcher (PHASE_PLAN_2026-09-04 amendment (n), 2026-09-07): the DP recipe of record
# (cluster/sbatch_dp.sh: lerobot Diffusion Policy, state-only, ABSOLUTE window-end joint targets at fps 7.5, 100k
# grad steps, batch 64, preemption-safe resume) on the SAME full-task tapes the world model and RLPD train on
# (baselines/rl/full_demos.py select -> convert_to_lerobot.py), then the end-to-end evaluation cells of
# cluster/e2e_eval_cells.sh on the LAST (100k) checkpoint, executed hold-4 through the env's delta integrator.
#
# Submit (from the code checkout root):
#   for S in $(seq 0 7); do ARM=dH  SEED=$S sbatch -J e2e_dp_dH_s$S  cluster/sbatch_dp_e2e.sh; done
#   for S in $(seq 0 7); do ARM=dDP SEED=$S sbatch -J e2e_dp_dDP_s$S cluster/sbatch_dp_e2e.sh; done
# Env vars:
#   ARM        dH | dDP   (dH -> $DEMO_ROOT/dHfull_all [74], dDP -> $DEMO_ROOT/dDPfull [72]; RAW npz + manifest + lerobot/)
#   SEED       required     STEPS 100000     DEMO_ROOT /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3
#   OUT_ROOT   baselines/outputs/dp_e2e  -> $OUT_ROOT/e2e_dp_${ARM}_s${SEED}     PROJ genesis_paper    DRYRUN=1
#   DEVICE     unset (default; the runs of record use lerobot's own device choice = cuda). Setting DEVICE=cpu adds
#             --policy.device=cpu so a pipeline smoke can run on an idle CPU node -- never for a reported number.
#   SAVE_FREQ  STEPS/2 -- DISK RULE (2026-09-07 incident): at most TWO numbered checkpoints exist during a run (one
#             mid-run resume point for the preempt queue), and after training every checkpoint except the final is
#             deleted together with the final's training_state -- 949 MB per run at rest. A requeue AFTER the budget
#             was reached skips training entirely instead of resuming.
# DISCLOSED (amendment (n) disconfirm branch (iii)): the human tapes here are RAW (unpruned), so DP carries the human
# arm's idle decisions; a machine-ahead gap of >= 0.10 triggers the registered human-PRUNED control before any
# "source effect" reading. The idle fractions of both sets are in their manifests.
#SBATCH -J e2e_dp
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
#SBATCH --time=0-16:00:00
#SBATCH --output=e2e_dp_%j.out
#SBATCH --error=e2e_dp_%j.out

set -eo pipefail
# GENESIS_PICKAPLACE_ROOT IS REQUIRED -- no `:=$PWD` default (LADDER_UNIFY_BRIEF D1). DP never
# reads a reward, so its TRAINING is ladder-independent; but its EVALUATION goes through
# baselines/eval_e2e.py and therefore through this tree's full_env, so which tree it runs is
# exactly as load-bearing here as for the other two learners.
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this run must use (no default: a launcher that silently takes \$PWD is how the two learners ended up on different ladders)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/full_env.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 1; }
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 1; fi
done
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
ARM=${ARM:?set ARM (dH | dDP | dDPfirst)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}; PROJ=${PROJ:-genesis_paper}; WAVE=${WAVE:-e2e}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
ACTION_REPEAT=4; EVAL_HORIZON=1200; DEVFLAG=(); [ -n "${DEVICE:-}" ] && DEVFLAG=(--policy.device="$DEVICE")
DEMO_ROOT=${DEMO_ROOT:-/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3}
case "$ARM" in
  dH)  SET=dHfull_all; N_EXP=74 ;;
  dDP) SET=dDPfull;    N_EXP=72 ;;
  dDPfirst) SET=dDPfull_first; N_EXP=72 ;;   # PHASE_PLAN (v): FIRST attempt per IC (de-selected)
  *) echo "FATAL: ARM must be dH | dDP | dDPfirst (got $ARM)"; exit 1 ;;
esac
RAW=$DEMO_ROOT/$SET; DATASET=$RAW/lerobot
OUT_ROOT=${OUT_ROOT:-baselines/outputs/dp_e2e}
OUT=$OUT_ROOT/e2e_dp_${ARM}_s${SEED}
RUN_NAME="e2e_dp_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
SAVE_FREQ=${SAVE_FREQ:-$(( STEPS / 2 ))}; [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1
export GENESIS_SIM_VARIANT=$SIM_VARIANT SIM_VARIANT_FOR_SIDECAR=$SIM_VARIANT

# ---- disk guard (2026-09-07 filesystem-full incident; registered floor 150 GB) ----
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9')
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: /cluster/tufts/shortlab free ${FREE_GB:-?} GB < 150 GB floor -- refusing to train"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"

# ---- provenance gates (sbatch_dp.sh rules: 6-digit rollout stems, contract-v1 manifest, sim_variant, n_kept, fps) ----
[ -d "$RAW" ] || { echo "FATAL: raw full-task set $RAW missing (cluster/e2e_build_sets.sh)"; exit 1; }
ls "$RAW" | grep -E '\.npz$' | head -5 | grep -qE '^[0-9]{6}\.npz$' || { echo "FATAL: $RAW contents are not 6-digit rollout stems"; exit 1; }
DEMO_SHA=$(python3 - "$RAW" "$SIM_VARIANT" "$ARM" "$N_EXP" <<'PY'
import json, os, sys
d, sv, arm, n_exp = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = json.load(open(os.path.join(d, 'manifest.json')))
assert m.get('contract') == 'v1' and m.get('scope') == 'full' and m.get('sim_variant') == sv, m
assert int(m['n_kept']) == len(files) == n_exp, (m.get('n_kept'), len(files), n_exp)
assert m.get('builder') == 'baselines/rl/full_demos.py select', m.get('builder')
assert (m.get('one_per_ic_best') is True) == (arm == 'dDP'), m
assert (m.get('one_per_ic_first') is True) == (arm == 'dDPfirst'), m   # PHASE_PLAN (v)
print(f'DEMO-SHA {arm} full n={len(files)} sha={m["content_sha256"][:16]} decisions={m["decisions_total"]} '
      f'tape_reward={m["tape_reward_total"]:.0f} idle={m["idle_frac"]:.3f} (pre-pick {m["idle_frac_prepick"]:.3f}) '
      f'stages={m["stage_yields"]}', file=sys.stderr)
print(m['content_sha256'][:16])
PY
) || exit 1
[ -d "$DATASET" ] || { echo "FATAL: lerobot dataset $DATASET missing (cluster/e2e_build_sets.sh)"; exit 1; }
python3 - "$DATASET" "$N_EXP" "$ACTION_REPEAT" "$RAW" <<'PY' || exit 1
import json, sys, pathlib as pl
ds, n_exp, rep, raw = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
info = json.loads((pl.Path(ds) / 'meta' / 'info.json').read_text())
man = json.loads((pl.Path(raw) / 'manifest.json').read_text())
# the raw set carries every tape (n_kept == n_exp, gated above); the lerobot dataset holds the tapes long enough to
# form a DP sample (n_lerobot; convert_to_lerobot drops episodes under MIN_FRAMES -- disclosed, never silent)
assert int(man['n_kept']) == n_exp, (man['n_kept'], n_exp)
assert info['total_episodes'] == int(man['n_lerobot']), (info['total_episodes'], man['n_lerobot'])
assert abs(float(info['fps']) - 30.0 / rep) < 1e-6, (info['fps'], rep)
assert int(info['total_frames']) == int(man['decisions_lerobot']), (info['total_frames'], man['decisions_lerobot'])
src = json.loads((pl.Path(ds) / 'genesis_source.json').read_text()); assert src.get('contract') == 'v1', src
print(f'PROVENANCE-OK dataset={ds} total_episodes={info["total_episodes"]}/{man["n_kept"]} '
      f'total_frames={info["total_frames"]}/{man["decisions_total"]} fps={info["fps"]} short_tapes={man["short_tapes"]}')
PY
REG_KNOBS=(steps="$STEPS" budget_unit=grad_steps batch_size=64 policy=diffusion dataset_root="$DATASET" action_repeat="$ACTION_REPEAT"
           eval_horizon="$EVAL_HORIZON" demo_format=full_tapes demo_sha="$DEMO_SHA" save_freq="$SAVE_FREQ" wave="$WAVE"
           sim_variant="$SIM_VARIANT" scope=full amendment=n)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS RAW=$RAW (N=$N_EXP sha=$DEMO_SHA) DATASET=$DATASET OUT=$OUT NODE=$NODE_CLASS SAVE_FREQ=$SAVE_FREQ"
  echo "[dry] train: lerobot-train --dataset.repo_id=local/${RUN_NAME} --dataset.root=$DATASET --policy.type=diffusion --policy.push_to_hub=false --seed=$SEED --output_dir=$OUT --batch_size=64 --steps=$STEPS --save_freq=$SAVE_FREQ --job_name=$RUN_NAME --wandb.enable=true --wandb.project=$PROJ --wandb.disable_artifact=true"
  echo "[dry] prune: keep checkpoints/$(printf %06d $STEPS)/pretrained_model only (drop other numbered ckpts + training_state)"
  echo "[dry] eval : KIND=dp CKPT=$OUT/checkpoints/$(printf %06d $STEPS)/pretrained_model OUT=$OUT ARM=$ARM SEED=$SEED bash cluster/e2e_eval_cells.sh"
  exit 0
fi
echo "== DP-E2E $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) raw=$RAW sha=$DEMO_SHA dataset=$DATASET restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_dp_e2e.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_dp_e2e.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
fi
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"
if command -v lerobot-train >/dev/null 2>&1; then LEROBOT_TRAIN="lerobot-train"
elif python -c 'import lerobot.scripts.lerobot_train' 2>/dev/null; then LEROBOT_TRAIN="python -m lerobot.scripts.lerobot_train"
else echo "FATAL: lerobot is not importable in this env ($CONDA_PREFIX)"; exit 1; fi
TC="$OUT/checkpoints/last/pretrained_model/train_config.json"
DONE_CK=$OUT/checkpoints/$(printf %06d "$STEPS")/pretrained_model
if [ -d "$DONE_CK" ]; then
  echo "== TRAIN-DONE-ALREADY: $DONE_CK exists; skipping training (requeue #${SLURM_RESTART_COUNT:-0})"
elif [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$TC" ]; then
  echo "== requeued (restart #${SLURM_RESTART_COUNT}); RESUMING via $TC"
  $LEROBOT_TRAIN --config_path="$TC" --resume=true
else
  rm -rf "$OUT"
  $LEROBOT_TRAIN --dataset.repo_id="local/${RUN_NAME}" --dataset.root="$DATASET" --policy.type=diffusion --policy.push_to_hub=false \
    --seed="$SEED" --output_dir="$OUT" --batch_size=64 --steps="$STEPS" --save_freq="$SAVE_FREQ" --job_name="$RUN_NAME" \
    --wandb.enable=true --wandb.project="$PROJ" --wandb.disable_artifact=true "${DEVFLAG[@]}"
fi
LAST_D=$(ls -d "$OUT"/checkpoints/[0-9]*/ 2>/dev/null | sort -V | tail -1)
[ -n "$LAST_D" ] && [ -d "$LAST_D/pretrained_model" ] || { echo "FATAL: no numbered checkpoint under $OUT/checkpoints"; exit 1; }
[ "$(basename "$LAST_D")" = "$(printf %06d "$STEPS")" ] || { echo "FATAL: last checkpoint $(basename "$LAST_D") != budget $STEPS -- training did not reach its budget; no evaluation of a partial run"; exit 1; }
# ---- DISK RULE (2026-09-07): keep ONLY the final checkpoint's weights ----------------------
BEFORE_KB=$(du -sk "$OUT" | cut -f1)
FINAL_N=$(basename "$LAST_D")
# KEEP_CKPTS=1 (2026-09-09): retain intermediate checkpoints so the convergence
# sweep has more than one point. DP is offline -- it has no rollouts and therefore no
# acquisition curve; a checkpoint-vs-performance sweep is the ONLY convergence evidence
# available for this learner, and the 100k budget currently rests on none.
if [ "${KEEP_CKPTS:-0}" = 1 ]; then echo "== KEEP_CKPTS=1: retaining intermediate checkpoints"; else
for D in $(ls -d "$OUT"/checkpoints/[0-9]*/ | sort -V); do
  N=$(basename "$D")
  [ "$N" = "$FINAL_N" ] && continue
  echo "== pruning superseded checkpoint $D"; rm -rf "$D"
done
fi
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
    'script': 'sbatch_dp_e2e.sh', 'arm': arm, 'seed': int(seed), 'scope': 'full', 'raw_demo_dir': raw, 'dataset_root': dataset, 'git': git,
    'action_repeat': int(rep), 'delta_cap': 0.025, 'delta_leash': 0.125, 'demo_sha': sha, 'demo_format': 'full_tapes', 'node': node, 'sim_variant': sv,
    'ckpt_step': pl.Path(d).name, 'amendment': 'n',
    'config': {'policy': 'diffusion', 'batch_size': 64, 'steps': int(steps), 'project': proj},
    'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')}, indent=1))
print(f'sidecar -> {d}dp_sidecar.json')
PY
done
set +e   # evals never fail an already-trained job
KIND=dp CKPT="$LAST_D/pretrained_model" OUT="$OUT" ARM="$ARM" SEED="$SEED" SIM_VARIANT="$SIM_VARIANT" \
  SETS="${SETS:-hold15 rnd30 spots60}" MODES="${MODES:-sample}" VIDEO_SETS="${VIDEO_SETS:-rnd30}" PAR=${PAR:-3} \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_eval.log"
echo "JOB DONE $(date)"
