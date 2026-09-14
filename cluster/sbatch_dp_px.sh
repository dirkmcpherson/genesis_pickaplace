#!/bin/bash
# {Diffusion Policy} on PIXEL observations, end-to-end (full task) -- PHASE_PLAN amendment (ah), 2026-09-14.
#
# A copy of cluster/sbatch_dp_e2e.sh (the DP e2e recipe of record: lerobot Diffusion Policy, 100k grad steps,
# batch 64, ABSOLUTE window-end joint targets at fps 7.5, executed hold-4 through the env's delta integrator,
# preemption-safe resume, final checkpoint only) with ONE thing changed: the observation. It trains on
# $RAW/lerobot_px -- the same tapes, the same action column byte for byte (gated by
# baselines/verify_px_lerobot.py at build time), proprio 8 + observation.images.{top,wrist} at 64x64, and NO
# observation.environment_state, so the ground-truth can pose and goal xy never reach the policy.
#
# Augmentation: --policy.crop_shape=[56,56] --policy.crop_is_random=true. This is lerobot's analogue of the
# (af) pixel world models' `image_aug shift4` (DrQ replicate-pad 4 px + random crop back to 64): +-4 px of
# random translation in training, a deterministic CENTRE crop at evaluation, which lerobot's own
# DiffusionRgbEncoder does in eval mode. DISCLOSED DIFFERENCE: lerobot CROPS (56x56 input, ~23 % of the pixels
# discarded) where shift4 PADS and keeps 64x64. The vision backbone is the lerobot default (ResNet-18, group
# norm, spatial softmax, pretrained_backbone_weights=None -- trained from scratch, as the (af) encoder is).
#
# Submit (from the code checkout root; cluster/submit_ah_dp_px.sh does this with the guards):
#   for S in 0 1 2 3; do ARM=dH SEED=$S sbatch -J ah_dp_px_dH_s$S cluster/sbatch_dp_px.sh; done
#   for S in 0 1 2 3; do ARM=dM SEED=$S sbatch -J ah_dp_px_dM_s$S cluster/sbatch_dp_px.sh; done
# Env vars:
#   ARM        dH (-> $DEMO_ROOT/dHfull_all, 74 human tapes) | dM (-> dDPfull_first, 72 machine tapes,
#              PHASE_PLAN (v) FIRST-attempt-per-IC, de-selected). SEED required. STEPS 100000.
#   DEMO_ROOT  /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3
#   OUT_ROOT   baselines/outputs/dp_px  -> $OUT_ROOT/ah_dp_px_${ARM}_s${SEED}     PROJ genesis_paper   DRYRUN=1
#   LADDER     nested_sparse10 | TIP_GUARD not_in_hand -- the EVALUATION objective, stamped into the sidecar so
#              eval_e2e.py takes it from there. NOT specified by the registration: chosen so the cells are
#              comparable with the (af) pixel world-model `home` cells, which terminate on `home` under the same
#              guard. DP training itself never reads a reward, so this changes only where an episode ENDS and
#              which stage is the paid terminal. Override both to score under a different objective.
#   SAVE_FREQ  STEPS/2 (the disk rule of 2026-09-07: at most two numbered checkpoints during a run; after
#              training every checkpoint except the final is deleted together with the final's training_state).
#SBATCH -J ah_dp_px
#SBATCH -p gpu
#SBATCH --qos=normal
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-16:00:00
#SBATCH --output=ah_dp_px_%j.out
#SBATCH --error=ah_dp_px_%j.out

set -eo pipefail
# GENESIS_PICKAPLACE_ROOT IS REQUIRED -- no `:=$PWD` default (LADDER_UNIFY_BRIEF D1). DP never reads a reward,
# so its TRAINING is ladder-independent; but its EVALUATION goes through baselines/eval_e2e.py and therefore
# through this tree's full_env, so which tree it runs is exactly as load-bearing here as for the other learners.
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this run must use (no default: a launcher that silently takes \$PWD is how the two learners ended up on different ladders)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/full_env.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 1; }
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 1; fi
done
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
ARM=${ARM:?set ARM (dH | dM)}; SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}; PROJ=${PROJ:-genesis_paper}; WAVE=${WAVE:-ah_px}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
LADDER=${LADDER:-nested_sparse10}; TIP_GUARD=${TIP_GUARD:-not_in_hand}
CROP=${CROP:-56}                      # square crop side; 56 of 64 = the +-4 px shift budget
ACTION_REPEAT=4; EVAL_HORIZON=1200; DEVFLAG=(); [ -n "${DEVICE:-}" ] && DEVFLAG=(--policy.device="$DEVICE")
DEMO_ROOT=${DEMO_ROOT:-/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3}
case "$ARM" in
  dH) SET=dHfull_all;    N_EXP=74 ;;
  dM) SET=dDPfull_first; N_EXP=72 ;;   # PHASE_PLAN (v): FIRST attempt per IC (de-selected)
  *) echo "FATAL: ARM must be dH | dM (got $ARM)"; exit 1 ;;
esac
AMEND=ah
RAW=$DEMO_ROOT/$SET; DATASET=$RAW/lerobot_px; REF_DATASET=$RAW/lerobot
OUT_ROOT=${OUT_ROOT:-baselines/outputs/dp_px}
OUT=$OUT_ROOT/ah_dp_px_${ARM}_s${SEED}
RUN_NAME="ah_dp_px_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
SAVE_FREQ=${SAVE_FREQ:-$(( STEPS / 2 ))}; [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1
export GENESIS_SIM_VARIANT=$SIM_VARIANT SIM_VARIANT_FOR_SIDECAR=$SIM_VARIANT

# ---- disk guard (2026-09-07 filesystem-full incident; registered floor 150 GB; lowered to 100 GB 2026-09-14 09:10 by the user) ----
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9')
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 100 ] || { echo "FATAL: /cluster/tufts/shortlab free ${FREE_GB:-?} GB < 100 GB floor -- refusing to train"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"

# ---- provenance gates: the raw tapes are the (n)/(ab) tapes; the dataset is the PIXEL one ----
[ -d "$RAW" ] || { echo "FATAL: raw full-task set $RAW missing (cluster/e2e_build_sets.sh)"; exit 1; }
ls "$RAW" | grep -E '\.npz$' | head -5 | grep -qE '^[0-9]{6}\.npz$' || { echo "FATAL: $RAW contents are not 6-digit rollout stems"; exit 1; }
DEMO_SHA=$(python3 - "$RAW" "$SIM_VARIANT" "$ARM" "$N_EXP" <<'PY'
import json, os, sys
d, sv, arm, n_exp = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = json.load(open(os.path.join(d, 'manifest.json')))
assert m.get('contract') == 'v1' and m.get('sim_variant') == sv, m
assert m.get('scope') == 'full', m
assert int(m['n_kept']) == len(files) == n_exp, (m.get('n_kept'), len(files), n_exp)
assert m.get('builder') == 'baselines/rl/full_demos.py select', m.get('builder')
# the machine arm is the DE-SELECTED first-attempt set (PHASE_PLAN (v)); the human arm is every tape
assert (m.get('one_per_ic_first') is True) == (arm == 'dM'), m
assert m.get('one_per_ic_best') is not True, ('best-of-3 selection is not the (ah) machine arm', m)
print(f'DEMO-SHA {arm} full n={len(files)} sha={m["content_sha256"][:16]} decisions={m["decisions_total"]} '
      f'tape_reward={m["tape_reward_total"]:.0f} idle={m["idle_frac"]:.3f} (pre-pick {m["idle_frac_prepick"]:.3f}) '
      f'stages={m["stage_yields"]}', file=sys.stderr)
print(m['content_sha256'][:16])
PY
) || exit 1
[ -d "$DATASET" ] || { echo "FATAL: pixel lerobot dataset $DATASET missing (cluster/ah_build_pixel_sets.sh)"; exit 1; }
[ -d "$REF_DATASET" ] || { echo "FATAL: state dataset of record $REF_DATASET missing -- the action gate has nothing to compare to"; exit 1; }
# gate 2 of the registration, re-run in the job: the action column IS the (n)/(ab) column, and the dataset
# carries no environment_state. Cheap (parquet only) and it makes a swapped dataset impossible to train on.
python3 baselines/verify_px_lerobot.py --px "$DATASET" --ref "$REF_DATASET" --raw "$RAW" || exit 1
python3 - "$DATASET" "$N_EXP" "$ACTION_REPEAT" "$RAW" <<'PY' || exit 1
import json, sys, pathlib as pl
ds, n_exp, rep, raw = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
info = json.loads((pl.Path(ds) / 'meta' / 'info.json').read_text())
man = json.loads((pl.Path(raw) / 'manifest.json').read_text())
assert abs(float(info['fps']) - 30.0 / rep) < 1e-6, (info['fps'], rep)
assert int(man['n_kept']) == n_exp, (man['n_kept'], n_exp)
assert info['total_episodes'] == int(man['n_lerobot']), (info['total_episodes'], man['n_lerobot'])
assert int(info['total_frames']) == int(man['decisions_lerobot']), (info['total_frames'], man['decisions_lerobot'])
src = json.loads((pl.Path(ds) / 'genesis_source.json').read_text())
assert src.get('contract') == 'v1', src
assert src.get('no_env_state') is True and src.get('images_from'), src
assert sorted(src.get('cameras') or []) == ['top', 'wrist'], src
print(f'PROVENANCE-OK dataset={ds} total_episodes={info["total_episodes"]}/{man["n_kept"]} '
      f'total_frames={info["total_frames"]}/{man["decisions_total"]} fps={info["fps"]} '
      f'cameras={src["cameras"]} img_dtype={src["img_dtype"]} images_from={src["images_from"]} '
      f'short_tapes={man["short_tapes"]}')
PY
REG_KNOBS=(steps="$STEPS" budget_unit=grad_steps batch_size=64 policy=diffusion dataset_root="$DATASET" action_repeat="$ACTION_REPEAT"
           eval_horizon="$EVAL_HORIZON" demo_format=full_tapes demo_sha="$DEMO_SHA" save_freq="$SAVE_FREQ" wave="$WAVE"
           sim_variant="$SIM_VARIANT" scope=full amendment="$AMEND" obs=pixels crop_shape="${CROP}x${CROP}"
           crop_is_random=true ladder="$LADDER" tip_guard="$TIP_GUARD")
TRAIN_ARGS=(--dataset.repo_id="local/${RUN_NAME}" --dataset.root="$DATASET" --policy.type=diffusion --policy.push_to_hub=false
            --seed="$SEED" --output_dir="$OUT" --batch_size=64 --steps="$STEPS" --save_freq="$SAVE_FREQ" --job_name="$RUN_NAME"
            --wandb.enable=true --wandb.project="$PROJ" --wandb.disable_artifact=true
            --policy.crop_shape="[$CROP,$CROP]" --policy.crop_is_random=true "${DEVFLAG[@]}")
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS RAW=$RAW (N=$N_EXP sha=$DEMO_SHA) DATASET=$DATASET OUT=$OUT NODE=$NODE_CLASS SAVE_FREQ=$SAVE_FREQ LADDER=$LADDER TIP_GUARD=$TIP_GUARD"
  echo "[dry] train: lerobot-train ${TRAIN_ARGS[*]}"
  echo "[dry] prune: keep checkpoints/$(printf %06d $STEPS)/pretrained_model only (drop other numbered ckpts + training_state)"
  echo "[dry] eval : KIND=dp CAMERA_RIG=1 CKPT=$OUT/checkpoints/$(printf %06d $STEPS)/pretrained_model OUT=$OUT ARM=$ARM SEED=$SEED bash cluster/e2e_eval_cells.sh"
  exit 0
fi
echo "== DP-PX $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) raw=$RAW sha=$DEMO_SHA dataset=$DATASET restart=${SLURM_RESTART_COUNT:-0}"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_dp_px.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_dp_px.sh --arm "$ARM" --seed "$SEED" --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
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
  $LEROBOT_TRAIN "${TRAIN_ARGS[@]}"
fi
LAST_D=$(ls -d "$OUT"/checkpoints/[0-9]*/ 2>/dev/null | sort -V | tail -1)
[ -n "$LAST_D" ] && [ -d "$LAST_D/pretrained_model" ] || { echo "FATAL: no numbered checkpoint under $OUT/checkpoints"; exit 1; }
[ "$(basename "$LAST_D")" = "$(printf %06d "$STEPS")" ] || { echo "FATAL: last checkpoint $(basename "$LAST_D") != budget $STEPS -- training did not reach its budget; no evaluation of a partial run"; exit 1; }

# ---- what the policy ACTUALLY resolved to (gate 3 of the registration, read back from the saved config,
# ---- never from the flags we passed) --------------------------------------------------------------------
python3 - "$LAST_D/pretrained_model" "$CROP" <<'PY' || exit 1
import json, pathlib as pl, sys
cfg = json.loads((pl.Path(sys.argv[1]) / 'config.json').read_text()); crop = int(sys.argv[2])
feats = {k: v for k, v in (cfg.get('input_features') or {}).items()}
shape = lambda k: tuple((feats[k].get('shape') or feats[k].get('_shape') or []))
print('POLICY-CONFIG crop_shape=%s crop_is_random=%s vision_backbone=%s pretrained_backbone_weights=%s '
      'use_group_norm=%s separate_rgb_encoder_per_camera=%s spatial_softmax_num_keypoints=%s n_obs_steps=%s '
      'horizon=%s n_action_steps=%s' % (
          cfg.get('crop_shape'), cfg.get('crop_is_random'), cfg.get('vision_backbone'),
          cfg.get('pretrained_backbone_weights'), cfg.get('use_group_norm'),
          cfg.get('use_separate_rgb_encoder_per_camera'), cfg.get('spatial_softmax_num_keypoints'),
          cfg.get('n_obs_steps'), cfg.get('horizon'), cfg.get('n_action_steps')))
print('POLICY-INPUTS ' + ' '.join(f'{k}{shape(k)}' for k in sorted(feats)))
assert tuple(cfg.get('crop_shape') or ()) == (crop, crop), ('crop_shape', cfg.get('crop_shape'))
assert cfg.get('crop_is_random') is True, ('crop_is_random', cfg.get('crop_is_random'))
assert cfg.get('pretrained_backbone_weights') in (None, 'null'), cfg.get('pretrained_backbone_weights')
want = {'observation.state', 'observation.images.top', 'observation.images.wrist'}
assert set(feats) == want, ('input_features', sorted(feats), sorted(want))
assert shape('observation.state') == (8,), shape('observation.state')
print('POLICY-CONFIG-OK: pixel inputs only, 8-d proprio, random 56x56 crop, backbone from scratch')
PY

# ---- DISK RULE (2026-09-07): keep ONLY the final checkpoint's weights ----------------------
BEFORE_KB=$(du -sk "$OUT" | cut -f1)
FINAL_N=$(basename "$LAST_D")
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
  python3 - "$D" "$ARM" "$SEED" "$RAW" "$DATASET" "$GIT_HASH" "$STEPS" "$PROJ" "$ACTION_REPEAT" "$DEMO_SHA" "$NODE_CLASS" "$SIM_VARIANT" "$AMEND" "$LADDER" "$TIP_GUARD" "$CROP" <<'PY'
import json, sys, pathlib as pl, datetime
d, arm, seed, raw, dataset, git, steps, proj, rep, sha, node, sv, amend, ladder, guard, crop = sys.argv[1:17]
pl.Path(d, 'dp_sidecar.json').write_text(json.dumps({
    'script': 'sbatch_dp_px.sh', 'arm': arm, 'seed': int(seed), 'scope': 'full', 'raw_demo_dir': raw, 'dataset_root': dataset, 'git': git,
    'action_repeat': int(rep), 'delta_cap': 0.025, 'delta_leash': 0.125, 'demo_sha': sha, 'demo_format': 'full_tapes', 'node': node, 'sim_variant': sv,
    'ckpt_step': pl.Path(d).name, 'amendment': amend,
    # eval_e2e.py takes the ladder and the tip guard from HERE (the sidecar is the source, --ladder/--tip-guard
    # are optional assertions), so a cell can never be scored under an objective nobody chose.
    'ladder': ladder, 'tip_guard': guard,
    'obs': 'pixels', 'image_keys': ['observation.images.top', 'observation.images.wrist'],
    'image_shape': [64, 64, 3], 'proprio_dim': 8, 'environment_state': False,
    'augmentation': {'kind': 'lerobot_crop', 'crop_shape': [int(crop), int(crop)], 'crop_is_random': True,
                     'eval': 'centre crop (lerobot eval-mode behaviour)',
                     'analogue_of': '(af) image_aug shift4 (DrQ replicate-pad 4 px + random crop back to 64)'},
    'config': {'policy': 'diffusion', 'batch_size': 64, 'steps': int(steps), 'project': proj},
    'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')}, indent=1))
print(f'sidecar -> {d}dp_sidecar.json')
PY
done
set +e   # evals never fail an already-trained job
KIND=dp CAMERA_RIG=1 CKPT="$LAST_D/pretrained_model" OUT="$OUT" ARM="$ARM" SEED="$SEED" SIM_VARIANT="$SIM_VARIANT" \
  SETS="${SETS:-hold15 rnd30}" MODES="${MODES:-sample}" VIDEO_SETS="${VIDEO_SETS:-rnd30}" ISO="${ISO:-0}" PAR=${PAR:-3} \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_eval.log"
echo "JOB DONE $(date)"
