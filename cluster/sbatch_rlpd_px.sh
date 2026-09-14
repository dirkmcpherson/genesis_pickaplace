#!/bin/bash
# RLPD END-TO-END *PIXEL* launcher (lane PXR-1, 2026-09-13; paper/PX_RLPD_PIPELINE_2026-09-13.md).
# = cluster/sbatch_rlpd_e2e.sh (the {RLPD} recipe of record: UTD 10, E10/Z2, LN critics, gamma 0.99, 50/50 demo
# batches, delta_joint cap 0.025 / leash 5x, repeat 4, FullTaskEnv(scope='full') with LADDER + TIP_GUARD as
# constructor arguments) with the policy observing PIXELS + PROPRIOCEPTION instead of the 17-dim state:
#   --obs pixels  = {image: genv.rig_obs() (64,64,6) uint8 top RGB ++ wrist RGB, proprio: state[:8]}, NO can/goal pose,
#                   DrQ-v2 encoder owned by the critic (actor detached), DrQ random shift on every update batch,
#                   demos from the RENDERED-image sets (relabel_reward.py --images) -- baselines/rl/rlpd_pixel.py.
# Differences from sbatch_rlpd_e2e.sh, and nothing else:
#   * OBS=pixels is REQUIRED (this launcher refuses anything else; a state run uses sbatch_rlpd_e2e.sh);
#   * the demo set is the `_img` build of the run's ladder ($W/demos_state_full/<arm>_rns10h_img | _rnrh_img) and the
#     provenance gate is the e2e gate PLUS the pixel rules (DERIVED from PXC-1's wmfix_full.sbatch gate, never weaker):
#     repeat.json `images == rendered` AND `state_only == false`, the set's recorded ladder AND tip guard equal the
#     run's, the first tape's image column is (T,64,64,6) uint8 with > 99.9 % non-blank frames;
#   * --buffer-size $BUFFER_SIZE (default 300000 = the recipe) with a HOST-MEMORY check: SB3's DictReplayBuffer holds
#     obs AND next_obs, 2 x 24576 B per row -> 14.7 GB at 300k rows; plus the demo half (<= 0.9 GB, pinned), the
#     Genesis world and the CUDA context (~5 GB RSS measured locally with a 5k buffer) -> ~21 GB; --mem=48g;
#   * the evaluation is NOT run on the GPU node: after TRAIN-OK this job submits ONE dependent CPU job
#     (cluster/sbatch_rlpd_px_eval.sh, -p batch, excluded = every non-64-physical-core node + the SMT nodes, the same
#     lists the milestone sweep uses, REQUIRE_CORES=64) that runs e2e_eval_cells.sh with EVAL_SCRIPT=eval_e2e_px.py:
#     hold15 + rnd30 x {sample, mode} (+ the rnd30 _iso cells), i.e. the pixel evaluator on the class of record;
#   * QOS: `-p gpu,preempt --qos=preempt --nice=0` (the 20-GPU preempt allocation, user 2026-09-13), one GPU, --requeue
#     (a requeued run restarts CLEAN, as every e2e launcher does).
# Submit (GENESIS_PICKAPLACE_ROOT is REQUIRED and must be a tree at/after 7c23d04, never a pinned/in-flight one):
#   GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr OBS=pixels LADDER=nested_sparse10 TIP_GUARD=not_in_hand ARM=dH SEED=0 \
#       sbatch -J e2e_rlpd_px_dH_s0 cluster/sbatch_rlpd_px.sh
#   ARM=dDPfirst for the machine arm. SMOKE: STEPS=2000 BUFFER_SIZE=5000 WAVE=pxsmoke EVAL_SETS=hold15 EVAL_LIMIT=3 EVAL_ISO=0
# Env vars:
#   OBS        pixels (required, the only accepted value)      ARM  dH | dDPfirst (required)     SEED required
#   LADDER     nested_sparse10 | nested_ramp (required; selects the `_img` set suffix)   TIP_GUARD not_in_hand (required)
#   STEPS      250000 decisions      BUFFER_SIZE 300000 rows      IMAGE_AUG shift4 | none      GAMMA 0.99
#   OUT_ROOT   baselines/rl/checkpoints/e2e_px -> $OUT_ROOT/e2e_rlpd_px_${ARM}_s${SEED}     CKPT_FRACS 0.4,1.0
#   EVAL_SETS "hold15 rnd30"  EVAL_MODES "sample mode"  EVAL_ISO 1  EVAL_ISO_SETS rnd30  EVAL_VIDEO_SETS rnd30
#   EVAL_LIMIT (starts per set, smokes)  EVAL_PAR 8  REQUIRE_CORES 64  THREADS ''  EXCL64 $LAB/gp_dp_e2e/.excl64.txt
#   NO_EVAL=1  train only (no dependent eval job)      DRYRUN=1  print the plan and exit
#SBATCH -J e2e_rlpd_px
#SBATCH -p gpu,preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=0
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=1-06:00:00
#SBATCH --output=e2e_rlpd_px_%j.out
#SBATCH --error=e2e_rlpd_px_%j.out

set -eo pipefail
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this run must use (no default: a launcher that silently takes \$PWD is how the two learners ended up on different ladders)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/full_env.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 1; }
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/rlpd_pixel.py" ] && [ -f "$GENESIS_PICKAPLACE_ROOT/baselines/eval_e2e_px.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT predates the pixel path (7c23d04)"; exit 1; }
case "$GENESIS_PICKAPLACE_ROOT" in *gp_ladderN*|*gp_unified*|*gp_e2e*|*gp_root*|*gp_px|*gp_px/*|*gp_ac*|*gp_aa4*) echo "FATAL: refusing the pinned/in-flight/other-lane tree $GENESIS_PICKAPLACE_ROOT"; exit 1;; esac
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 1; fi
done
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
OBS=${OBS:?set OBS=pixels (this is the pixel launcher; a state run uses sbatch_rlpd_e2e.sh)}
[ "$OBS" = pixels ] || { echo "FATAL: OBS must be 'pixels' for this launcher (got $OBS)"; exit 1; }
ARM=${ARM:?set ARM (dH | dDPfirst)}; SEED=${SEED:?set SEED}
LADDER=${LADDER:?set LADDER (nested_sparse10 | nested_ramp) -- the reward ladder is never defaulted}
case "$LADDER" in nested_sparse10) SFX=_rns10h_img ;; nested_ramp) SFX=_rnrh_img ;;
  *) echo "FATAL: LADDER must be nested_sparse10 | nested_ramp (the rendered-image sets that exist; got $LADDER)"; exit 1 ;; esac
FAR_RELEASE=${FAR_RELEASE:-}
case "${FAR_RELEASE}" in ""|0) FAR_FLAG="" ;; 1) FAR_FLAG="--far-release" ;; *) echo "FATAL: FAR_RELEASE must be 0 or 1"; exit 1 ;; esac
TIP_GUARD=${TIP_GUARD:?set TIP_GUARD (grip | not_in_hand) -- never defaulted; PHASE_PLAN (aa)}
case "$TIP_GUARD" in grip|not_in_hand) ;; *) echo "FATAL: TIP_GUARD must be grip | not_in_hand (got $TIP_GUARD)"; exit 1 ;; esac
STEPS=${STEPS:-250000}; WAVE=${WAVE:-e2e_px}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; GAMMA=${GAMMA:-0.99}
BUFFER_SIZE=${BUFFER_SIZE:-300000}; IMAGE_AUG=${IMAGE_AUG:-shift4}
case "$IMAGE_AUG" in shift4|none) ;; *) echo "FATAL: IMAGE_AUG must be shift4 | none (got $IMAGE_AUG)"; exit 1 ;; esac
ACTION_REPEAT=4; TRAIN_HORIZON=1200; EVAL_HORIZON=1200; DEVICE=${DEVICE:-cuda}
case "$ARM" in
  dH)       DEMO=${DEMO:-$W/demos_state_full/dHfull_all$SFX};       N_EXP=74 ;;
  dDPfirst) DEMO=${DEMO:-$W/demos_state_full/dDPfull_first$SFX};    N_EXP=72 ;;   # PHASE_PLAN (v): FIRST attempt per IC
  *) echo "FATAL: ARM must be dH | dDPfirst (got $ARM)"; exit 1 ;;
esac
OUT_ROOT=${OUT_ROOT:-baselines/rl/checkpoints/e2e_px}
OUT=$OUT_ROOT/e2e_rlpd_px_${ARM}_s${SEED}
RUN_NAME="e2e_rlpd_px_${ARM}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
export GENESIS_SIM_VARIANT=$SIM_VARIANT
# eval job knobs (the dependent CPU job; see the header)
EVAL_SETS=${EVAL_SETS:-"hold15 rnd30"}; EVAL_MODES=${EVAL_MODES:-"sample mode"}; EVAL_ISO=${EVAL_ISO:-1}
EVAL_ISO_SETS=${EVAL_ISO_SETS:-rnd30}; EVAL_VIDEO_SETS=${EVAL_VIDEO_SETS:-rnd30}; EVAL_LIMIT=${EVAL_LIMIT:-}
EVAL_PAR=${EVAL_PAR:-8}; REQUIRE_CORES=${REQUIRE_CORES:-64}; THREADS=${THREADS:-}
EXCL64=${EXCL64:-$LAB/gp_dp_e2e/.excl64.txt}     # READ ONLY: "every node that is NOT 64 physical cores" (hw census)
SMT_EXCL=${SMT_EXCL:-pax006,pax012,pax044,pax036,pax037,pax038,pax039,pax040,pax041,pax043,pax045,pax046,pax056,pax066}

# ---- disk guard (2026-09-07 filesystem-full incident; registered floor 150 GB) ----
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: /cluster/tufts/shortlab free ${FREE_GB:-?} GB < 150 GB floor -- refusing to train"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"
# the conda env FIRST: the pixel demo gate below reads a tape with numpy (the login node's python3 has none)
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'

# ---- host-memory arithmetic for the image buffer (never a silent OOM at 300k rows) ----
MEM_MB=${SLURM_MEM_PER_NODE:-49152}
python - "$BUFFER_SIZE" "$MEM_MB" <<'PY' || exit 1
import sys
rows, mem_mb = int(sys.argv[1]), int(sys.argv[2])
row_b = 2 * 64 * 64 * 6                      # obs + next_obs, uint8 (64,64,6) each (SB3 DictReplayBuffer)
buf_gb = rows * row_b / 1e9
need_gb = buf_gb + 0.9 + 5.0                 # + demo half (<= 0.9 GB pinned) + world/CUDA/torch (~5 GB RSS measured)
have_gb = mem_mb / 1024
print(f'MEM-CHECK buffer {rows} rows x {row_b} B = {buf_gb:.1f} GB; est. process {need_gb:.1f} GB; allocation {have_gb:.1f} GB')
assert need_gb < 0.85 * have_gb, f'FATAL: estimated {need_gb:.1f} GB > 85% of the {have_gb:.1f} GB allocation; raise --mem or lower BUFFER_SIZE'
PY

# ---- provenance gate: the e2e gate + the PIXEL rules (derived from PXC-1's wmfix_full.sbatch gate) ----
DEMO_SHA=$(LADDER=$LADDER TIP_GUARD=$TIP_GUARD python - "$DEMO" "$SIM_VARIANT" "$ARM" "$N_EXP" <<'PY'
import json, os, sys, glob, hashlib, numpy as np
d, sv, arm, n_exp = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
m = json.load(open(os.path.join(d, 'repeat.json')))
assert m['sim_variant'] == sv and m.get('scope') == 'full' and m.get('with_state') is True and int(m['action_repeat']) == 4, m
assert m.get('reward_from_tape') is True and abs(float(m['delta_cap']) - 0.025) < 1e-9, m
assert m.get('phase') in (None, 'null'), m
fs = sorted(glob.glob(os.path.join(d, '*.npz')))
assert len(fs) == int(m['n_written']) == n_exp, (len(fs), m.get('n_written'), n_exp)
assert not m.get('one_per_ic_best'), m
if arm == 'dDPfirst':                  # PHASE_PLAN (v): the de-selected arm must BE the first-attempt set
    assert m.get('one_per_ic_first') is True, m
# --- pixel rules (fail-closed): a RENDERED image column, built under THIS run's ladder and tip guard ---
r = m.get('relabel') or {}
assert m.get('images') == 'rendered' and m.get('state_only') is False, (
    f"{d}: the pixel path needs a RENDERED-image set (images={m.get('images')!r}, state_only={m.get('state_only')!r}); "
    f"build one with relabel_reward.py --images")
assert str(r.get('ladder')) == os.environ['LADDER'], f"{d}: set built under ladder {r.get('ladder')!r}, run trains under {os.environ['LADDER']!r}"
assert str(r.get('tip_guard')) == os.environ['TIP_GUARD'], f"{d}: set built under tip_guard {r.get('tip_guard')!r}, run uses {os.environ['TIP_GUARD']!r}"
z = np.load(fs[0]); im = z['image']; T = z['state'].shape[0]
assert im.shape == (T, 64, 64, 6) and im.dtype == np.uint8, (fs[0], im.shape, im.dtype, T)
nz = float((im.reshape(len(im), -1) != 0).any(1).mean())
assert nz > 0.999, f'{d}: {fs[0]} has blank frames (non-blank frame fraction {nz:.4f})'
h = hashlib.sha256()
for f in fs:
    h.update(os.path.basename(f).encode()); h.update(open(f, 'rb').read())
print(f'DEMO-SHA {arm} full-scope PIXEL segments n={len(fs)} sha={h.hexdigest()[:16]} total_reward={m["total_reward"]} '
      f'pick={m["n_pick"]} nopick={m["n_nopick"]} decisions_p50={m["decisions_median"]} ladder={r.get("ladder")} '
      f'tip_guard={r.get("tip_guard")} images={m.get("images")} first_tape_frames={len(im)} nonblank={nz:.4f} mean={float(im.mean()):.2f}',
      file=sys.stderr)
print(h.hexdigest()[:16])
PY
) || exit 1

TRAIN_ARGS=(--steps "$STEPS" --scope full --ladder "$LADDER" ${FAR_FLAG:+$FAR_FLAG} --tip-guard "$TIP_GUARD" --demo-format segment --demo-dir "$DEMO"
  --obs pixels --image-aug "$IMAGE_AUG" --buffer-size "$BUFFER_SIZE"
  --action-mode delta_joint --delta-ref target --action-repeat "$ACTION_REPEAT"
  --train-max-steps "$TRAIN_HORIZON" --eval-max-steps "$EVAL_HORIZON" --eval-freq 0
  --gamma "$GAMMA" --backup-entropy off --per-member-ln off --pick-hold-reward off --pick-shaping off
  --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128
  --demo-shaping off --pick-shaping-terminal-zero on --demo-terminal-guard on --sim-variant "$SIM_VARIANT"
  --ckpt-every 0 --ckpt-fracs "${CKPT_FRACS:-0.4,1.0}"
  --out-dir "$OUT" --run-name "$RUN_NAME" --project genesis_paper --seed "$SEED" --device "$DEVICE")
REG_KNOBS=(steps="$STEPS" budget_unit=decisions scope=full obs=pixels image_aug="$IMAGE_AUG" buffer_size="$BUFFER_SIZE"
           action_mode=delta_joint delta_ref=target action_repeat="$ACTION_REPEAT"
           train_horizon="$TRAIN_HORIZON" eval_horizon="$EVAL_HORIZON" gamma="$GAMMA" backup_entropy=off per_member_ln=off utd=10
           ensemble_size=10 subset_size=2 demo_batch=128 ladder="$LADDER" demo_format=segment demo_sha="$DEMO_SHA" wave="$WAVE"
           sim_variant="$SIM_VARIANT" entry_bank=none phase_sparse=off tip_guard="$TIP_GUARD" amendment=px)
EVAL_ENV=(GENESIS_PICKAPLACE_ROOT="$GENESIS_PICKAPLACE_ROOT" KIND=sac CKPT="$OUT/rlpd_final.zip" OUT="$OUT" ARM="$ARM" SEED="$SEED"
          SIM_VARIANT="$SIM_VARIANT" SETS="$EVAL_SETS" MODES="$EVAL_MODES" ISO="$EVAL_ISO" ISO_SETS="$EVAL_ISO_SETS"
          VIDEO_SETS="$EVAL_VIDEO_SETS" LIMIT="$EVAL_LIMIT" PAR="$EVAL_PAR" REQUIRE_CORES="$REQUIRE_CORES" THREADS="$THREADS")
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS(decisions = $((STEPS * ACTION_REPEAT)) sim steps) BUFFER=$BUFFER_SIZE DEMO=$DEMO sha=$DEMO_SHA OUT=$OUT NODE=$NODE_CLASS"
  echo "[dry] train: python baselines/rl/train_rlpd.py ${TRAIN_ARGS[*]}"
  echo "[dry] eval : env ${EVAL_ENV[*]} sbatch -p batch --exclude=<non-64-core + SMT nodes> cluster/sbatch_rlpd_px_eval.sh"
  exit 0
fi
echo "== RLPD-E2E-PX $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) gpu=${CUDA_VISIBLE_DEVICES:-?} demo=$DEMO sha=$DEMO_SHA restart=${SLURM_RESTART_COUNT:-0}"
echo "== TREE $GENESIS_PICKAPLACE_ROOT ($(git -C "$GENESIS_PICKAPLACE_ROOT" describe --always --dirty 2>/dev/null || echo 'no git'))"
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_rlpd_px.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_rlpd_px.sh --arm "$ARM" --seed "$SEED" --demo-dir "$DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
else
  echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial run dir and starting clean"; rm -rf "$OUT"
fi
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | sed 's/^/GPU /' || true
# ---- LADDER STAMP into the Slurm log, BEFORE training (LADDER_UNIFY_BRIEF D6) ----
LADDER=$LADDER TIP_GUARD=$TIP_GUARD python - <<'PYL' || { echo "FATAL: could not read the ladder from $GENESIS_PICKAPLACE_ROOT"; exit 1; }
import os, sys
R = os.environ['GENESIS_PICKAPLACE_ROOT']
sys.path.insert(0, R + '/baselines'); sys.path.insert(0, R + '/baselines/rl')
sys.path.insert(0, R + '/can_pos_recovery')
import full_env
full_env.refuse_legacy_gates()
L, TG = os.environ['LADDER'], os.environ['TIP_GUARD']
print('[ladder]', full_env.ladder_stamp(L, None, False, TG))
print('[ladder] max_return', full_env.max_return(L))
print('[ladder] tip_guard', TG, 'sustain', full_env.TIP_GUARD_SUSTAIN[TG], 'env frames')
PYL
# GNU time is not installed on the compute nodes (job 3684599 died with exit 127 on it): use it only where it
# exists; the wall clock below and the trainer's own "[rlpd] learn: ... decisions/s" line are the record either way.
TIMER=(); [ -x /usr/bin/time ] && TIMER=(/usr/bin/time -v)
T0=$SECONDS
"${TIMER[@]}" python baselines/rl/train_rlpd.py "${TRAIN_ARGS[@]}"
echo "TRAIN-WALL $((SECONDS - T0)) s (steps=$STEPS decisions, incl. world build + demo load) $(date)"
FINAL_CK=$OUT/rlpd_final.zip
[ -f "$FINAL_CK" ] && [ -f "${FINAL_CK%.zip}.action_mode.json" ] || { echo "FATAL: no final checkpoint + sidecar at $FINAL_CK -- no evaluation of a partial run"; exit 1; }
python3 - "$OUT" "$STEPS" "$IMAGE_AUG" "$BUFFER_SIZE" <<'PY' || exit 1
import json, sys, os
out, steps, aug, buf = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])
sc = json.load(open(os.path.join(out, 'rlpd_final.action_mode.json')))
assert sc['scope'] == 'full' and int(sc['steps']) == steps and abs(float(sc['delta_cap']) - 0.025) < 1e-9, sc
assert sc.get('phase_sparse') is False and sc.get('entry_bank') is None, sc
assert sc.get('obs') == 'pixels' and sc.get('image_aug') == aug and int(sc.get('buffer_size', -1)) == buf, (
    sc.get('obs'), sc.get('image_aug'), sc.get('buffer_size'))
w = sc.get('encoder_wiring') or {}
assert w.get('actor_sees_critic_encoder') and w.get('encoder_in_critic_optimizer') and not w.get('encoder_in_actor_optimizer'), w
ck = json.load(open(os.path.join(out, 'ckpt_100', 'rlpd_ckpt.action_mode.json')))
assert not [f for f in os.listdir(out) if f.endswith('_steps.zip')], 'periodic snapshot zips written despite --ckpt-every 0'
assert int(ck['ckpt_step']) >= steps, ck
print(f'TRAIN-OK {out}: budget {steps} decisions reached (ckpt_100 at {ck["ckpt_step"]}), sidecar obs={sc["obs"]} image_aug={sc["image_aug"]} '
      f'buffer_size={sc["buffer_size"]} ladder={sc["ladder"]} tip_guard={sc["tip_guard"]} cap={sc["delta_cap"]} leash={sc["delta_leash"]}')
PY
if [ -n "${NO_EVAL:-}" ]; then echo "NO_EVAL set: no evaluation job submitted"; echo "JOB DONE $(date)"; exit 0; fi
# ---- the pinned CPU evaluation job (64-physical-core class; the pixel evaluator) ----
[ -f "$EXCL64" ] || { echo "FATAL: no $EXCL64 (the non-64-core node list); the checkpoint is trained -- submit cluster/sbatch_rlpd_px_eval.sh by hand with --exclude"; exit 1; }
EXCL_ALL=$(printf '%s,%s' "$(cat "$EXCL64")" "$SMT_EXCL" | tr ',' '\n' | sed '/^$/d' | sort -u | paste -sd,)
EVAL_JOB=$(env "${EVAL_ENV[@]}" sbatch --parsable -J "e2e_rlpd_px_eval_${ARM}_s${SEED}" --exclude="$EXCL_ALL" cluster/sbatch_rlpd_px_eval.sh) || { echo "FATAL: eval job submission failed"; exit 1; }
echo "EVAL-SUBMITTED job=$EVAL_JOB sets='$EVAL_SETS' modes='$EVAL_MODES' iso=$EVAL_ISO limit='${EVAL_LIMIT}' require_cores=$REQUIRE_CORES excluded=$(echo "$EXCL_ALL" | tr ',' '\n' | wc -l) nodes -> $OUT/e2e_eval_px.log"
echo "JOB DONE $(date)"
