#!/bin/bash
# DP (Diffusion Policy, lerobot) round-robin launcher: one seed per GPU, env-var driven.
#
# 08-23 REWRITE for the FINAL round robin (paper/PREREG_final_round_robin_2026-08-23.md):
#   * ONE CLOCK: ACTION_REPEAT=4 -> the lerobot dataset MUST be decision-rate (fps = 30/4 =
#     7.5; gated against meta/info.json) and the policy is EXECUTED hold-4 through the
#     learners' delta_joint integrator at eval (wandb_eval --kind dp --action-repeat 4).
#   * ONE HARNESS: cluster/eval_sweep.sh (fresh process per episode, CPU, shared
#     baselines/eval_ics.json sel/hold/rnd, 1200 sim steps). K=5 checkpoints (lerobot
#     --save_freq=STEPS/5) each scored on `sel`; best CONFIRMED on hold+rnd; final also on
#     hold+rnd. Headline per seed = selected-on-hold+rnd. The 08-19 single-process 30-episode
#     eval with the silent 1200 default is GONE.
#   * DEMO_FORMAT=native: RAW=$DEMO_ROOT/<ARM> (contract-v1 tapes + manifest), DATASET=
#     $DEMO_ROOT/<ARM>/lerobot (prebuilt by baselines/make_matched_sets.py at fps 7.5).
#     DEMO_FORMAT=legacy keeps the 08-19 arms/datasets (stride 1; ACTION_REPEAT must be 1).
#   * REG_KNOBS: action_repeat, eval_horizon, budget_unit=grad_steps, demo_sha, demo_format,
#     dataset fps; registry check+register at job START; node recorded.
#
# (08-19 header follows, kept for the legacy arms.) Mirrors cluster/sbatch_rlpd.sh's structure (case-ARM demo resolution,
# stale-env case-guard, provenance gate, RUN_REGISTRY, DRYRUN) and reuses
# cluster/sbatch_ouro_train.sh's lerobot invocation + preemption-safe resume
# verbatim (same conda env, same lerobot-train flags, same resume contract).
#
# WHY: paper/ALGORITHM_STATE_2026-08-18.md — DP is DONE for dHpruned/dDP
# (n=8 each: dHpruned 0.62 in-dist [0.40-0.80], dDP 0.80 [0.67-0.93],
# P(model>human)=0.994). Thursday adds: dR2D_DP (not run at all — "trivial to
# add Thursday") and dH_DP unpruned at n=8 (currently n=1: 0.27/0.13, so the
# preprocessing-control row can stand alone). This script trains BOTH with the
# same recipe used for the n=8 verdict rows.
#
# --- Submit (from the genesis_pickaplace checkout root, AFTER git pull) -----------
#   for S in $(seq 0 7); do ARM=dHpruned SEED=$S sbatch cluster/sbatch_dp.sh; done
#   for S in $(seq 0 7); do ARM=dH       SEED=$S sbatch cluster/sbatch_dp.sh; done
#   for S in $(seq 0 7); do ARM=dDP      SEED=$S sbatch cluster/sbatch_dp.sh; done
#   for S in $(seq 0 7); do ARM=dR2D     SEED=$S sbatch cluster/sbatch_dp.sh; done
#
# Env vars:
#   ARM    dHpruned | dH | dDP | dR2D   (required; picks demo dir + run name;
#          provenance-checked -- see the demo-dir-naming-trap memory)
#   SEED   0..N        (required)
#   STEPS  100000       (the eval-of-record value: paper/PAPER_PLAN.md §3,
#                        "Diffusion Policy, joint obs + joint actions, 100k
#                        steps, batch 64" -- unchanged here, NOT re-tuned)
#   EVAL_EPS  15         (per IC set; eval-of-record uses --ic-mode both, i.e.
#                        15 demo-IC + 15 random-IC, matching launch_paper_week.sh)
#   PROJ   genesis_paper
#   DUPLICATE_OK  <reason>  (unset by default; bypasses a RUN_REGISTRY refusal
#                  on an exact (script,arm,seed,git,knobs,demo) repeat)
#   CONDA_ENV  /cluster/tufts/shortlab/jstale02/condaenv/genesis
#   DRYRUN=1   print the resolved plan and exit 0 before any conda/module/train call
#
# ARMS (raw npz dir used for the provenance/fingerprint gate -> lerobot dataset
# root actually passed to lerobot-train; "chunking" = the dataset's chunks_size,
# which is 1000 for every one of these, inherited unchanged from how each set
# was already converted -- nothing here re-chunks or re-tunes it):
#   dHpruned  baselines/episodes_pick_phase_dppruned  -> baselines/lerobot_dH_pick/genesis_pickaplace
#             human, DP-pruned (idle teleop collapsed, 29.6% frames dropped,
#             make_dp_pruned.py) -- THE functional human-DP arm (PAPER_PLAN 08-01
#             notation: dH_DP -> dHpruned_DP). Prebuilt; n=8 verdict: 0.62 in-dist.
#   dH        baselines/episodes_pick                 -> baselines/lerobot_x2x2v2_jobs_jact/genesis_pickaplace
#             human, UNPRUNED (raw teleop, same 66-of-91 success frames every
#             RL/WM row consumes before pruning). Prebuilt; n=1 control: 0.27/0.13.
#             THIS is the "cheap Thursday add if the row is to stand alone" (n=8).
#   dDP       baselines/m1all_harvest (success stems 1*.npz only, 63 of 93)
#             -> baselines/m1all_harvest_succ_lerobot/genesis_pickaplace
#             gen-0 DP-harvested model demos. Built INLINE if missing (identical
#             recipe to launch_paper_week.sh's CONV step: cp RAW/1*.npz to a
#             tmpdir, convert_to_lerobot.py <tmp> <dst> 8 4 none). n=8 verdict:
#             0.80 in-dist. Already run -- Thursday reruns are for completeness/
#             top-up only, not "not run".
#   dR2D      baselines/episodes_champion_pick (66, all success by construction)
#             -> baselines/lerobot_dR2D_pick/genesis_pickaplace
#             r2dreamer/RLPD "clean" champion demos -- the arm with NO DP row at
#             all before this script. Lerobot set BUILT LOCALLY 2026-08-18 via
#             convert_to_lerobot.py baselines/episodes_champion_pick
#             baselines/lerobot_dR2D_pick/genesis_pickaplace 8 4 none (PROPRIO=8,
#             MIN_FRAMES=4, no cameras -- identical args to the dDP inline build;
#             episodes_champion_pick has no images). Result: 66 episodes, 9184
#             frames -- exact match to the frame count paper/AUDIT_normalization
#             _2026-08-17.md's independent audit reports for this same raw set,
#             confirming the conversion is faithful. matched-N=66 (= human N).
# All four datasets share the SAME layout: observation.state (8: 6 joint +
# gripper pos + grip effort), observation.environment_state (9: can pose + goal
# xy), action (7: 6 joint + grip) -- confirmed by inspecting each meta/info.json
# feature block. No --cartesian/--obs flags needed anywhere in this script.
#
# DATA (rsync ONLY, never git -- run these from the dev box once before
# submitting; datasets are gitignored, see baselines/lerobot_*/ in .gitignore):
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_pick_phase_dppruned/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_pick_phase_dppruned/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/lerobot_dH_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/lerobot_dH_pick/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_pick/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/lerobot_x2x2v2_jobs_jact/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/lerobot_x2x2v2_jobs_jact/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/m1all_harvest/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/m1all_harvest/
#     # (dDP's lerobot dataset builds INLINE on the cluster from this -- no
#     # m1all_harvest_succ_lerobot/ rsync needed unless you want to skip that step)
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_champion_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_champion_pick/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/lerobot_dR2D_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/lerobot_dR2D_pick/
#
# Stages: git-ancestor gate -> RUN_REGISTRY check -> lerobot-train (100k, batch
# 64, diffusion policy, ~2.5-3h on L40S per sbatch_ouro_train.sh's measured
# rate) -> RUN_REGISTRY register -> in-job eval (baselines/wandb_eval.py --kind
# dp --ic-mode both, the SAME protocol dH_DP/dDP_DP's n=8 verdict rows used) ->
# one DP-RESULT line -> sidecar next to the checkpoint.
#
# --- WEDNESDAY SMOKE PLAN (run in this order; each step's proof is stated) --------
# Time budget note: a local single-IC-set 15-episode DP eval measured ~36 min
# on a dev box (baselines/eval_pruned_wandb.log, 2026-07-20); --ic-mode both
# doubles that to ~70+ min at EVAL_EPS=15. For a <30-min smoke, cut EVAL_EPS
# down (2-4) -- this is a conservative ESTIMATE from a non-cluster box, not a
# verified cluster number; adjust EVAL_EPS down further if it overruns.
#
# 1. DRYRUN, one per ARM:
#      for A in dHpruned dH dDP dR2D; do GENESIS_PICKAPLACE_ROOT=$PWD ARM=$A \
#        SEED=0 DRYRUN=1 bash cluster/sbatch_dp.sh; done
#    PROOF: each prints "[dry] ARM=<A> ... RAW=<dir> DATASET=<dir> OUT=<dir>
#    RUN_NAME=<name>", the PROVENANCE-OK line for RAW *and* (if DATASET already
#    exists) for DATASET, the planned RUN_REGISTRY check/register command lines
#    and the planned lerobot-train + wandb_eval command lines, then exits 0
#    with NO module/conda/pip/git-gate output above it.
# 2. One short REAL seed through the full path, cheapest arm (dR2D: smallest
#    dataset, 9184 frames, and the one arm with zero prior DP runs):
#      ARM=dR2D SEED=0 STEPS=3000 EVAL_EPS=2 sbatch cluster/sbatch_dp.sh
#    PROOF: the .out shows, in order: (git-ancestor gate line, if the placeholder
#    below has been filled in) "REGISTRY-OK ...", the lerobot-train progress log
#    (a "Training: NN%|...|3000/3000" tqdm line, matching the pattern seen in
#    baselines/train_pruned.log), a checkpoint at
#    baselines/outputs/dp_thu/dR2D_DP_s0/checkpoints/last/pretrained_model,
#    "REGISTRY-REGISTERED ...", the wandb_eval.py per-episode "None epN:
#    picked=... placed=... contact=... nested=..." lines (4 of them, 2 indist +
#    2 random), exactly one "DP-RESULT arm=dR2D seed=0 indist=X/N random=Y/N"
#    line, a sidecar JSON written next to the checkpoint, and "JOB DONE" with
#    exit code 0.
# 3. Re-submit the IDENTICAL seed (ARM=dR2D SEED=0 STEPS=3000 EVAL_EPS=2, same
#    git, no DUPLICATE_OK): sbatch cluster/sbatch_dp.sh.
#    PROOF: the .out shows "REGISTRY-REFUSE key=... full-key match ..." and the
#    job exits before touching conda/training (no lerobot-train progress line
#    appears, Slurm job state = FAILED, exit code 2). Re-running with
#    DUPLICATE_OK="smoke test" must instead print "REGISTRY-DUPLICATE-OK ..."
#    and proceed through the full path a second time.
# 4. Local wrong-ARM/dataset gate check (no cluster needed):
#      GENESIS_PICKAPLACE_ROOT=$PWD ARM=bogus SEED=0 DRYRUN=1 bash cluster/sbatch_dp.sh
#      # -> FATAL: ARM=bogus ...
#      GENESIS_PICKAPLACE_ROOT=$PWD ARM=dR2D DATASET=baselines/lerobot_dH_pick/genesis_pickaplace \
#        SEED=0 DRYRUN=1 bash cluster/sbatch_dp.sh
#      # -> FATAL: env DATASET=... is exported and does not match ARM=dR2D's dataset (stale-env guard)
#
#SBATCH -J dp-round-robin
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
# lerobot training touches no genesis, and its eval builds the env on the CPU
# backend -- A100 is fine (matches sbatch_ouro_train.sh's reasoning; only dv3/
# r2dreamer's GPU-backend genesis calls are pinned to l40s).
#SBATCH --constraint="l40s|a100"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-14:00:00
#SBATCH --output=dp_%j.out
#SBATCH --error=dp_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT="$PWD"
export PYTHONUNBUFFERED=1

# --- git-ancestor gate: refuse checkouts older than the commit that ADDS this
# script (mirrors sbatch_rlpd.sh's 2fbed2a gate, but self-referential -- there is
# no pre-existing "fix commit" this script depends on, only itself).
# COMMITTER: after committing this file, run
#   git log -1 --format=%H -- cluster/sbatch_dp.sh
# and paste that hash below in place of the placeholder, then commit that
# one-line edit. Until filled in, the gate WARNS instead of refusing (so the
# Wednesday smoke plan is not blocked by the chicken-and-egg problem of the
# script not yet knowing its own commit hash at the moment it is first written).
SBATCH_DP_ADD_COMMIT=${SBATCH_DP_ADD_COMMIT:-8d3de587318d0d37def80bc16178c9b51af77ebf}

ARM=${ARM:?set ARM (native: dH dDP dR2D dHunpruned; legacy: dHpruned dH dDP dR2D)}
SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}
EVAL_EPS=${EVAL_EPS:-15}          # legacy knob, unused by the one-harness path (IC file sizes rule)
PROJ=${PROJ:-genesis_paper}
ACTION_REPEAT=${ACTION_REPEAT:-4}
EVAL_HORIZON=${EVAL_HORIZON:-1200}
DEMO_FORMAT=${DEMO_FORMAT:-native}
DEMO_ROOT=${DEMO_ROOT:-baselines/matched_v1}
WAVE=${WAVE:-final}
IC_FILE=${IC_FILE:-baselines/eval_ics.json}
BUDGET_UNIT=grad_steps
case "$DEMO_FORMAT" in native|legacy) ;; *) echo "FATAL: DEMO_FORMAT must be native|legacy"; exit 1 ;; esac
if [ "$DEMO_FORMAT" = legacy ] && [ "$ACTION_REPEAT" != 1 ]; then
  echo "FATAL: legacy (stride-1, fps 30) datasets can only train/eval at ACTION_REPEAT=1 (got $ACTION_REPEAT)"; exit 1
fi
# lerobot checkpoints at 20/40/60/80/100% of the budget (K=5 archived, PREREG §5)
SAVE_FREQ=$(( STEPS / 5 )); [ "$SAVE_FREQ" -ge 1 ] || SAVE_FREQ=1

# capture any PRE-EXISTING exports of our own var names BEFORE the case block
# below unconditionally reassigns them -- otherwise the stale-env guard would
# just be comparing the resolved value to itself (dead code; the case-driven
# name always "self-heals"). A stale export under our own names is exactly as
# real a hazard as one under DEMO_DIR (see sbatch_ouro_train.sh, which uses
# DATASET as a REQUIRED external input -- a leftover export from an ouro_train
# shell is the realistic contamination case this guards against).
_PRE_RAW="${RAW:-}"
_PRE_DATASET="${DATASET:-}"

if [ "$DEMO_FORMAT" = native ]; then
  case "$ARM" in
    dH|dDP|dR2D|dHunpruned)
      RAW=$DEMO_ROOT/$ARM
      RAW_PAT='^[0-9]{6}\.npz$'          # contract v1: rollout indices for EVERY source
      DATASET=$DEMO_ROOT/$ARM/lerobot    # built by make_matched_sets.py at fps 30/ACTION_REPEAT
      SUCCESS_GLOB="$RAW"/*.npz          # native sets are success-only by construction
      ;;
    *) echo "FATAL: ARM=$ARM is not a native arm (dH dDP dR2D dHunpruned)"; exit 1 ;;
  esac
else
case "$ARM" in
  dHpruned)
    RAW=baselines/episodes_pick_phase_dppruned
    RAW_PAT='^[0-9]{3}\.npz$'
    DATASET=baselines/lerobot_dH_pick/genesis_pickaplace
    SUCCESS_GLOB="$RAW"/*.npz
    ;;
  dH)
    RAW=baselines/episodes_pick
    RAW_PAT='^[0-9]{3}\.npz$'
    DATASET=baselines/lerobot_x2x2v2_jobs_jact/genesis_pickaplace
    SUCCESS_GLOB="$RAW"/*.npz
    ;;
  dDP)
    RAW=baselines/m1all_harvest
    RAW_PAT='^[0-9]{6}\.npz$'
    DATASET=baselines/m1all_harvest_succ_lerobot/genesis_pickaplace
    SUCCESS_GLOB="$RAW"/1*.npz   # BC = success stems ONLY (1xxxxx; fails are 5xxxxx)
    ;;
  dR2D)
    RAW=baselines/episodes_champion_pick
    RAW_PAT='^[0-9]{6}\.npz$'
    DATASET=baselines/lerobot_dR2D_pick/genesis_pickaplace
    SUCCESS_GLOB="$RAW"/*.npz
    ;;
  *) echo "FATAL: ARM=$ARM (legacy arms: dHpruned, dH, dDP, dR2D)"; exit 1 ;;
esac
fi

# stale-env case-guard (mirrors sbatch_r2d_ms.sh's DEMO_DIR guard / sbatch_rlpd.sh's
# DEMO/DATASET guard, 08-18): a stale exported RAW/DATASET/DEMO_DIR from another
# submission or shell must not silently override this ARM's own dirs. Uses the
# PRE-case values captured above so a same-named stale export is actually caught
# (the case block below would otherwise have already clobbered it).
if [ -n "${DEMO_DIR:-}" ] && [ "$DEMO_DIR" != "$RAW" ]; then
  echo "FATAL: env DEMO_DIR=$DEMO_DIR is exported and does not match ARM=$ARM's raw-demo-dir ($RAW)."
  echo "       A stale 'export DEMO_DIR=...' from another submission is the usual cause: unset DEMO_DIR"
  exit 1
fi
if [ -n "$_PRE_RAW" ] && [ "$_PRE_RAW" != "$RAW" ]; then
  echo "FATAL: env RAW=$_PRE_RAW is exported and does not match ARM=$ARM's raw-demo-dir ($RAW)."
  echo "       A stale 'export RAW=...' from another submission is the usual cause: unset RAW"
  exit 1
fi
if [ -n "$_PRE_DATASET" ] && [ "$_PRE_DATASET" != "$DATASET" ]; then
  echo "FATAL: env DATASET=$_PRE_DATASET is exported and does not match ARM=$ARM's dataset ($DATASET)."
  echo "       A stale 'export DATASET=...' from another submission is the usual cause: unset DATASET"
  exit 1
fi

# provenance gate #1 (naming-trap rule): RAW filename pattern must match the
# claimed source (human 3-digit uid vs model 6-digit rollout index).
if [ -n "${DRYRUN:-}" ] && [ ! -d "$RAW" ]; then
  echo "[dry] NOTE: raw demo dir $RAW is absent on this box -- the dir/pattern/manifest/sha/fps gates would FATAL at a real run"
  N_SUCCESS=0; DEMO_SHA=dry-missing
else
[ -d "$RAW" ] || { echo "FATAL: raw demo dir missing: $RAW (rsync it -- gitignored, see header)"; exit 1; }
ls "$RAW" | grep -E '\.npz$' | head -5 | grep -qE "$RAW_PAT" || {
  echo "FATAL: $RAW contents do not match $ARM provenance pattern $RAW_PAT"; exit 1; }
N_SUCCESS=$(ls $SUCCESS_GLOB 2>/dev/null | wc -l)
[ "$N_SUCCESS" -gt 0 ] || { echo "FATAL: no success-stem npz in $RAW matching ${SUCCESS_GLOB}"; exit 1; }
# native: contract-v1 manifest gate; demo sha = manifest content sha (else sorted filenames)
DEMO_SHA=$(python3 - "$RAW" "$DEMO_FORMAT" "$ARM" <<'PY'
import json, os, sys, hashlib
d, fmt, arm = sys.argv[1:4]
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = os.path.join(d, 'manifest.json'); j = json.load(open(m)) if os.path.exists(m) else {}
if fmt == 'native':
    if not j:
        print(f'FATAL: {d}/manifest.json missing -- native arms are built by baselines/make_matched_sets.py', file=sys.stderr); sys.exit(1)
    c = str(j.get('contract') or j.get('tape_contract') or '')
    if c != 'v1':
        print(f'FATAL: {m} contract={c!r} != v1', file=sys.stderr); sys.exit(1)
    if j.get('n_kept') is not None and int(j['n_kept']) != len(files):
        print(f'FATAL: {m} n_kept={j["n_kept"]} != {len(files)} npz on disk', file=sys.stderr); sys.exit(1)
sha = j.get('content_sha256') or j.get('sha256'); src = 'manifest' if sha else 'filenames'
if not sha:
    sha = hashlib.sha256('\n'.join(files).encode()).hexdigest()
print(f'DEMO-SHA {arm} {fmt} n={len(files)} sha={sha[:16]} ({src})', file=sys.stderr)
print(sha[:16])
PY
) || exit 1
fi

# provenance gate #2: if DATASET already exists, its meta/info.json episode
# count must match the RAW success-stem count exactly (catches a stale/
# mismatched lerobot conversion sitting under the expected path -- the same
# class of bug the normalization audit found, but for the dataset root instead
# of the raw npz).
if [ -d "$DATASET" ]; then
  python3 - "$DATASET" "$N_SUCCESS" "$ARM" "$ACTION_REPEAT" <<'PYEOF'
import json, sys, pathlib as pl
ds, expect_n, arm, rep = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])
info = json.loads((pl.Path(ds) / 'meta' / 'info.json').read_text())
n = info['total_episodes']
if n != expect_n:
    print(f'FATAL: {ds} meta/info.json total_episodes={n} != {expect_n} raw '
          f'success-stem npz for ARM={arm} -- refusing (dataset provenance gate)')
    sys.exit(1)
# TIME-BASE GATE (08-23): the dataset's fps must be the decision rate 30/ACTION_REPEAT --
# a stride-1 (fps 30) dataset trained and then executed hold-4 is a different MDP.
fps = float(info.get('fps', 0)); want = 30.0 / rep
if abs(fps - want) > 1e-6:
    print(f'FATAL: {ds} fps={fps} but ACTION_REPEAT={rep} needs fps={want} (decision-rate dataset); '
          f'rebuild with make_matched_sets.py / convert at 30/{rep} fps, or set ACTION_REPEAT to match')
    sys.exit(1)
print(f'PROVENANCE-OK dataset={ds} total_episodes={n} fps={fps} (action_repeat {rep}) matches ARM={arm}')
PYEOF
else
  if [ "$DEMO_FORMAT" = native ]; then
    [ -z "${DRYRUN:-}" ] && { echo "FATAL: native dataset $DATASET missing -- build it with baselines/make_matched_sets.py (lerobot at fps 30/$ACTION_REPEAT); no inline build for native sets"; exit 1; }
    echo "[dry] NOTE: native dataset $DATASET absent -- a real run FATALs (make_matched_sets.py builds it at fps 30/$ACTION_REPEAT; no inline build)"
  else
    echo "PROVENANCE-DEFERRED dataset=$DATASET does not exist yet -- will be built "
    echo "  inline from $RAW ($N_SUCCESS success npz) before training; checked post-build."
  fi
fi

OUT=baselines/outputs/dp_${WAVE}/${ARM}_DP_s${SEED}
RUN_NAME="${ARM}_DP-${WAVE}_s${SEED}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
REG_KNOBS=(steps="$STEPS" budget_unit="$BUDGET_UNIT" batch_size=64 policy=diffusion dataset_root="$DATASET"
           action_repeat="$ACTION_REPEAT" eval_horizon="$EVAL_HORIZON" demo_format="$DEMO_FORMAT"
           demo_sha="$DEMO_SHA" save_freq="$SAVE_FREQ" wave="$WAVE")
SWEEP_COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED"
              --wandb-run "$RUN_NAME" --wandb-project "$PROJ")

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS($BUDGET_UNIT) ACTION_REPEAT=$ACTION_REPEAT EVAL_HORIZON=$EVAL_HORIZON DEMO_FORMAT=$DEMO_FORMAT WAVE=$WAVE PROJ=$PROJ SAVE_FREQ=$SAVE_FREQ"
  echo "[dry] RAW=$RAW (N_SUCCESS=$N_SUCCESS) DEMO_SHA=$DEMO_SHA DATASET=$DATASET (fps must be $(python3 -c "print(30/$ACTION_REPEAT)")) OUT=$OUT RUN_NAME=$RUN_NAME NODE=$NODE_CLASS"
  echo "[dry] git-ancestor gate: SBATCH_DP_ADD_COMMIT=$SBATCH_DP_ADD_COMMIT"
  echo "[dry] registry check:    python cluster/run_registry.py check    --script sbatch_dp.sh --arm $ARM --seed $SEED --demo-dir $RAW --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
  echo "[dry] registry register: python cluster/run_registry.py register --script sbatch_dp.sh --arm $ARM --seed $SEED --demo-dir $RAW --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]} --stage start"
  if [ ! -d "$DATASET" ] && [ "$DEMO_FORMAT" = legacy ]; then
    echo "[dry] build dataset (legacy inline build, if missing): T=\$(mktemp -d); cp $SUCCESS_GLOB \$T/; python baselines/convert_to_lerobot.py \$T $DATASET 8 4 none; rm -rf \$T"
  fi
  echo "[dry] train: lerobot-train --dataset.repo_id=local/${RUN_NAME} --dataset.root=$DATASET --policy.type=diffusion --policy.push_to_hub=false --seed=$SEED --output_dir=$OUT --batch_size=64 --steps=$STEPS --save_freq=$SAVE_FREQ --job_name=$RUN_NAME --wandb.enable=true --wandb.project=$PROJ --wandb.disable_artifact=true"
  echo "[dry] sidecar: write dp_sidecar.json {action_repeat=$ACTION_REPEAT, ...} into every $OUT/checkpoints/<step>/"
  echo "[dry] selection: for C in $OUT/checkpoints/<step>/pretrained_model (5): bash cluster/eval_sweep.sh dp <C> $OUT/sweep/<step> --sets sel --ckpt-step <step> --tag ckpt_<step> ${SWEEP_COMMON[*]}"
  echo "[dry] confirmation: bash cluster/eval_sweep.sh dp <SELECTED> $OUT/sweep/selected --sets hold,rnd --tag selected ${SWEEP_COMMON[*]}; same for <FINAL> --tag final"
  echo "[dry] headline: DP-HEADLINE arm=$ARM seed=$SEED repeat=$ACTION_REPEAT selected=<step> sel=a/15 hold=b/15 rnd=c/30 final_hold=d/15 final_rnd=e/30"
  exit 0
fi
echo "== DP $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) raw=$RAW sha=$DEMO_SHA dataset=$DATASET"
# ---- RUN_REGISTRY: refuse an exact repeat, warn on a git-only-diff repeat --------
# Runs BEFORE conda/module so a refusal costs no GPU time (stdlib-only helper,
# system python3). Skipped on a preemption requeue -- same logical run continuing.
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then
  python3 cluster/run_registry.py check --script sbatch_dp.sh --arm "$ARM" --seed "$SEED" \
    --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_dp.sh --arm "$ARM" --seed "$SEED" \
    --demo-dir "$RAW" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
fi

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl

# cuDNN loader guard (mirrors sbatch_ouro_train.sh: the shared nvidia/ tree must win)
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

# --- preflight: fail immediately with the remedy, not 50 lines in (mirrors
# sbatch_ouro_train.sh verbatim) -----------------------------------------------
if command -v lerobot-train >/dev/null 2>&1; then
  LEROBOT_TRAIN="lerobot-train"
elif python -c 'import lerobot.scripts.lerobot_train' 2>/dev/null; then
  LEROBOT_TRAIN="python -m lerobot.scripts.lerobot_train"   # console script absent
else
  echo "FATAL: lerobot is not importable in this env ($CONDA_PREFIX)."
  echo "  Run once:  conda activate <env> && bash cluster/install_lerobot.sh"
  exit 1
fi

if [ "$SBATCH_DP_ADD_COMMIT" = "__FILL_IN_AFTER_FIRST_COMMIT__" ]; then
  echo "WARN: SBATCH_DP_ADD_COMMIT placeholder not filled in yet -- ancestor gate skipped."
else
  git merge-base --is-ancestor "$SBATCH_DP_ADD_COMMIT" HEAD || {
    echo "FATAL: checkout predates the commit that added sbatch_dp.sh ($SBATCH_DP_ADD_COMMIT). git pull first."
    exit 1; }
fi

# --- build the dataset inline if it doesn't exist yet (dDP's normal path; the
# other three arms ship prebuilt via rsync, so this is a no-op for them) --------
if [ ! -d "$DATASET" ]; then
  [ "$DEMO_FORMAT" = legacy ] || { echo "FATAL: native dataset $DATASET missing (make_matched_sets.py builds it)"; exit 1; }
  echo "== $DATASET missing -- building inline from $RAW ($N_SUCCESS success npz)"
  T=$(mktemp -d)
  cp $SUCCESS_GLOB "$T"/
  python baselines/convert_to_lerobot.py "$T" "$DATASET" 8 4 none
  rm -rf "$T"
fi
# re-run provenance gate #2 (count + fps) now that the dataset is guaranteed to exist
python3 - "$DATASET" "$N_SUCCESS" "$ARM" "$ACTION_REPEAT" <<'PYEOF'
import json, sys, pathlib as pl
ds, expect_n, arm, rep = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])
info = json.loads((pl.Path(ds) / 'meta' / 'info.json').read_text())
n = info['total_episodes']
if n != expect_n:
    print(f'FATAL: {ds} meta/info.json total_episodes={n} != {expect_n} raw '
          f'success-stem npz for ARM={arm} -- refusing (dataset provenance gate)')
    sys.exit(1)
fps = float(info.get('fps', 0)); want = 30.0 / rep
if abs(fps - want) > 1e-6:
    print(f'FATAL: {ds} fps={fps} but ACTION_REPEAT={rep} needs fps={want}'); sys.exit(1)
print(f'PROVENANCE-OK dataset={ds} total_episodes={n} fps={fps} matches ARM={arm}')
PYEOF


# --- train (reused verbatim from sbatch_ouro_train.sh: same conda env, same
# lerobot-train flags, same preemption-safe resume contract) --------------------
TC="$OUT/checkpoints/last/pretrained_model/train_config.json"
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$TC" ]; then
  echo "== requeued (restart #${SLURM_RESTART_COUNT}); RESUMING via $TC"
  $LEROBOT_TRAIN --config_path="$TC" --resume=true
else
  rm -rf "$OUT"
  $LEROBOT_TRAIN \
    --dataset.repo_id="local/${RUN_NAME}" \
    --dataset.root="$DATASET" \
    --policy.type=diffusion --policy.push_to_hub=false \
    --seed="$SEED" --output_dir="$OUT" --batch_size=64 --steps="$STEPS" --save_freq="$SAVE_FREQ" \
    --job_name="$RUN_NAME" \
    --wandb.enable=true --wandb.project="$PROJ" --wandb.disable_artifact=true
fi
CKPT=$OUT/checkpoints/last/pretrained_model
[ -d "$CKPT" ] || { echo "FATAL: no checkpoint at $CKPT"; exit 1; }

# --- sidecar (action_repeat travels with EVERY checkpoint -- wandb_eval/eval_sweep read it
# and REFUSE a mismatched clock) -----------------------------------------------------------
GIT_HASH=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
mapfile -t CK_DIRS < <(ls -d "$OUT"/checkpoints/[0-9]*/ 2>/dev/null | sort -V)
if [ "${#CK_DIRS[@]}" -eq 0 ]; then
  echo "WARN: no numbered checkpoints under $OUT/checkpoints (save_freq not honoured?) -- falling back to last/ only"
  CK_DIRS=("$OUT/checkpoints/last/")
fi
for D in "${CK_DIRS[@]}"; do
  python3 - "$D" "$ARM" "$SEED" "$RAW" "$DATASET" "$GIT_HASH" "$STEPS" "$PROJ" "$ACTION_REPEAT" "$DEMO_SHA" "$DEMO_FORMAT" "$NODE_CLASS" <<'PYEOF'
import json, sys, pathlib as pl, datetime
d, arm, seed, raw, dataset, git, steps, proj, rep, sha, fmt, node = sys.argv[1:13]
sidecar = pl.Path(d) / 'dp_sidecar.json'
sidecar.write_text(json.dumps({
    'script': 'sbatch_dp.sh', 'arm': arm, 'seed': int(seed),
    'raw_demo_dir': raw, 'dataset_root': dataset, 'git': git,
    'action_repeat': int(rep), 'demo_sha': sha, 'demo_format': fmt, 'node': node,
    'ckpt_step': pl.Path(d).name,
    'config': {'policy': 'diffusion', 'batch_size': 64, 'steps': int(steps), 'project': proj},
    'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
}, indent=1))
print(f'sidecar -> {sidecar}')
PYEOF
done

# --- one-harness eval: K checkpoints scored on sel, best confirmed on hold+rnd, final too ---
SW=$OUT/sweep; mkdir -p "$SW"
set +e   # evals must never fail an already-trained job
BEST_TAG=""; BEST_CK=""; BEST_N=-1
for D in "${CK_DIRS[@]}"; do
  STEP=$(basename "$D"); TAG="ckpt_$STEP"; C="$D/pretrained_model"
  [ -d "$C" ] || { echo "WARN: $C missing, skipping"; continue; }
  bash cluster/eval_sweep.sh dp "$C" "$SW/$STEP" --sets sel --ckpt-step "$STEP" --tag "$TAG" "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/$STEP.log"
  N=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); s=d["sets"]["sel"]; print(s["picked"] if s["n_present"]==s["n_expected"] else -1)' "$SW/$STEP/sweep.json" 2>/dev/null || echo -1)
  echo "SELECT-RESULT arm=$ARM seed=$SEED ckpt=$STEP sel=$N/15"
  if [ "$N" -ge "$BEST_N" ]; then BEST_N=$N; BEST_TAG=$STEP; BEST_CK=$C; fi   # ties -> later checkpoint
done
FINAL_D=${CK_DIRS[-1]}; FINAL_TAG=$(basename "$FINAL_D"); FINAL_CK="$FINAL_D/pretrained_model"
echo "== selected $BEST_TAG (sel=$BEST_N/15); final=$FINAL_TAG"
bash cluster/eval_sweep.sh dp "$BEST_CK" "$SW/selected" --sets hold,rnd --ckpt-step "$BEST_TAG" --tag selected "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/selected.log"
if [ "$BEST_CK" = "$FINAL_CK" ]; then FINAL_SWEEP="$SW/selected/sweep.json"
else
  bash cluster/eval_sweep.sh dp "$FINAL_CK" "$SW/final" --sets hold,rnd --ckpt-step "$FINAL_TAG" --tag final "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/final.log"
  FINAL_SWEEP="$SW/final/sweep.json"
fi
python3 - "$SW/selected/sweep.json" "$FINAL_SWEEP" "$ARM" "$SEED" "$ACTION_REPEAT" "$BEST_TAG" "$BEST_N" "$FINAL_TAG" "$RUN_NAME" "$PROJ" "$NODE_CLASS" <<'PY'
import json, sys
sel_j, fin_j, arm, seed, rep, best, best_n, final_tag, run_name, proj, node = sys.argv[1:12]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception:
        return 'MISSING'
line = (f'DP-HEADLINE arm={arm} seed={seed} repeat={rep} selected={best} sel={best_n}/15 hold={rd(sel_j,"hold")} '
        f'rnd={rd(sel_j,"rnd")} final={final_tag} final_hold={rd(fin_j,"hold")} final_rnd={rd(fin_j,"rnd")} node={node}')
print(line)
try:
    import wandb
    api = wandb.Api(timeout=60)
    runs = list(api.runs(f'jambotime/{proj}', filters={'display_name': run_name}))
    if runs:
        run = sorted(runs, key=lambda x: x.created_at)[-1]
        run.summary['sweep/headline'] = line; run.summary['sweep/selected_ckpt'] = best; run.summary['sweep/node'] = node
        run.summary.update(); print(f'wandb: pushed sweep/headline to {run.id}')
    else:
        print(f'WARN: wandb headline push skipped -- no run named {run_name!r}')
except Exception as e:
    print(f'WARN: wandb headline push failed ({type(e).__name__}: {e}) -- DP-HEADLINE line is authoritative')
PY
echo "JOB DONE $(date)"
