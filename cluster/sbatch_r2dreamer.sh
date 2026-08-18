#!/bin/bash
# r2dreamer training on genesis: ONE run per GPU, env-var driven.
#
# --- Submit (from the genesis_pickaplace checkout root) ---------------------------
#   cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
#   CONFIG=genesis_pick_v3       SEED=0 sbatch cluster/sbatch_r2dreamer.sh   # msparity (abs joint)
#   CONFIG=genesis_pick_v4_delta SEED=0 sbatch cluster/sbatch_r2dreamer.sh   # delta-joint
#   # champion recipe, explicit demo-source ARM (hardened path, 08-18 -- see below):
#   CONFIG=genesis_pick_v5d4c_delta ARM=dH   SEED=0 sbatch cluster/sbatch_r2dreamer.sh
#   CONFIG=genesis_pick_v5d4c_delta ARM=dDP  SEED=0 sbatch cluster/sbatch_r2dreamer.sh
#   CONFIG=genesis_pick_v5d4c_delta ARM=dR2D SEED=0 sbatch cluster/sbatch_r2dreamer.sh
#
# Env vars (defaults):
#   CONFIG    genesis_pick_v3 | genesis_pick_v4_delta | genesis_pick_v5d4c_delta (hydra
#             env group name; genesis_pick_v5d4c_delta = the CHAMPION recipe, see
#             r2dreamer/configs/env/genesis_pick_v5d4c_delta.yaml -- untouched here,
#             this script never changes a training hyperparameter)
#   ARM       unset | dH | dDP | dR2D  (OPTIONAL -- explicit demo-SOURCE selection,
#             provenance-gated; see "ARM (demo-source hardening, 08-18)" below. Unset
#             = the ORIGINAL behavior, DEMO_DIR resolved by matching CONFIG's NAME
#             against a glob (kept for back-compat; this is fragile -- M7 below)
#   SEED      0
#   STEPS     3e6                 total SIM steps incl. prefill (v3/v4: repeat 4)
#   DEMO_DIR  auto per CONFIG (legacy glob, ARM unset) or per ARM (explicit dir,
#             ARM set) -- see below. DV3_DIR default: ../dreamerv3-torch.
#   LOGDIR    $R2D_DIR/runs/${CONFIG#genesis_}_s$SEED   (runs/ tree; wandb run name
#             = its basename via sync_runs_to_wandb.py)
#   VENV      /cluster/tufts/shortlab/$USER/r2d_venv if it has a python, else
#             $HOME/r2d_venv (see "\$HOME-default lesson" below)
#   R2D_DIR   /cluster/tufts/shortlab/$USER/r2dreamer if it has the genesis port,
#             else $HOME/r2dreamer (rsynced -- the port is uncommitted)
#   DUPLICATE_OK  <reason>  (unset by default; bypasses a RUN_REGISTRY refuse on an
#             exact (script,ARM-or-CONFIG,seed,git,knobs,demo) repeat)
#   ENV_NUM 6  BUFFER_MAX 450000  REINJECT 300000  PRETRAIN 1000  EXTRA (raw hydra
#   overrides)  EVAL_EPS 15  EVAL_MAX_STEPS 400 (SIM steps; v3/v4 tl)  NO_EVAL
#   DRYRUN=1 bash cluster/sbatch_r2dreamer.sh   # print the resolved plan, no submit
#   -- exits BEFORE any module/conda/venv/python line; safe to run on any box.
#
# ARM (demo-source hardening, 08-18 -- paper/AUDIT_silent_defaults_2026-08-17.md M7):
# "cluster/sbatch_r2dreamer.sh:70-75: DEMO_DIR chosen by glob on the CONFIG name
# (*v5* -> delta25 set, *delta* -> delta set, else non-delta) -- a config whose name
# doesn't match the pattern (or matches the wrong one first; *v5* outranks *delta*)
# trains on mismatched demo encodings with only a dir-exists preflight." ARM replaces
# the glob with an EXPLICIT, audited dir per demo source, at the champion encoding
# (delta_cap 0.025, matching genesis_pick_v5d4c_delta -- see cluster/R2DREAMER_CLUSTER.md
# "THE STATISTICS WAVE" + paper/AUDIT_normalization_2026-08-17.md 2.5):
#   dH    $DV3_DIR/demonstrations/genesis_pick_pruned_delta25   human champion set
#         (67 human teleop pick demos, converted baselines/rl/to_dreamer_demos.py
#         --action-encoding delta_joint --delta-cap 0.025 --grant-slack 48)
#   dDP   $DV3_DIR/demonstrations/genesis_m1all_delta25         DP-harvested model demos
#         (from baselines/m1all_harvest, 93 rollouts incl. no-pick negatives kept as
#         zero-reward dynamics data; convert + gate command in R2DREAMER_CLUSTER.md
#         "dDP demo set: convert + gate ON THE CLUSTER")
#   dR2D  $DV3_DIR/demonstrations/genesis_r2dchamp_delta25       r2dreamer-champion-
#         harvested model demos (baselines/harvest_champion_demos.py --images, teacher
#         = r2dreamer's OWN champion CHAMPION_1576820.pt, 52 kept / 54 IC attempts
#         measured 08-18, all label=success stage=picked; the RLPD-facing sibling of
#         this set, without images, is baselines/episodes_champion_pick / ARM=dR2D in
#         cluster/sbatch_rlpd.sh -- built from a DIFFERENT harvest run, so counts and
#         rollout ids need not match 1:1)
# ARM is OPTIONAL for back-compat: a bare CONFIG=... launch (no ARM) resolves
# DEMO_DIR via the ORIGINAL glob, byte-identical to before this hardening.
# When ARM is set, a provenance gate refuses to launch unless the resolved dir's
# filenames match the claimed source (human 4-digit uid vs model rollout-index
# naming) AND its episode count falls in the expected band for that source --
# catches both "points at the wrong directory" and "a stale/partial regeneration".
#
# STALE-ENV CASE-GUARD (mirrors cluster/sbatch_r2d_ms.sh's DEMO_DIR guard, 08-18):
# if ARM is set AND a DEMO_DIR is ALSO exported (e.g. left over from another
# submission's shell) that disagrees with ARM's own dir, this refuses rather than
# silently using the wrong one -- the same failure class that fed a 4-dim ManiSkill
# run 7-dim genesis actions in the r2d_ms incident (8e5a4e2).
#
# $HOME-DEFAULT LESSON (FABLE_HANDOFF_2026-08-13 cluster lessons: "Cluster paths:
# /cluster/tufts/shortlab/jstale02/ (never ~)." -- VENV/R2D_DIR defaulting under
# $HOME on the cluster resolved to an EMPTY directory and killed a wave of jobs at
# the python-not-found / port-not-found preflight). VENV/R2D_DIR now prefer the lab
# path (/cluster/tufts/shortlab/$USER/{r2d_venv,r2dreamer}) and fall back to $HOME
# ONLY if the lab path is genuinely absent; a SEPARATE guard below refuses outright
# if whatever VENV/R2D_DIR finally resolves to sits under $HOME AND still lacks its
# python / the genesis port -- the exact silent-empty-dir failure, now loud.
#
# RUN_REGISTRY: if cluster/run_registry.py exists (sibling-agent build, see
# paper/AUDIT_run_identity_2026-08-17.md), this script calls its check/register
# subcommands around the training call, mirroring cluster/sbatch_rlpd.sh. If it does
# not exist yet, this line is a clearly-marked TODO hook -- do not build a local
# ad-hoc substitute (see "RUN_REGISTRY hook" below).
#
# wandb (entity jambotime, project r2dreamer_genesis -- hardcoded in the repo's
# sync/eval scripts): train.py itself logs to tensorboard/metrics.jsonl only; a
# background loop runs sync_runs_to_wandb.py --only <run> every 30 min (idempotent,
# resume-safe on live runs) + a final sync, then eval_genesis.py --wandb uploads
# the honest post-train eval (videos + scalars). Needs WANDB_API_KEY in the
# environment or a prior `wandb login` on the cluster.
#
# PREEMPT/REQUEUE: --requeue resubmits on preemption. r2dreamer periodic-
# checkpoints latest.pt (trainer.save_every 1e5, atomic). train.py resume is a
# WARM RESTART ONLY (+resume=true, added 2026-08-10): agent weights + optimizer
# state reload from latest.pt, but the replay buffer is NOT persisted -- the
# buffer restarts at demo prefill and the step counter re-runs the full STEPS
# budget. Learned parameters survive preemption; buffer contents and step
# progress do not. Requeues therefore extend wall time rather than losing the
# model; budget --time accordingly.
#
# NOTE: #SBATCH lines are parsed by sbatch, NOT shell-expanded -- no $VARs in
# them; override at submit time (sbatch --time=... --mem=...) if needed.
#
# --- WEDNESDAY SMOKE PLAN (run in this order; each step's proof is stated) --------
# 1. DRYRUN, one per ARM at the champion config:
#      for A in dH dDP dR2D; do
#        CONFIG=genesis_pick_v5d4c_delta ARM=$A SEED=0 DRYRUN=1 \
#          GENESIS_PICKAPLACE_ROOT="$PWD" bash cluster/sbatch_r2dreamer.sh
#      done
#    PROOF: each prints "[dry] cd <R2D_DIR>", "[dry] <TRAIN_CMD>" with
#    "env.demo_dir=<ARM's explicit dir>" in the command line, "[dry] demo_dir=...
#    (<ARM_NOTE>)", the planned RUN_REGISTRY check/register command lines (or the
#    TODO line if run_registry.py is absent), exits 0, and touches NO
#    module/conda/venv/python process (no world build, no torch import).
# 2. wrong-ARM rejection: ARM=bogus SEED=0 DRYRUN=1 bash cluster/sbatch_r2dreamer.sh
#    PROOF: "FATAL: ARM=bogus (must be dH, dDP, or dR2D)", exit 1, BEFORE any
#    "[dry]" line (the case statement runs ahead of the DRYRUN print block).
# 3. $HOME-path rejection (testable on any box lacking the cluster lab path, e.g.
#    this dev box): unset VENV R2D_DIR; ARM=dR2D SEED=0 STEPS=100 NO_EVAL=1 \
#      GENESIS_PICKAPLACE_ROOT="$PWD" bash cluster/sbatch_r2dreamer.sh   # no DRYRUN
#    PROOF: "FATAL: R2D_DIR=$HOME/r2dreamer resolves under \$HOME and lacks the
#    genesis port" (or the VENV analogue, whichever resolves first), exit 1, before
#    any TRAIN_CMD / module / conda line -- this script has none, but the guard
#    must fire before the preflight python/port checks it duplicates-with-a-message.
# 4. One short REAL dR2D seed through the full path (needs a real GPU node + the
#    rsynced dR2D demo dir -- see "rsync manifest" in R2DREAMER_CLUSTER.md):
#      CONFIG=genesis_pick_v5d4c_delta ARM=dR2D SEED=0 STEPS=2e4 \
#        LOGDIR=$R2D_DIR/runs/smoke_dR2D_s0 sbatch cluster/sbatch_r2dreamer.sh
#    PROOF: the .out shows, in order: "== r2dreamer genesis_pick_v5d4c_delta seed 0
#    start ... demo=<dR2D dir> (N eps) logdir=..." (N must match the dR2D expected-n
#    band checked at launch); run.log's "Demo prefill: {'episodes': N, ...}" line
#    from train.py (the trainer's OWN load count -- must agree with N, catching a
#    silent partial load) plus its "Step accounting: trainer starts at step ..."
#    line; then (STEPS=2e4 is small enough to finish in one allocation) "==
#    confirming best checkpoint" and at least one "... episodes" eval summary line
#    in confirm.log/eval.log before "== r2dreamer ... done".

#SBATCH -J r2d-train
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
# Training worlds are CPU-backend (one Genesis world per spawn worker); the GPU
# runs only the dreamer model -> any verified type is fine (l40s|a100|l40|h200
# all pass cluster/verify_env.sh incl. the world build).
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=2-00:00:00
# 3e6 sim steps ran ~13 h on the dev box (64 sim-steps/s, CPU-bound stepping);
# margin for slower cluster cores + warm-restart budget re-runs after preemption.
#SBATCH --output=r2d_train_%j.out
#SBATCH --error=r2d_train_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT MUJOCO_GL=egl PYTHONUNBUFFERED=1

CONFIG=${CONFIG:-genesis_pick_v3}
SEED=${SEED:-0}
STEPS=${STEPS:-3e6}
ARM=${ARM:-}

# --- ARM: explicit demo-source dir + provenance gate (M7 hardening, 08-18) --------
# Validated up front (even under DRYRUN) so "wrong-ARM rejection" is a fast, always-
# reachable failure -- see WEDNESDAY SMOKE PLAN step 2.
if [ -n "$ARM" ]; then
  case "$ARM" in
    dH|dDP|dR2D) ;;
    *) echo "FATAL: ARM=$ARM (must be dH, dDP, or dR2D)"; exit 1 ;;
  esac
fi

# --- $HOME-default lesson: prefer the lab path, fall back to $HOME ONLY if the lab
# path is genuinely absent (see header). This is the PRIMARY fix; the explicit guard
# below is the belt-and-suspenders net for whatever this resolves to.
_LAB=/cluster/tufts/shortlab/$USER
VENV=${VENV:-$([ -x "$_LAB/r2d_venv/bin/python" ] && echo "$_LAB/r2d_venv" || echo "$HOME/r2d_venv")}
R2D_DIR=${R2D_DIR:-$([ -f "$_LAB/r2dreamer/envs/genesis.py" ] && echo "$_LAB/r2dreamer" || echo "$HOME/r2dreamer")}
DV3_DIR=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
PY="$VENV/bin/python"
# The explicit-path $HOME guard runs LATER, in the preflight block below (AFTER the
# DRYRUN exit) -- DRYRUN must be able to print the resolved plan on any box,
# including a dev box with neither the lab path nor a $HOME install; the guard
# itself is a real-run-only check (SMOKE PLAN step 3 runs it WITHOUT DRYRUN).

# --- DEMO_DIR: legacy CONFIG-glob (ARM unset, unchanged) or explicit ARM dir -------
case "$CONFIG" in
  *v5*)    DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta25 ;;  # cap 0.025 sets
  *delta*) DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta ;;
  *)       DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned ;;
esac
_DEMO_DIR_ENV="${DEMO_DIR:-}"   # captured BEFORE any default -- for the stale-env guard
if [ -n "$ARM" ]; then
  case "$ARM" in
    dH)   ARM_DEMO=$DV3_DIR/demonstrations/genesis_pick_pruned_delta25
          ARM_PAT='genesis-0[0-9][0-9][0-9]-[0-9]+\.npz$'; ARM_N_MIN=60; ARM_N_MAX=72
          ARM_NOTE='human champion pick set (dH), delta_cap 0.025, 4-digit uid filenames' ;;
    dDP)  ARM_DEMO=$DV3_DIR/demonstrations/genesis_m1all_delta25
          ARM_PAT='genesis-1[0-9][0-9][0-9][0-9][0-9]-[0-9]+\.npz$'; ARM_N_MIN=85; ARM_N_MAX=95
          ARM_NOTE='DP-harvested model demos (dDP, m1all_harvest-derived), delta_cap 0.025, rollout-index filenames' ;;
    dR2D) ARM_DEMO=$DV3_DIR/demonstrations/genesis_r2dchamp_delta25
          ARM_PAT='genesis-1[0-9][0-9][0-9][0-9][0-9]-[0-9]+\.npz$'; ARM_N_MIN=35; ARM_N_MAX=60
          ARM_NOTE='r2dreamer-champion-harvested model demos (dR2D), delta_cap 0.025, rollout-index filenames' ;;
  esac
  if [ -n "$_DEMO_DIR_ENV" ] && [ "$_DEMO_DIR_ENV" != "$ARM_DEMO" ]; then
    echo "FATAL: ARM=$ARM implies DEMO_DIR=$ARM_DEMO but env DEMO_DIR=$_DEMO_DIR_ENV is"
    echo "       ALSO exported and disagrees (stale-env case-guard, mirrors sbatch_r2d_ms.sh)."
    echo "       A stale 'export DEMO_DIR=...' from another submission's shell is the usual"
    echo "       cause: unset DEMO_DIR, or drop ARM and set DEMO_DIR alone."; exit 1
  fi
  DEMO_DIR=$ARM_DEMO
else
  DEMO_DIR=${DEMO_DIR:-$DEMO_DEFAULT}
fi
# ARM is part of the logdir/run name when set (08-18 smoke: dR2D s0 landed in the
# same dir wave-3 dH s0 would use -- names must never collide across arms).
LOGDIR=${LOGDIR:-$R2D_DIR/runs/${CONFIG#genesis_}${ARM:+_$ARM}_s$SEED}
RUN_NAME=$(basename "$LOGDIR"); RUNS_DIR=$(dirname "$LOGDIR")

# Warm restart on requeue only (see header): fresh submissions must NOT pick up
# a stale latest.pt from a reused LOGDIR.
RESUME=""
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -f "$LOGDIR/latest.pt" ]; then
  RESUME="+resume=true"
  echo "== requeued (restart #${SLURM_RESTART_COUNT}): WARM RESTART from $LOGDIR/latest.pt"
  echo "   (weights+optims reload; buffer + step counter start over -- by design)"
fi

TRAIN_CMD=("$PY" train.py "env=$CONFIG" "seed=$SEED" "env.steps=$STEPS" \
  "env.env_num=${ENV_NUM:-6}" "env.demo_dir=$DEMO_DIR" \
  ${REINJECT:+"env.demo_reinject_every=$REINJECT"} \
  ${DUPLICATE:+"env.demo_duplicate=$DUPLICATE"} \
  ${SAVE_EVERY:+"trainer.save_every=$SAVE_EVERY"} \
  "buffer.max_size=${BUFFER_MAX:-450000}" \
  "trainer.pretrain=${PRETRAIN:-1000}" "logdir=$LOGDIR" $RESUME ${EXTRA:-})
# REINJECT/DUPLICATE are passed ONLY when set: genesis_pick_v5d4_delta BAKES the
# champion values (150k / 4x); an unconditional sbatch default would silently
# override the config (the silent-defaults bug family).

# --- RUN_REGISTRY hook (paper/AUDIT_run_identity_2026-08-17.md §5) ----------------
# REG_ARM disambiguates a legacy (ARM-unset) launch by CONFIG, so two different
# glob-selected demo dirs under the same blank ARM don't collide in the registry.
REG_ARM=${ARM:-legacy-$CONFIG}
REG_KNOBS=(config="$CONFIG" steps="$STEPS" env_num="${ENV_NUM:-6}" \
           buffer_max="${BUFFER_MAX:-450000}" pretrain="${PRETRAIN:-1000}" \
           reinject="${REINJECT:-baked}" duplicate="${DUPLICATE:-baked}")
REGISTRY_PY="cluster/run_registry.py"

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] cd $R2D_DIR"
  echo "[dry] ${TRAIN_CMD[*]}"
  echo "[dry] demo_dir=$DEMO_DIR logdir=$LOGDIR venv=$VENV wandb-run=$RUN_NAME"
  [ -n "$ARM" ] && echo "[dry] ARM=$ARM ($ARM_NOTE) pattern=$ARM_PAT n-band=[$ARM_N_MIN,$ARM_N_MAX]"
  if [ -f "$REGISTRY_PY" ]; then
    echo "[dry] registry check:    python $REGISTRY_PY check    --script sbatch_r2dreamer.sh --arm $REG_ARM --seed $SEED --demo-dir $DEMO_DIR --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
    echo "[dry] registry register: python $REGISTRY_PY register --script sbatch_r2dreamer.sh --arm $REG_ARM --seed $SEED --demo-dir $DEMO_DIR --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
  else
    # TODO(run_registry): cluster/run_registry.py does not exist yet on this
    # checkout. Once the sibling build lands (paper/AUDIT_run_identity_2026-08-17.md
    # §5), this hook activates automatically -- no further edit needed here, the
    # real preflight block below already gates on [ -f "$REGISTRY_PY" ].
    echo "[dry] registry: cluster/run_registry.py not present -- skipping (TODO hook, see script header)"
  fi
  exit 0
fi

# --- preflight: fail immediately with the remedy ----------------------------------
# Explicit-path guard (FABLE_HANDOFF_2026-08-13 cluster lessons: VENV/R2D_DIR
# defaults pointed at $HOME and killed a wave of jobs; "never ~"). Filesystem
# tests only -- no module/conda/venv line has run yet -- so this is directly
# testable locally without DRYRUN (SMOKE PLAN step 3).
case "$VENV" in
  "$HOME"/*)
    [ -x "$PY" ] || { echo "FATAL: VENV=$VENV resolves under \$HOME and has no python at $PY."
      echo "       \$HOME-default lesson (FABLE_HANDOFF cluster lessons, 'never ~'): use"
      echo "       VENV=/cluster/tufts/shortlab/\$USER/r2d_venv, or install it there:"
      echo "       bash cluster/install_r2dreamer.sh $_LAB/r2d_venv"; exit 1; } ;;
esac
case "$R2D_DIR" in
  "$HOME"/*)
    [ -f "$R2D_DIR/envs/genesis.py" ] || { echo "FATAL: R2D_DIR=$R2D_DIR resolves under \$HOME and lacks the genesis port (envs/genesis.py)."
      echo "       \$HOME-default lesson: use R2D_DIR=/cluster/tufts/shortlab/\$USER/r2dreamer,"
      echo "       or rsync the (uncommitted) port there first -- see R2DREAMER_CLUSTER.md."; exit 1; } ;;
esac
[ -x "$PY" ] || { echo "FATAL: no python at $PY -- run: bash cluster/install_r2dreamer.sh $VENV"; exit 1; }
[ -f "$R2D_DIR/envs/genesis.py" ] || { echo "FATAL: $R2D_DIR lacks the genesis port (UNCOMMITTED -- rsync the dev-box working tree, not a clone)"; exit 1; }
[ -f "$R2D_DIR/configs/env/$CONFIG.yaml" ] || { echo "FATAL: no configs/env/$CONFIG.yaml in $R2D_DIR"; exit 1; }
[ -d "$DEMO_DIR" ] || { echo "FATAL: demo dir missing: $DEMO_DIR (rsync from <devbox>:~/workspace/dreamerv3-torch/demonstrations/ -- gitignored, rsync ONLY)"; exit 1; }
N_NPZ=$(ls "$DEMO_DIR"/*.npz 2>/dev/null | wc -l)
[ "$N_NPZ" -gt 0 ] || { echo "FATAL: $DEMO_DIR has no *.npz"; exit 1; }

# provenance gate (naming-trap rule + M7 fix, ARM-set launches only): filename
# pattern AND episode count must match the claimed ARM. Catches (a) a dir that
# exists but holds the wrong source's files (b) a partially-regenerated / stale
# dir with an unexplained count.
if [ -n "$ARM" ]; then
  ls "$DEMO_DIR" | grep -E '\.npz$' | head -5 | grep -qE "$ARM_PAT" || {
    echo "FATAL: $DEMO_DIR contents do not match ARM=$ARM provenance pattern $ARM_PAT"
    echo "       ($ARM_NOTE)"; exit 1; }
  if [ "$N_NPZ" -lt "$ARM_N_MIN" ] || [ "$N_NPZ" -gt "$ARM_N_MAX" ]; then
    echo "FATAL: $DEMO_DIR has $N_NPZ episodes, expected $ARM_N_MIN-$ARM_N_MAX for ARM=$ARM."
    echo "       Regenerate or point DEMO_DIR elsewhere -- do not proceed on an unexplained count."
    exit 1
  fi
fi

# RUN_REGISTRY: refuse an exact repeat, warn on a git-only-diff repeat. Sibling-
# agent tool (paper/AUDIT_run_identity_2026-08-17.md §5); if it is not on this
# checkout yet, skip with a loud note rather than inventing a local substitute.
if [ -f "$REGISTRY_PY" ] && [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ]; then
  echo "NOTE: preemption requeue (restart ${SLURM_RESTART_COUNT}) -- registry check/register skipped, same logical run"
elif [ -f "$REGISTRY_PY" ]; then
  python3 "$REGISTRY_PY" check --script sbatch_r2dreamer.sh --arm "$REG_ARM" --seed "$SEED" \
    --demo-dir "$DEMO_DIR" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  # register AT LAUNCH: a preempted/crashed 3M run must still leave its line
  python3 "$REGISTRY_PY" register --script sbatch_r2dreamer.sh --arm "$REG_ARM" --seed "$SEED" \
    --demo-dir "$DEMO_DIR" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
else
  echo "NOTE: cluster/run_registry.py not found -- run-identity duplicate check skipped (TODO hook)"
fi

mkdir -p "$LOGDIR"

# Double-submission guard (2026-08-12): dDP_R2D_s4 had TWO jobs writing one
# LOGDIR (1103 interleaved env_step jumps in wandb, checkpoint-clobber risk).
# flock is per-node only, so also refuse if another RUNNING slurm job claimed
# this LOGDIR (claim file carries the job id; stale claims from dead jobs pass).
if [ -f "$LOGDIR/.claim" ]; then
  OTHER=$(cat "$LOGDIR/.claim")
  if [ "$OTHER" != "${SLURM_JOB_ID:-none}" ] && squeue -h -j "$OTHER" -t RUNNING 2>/dev/null | grep -q .; then
    echo "FATAL: job $OTHER is already RUNNING on $LOGDIR (double-submission guard)"; exit 1
  fi
fi
echo "${SLURM_JOB_ID:-none}" > "$LOGDIR/.claim"

echo "== r2dreamer $CONFIG seed $SEED start $(date) demo=$DEMO_DIR ($N_NPZ eps) logdir=$LOGDIR"

# --- background wandb sync (idempotent; safe on the live run) ---------------------
(
  while true; do
    sleep 1800
    ( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
      >> "$LOGDIR/wandb_sync.log" 2>&1 || true
  done
) & SYNC_PID=$!
trap 'kill $SYNC_PID 2>/dev/null || true' EXIT

# --- checkpoint lottery coverage (2026-08-11): training is BISTABLE (diagnosed
# lambda-return explosion) -- final/latest checkpoints are a PHASE LOTTERY.
# Archive every latest.pt write, eval it (15 eps, sample, seed 0), keep only the
# best 2 + the newest (checkpoints are 145MB), record scores to ckpt_scores.tsv.
(
  T0=0
  while true; do
    sleep 120
    T1=$(stat -c %Y "$LOGDIR/latest.pt" 2>/dev/null || echo 0)
    if [ "$T1" != "$T0" ] && [ "$T1" -gt 0 ]; then
      T0=$T1; sleep 15
      CK="$LOGDIR/ckpt_$T1.pt"; cp "$LOGDIR/latest.pt" "$CK"
      # DEVICE CPU: precaution only -- keeps a second CUDA process off the
      # training GPU. (CORRECTION 2026-08-11: the "jobs died at 230k" alarm
      # that prompted this was a monitoring artifact -- sync_runs_to_wandb
      # marks the run 'finished' after EVERY sync cycle, and the step count
      # lagged; the trainers were healthy throughout. wandb run state is NOT
      # a liveness signal for these runs; poll env_step growth instead.)
      P=$( (cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$CK" \
            --episodes 15 --mode sample --seed 0 --device cpu --torch-threads 2 \
            --append-metrics "$LOGDIR" 2>/dev/null) \
           | grep -oP 'picked \K[0-9.]+' | head -1 )
      echo -e "$CK\t${P:-ERR}" >> "$LOGDIR/ckpt_scores.tsv"
      echo "== ckpt eval: $CK picked=${P:-ERR}"
      # prune: keep best 2 by score + the newest file
      NEWEST=$(ls -t "$LOGDIR"/ckpt_*.pt | head -1)
      KEEP=$( { sort -k2 -rn "$LOGDIR/ckpt_scores.tsv" | head -2 | cut -f1; echo "$NEWEST"; } | sort -u )
      for F in "$LOGDIR"/ckpt_*.pt; do echo "$KEEP" | grep -qxF "$F" || rm -f "$F"; done
    fi
  done
) & ARCH_PID=$!
trap 'kill $SYNC_PID $ARCH_PID 2>/dev/null || true' EXIT

# --- train ------------------------------------------------------------------------
( cd "$R2D_DIR" && "${TRAIN_CMD[@]}" ) 2>&1 | tee -a "$LOGDIR/run.log"

kill $SYNC_PID 2>/dev/null || true
( cd "$R2D_DIR" && "$PY" sync_runs_to_wandb.py --runs-dir "$RUNS_DIR" --only "$RUN_NAME" ) \
  >> "$LOGDIR/wandb_sync.log" 2>&1 || true

kill $ARCH_PID 2>/dev/null || true

# --- honest post-train eval + BEST-CHECKPOINT CONFIRMATION -------------------------
# Protocol (pre-registered): the in-run loop SELECTS the best checkpoint on its
# seed-0 eval; the headline number is the mean of x3 CONFIRMATION evals on fresh
# seeds 1-3 (+1 mode eval) -- selection and confirmation on independent draws.
# 08-18 smoke (job 2588648): eval_genesis.py exited 2 AFTER printing its success
# line, and set -eo pipefail killed the script there -- the job read FAILED with
# every result already produced. Post-train diagnostics must never fail the job:
# relax -e/pipefail for this block and report each stage's rc explicitly.
set +e +o pipefail
if [ -z "${NO_EVAL:-}" ]; then
  ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/latest.pt" \
      --episodes "${EVAL_EPS:-15}" --max-steps "${EVAL_MAX_STEPS:-400}" \
      --mode sample --seed 0 --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee "$LOGDIR/eval.log"
  _erc=${PIPESTATUS[0]}; [ "$_erc" -eq 0 ] || echo "WARN: final eval exited rc=$_erc (metrics line may still be present -- check $LOGDIR/metrics.jsonl)"
  BEST=$(sort -k2 -rn "$LOGDIR/ckpt_scores.tsv" 2>/dev/null | head -1 | cut -f1)
  if [ -n "$BEST" ] && [ -f "$BEST" ]; then
    cp "$BEST" "$LOGDIR/BEST_selected.pt"
    echo "== confirming best checkpoint $BEST (selected on seed-0 eval)"
    for ES in 1 2 3; do
      ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/BEST_selected.pt" \
          --episodes 15 --mode sample --seed $ES --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee -a "$LOGDIR/confirm.log"
    done
    ( cd "$R2D_DIR" && "$PY" eval_genesis.py --checkpoint "$LOGDIR/BEST_selected.pt" \
        --episodes 15 --mode mode --seed 0 --device cuda --wandb --append-metrics "$LOGDIR" ) 2>&1 | tee -a "$LOGDIR/confirm.log"
    grep -h "15 episodes" "$LOGDIR/confirm.log" | tail -4
  else
    echo "WARN: no scored checkpoints -- lottery coverage produced nothing (see ckpt_scores.tsv)"
  fi
fi
# ONE greppable result line (mirrors SWEEP-RESULT / DP-RESULT / DV3-RESULT):
# picked from the LAST eval/* line in metrics.jsonl (the latest.pt seed-0 eval;
# BEST-checkpoint confirmations, when present, are the paper number -- see log).
_pk=$(grep -h '"eval/picked"' "$LOGDIR/metrics.jsonl" 2>/dev/null | tail -1 | grep -oE '"eval/picked": *[0-9.]+' | grep -oE '[0-9.]+$')
echo "R2D-RESULT arm=${ARM:-legacy} seed=$SEED config=$CONFIG picked=${_pk:-NA} logdir=$LOGDIR"
echo "== r2dreamer $CONFIG seed $SEED done $(date)"
exit 0
