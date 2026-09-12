#!/usr/bin/env bash
# {r2dreamer} END-TO-END launcher, PORTED INTO THE REPO 2026-09-10 (LADDER_UNIFY_BRIEF D1).
# Fetched from $W/wmfix_full.sbatch (the in-flight copy, which is NOT modified) and changed
# in exactly three ways, all of them the brief's:
#   1. GENESIS_PICKAPLACE_ROOT is REQUIRED -- no `${GP_ROOT:-$W/gp_root}` default. Defaults on
#      both launchers are how {RLPD} and {r2dreamer} ended up on different ladders: one
#      defaulted to $PWD (= gp_e2e, a tree with no gate) and this one defaulted to gp_root
#      (which had one), while the same FULLENV_REWARD_X=1 was passed to all 64 jobs.
#   2. legacy gates are REFUSED, and the `[ladder]` stamp (ladder + terminals + sha256 of
#      full_env/genesis_can_env/stage_predicates + git describe) is printed into the Slurm log
#      BEFORE training. The old version echoed the gate VALUE, which says what was exported,
#      not what the job loaded.
#   3. the requeue guard stays FIXED (defect 4): the existence check yields on a restart, so
#      the handler that clears a partial logdir is reachable. Preemption killed 5 seeds
#      because it was not.
# Everything else -- partition, recipe, demo gate, eval loop -- is unchanged.
# usage: R2_TREE=<r2dreamer tree> GENESIS_PICKAPLACE_ROOT=<gp tree> LADDER=staged|sparse|nested_sparse|nested_sparse10|nested_ramp [FAR_RELEASE=1] \
#          sbatch cluster/wmfix_full.sbatch <ARM> <seed> <steps> [hydra overrides...]
#   R2_TREE is REQUIRED for the same reason as GENESIS_PICKAPLACE_ROOT (see below).
#   R2_LONG_RUN=1 + R2_MILESTONES='[...]' make <steps> mean ONLINE env steps and write
#   immutable milestone checkpoints (they must be sorted, unique and end at <steps>).
#SBATCH -J wmfix_full
#SBATCH -p gpu,preempt
#SBATCH --gres=gpu:1
#SBATCH --requeue          # preempt partition: requeue instead of dropping (2026-09-04); a requeued run restarts CLEAN (below)
#SBATCH --exclude=pax077   # Tesla P100 (CUDA cap 6.0): torch.compile/Triton refuses it (2026-09-04)
#SBATCH --constraint=l40s|a100|l40|h200
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 2-00:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
# per-job torch.compile cache: two jobs compiling on one node with a shared inductor cache crash (InductorError SubprocException, 09-03)
export TORCHINDUCTOR_CACHE_DIR=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/inductor_cache/${SLURM_JOB_ID:-local}
ARM=$1; SEED=$2; STEPS=$3; shift 3
# LADDER IS REQUIRED (user 2026-09-11), and the RETURN CLAMP FOLLOWS IT. r2dreamer's clamp is
# the task's maximum return, so a sparse run needs 1.0 where a staged run needs 8.0; leaving
# the config's 8.0 in place for a sparse arm would disable the clamp that made this port
# stable in the first place. Both env.return_clamp and model.return_clamp are set from the
# ladder below and printed, so the pair can never silently disagree with the reward.
LADDER=${LADDER:?set LADDER (staged | sparse | nested_sparse | nested_sparse10 | nested_ramp) -- the reward ladder is never defaulted}
case "$LADDER" in staged|sparse|nested_sparse|nested_sparse10|nested_ramp) ;; *) echo "FATAL: LADDER must be staged | sparse | nested_sparse | nested_sparse10 | nested_ramp (got $LADDER)"; exit 2 ;; esac
# Ladder N only (see sbatch_rlpd_e2e.sh): passed to the adapter as env.far_release.
FAR_RELEASE=${FAR_RELEASE:-}
case "${FAR_RELEASE}" in
  ""|0) FAR_CFG="env.far_release=false" ;;
  1) case "$LADDER" in nested_sparse|nested_sparse10|nested_ramp) FAR_CFG="env.far_release=true" ;;
       *) echo "FATAL: FAR_RELEASE=1 is a Ladder-N switch; ladder $LADDER has no farside rung"; exit 2 ;; esac ;;
  *) echo "FATAL: FAR_RELEASE must be 0 or 1 (got $FAR_RELEASE)"; exit 2 ;;
esac
# TIP_GUARD IS REQUIRED (PHASE_PLAN amendment (aa)). It decides where every episode ENDS, so
# it is never a default, for the same reason LADDER is not. grip = the rule of record
# (commanded grip < 0.3, first frame); not_in_hand = the tracker's in_hand (no gripper term)
# sustained 4 env frames. Threshold (60 deg), termination and penalty (0.0) are unchanged.
# Passed to the adapter as env.tip_guard, which passes it to FullTaskEnv's constructor.
TIP_GUARD=${TIP_GUARD:?set TIP_GUARD (grip | not_in_hand) -- the tip guard is never defaulted; PHASE_PLAN (aa)}
case "$TIP_GUARD" in grip|not_in_hand) ;; *) echo "FATAL: TIP_GUARD must be grip | not_in_hand (got $TIP_GUARD)"; exit 2 ;; esac
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
# THE WORLD-MODEL TREE IS REQUIRED (LADDER_UNIFY_BRIEF D1, 2026-09-11). It used to be the
# hardcoded $W/r2dreamer_fix, which (a) is an IN-FLIGHT tree that must not be touched and
# (b) is the same "the launcher, not the submission, decides which code runs" defect as the
# GENESIS_PICKAPLACE_ROOT default. Name the tree or do not run.
R2_TREE=${R2_TREE:?set R2_TREE to the r2dreamer tree this run must use (no default: a launcher default is how the two learners ended up on different ladders)}
TREE=$R2_TREE
[ -f "$TREE/train.py" ] && [ -f "$TREE/envs/genesis.py" ] || { echo "FATAL: $TREE is not an r2dreamer tree"; exit 2; }
PY=$LAB/r2d_venv/bin/python
DEMO=${DEMO_ROOT:-$W/demos_state_full}/$ARM   # DEMO_ROOT override: the repeat-1 pilot reads demos_state_r1/ (stride-1 tapes)
NAME=full_r2d_state_${ARM}${TAG:+_$TAG}_s${SEED}
LOGDIR=$W/runs/$NAME
# A REQUEUED job (preemption) legitimately finds its own partial logdir, and the handler
# below is written to clear it -- but this guard used to exit 2 first, so that handler was
# unreachable and every preempted run died instantly with elapsed 00:00:00.
# 2026-09-10: cost 5 seeds (dH s905/s906/s913, dM s929/s932), all preempted on pax111.
[ -e "$LOGDIR" ] && [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ] && { echo "FATAL: $LOGDIR exists"; exit 2; }
# REQUIRED, no default (D1). GP_ROOT is still accepted as the spelling the old submissions
# used, but one of the two must be set explicitly.
GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-${GP_ROOT:-}}
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT (or GP_ROOT) to the code tree this run must use (no default: a launcher default is how the two learners ended up on different ladders)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/full_env.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 2; }
export GENESIS_PICKAPLACE_ROOT MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
# This tree has ONE ladder and no env-var switches (D1). Refuse rather than run: a gate that
# is exported but inert is indistinguishable from one that works.
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 2; fi
done
cd $TREE
echo "# $(date -Is) host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) tree=$TREE variant=$R2D_SIM_VARIANT demo=$DEMO"
grep -q 'state_obs' envs/genesis.py demo_prefill.py configs/env/genesis_pick_state.yaml || { echo "FATAL: tree lacks the state patches"; exit 2; }
# demo-dir provenance gate: world + stride + with_state + terminal reward must match this run
"$PY" - "$DEMO" "$R2D_SIM_VARIANT" <<'PYG'
import json, sys, glob, numpy as np
d, want = sys.argv[1], sys.argv[2]
m = json.load(open(f"{d}/repeat.json"))
assert m["sim_variant"] == want, (m["sim_variant"], want)
import os as _o; assert int(m["action_repeat"]) == int(_o.environ.get("REPEAT", "4")) and m.get("with_state") is True and abs(float(m["terminal_reward"]) - 1.0) < 1e-9 and m.get("reward_from_tape") is True and m.get("scope") == "full", m
fs = sorted(glob.glob(f"{d}/*.npz")); assert len(fs) == int(m["n_written"]), (len(fs), m.get("n_written"))  # set-size agnostic (raw pair is N=66)
z = np.load(fs[0]); assert z["state"].shape == (len(z["reward"]), 17), z["state"].shape
print(f"[demo-gate] {d}: {len(fs)} tapes, variant {m['sim_variant']}, stride {m['action_repeat']} (expected {_o.environ.get('REPEAT', '4')}), with_state, terminal 1.0, total_reward {m['total_reward']}")
PYG
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$LOGDIR" ]; then echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial logdir and starting clean"; rm -rf "$LOGDIR"; fi
mkdir -p "$LOGDIR"
# RESUME_FROM=<latest.pt> (optional): warm restart from that checkpoint (weights+optims; buffer restarts at prefill,
# step counter re-runs env.steps) -- a DISCLOSED continuation, never a run of record without saying so.
RESUME=""
if [ -n "${RESUME_FROM:-}" ]; then cp "$RESUME_FROM" "$LOGDIR/latest.pt"; RESUME="+resume=true"; echo "# RESUME_FROM=$RESUME_FROM (saved step $($PY -c 'import torch,sys; print(torch.load(sys.argv[1],map_location="cpu",weights_only=False).get("step"))' $LOGDIR/latest.pt))"; fi
# RECIPE OF RECORD baked in (2026-09-05, after the repeat-1 chain was once submitted without them): bnormclamp1ent5 minus the clamp, which is scope-specific (config return_clamp 8 = max staged return)
RECIPE=${RECIPE:-"env.actor_dist=bounded_normal env.act_entropy=3e-5"}
# ---- LADDER STAMP into the Slurm log, BEFORE training (LADDER_UNIFY_BRIEF D6) ----
# The 09-09 version echoed the gate VALUES, which record what was EXPORTED, not what the job
# LOADED -- and the export was inert in the other learner's tree for 32 runs. This prints the
# ladder actually compiled into $GENESIS_PICKAPLACE_ROOT plus the sha256 of the three files
# that define it, and FAILS the job if it cannot. The trainer also writes it to the logdir.
CLAMP=$(LADDER=$LADDER TIP_GUARD=$TIP_GUARD "$PY" - <<'PYL'
import os, sys
R = os.environ['GENESIS_PICKAPLACE_ROOT']
for p in (R + '/baselines', R + '/baselines/rl', R + '/can_pos_recovery'):
    sys.path.insert(0, p)
import full_env
full_env.refuse_legacy_gates()
L, TG = os.environ['LADDER'], os.environ['TIP_GUARD']
# The stamp carries the TIP GUARD (amendment (aa)), passed POSITIONALLY on purpose: a tree
# too old to accept it raises here instead of stamping the rule of record for a job that
# asked for the new guard -- the FULLENV_REWARD_X failure repeated.
print('[ladder]', full_env.ladder_stamp(L, None, False, TG), file=sys.stderr)
print('[ladder] tip_guard', TG, 'sustain', full_env.TIP_GUARD_SUSTAIN[TG], 'env frames',
      file=sys.stderr)
print(repr(full_env.max_return(L)))          # stdout: the return clamp this ladder requires
PYL
) || { echo "FATAL: could not read the ladder from $GENESIS_PICKAPLACE_ROOT"; exit 2; }
echo "[ladder] ladder=$LADDER tip_guard=$TIP_GUARD return_clamp=$CLAMP (env.return_clamp AND model.return_clamp)"
CMD=("$PY" train.py env=${ENVCFG:-genesis_full_state} seed=$SEED env.steps=$STEPS env.demo_dir=$DEMO
     env.ladder=$LADDER $FAR_CFG env.tip_guard=$TIP_GUARD env.return_clamp=$CLAMP model.return_clamp=$CLAMP
     buffer.max_size=5e5 logdir=$LOGDIR $RESUME $RECIPE "$@")
printf '%q ' "${CMD[@]}"; echo
printf '%s %q ' "$(date -Is)" "${CMD[@]}" >> $W/runs/COMMANDS.log; echo >> $W/runs/COMMANDS.log
"${CMD[@]}"
LAST=$(tail -c 4000 $LOGDIR/metrics.jsonl 2>/dev/null | grep -oE "\"step\": [0-9]+" | tail -1 | grep -oE "[0-9]+" || echo 0)
[ "${LAST:-0}" -ge $((STEPS - 5000)) ] || { echo "FATAL: last logged step $LAST < $((STEPS - 5000)) -- training did not reach its budget; no evaluation"; exit 1; }
echo "# train rc=$? $(date -Is)"
# Find this job own log by JOB ID, never by an assumed job NAME: the log is named from
# #SBATCH -J, so any submission using -J (e.g. -J e2eL_r2_dH_s0) renames it and the old
# hardcoded "wmfix_full_<id>.out" no longer exists -> grep -c returns empty -> the [ -ge ]
# test dies with "integer expression expected" AFTER a full 12h training run.
# (2026-09-10: killed e2eL_r2_dH_s0 at 4M steps, train rc=0, artifacts intact.)
SLURM_OUT=$(ls -1 $W/slurm/*_${SLURM_JOB_ID}.out 2>/dev/null | head -1)
[ -n "$SLURM_OUT" ] || SLURM_OUT=$W/slurm/wmfix_full_${SLURM_JOB_ID}.out
n=$(grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' "$SLURM_OUT" 2>/dev/null || true); n=${n:-0}; echo "# [sim-variant] lines: $n (log: $SLURM_OUT)"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] line"; exit 3; }
grep -E 'Demo prefill:|Step accounting' $SLURM_OUT | head -3
# FRESH-process evals (sample mode, protocol horizon): 15 hold + 30 rnd (CONFOUNDS row 22: all episodes present)
for MODE in sample mode; do for SET in ${EVAL_SETS:-hold rnd}; do   # full task: rnd300 + the 74-IC set post hoc (sweep scripts)   # EVAL_SETS override (repeat-1 pilot: "hold rnd", the 74-IC set at 1200 decisions/episode is too slow in-job)   # sample = registered statistic; mode = DP/RLPD-like deterministic counterpart (disclosed); all = FULL demo IC set (user, 2026-09-03)
  N=15; [ $SET = rnd ] && N=30; [ $SET = all ] && N=1   # N ignored for 'all' (evaluator uses every success uid once)
  if [ $SET = all ]; then
    EV=("$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes 1 --mode $MODE --max-steps 1200 \
        --ic-set all --seed 0 --out $LOGDIR/fresh_eval_alldemo_${MODE} --device cpu)
  else
    EV=("$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes $N --mode $MODE --max-steps 1200 \
        --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $SET --seed 0 --out $LOGDIR/fresh_eval_${SET}${N}_${MODE} --device cpu)
  fi
  printf '%q ' "${EV[@]}"; echo
  "${EV[@]}"
  echo "# eval $SET $MODE rc=$? $(date -Is)"
done; done
echo "# DONE $NAME $(date -Is)"
