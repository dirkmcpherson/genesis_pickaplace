#!/usr/bin/env bash
# The five Ladder-N smokes (PHASE_PLAN amendment (aa)): one per learner per ladder, plus the
# {RLPD} control arm. Submitted BEFORE any `ln_*` training job, because (aa) says so and because
# the pilot found four defects this way.
#
#   bash cluster/submit_ln_smokes.sh            # submit
#   DRYRUN=1 bash cluster/submit_ln_smokes.sh   # print the five commands and exit
#
# What each smoke has to show in its Slurm log, per (aa) and the step-4 checklist:
#   * `[ladder] … ladder=<x> … tip=tilt>60deg&not_in_hand@4f`
#   * the same full_env / genesis_can_env / stage_predicates sha256 within a ladder
#   * `ramp:slide_gain_m=3/0.05m` on nested_ramp (the revision-1 scale and span)
#   * `max_return` 9 (ramp) / 1 (sparse) / 8 (staged control)
#   * {r2dreamer}: `return_clamp` equal to that maximum, on BOTH env and model
#   * the demo gate passing on the `_rnrh` / `_rnsh` / `_rzh` set
#   * an eval cell whose provenance carries ladder + tip_guard, success key `home` on the
#     nested ladders and `slide_success` on staged
#
# {RLPD} smokes run on CPU (the launcher's DEVICE=cpu exists for exactly this and never for a
# number); {r2dreamer} smokes need a GPU and take QOS `interactive`, so they do not eat the
# normal-QOS GPU cap the batch itself needs.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
R2=${R2:-$W/r2dreamer_ladderN}
DEMOS=$W/demos_state_full
TIP_GUARD=${TIP_GUARD:-not_in_hand}
RL_STEPS=${RL_STEPS:-1000}        # decisions
R2_STEPS=${R2_STEPS:-15000}       # ONLINE env steps (R2_LONG_RUN=1)
SMOKE_ROOT=${SMOKE_ROOT:-baselines/rl/checkpoints/ln_smoke}

[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] || { echo "FATAL: $R2 is not an r2dreamer tree"; exit 1; }
for s in dHfull_all_rnrh dHfull_all_rnsh dHfull_all_rzh; do
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json (build it with cluster/ladderN_sets.sbatch)"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($(git -C "$GP" describe --always --dirty)) | r2 $R2 ($(git -C "$R2" rev-parse --short HEAD))"

# {RLPD}: CPU. The launcher bakes -p preempt/--qos=preempt/--gres=gpu:1/--nice=9000/--constraint,
# all of which have to be overridden on the command line for a CPU smoke.
RLQ=(-p batch --qos=normal --gres=none --constraint= --nice=0)
# {r2dreamer}: GPU on the interactive QOS. That QOS caps walltime at 04:00:00 (and 1 GPU / 16 cpu /
# 64G), while cluster/wmfix_full.sbatch bakes `-t 2-00:00:00` -- so without an explicit -t every
# submission is refused with `QOSMaxWallDurationPerJobLimit`. 3 h is what the pilot's r2 smoke used
# and is ~6x its 32 min runtime.
R2Q=(-p gpu --qos=interactive -t 0-03:00:00)
# ONLY=rl|r2 submits just that learner's smokes (used when one half has already been submitted).
ONLY=${ONLY:-all}

smoke_rl() {   # ladder set seed
  local ladder=$1 set=$2 seed=$3 name=ln_smoke_rl_$4
  local cmd=(env GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD ARM=dH SEED=$seed
             STEPS=$RL_STEPS DEMO=$DEMOS/$set DEVICE=cpu CKPT_FRACS=1.0 OUT_ROOT=$SMOKE_ROOT
             SETS=hold15 MODES=mode VIDEO_SETS= ISO=0 LIMIT=3 PAR=3
             sbatch -J "$name" "${RLQ[@]}" cluster/sbatch_rlpd_e2e.sh)
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  ( cd "$GP" && "${cmd[@]}" ) | sed "s/$/  # $name/"
}

smoke_r2() {   # ladder set seed tag
  local ladder=$1 set=$2 seed=$3 name=ln_smoke_r2_$4
  local cmd=(env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD
             R2_LONG_RUN=1 R2_MILESTONES="[$R2_STEPS]" TAG=lnsmoke EVAL_SETS=hold
             sbatch -J "$name" "${R2Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$set" "$seed" "$R2_STEPS")
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  "${cmd[@]}" | sed "s/$/  # $name/"
}

if [ "$ONLY" = all ] || [ "$ONLY" = rl ]; then
  smoke_rl staged        dHfull_all_rzh  9980 ctl
  smoke_rl nested_ramp   dHfull_all_rnrh 9981 ramp
  smoke_rl nested_sparse dHfull_all_rnsh 9982 sparse
fi
if [ "$ONLY" = all ] || [ "$ONLY" = r2 ]; then
  smoke_r2 nested_ramp   dHfull_all_rnrh 9981 ramp
  smoke_r2 nested_sparse dHfull_all_rnsh 9982 sparse
fi
