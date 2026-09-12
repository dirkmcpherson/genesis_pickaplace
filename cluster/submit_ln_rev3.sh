#!/usr/bin/env bash
# The 12 jobs of PHASE_PLAN amendment (aa) REVISION 3 -- equal-n ignition read + the world-model
# guard control. Registered BEFORE submission (commit 4cf58af); this script IS the submit command
# of record, so what was registered and what ran are the same text rather than two retypings.
#
#   bash cluster/submit_ln_rev3.sh            # submit
#   DRYRUN=1 bash cluster/submit_ln_rev3.sh   # print the 12 commands and exit
#
# The (aa) revision-3 table, verbatim:
#   {r2dreamer} nested_sparse  +2v+2  s957-958 / s977-978   4M online       milestones 0.5M/1M/2M/4M
#   {RLPD}      nested_sparse  +2v+2  s957-958 / s977-978   500k decisions  ckpt 100k/250k/500k
#   {r2dreamer} staged + not_in_hand guard (control) 2v2  s962-963 / s982-983  1M online  miles 0.5M/1M
# Every job: TIP_GUARD=not_in_hand, FAR_RELEASE off, the demo set whose suffix matches its ladder,
# trees $LAB/gp_ladderN @ a40c8aa1 (PINNED, never pulled) and $W/r2dreamer_ladderN @ 0cf3d9e.
#
# QOS SPLIT, disclosed, and the same rule as the batch (between LEARNERS, never between arms):
# the 8 {r2dreamer} runs take QOS `normal` (they are the long pole and a preempted world-model run
# restarts CLEAN from step 0, so preemption costs the whole elapsed run), the 4 {RLPD} runs take the
# preempt allocation. At submission BOTH allocations were at their cap, so every job below queues;
# that is a scheduling delay, not a refusal.
#
# DEVIATION, disclosed (see paper/LADDER_N_PILOT_LOG_2026-09-11.md "## Revision 3"): the {RLPD}
# launcher names its run dir $OUT_ROOT/e2e_rlpd_${ARM}_s${SEED} with NO ladder component, and the
# batch's staged-control arm already occupies seeds s958 (dH) and s978 (dDPfirst). Submitting
# revision 3's registered seeds into the default root would have had two new jobs write into the run
# dirs of two RUNNING jobs (3581566 `ln_rl_ctl_dH_s958`, 3581568 `ln_rl_ctl_dM_s978`) -- the launcher
# has no exists-guard on a fresh start, and `run_registry.py check` refuses only a FULL-key match,
# which these are not (different ladder, set and budget). All four {RLPD} rev-3 runs therefore use
# OUT_ROOT=baselines/rl/checkpoints/e2e_rev3. Seeds, arms, sets, ladder, budget, checkpoints and job
# names are exactly as registered; only the storage root moves, symmetrically for both arms.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
R2=${R2:-$W/r2dreamer_ladderN}
DEMOS=$W/demos_state_full
TIP_GUARD=${TIP_GUARD:-not_in_hand}
RL_OUT_ROOT=${RL_OUT_ROOT:-baselines/rl/checkpoints/e2e_rev3}

[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] || { echo "FATAL: $R2 is not an r2dreamer tree"; exit 1; }
for s in dHfull_all_rnsh dDPfull_first_rnsh dHfull_all_rzh dDPfull_first_rzh; do
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json"; exit 1; }
done
# the pin: this script must not be the thing that moves gp_ladderN
DESC=$(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git')
case "$DESC" in *a40c8aa1*) ;; *) echo "FATAL: $GP is at '$DESC', not the pinned a40c8aa1 -- refusing"; exit 1 ;; esac
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to submit"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | tree $GP ($DESC) | r2 $R2 ($(git -C "$R2" rev-parse --short HEAD))"

RLQ=(-p preempt --qos=preempt --nice=0)
R2Q=(-p gpu --qos=normal)

sub_rl() {   # tag arm demo ladder seed steps ckpt_fracs shortladder
  local tag=$1 arm=$2 demo=$3 ladder=$4 seed=$5 steps=$6 fracs=$7 short=$8
  local name=ln_rl_${short}_${tag}_s${seed}
  # refuse to write into a run dir that already exists (the collision this script exists to avoid)
  if [ -e "$GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed}" ]; then
    echo "FATAL: $GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed} exists -- refusing to submit $name over it"; exit 1
  fi
  local cmd=(env GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD ARM=$arm SEED=$seed
             STEPS=$steps DEMO=$DEMOS/$demo CKPT_FRACS=$fracs OUT_ROOT=$RL_OUT_ROOT
             sbatch -J "$name" "${RLQ[@]}" cluster/sbatch_rlpd_e2e.sh)
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  ( cd "$GP" && "${cmd[@]}" ) | sed "s/$/  # $name/"
}

sub_r2() {   # tag demo ladder seed steps milestones shortladder
  local tag=$1 demo=$2 ladder=$3 seed=$4 steps=$5 miles=$6 short=$7
  local name=ln_r2_${short}_${tag}_s${seed}
  if [ -e "$W/runs/full_r2d_state_${demo}_s${seed}" ]; then
    echo "FATAL: $W/runs/full_r2d_state_${demo}_s${seed} exists -- refusing to submit $name over it"; exit 1
  fi
  local cmd=(env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD
             R2_LONG_RUN=1 R2_MILESTONES="$miles"
             sbatch -J "$name" "${R2Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$demo" "$seed" "$steps")
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  "${cmd[@]}" | sed "s/$/  # $name/"
}

# ---- {r2dreamer} nested_sparse: 4M ONLINE steps, milestones 0.5M / 1M / 2M / 4M --------------
for S in 957 958; do sub_r2 dH dHfull_all_rnsh    nested_sparse $S 4000000 '[500000,1000000,2000000,4000000]' sparse; done
for S in 977 978; do sub_r2 dM dDPfull_first_rnsh nested_sparse $S 4000000 '[500000,1000000,2000000,4000000]' sparse; done
# ---- {r2dreamer} staged + new tip guard (CONTROL): 1M ONLINE steps, milestones 0.5M / 1M -----
for S in 962 963; do sub_r2 dH dHfull_all_rzh     staged $S 1000000 '[500000,1000000]' ctl; done
for S in 982 983; do sub_r2 dM dDPfull_first_rzh  staged $S 1000000 '[500000,1000000]' ctl; done
# ---- {RLPD} nested_sparse: 500k decisions, ckpt 100k / 250k / 500k ---------------------------
for S in 957 958; do sub_rl dH dH       dHfull_all_rnsh    nested_sparse $S 500000 0.2,0.5,1.0 sparse500k; done
for S in 977 978; do sub_rl dM dDPfirst dDPfull_first_rnsh nested_sparse $S 500000 0.2,0.5,1.0 sparse500k; done
