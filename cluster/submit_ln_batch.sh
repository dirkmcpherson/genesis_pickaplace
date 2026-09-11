#!/usr/bin/env bash
# The 20 jobs of PHASE_PLAN amendment (aa) + REVISION 1 -- Ladder N: the ladders that pay the
# slide. Registered BEFORE submission; this script IS the submit command of record, so what was
# registered and what ran are the same text rather than two retypings of it.
#
#   bash cluster/submit_ln_batch.sh            # submit
#   DRYRUN=1 bash cluster/submit_ln_batch.sh   # print the 20 commands and exit
#
# The (aa) table, verbatim:
#   {RLPD}      nested_ramp    2v2  s950-951 / s970-971   250k decisions  ckpt 40k/100k/250k
#   {r2dreamer} nested_ramp    2v2  same seeds            2M online       milestones 0.5M/1M/2M
#   {RLPD}      nested_sparse  2v2  s955-956 / s975-976   250k decisions  ckpt 40k/100k/250k
#   {r2dreamer} nested_sparse  2v2  same seeds            4M online       milestones 0.5M/1M/2M/4M
#   {RLPD}      staged (control, new tip guard) 2v2  s958-959 / s978-979  100k  ckpt 40k/100k
# Every job: TIP_GUARD=not_in_hand, FAR_RELEASE off, the demo set whose suffix matches its ladder.
#
# QOS SPLIT (a scheduling choice, disclosed): the 8 {r2dreamer} runs take QOS `normal` (cap 10)
# because they are the long pole (2M / 4M online steps) and a preempted world-model run restarts
# CLEAN from step 0 -- the requeue guard clears the partial logdir -- so preemption costs the whole
# elapsed run. The 12 {RLPD} runs take the preempt allocation (cap 20), which is where the free
# capacity is, and restart clean the same way if preempted. The split is BETWEEN LEARNERS and never
# between arms, so it cannot touch the human-versus-machine contrast, which is the only contrast
# this batch makes; it does mean a cross-learner reading carries a scheduling difference.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
R2=${R2:-$W/r2dreamer_ladderN}
DEMOS=$W/demos_state_full
TIP_GUARD=${TIP_GUARD:-not_in_hand}

[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] || { echo "FATAL: $R2 is not an r2dreamer tree"; exit 1; }
for s in dHfull_all_rnrh dDPfull_first_rnrh dHfull_all_rnsh dDPfull_first_rnsh \
         dHfull_all_rzh dDPfull_first_rzh; do
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json (build it with cluster/ladderN_sets.sbatch)"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to submit"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | tree $GP ($(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git')) | r2 $R2 ($(git -C "$R2" rev-parse --short HEAD))"

RLQ=(-p preempt --qos=preempt --nice=0)     # the launcher's own partition, at normal priority
R2Q=(-p gpu --qos=normal)

sub_rl() {   # tag arm demo ladder seed steps ckpt_fracs shortladder
  local tag=$1 arm=$2 demo=$3 ladder=$4 seed=$5 steps=$6 fracs=$7 short=$8
  local name=ln_rl_${short}_${tag}_s${seed}
  local cmd=(env GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD ARM=$arm SEED=$seed
             STEPS=$steps DEMO=$DEMOS/$demo CKPT_FRACS=$fracs
             sbatch -J "$name" "${RLQ[@]}" cluster/sbatch_rlpd_e2e.sh)
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  ( cd "$GP" && "${cmd[@]}" ) | sed "s/$/  # $name/"
}

sub_r2() {   # tag demo ladder seed steps milestones shortladder
  local tag=$1 demo=$2 ladder=$3 seed=$4 steps=$5 miles=$6 short=$7
  local name=ln_r2_${short}_${tag}_s${seed}
  local cmd=(env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD
             R2_LONG_RUN=1 R2_MILESTONES="$miles"
             sbatch -J "$name" "${R2Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$demo" "$seed" "$steps")
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  "${cmd[@]}" | sed "s/$/  # $name/"
}

# ---- {RLPD} nested_ramp: 250k decisions, ckpt_016 = 40k, ckpt_040 = 100k, ckpt_100 = 250k ----
for S in 950 951; do sub_rl dH dH       dHfull_all_rnrh    nested_ramp $S 250000 0.16,0.4,1.0 ramp; done
for S in 970 971; do sub_rl dM dDPfirst dDPfull_first_rnrh nested_ramp $S 250000 0.16,0.4,1.0 ramp; done
# ---- {RLPD} nested_sparse: 250k decisions, same checkpoints -------------------------------
for S in 955 956; do sub_rl dH dH       dHfull_all_rnsh    nested_sparse $S 250000 0.16,0.4,1.0 sparse; done
for S in 975 976; do sub_rl dM dDPfirst dDPfull_first_rnsh nested_sparse $S 250000 0.16,0.4,1.0 sparse; done
# ---- {RLPD} staged control (new tip guard): 100k decisions, ckpt_040 = 40k, ckpt_100 = 100k -
for S in 958 959; do sub_rl dH dH       dHfull_all_rzh     staged $S 100000 0.4,1.0 ctl; done
for S in 978 979; do sub_rl dM dDPfirst dDPfull_first_rzh  staged $S 100000 0.4,1.0 ctl; done
# ---- {r2dreamer} nested_ramp: 2M ONLINE steps, milestones 0.5M / 1M / 2M ------------------
for S in 950 951; do sub_r2 dH dHfull_all_rnrh    nested_ramp $S 2000000 '[500000,1000000,2000000]' ramp; done
for S in 970 971; do sub_r2 dM dDPfull_first_rnrh nested_ramp $S 2000000 '[500000,1000000,2000000]' ramp; done
# ---- {r2dreamer} nested_sparse: 4M ONLINE steps, milestones 0.5M / 1M / 2M / 4M -----------
for S in 955 956; do sub_r2 dH dHfull_all_rnsh    nested_sparse $S 4000000 '[500000,1000000,2000000,4000000]' sparse; done
for S in 975 976; do sub_r2 dM dDPfull_first_rnsh nested_sparse $S 4000000 '[500000,1000000,2000000,4000000]' sparse; done
