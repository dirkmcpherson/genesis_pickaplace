#!/usr/bin/env bash
# PHASE_PLAN amendment (ac): the nested_sparse10 2 v 2 batch, MIRRORING (aa) revision 3's sparse arm
# line for line (cluster/submit_ln_rev3.sh) -- same seeds (PAIRED with rev-3's nested_sparse:
# s957/958 human, s977/978 machine), same budgets ({r2dreamer} 4M online with the milestone list,
# {RLPD} 500k with ckpt fracs 0.2/0.5/1.0), same QOS split (world model on normal, RLPD on preempt),
# same tip guard, far_release off. Only the ladder (nested_sparse10), the sets (_rns10h), the gp
# tree (a fresh clone at >= 8e54346, never the pinned gp_ladderN) and the {RLPD} OUT_ROOT (e2e_ac,
# so paired seeds cannot collide with rev-3's e2e_rev3 dirs) differ.
#
#   GP=$LAB/gp_ac bash cluster/submit_ac_batch.sh          (DRYRUN=1 to print the eight commands)
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
: "${GP:?set GP to a fresh clone at 8e54346 or later; never the pinned gp_ladderN}"
case "$GP" in *gp_ladderN*) echo "FATAL: refusing the pinned tree $GP"; exit 1;; esac
R2=${R2:-$W/r2dreamer_ladderN}
DEMOS=$W/demos_state_full
TIP_GUARD=${TIP_GUARD:-not_in_hand}
RL_OUT_ROOT=${RL_OUT_ROOT:-baselines/rl/checkpoints/e2e_ac}
[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] || { echo "FATAL: $R2 is not an r2dreamer tree"; exit 1; }
grep -q "'nested_sparse10'" "$GP/baselines/rl/full_env.py" || { echo "FATAL: $GP predates nested_sparse10"; exit 1; }
grep -q "nested_sparse10" "$GP/cluster/wmfix_full.sbatch" || { echo "FATAL: $GP launchers predate 8e54346 (preflight would refuse)"; exit 1; }
for s in dHfull_all_rns10h dDPfull_first_rns10h; do
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json"; exit 1; }
  python3 -c "import json,sys; m=json.load(open('$DEMOS/$s/repeat.json')); assert m['relabel']['ladder']=='nested_sparse10' and abs(m['total_reward']-120.0)<1e-9, m['total_reward']" \
    || { echo "FATAL: $DEMOS/$s is not the nested_sparse10 set (ladder / total_reward 120.0)"; exit 1; }
done
DESC=$(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git')
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to submit"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | tree $GP ($DESC) | r2 $R2 ($(git -C "$R2" rev-parse --short HEAD 2>/dev/null || echo 0cf3d9e-copy))"
RLQ=(-p preempt --qos=preempt --nice=0)
R2Q=(-p gpu --qos=normal)
sub_rl() {   # tag arm demo ladder seed steps ckpt_fracs shortladder
  local tag=$1 arm=$2 demo=$3 ladder=$4 seed=$5 steps=$6 fracs=$7 short=$8
  local name=ln_rl_${short}_${tag}_s${seed}
  if [ -e "$GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed}" ]; then
    echo "FATAL: $GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed} exists -- refusing to submit $name over it"; exit 1; fi
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
    echo "FATAL: $W/runs/full_r2d_state_${demo}_s${seed} exists -- refusing to submit $name over it"; exit 1; fi
  local cmd=(env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder TIP_GUARD=$TIP_GUARD
             R2_LONG_RUN=1 R2_MILESTONES="$miles"
             sbatch -J "$name" "${R2Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$demo" "$seed" "$steps")
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name"; return; fi
  "${cmd[@]}" | sed "s/$/  # $name/"
}
for S in 957 958; do sub_r2 dH dHfull_all_rns10h    nested_sparse10 $S 4000000 '[500000,1000000,2000000,4000000]' sparse10; done
for S in 977 978; do sub_r2 dM dDPfull_first_rns10h nested_sparse10 $S 4000000 '[500000,1000000,2000000,4000000]' sparse10; done
for S in 957 958; do sub_rl dH dH       dHfull_all_rns10h    nested_sparse10 $S 500000 0.2,0.5,1.0 sparse10_500k; done
for S in 977 978; do sub_rl dM dDPfirst dDPfull_first_rns10h nested_sparse10 $S 500000 0.2,0.5,1.0 sparse10_500k; done
