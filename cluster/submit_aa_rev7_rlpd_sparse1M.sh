#!/usr/bin/env bash
# (aa) REVISION 7: {RLPD} nested_sparse at the budget matched to the world model's 4M sim steps = 1M decisions
# (action_repeat 4), 4 v 4, normal QOS, one seed per GPU (the RLPD launcher does not pack). Registered BEFORE submission.
#   GP=$LAB/gp_aa4 bash cluster/submit_aa_rev7_rlpd_sparse1M.sh
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
source "$(dirname "$0")/r2_milestones.sh"
: "${GP:?set GP to gp_aa4}"; case "$GP" in *gp_ladderN*|*gp_ac) echo "FATAL: refusing pinned tree $GP"; exit 1;; esac
DEMOS=$W/demos_state_full; TIP_GUARD=${TIP_GUARD:-not_in_hand}; RL_OUT_ROOT=baselines/rl/checkpoints/e2e_aa7
STEPS=1000000; FRACS=$RLPD_FRACS_1M
for s in dHfull_all_rnsh dDPfull_first_rnsh; do
  python3 -c "import json; m=json.load(open('$DEMOS/$s/repeat.json')); assert m['relabel']['ladder']=='nested_sparse'" || { echo "FATAL: $s ladder"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab | tail -1 | tr -dc '0-9'); [ "$FREE_GB" -ge 150 ] || { echo "FATAL: ${FREE_GB} GB < 150"; exit 1; }
echo "DISK-OK ${FREE_GB} GB | gp $(git -C "$GP" describe --always --dirty) | steps $STEPS fracs $FRACS"
# normal QOS (never preempt: user 2026-09-13), 2-day walltime (1M decisions ~36 h at the measured ~18 h/500k), nice 0
RLQ=(-p gpu --qos=normal --nice=0 --time=2-00:00:00)
sub_rl() {  # arm demo seed
  local arm=$1 demo=$2 seed=$3 name=ln_rl_sparse1M_${arm}_s${seed}
  [ -e "$GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed}" ] && { echo "FATAL: $GP/$RL_OUT_ROOT/e2e_rlpd_${arm}_s${seed} exists"; exit 1; }
  ( cd "$GP" && env GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse TIP_GUARD=$TIP_GUARD ARM=$arm SEED=$seed \
      STEPS=$STEPS DEMO=$DEMOS/$demo CKPT_FRACS=$FRACS OUT_ROOT=$RL_OUT_ROOT \
      sbatch -J "$name" "${RLQ[@]}" cluster/sbatch_rlpd_e2e.sh ) | sed "s/$/  # $name/"
}
for S in 1955 1956 1957 1958; do sub_rl dH      dHfull_all_rnsh    $S; done
for S in 1975 1976 1977 1978; do sub_rl dDPfirst dDPfull_first_rnsh $S; done
