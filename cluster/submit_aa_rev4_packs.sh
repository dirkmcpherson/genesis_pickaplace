#!/usr/bin/env bash
# (aa) REVISION 4: four packed {r2dreamer} nested_ramp packs (+4 v +4, 4M, milestones).
#   GP=$LAB/gp_aa4 bash cluster/submit_aa_rev4_packs.sh
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
: "${GP:?set GP to the fresh clone for this revision (gp_aa4); never gp_ladderN or gp_ac}"
case "$GP" in *gp_ladderN*|*gp_ac) echo "FATAL: refusing pinned tree $GP"; exit 1;; esac
R2=${R2:-$W/r2dreamer_ladderN}; DEMOS=$W/demos_state_full; TIP_GUARD=${TIP_GUARD:-not_in_hand}
grep -q 'PACK_SEEDS' "$GP/cluster/wmfix_full.sbatch" || { echo "FATAL: no PACK_SEEDS mode in $GP"; exit 1; }
for s in dHfull_all_rnrh dDPfull_first_rnrh; do
  python3 -c "import json; m=json.load(open('$DEMOS/$s/repeat.json')); assert m['relabel']['ladder']=='nested_ramp', m['relabel']['ladder']" || { echo "FATAL: $s is not a nested_ramp set"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($(git -C "$GP" describe --always --dirty)) | r2 $R2"
Q=(-p gpu --qos=normal -n 16 --mem=96g)
pack () {  # arm set seedA seedB
  local arm=$1 set=$2 a=$3 b=$4
  for s in $a $b; do [ -e "$W/runs/full_r2d_state_${set}_s$s" ] && { echo "FATAL: run dir for s$s exists"; exit 1; }; done
  env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_ramp TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
      R2_MILESTONES='[500000,1000000,2000000,4000000]' PACK_SEEDS="$a $b" \
      sbatch -J ln_r2_ramp4M_pack_${arm}_s${a}_${b} "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$set" "$a" 4000000 | sed "s/$/  # ln_r2_ramp4M_pack_${arm} (s$a s$b)/"
}
pack dH dHfull_all_rnrh    954 959
pack dH dHfull_all_rnrh    960 961
pack dM dDPfull_first_rnrh 974 979
pack dM dDPfull_first_rnrh 980 981
