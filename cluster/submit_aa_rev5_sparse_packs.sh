#!/usr/bin/env bash
# (aa) REVISION 5: four packed {r2dreamer} nested_sparse packs (+4 v +4, 4M, milestones). Registered in
# PHASE_PLAN (aa) rev 5 BEFORE submission. Mirrors submit_aa_rev4_packs.sh with the sparse ladder/sets/seeds.
#   GP=$LAB/gp_aa4 bash cluster/submit_aa_rev5_sparse_packs.sh
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
: "${GP:?set GP to the pinned rev-4/5 clone (gp_aa4); never gp_ladderN or gp_ac}"
case "$GP" in *gp_ladderN*|*gp_ac) echo "FATAL: refusing pinned tree $GP"; exit 1;; esac
R2=${R2:-$W/r2dreamer_ladderN}; DEMOS=$W/demos_state_full; TIP_GUARD=${TIP_GUARD:-not_in_hand}
grep -q 'PACK_SEEDS' "$GP/cluster/wmfix_full.sbatch" || { echo "FATAL: no PACK_SEEDS mode in $GP"; exit 1; }
for s in dHfull_all_rnsh dDPfull_first_rnsh; do
  python3 -c "import json; m=json.load(open('$DEMOS/$s/repeat.json')); assert m['relabel']['ladder']=='nested_sparse', m['relabel']['ladder']" || { echo "FATAL: $s is not a nested_sparse set"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($(git -C "$GP" describe --always --dirty)) | r2 $R2 ($(git -C "$R2" rev-parse --short HEAD))"
Q=(-p gpu --qos=normal -n 16 --mem=96g)
pack () {  # arm set seedA seedB
  local arm=$1 set=$2 a=$3 b=$4
  for s in $a $b; do [ -e "$W/runs/full_r2d_state_${set}_s$s" ] && { echo "FATAL: run dir for s$s exists"; exit 1; }; done
  env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
      R2_MILESTONES='[500000,1000000,2000000,4000000]' PACK_SEEDS="$a $b" \
      sbatch -J ln_r2_sparse4M_pack_${arm}_s${a}_${b} "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$set" "$a" 4000000 | sed "s/$/  # ln_r2_sparse4M_pack_${arm} (s$a s$b)/"
}
pack dH dHfull_all_rnsh    1955 1956
pack dH dHfull_all_rnsh    1957 1958
pack dM dDPfull_first_rnsh 1975 1976
pack dM dDPfull_first_rnsh 1977 1978
