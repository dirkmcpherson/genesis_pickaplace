#!/usr/bin/env bash
# (aa) REVISION 6: one wave of {r2dreamer} nested_sparse 4M packs. Registered BEFORE submission.
#   GP=$LAB/gp_aa4 WAVE=A bash cluster/submit_aa_rev6_sparse_wave.sh   # 1 pack of 4 per arm (4/GPU)
#   GP=$LAB/gp_aa4 WAVE=B bash cluster/submit_aa_rev6_sparse_wave.sh   # 2 packs of 2 per arm (2/GPU)
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
source "$(dirname "$0")/r2_milestones.sh"
: "${GP:?set GP to gp_aa4}"; : "${WAVE:?A or B}"
case "$GP" in *gp_ladderN*|*gp_ac) echo "FATAL: refusing pinned tree $GP"; exit 1;; esac
R2=${R2:-$W/r2dreamer_ladderN}; DEMOS=$W/demos_state_full; TIP_GUARD=${TIP_GUARD:-not_in_hand}
for s in dHfull_all_rnsh dDPfull_first_rnsh; do
  python3 -c "import json; m=json.load(open('$DEMOS/$s/repeat.json')); assert m['relabel']['ladder']=='nested_sparse'" || { echo "FATAL: $s ladder"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab | tail -1 | tr -dc '0-9'); [ "$FREE_GB" -ge 150 ] || { echo "FATAL: ${FREE_GB} GB < 150"; exit 1; }
echo "DISK-OK ${FREE_GB} GB | gp $(git -C "$GP" describe --always --dirty) | r2 $(git -C "$R2" rev-parse --short HEAD) | wave $WAVE"
pack () {  # arm set mem seeds...
  local arm=$1 set=$2 mem=$3; shift 3; local seeds="$*"; local tag=$(echo $seeds | tr ' ' '_')
  for s in $seeds; do [ -e "$W/runs/full_r2d_state_${set}_s$s" ] && { echo "FATAL: run dir for s$s exists"; exit 1; }; done
  env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
      R2_MILESTONES="$R2_MILESTONES_4M" PACK_SEEDS="$seeds" \
      sbatch -J ln_r2_sparse4M_w${WAVE}_${arm}_s${tag} -p gpu --qos=normal -n 16 --mem=$mem "$GP/cluster/wmfix_full.sbatch" "$set" "${1}" 4000000 | sed "s/$/  # wave $WAVE $arm ($seeds)/"
}
if [ "$WAVE" = A ]; then
  pack dH dHfull_all_rnsh    160g 1959 1960 1961 1962
  pack dM dDPfull_first_rnsh 160g 1979 1980 1981 1982
else
  pack dH dHfull_all_rnsh    96g 1963 1964
  pack dH dHfull_all_rnsh    96g 1965 1966
  pack dM dDPfull_first_rnsh 96g 1983 1984
  pack dM dDPfull_first_rnsh 96g 1985 1986
fi
