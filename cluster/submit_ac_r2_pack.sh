#!/usr/bin/env bash
# (ac) REVISION 1: the four {r2dreamer} nested_sparse10 runs as ONE packed allocation.
#   GP=$LAB/gp_ac bash cluster/submit_ac_r2_pack.sh              # the batch (4 seeds, 4M, normal QOS)
#   SMOKE=1 GP=$LAB/gp_ac bash cluster/submit_ac_r2_pack.sh      # 2 seeds x 15k, interactive QOS
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; W=${W:-$LAB/wm_fix_2026-09-03}
: "${GP:?set GP to a fresh clone at 8e54346 or later; never the pinned gp_ladderN}"
case "$GP" in *gp_ladderN*) echo "FATAL: refusing the pinned tree $GP"; exit 1;; esac
R2=${R2:-$W/r2dreamer_ladderN}; DEMOS=$W/demos_state_full; TIP_GUARD=${TIP_GUARD:-not_in_hand}
grep -q 'PACK_SEEDS' "$GP/cluster/wmfix_full.sbatch" || { echo "FATAL: $GP launcher has no PACK_SEEDS mode"; exit 1; }
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($(git -C "$GP" describe --always --dirty)) | r2 $R2"
if [ -n "${SMOKE:-}" ]; then
  SEEDS="9985 9986"; STEPS=15000; MILES='[15000]'; Q=(-p gpu --qos=interactive -t 0-03:00:00 -n 16 --mem=60g)   # interactive QOS caps 1 GPU / 16 cpu / 64 GB per user; TAGV=lnsmoke; NAME=ln_smoke_r2_sparse10_pack
  # a smoke pack: both seeds on the HUMAN set (the set is not what packing changes)
  for s in $SEEDS; do rm -rf "$W/runs/full_r2d_state_dHfull_all_rns10h_${TAGV}_s$s"; done
  env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse10 TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
      R2_MILESTONES="$MILES" TAG=$TAGV EVAL_SETS=hold PACK_SEEDS="$SEEDS" \
      sbatch -J $NAME "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" dHfull_all_rns10h 9985 $STEPS | sed "s/$/  # $NAME/"
  exit 0
fi
# THE BATCH: ONE PACK PER ARM (the launcher's once-only demo gate binds one set per job, so a
# pack cannot mix the two sets). Both arms are packed identically -- that is the symmetry that
# matters -- and the two packs are submitted together with the same request and QOS.
for s in 957 958; do [ -e "$W/runs/full_r2d_state_dHfull_all_rns10h_s$s" ] && { echo "FATAL: run dir for s$s exists"; exit 1; }; done
for s in 977 978; do [ -e "$W/runs/full_r2d_state_dDPfull_first_rns10h_s$s" ] && { echo "FATAL: run dir for s$s exists"; exit 1; }; done
Q=(-p gpu --qos=normal -n 16 --mem=96g)
env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse10 TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
    R2_MILESTONES='[500000,1000000,2000000,4000000]' PACK_SEEDS="957 958" \
    sbatch -J ln_r2_sparse10_pack_dH "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" dHfull_all_rns10h 957 4000000 | sed 's/$/  # ln_r2_sparse10_pack_dH (s957 s958)/'
env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=nested_sparse10 TIP_GUARD=$TIP_GUARD R2_LONG_RUN=1 \
    R2_MILESTONES='[500000,1000000,2000000,4000000]' PACK_SEEDS="977 978" \
    sbatch -J ln_r2_sparse10_pack_dM "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" dDPfull_first_rns10h 977 4000000 | sed 's/$/  # ln_r2_sparse10_pack_dM (s977 s978)/'
