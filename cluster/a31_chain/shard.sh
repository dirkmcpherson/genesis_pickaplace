#!/bin/bash
# Stage 2b: one dDPv2 harvest shard. Usage: sbatch -p gpu --gres gpu:1 -c 4 --mem 16g ... shard.sh <old|w3> <K>
# Mirrors cluster/sbatch_record.sh's dp/GRES command (V2_BUILD §7 / addendum 3 harvest) with A31's
# --attempts 8 --verify, over the union IC pool (HARVEST_UIDS.txt); CKPT read from TEACHER_SELECTED.txt.
source /cluster/tufts/shortlab/jstale02/genesis_pickaplace/cluster/a31_chain/common.sh; STAGE=shard; world_cfg "${1:?world}"; K=${2:?shard idx}; cd "$GPR" || exit 1
CKPT=$(sed -n 's/^ckpt_path=//p' "$TDIR/TEACHER_SELECTED.txt"); [ -d "$CKPT" ] || fatal "ckpt missing: '$CKPT'"
UIDS=$(cat "$TDIR/HARVEST_UIDS.txt"); [ -n "$UIDS" ] || fatal "HARVEST_UIDS empty"
export GENESIS_PICKAPLACE_ROOT=$GPR MUJOCO_GL=egl     # GPU kept for the policy (GRES job), sim is CPU
activate
clog "shard $K/$SHARD_N start ckpt=$CKPT out=$OUTDIR node=$(hostname)"
python baselines/record_demos.py --teacher dp --checkpoint "$CKPT" --outdir "$OUTDIR" --seed 0 \
  --shard-idx "$K" --shard-n "$SHARD_N" --rollout-base "$ROLLOUT_BASE" \
  --ic-mode demo --attempts 8 --mode sample --verify --uids $UIDS $SIMEXTRA
rc=$?
MAN=$OUTDIR/manifest_shard${K}_base${ROLLOUT_BASE}.json
if [ $rc -ne 0 ] && [ -f "$MAN" ]; then clog "python rc=$rc AFTER shard manifest was written (interpreter-teardown crash class, cf. 3120441) -> treated as OK"; rc=0; fi
[ -f "$MAN" ] || { clog "shard $K FAILED rc=$rc (no $MAN)"; exit 1; }
clog "shard $K done rc=$rc kept=$(python3 -c "import json;print(json.load(open('$MAN')).get('n_kept'))" 2>/dev/null)"
exit $rc
