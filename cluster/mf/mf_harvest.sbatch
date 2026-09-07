#!/bin/bash
# Machine-first arm: harvest the reward-only teacher on the 66 human raw ICs (the ic_uids of
# baselines/matched_w3/dHv2raw), pick scope, world w3, contract-v1 tapes, --verify, 3 attempts per IC,
# sampled actions -- exactly how dDPv2 was harvested (baselines/demos_v2/dDPv2_w2 shard manifests:
# --mode sample --ic-mode demo --uids <66> --verify --sim-variant gc_kp4_riser3_shelf6; attempts 8 there,
# 3 here as registered in paper/MACHINE_FIRST_PLAN_2026-09-07.md §2).
#
# usage (array of 6 shards, CPU):
#   TEACHER=r2d CKPT=$W/runs/mf_r2d_t1_s<k>/latest.pt OUTDIR=baselines/demos_v2/dRL_w3 \
#     sbatch -p batch,preempt --array=0-5 cluster/mf/mf_harvest.sbatch
#   negative control (must keep ~0):  TEACHER=random OUTDIR=baselines/demos_v2/dRL_w3_randomctl ATTEMPTS=1 \
#     sbatch -p batch,preempt --array=0-5 cluster/mf/mf_harvest.sbatch
#   merge after the array:  MERGE=1 TEACHER=r2d OUTDIR=... sbatch ... (or run the merge line by hand)
#SBATCH -J mf_harvest
#SBATCH -p batch,preempt
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=24g
#SBATCH -t 0-12:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%A_%a.out
set -eo pipefail
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
cd "${GENESIS_PICKAPLACE_ROOT:=$LAB/mf_gp}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
export R2DREAMER_ROOT=${R2DREAMER_ROOT:-$W/r2dreamer_fix}     # the r2dreamer tree of record (read-only)
PY=${PY:-$LAB/r2d_venv/bin/python}                              # the venv the T1 runs train in (genesis + r2dreamer deps)
TEACHER=${TEACHER:?r2d|random}
OUTDIR=${OUTDIR:?e.g. baselines/demos_v2/dRL_w3}
ATTEMPTS=${ATTEMPTS:-3}
SHARD=${SLURM_ARRAY_TASK_ID:-0}; NSHARD=${NSHARD:-6}
SEED=${SEED:-0}
# the 66 human raw ICs = ic_uid_histogram keys of matched_w3/dHv2raw/manifest.json (66 resettable uids)
UIDS=$($PY - <<'PYU'
import json; m = json.load(open('baselines/matched_w3/dHv2raw/manifest.json'))
u = sorted(int(k) for k in m['ic_uid_histogram']); assert len(u) == 66, len(u); print(' '.join(map(str, u)))
PYU
)
echo "# $(date -Is) host=$(hostname) teacher=$TEACHER ckpt=${CKPT:-} outdir=$OUTDIR shard=$SHARD/$NSHARD attempts=$ATTEMPTS git=$(git rev-parse --short HEAD) r2d_root=$R2DREAMER_ROOT py=$PY"
echo "# uids ($(echo $UIDS | wc -w)): $UIDS"
if [ "${MERGE:-0}" = 1 ]; then
  $PY baselines/record_demos.py --teacher "$TEACHER" --outdir "$OUTDIR" --merge; exit $?
fi
CK=()
if [ "$TEACHER" = r2d ]; then
  CKPT=${CKPT:?path to latest.pt}; [ -f "$CKPT" ] || { echo "FATAL: $CKPT missing"; exit 2; }
  CFG=$(dirname "$CKPT")/.hydra/config.yaml; [ -f "$CFG" ] || { echo "FATAL: $CFG missing"; exit 2; }
  CK=(--checkpoint "$CKPT" --config "$CFG")
fi
CMD=($PY baselines/record_demos.py --teacher "$TEACHER" "${CK[@]}" --mode sample --ic-mode demo --uids $UIDS
     --attempts "$ATTEMPTS" --scope pick --verify --sim-variant gc_kp4_riser3_shelf6
     --outdir "$OUTDIR" --shard-idx "$SHARD" --shard-n "$NSHARD" --seed "$SEED" --torch-threads 2)
printf '%q ' "${CMD[@]}"; echo
[ "$SHARD" = 0 ] && { printf '%s ' "$(date -Is)" >> $W/COMMANDS.log; printf '%q ' "${CMD[@]}" >> $W/COMMANDS.log; echo " # array ${SLURM_ARRAY_JOB_ID:-local} x$NSHARD" >> $W/COMMANDS.log; }
"${CMD[@]}"
echo "# DONE shard $SHARD $(date -Is)"
