#!/bin/bash
# THE one smoke before the paper week (2026-07-31).
#
# It deliberately does NOT re-test DP/ACT training-on-harvest: the live ouroboros
# lineages (DP + ACT, wandb genesis_pickaplace_ouro) ARE that smoke -- read their
# gen-1 harvest manifests and train curves as the DP/ACT pass criteria. What has no
# precedent, and what this launches:
#
#   job A  verified --images harvest from the gen-0 joint DP teacher
#          -> pick-scope dreamer demos (the M1-DV3 data path, never run)
#   job B  (afterok A) ONE dv3 multi job, 3 runs on one GPU:
#            hdv3_pick_s0 / hdv3_pick_s1   human demos, scope=pick, seeds 0/1
#            m1dv3_pick_s0                 model demos from job A, scope=pick
#
# Job B is double-duty: it is the cluster-dv3 plumbing smoke (first dv3 run ever on
# the repaired env; per-run SEED threading still unexercised -- audit B4) AND the H4
# learnability gate (no dv3 run anywhere has demonstrated nonzero picked; the local
# joint_pick launch on 07-31 08:47 silently failed and full-scope joint_ref_local
# sat at log_picked 0.0 for 582k steps). Runs get the full 5M budget: if the first
# hours look healthy they simply CONTINUE as real H-DV3/M1-DV3 pick-scope seeds.
#
# PASS CRITERIA (check ~6-12h in):
#   A: manifest kept>=12, rejected_by_verify <=5%, genesis_m1_smoke populated
#   B: all 3 run.log past genesis banner; train_return + log_picked/log_tipped in
#      metrics.jsonl; the two hdv3 seeds' train curves DIVERGE (identical => SEED
#      never reached dreamer, audit B4); periodic eval rows in wandb WITH videos
#   H4 gate (longer horizon): log_picked > 0 by ~1-2M steps on either hdv3 seed
#
# PREFLIGHT (this script checks): demonstrations/genesis_pick on the cluster --
# rsync from the dev box, demos NEVER travel by git:
#   rsync -av <devbox>:~/workspace/dreamerv3-torch/demonstrations/genesis_pick/ \
#     <dv3_checkout>/demonstrations/genesis_pick/
#
# Usage (from the genesis_pickaplace checkout root, after git pull in BOTH repos):
#   bash cluster/launch_paper_smoke.sh          # submit
#   DRYRUN=1 bash cluster/launch_paper_smoke.sh # print the plan
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:-$PWD}"
export GENESIS_PICKAPLACE_ROOT="$PWD"
DV3=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
TEACHER_CKPT=${TEACHER_CKPT:-ouroboros/ouro_dp_joint/gen0/dp/checkpoints/last/pretrained_model}

FAIL=0
[ -d "$TEACHER_CKPT" ] || { echo "PREFLIGHT FAIL: teacher missing: $TEACHER_CKPT"; FAIL=1; }
NPICK=$(ls "$DV3/demonstrations/genesis_pick"/*.npz 2>/dev/null | wc -l)
[ "$NPICK" -ge 80 ] || { echo "PREFLIGHT FAIL: $NPICK/91 demos in $DV3/demonstrations/genesis_pick -- rsync it (see header)"; FAIL=1; }
[ -f "$DV3/sbatch_genesis_multi.sh" ] || { echo "PREFLIGHT FAIL: no $DV3/sbatch_genesis_multi.sh (git pull the dv3 repo)"; FAIL=1; }
grep -q 'DEMODIR' "$DV3/sbatch_genesis_multi.sh" || { echo "PREFLIGHT FAIL: dv3 sbatch predates DEMODIR support (git pull the dv3 repo)"; FAIL=1; }
[ "$FAIL" -eq 0 ] || exit 1
echo "== preflight OK ($NPICK human pick-scope demos, teacher present)"

RUNS="TAG=hdv3_pick_s0 VEC=1 SEED=0 SCOPE=pick DEMODIR=genesis_pick | TAG=hdv3_pick_s1 VEC=1 SEED=1 SCOPE=pick DEMODIR=genesis_pick | TAG=m1dv3_pick_s0 VEC=1 SEED=0 SCOPE=pick DEMODIR=genesis_m1_smoke"

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] sbatch cluster/sbatch_paper_smoke_harvest.sh   (TEACHER_CKPT=$TEACHER_CKPT)"
  echo "[dry] sbatch --dependency=afterok:<A> $DV3/sbatch_genesis_multi.sh RUNS='$RUNS'"
  exit 0
fi

AID=$(sbatch --parsable \
  --export=ALL,GENESIS_PICKAPLACE_ROOT=$PWD,DV3_DIR=$DV3,TEACHER_CKPT=$TEACHER_CKPT,SMOKE_KEPT=${SMOKE_KEPT:-20},CONDA_ENV=${CONDA_ENV:-} \
  cluster/sbatch_paper_smoke_harvest.sh)
echo "== job A (harvest+convert): $AID"

BID=$(env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-}" WANDB=1 RUNS="$RUNS" \
  GENESIS_PICKAPLACE_ROOT="$PWD" \
  sbatch --parsable --dependency=afterok:$AID --job-name=dv3-paper-smoke \
  "$DV3/sbatch_genesis_multi.sh")
echo "== job B (dv3 x3, waits for A): $BID"
echo
echo "== watch:  squeue -u \$USER ; tail -f paper_smoke_harvest_${AID}.out"
echo "== dv3 logs land in $DV3/logs_cluster/genesis_pixels_{hdv3_pick_s0,hdv3_pick_s1,m1dv3_pick_s0}/run.log"
echo "== pass criteria are in this script's header"
