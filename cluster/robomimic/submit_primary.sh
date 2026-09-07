#!/bin/bash
# Robomimic leg: the PRIMARY run matrix (plan §4/§5) = 3 sources x 3 learners x 8 seeds = 72 runs
# (RLPD 100k decisions, r2dreamer 500k online decisions, DP 100k grad steps; LAST checkpoint on the 50-state bank).
# DRY-RUN BY DEFAULT: prints every sbatch line and the GPU-hour estimate. Submits ONLY with GO=1 -- the user's go.
#   bash cluster/robomimic/submit_primary.sh            # dry run (prints)
#   GO=1 bash cluster/robomimic/submit_primary.sh       # submits (the Monday go)
# Optional: SOURCES="PH200 MH200 MG200s" LEARNERS="rlpd r2d dp" SEEDS="0 1 2 3 4 5 6 7" MAXQ (default 60: refuse to
# submit if more than this many of our jobs are already queued/running, so the preempt QOS is not flooded).
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02
ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace}; C=$ROOT/cluster/robomimic
SOURCES=${SOURCES:-"PH200 MH200 MG200s"}; LEARNERS=${LEARNERS:-"rlpd r2d dp"}; SEEDS=${SEEDS:-"0 1 2 3 4 5 6 7"}; MAXQ=${MAXQ:-60}
declare -A HOURS=([rlpd]=3.0 [r2d]=6.0 [dp]=3.0)      # per-run GPU-hour estimates (plan §4; r2d/dp UNVERIFIED until the pilots)
declare -A SCRIPT=([rlpd]=sbatch_rlpd_robo.sh [r2d]=sbatch_r2d_robo.sh [dp]=sbatch_dp_robo.sh)
for a in $SOURCES; do for l in $LEARNERS; do
  case $l in rlpd) f=$LAB/robomimic_data/arms/$a/rlpd/transitions.npz ;; r2d) f=$LAB/robomimic_data/arms/$a/r2d/repeat.json ;; dp) f=$LAB/robomimic_data/arms/$a/lerobot/meta/info.json ;; esac
  [ -f "$f" ] || { echo "FATAL: arm $a not converted for $l ($f missing)"; exit 1; }
done; done
[ -f $LAB/robomimic_data/bank_can50.npz ] || { echo "FATAL: bank missing"; exit 1; }
[ -f $LAB/robomimic_data/g0_report.json ] && python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("G0:", r["verdict"], str(r["n_pass"]) + "/" + str(r["n"]), "success-flag agree", r.get("n_success_flag_agree")); sys.exit(0 if r["verdict"]=="PASS" else 1)' $LAB/robomimic_data/g0_report.json || { echo "FATAL: G0 not passed/recorded"; exit 1; }
n=0; total=0
for l in $LEARNERS; do for a in $SOURCES; do for s in $SEEDS; do
  echo "ARM=$a SEED=$s sbatch $C/${SCRIPT[$l]}"; n=$((n + 1)); total=$(python3 -c "print($total + ${HOURS[$l]})")
done; done; done
echo "# $n runs, ~$total GPU-h (rlpd ${HOURS[rlpd]}h, r2d ${HOURS[r2d]}h, dp ${HOURS[dp]}h per run; preempt QOS, <= 20 GPUs per user)"
if [ -z "${GO:-}" ]; then echo "# DRY RUN (set GO=1 to submit)"; exit 0; fi
Q=$(squeue -u "$USER" -h | wc -l); [ "$Q" -le "$MAXQ" ] || { echo "FATAL: $Q jobs already queued/running > MAXQ=$MAXQ"; exit 1; }
for l in $LEARNERS; do for a in $SOURCES; do for s in $SEEDS; do
  J=$(ARM=$a SEED=$s sbatch --parsable $C/${SCRIPT[$l]}); echo "$(date -Is) submitted $J $l $a s$s" | tee -a $LAB/robomimic_runs/SUBMITTED.log
done; done; done
