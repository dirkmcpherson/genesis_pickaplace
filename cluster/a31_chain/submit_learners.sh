#!/bin/bash
# Stage 4: DRYRUN gates -> DP machine arm (dDPv2p, v2fullP/v2fullPw3, s50-57) + RLPD machine arm (dDPv2, g99v2full/g99v2fullw3,
# s50-57) -> release A28 packs -> A27 WM packs (corrected world, dHv2raw vs dDPv2, seeds 90-101 as 2-seed packs) if N>=45.
source "$(dirname "$0")/common.sh"; STAGE=submit; cd "$GPR" || exit 1
export GENESIS_PICKAPLACE_ROOT=$GPR
unset DEMO_DIR RAW DATASET ARM SEED WAVE DEMO_ROOT SIM_VARIANT IC_FILE   # stale-env guards
clog "submit stage start (git $(git rev-parse --short HEAD))"
FAT=0
gate() { # $1 label, rest = command
  local label=$1; shift; local out; out=$("$@" 2>&1); local rc=$?
  echo "$out" | grep -E "FATAL|DEMO-SHA|PROVENANCE-OK|\[dry\] (ARM|RAW|demo_dir)|n-band|R2D-DEMOSET" | head -4 | sed "s|^|[$label] |" | tee -a "$CHAIN_LOG"
  if [ $rc -ne 0 ] || echo "$out" | grep -q FATAL; then clog "GATE FAILED: $label"; FAT=1; fi
}
for W in old w3; do world_cfg $W
  gate "dp dDPv2p $W"  env DRYRUN=1 ARM=dDPv2p SEED=50 WAVE=$DPWAVE DEMO_ROOT=$DEMO_ROOT SIM_VARIANT=$SIM_VARIANT IC_FILE=$IC_FILE bash cluster/sbatch_dp.sh
  gate "rlpd dDPv2 $W" env DRYRUN=1 ARM=dDPv2 SEED=50 REWARD=sparse GAMMA=0.99 UTD=10 WAVE=$RLWAVE DEMO_ROOT=$DEMO_ROOT SIM_VARIANT=$SIM_VARIANT IC_FILE=$IC_FILE bash cluster/sbatch_rlpd.sh
done
[ $FAT -eq 0 ] || fatal "DRYRUN gate(s) failed -- NOTHING submitted"
N=0; IDS=""
for S in 50 51 52 53 54 55 56 57; do for W in old w3; do world_cfg $W
  id=$(env ARM=dDPv2p SEED=$S WAVE=$DPWAVE DEMO_ROOT=$DEMO_ROOT SIM_VARIANT=$SIM_VARIANT IC_FILE=$IC_FILE sbatch --parsable --exclude=pax007 -p gpu cluster/sbatch_dp.sh) && { N=$((N+1)); IDS="$IDS ${id%%;*}"; }
  id=$(env ARM=dDPv2 SEED=$S REWARD=sparse GAMMA=0.99 UTD=10 WAVE=$RLWAVE DEMO_ROOT=$DEMO_ROOT SIM_VARIANT=$SIM_VARIANT IC_FILE=$IC_FILE sbatch --parsable --exclude=pax007 -p gpu cluster/sbatch_rlpd.sh) && { N=$((N+1)); IDS="$IDS ${id%%;*}"; }
done; done
clog "machine-arm learners submitted=$N/32 ids:$IDS"
scontrol release 3120763 3120764 3120765 3120766 3120767 3120768 && clog "A28 packs released (3120763-68)" || clog "WARN: A28 release returned rc=$?"
NW3=$(python3 -c "import json;print(json.load(open('baselines/matched_w3/dDPv2/manifest.json'))['N'])")
if [ "${NW3:-0}" -ge 45 ]; then
  FAT=0
  for A in dHv2raw dDPv2; do
    gate "r2d $A" env DRYRUN=1 ARM=$A SEED=90 DEMOSET=w3 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400 bash cluster/sbatch_r2dreamer.sh
  done
  if [ $FAT -eq 0 ]; then
    P=0; PIDS=""
    for A in dHv2raw dDPv2; do for pair in "90 91" "92 93" "94 95" "96 97" "98 99" "100 101"; do
      id=$(env ARM=$A PACK_SEEDS="$pair" DEMOSET=w3 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400 sbatch --parsable --exclude=pax007 -p gpu -n 16 --mem 90g --time=30:00:00 cluster/sbatch_r2dreamer_pack.sh) && { P=$((P+1)); PIDS="$PIDS ${id%%;*}"; }
    done; done
    clog "A27 packs submitted=$P/12 (matched_w3/dDPv2 N=$NW3) ids:$PIDS"
  else clog "A27 pack DRYRUN gate failed -- packs NOT submitted (N=$NW3)"; fi
else clog "A27-SKIPPED-LOW-N (matched_w3/dDPv2 N=$NW3 < 45)"; fi
clog "submit stage done"
