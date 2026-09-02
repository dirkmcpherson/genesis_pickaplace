#!/bin/bash
# Stage 3: adapted from $LAB/ddpv2_build.sh (merges moved to merge.sh). ONE harvest per world -> TWO per-IC matched views:
#   dDPv2  = matched vs dHv2raw (A25/A26 raw base; RLPD machine arm; WM arm via r2d in w3)
#   dDPv2p = matched vs dHv2    (pruned base;       DP machine arm; lerobot)
# --allow-short fallback exactly as ddpv2_build.sh (strict first; rerun only on the per-IC-shortfall FATAL).
source "$(dirname "$0")/common.sh"; STAGE=build; cd "$GPR" || exit 1
export CUDA_VISIBLE_DEVICES= GENESIS_PICKAPLACE_ROOT=$GPR MUJOCO_GL=egl; activate
clog "build start"
build() { # $1 src $2 name $3 out-root $4 base
  local L=$CHAIN_LOGDIR/mv2_$2_$(basename $3).log
  if python baselines/make_v2_matched.py --src $1 --name $2 --out-root $3 --base $4 2>&1 | tee "$L"; then return 0; fi
  if grep -q "FATAL: per-IC shortfall" "$L"; then
    clog "$3/$2: strict per-IC match FATAL on shortfall ONLY -> rerun --allow-short (manifest short_ics reports it)"
    python baselines/make_v2_matched.py --src $1 --name $2 --out-root $3 --base $4 --allow-short || fatal "make_v2_matched --allow-short $3/$2"
  else fatal "make_v2_matched $3/$2 failed (not a shortfall)"; fi
}
for W in old w3; do
  world_cfg $W
  build $OUTDIR dDPv2  $DEMO_ROOT $RAWBASE
  build $OUTDIR dDPv2p $DEMO_ROOT $PRUNEDBASE
  python baselines/convert_to_lerobot.py $DEMO_ROOT/dDPv2p $DEMO_ROOT/dDPv2p/lerobot 8 4 none image || fatal "lerobot $DEMO_ROOT/dDPv2p"
done
python baselines/rl/to_dreamer_native.py --src baselines/matched_w3/dDPv2 --dst baselines/matched_w3/r2d/dDPv2 --repeat 4 --terminal-reward 1 --scope pick --force || fatal "to_dreamer_native matched_w3/dDPv2"
clog "r2d npz: $(ls baselines/matched_w3/r2d/dDPv2/*.npz | wc -l)"
for S in matched_v2/dDPv2 matched_v2/dDPv2p matched_w3/dDPv2 matched_w3/dDPv2p; do
  python3 -c "import json;m=json.load(open('baselines/$S/manifest.json'));n=m.get('notes',{}) or {};print('SET-RESULT','$S','N=',m['N'],'sim_variant=',m['sim_variant'],'sha=',m['content_sha256'][:16],'short=',n.get('short_ics'),'identical=',n.get('ic_multiset_identical'))" | tee -a "$CHAIN_LOG"
done
clog "DDPV2-BUILD-OK"
