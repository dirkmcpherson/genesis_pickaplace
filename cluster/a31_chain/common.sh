#!/bin/bash
# PREREG A31 chain (2026-09-01): cluster-resident slurm dependency chain
#   teachers (DP on PRUNED dHv2, 2 seeds/world) -> select (§3.1) -> dDPv2 harvest shards (attempts 8, verify)
#   -> merge -> build (dDPv2 vs dHv2raw, dDPv2p vs dHv2; lerobot; r2d) -> submit machine-arm learners + A27 packs.
# Sourced by every stage. All stages append to $CHAIN_LOG.
LAB=/cluster/tufts/shortlab/jstale02
GPR=$LAB/genesis_pickaplace
CONDA_ENV=$LAB/condaenv/genesis
CHAIN_LOG=$LAB/ddpv2_chain.log
CHAIN_LOGDIR=$LAB/ddpv2_chain_logs
SHARD_N=6; ROLLOUT_BASE=100000
clog() { echo "$(date '+%F %T') [${SLURM_JOB_ID:-login}:${STAGE:-?}${WORLD:+/$WORLD}] $*" | tee -a "$CHAIN_LOG"; }
fatal() { clog "FATAL: $*"; exit 1; }
world_cfg() {
  WORLD=$1
  case "$WORLD" in
    old) WAVE_T=pilotv2P;   DEMO_ROOT=baselines/matched_v2; SIM_VARIANT=base;                 IC_FILE=baselines/eval_ics_v2.json;    SET=dDPv2;    SIMEXTRA="";                                      DPWAVE=v2fullP;   RLWAVE=g99v2full ;;
    w3)  WAVE_T=pilotv2Pw3; DEMO_ROOT=baselines/matched_w3; SIM_VARIANT=gc_kp4_riser3_shelf6; IC_FILE=baselines/eval_ics_v2_w3.json; SET=dDPv2_w2; SIMEXTRA="--sim-variant gc_kp4_riser3_shelf6"; DPWAVE=v2fullPw3; RLWAVE=g99v2fullw3 ;;
    *) echo "FATAL: world=$WORLD (old|w3)"; exit 1 ;;
  esac
  OUTDIR=baselines/demos_v2/$SET            # harvest success dir (+ ${SET}_fails)
  TDIR=baselines/outputs/dp_$WAVE_T         # teacher outputs: $TDIR/dHv2_DP_s{0,1}
  RAWBASE=$DEMO_ROOT/dHv2raw                # RLPD/WM human arm (A25/A26)  -> dDPv2 view
  PRUNEDBASE=$DEMO_ROOT/dHv2                # DP human arm of record (A29) -> dDPv2p view
}
activate() { source ~/.bashrc >/dev/null 2>&1; conda activate "$CONDA_ENV" || fatal "conda activate $CONDA_ENV"; }
