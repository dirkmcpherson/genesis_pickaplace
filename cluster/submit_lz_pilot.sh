#!/usr/bin/env bash
# The 16 jobs of PHASE_PLAN amendment (z) -- the unified-ladder staged-versus-sparse pilot.
# Registered BEFORE submission; this script IS the submit command of record, so that what was
# registered and what ran are the same text rather than two retypings of it.
#
#   bash cluster/submit_lz_pilot.sh            # submit
#   DRYRUN=1 bash cluster/submit_lz_pilot.sh   # print the 16 commands and exit
#
# QOS **normal** on partition **gpu** (both launchers bake a different partition/QOS, so those are
# overridden on the command line): the old 64-run batch occupies the preempt allocation, and a
# pilot that queues behind it is a pilot that never reports. Jobs are named `lz_*` so the health
# monitor and `squeue | grep lz_` can see them as one batch.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_unified}
R2=${R2:-$W/r2dreamer_unified}
DEMOS=$W/demos_state_full

[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] || { echo "FATAL: $R2 is not an r2dreamer tree"; exit 1; }
for s in dHfull_all_rz dDPfull_first_rz dHfull_all_rs dDPfull_first_rs; do
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json (build it with cluster/relabel_e2e_sets.sbatch)"; exit 1; }
done
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to submit"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | tree $GP ($(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git')) | r2 $R2"

GPUQ=(-p gpu --qos=normal)

sub_rl() {   # arm_tag arm demo ladder seed steps ckpt_fracs
  local tag=$1 arm=$2 demo=$3 ladder=$4 seed=$5 steps=$6 fracs=$7
  local name=lz_rl_${ladder}_${tag}_s${seed}
  if [ -n "${DRYRUN:-}" ]; then
    echo "GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder ARM=$arm SEED=$seed STEPS=$steps DEMO=$DEMOS/$demo CKPT_FRACS=$fracs sbatch -J $name ${GPUQ[*]} --nice=0 cluster/sbatch_rlpd_e2e.sh"
    return
  fi
  ( cd "$GP" && GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder ARM=$arm SEED=$seed STEPS=$steps \
      DEMO=$DEMOS/$demo CKPT_FRACS=$fracs \
      sbatch -J "$name" "${GPUQ[@]}" --nice=0 cluster/sbatch_rlpd_e2e.sh ) | sed "s/$/  # $name/"
}

sub_r2() {   # arm_tag demo ladder seed steps milestones
  local tag=$1 demo=$2 ladder=$3 seed=$4 steps=$5 miles=$6
  local name=lz_r2_${ladder}_${tag}_s${seed}
  if [ -n "${DRYRUN:-}" ]; then
    echo "R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder R2_LONG_RUN=1 R2_MILESTONES='$miles' sbatch -J $name ${GPUQ[*]} $GP/cluster/wmfix_full.sbatch $demo $seed $steps"
    return
  fi
  R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$ladder R2_LONG_RUN=1 R2_MILESTONES="$miles" \
    sbatch -J "$name" "${GPUQ[@]}" "$GP/cluster/wmfix_full.sbatch" "$demo" "$seed" "$steps" | sed "s/$/  # $name/"
}

# ---- {RLPD} staged: 100k decisions, ckpt_040 = 40k, ckpt_100 = 100k -----------------
for S in 940 941; do sub_rl dH  dH       dHfull_all_rz       staged $S 100000 0.4,1.0; done
for S in 960 961; do sub_rl dM  dDPfirst dDPfull_first_rz    staged $S 100000 0.4,1.0; done
# ---- {RLPD} sparse: 250k decisions, ckpt_016 = 40k, ckpt_040 = 100k, ckpt_100 = 250k -
for S in 945 946; do sub_rl dH  dH       dHfull_all_rs       sparse $S 250000 0.16,0.4,1.0; done
for S in 965 966; do sub_rl dM  dDPfirst dDPfull_first_rs    sparse $S 250000 0.16,0.4,1.0; done
# ---- {r2dreamer} staged: 1M ONLINE steps, milestones 0.5M / 1M ----------------------
for S in 940 941; do sub_r2 dH dHfull_all_rz    staged $S 1000000 '[500000,1000000]'; done
for S in 960 961; do sub_r2 dM dDPfull_first_rz staged $S 1000000 '[500000,1000000]'; done
# ---- {r2dreamer} sparse: 4M ONLINE steps, milestones 0.5M / 1M / 2M / 4M ------------
for S in 945 946; do sub_r2 dH dHfull_all_rs    sparse $S 4000000 '[500000,1000000,2000000,4000000]'; done
for S in 965 966; do sub_r2 dM dDPfull_first_rs sparse $S 4000000 '[500000,1000000,2000000,4000000]'; done
