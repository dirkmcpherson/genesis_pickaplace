#!/bin/bash
#SBATCH -J e2e_annot
#SBATCH -p batch,preempt
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=24g
#SBATCH -t 02:00:00
#SBATCH --output=/cluster/tufts/shortlab/jstale02/annot_%j.out
set -u
L=/cluster/tufts/shortlab/jstale02
E=$L/gp_e2e
OUT=$L/rlpd_annot
PY=$L/condaenv/genesis/bin/python
cd $E
mkdir -p $OUT
# MUST match the ladder the runs were trained and scored under: full_env gates which stages
# enter _granted on `for stage, r in STAGE_REWARD.items()`, so the old ladder never grants
# placed_v2 and the overlay would show a different sticky set than the cells of record.
export FULLENV_REWARD_X=1
echo "[gate] FULLENV_REWARD_X=$FULLENV_REWARD_X"
run () {  # arm_label run_dir ic_index tag
  local lbl=$1 run=$2 ic=$3 tag=$4
  local ck=baselines/rl/checkpoints/e2e/$run/rlpd_final.zip
  if [ ! -f "$ck" ]; then echo "MISSING checkpoint $ck"; return; fi
  echo "=== $lbl $run ic=$ic ($tag) ==="
  $PY baselines/eval_e2e_annot.py --kind sac --checkpoint "$ck" \
      --ic-file baselines/eval_ics.json --ic-set rnd --mode mode --seed 0 \
      --max-steps 1200 --ic-index "$ic" --video \
      --out "$OUT/${lbl}_${run}_ic${ic}_${tag}" 2>&1 | tail -4
}
run human   e2e_rlpd_dH_s901       17 slide
run human   e2e_rlpd_dH_s900        0 nested_proxy
run machine e2e_rlpd_dDPfirst_s920  0 nested_proxy
run human   e2e_rlpd_dH_s900       10 tipped
run machine e2e_rlpd_dDPfirst_s920 19 tipped
run human   e2e_rlpd_dH_s900       13 timeout
run machine e2e_rlpd_dDPfirst_s920 13 timeout
echo "=== produced ==="
find $OUT -name '*.mp4' | sed "s|$OUT/||" | sort
