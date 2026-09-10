#!/bin/bash
#SBATCH -J e2e_push
#SBATCH -p batch,preempt
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=24g
#SBATCH -t 02:00:00
#SBATCH --output=/cluster/tufts/shortlab/jstale02/push_%j.out
set -u
L=/cluster/tufts/shortlab/jstale02
E=$L/gp_e2e
OUT=$L/rlpd_push
PY=$L/condaenv/genesis/bin/python
cd $E
rm -rf $OUT; mkdir -p $OUT
export FULLENV_REWARD_X=1
echo "[gate] FULLENV_REWARD_X=$FULLENV_REWARD_X"
run () {
  local lbl=$1 run=$2 ic=$3 tag=$4
  local ck=baselines/rl/checkpoints/e2e/$run/rlpd_final.zip
  [ -f "$ck" ] || { echo "MISSING $ck"; return; }
  echo "=== $lbl $run ic=$ic ($tag) ==="
  $PY baselines/eval_e2e_annot.py --kind sac --checkpoint "$ck" \
      --ic-file baselines/eval_ics.json --ic-set rnd --mode mode --seed 0 \
      --max-steps 1200 --ic-index "$ic" --video \
      --out "$OUT/${lbl}_${run}_ic${ic}_${tag}" 2>&1 | tail -3
}
run human   e2e_rlpd_dH_s901       29 push_nestedHONEST
run machine e2e_rlpd_dDPfirst_s924 18 push_nestedHONEST
run human   e2e_rlpd_dH_s900       18 push_proxyONLY
run machine e2e_rlpd_dDPfirst_s920 18 push_proxyONLY
echo "=== produced ==="; find $OUT -name '*.mp4' | sed "s|$OUT/||" | sort
