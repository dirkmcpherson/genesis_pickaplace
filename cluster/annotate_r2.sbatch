#!/bin/bash
#SBATCH -J r2_annot
#SBATCH -p batch,preempt
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=24g
#SBATCH -t 03:00:00
#SBATCH --output=/cluster/tufts/shortlab/jstale02/r2annot_%j.out
set -u
L=/cluster/tufts/shortlab/jstale02
W=$L/wm_fix_2026-09-03
T=$W/r2dreamer_fix
OUT=$L/r2_annot
PY=$L/r2d_venv/bin/python
export GENESIS_PICKAPLACE_ROOT=$W/gp_root MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export FULLENV_REWARD_X=1
echo "[gate] FULLENV_REWARD_X=$FULLENV_REWARD_X ROOT=$GENESIS_PICKAPLACE_ROOT"
cd $T
rm -rf $OUT; mkdir -p $OUT
run () {   # label run_dir target_ep tag
  local lbl=$1 run=$2 ep=$3 tag=$4
  local ck=$W/runs/$run/latest.pt
  [ -f "$ck" ] || { echo "MISSING $ck"; return; }
  local n=$((ep+1)) skip=""
  if [ "$ep" -gt 0 ]; then skip="--ic-skip $(seq 0 $((ep-1)) | tr '\n' ' ')"; fi
  echo "=== $lbl $run ep=$ep ($tag) ==="
  $PY eval_genesis_annot.py --checkpoint "$ck" --episodes "$n" --mode mode --max-steps 1200 \
      --ic-file $L/genesis_pickaplace/baselines/eval_ics.json --ic-set rnd --seed 0 \
      --scale 6 --device cpu --out "$OUT/${lbl}_${run}_ep${ep}_${tag}" $skip 2>&1 | tail -3
}
run human   full_r2d_state_dHfull_all_rx_s903      0  push_nestedHONEST_labelledTIMEOUT
run machine full_r2d_state_dDPfull_first_rx_s921  23  push_nestedHONEST
run human   full_r2d_state_dHfull_all_rx_s901     18  push_proxyONLY
run machine full_r2d_state_dDPfull_first_rx_s926  11  push_proxyONLY
echo "=== produced ==="; find $OUT -name '*.mp4' ! -name 'rollouts_grid.mp4' | sed "s|$OUT/||" | sort
