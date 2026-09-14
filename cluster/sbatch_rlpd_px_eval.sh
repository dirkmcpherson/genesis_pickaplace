#!/bin/bash
# Pinned CPU evaluation of ONE {RLPD} PIXEL checkpoint with baselines/eval_e2e_px.py (lane PXR-1, 2026-09-13).
# Submitted by cluster/sbatch_rlpd_px.sh after TRAIN-OK (with --exclude = every non-64-physical-core node + the SMT
# nodes, the milestone sweep's lists) and usable by hand for a re-score:
#   env GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr KIND=sac CKPT=<run>/rlpd_final.zip OUT=<run> ARM=dH SEED=0 \
#       [SETS="hold15 rnd30"] [MODES="sample mode"] [ISO=1] [ISO_SETS=rnd30] [VIDEO_SETS=rnd30] [LIMIT=n] [PAR=8] \
#       [REQUIRE_CORES=64] [THREADS=] [ROLE=preview] [CELL_DIR=] \
#       sbatch -J e2e_rlpd_px_eval_dH_s0 --exclude=<non-64-core + SMT nodes> cluster/sbatch_rlpd_px_eval.sh
# It is cluster/e2e_eval_cells.sh with EVAL_SCRIPT=baselines/eval_e2e_px.py: the same cells, the same file names
# (fresh_eval_<set>_<mode>[_iso]/metrics.json), the same E2E-HEADLINE line, the pixel observation function.
# REQUIRE_CORES=64 is the guard of record (eval_e2e_px.py refuses to produce a cell on another machine size);
# ROLE stays 'preview' unless the caller pins THREADS too and asks for 'record' (eval_e2e.py's rule, unchanged).
#SBATCH -J e2e_rlpd_px_eval
#SBATCH -p batch
#SBATCH --qos=normal
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=24g
#SBATCH --time=12:00:00
#SBATCH --output=e2e_rlpd_px_eval_%j.out
#SBATCH --error=e2e_rlpd_px_eval_%j.out
set -eo pipefail
: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT}"; : "${CKPT:?CKPT}"; : "${OUT:?OUT}"; : "${ARM:?ARM}"; : "${SEED:?SEED}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/eval_e2e_px.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT has no eval_e2e_px.py"; exit 1; }
cd "$GENESIS_PICKAPLACE_ROOT"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
[ -f "$CKPT" ] && [ -f "${CKPT%.zip}.action_mode.json" ] || { echo "FATAL: checkpoint or sidecar missing at $CKPT"; exit 1; }
python3 -c "import json,sys; sc=json.load(open('${CKPT%.zip}.action_mode.json')); assert sc.get('obs')=='pixels', sc.get('obs'); print('SIDECAR-OK obs=pixels image_aug=%s ladder=%s tip_guard=%s' % (sc.get('image_aug'), sc.get('ladder'), sc.get('tip_guard')))" || exit 1
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
echo "== RLPD-PX-EVAL $ARM s$SEED start $(date) node=$(hostname) cores=$(python3 -c "
import re; t=open('/proc/cpuinfo').read(); c=re.search(r'^cpu cores\s*:\s*(\d+)', t, re.M); s=len(set(re.findall(r'^physical id\s*:\s*(\d+)', t, re.M))) or 1
print(int(c.group(1))*s if c else '?')") ckpt=$CKPT"
echo "== TREE $GENESIS_PICKAPLACE_ROOT ($(git -C "$GENESIS_PICKAPLACE_ROOT" describe --always --dirty 2>/dev/null || echo 'no git'))"
mkdir -p "$OUT"
set +e   # the cells driver reports per cell; a failed cell must not hide the others
KIND=sac EVAL_SCRIPT=baselines/eval_e2e_px.py CKPT="$CKPT" OUT="$OUT" ARM="$ARM" SEED="$SEED" \
  SIM_VARIANT="${SIM_VARIANT:-gc_kp4_riser3_shelf6}" SETS="${SETS:-hold15 rnd30}" MODES="${MODES:-sample mode}" \
  ISO="${ISO:-1}" ISO_SETS="${ISO_SETS:-rnd30}" VIDEO_SETS="${VIDEO_SETS:-rnd30}" LIMIT="${LIMIT:-}" PAR="${PAR:-8}" \
  REQUIRE_CORES="${REQUIRE_CORES:-64}" THREADS="${THREADS:-}" ROLE="${ROLE:-preview}" CELL_DIR="${CELL_DIR:-}" \
  bash cluster/e2e_eval_cells.sh 2>&1 | tee "$OUT/e2e_eval_px.log"
RC=${PIPESTATUS[0]}
for D in "$OUT"/fresh_eval_*; do [ -f "$D/eval.log" ] && grep -E "^\[eval-e2e\] (PX|[0-9]+ episodes)|^\[sim-variant\]|^ep[0-9]+:|^\[ladder\]" "$D/eval.log" | sed "s|^|$(basename "$D"): |"; done
echo "EVAL DONE rc=$RC $(date)"
