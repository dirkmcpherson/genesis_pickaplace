#!/bin/bash
# PAPER SMOKE job A: verified IMAGE harvest from the gen-0 joint DP teacher, then
# conversion to pick-scope DreamerV3 demos (demonstrations/genesis_m1_smoke).
#
# This is the M1-DV3 data path, which has NO end-to-end precedent: --images
# harvesting has never run at scale, and to_dreamer_demos has never consumed a
# harvest. It also delivers the pending July30th_Fable.md §1 check: the
# verify-rejection rate on a REAL teacher harvest (must stay ~<=5%).
#
# Submitted by cluster/launch_paper_smoke.sh (job B depends on this one). Env vars:
#   TEACHER_CKPT  (default: the ouro gen-0 joint DP)   SMOKE_KEPT (default 20)
#   DV3_DIR       (default: ../dreamerv3-torch)        MIN_KEPT   (gate, default 12)
#   HARV_OUT      (default paper_smoke/m1_harvest)     DEMO_NAME  (default genesis_m1_smoke)
# Reusable for the FULL-SIZE M1-DV3 harvest (matched N) by overriding, e.g.:
#   sbatch --time=0-12:00:00 --export=ALL,...,SMOKE_KEPT=66,MIN_KEPT=40,\
#     HARV_OUT=paper_smoke/m1_harvest_full,DEMO_NAME=genesis_m1_full  <this file>

#SBATCH -J paper-smoke-harvest
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
# Teacher inference wants a GPU; the harvest env itself is CPU-backend, so A100 is
# fine here (genesis-on-GPU is not involved).
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-06:00:00
#SBATCH --output=paper_smoke_harvest_%j.out
#SBATCH --error=paper_smoke_harvest_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT
DV3_DIR=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
TEACHER_CKPT=${TEACHER_CKPT:-ouroboros/ouro_dp_joint/gen0/dp/checkpoints/last/pretrained_model}
HARV_OUT=${HARV_OUT:-paper_smoke/m1_harvest}
DEMO_NAME=${DEMO_NAME:-genesis_m1_smoke}
MIN_KEPT=${MIN_KEPT:-12}
[ -e "$TEACHER_CKPT" ] || { echo "FATAL: teacher checkpoint missing: $TEACHER_CKPT"; exit 1; }   # -e: SAC ckpts are FILES (.zip)

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl PYTHONUNBUFFERED=1
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

echo "== paper smoke harvest start $(date)  teacher=$TEACHER_CKPT (${TEACHER_TYPE:-dp})  target=${SMOKE_KEPT:-20} -> $HARV_OUT"
python baselines/harvest_ai_demos.py --teacher-type "${TEACHER_TYPE:-dp}" \
  --checkpoint "$TEACHER_CKPT" --action-space joint \
  --ic-mode demo --scope pick --verify --images \
  --keep-fails "${KEEP_FAILS:-0}" \
  --n "${HARVEST_N:-200}" --target-kept "${SMOKE_KEPT:-20}" --cap "${CAP:-600}" --seed 7 \
  --outdir "$HARV_OUT"

echo "== harvest manifest:"
python - "$HARV_OUT/manifest.json" "$MIN_KEPT" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
min_kept = int(sys.argv[2])
print(json.dumps(m, indent=2))
kept = m.get('kept', 0)
rej = m.get('rejected_by_verify', m.get('n_reject_verify', 0))
att = kept + rej
if kept < min_kept:
    print(f'FATAL: only {kept} kept (< {min_kept}) -- teacher or harvest path broken'); sys.exit(1)
if att and 100.0 * rej / att > 5.0:
    print(f'FATAL: verify rejected {rej}/{att} ({100*rej/att:.0f}%) -- the July30th '
          f'section-1 gate. STOP and diagnose before trusting any generated dataset.')
    sys.exit(1)
print(f'== verify gate OK: {rej}/{att} rejected')
PY

echo "== converting to pick-scope dreamer demos -> $DV3_DIR/demonstrations/$DEMO_NAME"
python baselines/rl/to_dreamer_demos.py --scope pick \
  --src "$HARV_OUT" \
  --dst "$DV3_DIR/demonstrations/$DEMO_NAME"
python - "$DV3_DIR/demonstrations/$DEMO_NAME" "$MIN_KEPT" <<'PY'
import glob, sys
import numpy as np
fs = glob.glob(sys.argv[1] + '/*.npz')
rew = sum(1 for f in fs if (np.load(f)['reward'] > 0).any())
print(f'== {len(fs)} dreamer demo episodes, {rew} carry a pick grant')
# Every kept harvest episode ended on an env-granted pick; if the relabel proxy
# (can_z>PICK_Z & grip closed) disagrees with the env grant, episodes come out
# reward-free and the world model gets a demo set that never shows the reward.
if rew < int(sys.argv[2]):
    print(f'FATAL: <{sys.argv[2]} rewarded demos -- env pick grant vs relabel proxy mismatch?')
    sys.exit(1)
PY
echo "== paper smoke harvest DONE $(date)"
