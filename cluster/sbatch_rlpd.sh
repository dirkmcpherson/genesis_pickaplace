#!/bin/bash
# RLPD n=20 wave: one seed per GPU, env-var driven. Post-E3-demo-rng code
# (>= 2fbed2a REQUIRED — the demo buffer ignores --seed before that commit).
#
# --- Submit (from the genesis_pickaplace checkout root, AFTER git pull) -----------
#   for S in $(seq 0 19); do ARM=dH   SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#   for S in $(seq 0 19); do ARM=dR2D SEED=$S sbatch cluster/sbatch_rlpd.sh; done
#
# Env vars:
#   ARM    dH | dR2D   (required; picks demo dir + run name; provenance-checked)
#   SEED   0..19       (required)
#   STEPS  100000
#   CONDA_ENV  /cluster/tufts/shortlab/jstale02/condaenv/genesis
#
# DATA (rsync ONLY, never git — run these from the dev box once before submitting):
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_pick_phase_all/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_pick_phase_all/
#   rsync -av ~/workspace/genesis_pickaplace/baselines/episodes_champion_pick/ \
#     <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/episodes_champion_pick/
#
# Stages: train (100k, ~2-3h) -> §4a-2 sweep IN-JOB (15 demo-IC + 15 random-IC,
# one fresh process per episode, CPU) -> per-seed summary line in the .out.
# Same-machine rule: the arm CONTRAST is internal to the cluster/node class;
# do not compare absolute rates against local-box numbers without a caveat.

#SBATCH -J rlpd
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-08:00:00
#SBATCH --output=rlpd_%j.out
#SBATCH --error=rlpd_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export PYTHONUNBUFFERED=1

ARM=${ARM:?set ARM=dH or ARM=dR2D}
SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"

# sb3 is pip-only in this env; install once, guarded. NEVER conda install here.
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'

# git gate: demo-RNG fix must be present
git merge-base --is-ancestor 2fbed2a HEAD || {
  echo 'FATAL: checkout predates the demo-RNG fix (2fbed2a). git pull first.'; exit 1; }

case "$ARM" in
  dH)   DEMO=baselines/episodes_pick_phase_all;  PAT='^[0-9]{3}\.npz$' ;;
  dR2D) DEMO=baselines/episodes_champion_pick;   PAT='^1[0-9]{5}\.npz$' ;;
  *) echo "FATAL: ARM=$ARM"; exit 1 ;;
esac
# provenance gate (naming-trap rule): filename pattern must match the claimed source
ls "$DEMO" | grep -E '\.npz$' | head -5 | grep -qE "$PAT" || {
  echo "FATAL: $DEMO contents do not match $ARM provenance pattern $PAT"; exit 1; }

OUT=baselines/rl/checkpoints/rlpd_n20_${ARM}_s${SEED}
python baselines/rl/train_rlpd.py \
  --steps "$STEPS" --scope pick --action-mode delta_joint --delta-ref target \
  --action-repeat 1 --gamma 0.998 --backup-entropy off --per-member-ln off \
  --pick-hold-reward off --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128 \
  --demo-dir "$DEMO" --out-dir "$OUT" \
  --run-name "${ARM}_RLPD-n20_s${SEED}" --project genesis_paper \
  --seed "$SEED" --device cuda

# ---- §4a-2 sweep, in-job, CPU, fresh process per episode ------------------------
CK=$OUT/rlpd_100000_steps.zip
[ -f "$CK" ] || { echo "FATAL: no 100k checkpoint at $CK"; exit 1; }
SW=$OUT/sweep; mkdir -p "$SW"
UIDS="232 234 235 236 237 239 242 243 244 245 246 247 248 250 251"
for u in $UIDS; do
  while [ "$(jobs -rp | wc -l)" -ge 5 ]; do sleep 3; done
  ( CUDA_VISIBLE_DEVICES="" python baselines/wandb_eval.py --kind sac --checkpoint "$CK" \
      --action-mode delta_joint --action-repeat 1 --ic-mode demo --random 0 --reps 1 \
      --uids "$u" --max-steps 400 --record-dir "$SW/v_d_$u" \
      --json "$SW/d_$u.json" --no-wandb > "$SW/d_$u.log" 2>&1 ) &
done
for k in $(seq 0 14); do
  while [ "$(jobs -rp | wc -l)" -ge 5 ]; do sleep 3; done
  ( CUDA_VISIBLE_DEVICES="" python baselines/wandb_eval.py --kind sac --checkpoint "$CK" \
      --action-mode delta_joint --action-repeat 1 --ic-mode random --random 1 --seed "$k" \
      --max-steps 400 --record-dir "$SW/v_r_$k" \
      --json "$SW/r_$k.json" --no-wandb > "$SW/r_$k.log" 2>&1 ) &
done
wait

python - "$SW" "$ARM" "$SEED" <<'PYEOF'
import json, glob, sys
sw, arm, seed = sys.argv[1], sys.argv[2], sys.argv[3]
def count(pref):
    n = p = 0
    for f in sorted(glob.glob(f'{sw}/{pref}_*.json')):
        v = json.load(open(f))['metrics']['eval/picked']   # nested key — never top-level
        n += 1; p += (v > 0)
    return p, n
d, dn = count('d'); r, rn = count('r')
print(f'SWEEP-RESULT arm={arm} seed={seed} demoIC={d}/{dn} randomIC={r}/{rn}')
PYEOF
echo "JOB DONE $(date)"
