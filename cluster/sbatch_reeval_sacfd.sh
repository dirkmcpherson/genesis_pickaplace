#!/bin/bash
# POST-HOC re-eval of ALL SACfD checkpoints under the HARDENED pick predicate
# (aa762ac): the old z+grip predicate counted airborne flings; every SACfD
# official number (0.07-0.60) was measured by it. BC rows are certified
# unaffected (DP control 0.67 unchanged); these runs re-measure RLfD honestly.
# New wandb runs get "-hardened" names; originals stay for the record.
#
#   sbatch cluster/sbatch_reeval_sacfd.sh          # after git pull

#SBATCH -J reeval-sacfd
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-16:00:00
#SBATCH --output=reeval_sacfd_%j.out
#SBATCH --error=reeval_sacfd_%j.out

set -o pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
export MUJOCO_GL=egl PYTHONUNBUFFERED=1
SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH="$(ls -d "$SITE"/nvidia/*/lib 2>/dev/null | tr '\n' ':')${LD_LIBRARY_PATH:-}"

# hardened-predicate guard: refuse to run on pre-fix code
grep -q "sustain" baselines/genesis_can_env.py || { echo "FATAL: predicate fix not present -- git pull"; exit 1; }

FAIL=0; DONE=0
for COND in dH_SACfD dDP_SACfD; do
  for S in 0 1 2 3 4 5 6 7; do
    CK=baselines/outputs/paper/${COND}_s${S}/sacfd_final.zip
    [ -f "$CK" ] || { echo "== SKIP ${COND}_s$S (no checkpoint)"; continue; }
    echo "== re-eval ${COND}_s$S $(date)"
    python baselines/wandb_eval.py --kind sac --ic-mode both \
      --checkpoint "$CK" --random 15 --seed 0 \
      --project genesis_paper --group "${COND}-hardened" \
      --name "${COND}_s${S}-hardened-eval" \
      && DONE=$((DONE+1)) || { echo "== FAILED ${COND}_s$S"; FAIL=$((FAIL+1)); }
  done
done
echo "== reeval done: $DONE ok, $FAIL failed $(date)"
[ "$FAIL" -eq 0 ]
