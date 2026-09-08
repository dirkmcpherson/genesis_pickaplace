#!/usr/bin/env bash
# DV3 DEBUG: MODE-action cells for every dv3dbg pick run, in ONE job (each eval is 2-7 min; keeps the >=4-jobs-in-flight
# rule trivially satisfied). dv3's evaluator defaults to eval_action_mode 'sample' (dreamer.py _policy), so the cells
# written by the training launcher and by dv3dbg_eval.sbatch are SAMPLED-action cells despite the per-episode sidecar's
# act_selection='deterministic' stamp (a known mislabel, DV3_DIAGNOSIS 2026-08-28 §1). This job adds the mode cells so
# the dv3 row can be reported in both modes like the r2dreamer row.
# usage: sbatch dv3dbg_modesweep.sbatch <run_name>[,<run_name>...]   (subdir auto-detected: newest with latest.pt)
#SBATCH -J dv3dbg_modesweep
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --gres=gpu:1
#SBATCH --constraint=l40s|a100|l40|h200
#SBATCH --exclude=pax077
#SBATCH -n 8
#SBATCH --mem=32g
#SBATCH -t 0-08:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
export TORCHINDUCTOR_CACHE_DIR=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/inductor_cache/${SLURM_JOB_ID}
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/condaenv/genesis/bin/python
export GENESIS_PICKAPLACE_ROOT=$W/gp_root MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
CFGS="genesis_pickplace genesis_pick_msrecipe genesis_dv3std usestateonly nowandb genesis_dv3dbg_evalmode"
cd $W/dv3_dbg
grep -q 'genesis_dv3dbg_evalmode' configs.yaml || { echo "FATAL: tree lacks the genesis_dv3dbg_evalmode block"; exit 2; }
IFS=',' read -ra RUNS <<< "$1"
for RUN in "${RUNS[@]}"; do
  LOGDIR=$W/runs/$RUN
  SUB=$(dirname "$(ls -t $LOGDIR/*/latest.pt | head -1)")
  echo "# $(date -Is) $RUN sub=$(basename $SUB) sha=$(sha256sum $SUB/latest.pt | cut -c1-16)"
  for cell in "hold 15" "rnd 30"; do set -- $cell
    "$PY" genesis_eval.py --logdir $SUB --configs $CFGS \
      --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $1 --episodes $2 \
      --max-steps 1200 --seed 0 --no-wandb --out $LOGDIR/fresh_eval_$1$2_mode || echo "# eval $RUN $1 rc=$?"
  done
done
echo "# DONE $(date -Is) mode sweep: ${RUNS[*]}"
