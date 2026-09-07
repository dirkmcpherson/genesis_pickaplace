#!/usr/bin/env bash
# DV3 DEBUG: fresh-process eval of ONE checkpoint dir of a dv3dbg run (deterministic actor, dv3 genesis_eval.py).
# Unlike wmfix_eval_dv3.sbatch (which picks the newest latest.pt), the checkpoint SUBDIR is explicit, so a copied
# mid-run checkpoint (e.g. <run>/ckpt500k/{latest.pt,run_config.json}) can be scored as a disclosed early read.
# usage: sbatch dv3dbg_eval.sbatch <run_name> <subdir> <hold|rnd|sel|all> <episodes> [out_suffix]
#SBATCH -J dv3dbg_eval
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --gres=gpu:1
#SBATCH --constraint=l40s|a100|l40|h200
#SBATCH --exclude=pax077
#SBATCH -n 8
#SBATCH --mem=32g
#SBATCH -t 0-04:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
export TORCHINDUCTOR_CACHE_DIR=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/inductor_cache/${SLURM_JOB_ID}
RUN=$1; SUBNAME=$2; SET=$3; N=$4; SUF=${5:-}
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/condaenv/genesis/bin/python
LOGDIR=$W/runs/$RUN; SUB=$LOGDIR/$SUBNAME
[ -f "$SUB/latest.pt" ] || { echo "FATAL: $SUB/latest.pt missing"; exit 2; }
[ -f "$SUB/run_config.json" ] || { echo "FATAL: $SUB/run_config.json missing"; exit 2; }
export GENESIS_PICKAPLACE_ROOT=$W/gp_root MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
CFGS="genesis_pickplace genesis_pick_msrecipe genesis_dv3std usestateonly nowandb"
cd $W/dv3_dbg
echo "# $(date -Is) eval $RUN sub=$SUBNAME set=$SET n=$N ckpt_mtime=$(stat -c %y $SUB/latest.pt) sha=$(sha256sum $SUB/latest.pt | cut -c1-16)"
OUT=$LOGDIR/fresh_eval_${SET}${N}${SUF:+_$SUF}
"$PY" genesis_eval.py --logdir $SUB --configs $CFGS $([ "$SET" = all ] && echo "--ic-set all" || echo "--ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $SET") --episodes $N --max-steps 1200 --seed 0 --no-wandb --out $OUT
echo "# eval rc=$? $(date -Is) DONE $RUN $SUBNAME $SET -> $OUT"
