#!/usr/bin/env bash
# DV3 DEBUG ladder (paper/DV3_DEBUG_2026-09-05.md) — reach_goal proxy, STATE input, no demos, dreamerv3-torch tree
# $W/dv3_fix (unchanged; levers are CLI flags). Derived from wmfix_s1b_dv3.sbatch; differences: dv3dbg_ names,
# preempt-safe (--requeue + dv3's own logdir re-entry), pax077 excluded, 2-day limit, eval refused unless training
# reached its budget (an untrained checkpoint must never be evaluated as if complete).
# usage: TAG=<tag> sbatch dv3dbg_s1b.sbatch <seed> <env_steps> <reach_goal_dist_m> [dreamer flags...]
#        env: DV3_TREE=<tree dir> (default $W/dv3_fix), DV3_EXTRA_CONFIGS="<config blocks>" spliced into --configs
#SBATCH -J dv3dbg_s1b
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint=l40s|a100|l40|h200
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 2-00:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
export TORCHINDUCTOR_CACHE_DIR=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/inductor_cache/${SLURM_JOB_ID:-local}
SEED=$1; STEPS=$2; RGD=$3; shift 3
: "${TAG:?TAG required (run tag, e.g. clamp1)}"
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
TREE=${DV3_TREE:-$W/dv3_fix}              # round 1: dv3_fix (frozen); later rounds may use $W/dv3_dbg (= dv3_fix + G2 demo hunk + config blocks)
PY=$LAB/condaenv/genesis/bin/python
NAME=dv3dbg_reach_r${RGD}_${TAG}_s${SEED}
LOGDIR=$W/runs/$NAME
RESTARTS=${SLURM_RESTART_COUNT:-0}
if [ -e "$LOGDIR" ] && [ "$RESTARTS" -eq 0 ]; then echo "FATAL: $LOGDIR exists"; exit 2; fi
export REACH_GOAL_DIST=$RGD GENESIS_PICKAPLACE_ROOT=$W/gp_root MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
cd $TREE
echo "# $(date -Is) host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) tree=$TREE variant=$GENESIS_SIM_VARIANT job=$SLURM_JOB_ID restarts=$RESTARTS"
grep -q 'GENESIS_SIM_VARIANT' envs/genesis.py || { echo "FATAL: tree lacks the sim-variant hook"; exit 2; }
"$PY" -c 'import genesis, torch, ruamel.yaml; print("genesis", genesis.__version__, "torch", torch.__version__)'
mkdir -p "$LOGDIR"
[ "$RESTARTS" -gt 0 ] && echo "# REQUEUED restart #$RESTARTS — dv3 re-enters the existing subdir if latest.pt is present"
CFGS="genesis_pickplace genesis_pick_msrecipe genesis_touchgoal genesis_reach_goal genesis_dv3std usestateonly nowandb ${DV3_EXTRA_CONFIGS:-}"
CMD=("$PY" dreamer.py --configs $CFGS --time_limit 1200 --steps $STEPS --seed $SEED --logdir $LOGDIR --video_pred_log False "$@")
printf '%q ' "${CMD[@]}"; echo
printf '%s %q ' "$(date -Is)" "${CMD[@]}" >> $W/COMMANDS.log; echo >> $W/COMMANDS.log
"${CMD[@]}"
echo "# train rc=$? $(date -Is)"
SLURM_OUT=$W/slurm/${SLURM_JOB_NAME}_${SLURM_JOB_ID}.out
n=$(grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' $SLURM_OUT || true); echo "# [sim-variant] lines: $n"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] gc_kp4_riser3_shelf6 line"; exit 3; }
SUB=$(dirname "$(ls -t $LOGDIR/*/latest.pt | head -1)")
# budget gate: last logged env step must reach 95% of the budget, else no eval (exit 4, requeue-able by hand)
LAST=$(python3 -c "import json,sys; print(max(json.loads(l)['step'] for l in open(sys.argv[1])))" $SUB/metrics.jsonl)
echo "# last logged step $LAST of $STEPS"
python3 -c "import sys; sys.exit(0 if int(sys.argv[1]) >= 0.95*int(sys.argv[2]) else 4)" "$LAST" "$STEPS" || { echo "TRAIN-INCOMPLETE: $LAST < 0.95*$STEPS — no eval"; exit 4; }
# FRESH-process eval (dv3 evaluator: deterministic actor), 15 hold ICs, 1200 sim steps, END checkpoint
EV=("$PY" genesis_eval.py --logdir $SUB --configs $CFGS \
    --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --episodes 15 --max-steps 1200 --seed 0 --no-wandb --out $LOGDIR/fresh_eval_hold15)
printf '%q ' "${EV[@]}"; echo
"${EV[@]}"
echo "# eval rc=$? $(date -Is) DONE $NAME"
