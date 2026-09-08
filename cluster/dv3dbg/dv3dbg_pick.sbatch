#!/usr/bin/env bash
# DV3 DEBUG ladder G2/G3 (paper/DV3_DEBUG_2026-09-05.md): PICK scope with the native stride-4 STATE demos
# ($W/demos_state/<set>), dreamerv3-torch tree $W/dv3_dbg (= dv3_fix + the state-only demo-image placeholder fix).
# Same stack as the stage-1b reach launcher minus the touchgoal/reach_goal overlays; the msrecipe overlay sets
# scope pick, delta_joint 0.025 / leash 5, action_repeat 4, batch 16x64, prefill 2500, pretrain 100; dv3std sets
# reward_scale 1 / train_ratio 512; --time_limit 1200 = 300 decisions. dreamer.py asserts the demo dir's repeat.json
# (action_repeat 4, terminal_reward 1.0 == genesis_reward_scale 1.0) before training.
# usage: TAG=<tag> sbatch dv3dbg_pick.sbatch <set:dHv2raw|dDP> <seed> <env_steps> [dreamer flags...]
#   SMOKE=1  -> CPU partition, --device cpu --compile False, tiny budget, eval skipped (pipeline check only)
#SBATCH -J dv3dbg_pick
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
SET=$1; SEED=$2; STEPS=$3; shift 3
: "${TAG:?TAG required (run tag, e.g. clamp1_fp32)}"
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
TREE=$W/dv3_dbg
PY=$LAB/condaenv/genesis/bin/python
DEMODIR=$W/demos_state/$SET
NAME=dv3dbg_pick_${SET}_${TAG}_s${SEED}${SMOKE:+_smoke}
LOGDIR=$W/runs/$NAME
RESTARTS=${SLURM_RESTART_COUNT:-0}
if [ -e "$LOGDIR" ] && [ "$RESTARTS" -eq 0 ]; then echo "FATAL: $LOGDIR exists"; exit 2; fi
[ -f "$DEMODIR/repeat.json" ] || { echo "FATAL: $DEMODIR has no repeat.json"; exit 2; }
# disk guard (2026-09-07 incident: /cluster/tufts/shortlab hit 100% and killed every job on the cluster).
# Standing rule: never start when the project filesystem has < 150 GB free. dv3 keeps only latest.pt (overwritten),
# so a run costs ~250 MB total (203 MB checkpoint + ~37 MB train_eps) and no periodic checkpoints accumulate.
FREE_G=$(df -BG --output=avail /cluster/tufts/shortlab | tail -1 | tr -dc '0-9')
echo "# free space on /cluster/tufts/shortlab: ${FREE_G}G"
[ "$FREE_G" -ge 150 ] || { echo "FATAL: only ${FREE_G}G free (< 150G) — not starting"; exit 5; }
export GENESIS_PICKAPLACE_ROOT=$W/gp_root MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
cd $TREE
echo "# $(date -Is) host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo none) tree=$TREE variant=$GENESIS_SIM_VARIANT demodir=$DEMODIR job=${SLURM_JOB_ID:-local} restarts=$RESTARTS"
grep -q 'GENESIS_SIM_VARIANT' envs/genesis.py || { echo "FATAL: tree lacks the sim-variant hook"; exit 2; }
grep -q 'state-only\] demo image placeholder' dreamer.py || { echo "FATAL: tree lacks the state-only demo image fix"; exit 2; }
python3 -c "import json,sys; m=json.load(open(sys.argv[1])); assert m['sim_variant']=='gc_kp4_riser3_shelf6' and m['action_repeat']==4 and m['with_state'] and float(m['terminal_reward'])==1.0 and m['scope']=='pick', m; print('# demo gate OK:', {k:m[k] for k in ('sim_variant','action_repeat','with_state','terminal_reward','scope','n_written','decisions_median')})" $DEMODIR/repeat.json
"$PY" -c 'import genesis, torch, ruamel.yaml; print("genesis", genesis.__version__, "torch", torch.__version__)'
mkdir -p "$LOGDIR"
[ "$RESTARTS" -gt 0 ] && echo "# REQUEUED restart #$RESTARTS — dv3 re-enters the existing subdir if latest.pt is present"
CFGS="genesis_pickplace genesis_pick_msrecipe genesis_dv3std usestateonly nowandb ${DV3_EXTRA_CONFIGS:-}"
EXTRA=()
[ -n "${SMOKE:-}" ] && EXTRA=(--device cpu --compile False --precision 32 --log_every 200 --prefill 800 --pretrain 5)
CMD=("$PY" dreamer.py --configs $CFGS --time_limit 1200 --steps $STEPS --seed $SEED --logdir $LOGDIR --demodir $DEMODIR --video_pred_log False "${EXTRA[@]}" "$@")
printf '%q ' "${CMD[@]}"; echo
printf '%s %q ' "$(date -Is)" "${CMD[@]}" >> $W/COMMANDS.log; echo >> $W/COMMANDS.log
"${CMD[@]}"
echo "# train rc=$? $(date -Is)"
SLURM_OUT=$W/slurm/${SLURM_JOB_NAME}_${SLURM_JOB_ID}.out
n=$(grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' $SLURM_OUT || true); echo "# [sim-variant] lines: $n"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] gc_kp4_riser3_shelf6 line"; exit 3; }
grep -q 'Total reward of demos: 66\|Total reward of demos: 58' $SLURM_OUT || { echo "FATAL: demo reward line missing/unexpected"; grep 'Total reward of demos' $SLURM_OUT || true; exit 3; }
[ -n "${SMOKE:-}" ] && { echo "# SMOKE DONE $NAME (no eval)"; exit 0; }
SUB=$(dirname "$(ls -t $LOGDIR/*/latest.pt | head -1)")
LAST=$(python3 -c "import json,sys; print(max(json.loads(l)['step'] for l in open(sys.argv[1])))" $SUB/metrics.jsonl)
echo "# last logged step $LAST of $STEPS"
python3 -c "import sys; sys.exit(0 if int(sys.argv[1]) >= 0.95*int(sys.argv[2]) else 4)" "$LAST" "$STEPS" || { echo "TRAIN-INCOMPLETE: $LAST < 0.95*$STEPS — no eval"; exit 4; }
# FRESH-process evals (deterministic actor), END checkpoint, 1200 sim steps: hold15 then rnd30
for cell in "hold 15" "rnd 30"; do set -- $cell
  EV=("$PY" genesis_eval.py --logdir $SUB --configs $CFGS --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $1 --episodes $2 --max-steps 1200 --seed 0 --no-wandb --out $LOGDIR/fresh_eval_$1$2)
  printf '%q ' "${EV[@]}"; echo
  "${EV[@]}" || echo "# eval $1 rc=$?"
done
echo "# evals done $(date -Is) DONE $NAME"
