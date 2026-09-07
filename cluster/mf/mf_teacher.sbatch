#!/usr/bin/env bash
# Machine-first arm, teacher T1 = REWARD-ONLY r2dreamer (no demo prefill) + pick-scope potential shaping.
# paper/MACHINE_FIRST_PLAN_2026-09-07.md §1 T1 + amendment (a). Copy of wmfix_s2.sbatch (the WM launcher of
# record) with the demo-dir gate REMOVED and env.demo_dir left unset (null in genesis_pick_state.yaml ->
# demo_prefill is a no-op); the caller passes +env.pick_shaping=true env.return_clamp=1.6714 and the
# recipe overrides. Run dir $W/runs/mf_r2d_t1_s<seed>; in-job fresh-process evals (EVAL_SETS default hold rnd).
# usage: sbatch -p gpu mf_teacher.sbatch <seed> <steps> env.pick_shaping=true env.return_clamp=1.6714 \
#          env.actor_dist=bounded_normal env.act_entropy=3e-5
#SBATCH -J mf_r2d_t1
#SBATCH -p gpu
#SBATCH --gres=gpu:1
#SBATCH --requeue          # preempt partition: requeue instead of dropping (2026-09-04); a requeued run restarts CLEAN (below)
#SBATCH --exclude=pax077   # Tesla P100 (CUDA cap 6.0): torch.compile/Triton refuses it (2026-09-04)
#SBATCH --constraint=l40s|a100|l40|h200
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 2-00:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
# per-job torch.compile cache: two jobs compiling on one node with a shared inductor cache crash (InductorError SubprocException, 09-03)
export TORCHINDUCTOR_CACHE_DIR=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/inductor_cache/${SLURM_JOB_ID:-local}
SEED=$1; STEPS=$2; shift 2
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
TREE=$W/r2dreamer_fix
PY=$LAB/r2d_venv/bin/python
NAME=mf_r2d_t1${TAG:+_$TAG}_s${SEED}
LOGDIR=$W/runs/$NAME
[ -e "$LOGDIR" ] && { echo "FATAL: $LOGDIR exists"; exit 2; }
export GENESIS_PICKAPLACE_ROOT=$LAB/genesis_pickaplace MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
cd $TREE
echo "# $(date -Is) host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) tree=$TREE variant=$R2D_SIM_VARIANT demo=NONE (reward-only teacher)"
grep -q 'state_obs' envs/genesis.py demo_prefill.py configs/env/genesis_pick_state.yaml || { echo "FATAL: tree lacks the state patches"; exit 2; }
# NO demo gate: reward-only teacher, env.demo_dir stays null (no prefill)
case " $* " in *' env.demo_dir='*) echo 'FATAL: env.demo_dir must stay unset for the reward-only teacher'; exit 2;; esac
case " $* " in *' +env.pick_shaping=true '*) ;; *) echo 'FATAL: pass +env.pick_shaping=true (amendment (a); the + adds the key to the struct config)'; exit 2;; esac
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$LOGDIR" ]; then echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial logdir and starting clean"; rm -rf "$LOGDIR"; fi
mkdir -p "$LOGDIR"
# RESUME_FROM=<latest.pt> (optional): warm restart from that checkpoint (weights+optims; buffer restarts at prefill,
# step counter re-runs env.steps) -- a DISCLOSED continuation, never a run of record without saying so.
RESUME=""
if [ -n "${RESUME_FROM:-}" ]; then cp "$RESUME_FROM" "$LOGDIR/latest.pt"; RESUME="+resume=true"; echo "# RESUME_FROM=$RESUME_FROM (saved step $($PY -c 'import torch,sys; print(torch.load(sys.argv[1],map_location="cpu",weights_only=False).get("step"))' $LOGDIR/latest.pt))"; fi
CMD=("$PY" train.py env=${ENVCFG:-genesis_pick_state} seed=$SEED env.steps=$STEPS buffer.max_size=5e5 logdir=$LOGDIR $RESUME "$@")
printf '%q ' "${CMD[@]}"; echo
printf '%s %q ' "$(date -Is)" "${CMD[@]}" >> $W/runs/COMMANDS.log; echo >> $W/runs/COMMANDS.log
"${CMD[@]}"
LAST=$(tail -c 4000 $LOGDIR/metrics.jsonl 2>/dev/null | grep -oE "\"step\": [0-9]+" | tail -1 | grep -oE "[0-9]+" || echo 0)
[ "${LAST:-0}" -ge $((STEPS - 5000)) ] || { echo "FATAL: last logged step $LAST < $((STEPS - 5000)) -- training did not reach its budget; no evaluation"; exit 1; }
echo "# train rc=$? $(date -Is)"
SLURM_OUT=$W/slurm/${SLURM_JOB_NAME}_${SLURM_JOB_ID}.out
n=$(grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' $SLURM_OUT || true); echo "# [sim-variant] lines: $n"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] line"; exit 3; }
grep -q 'Demo prefill:' $SLURM_OUT && { echo 'FATAL: a demo prefill ran in the reward-only teacher'; exit 3; }
# shaping proof: the run's resolved hydra config + shaped (non-integer) episode rewards
grep -E '^\s*(pick_shaping|return_clamp|demo_dir|actor_dist|act_entropy):' $LOGDIR/.hydra/config.yaml | sed 's/^/# cfg /'
"$PY" - $LOGDIR/metrics.jsonl <<'PYS'
import json, sys
rs, sc = [], []
for l in open(sys.argv[1]):
    try: d = json.loads(l)
    except Exception: continue
    if d.get('train/data/reward_sum') is not None: rs.append(float(d['train/data/reward_sum']))
    if d.get('episode/score') is not None: sc.append(float(d['episode/score']))
def rep(name, v):
    if not v: return f"{name}: 0 samples"
    return f"{name}: {len(v)} samples, non-integer {sum(abs(x-round(x))>1e-6 for x in v)}, min {min(v):.3f} max {max(v):.3f}"
print("# shaping check:", rep('reward_sum', rs), '|', rep('episode/score', sc))
allv = rs + sc
if len(allv) >= 20 and not any(abs(x-round(x)) > 1e-6 for x in allv):
    print("FATAL: every logged reward/score is an integer -- shaping is NOT active"); sys.exit(3)
PYS
# FRESH-process evals (sample mode, protocol horizon): 15 hold + 30 rnd (CONFOUNDS row 22: all episodes present)
for MODE in sample mode; do for SET in ${EVAL_SETS:-hold rnd}; do   # EVAL_SETS override (repeat-1 pilot: "hold rnd", the 74-IC set at 1200 decisions/episode is too slow in-job)   # sample = registered statistic; mode = DP/RLPD-like deterministic counterpart (disclosed); all = FULL demo IC set (user, 2026-09-03)
  N=15; [ $SET = rnd ] && N=30; [ $SET = all ] && N=1   # N ignored for 'all' (evaluator uses every success uid once)
  if [ $SET = all ]; then
    EV=("$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes 1 --mode $MODE --max-steps 1200 \
        --ic-set all --seed 0 --out $LOGDIR/fresh_eval_alldemo_${MODE} --device cpu)
  else
    EV=("$PY" eval_genesis.py --checkpoint $LOGDIR/latest.pt --episodes $N --mode $MODE --max-steps 1200 \
        --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set $SET --seed 0 --out $LOGDIR/fresh_eval_${SET}${N}_${MODE} --device cpu)
  fi
  printf '%q ' "${EV[@]}"; echo
  "${EV[@]}"
  echo "# eval $SET $MODE rc=$? $(date -Is)"
done; done
echo "# DONE $NAME $(date -Is)"
