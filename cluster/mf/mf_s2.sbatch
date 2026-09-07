#!/usr/bin/env bash
# Machine-first arm: the WM LEARNER of record on dRL = byte-for-byte wmfix_s2.sbatch (stage-2 pick, STATE input,
# sparse rewards, r2dreamer) except (1) the job name carries the mf_ prefix and (2) SLURM_OUT is derived from
# $SLURM_JOB_NAME instead of the hard-coded 'wmfix_s2' (else the [sim-variant] proof grep reads the wrong file).
# usage: TAG=bnormclamp1ent5 sbatch mf_s2.sbatch dRL <seed> 1000000 env.actor_dist=bounded_normal env.return_clamp=1.0 env.act_entropy=3e-5
#SBATCH -J mf_s2
#SBATCH -p gpu,preempt
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
ARM=$1; SEED=$2; STEPS=$3; shift 3
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
TREE=$W/r2dreamer_fix
PY=$LAB/r2d_venv/bin/python
DEMO=${DEMO_ROOT:-$W/demos_state}/$ARM   # DEMO_ROOT override: the repeat-1 pilot reads demos_state_r1/ (stride-1 tapes)
NAME=s2_r2d_pick_state_${ARM}${TAG:+_$TAG}_s${SEED}
LOGDIR=$W/runs/$NAME
[ -e "$LOGDIR" ] && { echo "FATAL: $LOGDIR exists"; exit 2; }
export GENESIS_PICKAPLACE_ROOT=$LAB/genesis_pickaplace MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
cd $TREE
echo "# $(date -Is) host=$(hostname) gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) tree=$TREE variant=$R2D_SIM_VARIANT demo=$DEMO"
grep -q 'state_obs' envs/genesis.py demo_prefill.py configs/env/genesis_pick_state.yaml || { echo "FATAL: tree lacks the state patches"; exit 2; }
# demo-dir provenance gate: world + stride + with_state + terminal reward must match this run
"$PY" - "$DEMO" "$R2D_SIM_VARIANT" <<'PYG'
import json, sys, glob, numpy as np
d, want = sys.argv[1], sys.argv[2]
m = json.load(open(f"{d}/repeat.json"))
assert m["sim_variant"] == want, (m["sim_variant"], want)
import os as _o; assert int(m["action_repeat"]) == int(_o.environ.get("REPEAT", "4")) and m.get("with_state") is True and abs(float(m["terminal_reward"]) - 1.0) < 1e-9, m
fs = sorted(glob.glob(f"{d}/*.npz")); assert len(fs) == int(m["n_written"]), (len(fs), m.get("n_written"))  # set-size agnostic (raw pair is N=66)
z = np.load(fs[0]); assert z["state"].shape == (len(z["reward"]), 17), z["state"].shape
print(f"[demo-gate] {d}: {len(fs)} tapes, variant {m['sim_variant']}, stride {m['action_repeat']} (expected {_o.environ.get('REPEAT', '4')}), with_state, terminal 1.0, total_reward {m['total_reward']}")
PYG
if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$LOGDIR" ]; then echo "# requeued (restart ${SLURM_RESTART_COUNT}): clearing the partial logdir and starting clean"; rm -rf "$LOGDIR"; fi
mkdir -p "$LOGDIR"
# RESUME_FROM=<latest.pt> (optional): warm restart from that checkpoint (weights+optims; buffer restarts at prefill,
# step counter re-runs env.steps) -- a DISCLOSED continuation, never a run of record without saying so.
RESUME=""
if [ -n "${RESUME_FROM:-}" ]; then cp "$RESUME_FROM" "$LOGDIR/latest.pt"; RESUME="+resume=true"; echo "# RESUME_FROM=$RESUME_FROM (saved step $($PY -c 'import torch,sys; print(torch.load(sys.argv[1],map_location="cpu",weights_only=False).get("step"))' $LOGDIR/latest.pt))"; fi
CMD=("$PY" train.py env=${ENVCFG:-genesis_pick_state} seed=$SEED env.steps=$STEPS env.demo_dir=$DEMO buffer.max_size=5e5 logdir=$LOGDIR $RESUME "$@")
printf '%q ' "${CMD[@]}"; echo
printf '%s %q ' "$(date -Is)" "${CMD[@]}" >> $W/runs/COMMANDS.log; echo >> $W/runs/COMMANDS.log
"${CMD[@]}"
LAST=$(tail -c 4000 $LOGDIR/metrics.jsonl 2>/dev/null | grep -oE "\"step\": [0-9]+" | tail -1 | grep -oE "[0-9]+" || echo 0)
[ "${LAST:-0}" -ge $((STEPS - 5000)) ] || { echo "FATAL: last logged step $LAST < $((STEPS - 5000)) -- training did not reach its budget; no evaluation"; exit 1; }
echo "# train rc=$? $(date -Is)"
SLURM_OUT=$W/slurm/${SLURM_JOB_NAME}_${SLURM_JOB_ID}.out
n=$(grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' $SLURM_OUT || true); echo "# [sim-variant] lines: $n"
[ "$n" -ge 1 ] || { echo "FATAL: no [sim-variant] line"; exit 3; }
grep -E 'Demo prefill:|Step accounting' $SLURM_OUT | head -3
# FRESH-process evals (sample mode, protocol horizon): 15 hold + 30 rnd (CONFOUNDS row 22: all episodes present)
for MODE in sample mode; do for SET in ${EVAL_SETS:-hold rnd all}; do   # EVAL_SETS override (repeat-1 pilot: "hold rnd", the 74-IC set at 1200 decisions/episode is too slow in-job)   # sample = registered statistic; mode = DP/RLPD-like deterministic counterpart (disclosed); all = FULL demo IC set (user, 2026-09-03)
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
