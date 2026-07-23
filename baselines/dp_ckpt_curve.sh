#!/bin/bash
# Short-horizon eval of EVERY saved DP checkpoint -> overfitting curve.
# Usage: dp_ckpt_curve.sh <output_dir e.g. baselines/outputs/dp_pick> <wandb_group> [n_random] [max_steps]
# Waits for any running eval_policy procs to clear the GPU first, then evals all
# numbered checkpoints in parallel (one proc each), aggregates to ONE wandb run.
set -e
cd /home/j/workspace/genesis_pickaplace
PY=.venv-eval/bin/python
OUT=$1; GROUP=$2; NRAND=${3:-8}; MAXSTEPS=${4:-400}
CURVE=$OUT/ckpt_curve
mkdir -p $CURVE

while pgrep -f "eval_policy" > /dev/null; do sleep 30; done

for d in $OUT/checkpoints/[0-9]*/; do
  step=$(basename $d)
  [ -f "$CURVE/m_$step.json" ] && continue
  $PY baselines/wandb_eval.py --kind dp --checkpoint $d/pretrained_model \
      --random $NRAND --max-steps $MAXSTEPS --seed 0 \
      --record-dir $CURVE/videos_$step --json $CURVE/m_$step.json --no-wandb \
      > $CURVE/eval_$step.log 2>&1 &
done
wait

$PY - "$CURVE" "$GROUP" <<'EOF'
import json, glob, sys, pathlib as pl
curve, group = sys.argv[1], sys.argv[2]
rows = []
for p in sorted(glob.glob(f'{curve}/m_*.json')):
    step = int(pl.Path(p).stem.split('_')[1])
    m = json.loads(open(p).read())['metrics']
    rows.append((step, m))
print(f"{'step':>8} {'picked':>7} {'placed':>7} {'contact':>8} {'nested':>7}")
for s, m in rows:
    print(f"{s:>8} {m['eval/picked']:>7.2f} {m['eval/placed']:>7.2f} "
          f"{m['eval/contact']:>8.2f} {m['eval/nested']:>7.2f}")
import wandb
run = wandb.init(project='genesis_pickaplace', group=group, name=f'{group}-ckpt-curve',
                 job_type='eval')
for s, m in rows:
    log = dict(m)
    tiled = pl.Path(curve) / f'videos_{s:06d}' / 'tiled.mp4'
    if tiled.exists():
        log['eval/rollouts_tiled'] = wandb.Video(str(tiled), format='mp4',
                                                 caption=f'ckpt {s}')
    run.log(log, step=s)
run.finish()
EOF
echo "CKPT-CURVE DONE"
