#!/bin/bash
# Re-evaluate every BC-RNN run's TRUE last checkpoint (highest epoch, basename-sorted) on the 50-state bank, keeping any
# earlier eval dir under its epoch (eval_bank50_ep<N>). CPU, fresh process per run (~100 s each). Idempotent.
#   bash cluster/robomimic/bcrnn_reeval_last.sh [run_dir ...]   (default: all $LAB/robomimic_runs/bcrnn/bcrnn_*_s?)
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; PY=$LAB/robo_venv/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl CUDA_VISIBLE_DEVICES="" OMP_NUM_THREADS=2
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic
RUNS=("$@"); [ ${#RUNS[@]} -gt 0 ] || RUNS=($LAB/robomimic_runs/bcrnn/bcrnn_*_s?)
for OUT in "${RUNS[@]}"; do
  NAME=$(basename $OUT); ARM=${NAME#bcrnn_}; ARM=${ARM%_s?}
  grep -q "finished run successfully" $OUT/train.log 2>/dev/null || { echo "$NAME: training not finished -- skip"; continue; }
  CKPT=$(ls -1 $OUT/trained/$NAME/*/models/model_epoch_*.pth 2>/dev/null | awk -F'model_epoch_' '{split($2,a,".pth"); print a[1]"\t"$0}' | sort -n | tail -1 | cut -f2)
  [ -f "$CKPT" ] || { echo "$NAME: no checkpoint -- skip"; continue; }
  EP=$(basename $CKPT | sed 's/model_epoch_//; s/.pth//')
  if [ -f $OUT/eval_bank50/metrics.json ]; then
    OLD=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["checkpoint"].split("/")[-1])' $OUT/eval_bank50/metrics.json)
    if [ "$OLD" = "$(basename $CKPT)" ]; then echo "$NAME: eval_bank50 already at $OLD -- skip"; continue; fi
    OE=$(echo $OLD | sed 's/model_epoch_//; s/.pth//'); mv $OUT/eval_bank50 $OUT/eval_bank50_ep$OE; echo "$NAME: kept old eval as eval_bank50_ep$OE"
  fi
  $PY $B/eval_bcrnn_robosuite.py --checkpoint "$CKPT" --arm $ARM --out $OUT/eval_bank50 2>&1 | grep -E "^\[eval bcrnn\] [0-9]+/|Traceback|Error" | tail -2
  python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print("BCRNN-RESULT-LAST", sys.argv[2], "last=" + str(d["n_success"]) + "/" + str(d["episodes"]), "epoch=" + sys.argv[3])' $OUT/eval_bank50/metrics.json $NAME $EP
done
