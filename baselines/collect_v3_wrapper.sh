#!/bin/bash
# Per-trial isolation for dataset collection: a solver hang on one trial cannot stall
# the chain (lesson from the sweep_v2 22h wedge, relearned 2026-07-11 on the collector).
PY=/home/j/workspace/genesis_pickaplace/.venv-eval/bin/python
cd /home/j/workspace/genesis_pickaplace
for u in "$@"; do
    [ -f "baselines/episodes_raw_v3/$u.npz" ] && continue
    timeout 1800 $PY -u baselines/collect_lerobot_dataset.py \
        --outdir baselines/episodes_raw_v3 --uids "$u" >> baselines/collect_v3.log 2>&1
    if [ $? -eq 124 ]; then
        echo "$u TIMED OUT (solver hang) -- excluded" >> baselines/collect_v3.log
    fi
done
echo "collection wrapper done: $(date)" >> baselines/collect_v3.log
