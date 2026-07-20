#!/bin/bash
# END-TO-END DP-on-successful-picks pipeline ("Go for it"). Fully detached; each stage waits
# for its prerequisite so it survives long gaps. Respects dreamer4: the GPU train stage only
# fires once the GPU has been free of compute procs for a sustained window.
cd /home/james/workspace/genesis_pickaplace
LOG=baselines/pick_pipeline.log
echo "[$(date)] PICK PIPELINE START" > $LOG

# 1. wait for the keep-everything collection to finish
while [ "$(ps aux | grep collect_all_classified | grep -v grep | wc -l)" -gt 0 ]; do sleep 120; done
echo "[$(date)] collection done: $(ls baselines/episodes_all/*.npz 2>/dev/null | wc -l)/91 npz" >> $LOG

# 2. finalize: authoritative manifest + episodes_pick (solved demos that reached >= picked)
.venv-eval/bin/python baselines/finalize_pick_dataset.py >> $LOG 2>&1
NPICK=$(ls baselines/episodes_pick/*.npz 2>/dev/null | wc -l)
echo "[$(date)] finalize done: $NPICK successful-pick demos" >> $LOG
if [ "$NPICK" -lt 5 ]; then echo "[$(date)] ABORT: too few pick demos" >> $LOG; exit 1; fi

# 3. wait for the GPU to be genuinely free (no compute procs) for ~2 min sustained
echo "[$(date)] waiting for GPU free (respecting dreamer4)..." >> $LOG
free=0
while [ $free -lt 4 ]; do
  n=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
  if [ "$n" -eq 0 ]; then free=$((free+1)); else free=0; fi
  sleep 30
done
echo "[$(date)] GPU free -> launching DP-pick convert+train" >> $LOG

# 4. convert + train DP on successful picks (GPU)
bash baselines/run_dp_pick.sh >> $LOG 2>&1
echo "[$(date)] PICK PIPELINE DONE (exit $?)" >> $LOG
