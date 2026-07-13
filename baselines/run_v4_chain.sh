#!/bin/bash
# Detached v4 chain: finish collection -> convert -> train. Launched via setsid+nohup
# so it survives agent-session teardown (harness background tasks die with the session).
set -e
cd /home/james/workspace/genesis_pickaplace
SIMPY=/home/james/workspace/genesis_sim2real/venv/bin/python
LEROBOT=/home/james/workspace/lerobot/.venv

# remaining solved-success uids not yet collected and not already logged as skip/timeout
UIDS=$($SIMPY -c "
import json, pathlib as pl, re
t=json.load(open('can_pos_recovery/trial_placements.json'))['trials']
solved=sorted((u for u,r in t.items() if r['status'] in ('ok','ok_batch') and r.get('label')=='success'), key=int)
have={p.stem for p in pl.Path('baselines/episodes_raw_v4').glob('*.npz')}
done=set()
try:
    for line in open('baselines/collect_v4.log'):
        m=re.match(r'^(\d+) (TIMED OUT|.*SKIPPED)', line)
        if m: done.add(m.group(1))
except FileNotFoundError: pass
print(' '.join(u for u in solved if u not in have and u not in done))")

echo \"[$(date)] v4 chain: remaining uids = $UIDS\" >> baselines/run_v4_chain.log
[ -n \"$UIDS\" ] && baselines/collect_v4_wrapper.sh $UIDS

rm -rf baselines/lerobot_dataset_v4
$LEROBOT/bin/python baselines/convert_to_lerobot.py baselines/episodes_raw_v4 \
    baselines/lerobot_dataset_v4/genesis_pickaplace 8 >> baselines/convert_v4.log 2>&1
echo \"[$(date)] v4 convert done\" >> baselines/run_v4_chain.log

$LEROBOT/bin/lerobot-train --dataset.repo_id=local/genesis_pickaplace_v4 \
    --dataset.root=baselines/lerobot_dataset_v4/genesis_pickaplace \
    --policy.type=diffusion --policy.push_to_hub=false \
    --output_dir=baselines/outputs/dp_v4 --batch_size=64 --steps=100000 \
    --wandb.enable=false > baselines/train_v4.log 2>&1
echo \"[$(date)] v4 TRAIN DONE\" >> baselines/run_v4_chain.log
