"""Verify direct reset/step execution against every saved observation and target."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

repo = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(repo / 'baselines'), str(repo / 'can_pos_recovery')]
os.environ['TI_CPU_MAX_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', '/tmp/eef-recovery-ti')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/eef-recovery-mpl')

import numpy as np
import torch
torch.set_num_threads(1)
from eef_replay_env import EEFReplayEnv
from genesis_can_env import GenesisCanEnv
from replay_harness import STATIC_BOTTLE_POSITION
from sim_variant_hook import apply_pre, apply_post

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('trace', type=Path)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
if args.out.exists():
    raise FileExistsError(args.out)
meta = json.loads(args.trace.with_suffix('.json').read_text())
assert meta['decision_dt_s'] == .03 and meta['precision_ik']
os.environ['GENESIS_SIM_VARIANT'] = meta['variant']
apply_pre(meta['variant'])
base = GenesisCanEnv(backend='cpu', max_steps=10**9)
apply_post(base, meta['variant'])
env = EEFReplayEnv(base)
with np.load(args.trace) as tape:
    observation = env.reset(can_pos=meta['can_pos'], can_quat=meta['can_quat'],
        goal_pos=(*STATIC_BOTTLE_POSITION[:2], base.w['goal_start_z']))
    assert np.array_equal(observation['state'], tape['observations'][0]), 'Reset differs'
    for i, action in enumerate(tape['actions_eef']):
        observation, _, _ = env.step(action)
        assert np.array_equal(observation['state'], tape['observations'][i+1]), ('Observation differs', i)
        assert np.array_equal(env.last_joint_target, tape['actions_joint'][i]), ('Joint target differs', i)
        if (i+1) % 300 == 0:
            print('VERIFIED', i+1, flush=True)
    report = dict(uid=meta['uid'], frames=len(tape['actions_eef']),
        all_observations_bit_identical=True, all_joint_targets_bit_identical=True,
        source_sha256=hashlib.sha256(args.trace.read_bytes()).hexdigest(),
        adapter_sha256=hashlib.sha256((repo / 'baselines/eef_replay_env.py').read_bytes()).hexdigest(),
        scope='Fresh configured world; reset and every direct EEF step compared to packaged tape',
        limitation='This validates the adapter on this trace, not broader physical fidelity or a policy interface')
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
