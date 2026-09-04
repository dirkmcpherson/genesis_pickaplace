#!/usr/bin/env python
"""PHASE_PLAN amendment (b), 2026-09-04 (James): a MUCH broader random-IC evaluation set for the phase-1 (pick) models.
300 can placements uniform over the FROZEN support box recorded in baselines/eval_ics.json (the box rnd30 was drawn
from; the live placements table has since drifted -- hi_x 0.575 vs 0.592 -- so the table is deliberately NOT used),
same can_z and goal_pos as rnd30's entries, seed 1 so rnd30 (seed 0) is not a subset. No env needed.
    python baselines/make_eval_ics_rnd300.py            # writes baselines/eval_ics_rnd300.json
Consumed exactly like eval_ics.json (`--ic-file baselines/eval_ics_rnd300.json --ic-set rnd`)."""
import json, os, subprocess
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
base = json.load(open(os.path.join(HERE, 'eval_ics.json')))
lo, hi = base['support_box']['lo'], base['support_box']['hi']
can_z = float(base['rnd'][0]['can_pos'][2]); goal = list(base['rnd'][0]['goal_pos'])
assert all(abs(e['can_pos'][2] - can_z) < 1e-9 and list(e['goal_pos']) == goal for e in base['rnd'])
N, SEED = 300, 1
rng = np.random.default_rng(SEED)
rnd = [dict(can_pos=[float(rng.uniform(lo[0], hi[0])), float(rng.uniform(lo[1], hi[1])), can_z], goal_pos=goal, uid=None) for _ in range(N)]
try: git = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=HERE).decode().strip()
except Exception: git = None
out = dict(version=1, generator='baselines/make_eval_ics_rnd300.py', seed=SEED, git=git, base_file='baselines/eval_ics.json', base_git=base.get('git'),
           rnd=rnd, rnd_note=f'{N} uniform draws (seed {SEED}) over eval_ics.json support_box; can_z/goal_pos copied from its rnd entries; disjoint from rnd30 (seed 0)',
           support_box=base['support_box'], world_cfg=base.get('world_cfg'))
p = os.path.join(HERE, 'eval_ics_rnd300.json'); json.dump(out, open(p, 'w'), indent=1)
xs = [e['can_pos'][0] for e in rnd]; ys = [e['can_pos'][1] for e in rnd]
print(f'wrote {p}: {len(rnd)} ICs, x [{min(xs):.3f},{max(xs):.3f}] y [{min(ys):.3f},{max(ys):.3f}], can_z {can_z}, goal {goal}')
