"""Compare the declared iteration-cap refinement with its exact observed control."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

D = Path(__file__).resolve().parent
R = D.parents[3]
sys.path.insert(0, str(R / 'can_pos_recovery'))
from eef_task_sequence import score_sequence
from score_recovered_slides import adapt

plan = json.loads((D / 'plan.json').read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
for key in ['source', 'preset', 'reference']:
    assert sha(plan[key]) == plan[key + '_sha256']
for f, h in plan['source_code_sha256'].items():
    assert sha(D / 'executed_sources' / Path(f).name) == h
assert json.loads((D / 'execution.json').read_text())['returncode'] == 0
assert all(json.loads((D / 'source_identity.json').read_text()).values())
folders = {100: D.parent / 'constraint_convergence/233_observed', 1000: D / '233_cg1000'}
rows, traces, audits, runtimes = {}, {}, {}, {}
for cap, folder in folders.items():
    path = folder / '233_eef_delta.npz'
    z = np.load(path)
    m = json.loads(path.with_suffix('.json').read_text())
    audits[cap] = json.loads((folder / 'transmission_audit.json').read_text())
    runtimes[cap] = json.loads((folder / 'constraint_runtime.json').read_text())
    stats = json.loads((folder / 'transmission_stats.json').read_text())
    assert len(z['trajectory']) == m['frames'] == 962
    assert m['decision_dt_s'] == .03 and m['extension_m'] == 0 and not m['hold_contact']
    assert stats['substep_calls'] == 23096 and stats['max_used_feedback_difference_rad'] == 0
    assert runtimes[cap]['options']['iterations'] == str(cap)
    assert score_sequence(z) == m['sequence']
    v = np.load(folder / 'constraint_convergence.npz')['values'][8:]
    assert v.shape == (23088, 7) and np.isfinite(v).all()
    t = np.arange(len(v)) * .00125
    intervals = []
    for lo, hi in [(0, 28.86), (0, 6), (6, 20), (20, 23), (23, 28.86)]:
        w = v[(t >= lo) & (t < hi) & (v[:, 1] > 0)]
        intervals.append(dict(interval_s=[lo, hi], constrained_substeps=len(w),
                              improved_still_true=int(w[:, 6].sum()),
                              fraction_improved_still_true=float(w[:, 6].mean())))
    shift = 1000 * np.linalg.norm(z['goal_pose'][:, :2] - z['goal_pose'][0, :2], axis=-1)
    rows[cap] = dict(sequence=m['sequence'], metric=adapt(path, folder / 'metric_adapter')['metric'],
                     final_goal_shift_mm=float(shift[-1]), intervals=intervals,
                     trace_sha256=sha(path), diagnostic_sha256=sha(folder / 'constraint_convergence.npz'))
    traces[cap] = z
for key in ['geometry', 'collision_policy', 'armature_readback', 'normal_parameters',
            'finger_inertial_readback', 'can_mass_kg', 'goal_mass_kg', 'limits_readback']:
    assert audits[100][key] == audits[1000][key], key
for key in runtimes[100]:
    if key == 'options':
        for opt in runtimes[100][key]:
            if opt != 'iterations':
                assert runtimes[100][key][opt] == runtimes[1000][key][opt], opt
    else:
        assert runtimes[100][key] == runtimes[1000][key], key
a, b = traces[100], traces[1000]
diff = 1000 * np.linalg.norm(a['trajectory'][:, 13:16] - b['trajectory'][:, 13:16], axis=-1)
comparison = dict(max_can_difference_mm=float(diff.max()), final_can_difference_mm=float(diff[-1]),
                  first_over_1mm_s=float((np.flatnonzero(diff > 1)[0] + 1) * .03) if np.any(diff > 1) else None,
                  max_finger_difference_rad=float(abs(a['finger_joint'] - b['finger_joint']).max()))
report = dict(records=rows, comparison=comparison,
              qualification='Only the iteration cap changed. Intended physics and source inputs verified. '
              'Different trajectories also encounter different contacts; cap-exit counts are full-replay '
              'diagnostics, not a paired residual test at identical physical states. '
              'The frozen plan phrase "No material or numerical change" is an inherited wording error: '
              'the plan and command explicitly change the numerical cap100 to1000. No material change.')
(D / 'summary.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
