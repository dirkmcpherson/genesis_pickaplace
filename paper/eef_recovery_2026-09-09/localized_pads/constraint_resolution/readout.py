"""Verify physical invariants and compare numerical refinement with CG1000 fixed."""
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

assert json.loads((D / 'execution.json').read_text())['returncode'] == 0
assert all(json.loads((D / 'source_identity.json').read_text()).values())
plan = json.loads((D / 'plan.json').read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
for k in ['source', 'reference', 'preset']:
    assert sha(plan[k]) == plan[k + '_sha256']
for f, h in plan['source_code_sha256'].items():
    assert sha(D / 'executed_sources' / Path(f).name) == h
folders = {8: D.parent / 'constraint_iterations/233_cg1000', 16: D / '233_cg1000_ss16'}
rows, traces, audits, runtimes = {}, {}, {}, {}
for ss, folder in folders.items():
    path = folder / '233_eef_delta.npz'
    z = np.load(path)
    m = json.loads(path.with_suffix('.json').read_text())
    stats = json.loads((folder / 'transmission_stats.json').read_text())
    audits[ss] = json.loads((folder / 'transmission_audit.json').read_text())
    runtimes[ss] = json.loads((folder / 'constraint_runtime.json').read_text())
    assert m['frames'] == len(z['trajectory']) == 962 and m['decision_dt_s'] == .03
    assert m['extension_m'] == 0 and not m['hold_contact']
    assert stats['substep_calls'] == ss * (1 + 3 * 962)
    assert stats['max_used_feedback_difference_rad'] == 0
    assert runtimes[ss]['options']['iterations'] == '1000'
    assert runtimes[ss]['substep_dt_s'] == .01 / ss
    assert score_sequence(z) == m['sequence']
    v = np.load(folder / 'constraint_convergence.npz')['values'][ss:]
    assert len(v) == 3 * 962 * ss and np.isfinite(v).all()
    shift = 1000 * np.linalg.norm(z['goal_pose'][:, :2] - z['goal_pose'][0, :2], axis=-1)
    rows[ss] = dict(sequence=m['sequence'], metric=adapt(path, folder / 'metric_adapter')['metric'],
                    final_goal_shift_mm=float(shift[-1]), source_substeps=len(v),
                    improved_still_true=int(v[:, 6].sum()), trace_sha256=sha(path))
    traces[ss] = z
for k in ['geometry', 'collision_policy', 'armature_readback', 'normal_parameters',
          'finger_inertial_readback', 'can_mass_kg', 'goal_mass_kg', 'limits_readback']:
    assert audits[8][k] == audits[16][k], k
for k in runtimes[8]:
    if k != 'substep_dt_s':
        assert runtimes[8][k] == runtimes[16][k], k
a, b = traces[8], traces[16]
diff = 1000 * np.linalg.norm(a['trajectory'][:, 13:16] - b['trajectory'][:, 13:16], axis=-1)
report = dict(records=rows, comparison=dict(max_can_difference_mm=float(diff.max()),
              final_can_difference_mm=float(diff[-1]), max_finger_difference_rad=float(abs(a['finger_joint'] - b['finger_joint']).max())),
              qualification='Both runs have zero observed cap exits, yet full-task trajectories remain sensitive '
              'to integration resolution. Cap exhaustion is therefore not a sufficient explanation or remedy. '
              'Physical and source inputs fixed; no setting chosen by task yield.')
(D / 'summary.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
