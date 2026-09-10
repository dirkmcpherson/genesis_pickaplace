"""Read the completed native iteration-cap diagnostic and retain CPU/GPU differences."""
from pathlib import Path
import hashlib
import json
import numpy as np

D = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
plan = json.loads((D / 'plan.json').read_text())
assert all(json.loads((D / 'cpu_identity.json').read_text()).values())
execution = json.loads((D / 'execution.json').read_text())
assert len(execution) == 2 and all(r['returncode'] == 0 for r in execution)
for f, h in plan['source_code_sha256'].items():
    assert sha(D / 'executed_sources' / Path(f).name) == h
for k in ['trace', 'reference', 'preset']:
    assert sha(plan[k]) == plan[k + '_sha256']
rows, traces, reports, runtimes = {}, {}, {}, {}
for case in plan['cases']:
    name = case['name']
    folder = D / name
    z = np.load(folder / 'trace.npz')
    diag = np.load(folder / 'full_task_diagnostic.npz')
    report = json.loads((folder / 'report.json').read_text())
    runtime = json.loads((folder / 'iteration_runtime.json').read_text())
    assert report['steps'] == 962 and report['full_source_length_verified']
    assert report['calls'] == report['pad_calls'] == 23096
    assert runtime['actual_iterations'] == 1000 and runtime['substep_dt_s'] == .00125
    for key in ['state', 'arm_commands', 'source_grip']:
        for lane in range(1, report['actual_envs']):
            np.testing.assert_array_equal(z[key][:, lane], z[key][:, 0])
    for key in ['trajectory', 'contact_counts', 'goal_pose', 'wrist_pose', 'picked']:
        for lane in range(1, report['actual_envs']):
            np.testing.assert_array_equal(diag[key][:, lane], diag[key][:, 0])
    for result in report['task_results'][1:]:
        for key in ['sequence', 'metric', 'final_goal_shift_mm', 'max_goal_shift_mm']:
            assert result[key] == report['task_results'][0][key]
    rows[name] = dict(first_lane=report['task_results'][0], actual_envs=report['actual_envs'],
                      aggregate_decisions_per_second=report['aggregate_decisions_per_second'],
                      trace_sha256=sha(folder / 'trace.npz'),
                      diagnostic_sha256=sha(folder / 'full_task_diagnostic.npz'))
    traces[name], reports[name], runtimes[name] = z, report, runtime
cpu, gpu = plan['cases'][0]['name'], plan['cases'][1]['name']
assert runtimes[cpu] == runtimes[gpu]
for key in ['candidate_urdf_sha256', 'preset_sha256', 'source_trace_sha256']:
    assert reports[cpu][key] == reports[gpu][key]
for key in ['arm_commands', 'source_grip']:
    np.testing.assert_array_equal(traces[cpu][key][:, 0], traces[gpu][key][:, 0])
diff = 1000 * np.linalg.norm(traces[cpu]['state'][:, 0, 10:13] - traces[gpu]['state'][:, 0, 10:13], axis=-1)
out = dict(records=rows, comparison=dict(max_can_difference_mm=float(diff.max()),
           final_can_difference_mm=float(diff[-1]), first_over_1mm_s=float((np.flatnonzero(diff > 1)[0] + 1) * .03)),
           qualification='CPU port exactly matches independent cap1000 replay; all duplicate GPU lanes match. '
           'CPU and GPU both complete233 at cap1000, whereas earlier GPU cap100 tipped. '
           'Physical state equivalence is still not established; this is one recording, '
           'not a new pad configuration or independent16-demo sample. This cap1000 GPU case has not been repeated.')
(D / 'summary.json').write_text(json.dumps(out, indent=2))
print(json.dumps(out, indent=2))
