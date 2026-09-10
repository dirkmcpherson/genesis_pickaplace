"""Compare fixed-schedule unloaded hand trajectories at scene-step boundaries."""
from pathlib import Path
import json
import numpy as np

D = Path(__file__).resolve().parent
reports = {ss: json.loads((D / f'ss{ss}/report.json').read_text()) for ss in [8, 16, 32]}
traces = {ss: np.load(D / f'ss{ss}/trace.npz')['values'] for ss in reports}
assert all(json.loads((D / 'identity.json').read_text()).values())
for ss, report in reports.items():
    assert report['frames'] == 951 and report['error'] is None
    assert report['substep_count'] == ss * 951
    assert report['max_used_physical_q_difference_rad'] == 0
    assert report['preset_sha256'] == reports[8]['preset_sha256']
    assert report['urdf_sha256'] == reports[8]['urdf_sha256']
    np.testing.assert_array_equal(traces[ss][:, :2], traces[8][:, :2])
    np.testing.assert_array_equal(traces[ss][:, 10:14], traces[8][:, 10:14])
comparisons = []
for lo, hi in [(8, 16), (16, 32), (8, 32)]:
    diff = np.rad2deg(abs(traces[lo][:, 2:6] - traces[hi][:, 2:6]))
    comparisons.append(dict(substeps=[lo, hi], max_joint_difference_deg=float(diff.max()),
                            final_joint_difference_deg=float(diff[-1].max())))
out = dict(reports=reports, comparisons=comparisons, completed_schedules=3,
           qualification='The inherited per-bench qualification mentions only feedback freshness; '
           'in this frozen experiment freshness is fixed and substep resolution varies. '
           'Same scene schedule, hand and physical parameters; no contacts or gravity. '
           'These are unloaded benches, not independent real demonstrations. '
           'Small unloaded differences do not exclude their amplification under contact.')
(D / 'summary.json').write_text(json.dumps(out, indent=2))
print(json.dumps(comparisons, indent=2))
