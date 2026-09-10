"""Summarize raw solver termination flags, retaining alternative exit criteria."""
from pathlib import Path
import hashlib
import json
import numpy as np

D = Path(__file__).resolve().parent
assert all(json.loads((D / 'identity.json').read_text()).values())
trace = D / '233_observed/constraint_convergence.npz'
v = np.load(trace)['values']
assert v.shape == (8 * (1 + 3 * 962), 7)
assert np.isfinite(v).all() and np.all(v[:, 5] > 0)
t = (v[:, 0] - 8) * .00125  # Exclude the one-scene reset from source replay time.
rows = []
for lo, hi in [(0, 28.86), (0, 6), (6, 20), (20, 23), (23, 28.86)]:
    w = v[(t >= lo) & (t < hi) & (v[:, 1] > 0)]
    rows.append(dict(interval_s=[lo, hi], constrained_substeps=len(w),
                     improved_still_true=int(w[:, 6].sum()),
                     fraction_improved_still_true=float(w[:, 6].mean()),
                     gradient_to_tolerance_percentiles=dict(zip(
                         ['p50', 'p90', 'p99', 'max'],
                         np.percentile(w[:, 2] / w[:, 5], [50, 90, 99, 100]).tolist())),
                     positive_small_cost_improvement=int(((w[:, 4] > 0) & (w[:, 4] < w[:, 5])).sum())))
engine = Path('/tmp/genesis-contact-1.4/lib/python3.10/site-packages/genesis/engine/solvers/rigid/constraint/solver.py')
report = dict(intervals=rows, callbacks=len(v), reset_callbacks=8,
              diagnostic_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
              inspected_engine_source=str(engine),
              inspected_engine_source_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
              qualification='Exact14-array observation identity passed. In this monolithic CG solver, '
              'the loop breaks when improved is false and otherwise runs to the iteration cap. '
              'A true final flag therefore indicates cap exhaustion, inferred from the installed '
              'loop; actual iteration counts were not instrumented. A high gradient alone is not '
              'a failed convergence check because positive-small-cost-improvement exits are allowed. '
              'Does not establish the cause of GPU or timestep trajectory differences.')
(D / 'summary.json').write_text(json.dumps(report, indent=2))
print(json.dumps(rows, indent=2))
