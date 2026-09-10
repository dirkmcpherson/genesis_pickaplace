# Declared64-substep numerical refinement

The preceding8/16/32 test produced pass/fail/pass for the same soft233 model and
unchanged recorded motion. Final positions remain substantially different. This
single follow-up halves the32-substep integration interval again to0.15625ms;
scene/control intervals and source endpoint stay fixed. Material parameters,
geometry, initial poses and scoring remain unchanged. No resolution is selected
by which one passes. The previous8-substep wrapper identity matched all14 arrays.

`run_g14_hand_resolution_v2.py` differs only by accepting64 on its CLI. `run.py`
freezes source/code/preset hashes and runs one CPU worker. `readout.py` checks
full-source and physical invariants; `compare.py` combines successive refinements.
The physical-model objective remains open; this diagnostic is not a new demo.

The 64-substep case completed all 962 recorded decisions and passed the input
and physical-parameter checks, but the can leaves the shelf and ends 0.99228 m
from the goal. The strict sequence fails supported push-to-contact. Final can
positions at 32 and 64 substeps differ 945.58 mm; goal motion at 64 is 0.239 mm.
The sequence across 8/16/32/64 is therefore pass/fail/pass/fail, not convergence.
Further automatic timestep halving is not justified by these outcomes alone.
`combined_summary.json` retains all four cases and successive comparisons.
