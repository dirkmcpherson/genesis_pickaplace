# Unloaded hand resolution check

All three frozen 951-step schedules completed with refreshed controller feedback,
zero gravity and no collision. The 8-substep control reproduces both archived
scene and substep trace arrays exactly. Material, geometry, spring constants,
damping, armature, command schedule and scene-step duration remain fixed.

Maximum joint-trajectory differences at scene boundaries are 0.08646 degrees
for 8 versus 16 substeps, 0.01526 degrees for 16 versus 32, and 0.09684 degrees
for 8 versus 32. Final differences are below 0.000033 degrees. The unloaded
trajectory becomes closer with refinement; it does not show a divergence
comparable to the full-world can trajectory. This directs further diagnosis
toward contacts and their numerical solution, without excluding amplification
of small hand differences under contact. No physical parameter is selected.

`readout.py` checks the fixed schedule and hashes and writes `summary.json`.
The inherited qualification inside each raw report describes the parent freshness
experiment; this experiment instead varies numerical substeps with freshness fixed.
These three schedules do not count as full-source demonstration replays.
