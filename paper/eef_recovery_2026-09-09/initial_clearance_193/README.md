# Trial 193 initial-clearance correction

The archived initial can at x=0.518356 m overlaps the full-world shelf by 1.356 mm.
Its first recorded simulated position is already displaced to x=0.515214 m as the
solver resolves contact. The uncorrected trace passes the supplied slide metric,
but the geometry audit prevents it from being admitted as an ordinary recovery.

This single declared rerun sets x=0.516 m: shelf near edge 0.55 m minus can radius
0.033 m minus 0.001 m clearance. The shift is -2.356 mm; y, z, orientation, real
measurement stream, timing, gripper mapping, physics and -19.2 degree mount yaw
remain unchanged. No og4 or added motion is used. There is no position sweep.

This is a geometric correction to an estimated initial position, not ground-truth
metrology. The result must retain that provenance even if it succeeds. It requires
fresh collection, independent EEF replay and visual review before any admission.
Trial 196 is not treated the same way: its 77.4 mm horizontal clearance deficit is
too large to justify as this kind of small placement correction.

The archived sources and ordinary early-census results are preserved. See
`plan.json`, `source.json` and `run.log` for the declared correction and execution.
