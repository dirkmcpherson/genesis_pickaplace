# Real tool-position consistency audit

All 163 declared candidates were checked: 49 December 16, 40 December 17 and
74 December 18. Forward kinematics uses the same URDF chain as the precision EEF
controller, with the identity mount because both recorded signals are in the
real arm's base frame. It is compared with the recorded Cartesian tool xyz.

| Day | Trials | Median per-trial p95 error | Worst per-trial p95 error | Largest per-trial mean-bias norm |
|---|---:|---:|---:|---:|
| December 16 | 49 | 3.132 mm | 3.833 mm | 0.271 mm |
| December 17 | 40 | 0.293 mm | 3.092 mm | 0.234 mm |
| December 18 | 74 | 0.380 mm | 3.561 mm | 0.265 mm |

The raw-timestamp four-trial spot check also gave sub-0.1 mm componentwise mean
errors; instantaneous residuals can reflect timing differences between joint-state
and feedback messages. The full audit uses window-averaged fields from the existing
Cartesian tapes and retains all per-trial results and file hashes in `results.json`.
The vectorized FK was checked against scalar controller FK at three frames per tape.

This gives no evidence for a centimeter-scale constant error in the arm-base
wrist-to-tool transform used to reconstruct EEF targets. It is an internal signal
consistency check: reported tool coordinates may themselves be computed from robot
kinematics, so agreement is not independent physical metrology. It does not verify
camera/world alignment, shelf dimensions, can placement, arm deflection or finger
compliance. Changing the tool transform to improve recovery is not justified by
these measurements.

Orientation is not scored here: averaging reported Euler angles across wrapping
boundaries can produce an invalid rotation reference. No replay commands, gains,
physics, geometry or acceptance thresholds were changed by this audit.
