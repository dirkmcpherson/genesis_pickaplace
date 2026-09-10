# Trial 113: descent observation and conditional seating check

Clock-matched real frames show the can upright on the shelf after opening.
The earlier impression that it was already settled at 20.04 s is withdrawn:
a closer check finds about 26 px of downward lid motion relative to the hand by
20.7 s (`113_real_drop_check.json` and `.png`). This is consistent with a brief
drop/settling motion; exact support onset and metric drop height are not measured.
The elliptic soft simulation first loses sampled hand contact at 20.16 s with
the can bottom roughly 4 cm above the simulated shelf. That gap alone does not
identify a shelf-height error. The exact force audit instead identifies a
backward shelf-edge impulse after finger separation (`../release_wrenches/`).
Confound 48's shelf-height claim remains withdrawn.

`real_frames.json` records extraction and clock matching; the three
`113_real_release_sheet*.jpg` files show the observed interval. The already
exported `../g14_feedback_full/113_elliptic_carry_release_real_sim.mp4` shows
improved carry followed by failed release, not a successful recovery.

For a conditional image comparison, manually annotated rigid cap landmarks at
8, 10 and 14 s fit camera extrinsics using recorded arm FK. Can rim annotations
are excluded from camera fitting. The nominal training RMS is 1.38 px; held-out
cap errors at 12 and 20.04 s range from 1.47 to 5.53 px. A partially occluded
18 s cap has 8.95 px error. Landmark uncertainty is approximately 5 px.

Across 15 focal-length/distortion hypotheses, cap-relative can-rim disagreement
is 23.64–25.34 px already at 8 s and 19.56–25.73 px at 20.04 s. It is not a
monotonically growing error. This supports examining an early grasp-position
mismatch as well as subsequent slip, conditional on the CAD cap correspondence
and camera model. It is not a calibrated three-dimensional can-pose measurement.
`113_conditional_seating.json` and `.png` preserve all hypotheses and checkpoints.

A separate vision-only fit uses the unheld can's outer rim at 4 s, assuming the
existing 33 mm radius and 101 mm height. It uses no task-success term. The nominal
fit is only 2.7 mm from archived settled XY, but its inferred height is unstable:
15 camera hypotheses times seven annotation perturbations produce world-height
estimates from 75.2 to 161.8 mm and X from 398.6 to 448.1 mm. These are sensitivity
ranges, not confidence intervals. The small nominal pixel residual does not
resolve this ambiguity. Neither initial placement nor object/shelf height is
changed. More independent camera constraints are needed before adopting a
vision-derived placement correction.

`113_landmarks.json`, `113_initial_rim.json`, extraction/fit scripts and
`113_initial_can_fit.json` retain the annotations, provenance and uncertainty.
The archived initial placement was outcome-fitted; this diagnostic does not
retroactively establish its accuracy or fit a replacement to recovery success.

## Multi-view held-can follow-up

`113_carry_rims.json` records approximate outer-rim points independently of the
next fit. `fit_carry_seating.py` fits one fixed can-to-tool pose to real rims at
10, 12 and 14 s, then evaluates 8, 18 and 20.04 s. The camera is still fitted
only to rigid caps. Axial symmetry leaves the can's rotation about its own axis
unidentified. No simulation success or world placement enters the objective.

The nominal training rim RMS is 2.75 px. Evaluation RMS is 3.56 px at 8 s,
6.07 px at 18 s and 4.00 px at 20.04 s; the larger 18 s error can include actual
relative movement. All 45 orientation-start fits and 90 annotation-sensitivity
fits converge. Across the declared 15 lens hypotheses, plus global ±3 px image
shifts and ±2 px radial perturbations, the fitted tool-z center is 18.42–26.50 mm.
The saved simulation's tool-z center is about 1–5 mm during this interval.
Thus the conditional real estimate is roughly 1–3 cm farther toward the
fingertips, already near pickup. This is not a measured pad thickness, a
confidence interval, or a validated rigid-grasp assumption across the whole demo.

The nominal fitted center is [0.81, 39.15, 23.02] mm in tool coordinates.
Depth along tool-y remains substantially less constrained (combined sensitivity
26.24–50.11 mm). `113_carry_seating_fit.json` retains all fits, and the inspected
`.png` overlays the real fitted and saved simulated rims. The physical direction
differs from earlier 233 seating observations; one demo's correction cannot be
assumed to transfer across closure regimes.

`113_finger_projection.png` compares unloaded commanded and saved simulated
finger outlines at the real wrist. Blue covering is visible predominantly on
the proximal fingers; distal surfaces are largely exposed. Covering thickness
and stiffness remain unknown, and no hidden finger angle is fitted. This motivates
a bounded proximal-only compliance comparison (`../proximal_compliance/plan.json`)
with original distal contact response, not added collision bumps or a success-fit
can displacement.

`113_fk_tracking_check.json` also separates raw-joint FK from the saved simulation
tool: the sampled position difference is 2.77–9.23 mm. The seating projections
deliberately remove this tracking difference by placing both relative can poses
at the raw FK wrist. At 20.04 s, tool +z is mostly horizontal in the current
world, so the roughly 20 mm axial seating difference does not explain a 40 mm
vertical release gap. Shelf height and absolute camera/world calibration remain
unresolved; the old shelf-height claim is not reinstated.

The final fitter uses the initial can axis's two transverse rotation coordinates, avoiding an almost axial tool-frame coordinate. The prior equivalent parameterization and results are archived in `carry_fit_v1/`; the change shifts nominal axial seating by only0.16 mm and does not alter the conclusion. Neither fit parameterization affected a simulation preset.

## Conditional initial XY probe

`fit_initial_on_plane.py` fits the unheld rim at 4 s with XY free and the existing
table plane assumed. The nominal initial XY is [0.433524, -0.154337] m, 17.24 mm
forward and 5.41 mm sideways from the archive. The 315 declared camera, plane-
height and annotation sensitivity fits span x=0.42456–0.44288 m and
y=-0.15915–-0.14909 m. These are conditional scenarios, not confidence intervals.

All three full comparisons at this nominal position are complete
(`../vision_initial_probe/README.md`). Original fixed, adaptive rigid and adaptive
soft all fail both full-task criteria. Rigid gains supported release but fails
the final supported push; soft still loses release. Both adaptive variants
substantially improve the image seating checkpoints. This separates better
seating from a soft-pad or end-to-end recovery claim. No correction is adopted.
The approximately horizontal seating difference can affect shelf-edge clearance
without explaining the vertical gap: a short drop also occurs in the real video.
