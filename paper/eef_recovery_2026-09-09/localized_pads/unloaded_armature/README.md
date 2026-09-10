# Added finger inertia: verified unloaded dynamics confound

The previous turn completed 56 full replays and rejected the frozen soft hand as
a general improvement. This turn found a separate dynamics confound: all four
finger joints inherited Genesis's generic 0.1 kg m² armature. The surrogate makes
the distal joints passive, but leaves that independent added inertia on each.

The pinned loader assigns `default_dofs_armature` to every revolute joint;
`geom.py` returns 0.1. The rigid solver adds it to the mass-matrix diagonal. Both
source hashes and URDF link calculations are in `source_audit.json`; the bench
also checks the live readback before changing it. Each distal link's URDF
inertia about its joint axis is approximately 5.30e-6 kg m², so the generic
armature is about 18,879 times that link inertia. This does not measure the real
actuator's reflected inertia.

The especially relevant direction is the passive internal mode at a fixed
assumed actuator coordinate: n = [-1, 1, -2, -2]. Since the motor gradient dotted
with n is zero, a physical scalar motor inertia reflected through this assumed
transmission would contribute zero along that direction. Four independent
0.1 armatures instead contribute 1.0 to its generalized mass. This is a
self-consistency issue in the surrogate, independent of the real motor's unknown
inertia.

## Controlled bench

Four fresh hand-only runs use identical transmission parameters from the frozen
fast-return preset, original URDF link inertias, a fixed base, zero gravity and
no collisions. The same input ramps at 60 percentage points/s with two-second
holds. This is an unloaded dynamics diagnostic, not a recovered demo or a
measured hardware trajectory. No pad contact occurs.

| Added joint inertia | Result | Largest joint error from unloaded relation | Largest error at the three hold endpoints |
|---|---|---:|---:|
| Original 0.1 on all four | 951/951 frames, oscillatory | 34.46° | 12.04° |
| Zero on passive tips only | Instability guard stops after 1 saved frame | Not a valid full comparison | — |
| 0.0001 on all four | 951/951 frames, settles | 10.57° | 0.011° |
| Zero on all four | Instability guard stops after 23 saved frames | Not a valid full comparison | — |

`unloaded_response.png` plots the two completed responses. `mode_readout.json`
subtracts damping*substep_dt from the solver matrix before computing inertial
properties. At the final pose, the linearized internal mode has a natural period
of 7.50 s and damping ratio .069 with the original armature, versus .248 s and
2.10 with the small armature.

**Correction to the earlier damping explanation:** the previous D/K = .165 s
calculation omitted inertia and assumed an overdamped regime. The original
hand is strongly underdamped, so .165 s is not its relaxation time. The algebra
is retained in `../return_mode_diagnostic.json`, but the earlier interpretation
is superseded by this mass-inclusive calculation and the executed bench.

Zero armature is unstable with this explicit spring-force update and fixed
1.25 ms substep. The small nonzero setting is therefore a declared numerical
regularization, not a claimed hardware correction. Its stability under loaded
contact and its full-task value still require testing.

## Next full-world test

`../low_armature/plan.json` fixes the 0.0001 value from this unloaded bench, then
compares original-contact and soft pads on calibration 113/184/233. All other
hand parameters and the full source/world are unchanged. One additional 233
control sets the old 0.1 value explicitly and must reproduce the previous rigid
hand trace exactly. Reserved 176/185/237 are excluded from this new fitting.
No training-bank change or end-to-end improvement is established yet.
