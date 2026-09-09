# Softer fingers versus softer can

The user requested more compliant gripper or can behavior after reviewing the
adaptive-hand video. This experiment retains that same URDF and transmission.
It changes compliant rigid contact, not mesh deformation or a measured material
modulus. Solver overlap is recorded as a compression proxy and cannot be called
measured rubber deformation or can strain.

The fixed comparison is two trials (233 and early-day 113), each with baseline,
softer fingers and softer manipulated can. The selected geometry time constant
changes from 0.02 to 0.04 s; all other contact parameters are unchanged. Because
this engine averages pair parameters, finger-can contact changes from 0.02 to
0.03 s in both treatments. Finger treatment also affects finger-other contacts;
can treatment also affects can-table/shelf/goal contacts. The goal can is unchanged.

All four finger collision geometries are included in the finger treatment;
the current mesh decomposition does not isolate the rubber contact surface.
This limitation matters when interpreting the result as a realistic pad model.

Original EEF/arm/grip commands, initial poses, corrected early-day yaw, contact
friction/impedance/damping ratio, actuator force/spring parameters and joint stops
are fixed. The two baseline reruns collect new force/penetration measurements and
must reproduce the previous adaptive traces exactly. The observer samples contact
data after every original 10 ms scene step. No extra physics steps or settling
are added. The original slide predicate and separate strict sequence rule remain
unchanged. Plan, per-geometry before/after readbacks, callback counts, traces and
all run logs are retained; no treatment is admitted into a training bank.

## Completed trial 233 comparison

| Adaptive hand contact | Median carry overlap | Maximum inward curl | Final center distance |
|---|---:|---:|---:|
| Original | 4.68 mm | 18.36 degrees | 67.75 mm |
| Softer fingers | 8.92 mm | 22.19 degrees | 183.24 mm |
| Softer can | 9.63 mm | 23.09 degrees | 185.29 mm |

All three pick and eventually achieve upright supported release; none completes
the strict supported slide/contact sequence. Original adaptive contact passes
the supplied proximity-based metric; both softer cases lose it. The original
trace reproduces exactly with the added observer. Carry overlap and force are
descriptive statistics over each run's own carry/contact samples, not identical
paired contact configurations.

There is an observable grasp improvement despite the later regression. Using
the same real-image annotations and cap-based cameras, the nominal cap-relative
rim error at 14/16 s is 14.4/13.6 px for original adaptive contact, 4.9/3.2 px
for softer fingers, and 2.4/1.4 px for softer can contact. The latter values are
at the scale of annotation/camera error; they do not establish millimetre accuracy
or prove the real material is that soft. At 25 s the errors instead rise to about
135/136 px for the soft treatments, versus 43 px for original adaptive contact.

[Real-image comparison](233_softness_real_comparison.png) shows both phases.
The soft cases hold deeper and remain in hand contact longer during opening,
then displace the can sideways during release/placement. Neither is an adopted
recovery improvement.

## Completed early-day trial 113 comparison

All six declared executions finished successfully. The early-day trial uses its
corrected yaw and unchanged recorded commands. Neither softness treatment rescues
the carry or achieves upright supported shelf release.

| Adaptive hand contact | Median carry overlap | First sustained hand separation | Final center distance |
|---|---:|---:|---:|
| Original | 0.92 mm | 9.96 s | 243.87 mm |
| Softer fingers | 1.74 mm | 9.84 s | 256.63 mm |
| Softer can | 1.47 mm | 9.90 s | 248.93 mm |

Separation means ten consecutive sampled frames without hand contact after the
picked flag; it does not establish successful placement. All three fail both the
strict physical sequence and the supplied metric. Both baseline reruns exactly
reproduce their previous adaptive trajectories. All six retain identical source
EEF actions, refined arm targets, gripper inputs and mounting transforms, and pass
the callback-count and contact-observation-count assertions.

This is a two-trial mechanism probe at one predefined softness level, not a
population recovery-rate estimate. More compliant contact improves the observed
loaded grasp in 233 but does not improve recovery in either selected trial.
The 9–10 mm solver overlaps in the softer 233 cases are not independently validated
as physical compression. A next candidate should separate pad compliance from
joint actuation/return mechanics and check opening/release against the real video;
these results do not justify adopting bulk can softness.
