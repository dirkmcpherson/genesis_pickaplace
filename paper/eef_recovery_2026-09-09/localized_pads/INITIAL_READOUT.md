# Divided-pad geometry: rejected initial candidate

All five declared trial-233 runs finished successfully. No new candidate
completed the strict physical sequence. Every divided-geometry run ended with
the can lying on its side (approximately 90 degrees tilt), including the
unchanged-material control.

| Geometry | Pad geometry time constant | Final goal-center distance | First sustained hand separation | Strict outcome |
|---|---:|---:|---:|---|
| Reconstructed single hull | 0.020 s | 67.54 mm | 22.44 s | No supported push to contact |
| Divided, all original contact | 0.020 s | 102.78 mm | 21.12 s | No upright supported release |
| Divided, soft pad | 0.025 s | 84.91 mm | 21.15 s | No upright supported release |
| Divided, soft pad | 0.030 s | 101.54 mm | 21.18 s | No upright supported release |
| Divided, soft pad | 0.040 s | 117.67 mm | 21.18 s | No upright supported release |

The reconstructed control retains the prior adaptive hand's supplied-metric pass
but not its original fixed-coupling full-sequence success. Relative to the prior
adaptive trace, its final XYZ difference is 0.63 mm and its goal-distance
difference is 0.21 mm. These are distinct quantities. Export/reconstruction is
not bit-identical replay, despite the near-identical outer support geometry.

In the divided rigid control, carried contact overlap is about 4.65 mm at the pad,
1.75 mm at the backing and 2.42 mm at the joint-adjacent region (per-region
medians, differing sample populations). At 14 s the contact count is 22 versus
8 in the reconstructed single-hull control. At 21 s the can is still held; by
22 s it is on its side on the shelf. This identifies a material change in
contact dynamics from subdivision, not evidence that rubber pads inherently
cause poor release. The 84.91 mm endpoint is not a useful upright near-success.

All runs retain exact EEF actions, refined arm targets, gripper inputs and mount
relative to the prior adaptive trace; callback and contact-sample checks pass.
No metric thresholds, source commands or initial conditions changed. Trial 233
was already a calibration trial; none of these results estimates population
recovery yield.

Next executed comparison: `surface_plan.json`. It retains the original collision
meshes and contact generation, classifies inner pad contacts in finger-local
coordinates, and applies a layered restoring law. Stiffness is lower over the
assumed pad thickness; penetration beyond the backing plane receives the
original incremental stiffness. This is an exploratory contact approximation,
not continuum deformation or a measured force/displacement curve. Its 0.020 s
control must reproduce the original adaptive trace exactly before interpreting
the softer settings. No additional calibration or validation trials have yet
been run with a pad candidate.
