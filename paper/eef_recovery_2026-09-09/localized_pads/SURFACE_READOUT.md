# Surface material without subdivision: no end-to-end recovery yet

All four declared trial-233 surface-law replays completed. The 0.020 s control
reproduces the previous adaptive trajectory exactly. The contact classifier
identifies 6,428 pad-contact observations in that control; softened runs record
actual changed contacts. Per-step callback assertions and unchanged source-action
checks pass.

| Pad time constant | 14/16 s cap-relative rim error | Final center distance | Strict result |
|---|---:|---:|---|
| 0.020 s control | 14.4/13.6 px | 67.75 mm | No supported push to contact |
| 0.025 s | 8.4/8.2 px | 225.87 mm | No upright supported release |
| 0.030 s | 8.5/7.4 px | 106.57 mm | No upright supported release |
| 0.040 s | 8.3/8.0 px | 185.05 mm | No supported push to contact |

All three soft treatments fail the unchanged supplied metric too. The first two
finish tipped; 0.040 s eventually achieves upright supported release but places
the can poorly. First sustained hand separation is 22.47 s in the control versus
21.24/21.39/25.59 s in the soft treatments. Separation timing alone does not
establish successful release.

Real-image comparisons use the existing four annotated frames and fixed camera
assumptions, with simulated can/tool geometry attached to the real wrist pose.
The loaded grasp improvement is conditional image-space evidence, not calibrated
3D pose or pad strain. All image comparison processes completed successfully.

The initial six finite-return-spring comparisons are declared in `spring_plan.json`.
They test a different mechanical factor, with original-contact controls at each
spring setting, after contact-only settings failed to restore the full task.
No pad candidate has yet been extended to early-day calibration or validation.
The goal remains active and no training data have been admitted.
