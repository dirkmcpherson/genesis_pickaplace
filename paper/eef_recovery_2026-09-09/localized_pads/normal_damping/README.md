# Normal-only pad damping: near-contact slide, no adoption

Completed full replays: 10/10. All terminal: True. The verified implementation changes only the normal contact reference damping at existing inner-pad regions. The normal elastic term, tangent and all other references, contact parameters, friction coefficient and geometry are unchanged by the hook. Source motions, corrected early yaw and world geometry remain fixed.

| Trial / placement | Pads | Gain | Status | Strict result | Final center distance (m) | Goal movement (mm) |
| --- | --- | ---: | --- | --- | ---: | ---: |
| 233 original | soft | 1 | complete | complete | 0.06599 | 9.863 |
| 233 original | rigid | 2 | complete | no_supported_push_to_contact | 0.12234 | 0.033 |
| 233 original | soft | 2 | complete | no_supported_push_to_contact | 0.06647 | 0.636 |
| 113 conditional | soft | 1 | complete | no_supported_release | 26.00946 | 60.800 |
| 113 conditional | rigid | 2 | complete | no_supported_release | 16.81716 | 60.230 |
| 113 conditional | soft | 2 | complete | no_supported_release | 30.22378 | 60.485 |
| 113 original | rigid | 2 | complete | no_supported_release | 0.24371 | 58.824 |
| 113 original | soft | 2 | complete | no_supported_release | 40.42920 | 62.435 |
| 184 original | rigid | 2 | complete | no_supported_release | 18.52405 | 25.788 |
| 184 original | soft | 2 | complete | no_supported_release | 20.24470 | 74.889 |

Soft233 preserves the supplied proximity-metric pass but fails the strict
push-to-contact sequence. After supported release at22.65s, the supported hand
push starts22.86s and moves40.90mm toward the goal by the endpoint. Support
fraction is.935, maximum height error1.37mm and maximum tilt2.54degrees during
that interval. The final geometric surface gap is0.468mm; saved decision states
contain no goal-contact frames. This does not prove absence of every substep
contact. Goal displacement is0.636mm, versus9.863mm for the undamped soft
reference. Reduced simulated goal movement alone is not a calibrated real-world
fidelity measurement. `final_slide_diagnostic.py` retains descriptive measurements
without changing either success criterion.

The project methods explicitly record user-validated233 as ending in contact.
The strict miss remains a miss. `hardware_and_task_check.md` preserves that
requirement and the manufacturer-document check; no numerical material damping
law was found in the checked hardware sections.

Every substep verifies assembled normal/tangent reference rows against the engine function and current Jacobian velocity. Maximum normalized row/target discrepancy across completed runs is 8.43e-06; all nonmutation violation counts are zero. 2 completed identity controls match every archived NPZ array. These forward-only checks do not establish backward/autodiff correctness or calibrated physical material behavior.

The matched elliptic soft gain1 / normal gain2 cap-relative rim errors at
14/16/22/25s are17.76/18.64/40.42/27.91 versus18.26/19.14/31.36/26.87px.
Carry alignment is nearly unchanged; the22s release image is closer but remains
substantially mismatched. The gain2 rigid errors are16.54/17.52/33.12/60.46px.
Camera hypotheses and annotations are unchanged and no object pose is fitted
to improve the result. Both new rigid and soft overlays were inspected.

`233_normal_damping_real_sim.mp4` covers the complete28.86s trial beside two
real cameras, with proximity pass / strict contact test fails in the title.
All161 frames decode and trace/video hashes are verified; timing differences
are at most16.24/20.87ms. Carry, release, slide and endpoint stills were inspected.
It renders saved poses without rerunning physics. The initial unpublished title
was corrected to avoid claiming absence of all substep contacts. The first
title-correction attempt found no ffmpeg drawtext filter; the completed OpenCV
annotation and H264 verification scripts are preserved.

The broader day-balanced elliptic comparison is prepared in `../elliptic_panel/`.
It separates softness and normal damping across the existing9-demo panel with
four conditions per recording. This expands evidence gathering rather than
relaxing the end-to-end requirement or declaring the calibration near miss a
success. Prior broad validation covered pyramidal contact, not this elliptic
configuration. No new damping level or outcome-selected initial pose is used.
