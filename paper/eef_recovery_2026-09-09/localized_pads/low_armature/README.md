# Lower added finger inertia: complete calibration readout

Seven runs completed: one exact old-inertia reproduction control and six paired full-task comparisons. Lower inertia corrects a demonstrated unloaded dynamics problem but does not rescue the two early grasps. No general soft-pad recovery improvement is established.

| Demo / pad time constant | Final center distance | Supplied metric | Strict sequence |
|---|---:|---|---|
| 113_tc0.02 | 314.50 mm | fail | not_picked |
| 113_tc0.03 | 304.22 mm | fail | not_picked |
| 184_tc0.02 | 208.47 mm | fail | not_picked |
| 184_tc0.03 | 212.63 mm | fail | not_picked |
| 233_default_control | 179.33 mm | fail | no_supported_push_to_contact |
| 233_tc0.02 | 66.89 mm | pass | complete |
| 233_tc0.03 | 71.44 mm | pass | no_supported_push_to_contact |

Both early demos remain not-picked under both contact settings. In 233, the original-contact hand now completes strict goal contact (it previously failed at 179.33 mm final center distance). Soft contact still passes the supplied metric but lacks strict goal contact: 71.44 mm center distance, 5.44 mm surface gap. Thus inertia correction helps 233 independently of pad softness.

233 image comparisons use the same pre-existing camera hypothesis and annotations. Original-contact cap-relative rim errors at 14/16/22/25 s are .68/4.05/24.83/31.44 px; soft-pad errors are 4.27, 8.45, 27.00, 33.26 px. Loaded seating improves substantially over the original fixed-coupling hand (29.29/25.86 px), while placement mismatch remains.

The small added inertia produces peak substep joint velocities of roughly 10–18 rad/s in the task replays, despite much smaller ramp-average motion. Source gripper targets are held over each 30 ms action interval; the unloaded bench uses 10 ms updates. These transients passed the existing stability guard but are not validated hardware joint speeds. Numerical regularization, actuator bandwidth and loaded forces remain uncertainties.

The next preset is derived from the mass-inclusive internal-mode dynamics: return stiffness .9 (critical-damping estimate .880) and actuator force cap 4, retaining all other lower-inertia parameters. Its independent unloaded bench reduced peak joint error to 2.12° and worst hold-end error to .0019°. The increased cap permits the modeled return torque at the previous image-consistent loaded curl; it is not hardware force calibration. Full calibration comparisons are separate in `../critical_return/`.

No reserved 176/185/237 outcome was used to select these new mechanics. No source command, yaw, initial placement, timestep, metric threshold or training bank changed.
