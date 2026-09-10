# Mass-informed return spring: calibration readout

This candidate is frozen for expanded validation. It preserves full supported-slide completion in 233 and improves its image checkpoints relative to the matching rigid-contact hand. Both early calibration soft cases still fail pickup. It is not yet a general recovery improvement.

The hand uses actuator stiffness 320, generalized force cap 4, distal moment ratio .5, return stiffness .9, proximal/distal damping .05/.002, added finger inertia .0001 kg m², and the previously disclosed mirrored -1.03 rad distal range. The soft pad retains the original geometry, a 3 mm maximum normal layer depth and .03 s time constant; the paired rigid-contact control is .02 s.

Return stiffness was rounded from the unloaded internal-mode critical-damping estimate .880. The force cap permits the modeled return torque at the previous image-consistent curl; it is not a hardware force measurement. The independent unloaded bench reduced peak joint error to 2.12° and worst hold-end error to .0019°. High substep velocity peaks remain an actuator-bandwidth and numerical-fidelity limitation.

| Demo / pad | Final center distance | Supplied metric | Strict sequence |
|---|---:|---|---|
| 113_tc0.02 | 234.12 mm | fail | not_picked |
| 113_tc0.03 | 234.73 mm | fail | not_picked |
| 184_tc0.02 | 91.81 mm | fail | no_supported_push_to_contact |
| 184_tc0.03 | 197.33 mm | fail | not_picked |
| 233_tc0.02 | 65.84 mm | pass | complete |
| 233_tc0.03 | 67.40 mm | pass | complete |

In 113 both treatments knock the can away during closure, before lift. The real camera shows it staying centered at 5.4/5.85/6.3/7.8 s (`113_real_approach.jpg`). This localizes the failure to approach/contact geometry or grasp mechanics; it does not identify which initial-pose or material parameter is wrong. In 184, rigid contact restores pickup and supported release, but soft contact fails pickup.

In 233 the soft case releases at 22.17 s, begins its qualifying push at 24.84 s and reaches sustained goal contact at 25.80 s. Goalward push is 23.77 mm with 100% sampled upright shelf support over the scored interval. The final surface gap is +1.405 mm, inside the unchanged strict 2 mm tolerance. **There is no sampled goal contact at the final frame**: strict completion must not be described as continuous physical contact retention.

| 233 camera checkpoint | Matching rigid error | Soft-pad error |
|---|---:|---:|
| 14 s | 10.26 px | 7.47 px |
| 16 s | 8.95 px | 5.97 px |
| 22 s | 24.30 px | 21.17 px |
| 25 s | 34.27 px | 29.54 px |

These are cap-relative rim errors under the same existing camera hypothesis, not calibrated 3D positions. The original fixed-coupling hand had 29.29/25.86/15.59/26.99 px errors: the new hand improves loaded seating while its placement checkpoints remain worse than that original reference.

During carry, median/p95 maximum-per-contact force is 5.18/7.91 N rigid versus 5.41/7.45 N soft; overlap median/p95 is .551/.654 mm versus .707/.833 mm. These are solver quantities on different contact populations, not total grip force or measured pad strain.

`../233_critical_soft_pad_real_sim.mp4` is the inspected, clock-matched review video. All 321 frames decoded. Fresh saved-action replay reproduces every array exactly (`../critical_validation/summary.json`, verifications).

Expanded validation freezes nine recordings: old reserved 176/185/237 plus source-signal-selected 156/181/224/198/243/246. One new lower-closure and one higher-closure source per day were chosen with a fixed random seed; no simulation outcome was used in selection. See `../critical_validation/selection.json`. Include all failures/regressions; do not fit on this panel. No source motion, thresholds, world placements, yaw or training banks were changed.
