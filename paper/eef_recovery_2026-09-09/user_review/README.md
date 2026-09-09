# Annotated real/sim review

Start with [176 real/sim video](176_real_sim_annotated.mp4): exact EEF replay and
physical pick–place–slide diagnostic pass. The supported release starts at
36.42 s, push at 36.51 s and sustained goal contact at 64.29 s in simulation.
The diagnosed push advances 48.90 mm with 99.68% shelf support. The real video
uses elapsed time with no stretching; camera-to-bag offset is uncalibrated.
The final real can is partly occluded. Check grasp seating, placement and the
late hand approach; this does not establish quantitative real finger angles.

Then view [202 real/sim video](202_real_sim_annotated.mp4): exact EEF replay and
supplied proximity metric pass, but no independently established supported
post-release slide. Final simulated center distance is 66.60 mm with hand contact.
Real footage is the embedded 96×96 camera, matched by bag receipt timestamps;
hardware acquisition delay remains unknown. Check whether the can is placed
near the goal while still held, and how the subsequent hand approach differs.

Real imagery is on the left, rotated 90 degrees counterclockwise for viewing,
with no crop. Simulation is on the right. Distance, tilt, solver contacts and
physical diagnostic events describe simulation only. Videos use existing measured
pose renders, not new physics runs; independent replay evidence is separate.
The supplied slide metric can mark release before pickup, so its pass is not
by itself evidence of a supported slide. No thresholds were changed.

Companion comparison JSONs retain per-frame timing and source trace hashes.
Phase sheets were inspected; complete videos have not been watched frame by frame.

[Trial 225 event video](225_slide_event_annotated.mp4) adds a Day 2 physical slide
pass: inspect 23.6–25.3 s for release, recontact and goal contact. It advances
19.23 mm with 94.12% support. Event-focused frames were reviewed and video decoding
checked. No real camera is available in its local recording.

[Trial 113 failure comparison](113_real_sim_grasp_failure.mp4) shows a direct
grasp-retention mismatch: real selected frames retain the can, while simulation
drops it around 13.4 s under steady recorded grip. First 18 seconds only, matched
using per-camera-frame epoch timestamps. Maximum nearest-frame clock mismatch is
18.05 ms; acquisition delay is uncalibrated.
Six phase frames reviewed; video decoding checked. This does not quantify passive
finger curl or establish why the simulated grasp fails.

[Trial 113 changed-contact comparison](113_real_sim_contact_probe.mp4) extends the
review through 30 s. Inspect 19–21 s: the real can remains upright while the
changed-contact simulation briefly lands at the shelf edge and falls off. This
is an explicitly labeled physics experiment, not an admitted recovery. The event
reveals a placement mismatch after retaining the early grasp; it does not by
itself identify the correct shelf geometry or gripper compliance. Six selected
phase frames reviewed and output decoding checked.
