# Pads on the original recovery hand: attribution control

All six calibration replays completed. Every unchanged-material (.02 s) control
reproduces **every saved array** of its original full-world reference exactly.
Softened runs preserve source EEF actions, joint commands, gripper feedback,
timestamps, early-day yaws and initial placements. Coupling and inertia are the
original reference values; only the spatial pad contact law changes.

| Demo | Original-contact final distance | Soft-pad final distance | Strict result, original / soft |
|---|---:|---:|---|
| 113 | 697.16 mm | 459.73 mm | no supported release / no supported release |
| 184 | 92.51 mm | 65.96 mm | no supported push / no supported push |
| 233 | 65.815 mm | 65.797 mm | complete / complete |

The 184 soft case crosses the supplied metric threshold, but it is **not a
recovered final slide**. It contacts the goal during carry (from about 21.1 s),
releases on the shelf at 25.02 s with center distance 71.94 mm, and then gains
only about 7.24 mm at best. Its can rests on the opposite side of the goal
(y about -.2695 m). The unchanged strict scorer fails. The real videos still
show the can in the hand at 27 and 33 s; at 40 s the real hand is withdrawn.
`184_real_release_check.jpg` contains clock-matched recorded camera 4/0 views
at 24/27/33/40 s (source time offset .044665098 s). Thus the better endpoint is
accompanied by an early-release and placement mismatch.

In 233, pad contact preserves strict completion. Existing camera-hypothesis
cap-relative rim errors with soft pads at 14/16/22/25 s are:
31.41 px, 28.80 px, 15.46 px, 28.56 px.
Original fixed values were 29.29/25.86/15.59/26.99 px. The fixed coupling's
loaded-curl limitation remains. This control does not establish the requested
end-to-end fidelity improvement and is not adopted.

The more promising next diagnostic is the independently verified generic
finger-inertia issue (`../unloaded_armature/README.md`). No new reserved trials
were used for fitting this original-hand control.
