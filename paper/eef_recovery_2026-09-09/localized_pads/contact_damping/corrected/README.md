# Completed damping comparison: candidate rejected

All ten corrected full replays are complete. Added damping yields no
end-to-end improvement: soft and matching rigid each complete 0/3 original-
placement calibration demos under both the supplied metric and strict
sequence. Original fixed coupling completes 1/3. Both conditional113
treatments also fail supported release. No pad, engine or pose is adopted.

| Trial and placement | Pads | Final center distance (m) | Final goal movement (mm) | Strict failure |
| --- | --- | ---: | ---: | --- |
| 113 conditional | rigid | 0.3493 | 59.78 | no_supported_release |
| 113 conditional | soft | 3.4653 | 60.75 | no_supported_release |
| 233 original | rigid | 0.9406 | 0.03 | no_supported_release |
| 233 original | soft | 0.8910 | 0.03 | no_supported_release |
| 113 original | rigid | 34.2052 | 58.77 | no_supported_release |
| 113 original | soft | 13.6405 | 59.67 | no_supported_release |
| 184 original | rigid | 24.0635 | 26.09 | no_supported_release |
| 184 original | soft | 16.6381 | 2803.33 | no_supported_release |

The undamped soft233 identity still completes, exactly matching its archived
trace (65.99mm center distance, 9.86mm goal movement). Both damped233 variants
carry through18s, retain simultaneous shelf/hand contact around22s, then tip
by24s. The original fixed233 reference completes with65.81mm center distance
and1.00mm goal movement; its different hand and engine remain disclosed.

Conditional113 rigid previously settled upright but failed the later push.
At doubled damping, both pad variants briefly encounter the shelf and fall
off. `../conditional_release_comparison.png` compares all four saved trajectories
and was visually inspected. The treatment acts throughout approach and carry,
so it also changes the configuration from which opening starts. Preserving
the elastic coefficient at fixed penetration does not preserve that history.

Every treated contact checks the executed engine reference-acceleration function. Maximum static-term relative error is 0; maximum error across static/normal-velocity/tangent-velocity checks is 1.13e-07. Time constants remain above the2.5ms safety floor. Geometry, friction coefficient, hand parameters, early yaw, full source commands and scoring are unchanged. Paired collision policies match.

All four identity executions (two preserved from the first implementation
attempt and two in the corrected batch) reproduce every archived NPZ array.
There are12 completed full replays in this investigation. Eight separate
first-attempt treatment jobs failed compilation before stepping and are retained
as implementation failures, not full demonstrations or recovery failures.

The tested intervention doubles both normal and tangential reference damping.
Its rejection does not settle whether normal-only damping could help. The
source inspection in `../normal_only_capability.json` identifies a possible
isolated constraint-row test; it is not implemented or validated. Any such test
must verify row mapping, exact identity controls and untouched tangent response
before full-task interpretation. Neither a new friction coefficient nor a
success-selected placement is justified by this negative result.
