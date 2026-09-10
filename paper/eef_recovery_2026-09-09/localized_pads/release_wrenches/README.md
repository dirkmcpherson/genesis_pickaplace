# Exact release-force audit: shelf-edge ejection in 113

Both complete replays (113 and 233, elliptic/all-soft) reproduce every archived
array exactly. The observer reads contact forces after solving constraints and
before integration at every original physics substep in the declared windows.
No material, command, placement, shelf or metric changes are made.

In 113, the last loaded finger contact is at 20.1575 s. The first loaded shelf
contact follows at 20.21125 s. During 20.16–20.24 s, the can's horizontal velocity
changes from about -0.026 to -0.508 m/s. The fingers exert no force in this
interval; the shelf supplies about -0.167 N s of horizontal impulse. The center
is initially barely past the shelf's x=0.55 m front edge and then crosses back
outside it. The subsequent whole shelf-contact episode supplies -0.216 N s in x.
The shelf edge, rather than a continuing grip, drives the large backward escape.
`113_edge_impulse.png` shows the sequence. Shelf/table labels follow the unchanged
builder's plane/shelf/pick-table single-geometry creation order (indices0/1/2).

Force signs and timing are checked against momentum change. In the key
20.16–20.24 s interval, the vector discrepancy is approximately
[-0.000011, -0.000047, -0.000002] N s. The successful 233 run has larger
instantaneous acceleration residuals (up to5.93 m/s²), and its20–21 s integrated
discrepancy is about[.00175,.00329,.00339] N s. Its reported contact forces must
not be treated as exact physical measurements. `summary.json` preserves all
balance checks; no solver or sign correction was fitted to produce agreement.

This clarifies why the conditional real seating difference matters: tool+z is
mostly horizontal here, so greater real seating distance toward the fingertips
can add shelf-edge clearance. It need not explain a vertical height difference
to affect whether the released can lands safely. It does not prove that a pad
change will produce the right seating or that the existing shelf position is
perfectly calibrated.

A closer real-video check also corrects the earlier provisional statement that
the can was already settled at20.04 s. Its lid moves down about26 px relative to
the opening hand by20.7 s (`../g14_release_diagnostic/113_real_drop_check.png`).
That is consistent with a brief drop/settling motion; exact support onset and
metric drop height are not measured. No shelf-height change is justified by it.

An independent unheld-can rim fit under the current table-to-base alignment
places initial XY17.24 mm forward and5.41 mm sideways from the archived
outcome-fitted position. It is conditional on that alignment and the camera
model; all declared sensitivity cases span x0.4246–0.4429 m. The original is
x0.4163 m. `../vision_initial_probe/plan.json` freezes a diagnostic comparison at
the nominal image-only estimate before any changed-position outcomes. These
conditional runs do not replace the source corpus or establish a soft-pad gain.

`113_bag_topics.json` confirms that the local113 bag does not carry CameraInfo.
The local recording configuration lists the same camera-image topics without
intrinsic calibration. This does not establish that no calibration exists
elsewhere; the camera assumptions remain explicit.
