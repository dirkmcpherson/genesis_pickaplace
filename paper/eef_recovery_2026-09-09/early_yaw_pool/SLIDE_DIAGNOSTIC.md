# Early-day slide diagnostic, September 9

Correction after inspecting initial geometry: trial 131's apparent pickup/release
is invalid. Its can initially intersects the solid shelf, rises through contact
resolution, and later receives the legacy picked flag with zero sampled robot-can
contacts over the whole episode. Its row below describes the trace, not a real
pick/place recovery. `initial_geometry_audit.json` flags 12/89 initial positions
intersecting the current shelf model. These cases require geometric reconciliation,
not permanent exclusion or interpretation as hand-control failures. Real cam0/cam4
samples for 131 are in `131/131_real_pick_review.jpg`.

The user subsequently supplied `slide_predicate.py` as the metric of record.
See `../slide_metric_of_record/README.md`; this document retains stricter contact
diagnostics and does not define the headline slide-success rate.

Snapshot: 18/89 completed full-world timestamp EEF trials, zero complete sequences.
Fourteen are December 16 and four December 17. Six have a supported release but
fail the slide. These are intermediate counts, not the final cohort result.

`audit_slide_failures.py` records contact events, displacement and final state for
every finished slide failure, with trace hashes in `slide_failure_audit.json`.
It does not alter the scorer, physics, initial positions or commands.

| Trial | After supported release | Final surface gap |
|---|---|---:|
| 124 | No hand recontact; can moves at most 0.122 mm | 57.7 mm |
| 131 | No hand recontact; can moves at most 0.033 mm | 123.5 mm |
| 128 | 48 frames of supported hand contact; no goal contact | 78.7 mm |
| 132 | 85 frames of supported hand contact; no goal contact | 76.5 mm |
| 184 | Recontacts, but ends tipped 90 degrees | 26.5 mm |
| 185 | Recontacts; minimum gap 5.53 mm, then moves away | 42.3 mm |

None has solver goal contact after release or supported goal contact before it.
The surface-gap metric is horizontal center distance minus two can radii; it is
not a reliable surface-distance estimate for the tipped final state of 184.
These findings do not identify finger compliance as the cause. They distinguish
missing recontact from unsuccessful pushing and instability during contact.

## Trial 124 visual review

`124/124_sim_phases.jpg` samples six frames of the saved-pose renderer;
`124/124_timed.mp4` retains the full rendering. The late frames show an upright can
on the shelf separated from the goal, consistent with the numerical trace.
Rendering measured poses is not an independent physics replay.

`124/124_real_phases.jpg` samples six frames from archived cam4. The real late
frames show the two cans adjacent and the hand open nearby. Occlusion and sparse
sampling prevent a precise distal-angle measurement or proof of continuous
supported sliding. Camera and bag clocks have not been calibrated against each
other, so these sheets are qualitative phase comparisons, not synchronized pairs.

## Next controlled comparison

`fixed_clock_pair/plan.json` declares trials 124 and 185, chosen to cover absent
recontact and near-goal recontact. Replay their original measurement samples at
30 ms each, matching the historical clock, with the same full world, yaw, fitted
initial positions and precision EEF controller as the timestamp census. This
changes both sampling and timing, not timing alone. It is a diagnostic selection,
not a representative success-rate estimate. No og4, added push or hold is used.
Any successful result still requires independent action replay and visual review.

Both fixed-clock comparisons have now completed without recovery. Trial 124 fails
supported release under the fixed clock (67.2 mm final gap), while timestamp
reconstruction reaches supported release (57.7 mm gap). Trial 185 fails the slide
under both clocks (40.8 mm fixed-clock final gap versus 42.3 mm timestamp gap).
Exact paired results are retained in `fixed_clock_pair/paired_summary.json`.
Neither result justifies reverting the early cohort wholesale to the fixed clock.

The separate `../fine_interval/` experiment now tests 10 ms timestamp sampling on
these two trials and two December 18 controls, preserving physical dwell durations.
