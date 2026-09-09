# Full early-day EEF census

All 89 declared candidates completed with full world hooks, precision EEF IK,
recorded timestamp reconstruction and actual mount yaw readback: -9.7 degrees
on December 16 and -19.2 degrees on December 17. No og4 or added motion.
This is the declared 89-candidate pool, not all 120 early recordings.

| Day | Trials | Raw supplied-metric passes | Passes without initial overlap | Physical slide diagnostic passes without initial overlap |
|---|---:|---:|---:|---:|
| December 16 | 49 | 3 | 3 | 1 (176) |
| December 17 | 40 | 6 | 3 | 1 (225) |

The six qualifying metric trials (162, 167, 176, 202, 224, 225) passed independent
saved-EEF replay and selected-phase visual review. Initial positions are archived
outcome-fitted estimates, not measured ground truth. Original 193, 196 and 222
have initial shelf overlap; their raw passes do not count as valid recoveries.
A separate 2.356 mm geometric correction to 193 is independently verified and
packaged with its own provenance, outside this unchanged-IC census.

Trial 225's strict event starts are release 23.61 s, push 24.03 s, supported goal
contact 24.96 s. It advances 19.23 mm with 94.12% shelf support. The six broad
phase samples alone missed that short event; the event-focused review is separate.
The local 225 bag has no camera/image topics and no separate camera files.

There are 32 tipped endpoints on December 16 and 17 on December 17. Among cases
without initial shelf overlap, 23 and eight respectively first tip before any
upright shelf support after the picked flag. Therefore many failures arise before
the final slide. This ordering does not identify finger compliance or any other
specific physical cause. `final_failure_funnel.json` retains every event record.

The supplied metric remains unchanged. Its release can precede pickup, allowing
carrying to satisfy gain; metric passes alone do not establish the requested
physical sequence. Separate support/contact evidence is necessary.

Video export correction: phase images were valid, but the batch renderer's fixed
writer dimensions produced empty MP4s. The 224/225 and December 18 batch videos
were regenerated with explicit resize and decode checks; all 23 repaired videos
now decode at the first and final frames. Independent physics
traces are unaffected. The previously delivered 176/202 side-by-side videos decode
correctly. See `../slide_metric_of_record/empty_video_export_correction.json`.
