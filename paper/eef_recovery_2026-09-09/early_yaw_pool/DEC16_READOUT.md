# December 16 full-world timestamp EEF result

All 49 declared December 16 candidates have finished collection. The mount yaw is
-9.7 degrees, with the full world hooks applied and read back before execution.
No og4, added push, terminal hold or new initial-position search is used.

Three pass the supplied `slide_predicate.py` through the disclosed 120 ms adapter:
162, 167 and 176. All three pass exact independent EEF action replay and
selected-phase visual review and are packaged in
`baselines/demos_eef_recovery_2026-09-09/manifest_early_slide_metric_timestamp.json`.
Their initial positions remain archived outcome-fitted estimates, not ground truth.

| Endpoint / diagnostic | Trials |
|---|---:|
| Supplied slide metric passes | 3 |
| Final physical tilt above 60 degrees | 32 |
| Final upright shelf-supported state, but center distance above 81 mm | 11 |
| Initial cylinder intersects the current solid shelf model | 6 |
| Legacy picked flag never fires | 4 |

These categories overlap and are not an additive partition. In particular, the
initial-overlap cases can receive spurious picked flags, as demonstrated by 131.
The stricter contact diagnostic has one complete sequence (176), 17 slide failures,
27 without supported release and four without a picked flag. That diagnostic is
not the supplied slide metric of record.

Among the 32 tipped endpoints, five have initial shelf overlap. Of the remaining
27, twenty-three first exceed 60 degrees before any upright shelf-supported state
following the picked flag; four tip after such support. Thus this day's failures
are not predominantly an upright can simply stopping short on its final push.
This event ordering locates the problem; it does not establish finger compliance,
world geometry or arm tracking as its cause.

`failure_funnel_by_day.json` retains each endpoint and event frame. Support here
means solver shelf contact, center within 4 mm of the modeled shelf rest height,
and tilt below 20 degrees. These are secondary diagnostics, not changes to the
supplied slide thresholds. `initial_geometry_audit.json` retains all overlap flags.

For comparison, the separate December 18 timestamp cohort has 21/74 metric passes
(versus 19/74 under its fixed clock), all now independently verified, reviewed and
packaged. It is a different recording day and must not be pooled into a claim
about correcting December 16 yaw. The 40-candidate December 17 census is ongoing.
