# Metric-of-record reconciliation

The user supplied `can_pos_recovery/slide_predicate.py` on September 9. It is now
the slide metric of record for this recovery work. Its constants and classifier
are unchanged. `eef_task_sequence.py` remains a separate, stricter physical-contact
diagnostic; its historical 10/74 timestamp result must not be called the result of
the supplied predicate.

All 21 December 18 timestamp qualifiers now have exact independent EEF replay
and selected-phase visual review, and are packaged in
`baselines/demos_eef_recovery_2026-09-09/manifest_slide_metric_timestamp.json`.
The validation file beside it checks all hashes, finite action/observation arrays,
N+1 observation alignment and complete coverage of the 21 qualifiers (43,050
action frames). Videos are retained; visual review sampled six phases per trial.
This completes packaging of that cohort, not the broader recovery objective.

Updated early-day snapshot: 42/89 scored, released 42, pushed 38, arrived 2.
Later snapshot: 54/89 scored, released 54, pushed 50, arrived 3. Trial 176 is the
third verified and reviewed early-day package. It additionally passes the strict
contact diagnostic: 48.90 mm goalward movement with 99.68 percent support,
upright hand-separated endpoint, final center distance 65.836 mm.
December 16 trials 162 and 167 qualify with 65.877 and 80.917 mm final center
distances and upright, hand-separated endpoints. Neither initial position overlaps
the modeled shelf. Both now pass exact independent EEF replay and selected-phase
visual review and are packaged in `manifest_early_slide_metric_timestamp.json`;
both retain archived outcome-fitted initial-position provenance. Trial 167 is
only 0.083 mm inside the fixed arrival threshold. Trial 162's strict diagnostic
does not establish a separate supported recontact-and-slide interval, despite
its metric pass and near-goal endpoint. These qualifications remain in the review.

The exact script reproduces the original 64 tapes: released 64, pushed 64,
arrived 15, slide_success 15. See `original64_reproduction.json`.

## Schema and clock mapping

The original recorder uses repeat four at 30 ms per environment step: 120 ms per
stored state. `score_recovered_slides.py` samples recovered traces at 120 ms,
maps post-action observations to states, and uses saved measured URDF tool xyz.
The final actual observation is retained, with a disclosed shorter last interval
where needed. This avoids silently shortening the predicate's ten-frame window
on the 30 ms or experimental 10 ms traces. It does not add a terminal settle.

The recorder's `tipped` is a 60-degree, open-gripper-gated termination flag.
Recovery stores continuous physical tilt, not that termination state machine.
The primary mapping marks physical tilt above the same 60-degree threshold;
the parallel legacy proxy also requires grip command below 0.3. Both results are
retained. This proxy is not an exact reconstruction of termination behavior.
There is no gripper term added to the release or push conditions.

## Results

| Cohort | Scored | Released | Pushed | Arrived / slide_success |
|---|---:|---:|---:|---:|
| Original processed human tapes | 64 | 64 | 64 | 15 |
| December 18 fixed-clock EEF | 74 | 72 | 72 | 19 |
| December 18 timestamp EEF | 74 | 74 | 74 | 21 |
| December 16/17 yaw-corrected timestamp EEF, partial | 27 | 27 | 23 | 0 |

The last three rows use physical tilt mapping. The legacy tip proxy additionally
admits timestamp trial 295, which ends physically tipped, and can differ on other
intermediate flags. Inspect the recorded mapping disagreements rather than mixing
these numbers. Original64 is a different processed cohort from the 74 direct
reconstructions and is not a paired baseline.

Compared with fixed-clock EEF, timestamp reconstruction gains 256, 309, 317, 328,
330 and loses 244, 297, 298 under the primary mapping: a net gain of two. The fixed
pool retains historical bad initial orientations for 234/318; neither passes this
metric. Timing and interpolation both change, so this is not timing alone.

These are rescoring results, not eleven newly simulated physical-contact
recoveries. Ten timestamp passes already have independent saved-action replay;
`verify_additional.py` checks the eleven additional metric passes against their
saved EEF actions. No tape is automatically admitted by this report. Visual
review, initial-condition provenance, final distance, physical tilt and solver
contacts remain available for judging what each reconstruction achieves.

## Release-order audit

The unchanged predicate can mark a stationary initial can as released while the
hand approaches it. In the 162-completed-trace snapshot (74 December 18 and 88
early), every metric pass receives its first release before the legacy picked
flag: all 21 December 18 and all nine raw early passes. Thus carrying after that
initial release can satisfy the 10 mm gain clause. This does not prove those
trials lack a later slide; it means the predicate alone does not establish one.
`release_order_audit.json` retains source hashes, mapped event frames, can height
and solver contacts. The legacy picked flag is imperfect, particularly with
initial shelf overlap. Thresholds and classifier remain unchanged. Use the
separate contact diagnostic and visual evidence for physical sequence claims.
