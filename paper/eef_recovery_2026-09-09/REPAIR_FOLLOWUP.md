# Additional EEF recoveries — September 9

**Fidelity correction (September 9):** The default development manifest contains **eight
unmodified-command reconstructions**. Trial 242's outcome-fitted initial pose is separate in
`manifest_fitted_initial_pose.json`. Three added-motion or grip/timing records (242, 275, 308)
are separate in `manifest_modified_commands.json`. The historical 11-source total is **not a
faithful-recovery count**. Og4 accelerates recorded opening; it is a diagnostic command modification,
not a fidelity fix or a passive-finger model. Unmodified commands alone do not validate timing,
estimated initial poses or contact physics. Files remain in place; select through a manifest,
not a directory glob. `manifest_mixed_historical.json` preserves the old packaging for provenance.


Two additional source trials now have independently verified repaired episodes: **275 and 308**.
The bank contains **11 independent source trials**, 11 primaries and one alternative of 242.
Eight primaries are unchanged-motion census conversions; 242, 275 and 308 use disclosed repairs.
None of these additions changes the frozen training sets or establishes a new source-comparison result.

## Trial 275: longer final push

The same eight near-contact candidates from the 10 mm sweep were tested with a 30 mm maximum.
Only the bound changed; the default remains 10 mm. The controller still stops at three contact
frames or loss of support, optionally holds for at most 0.3 seconds, then resumes the original path.

| UID | Added travel, mm | Result |
|---|---:|---|
| 237 | 14.0 | Qualified contact, final arrangement not retained |
| 242 | 6.5 | Complete; already recovered source |
| 247 | 18.0 | Qualified contact, final arrangement not retained |
| 248 | 7.0 | Contact diagnostic fires, full support/motion rule fails |
| 251 | 30.0 | No qualifying push/contact |
| 275 | 12.5 | **Complete, independently verified and packaged** |
| 297 | 9.5 | Contact diagnostic fires, full support/motion rule fails |
| 302 | 30.0 | No qualifying push/contact |

Trial 275 preserves its initial objects, original grip, and physics. It adds 25 half-millimetre
tool increments (0.75 seconds) during the final slide. A fresh saved-action replay reproduces
trajectory, actions and observations bit-for-bit. Inspected phase frames show pickup, shelf release,
a later push into contact and the final upright cans after robot contact ends.
[Video](slide_repair_30mm/275/275_repaired.mp4), [phase sheet](slide_repair_30mm/275/275_repaired.jpg).

The original absolute-path resumption subtracts the inserted offset in one action. On 237 and 247,
a separate declared probe removed only that subtraction and continued the recorded EEF deltas,
leaving the suffix translated by the added displacement. **Both still fail final retention.**
Their identical pre-change prefixes and retained outputs support a negative result, not adoption
of a new continuation rule. Plans and all outcomes are in `slide_repair_30mm/` and
`slide_relative_suffix/`.

## Existing og4 release correction in the EEF path

The September-3 [release audit](../RELEASE_OPEN_PREREG_2026-09-03.md) adopted a recorder-side
opening correction. Selecting its variant does not itself filter grip: `HumanFollower` applies it.
The earlier 74-trial EEF census deliberately preserved the raw grip stream, so **8/74 is a raw
EEF control, not the project's best previously adopted recorder's recovery rate**.

The new pilot reuses `HumanFollower._filter_grip` directly, with gain 4 and deadband 0.05, once
per 0.03-second tape waypoint. That cadence differs from the older recorder; this is a new EEF
test, not a claim of equivalence to its old census. Arm targets, initial poses and physics are
unchanged. The base and og4 physical variant dictionaries are identical after removing the
recorder-only gain. Filter identity, source hash, changed-frame count and original grip arrays
are retained. This is a grip correction, not a calibrated passive-finger model.

| UID | Raw EEF control | EEF with og4 |
|---|---|---|
| 235 | No upright supported release | Released; no qualifying final slide |
| 243 | No upright supported release | Released; no qualifying final slide |
| 245 | No upright supported release | Released; no qualifying final slide |
| 257 | No upright supported release | Still no supported release |
| 308 | No upright supported release | Qualified release/slide/contact; terminal support cutoff fails |
| 233 | Complete | Complete |

This is a deliberately selected development cohort, not a held-out success estimate. Four of five
selected placement failures gain supported release, and the successful control remains complete.
Applying the 30 mm-bound slide repair to filtered 243 still does not complete the sequence.

## Trial 308: recording boundary and retention

The og4 trace ends while settling at 7.918° tilt, with shelf, robot and goal contacts. Its centre
height is 4.000015 mm above the resting-height reference—just beyond the declared 4 mm cutoff.
That makes the scalar verdict boundary-sensitive; it is not evidence of an irrecoverable physical
failure. The scorer was **not** changed.

A declared **0.3-second stationary hold** appends ten zero-EEF-delta actions at the same final
grip. The entire earlier trajectory remains bit-identical. It passes the complete-sequence rule;
a second fresh saved-action replay reproduces trajectory, actions and observations exactly.
The packaged endpoint still has about 7.3° tilt and robot contact. A separate three-second hold
retains the sequence and reduces tilt to 2.64°; this is evidence of continued settling and
retention, not full hand withdrawal or exact rest. The bank record carries this limitation.
[Video](eef_og4_pilot/308/terminal_hold/308_repaired.mp4),
[phase sheet](eef_og4_pilot/308/terminal_hold/308_repaired.jpg).

`308_grip_timing_repaired.npz` is explicitly grip- and timing-repaired. Its og4 recording variant
builds the same physical world as w3. **Do not apply og4 again to the saved actions.**

## Reusable path and validation

`prepare_eef_og4.py` creates a filtered kinematic source while preserving the raw grip array.
Its output exactly matches the tested pilot input. Example, into fresh directories:

```bash
PY=/home/james/workspace/genesis_sim2real/venv/bin/python
$PY can_pos_recovery/prepare_eef_og4.py \
  paper/eef_recovery_2026-09-09/full_pool/233/source.npz --out /tmp/new_233_og4_source
$PY can_pos_recovery/repair_eef_slide.py /tmp/new_233_og4_source/source.npz \
  --max-extension 0 --polish-ik --out /tmp/new_233_og4_replay
```

The replay runner now carries grip/timing/offset provenance through verification, preserves the
generation's hold/splice metadata, and distinguishes first execution of an unexecuted action plan
from independent replay of an actual trace. A three-frame real control test verifies the plan
role; the full 308 replay verifies the independent-trace role. It captures code hashes before
stepping. Historical runner versions are retained by hash in `code_snapshots/`.

All 12 packaged episodes load without pickle, contain finite correctly paired arrays, and match
their manifest hashes. Existing eight adversarial scorer tests still pass. The broader recovery
goal remains open: most slide repairs fail, og4 has only this small EEF cohort so far, and the
real hand's passive adaptation remains uncalibrated.
