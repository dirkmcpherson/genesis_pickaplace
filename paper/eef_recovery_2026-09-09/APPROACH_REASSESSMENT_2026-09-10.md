# Recovery approach reassessment

The completed evidence does not support making a successful soft-pad model the
project objective. Pads remain a possible physical mechanism, but the objective
should concern faithful reconstruction of the demonstrated task. The current
soft-pad goal is unachieved; this document does not mark it complete or claim a
replacement goal has already been registered.

Proposed objective: **Improve faithful end-to-end reconstruction of the real
demonstrations, especially supported release and the final slide, and identify
recordings whose reconstruction remains unreliable.**

## What the completed comparison establishes

The expanded sample has36 recordings (12 per day), all144 declared replays
completed. Early-day mount corrections remain−9.7° and−19.2°. Original / soft1 /
soft2 pass the supplied slide metric on6/36,6/36,7/36, and the stricter complete
sequence on3/36,1/36,1/36. Softness improves proximity relative to the matching
adaptive rigid controls, but adds no strict completion over them. Trial262's
new completion also occurs with rigid pads. The extra soft2 proximity gain191
moves the goal178.10mm and has no supported release. No configuration is adopted.
See `localized_pads/expanded_panel/FINAL_READOUT.md` for paired/day results.

Progress is mostly better intermediate behavior and diagnosis, not demonstrated
overall e2e recovery. Soft2's failures include23 unsupported/incomplete final
pushes,8 missing supported releases,3 missing pickups and1 lost final retention.
The mechanisms selected to improve the grasp can change release and the can's
pose relative to the later tool path; the final slide cannot be inferred from
grasp appearance alone.

Numerical work was necessary but must remain bounded. Raising CG's iteration
cap100 to1000 removes observed cap exits in the tested CPU source and restores
233 GPU completion. CPU/GPU trajectories still differ up to14.02mm. At cap1000,
8 versus16 substeps still gives different task outcomes despite zero cap exits.
The already launched float64 CG/Newton references may clarify this; they do not
justify another open-ended engine or material sweep.

## Recommended next investigation

1. Close the currently running numerical reference comparison and record a
   reproducibility limitation if it does not produce sufficiently consistent
   trajectories. A passing233 alone is not an adoption criterion.
2. Localize final-slide errors against the real video. Use a small, declared
   sample across both early days and the late day, including retained failures.
   Record real release timing, upright support, can/tool relative position and
   goal motion, with camera/annotation uncertainty. Distinguish inherited
   placement errors from errors accumulated during grasp/carry and errors during
   the push. Five expanded source poses already overlap the shelf.
3. If image constraints permit it, use a release-state reset only as a diagnostic
   to ask whether the recorded final tool motion can produce the observed slide
   from a plausible real release state. Such a reset must never be scored as a
   recovered full demonstration. If uncertain release geometry spans different
   outcomes, retain that ambiguity instead of choosing the state that succeeds.
4. Select the next intervention from that diagnosis, then rerun complete source
   episodes with matched controls and broader verification. Prioritize recorded
   motion/scene alignment or release mechanics only when independently supported;
   retain pads as a secondary hypothesis.

Existing checks constrain this work: full-corpus joint/tool consistency finds
no centimeter-scale constant tool-transform error (`cartesian_consistency/`),
raw measurement timing did not add successes in its four-trial pilot
(`raw_measurement_clock/`), and conditional image-derived113 placement improved
seating without recovering the task (`localized_pads/vision_initial_probe/`).
Do not repeat those approaches or present them as untested fixes.

Keep the supplied thresholds, original motion endpoint and all failures visible.
No og4, extra push/hold, success-fitted goal movement or pose correction. Any
scene correction must come from independent physical/image evidence and be
validated across complete trajectories. A realistic-looking video, more GPU
lanes, a closer final can or an isolated successful recording does not establish
the proposed objective.

The user has asked to reconsider the approach. No further pad sweep is queued;
only the previously launched numerical reference cases remain running at this
reassessment. Choosing a mechanism in advance was too restrictive, and more
parameter trials alone are not the next justified step.

The final numerical reference process has now terminated successfully: float32
control and both float64 references saved complete233 sequences. Their detailed
physical/numerical comparison remains to be reviewed; this single-source result
does not change the expanded material conclusion. No simulation jobs remain live.
