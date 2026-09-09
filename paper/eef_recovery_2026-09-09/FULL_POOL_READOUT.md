# Full-pool EEF recovery — September 9

## Completed readout

All **74/74** declared trials completed, retaining **148,736 action steps** and every failure.
There are no pending or failed worker executions after resuming the interrupted jobs.

**Scope clarification:** this is the unmodified-grip control for the new EEF path, not the
project's best previously adopted recorder. The September-3 `og4` opening correction is applied
inside `HumanFollower`, not by selecting its world variant alone, and was deliberately absent
from this raw-motion census. Its 8/74 result must not be presented as the best existing project
recovery rate. The next EEF probe explicitly applies and records that correction.

| Physical development filter | Trials |
|---|---:|
| Complete pick–release–slide sequence | 8 |
| No pickup | 5 |
| Picked, but no upright supported release | 25 |
| Released, but no qualifying supported push into contact | 35 |
| Qualified contact, but final arrangement not retained | 1 |

The eight complete unchanged-motion trials are **232, 233, 255, 273, 294, 299, 300 and 305**.
All eight independently reproduce their measured trajectories, actions and observations exactly;
all eight passed phase-frame visual review and entered the bank. Together with the previously
rescued 242, the bank has **nine independent source trials**, nine primary episodes and one
explicitly motion-repaired alternative of 242. The alternative is not another independent demo.

**EEF command equivalence is established for this pool:** 73/74 episodes have bit-identical
float32 joint commands to their raw tapes; the remaining episode is 294. Across all 74 episodes,
maximum unwrapped joint-target difference is 1.124×10⁻⁹ radians. This audits commands, not actual
tracking error or real calibration. Replay/controller/scorer hashes match for every trial, including
the resumed jobs. [Numerical validation](full_pool/numerical_validation.json) and
[per-trial command audit](full_pool/joint_target_equivalence.json) retain the checks.

Of the 35 failures after release without a qualifying push/contact, 24 never contact the goal
after release, three never re-engage the robot, and eight contact the goal but fail support or
motion requirements. These categories identify different work; they are not all finger-curl failures.

## Scope and interpretation

This pass replays the declared 74 December-18 source tapes through the precision EEF controller
in the existing `gc_kp4_riser3_shelf6` world. Both variant hooks run. Archived initial positions
and can quaternions are preserved, including initially lying cans. Recorded arm/grip motion is
unchanged. This is the existing fixed 0.03-second tape clock, not timestamp-faithful reconstruction.

CONFOUNDS 51 already identifies the lying-can initial quaternions for 234 and 318 as corrupt
recovery artifacts: their real videos begin upright. They were retained here for a controlled
comparison, not endorsed as ground truth or counted as inherently unrecoverable real demos.

The pool includes failures and is not a hand-labeled set of 74 successful real tasks. The physical
sequence filter is the separately declared development scorer, not the frozen paper evaluator.
Consequently, its counts must not be substituted into published learner comparisons or interpreted
as the fraction of successful real trials recovered. No final-gap-only or feedback-only proxy is
used to admit complete episodes.

Authoritative artifacts: [plan](full_pool/plan.json), [per-trial readout](full_pool/summary.json),
[execution records](full_pool/execution.json), [runtime](full_pool/runtime.json), and
[visual review](full_pool/visual_review.json). Every completed trace is retained, including failures.

## Recovery versus conversion

The development bank contains one primary per admitted source UID. A complete unchanged-motion
census conversion is useful EEF data; it is not automatically a newly rescued joint-replay failure.
Trial 242 remains the distinct rescue demonstrated earlier: +10 mm initial x correction preserves
the recorded path, and precise IK restores its successful joint-replay trajectory. Its alternative
with an extended push remains explicitly motion-repaired and does not add another source UID.

Every admitted census episode must reproduce its measured trajectory, actions and observations
bit-for-bit in a fresh process using saved EEF actions. It must also pass the sequence filter and
phase-frame visual review. Videos are rendered from saved measured poses, not independent physics
evidence. `admit_verified_eef.py` enforces the replay/array checks against an explicit visual-review
list. Bank [manifest](../../baselines/demos_eef_recovery_2026-09-09/manifest.json) and
[array/hash validation](../../baselines/demos_eef_recovery_2026-09-09/validation.json) carry the count
and provenance. All inspected phase sheets and full measured-pose videos are retained.

## What the failures mean

The readout distinguishes no pickup, no supported upright release, no qualifying push/contact,
and contact that is not retained. Additional diagnostic labels distinguish no robot re-engagement,
no later goal contact, and goal contact that fails the support or motion rule. These diagnostic
labels explain the same outcomes; they do not change acceptance thresholds.

Trial 235 is a concrete real/sim mismatch. The real video shows upright placement, release, slide
and withdrawal. The simulator topples the can during placement: by tape frame 550 its tilt is 90°
with no robot contact. Its `no_supported_release` label means no qualifying upright release,
not that the fingers never open. See [real overview](passive_probe/235_real_overview.jpg) and
[simulation phases](full_pool/235/235_failure.jpg). Camera and simulation clocks are not aligned.

Trial 248 reaches the goal, but rocks and remains tilted near the hand; it fails the conservative
support/retention filter. The [slide diagnostic](full_pool/248/248_slide_diagnostic.mp4) and
[phase sheet](full_pool/248/248_slide_diagnostic.jpg) keep this visible. Such cases must not be
described as simple misses or declared permanently unrecoverable from the scalar verdict alone.

## Final-slide repair tests

The [common repair sweep](slide_repair_pool/plan.json) records a disposition for every census UID.
Eligibility requires a failed sequence, absolute final gap at most 12 mm, and a supported robot
contact after release with gap between 2 and 25 mm. The existing controller inserts up to 10 mm
of half-millimetre pushes toward the live goal at the last eligible contact, optionally holds the
target for up to 0.3 seconds, then resumes the recorded path. Initial objects, grip and physics
remain fixed. All attempted outcomes are retained in [execution.json](slide_repair_pool/execution.json).

The sweep accounts for all 74 UIDs and attempts eight: **237, 242, 247, 248, 251, 275, 297, 302**.
Only **242** completes, using a 6.5 mm extension in this precision-IK run. That is the same already
rescued source UID, not an additional recovery. This new motion-repair version is retained as an
experiment; it does not replace the independently verified alternative already in the bank.

A second declared probe on 247, 251 and 275 inserts the same bounded push at an earlier stable
contact while recorded tool motion is forward. **None of the three completes.** The earlier
insertion does not establish a fix; [plan and outcomes](slide_repair_early_contact/plan.json) are retained.
Neither a reduced surface gap nor the repair loop's `contact_reached` diagnostic substitutes for
the full physical-sequence test.

## Gripper and engine

The [separate distal-compliance probe](PASSIVE_FINGER_PROBE.md) removes direct distal PD actuation
and softens only the two mimic equalities. Both settings preserve 233's completion but neither
rescues 235. Maximum independent distal motion rises to 2.9° on 233, but stays below 0.24° on 235.
This uncalibrated surrogate is not adopted. Two ±10 mm initial grasp-depth probes also fail 235;
that trial is parked. The original bank finger model remains unchanged.

The working engine reports Genesis 0.2.1 at `f41427d2c3b334a6b0bdb92471867dc7b079bd2d`.
The earlier three-trial native-default 1.2.1 check failed every pickup, so there is no drop-in
upgrade here. This does not reject every separately calibrated modern-engine model.

The early-day rotation corrections remain separate: December 16 (−9.7°) and 17 (−19.2°) require mount yaw and
rotated can seeds. Their archived recovery omitted the post-build hook, which changes arm gains
and goal spawn height; gravity compensation was already present in the pre-hook. This census
does not validate or replace the 88 provisional early winners.

## Execution and limitations

The first collector was terminated with exit code 143 after 67 completed trials. Its remaining
workers had produced no complete trajectories. The seven unfinished UIDs were resumed with the
same source poses, replay code, controller, scorer and engine; completed simulations were not rerun.
Original logs are preserved, and [resume.json](full_pool/resume.json) records the interruption.
The external cause of termination was not established. Interrupted jobs are not task failures.

Eight adversarial sequence-filter tests pass. Packaged arrays are finite, load without pickle,
have `(N+1,17)` observations paired with `(N,7)` actions, and match their recorded hashes. Final
goal heights were separately checked for the bank. These checks establish artifact consistency,
not real-world physical calibration or a corpus-wide recovery solution. The broader goal remains open.
