# Recovery approach reassessment — September 9

The objective is to reproduce the recorded pick, shelf placement and final slide
with EEF differential actions and credible physical behavior. Replaying the same
simulated trajectory exactly, or increasing a proximity score, is insufficient.

## What the evidence establishes

- EEF conversion is operational: seven actions, world translation/rotation deltas
  plus recorded grip, with precision IK. Independent saved-action replays and the
  direct adapter checks establish numerical reproducibility. They do not validate
  the physical hand or object motion.
- The baseline census covers 74 December 18 and 89 early-day candidates. The early
  pool uses full hooks and -9.7/-19.2 degree yaw; older pre-only winners are not an
  equivalent world. More unguided sampling is not the principal missing work.
- The timestamp collection packages 28 source UIDs under the supplied metric,
  including the separate initial-clearance correction. This is not 28 newly
  rescued, physically verified pick–place–slide demonstrations. The unmodified-IC
  timestamp censuses have ten December 18 and two valid-IC early strict physical
  completions; even those are simulator evidence, not quantitative real matching.
- All supplied-metric passes in the release-order audit can mark release before
  pickup. Keep the user’s classifier unchanged, but do not use it as the sole
  evidence that a physical slide was recovered.
- Trial 113 supplies direct paired evidence: real retention versus simulated
  loss under steady recorded grip. A reproduced static-target hold retains about
  22 mm of relative slip. The .005 contact experiment delays its first loss, but
  exposes a high, edge-adjacent landing. Across ten trials it gains no completion
  and loses both successful controls. No adoption follows.
- Independent passive distal curl remains unmodeled and uncalibrated. Synthetic
  joint feedback and official URDF mimic compliance are not measurements of the
  loaded real mechanism. A loosened mimic is not a reconstructed Gen3 Lite hand.

## Where the previous approach went wrong

1. It emphasized reproducibility, census counts and many local ablations before
   measuring where simulated object/hand behavior first diverges from reality.
2. It let score changes and packaging work dominate progress reports, although
   those changes often did not recover an additional physical sequence.
3. It used already-successful simulator trials too heavily as acceptance controls.
   Losing them disqualifies an intervention as a drop-in recovery improvement;
   it does not alone prove the intervention is less physically accurate.
4. It treated finger compliance, initial object pose, shelf geometry and contact
   regularization as separable tweaks despite their coupled effects on the grasp
   pose inherited by the final slide.
5. It repeatedly ran small follow-ups on mechanisms already explored in earlier
   audits. Negative evidence should terminate a branch, not prompt another nearby
   setting without a new physical prediction.

## Revised working order

Pause the proposed trial-236 camera-position rollout and additional knob sweeps.
No such rollout has been prepared or launched. Preserve completed artifacts and
frozen datasets. The next work is a coherent calibration and validation problem:

1. Establish a small reference set from the existing successful and failed paired
   recordings. Include both early yaws and December 18, with visible grasp,
   placement and slide. Match camera timestamps and record occlusion/measurement
   uncertainty. Select calibration and held-out validation trials before fitting.
2. Measure observable behavior, not just labels: can pose relative to the hand
   during carry; loaded versus unloaded finger shape where visible; can height
   and position at first shelf support; hand/can separation and recontact; final
   can-to-goal geometry. Locate the earliest divergence in each sequence. Distinguish
   directly measured values from estimates based on a fitted camera or initial pose.
3. Reconstruct a shared physical explanation. Check the actuator-to-finger model
   against loaded real behavior; reconcile the open acrylic rack and initial poses
   with imagery. Shared mechanical parameters must not be fitted per trial to
   force success. Use uncertainty or mark unidentifiable parameters rather than
   inventing stiffness or changing commands. The pending rack-dimension question
   is useful evidence, not authorization to choose a convenient height.
4. Assess candidate physics/engine implementations on these local real observables
   and the held-out references. Do not select an engine just because an uncalibrated
   drop-in version loses old simulated successes, or select a model because its
   final reward rises. Existing audits constrain hypotheses but are not substitutes
   for matched physical validation.
5. Only then run the broad 163-candidate recovery census in the chosen consistent
   setup, preserve the fixed supplied metric, report physical-sequence evidence
   alongside it, verify EEF replay, and supply representative paired reviews.

Completion requires improved reproduction of real object/hand trajectories and
more complete recorded sequences, not another bank-manifest count. Some recordings
will remain unrecoverable or unidentifiable. No claim of full recovery is made.

## Evidence pointers

- baselines/eef_replay_env.py and eef_adapter_{237,202}_validation.json
- baselines/demos_eef_recovery_2026-09-09/timestamp_collection_index.json
- early_yaw_pool/FINAL_READOUT.md and final_failure_funnel.json
- slide_metric_of_record/release_order_audit.json
- static_grasp_113/result.json
- early_contact_ts5/summary.json and SHELF_EVIDENCE.md
- user_review/README.md and timestamped comparison JSONs
- paper/CONFOUNDS.md, especially 55–60; gripper_lab_2026-08-25.md
