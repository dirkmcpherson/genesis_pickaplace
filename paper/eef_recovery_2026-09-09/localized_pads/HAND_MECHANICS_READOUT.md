# Localized pads and hand mechanics: current readout

The surface-pad model has a useful **within-hand** effect on late-day trial 233:
it restores release and a supported final slide. It is **not yet an end-to-end
improvement over the original recovery model across demonstrations**. Both early
calibration trials fail pickup with this hand, and the original fixed-coupling
233 already completed goal contact. No training bank has changed.

## Executed comparisons

There are 48 completed full replays before reserved validation: 36 parameter
comparisons on 233 and 12 on early calibration trials 113/184. These are three
independent demonstrations, not 48 independent samples. Every execution is
recorded under `logs/`; `summary.json` checks identical source EEF actions, joint
commands, gripper feedback and mount against the prior reference. Early trials
retain the corrected day-specific yaws and full variant hook. There is no og4,
added push, terminal settling or changed initial placement.

| Family | Runs | Finding |
|---|---:|---|
| Reconstructed/divided geometry | 5 | Division changes contacts and causes tipped failures; rejected. |
| Original geometry, surface-only pads | 4 | Exact original-contact reproduction; softness improves loaded seating but loses the sequence. |
| Stronger return springs, original actuator | 6 | None restores the full sequence. |
| Match the original drive along its unloaded coupled direction | 9 | No full-sequence recovery. Matching this one direction does not calibrate internal articulation. |
| Wider distal range, original or matched drive | 6 | More curl, but all full-sequence failures. |
| Early narrow/wide travel and rigid/soft factorial | 8 | 113 never achieves supported release; 184 never picks. |
| Lower-force actuator, different moment ratio and weak return | 3 | Better loaded seating; slow return can catch the can during withdrawal. |
| Same hand, reduced passive damping | 3 | Rigid contact fails 233; soft settings .03/.04 pass the supplied metric and perform a supported push, but stop short of goal contact. |
| Frozen reduced-damping hand on early calibration | 4 | Both pad settings fail pickup in both 113 and 184. |

## Candidate, controls and qualifications

`fast_return_presets/tc0.03.json` declares actuator stiffness 320, generalized
force limit 2, distal moment ratio .5, return stiffness .2, proximal damping .05,
distal damping .002, and a mirrored distal lower limit of -1.03 rad. The pad
has a 3 mm maximum normal depth and .03 s contact time constant; its paired
original-contact control is .02 s. Surface classification preserves the original
collision geometry. The finite layer transitions to original backing stiffness.
These are uncalibrated mechanics and solver parameters, not measured material
moduli, actuator force or pad thickness. The .03 setting was frozen as the least
compliant tested passing setting in this hand family; .04's sub-mm endpoint
advantage was not used to select it.

The gripper input comes from recorded **motor position feedback**, not the
command topic. Finger joint values in the inspected real record follow the
fixed mimic equations and are not independent measurements of passive curl.
The previous surrogate can miss its own assumed motor coordinate substantially
under load. Changing its transmission and force limit improves that internal
consistency, but does not validate the assumed motor-to-joint mapping.

The faster-return damping follows a specific diagnostic: along the symmetric
internal mode at fixed actuator coordinate, the approximate overdamped D/K
relaxation time falls from 1.65 s to .165 s. This is a linearized explanation,
not a prediction of exact settling under contact (`return_mode_diagnostic.json`).

Official Kinova ROS and raw CAD joint ranges differ. The raw right tip permits
-1.03 rad; the raw left has different signed limits even though the local axis
already has the same sign as ours. Mirroring the right range to both fingers is
a symmetry hypothesis, **not** a verified sign conversion or measured hardware
correction (`hardware_reference/TRAVEL_REVIEW.md`).

## Trial 233: what improved, what did not

| Setting, same reduced-damping hand | Final center distance | Supplied metric | Strict goal-contact sequence |
|---|---:|---|---|
| Original contact .02 s | 179.33 mm | fail | fail |
| Soft pad .03 s | 70.19 mm | pass | fail: 4.19 mm surface gap |
| Soft pad .04 s | 69.63 mm | pass | fail: 3.63 mm surface gap |

The .03 candidate releases on the shelf at 22.29 s. A supported push begins at
24.72 s and produces 24.97 mm of goalward movement by the actual last frame;
98.56% of that interval has sampled upright shelf support and the can stays
within the existing 8 mm height bound. There are no sampled goal contacts after
release. The unchanged supplied predicate also passes when applied only to the
suffix after confirmed supported release: gain 24.53 mm. This additional audit
rules out a pass caused solely by counting the earlier carry. Neither scorer's
thresholds were changed (`candidate_sequence_audit.json`).

The original fixed-coupling reference completed strict contact with a final
surface gap of -0.185 mm. The candidate improves loaded grasp appearance while
losing that exact contact result. Under the existing camera hypothesis, loaded
cap-relative rim errors at 14/16 s are 1.13/5.59 px (original fixed:
29.29/25.86 px). At 22/25 s they are 38.23/42.47 px (original fixed:
15.59/26.99 px). Thus **placement image fidelity remains worse**. These are
conditional image errors, not calibrated 3D position errors or measured curl
angles. Camera sensitivity is retained in the image-comparison JSON.

`233_soft_pad_real_sim.mp4` is clock-matched to recorded camera 4; all 321 frames
were decoded and the contact sheet visually inspected. The largest nearest-frame
time error is 16.74 ms. It shows the full task plus a hand crop and contact/gap
annotations. Viewpoints differ; it is not a registered overlay. Rendering uses
saved poses and does not add physics or settling.

## Early-day results and next gate

With the frozen hand, 113 ends 312.51/293.91 mm from the goal for rigid/soft pads;
184 ends 206.38/218.52 mm. All four fail pickup. In the soft traces, the maximum
can center height is only .10491 m. This is not a final-slide bottleneck. Original
184 did pick and release; this candidate regresses that behavior. Early 113's
initial grasp occurs around 55–57% recorded motor closure, versus roughly 88%
during 233's loaded carry; the different loading and closure regimes need to be
addressed by a physically supported hand model, not per-trial success tuning.

`validation_plan.json` freezes the same candidate and rigid controls for reserved
176/185/237 and requests independent saved-action replay of 233. These trial
outcomes were seen historically but excluded from the new fitting. Validation
results must be reported even if they reject this candidate. No population
recovery-rate improvement can be inferred from this selected six-demo set.

## Frozen validation completed

All six reserved comparisons and both independent saved-action replays are now
terminal and successful executions. Both saved-action replays reproduce every
array exactly. The soft candidate does not improve the original model across
the selected six demos: supplied-metric passes remain 3/6 and strict completions
fall from 3/6 to 1/6. See `VALIDATION_READOUT.md` for paired results and limitations.
The goal remains unresolved; no experimental candidate was adopted.
