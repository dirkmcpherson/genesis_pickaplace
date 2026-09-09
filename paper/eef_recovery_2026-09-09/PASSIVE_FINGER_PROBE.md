# Distal compliance probe — September 9

Status: exploratory mechanism test, not a calibrated hardware model or a bank admission rule.

The existing distal joints are both position-driven and bound by mimic equalities. Removing
only their PD gains does not remove the fixed coupling. This experiment removes distal PD
actuation and softens only the two distal mimic equalities. The proximal coupling, proximal
actuation, original joint limits, collision shapes, contact parameters, world and recorded
EEF/grip path are unchanged. The soft constraints preserve a reaction on the driver joint;
this is still a numerical surrogate, not a measured spring-return mechanism.

Before reading treatment outcomes, declare two mimic time constants: 0.1 and 0.3 seconds.
Test trial 235 (known pickup/release difficulty) and trial 233 (known complete control).
Use archived starting poses and precision EEF playback, with no slide repair. Compare with
the unchanged-world census controls. Retain all four outcomes, including regressions.

Read back equality parameters and distal PD gains before stepping. Measure deviations from
the original mimic equation, task stage failures and supported release. Independent motion
alone does not establish correct inward curl, better real-world fidelity, or improved recovery.
Neither constant may be adopted as a hardware estimate from these results. No treatment
output enters the verified demonstration bank.

Runner: `can_pos_recovery/probe_passive_fingers.py`. The treatment sidecar records the two
named constraints, all before/after equality parameters and zero distal gains. The output
episode is explicitly labeled as a changed-physics experiment.

## Readout

All four full treatment replays completed. Solver field readback confirmed the intended equality
parameters and zero distal PD gains. An initial three-frame smoke test exposed a broken gain-getter
index path in the pinned engine; the runner reads the actual solver PD fields directly instead.

| Trial | Original max independent tip motion | 0.1 s | 0.3 s | Task outcome |
|---|---:|---:|---:|---|
| 233 | 0.043° | 0.331° | 2.903° | All complete |
| 235 | 0.027° | 0.048° | 0.239° | All pick, none achieve supported release |

Values are maximum absolute deviation of either tip from the original mimic relation over the
whole episode, not measured real angles. In all six runs the maximum occurred during robot–can
contact. Trial 233 shows that this surrogate can permit contact-associated independent motion;
trial 235 shows no rescue and substantially different final can positions. Neither result validates
the direction, magnitude, transmission mechanism or stiffness against the real hand.

No treatment is adopted. The parameters are not spring measurements, and the original bank
physics is unchanged. Full traces and exact two-tip results are in `passive_probe/summary.json`.
Kinova's [user guide, Figure 9](https://static.generation-robots.com/media/Kinova-lite-fiche-technique.pdf#page=21)
documents the flexible finger structure and multiple grasp configurations, but does not provide
the compliance parameters needed to turn this probe into a calibrated mechanism.

## Trial 235: locating the failure

The [real-video overview](passive_probe/235_real_overview.jpg), sampled from
`real_videos_480p/235_real.mp4` at nine times over 30.5 seconds, shows an upright shelf placement,
release, subsequent slide and hand withdrawal. It is not a failed source recording.
The [simulated phase frames](full_pool/235/235_failure.jpg) show a different event: the can
topples during placement. By tape frame 550 it lies at 90° on the shelf with no robot contact.
Thus `no_supported_release` here means no qualifying **upright** release, not a permanently
closed gripper. Video clocks have not been calibrated between these views.

Two further declared initial-position probes moved the can ±10 mm along the horizontal tool
approach direction at baseline first pickup (axis approximately [0.969, 0.248]). Both retained
the original finger model and recorded EEF/grip motion. Both failed the supported-release rule;
neither enters the bank. Positions, control reference and full outcomes are in
`position_depth_235/`. This narrowly rejects those two depth adjustments, not every possible
placement correction. Trial 235 is parked after these failures.

## Follow-up: limits and unconstrained tips

[Free-tip diagnostic](free_tip_probe/README.md) rules out active distal limits during
the preceding carry intervals. Removing the coupling entirely permits large independent
motion, including without can contact, but loses 233 pickup and does not rescue 235.
The real bags' distal joint channels are exactly mimic-derived and cannot calibrate
that missing passive response. No new physical model is adopted.
