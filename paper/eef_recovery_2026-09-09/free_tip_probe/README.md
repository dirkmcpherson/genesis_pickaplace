# Distal freedom diagnostic and real telemetry check

The preceding 0.3 s compliance runs leave distal joints at least 0.215 rad (233)
and 0.260 rad (235) from either joint limit during carrying. Joint limits therefore
do not explain their small independent carry motion. See preceding_limit_audit.json.

A declared diagnostic disables only the two distal equality dispatches and tip PD.
Proximal actuation/coupling, original joint limits, geometry, commands and initial
poses remain. Solver readback verifies the change. This hand has no return spring
and is intentionally incomplete; it is not a physical replacement candidate.

| Trial | Baseline | Free distal tips | Max independent motion during can contact |
|---|---|---|---|
| 233 | Complete | No pickup | 17.2 degrees |
| 235 | Pickup, failed upright release | Pickup, failed upright release | 22.2 degrees |

Independent motion also reaches 24.9 and 33.2 degrees without can contact, respectively.
No-can-contact can include other environmental contacts; this does not isolate gravity.
The diagnostic demonstrates joint freedom but does not demonstrate the user's passive
inward curl or a successful grasp model. No episode is admitted and no simulation video
review or independent saved-action replay was performed for these failed diagnostics.

## Real bags contain finger names, but no independent distal signal

Full joint_states messages in trials 118, 233 and 235 contain four finger names.
Across all 2046, 1155 and 1193 messages respectively, both tips equal
-0.676 * right_bottom + 0.149 with **exactly zero floating-point residual**;
left_bottom is exactly -right_bottom. Each bag has one gripper feedback motor.
The numeric checks and topic structure are retained beside this report.

Kinova's official driver publishes a gripper joint value computed from motor position
and configured limits, rather than reading four finger encoders:
[driver source, publishRobotFeedback](https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/noetic-devel/kortex_driver/src/non-generated/driver/kortex_arm_driver.cpp).
This current upstream source is explanatory, not proof of the exact installed study commit.
Together with the exact recorded coupling, the evidence means these distal channels
cannot validate independent physical curl. Treating them as ground-truth distal angles
would circularly validate the same mimic equation used in simulation.

A compliant transmission or return mechanism still needs evidence beyond these synthetic
joint channels. Existing videos and the user's hardware observation remain relevant;
no measured passive stiffness has been recovered here.
