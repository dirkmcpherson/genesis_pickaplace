# Adaptive closure candidate

User requested an actual URDF correction rather than further observation alone.
This is an isolated implementation candidate, not a measured Kinova transmission.
The original URDF, world of record, demo banks and trainers are unchanged.

The candidate URDF removes the two distal mimic constraints and retains the
opposed proximal coupling. Arm geometry, finger meshes, masses and joint limits
are unchanged. Package mesh paths are resolved to absolute paths for loading.
The experimental driver replaces all four finger position servos with one shared
actuator coordinate and two return springs. It has no object-distance, contact,
success, or phase switch. Contact blocking a proximal joint can redistribute the
actuator load into inward distal rotation; spring reactions act on both joints.

For q = [left proximal, right proximal, left distal, right distal], define
theta = (q_right - q_left)/2 and s = theta + a*(q_left_distal + q_right_distal)/2.
The recorded motor still defines the same unloaded target theta and distal angle.
Actuator energy is a quadratic in s-s_target, becoming linear at saturation.
Each return spring penalizes distal + 0.676*its_opposed_proximal_angle - 0.149.
Generalized torques are the negative gradient of the combined energy. Negative
distal rotation curls inward in this URDF; the sign is checked from joint geometry.

Initial assumptions, fixed before the first valid replay: actuator stiffness 80
and cap 100 in generalized angular units (the sum of the existing two proximal
40/50 settings), distal moment-arm ratio 0.2, return stiffness 2, proximal damping
1, distal damping 0.02. These are surrogate parameters, not manufacturer values.
The sum/cap convention preserves a starting scale, not equivalent full dynamics.
In particular, damping and distal load distribution change. No performance-based
parameter search or physical calibration has yet been performed.

`check_adaptive_gripper_candidate.py` verifies torque/energy consistency including
saturation, unloaded equilibrium across the motor range, and the direction of
blocked-proximal curl. `run_adaptive_gripper_candidate.py` uses the unchanged EEF
source path, full variant hooks and no added motion. It applies forces at each
original 1.25 ms substep, with an assertion on callback counts at every scene step.
URDF hashes, remaining equalities, actual zero finger PD gains, damping and torque
statistics are recorded. No physical timestep or contact parameter is changed.

The first integration run in `233/` is INVALID: the rigid-only fast path skipped
the original callback, leaving substep_calls=0. It is retained with INVALID_RUN.json
and must not be interpreted as a model failure. The corrected dispatch wraps
rigid_solver.substep itself. `233_active/` is the first actual transmission trial,
using a known later-day completion. Inspect this mechanism before testing the
early-day 113 retention failure; a broad recovery census is premature until the
implementation operates correctly and produces useful hand behavior.

## First valid replay: trial 233

`233_active/` completed with 23,096 actuator updates, exactly the reset plus all
962 recorded 30 ms actions at eight substeps per 10 ms scene step. Original EEF
actions, refined arm targets and recorded grip are byte-identical to the baseline.
The maximum joint speed was 1.63 rad/s; no instability guard fired.

The fingers curl inward independently by up to 18.36 degrees, retain the can,
and achieve upright shelf release. [Loaded hand comparison](loaded_hand_comparison.png)
shows projected CAD geometry and the actual can/tool transforms from both traces.
This is simulated geometry, not a measured real finger angle. Both distal joints
remain near their inherited -0.50 rad lower limits for 502 frames; those bounds
were derived from the old mimic range and are not measured adaptive travel.

Using the same real-rim annotations and unchanged cap-based camera hypotheses,
the nominal cap-relative rim error at the two loaded checkpoints falls from
29.3/25.9 pixels to 14.4/13.6 pixels. The improvement persists across the declared
lens assumptions. At the two later checkpoints it worsens from 15.6/27.0 pixels
to 34.5/43.3 pixels. See `233_active/can_seating_comparison.{json,png}`. No camera
or finger parameter was fitted to this candidate's outcome.

The stricter complete sequence regresses: release occurs, but no qualifying
supported slide to contact follows; final can surface gap is 1.75 mm. Thus this
is a working inward-curl mechanism with a closer loaded-can image projection,
not an adopted recovery improvement. Model parameters, inherited joint stops,
opening dynamics and the placement discrepancy need further evaluation. The
early-day 113 run uses the same fixed parameters, not a separately tuned hand.

## Early-day trial 113

`113_active/` also completed with the same fixed transmission parameters and
100,496 checked actuator updates. EEF commands, refined arm targets, grip and
mount transform are identical to the corrected-yaw baseline. Independent inward
curl reaches about 12.1 degrees, but the can is lost during carry and settles
upright on the starting table. At 13.02 s it has no hand contact and its center
is at 0.10049 m. No upright shelf support or complete sequence is recovered.
The absence of a >60 degree tipping event here must not be counted as retention:
the can is already separated from the hand. The 198 mm relative displacement
over the old 8.01–13.02 s diagnostic interval is a dropped can, not held-can creep.

The first valid two-trial experiment therefore demonstrates implemented inward
curl but no new full recovery. One successful control regresses. The candidate
is retained for mechanical investigation and is not adopted into training data.
