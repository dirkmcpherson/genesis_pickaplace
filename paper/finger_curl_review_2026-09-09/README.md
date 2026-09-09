# Finger curl: visual inspection, 2026-09-09

**Clarified question and current conclusion:** the user reports passive distal-joint curl when the can seats deeper toward the wrist. Ordinary commanded articulation does not test that behavior. The current simulator effectively enforces a fixed relation between proximal and distal joint angles; the trial-118 trace confirms negligible independent distal motion. Kinova's own user-guide Figure 9 corroborates a distinct curled, enveloping grasp configuration (see manufacturer-source follow-up below). The real footage does not isolate passive adaptation from commanded opening/closing, and the physical compliance parameters remain unmeasured.

Question: does the real Gen3 Lite curl its fingertips during closure while the simulated fingers stay straight?

## Evidence inspected

Trials 233 and 242 (December 18): original 1280×720, 30 fps recordings from both cam_dev_video4 (above the workspace) and cam_dev_video0 (beneath the shelf), plus the existing recorder-path `gc_kp4_riser3_shelf6` census videos, including their wrist-camera panels. Surveyed frames through approach, grasp, carry, and release; inspected enlarged open/holding views. This is a two-trial visual check, not a corpus-wide or current-runtime kinematics test. Early-day yaw variants were not inspected.

Sources, relative to repository root:

- `inthewild_trials/raw/user_{233,242}/cam_dev_video{0,4}/output.mp4`
- `can_pos_recovery/videos_census/{233,242}_full_gc_kp4_riser3_shelf6.mp4`

## Finding

Both real and simulated fingers visibly change their articulated configuration between open approach and holding the can. The simulated wrist view shows the finger bases folding inward and the tip-bearing segments changing orientation; the fingers are not frozen straight throughout closure. The real fingers also fold inward, with curved-looking dark tips beside the can.

These views do **not** establish equality of real and simulated curl, nor a quantitative shortfall in simulated fingertip rotation. Blue material covers much of the real finger structure, the can occludes the contact surfaces, and the real and simulated cameras have different viewpoints. A joint-angle discrepancy must not be inferred from silhouettes alone. The under-shelf camera gives a close view of the real contact region, but much of the linkage remains occluded.

Prior model-linkage tests establish agreement with the URDF coupling, not agreement with real finger poses. A real linkage or pad-shape mismatch remains possible; this inspection neither measures it nor establishes its role in replay failures.

## Saved frames

- `233_open_hold.jpg` and `242_open_hold.jpg`: cropped camera-4 images at video times 4 s and 10 s, rotated 90° clockwise for viewing, beside simulated wrist-camera crops at the same video times. Rows represent approximately corresponding stages, **not timestamp-calibrated or equal-command poses**. Video clocks/follower timing differ. Ordinary cubic resizing was used; no generative enhancement.
- `233_real_under_shelf_12s.jpg`: original-resolution camera-0 frame at video time 12 s, no crop or rotation.

No simulator, demonstration set, or stage label was changed.

## Follow-up: user identifies trial 118 and the distal joint specifically

The initial comparison did not adequately distinguish movement at the palm from bending at the second, distal finger joint. It must not be treated as verification of the real distal-joint curl.

Inspected `can_pos_recovery/videos_recovered/118_1216_picked.mp4` through its approximately 58.7 s duration, with enlarged sim-hand frames. `118_distal_open_closed.jpg` shows video times 39 s and 45 s: the hand changes from spread fingers to tips meeting. This alone does not quantify how much of the change comes from each joint, especially with the changing wrist pose.

The rendering path is different from the earlier census videos: `render_recovered.py` directly replays the raw joint/gripper tape through `replay_harness.gripper_targets`. It commands both named tip joints using `q_tip = -0.676*q_driver + 0.149`. This counterrotation can preserve a relatively straight-looking distal pad orientation while the proximal finger moves. It is not proof of real linkage fidelity. The recovered video's real pane is resampled across its full duration using `linspace`, not aligned by physical timestamps.

A fresh, unrendered replay of all 1,760 tape frames used the current renderer's build/reset/control sequence, winner position, `gc_kp4_riser3_shelf6_yaw16`, and world settings. Measured DOF positions are in `118_joint_trace.json` (angles in radians, both tip joints queried by name through `JOINT_NAMES`). Both tip joints range from approximately -28.65 degrees to +12.02 degrees: about 40.67 degrees of rotation relative to their proximal links. They are not stuck. For example, tape frame 600 has right-base 30.26 degrees / right-tip -11.92 degrees; frame 810 has 14.85 / -1.50; frame 1350 has -5.16 / +12.02. Joint-coordinate zero is the URDF reference, not an independently measured anatomical straight-finger pose.

The opposing rotations substantially cancel in distal-link orientation: right-base + right-tip goes from about 26.35 degrees open to 6.87 degrees closed. Thus a moving second joint does not imply a large inward rotation of the outer pad relative to the gripper body. This checks current replay execution, not telemetry embedded in the archived video, and does not measure the real second-joint trajectory. No claim that real and simulated distal curl match is justified by this check.

## Clarification: passive response when the can seats deeper

The user clarified that the real outer fingers curl passively when the can is sufficiently close to the wrist. Treat this as a firsthand hardware observation, distinct from what this video audit independently establishes.

Computed over the full trial-118 measured trace, the residual `q_tip - (-0.676*q_right_base + 0.149)` has maximum absolute values 0.00043741 radians (left) and 0.00042101 (right), or about 0.025 degrees. Thus the running model allows effectively no independent distal rotation away from the fixed coupling in this replay. Both direct tip position control and the URDF mimic constraints enforce that coupling. A contact-adaptive mechanism of appreciable size is not represented by this behavior.

Inspected original trial-118 cam0 and cam4 footage over their full durations using frame sheets and original-resolution close-ups. `118_real_contact_release.jpg` shows raw cam0 times 24 s (pad against the can) and 28 s (pad extended clear of the can). These frames show the contact-region shape change, but grip command and wrist pose are not controlled between them; they do not independently prove passivity or quantify extra curl at matched proximal-joint angles. Raw camera times are not recovered-video times.

The August lab explicitly left rigid versus spring-returned/underactuated real linkage unresolved (§2.1, around lines 256–261). Its tiny simulated mimic residual cannot establish that real compliance occurs only in the pads. Likewise, its 49 mm throat/deep-grasp restriction describes the rigid simulated geometry, not a validated limit of the real adaptive hand.

Potential consequence: a missing adaptive grasp could affect deep seating, retention, and release. Whether it explains trial 118's simulated drop or any corpus-level failure remains untested. A discriminating validation would compare real distal angles at matched proximal angles with shallow versus deep object contact, then test a separately named compliant model against those measurements. No physics change or training-set rebuild has been made.

## Manufacturer-source follow-up

Accessed 2026-09-09. Primary manufacturer material, including a manufacturer-authored manual hosted by a distributor:

1. [Kinova Gen3 lite user guide, page 21, Figures 8–9](https://static.generation-robots.com/media/Kinova-lite-fiche-technique.pdf#page=21). The text describes one linear actuator in the wrist, flexible structural plastic and rubber-like grip material. **Direct visual finding:** Figure 9 contrasts a small-object grasp with nearly straight outer segments against a 62 mm grasp with outer segments bent inward around a deeper central opening; 80 and 100 mm grasp configurations are also illustrated. The 62 mm example is close in diameter to the project's 66 mm can. `kinova_manual_figure9.png` is a cropped rasterization of this figure, attributed to Kinova; it is not a newly generated diagram. This documents the grasp configuration, but does not specify a spring constant, transmission model, or contact-trigger threshold.
2. [Kinova's official Gen3 lite 2F URDF/Xacro](https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_macro.xacro). Both tip joints use multiplier -0.676 and offset 0.149, with limits [-0.50, 0.21] radians. This confirms the provenance of our simplified coupling, not its sufficiency for adaptive contact. Our measured trace verifies that this coupling is effectively enforced.
3. [Kinova API end-effector identifiers](https://docs.kinovarobotics.com/ref/autogen/Enums/ProductConfiguration.html). The L31/Gen3 lite two-finger gripper has its own identity, distinct from the Robotiq 2F-85 and 2F-140. The repository filename containing `robotiq_85` must not guide hardware-mechanism assumptions.

**Revised inference:** the curled grasp is supported by manufacturer illustrations as well as the user's hardware observation. A fixed-coupling simulation is not sufficient evidence of fidelity for that configuration. The specific passive transmission/compliance mechanism is still unresolved by the primary sources inspected, and any causal explanation of replay failures remains to be tested. The earlier conclusion that contact physics alone was the remaining gripper problem was too strong.

## Recorded finger-channel audit

A later check of every joint_states message in 118, 233 and 235 found four finger
names, but both distal values follow the fixed mimic equation with exactly zero
residual throughout all three recordings. These channels cannot independently
measure the passive curl. The bags carry one gripper motor; the upstream driver
maps its feedback into a joint angle. See the [telemetry and free-tip diagnostic](../eef_recovery_2026-09-09/free_tip_probe/README.md).
