# Distal travel differs between two official Kinova descriptions

The files are archived at Kinova `ros_kortex` commit
`c3280d8a6a6b5d96590d51538e3f265710a6c77d`; `sources.json` records URLs,
content hashes, axes, limits and mimic definitions.

- The ROS `gen3_lite_2f_macro.xacro` has distal limits [-0.50, 0.21] radians,
  with right/left distal axes respectively positive/negative local Z and tied to the proximal
  joint by the -0.676 multiplier and 0.149 offset.
- `GEN3-LITE-GRIPPER_URDF_SIMPLIFIED_V2.urdf` has no mimic tags. Its right
  fingertip range is [-1.03, 0.21] about positive Z. Its left range is
  [-0.21, 1.03] about negative Z. The local ROS model also uses negative Z for
  the left joint, so the raw-left limit signs do NOT directly match its
  coordinate convention. The raw-right range provides an alternative additional
  0.53 radians of inward travel, approximately 30.4 degrees.

Sources: [ROS macro](https://github.com/Kinovarobotics/ros_kortex/blob/c3280d8a6a6b5d96590d51538e3f265710a6c77d/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_macro.xacro),
[separate gripper CAD export](https://github.com/Kinovarobotics/ros_kortex/blob/c3280d8a6a6b5d96590d51538e3f265710a6c77d/kortex_description/grippers/gen3_lite_2f/urdf/GEN3-LITE-GRIPPER_URDF_SIMPLIFIED_V2.urdf).

The local replay URDF already contains the wider right-joint limit as a commented
line, but the adaptive candidate retained the active mimic-range limits. Its
original trial-233 trace spends 502 frames near the -0.50 rad stops. This makes
travel restriction a plausible confound in the adaptive contact experiments.

The CAD export is an authoritative alternative model, not a measurement of the
specific study robot's passive travel. Its limits do not establish spring
stiffness, tendon routing, pad material or the force law. A bounded comparison
may mirror the CAD right-tip range to both fingers as an explicit symmetry
hypothesis, without changing meshes, inertia,
commands or initial conditions. Any result must still pass grasp, release and
supported-slide checks and real-video review. It is not appropriate to call the
wider range a verified hardware correction solely because it improves replay.
