# Existing shelf evidence, revisited

The August 31 camera model stores fitted goal-can top z = 0.220537 m in its
robot-base coordinate convention. Subtracting the documented 0.101 m can height
gives a conditional deck height of 0.119537 m above that origin. With that
calibration's +0.05 m world offset, the deck is 0.169537 m, close to the current
shelf6 deck at world z = 0.170 m. It does not support raising the shelf by 4 cm
solely to remove trial 113's release drop.

This is not an independent physical measurement: fit_camera.py anchors camera
extrinsics to recovered can positions and fixes goal xy = (0.672, -0.221), itself
outcome-derived. Tape-only calibration was degenerate, and the gate reports
1.5–3.5 cm central-plane position accuracy, not submillimeter height accuracy.
The model was fitted to December 18 videos, so it cannot be applied to earlier
rigs without checking camera and coordinate changes. The current 3 cm simulated
arm riser is also not the camera fit's base-to-world convention.

Sources read: can_pos_recovery/camera_audit/{README.md,fit_camera.py,
cam4_model_final.npz}, paper/CAMERA_GATE_2026-08-31.md, and CAN_STARTING_POSITION.md.
The request for a measured real rack height remains unanswered. Keep geometry
unchanged until independent evidence justifies a correction.
