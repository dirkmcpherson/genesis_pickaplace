# Physical reference set

Calibration references: 113 (December 16 carry failure), 184 (December 17 placement
failure), 233 (December 18 complete sequence). Validation references: 176, 185, 237,
one per day. All have two high-resolution videos with monotonic per-frame epoch
timestamps matching decoded video counts. Existing outcomes were known before
selection; validation excludes future fitting, not previous inspection. No model
parameters have been fitted or new physics runs launched.

Calibration camera sheets:
- [113 paired views](113/camera_visibility_sheet.jpg)
- [184 paired views](184/camera_visibility_sheet.jpg)
- [233 paired views](233/camera_visibility_sheet.jpg)

Camera 4 exposes gross can retention and landing. Camera 0 looks upward through
the shelf and is blind to the initial grasp; near placement it can expose the
loaded finger silhouette. [184 before motor opening, full size](184/before_motor_opening_cam0.jpg)
is a useful loaded-hand reference. Tape/covering hides linkage detail, so neither
a passive joint angle nor a stiffness is measured by this image. Can, rack and
hand occlusions vary by event. Camera extrinsics remain a separate calibration
requirement for metric 3D measurements.

Event times are selected from recorded motor change and simulated can height,
then mapped to camera timestamps; they are not claims of observed real release
or maximum height. Raw first-window time is restored before matching normalized
source times. Post-step clock and camera acquisition/robot follower delays remain
explicitly distinct. Each view_matches.json retains exact frame mappings.

Next measurement work is to annotate visible can/rack/hand features and first
real/sim divergence within these calibration references, reporting image-space
uncertainty before attempting 3D reconstruction. Hidden pivots must remain unknown.
Keep validation references excluded from parameter estimation.

## Trial 184 loaded-release measurement

[Annotated image-space measurement](184/release_pair_measurement.png) compares
30.901 s and 33.004 s. The grey distal pad's manually selected projected axis
changes by about 23 degrees while tool translation is 1.97 mm and URDF-FK tool
rotation is 0.25 degrees. Motor feedback changes from 65.79 to 29.25. The saved
JSON includes exact points, source image hashes and a +/-8 pixel point-selection
sensitivity analysis (approximately 16–31 degrees change; not a confidence interval).

This establishes a visible pad-shape/orientation change during near-stationary-tool
opening. It does not establish a 23-degree joint rotation, independent passive
curl, or a mismatch with the rigid mimic model: projection and motor motion still
need accounting. Both load and motor position change. No mechanical parameter was
fit to this pair. Rigid-base landmark inspection found only a small visible pose
range around this event, so independent camera calibration is not solved by it.

## Rigid gripper-cap camera pilot (trial 233, camera 4)

Two visible circular cap centers correspond plausibly to the proximal pivot axes.
The CAD bounds near those axes support surface x=+/-15.2 mm, y=+/-30.5 mm,
z=70.003 mm in the gripper-base frame. Six manually annotated centers across three
poses condition a camera fit without any can-position or task-success anchors.
Old camera intrinsics are retained as a conditional prior; this is not yet a fully
independent calibration. Source joint measurements supply the moving robot poses.

The viable correspondence fits at 1.40 pixel RMS. An unused 14 s pose gives 5.28
and 4.18 pixel errors, larger than the nominal 3 pixel manual annotation scale.
Thus the low fitting residual must not be used as the claimed measurement accuracy.
The opposite cap surface produces a similar numerical fit but faces away from the
camera under its inferred pose; the visible-face hypothesis is x=+15.2 mm. This
visibility check still depends on correct CAD feature correspondence.

`fit_rigid_caps_pilot.py`, `check_rigid_caps_pilot.py` and the 233 JSONs retain all
manual coordinates, camera times, hypotheses and projections. The next checks are
more independent poses, intrinsic sensitivity and CAD correspondence; no shelf
height, object position, passive joint angle or physics parameter has been adopted
from this pilot. The pad-opening pair remains insufficient to isolate passive curl.

## Expanded camera check

Four unused poses (14, 16, 22 and 25 s) give 4.74 pixel RMS under the original
intrinsics; individual errors range 0.14–8.89 pixels. The 10 s view has one
occluded cap and was excluded instead of inventing a correspondence.

A conditional sensitivity test changes both focal lengths by 0.8–1.2 and brackets
k1 with -0.25, the previous fit, and zero. Extrinsics are refitted only to the
original three poses for each assumption. Among fits with training RMS <=3 pixels,
camera z spans 0.609–0.797 m. This range is not a calibrated uncertainty interval;
it demonstrates that low fitting error alone does not fix metric camera position.
No lens was selected using the unused poses. `233/cap_camera_sensitivity.json`
retains all predictions and manual annotations.

Do not infer a precise shelf-height correction from this pilot. Further work must
check the real-to-CAD cap correspondence and its rigidity through loaded/open
configurations, then obtain additional independent camera constraints or more
varied robot poses. Neither task permits using simulation success to choose the
geometry. The physical reference validation trials remain unused in fitting.

## Projected gripper check: targets versus actual simulated joints

[Four timestamped real/model comparisons](233/gripper_projection_check.png) show
the real image, the motor-derived URDF target geometry, and the actual simulated
finger angles attached to the real FK wrist pose. This isolates hand configuration;
it is not a render of the whole simulated scene. `project_gripper_reference.py`
uses existing traces, performs no finger fitting, and retains image/source hashes,
angles, projected outlines and camera-assumption sensitivity in the companion JSON.

At 14 and 16 s, simulated proximal joints remain approximately 15.3–15.9 degrees
away from their commanded values under load; the distal difference is about
10.3–10.7 degrees and follows the fixed mimic. After opening, at 22 and 25 s,
all four joints are within 0.09 degrees of their targets. Thus commanded geometry
alone substantially misrepresents the simulated loaded grasp. This difference is
servo/contact deflection along the existing coupling, not independent passive curl.

Visual inspection finds the loaded real tips partly concealed by the can and tape.
The projected mesh outlines include hidden surfaces; their apparent overlap with
the can is not evidence of penetration or a measured real/model joint discrepancy.
Open views expose distal tips and are useful for checking correspondence before
interpreting the loaded configuration. Padding and covering also differ from CAD.
No passive angle or spring parameter is inferred from this panel.

Across the previously declared lens assumptions, the maximum displacement of any
projected gripper vertex from the nominal projection is 3.9–7.6 pixels across these
views/configurations. This does not include landmark correspondence, annotation,
timing or model errors and is not a confidence bound. The large camera-height
sensitivity therefore need not prevent a local image-space hand comparison, but
it still prevents precise world-geometry correction. Next use visible open-hand
landmarks and the second camera to separate correspondence from loaded adaptation;
do not tune a spring from concealed finger tips.
