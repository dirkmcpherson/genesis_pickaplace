# Matched EEF replay pilot, September 9

Development examples only; not a new demonstration bank or a held-out improvement claim. Registration: [EEF_GRASP_RECOVERY_2026-09-09.md](../EEF_GRASP_RECOVERY_2026-09-09.md). Frozen learners and datasets are unchanged.

## First paired readout

Each run uses a fresh CPU process, one effective Taichi CPU thread (checked), raw joint-derived tool waypoints, raw gripper feedback, three 0.01 s physics steps per tape frame, and both variant hooks. This fixed tape clock is still CONFOUNDS 46. It is not timestamp-correct real replay.

| Trial | IK target position p99, mm | IK target rotation p99, degrees | Actual tool error p95 joint / EEF, mm | Legacy endpoint joint / EEF |
|---|---:|---:|---:|---|
| 233 | 0.0375 | 0.0139 | 6.553 / 6.540 | nested / nested |
| 242 | 0.0299 | 0.0741 | 6.856 / 6.822 | nested / nested |
| 118 | 0.3899 | 0.1425 | 25.763 / 25.812 | not picked / not picked |

233 and 242 pass the registered p99 IK gates. Their pairwise tool-position differences p95 are 0.172 and 0.142 mm; can-position differences p95 are 2.254 and 5.793 mm. Similar arm tracking does not make contact trajectories identical.

118 fails both IK gates. Inspection identifies an action-target mismatch: raw joint 4 reaches -2.6115104 rad (frame 253), beyond the URDF lower limit -2.6. Joint replay issues the raw target; default Genesis IK clamps its answer at the limit. At that frame, independently computed target error is 2.769 mm / 0.659 degrees. This is not a finger effect or a coordinate-frame error. A separately saved `unclamped_targets` diagnostic disables IK target clipping while preserving the simulation's physical joint limits; this is for equivalence to raw tape targets, not an authorization for unconstrained hardware control. **Result:** all 1,760 frames replayed, p99 target error 0.0276 mm / 0.0127 degrees, passing both gates. Pickup still fails. Actual tool p95 error remains 25.812 mm; solving IK accurately does not remove physical tracking error at this approach.

## Physical task sequence versus stage labels

This corroborates CONFOUNDS 47; it does not establish a new scoring rule or change any existing result.

- **233, both modes:** upright shelf contact begins at frame 679. The can is supported and has no robot contact for a long interval (joint frames 715–797, EEF 716–797), then robot contact resumes and moves it toward the goal. Supported can–goal contact occurs at frames 822–881 in joint replay and 822–850 in EEF replay. This trajectory supports a set-down, release, then push-to-contact sequence. Visual review of the EEF saved-pose sequence confirms release, a later fingertip push, and withdrawal; see [video](233_eef_setdown_slide.mp4) and [frame sheet](233_eef_sequence.jpg). Final sustained goal contact is not guaranteed: the cans subsequently separate slightly.
- **242, both modes:** supported, upright release occurs, followed by a later push. Neither has sustained supported can–goal contact for three tape frames. Both finish within the permissive nested proximity threshold. Treat this as near-contact, not verified complete slide-to-contact.
- **Both trials:** `slide_success` remains false. For 233, feedback during supported shelf contact is 38.16–38.94 despite 159 supported frames with no robot contact; for 242 it is 29.92–32.88, with 382/405 supported frames without robot contact. Feedback below 30 is not a necessary condition for physical release in these replays.

Frame numbers above are zero-based tape indices; they are not raw-camera or recovered-video timestamps. Support uses actual can–shelf solver contact and can tilt below 20 degrees after a picked flag. These are diagnostic observations, not a replacement preregistered evaluator.

## Early recovery provenance discrepancy

Source inspection: `recover_early.py` and `render_recovered.py` call `apply_pre` and `build_world`, but do not call `apply_post` / `post_build`. The pre-hook changes mount yaw, riser and shelf geometry; the missing post-hook sets arm gains and the shifted goal spawn height. Gravity compensation is already applied by the pre-hook. `build_world` itself initializes base arm gains and does not invoke the post-hook. Therefore the archived early recovery path is not the complete named w3+yaw configuration.

The matched full-hook trial-118 control does not pick up the can at all: its maximum can z is the initial falling spawn pose, about 0.1049 m. This differs from the archived picked label/video and cannot be blamed on EEF actions, since joint mode also fails. **Separate pre-only diagnostic confirmed the discrepancy:** all 1,760 frames replayed with the same initial can pose and tape; picked=true, contact=false, nested=false, matching the archived picked-stage outcome. Measured base arm kp is [200,200,150,100,60,60], versus [800,800,600,400,240,240] after the full hook. This ablates the hook as a package, not arm gains versus goal spawn individually. Do not extrapolate one example to a corrected count for the 88 early winners. The December 16/17 yaw corrections remain a separate, necessary issue. Logged in CONFOUNDS 56.

## Artifacts and reproduction

Run `can_pos_recovery/eef_recovery_probe.py --uid UID --mode joint|eef_delta --out OUTPUT_DIRECTORY` using `/home/james/workspace/genesis_sim2real/venv/bin/python`. Add `--ik-unclamped-targets` only for the separately identified diagnostic. Completed JSON results are never overwritten.

`pilot/` has six full traces and JSON summaries. Arrays in each NPZ:

- `trajectory`: six actual arm joints, tool xyz, tool quaternion wxyz, can xyz, can quaternion wxyz, can tilt degrees (21 columns).
- `target_tool`: world-frame 4×4 tool targets; `actions_eef`: world translation delta metres, left-composed world rotation delta rotvec radians, normalized clipped grip feedback. `source_grip` preserves the raw feedback actually passed to the environment's gripper mapping.
- `actions_joint`, `source_joint`, `ik_error`, and independent URDF-FK `command_pose_error`.
- `finger_joint`: actual left base, right base, left tip, right tip angles. `goal_pose`: xyz + wxyz.
- `contact_counts`: can–shelf, can–robot, can–goal. `stages`: picked, placed, contact, contact_push, slide_ok, slide_success; these preserve existing semantics.

JSON includes source/code hashes, world name, initial can position, clock/frame counts, thread count, tracking metrics and existing endpoint flags. Pilot runs used default IK clipping (the later explicit `ik_respect_joint_limit` field is absent in those first six JSONs). No passive-gripper modification has been tested.

The video uses `render_eef_trace.py` to display measured arm/finger/can/goal poses from the saved
EEF physics run, without recomputing dynamics. It spans tape frames 650–923 at the original
simulation rate (one rendered frame per three tape frames). Its contact annotations come from
the measured trace. The renderer explicitly invalidates Genesis's visual caches between poses;
an initial stale-transform render was rejected and moved out of this artifact directory. The
corrected frame sheet was inspected at frames 650, 716, 797, 824, 851 and 920. This is visual
validation of the recorded run, not a second independent dynamics trial.
