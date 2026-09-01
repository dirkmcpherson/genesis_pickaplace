# Adversarial critique — DEMO_RECOVERY_PLAN_2026-08-31 (2026-08-31)

Verdict: the audit is worth doing — but the plan as written would run the wrong pipeline, on a
false machine premise, against a wrong inventory, with an instrument it never validates, and with
no pre-registered failure branch. Every factual claim below was checked against the repo, not the
session notes the plan cites.

## A. Factual errors

1. **The raw bags are ON THIS BOX.** The plan's central logistical premise ("Runs on the machine
   that has the raw bags — this box has none") is false. `inthewild_trials/raw/` = 26 GB, 224
   `user_*` dirs; every labeled uid spot-checked (232, 233, 255, 300, 303, 334) has
   `trial_data.bag` + `cam_dev_video0` + `cam_dev_video4` + `config.yaml`. The 07-08 "all-bags
   rsync (in progress)" completed. There is no bag-machine bring-up task.
2. **No ROS1 needed anywhere in the pipeline.** `read_bag.py`'s own docstring: "Needs the
   pure-python `rosbags` package (not ROS)". And the cameras are not bag topics at all — the bag
   records only /joy, base_feedback, cartesian_velocity, joint_state, /tf, /reward; video is
   sidecar MP4s (1280×720@30) with epoch-ns `video_frame_timestamps.txt`. The plan's step 1
   ("extract the t0 camera frame(s)" from the bag, with ROS1) is wrong on requirement, data
   location, and machine. Everything runs here, today, with ffmpeg + rosbags.
3. **Inventory is wrong: 61 solved successes, not 72; 14 missing successes, not 3.**
   `can_pos_recovery/trial_placements.json` (post-merge, matches the 07-20 recovered list
   exactly): success×{ok,ok_batch} = 61; success with `can_pos: null` = 16 →
   **233, 255, 259, 262, 266, 267, 275, 278, 290, 301, 303, 319, 321, 322, 329, 333**.
   The 07-20 note "solved 61→72" counted fail-labeled trials too (11 fails carry ok/ok_batch
   placements: 2 ok + 9 ok_batch). "72 solved successes" is a misread, and "one more never
   re-solved" was invented to reconcile 75−72. (The census's dHrerec_all=72 tapes is also not
   72 successes — it is 54 success + 18 fail tapes.)
4. **Trial 233 has no recovered position.** The demo the user validated as "pick/place/slide
   perfect" — one of the two that pinned the goal — is FK-seed-only (`status no_shelf_batch`,
   `can_pos null`). This both raises the audit's stakes and changes the prize: not "optionally
   +3 successes" but up to **+14**, with a correspondingly larger re-solve workload.
5. **The Hough prototype does not exist.** No `HoughCircles` (or any hough code) anywhere in the
   tree or in git history (`git log -S`). "Prototyped, works" traces to a 07-08 session note whose
   code was never committed. The effort line "prototype exists for detection" is unfounded.
6. **The scoring target is undefined for the trials that matter most.** "Camera vs
   trial_placements.json" has nothing to compare for the 16 null-can_pos successes; their
   comparison point is the FK position (`fk_all_trials.py`), a different file. (The placements
   file also carries stale per-trial `goal_pos` from the relocation era — harmless since the goal
   is static now, but the plan should state the file is authoritative for can_pos only.)

## B. Technical objections (from actual t0 frames, extracted today)

7. **Parallax breaks the table-plane homography.** cam4 (the "top cam") is an elevated OBLIQUE
   view stored rotated 90°; in the u232/u255 t0 frames the can shows its top face AND its side
   label simultaneously → inclination is tens of degrees off nadir. Hough finds the TOP rim;
   mapping its center through a TABLE-plane homography errs by ≈ h·tanθ with h = 0.101 m (the
   world's own can_height) → **4–8 cm systematic error, larger than the plan's 3 cm threshold**,
   correlated across all trials. Fix: skip the homography entirely — fit a full projection matrix
   (DLT) from FK 3D↔2D pairs (tool_pose supplies 3D points at many heights, which a homography
   discards), then intersect the pixel ray with the z=can-top plane; or click the can BASE and
   use the table plane.
8. **No lens model.** No factory intrinsics; consumer-webcam radial distortion at 720p is
   multi-pixel at frame edges → cm-level on the table. The bottom-up cam0 sees the start region
   only in its extreme corner (max distortion). Undistort or restrict to the central region, and
   report residuals — otherwise 3 cm verdicts are noise.
9. **The instrument itself is never validated — the go/no-go gate is missing.** The plan
   validates placements with a measurement it never validates, i.e. the panel's original sin at
   one remove. Mandatory first deliverable: **held-out FK reprojection error** (calibrate on a
   subset of trials/frames, test on held-out gripper positions, report cm). If held-out error is
   >~2 cm, STOP — the instrument cannot adjudicate its own threshold. Half a day, decides
   everything; the plan spends that half-day on detection instead.
10. **Hough cannot measure the fails-audit cases.** A lying can presents no circle — and lying-
    at-t0 is precisely step 6's question (u255's t0 frame shows a can lying tipped on the slide
    rails). Oblique ellipse ratio ~0.7–0.8 also biases HoughCircles centers, and the scene has
    multiple circular features. Given N=91 and the project's own [[human-perception-beats-fits]]
    track record: **click the can base in 91 t0 frames (~1 hour, definitive, gives the
    upright/lying label for free)**; keep automated detection only if it agrees with the clicks.
    Meanwhile the genuinely unspecified hard part — localizing gripper pixels for the calibration
    pairs (manual? a new CV detector?) — is unbudgeted and could dominate the schedule.

## C. Methodological objections

11. **No prereg, no failure branch — in a PREREG-culture project.** "Either the camera confirms
    or it doesn't" is not a decision rule. The disconfirm branch DOES touch existing claims: the
    56 matched-set human demos were recorded in-sim at these placements; "matched-N so nothing
    changes" holds only on the confirm branch. Before looking: register expected confirm rate,
    the threshold (derived from the MEASURED instrument error, not can geometry), and committed
    consequences per disconfirmation level (disclosure only? sensitivity rerun of which arm?).
    An audit run mid-writing whose bad outcome has no committed consequence is exactly the shape
    the 07-08 panel dinged.
12. **Step 5 re-creates the fit-not-recovery criticism.** cpu_research seeded at the camera
    position is still a search; without an acceptance rule (winner must lie within the
    instrument-error radius of the camera position, else "unconfirmed") the "corrected and
    re-solved" bucket is the same machinery wearing a camera hat. The audit metric is
    final-placement-vs-camera distance, never "replay succeeded".
13. **Selection bias in "X/91 confirmed".** Occluded-at-t0 trials are unmeasurable; if occlusion
    correlates with drift, the confirmed fraction overstates. Report the unmeasurable set
    explicitly and show it looks random.
14. **Step 7's side-by-sides cannot adjudicate placements.** Bag-vs-sim divergence is dominated
    by open-loop replay fidelity (fig7's low-pass finding; 07-08 "control-limited, not
    placement-limited"). Right artifact for the user's eyeball; wrong evidence for the paper's
    placement claim. Only the t0 distance measures placement — don't let the paper cite the
    videos for it.
15. **Scope the paper claim to STARTING positions, and decide the goal question explicitly.**
    The panel's headline number (60 % goal-relocation dependence) was answered by the
    static-goal decision + user validation, not by this audit. cam0 (bottom-up through the
    shelf) could in principle audit the goal — and u255's t0 shows the goal can LYING on the
    rails, evidence the goal state genuinely varied across trials. Opening that box
    accidentally mid-writing is a real risk; if out of scope, say so and why.

## D. Cheap wins the plan misses

16. **Eyeball the unsolved trials' t0 frames TODAY** (10 minutes, this box, no pipeline):
    u255 already suggests why it never solved — the goal can lies on the rails at t0, a
    different world than every placement search assumed. The 16 unsolved successes + 5 unsolved
    fails may partition in one sitting.
17. **Census of measurability first** (visible / occluded / lying, per trial) before building
    anything — it sizes every downstream bucket.
18. For the record: the unlabeled-200–231 hope has holes — 186–195 & 220–231 have NO cameras,
    208–219 are video4-only. (All 91 labeled trials have both cams.)

## What the plan gets right

Matched-N discipline; refusing to rescore the complete (1b) result; PREREG-amendment framing for
re-admitted fails; the side-by-side eyeball artifact (aligned with how every hard call here has
actually been settled); and "better now than from a reviewer".

## Revised order of operations

0. (10 min) Look at t0 frames of the 21 unsolved trials. 1. (½ day) DLT calibration + held-out
validation → measured error budget → GO/NO-GO + data-derived threshold. 2. (1 h) Click 91 can
bases (+ upright/lying). 3. PREREG the audit incl. the disconfirm branch. 4. Score vs placements
(solved) / FK (unsolved); re-solve with the within-radius acceptance rule. 5. Side-by-sides,
labeled qualitative. — No bag-machine or ROS1 line items; total effort similar to planned but
redistributed toward calibration/validation and away from detection.
