# Camera audit pipeline (2026-08-31)

Camera-derived can positions from the raw trial videos, for auditing the recovered
placements. Built and gate-tested in the 2026-08-31 session; verdict + numbers in
`paper/CAMERA_GATE_2026-08-31.md`.

## Environment
No ROS needed. `rosbags` is pure python:
`~/workspace/genesis_sim2real/venv/bin/pip install --target <libs> rosbags`
(then DELETE the numpy it drags in from `<libs>` so the venv's numpy is used), run
everything as `PYTHONPATH=<libs> <venv>/bin/python <script>`.
Data: `inthewild_trials/raw/user_<uid>/` — `trial_data.bag` (no image topics; 96×96
downsampled `/camera_obs__*` only) + `cam_dev_video{0,4}/output.mp4` (1280×720@30,
sidecar `video_frame_timestamps.txt` in epoch ns, same clock as the bag; video starts
0.17–0.25 s before the bag in every trial checked). Frames are stored rotated 90°:
upright view = ROTATE_90_COUNTERCLOCKWISE; raw→rotated (xr,yr)=(y, 1279−x).
cam4 = the elevated oblique "top cam" (measurement camera); cam0 = bottom-up through
the shelf. 9 labeled trials have NO video: 245 247 263 270 300 303 308 326 330
(272/291 also lack video but are not labeled trials); of these only 303 is unsolved.

## Scripts (in pipeline order)
- `extract_pairs.py <uid>` — (tool_pose, wrist-tape pixel) pairs: samples frames
  ~0.25 s apart while the gripper is OPEN, vetoes static blue (can labels) via a
  median-frame mask, keeps top-3 blue blobs per frame → `pairs3_<uid>.npz`.
- `fit_camera.py` — joint robust fit: pinhole (fx fy cx cy k1) + extrinsics + TWO
  tool-frame tape offsets (EM over blob/offset assignment, euler conv `zyx`) + can-top
  anchors + goal rays + fitted goal-top z → `cam4_model_final.npz`.
  Frame convention: 3D in ROBOT BASE frame (sim world = base + (0,0,0.05); robot at
  world (0,0,0.05) per replay_harness).
- `detect_tops.py` — start-can + goal-can top centers at t0 (upright cans only) →
  `anchors.json` + `top_<uid>.jpg` overlays.
- `detect_tops_close.py` — same at (first-closure − dt): the position that must match
  the placement. Auto-detection is UNRELIABLE here (gripper occlusion, goal-can
  confusion) — the overlays carry a cyan marker at the PROJECTED placement, which is
  what to eyeball/click against. → `anchors_close.json`, `topc_<uid>.jpg`.
- `close_consistency.py` — camera-free stat: |tool close-xy − placement| + post-close
  drag, from the bag alone.

## Hard-won protocol facts
1. **Audit at close-time, not t0.** Placements encode the GRASP position; cans get
   nudged/re-placed between t0 and closure (286: 71 s of maneuvering) and dragged after
   it (295: 11 cm, 260: 17 cm). A t0 comparison flags correct placements.
2. **Regrasps**: "first closure >30" is the wrong event for multi-grasp demos (261:
   first closure at 6 s + 19 cm drag; the placement matches a later grasp). Score
   against the closure that initiates the persisting lift.
3. The wrist tape, elbow patch, and BOTH cans are all blue — every naive "largest blue
   blob" heuristic failed at least once. The home-pose wrist sits at rotated
   (350,818)±20 in every t0 frame.
4. Distortion is mild (k1 ≈ −0.13, straight table edges); an unconstrained distortion
   fit runs away and warps the world — keep k2=0 and pp near center.
5. Tape-only calibration is DEGENERATE (clustered wrist orientations let tool-frame
   offsets trade off against extrinsics ~10–20 cm). The static anchors (goal rays +
   can tops) are what pin the frame.
