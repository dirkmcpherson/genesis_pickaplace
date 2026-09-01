# Camera-audit gate: PASS, with a protocol correction (2026-08-31)

Executed steps 0–1 of the revised demo-recovery order (see
`CRITIQUE_demo_recovery_2026-08-31.md`): t0-frame census of the 21 unsolved trials, then
the calibration go/no-go gate. Both done on THIS box (the plan's "bag machine" premise
was wrong — bags + videos are local in `inthewild_trials/raw/`, no ROS needed).
Pipeline + model + overlays: `can_pos_recovery/camera_audit/`.

## Verdict
**GO — the instrument reaches the required precision, and the eyeball pass already
paid for itself.** But the audit as originally designed (camera-at-t0 vs
trial_placements) would have produced mass false flags: placements encode GRASP-time
positions, and cans move between t0 and grasp. Score at close-time or don't score.

## 1. t0 census of the 21 unsolved trials (10 minutes, settled real questions)
Viewed t0 (+3 s) frames for all 16 unsolved successes + 5 unsolved fails
(overlays for the notable ones in `camera_audit/overlays/`):

- **No lying start can anywhere.** The lying-can phenomenon is absent from the
  unsolved set at t0.
- **Arm-base edge cluster**: 4/5 unsolved fails (268, 282, 288, 324) AND successes
  301, 319 start with the can at the far-left frame edge, tight to the robot base —
  plausibly outside the placement search basin, and the worst camera region
  (edge distortion + partial occlusion + frame cutoff). The unsolved and the
  hard-to-measure sets overlap, as the critique's selection-bias point predicted.
- **Goal-state anomalies**: 255's goal can is visibly disturbed (leaning behind the
  shelf leg, persists at 3 s) — likely why 255 never solved: the static-goal world is
  wrong for it BY CONSTRUCTION. 262 suspect (may be acrylic refraction), 329 mild.
- **Clean mid-table starts** (prime camera-seeded re-solve candidates): 233, 259, 266,
  267, 275, 278, 321, 333. Trial 233's world looks completely normal — its
  never-solved status is a search/trajectory failure, not a world anomaly.
- **303 is permanently unmeasurable** (camera dirs empty — no video recorded). Also
  no video: 245 247 263 270 300 308 326 330 (9 labeled trials total — 272/291 have
  empty camera dirs too but are not labeled trials — so the audit ceiling is 82/91,
  not 91; all 9 no-video trials except 303 are already solved).
- Arm starts moving <3 s in; by 3 s it can occlude the start can (255). t0-frame
  measurement window is real but tight.

## 2. Calibration gate (cam4)
Model: pinhole (fx 917, fy 892, cx 639, cy 360, k1 −0.127; k2 pinned 0 — table edges
are straight, big-distortion fits are runaway artifacts) + extrinsics + two tool-frame
wrist-tape offsets (|o| 6.7/4.0 cm, physical) + fitted goal-top z. Data: 407
(tool_pose, tape-pixel) pairs from 13 trials spanning 232→333 + 5 can-top anchors +
8 goal rays. Euler conv `zyx` on kortex tool_pose_theta.

- **Goal rays: median 5 px, max 11 px** (≈0.5–1 cm) — and the goal-top pixel agrees
  within ±6 px across every ok-trial t0 frame ⇒ the goal can physically sat at ONE
  spot across these trials. First direct visual confirmation of the static-goal
  assumption (with per-trial exceptions like 255 now identifiable by camera).
- Tape residuals: median 9 px, 81 % < 20 px.
- **End-to-end check on real frames**: projecting ok-class placements into
  close-time frames lands ON the visible can tops — 242 ≈ 15 px (~1.5 cm),
  286 ≈ 35 px (~3.5 cm) — verified by eye (`overlays/topc_242.jpg`, `topc_286.jpg`,
  cyan = projected placement).
- Failure modes burned through en route (details in camera_audit/README): tape-only
  calibration is degenerate (10–20 cm off while self-consistent — the first
  "held-out 1–2 cm" result was a lenient-metric illusion); unconstrained distortion
  warps the world; every largest-blue-blob heuristic failed at least once (cans,
  tape, and elbow patch are all blue). Static anchors are what pin the frame.

**Instrument precision: ~1.5–3.5 cm on the can-top plane in the central workspace.**
Good enough to adjudicate a ~3 cm threshold there; NOT yet demonstrated at the
arm-base edge cluster (no anchors there — flag edge trials separately in the audit).

## 3. Protocol correction (would have invalidated the naive audit)
- t0 detections of 250/256/286/295/311 (all pos-class 2) cluster within ~10 px while
  their placements spread 10 cm → looked like the panel's "search fits" smoking gun.
  It isn't: bags show cans are nudged/re-placed between t0 and grasp (286: closure at
  t=71 s) and dragged after closure (295: 11.4 cm, 260: 17 cm). Placements encode the
  grasp position — **the audit must compare at the closure that initiates the
  persisting lift, not at t0** (261's first closure is a 19 cm drag/regrasp; naive
  first-closure scoring would flag it at 20 cm).
- **Camera-free consistency stat** (new, `close_consistency.py`): |tool close-xy −
  placement| over the 15 videoed ok-class trials: **median 2.9 cm, 14/15 ≤ 5 cm**
  (the 20 cm outlier is the 261 regrasp bookkeeping case). The ok-class placements
  are consistent with the robot's own grasp locations at roughly the grasp-offset
  noise floor — an audit-grade number that needed no camera at all.

## 4. What remains for the full audit (NOT run today — PREREG first)
1. **PREREG the audit** before scoring anything: expected confirm rate, threshold
   derived from the measured instrument error (~3.5 cm central, edge trials reported
   separately), and committed decision rules for the disconfirm branch (which arms
   rerun/disclose at which flag rates). The 07-08 panel shape must not recur.
2. Click pass (human/agent, zoomed crops) for can tops at close-time across the 80
   videoed trials — auto-detection at close-time is not reliable (occlusion +
   goal-can confusion); the pipeline's projections make clicking fast (cyan marker
   = where the placement claims the can is).
3. Refit with all clicked anchors, leave-out validation, then score; re-solve flagged
   + unsolved with the within-instrument-radius acceptance rule
   (`cpu_research.py --seed` from camera positions).
4. Goal-state audit for 255/262/329 (+ any flagged) from cam0/cam4 — decide
   explicitly whether per-trial goal exceptions enter the paper.
5. 234/318 (lying-spawn ok-class) and any lying-can fails need a side-view observable
   (top-face detection assumes upright); handle in the click pass.

## Effort actually spent vs the plan's estimate
Census + full calibration gate + protocol correction: one session, zero bag-machine
work, zero ROS. The plan's "1–2 h bring-up + half-day calibration" was wrong in both
directions: the bring-up was free, the calibration was the hard part (three degenerate
fits before the anchored one).
