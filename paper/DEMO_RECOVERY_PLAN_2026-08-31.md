# Demo recovery plan — getting from 72+8 usable tapes back to all 91 (2026-08-31)

## Inventory (verified against CLAUDE.md accounting + METHODS §4.6)
- Recorded: 93 labeled trials = **75 success + 16 fail + 2 stubs** (stubs excluded forever: gripper
  never closes). The unlabeled trials 200–231 (~32 more) were never ingested — out of scope here.
- Usable today: **72 solved successes** (can-position recovered well enough for commanded replay;
  56 of them enter the matched sets) + **8 of the 16 fails** (the other 8 are excluded by the
  lying-can/success-label filters — incl. the two known-bogus 90°@z=0.085 placement artifacts).
- Missing from 91: **3 unsolved successes** (255 and 303 confirmed; one more never re-solved after
  the world corrections) + **8 excluded fails**.

## Why this is worth doing (beyond the 11 tapes)
The 07-08 panel's standing criticism: many "solved" placements are SEARCH FITS, not recoveries —
replay-optimized winners drifting up to 17 cm from FK, 60 % dependent on goal relocation. A
camera-derived can position per trial converts the weakest methodological point of the pipeline
("placements are optimized to make replay succeed") into a validated measurement. Either the
camera confirms the fits (a strong audit result to cite) or it doesn't (better now than from a
reviewer). Note: every live comparison is matched-N, so recovered tapes buy IC coverage and audit
strength — they do not change existing claims.

## The pipeline (side-by-side from the original ROS bags and the sim)
Runs on the machine that has the raw bags (this box has none — `inthewild_trials/` is extracted
command tapes only) with a ROS1 environment (`trial_reader.py` / `can_pos_recovery/read_bag.py`
already require it).

1. **Extract** per trial: the t0 camera frame(s) + tool_pose stream (already parsed by
   trial_reader). Camera of record: the overhead/oblique view used in the 07-08 Hough prototype.
2. **Calibrate without factory intrinsics — FK-homography** (the 07-08 plan, prototyped): collect
   (image-point, table-plane-point) pairs from the gripper at known FK poses across many trials
   → fit a table-plane homography per camera mount epoch (check for remounts across the
   collection dates; fit one H per epoch).
3. **Detect the can at t0**: Hough circles (worked in the 07-08 prototype on bag frames). The
   STARTING can sits near the workspace center — a much friendlier target than the goal can that
   killed the "diameter ruler" idea (clipped, oblique corner view).
4. **Score all 91**: camera (x,y) vs `trial_placements.json` distance; flag > ~3 cm. Three buckets
   expected: confirmed fits; fixable drifts (re-solve with camera-seeded position); genuinely
   ambiguous (occlusion at t0 → try the first N frames instead).
5. **Re-solve** the 3 unsolved successes + every flagged trial with `cpu_research.py` seeded at the
   camera position (both modes exist: ok-class construction and `--env-path` collector-basin).
   CPU-parallel, cheap.
6. **Re-audit the 8 excluded fails** with camera positions: which exclusions were placement
   artifacts (re-admit with corrected placement) vs genuine (can already lying at t0 — visible in
   the frame, which is itself the definitive label).
7. **Side-by-side validation videos**: bag camera vs sim rig camera, synced by command index —
   the sim-side render machinery exists (`can_pos_recovery/videos_realsim` pipeline); the new
   piece is a small compositor stacking the bag frame next to the sim frame per step. This is the
   artifact the user eyeballs, and the paper's audit figure.

## Effort estimate
- Bag machine bring-up + extraction: the existing scripts do most of it (needs ROS1; ~1–2 h).
- Calibration + detection + scoring: ~half a day (prototype exists for detection; homography new).
- Re-solves: CPU-parallel, hours, unattended.
- Side-by-side compositor: ~1–2 h; renders unattended.
- Decision points: threshold for "confirmed" (suggest 3 cm ≈ half can diameter); whether
  re-admitted fails re-enter the fails arms (would need a PREREG amendment — the current (1b)
  result is complete at 8 fails/source and should NOT be rescored; new tapes go to a disclosed
  v2 set if used at all).

## What lands in the paper
- Audit paragraph: "camera-derived can positions confirm X/91 recovered placements within 3 cm
  (median error Y cm); Z placements were corrected and re-solved" — answering the panel's
  17 cm-drift criticism with a measurement.
- Optionally +3 successes (75/75 solved) and a fails census with ground-truthed exclusions.
