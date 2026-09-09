# The 120 never-ingested trials: extraction, the arm-yaw finding, and placement recovery (2026-09-08/09)

Sim-box session. Prompted by the user asking how many participants are in the dataset, which turned up
a much larger fact: of 224 raw trial recordings, only 104 (uids >= 232, the 2024-12-18 session) had
ever been extracted. **120 trials from 12-15, 12-16 and 12-17 had never been ingested at all.**

## 1. What was there

`inthewild_trials/raw/user_*` holds 224 trial directories, uids 110-335, recorded over four days.
Every `_episodes.npy` and `_cartesian.npy` on disk was uid >= 232. The other 120 have bags (120/120,
median 73 MB vs 76 MB for the ingested set), the identical topic set at the same rates, 88 with the
cam4 video and 80 with cam0. They are real manipulation attempts: 101/120 show a gripper closure
whose tool z then rises >= 5 cm, median lift 19.1 cm against 19.9 cm for the known-good set.

Sessions, from the per-frame video timestamps (bag start_time for the 29 without them):

| session | date | time | n | uids |
|---|---|---|---|---|
| 1 | 2024-12-15 | 13:49-13:51 | 3 | 110-112 |
| 2 | 2024-12-16 | 12:49-16:49 | 67 | 113-181 |
| 3 | 2024-12-17 | 13:08-16:41 | 50 | 182-231 |
| 4 | 2024-12-18 | 12:41-17:51 | 104 | 232-335 |

**No participant identifier exists anywhere.** `config.yaml` is byte-identical across all 224 trials
except its `data_dir` line; `user_233` is a trial counter, not a person; the bag topic list has no
participant topic; `METHODS_draft_2026-08-28.md` states no participant count. Four sessions on four
consecutive days is the only grouping the data supports, and it is equally consistent with one person
over four days or four people on one day each. The 74 census uids are all from session 4 -- **the
dataset of record is one afternoon's recording.**

## 2. The arm moved between sessions

The tool poses are camera-independent, so this is measurable regardless of the camera (which also
moved: per-day median backgrounds differ by 9.6 and 11.1 grey levels against 12-18). Taking the
bearing from the robot base to the grasp and to the release, on clean carries only:

| day | n | pick bearing | place bearing | vs 12-18 |
|---|---|---|---|---|
| 12-16 | 44 | +5.5 deg | +1.9 deg | **~+7 to +10 deg** |
| 12-17 | 23 | +14.5 deg | +12.3 deg | **~+16 to +20 deg** |
| 12-18 | 73 | -1.7 deg | -7.9 deg | reference |

Pick and place bearings shift TOGETHER, which a moved goal can cannot produce -- it is the arm turned
relative to the table/shelf/goal. cam0 is bolted to the acrylic stand and its view barely changes
across 12-16/17/18, so the stand stayed put. Fitting the angle on the goal alone and then checking the
held-out pick cloud: 12-16 phi -9.7 deg (goal residual 1.9 cm, pick 2.8 cm), 12-17 phi -19.2 deg
(0.2 cm, 6.3 cm).

## 3. Tools built

- **`yaw` sim-variant parameter** + `gc_kp4_riser3_shelf6_yaw16` / `_yaw17`. Each differs from w3 by
  exactly one key; kp_mult 4.0, kv_mult 2.0, gravity_comp 1.0, riser 0.03, shelf_dz 0.06 unchanged.
  The scene is authored in base-frame coordinates, so turning the MOUNT realizes "the arm is turned
  relative to the scene". Selftests read the BUILT base orientation back and assert the scene did not
  move with it.
- **`trial_reader_episodes_rosbags.py`** -- a port of `trial_reader.py`'s windowing to pure-python
  `rosbags`, since ROS1 is not installed here. `--verify` re-reads the 96 trials that already have a
  tape: frame counts identical 96/96, values identical on most, worst residual 1.37e-02 rad.
- **`recover_early.py`** (seeds -> grid -> parallel rollouts -> winners) and
  **`render_recovered.py`** (real | sim replay with both stage verdicts stamped per frame).

## 4. Recovery result

| | run 1 (bad seed) | run 2 (corrected) |
|---|---|---|
| recovered | 40 / 89 | **88 / 89** |
| 12-16 | 36 / 49 | **49 / 49** |
| 12-17 | 4 / 40 | **39 / 40** |
| nested | 5 | **23** |
| contact | 5 | **14** |
| picked only | 30 | 51 |
| rollouts | 2222 | 1895 |

**37 complete the full task** (23 nested + 14 contact). Only uid 183 unsolved. Winners are in
`~/wm_fix_2026-09-03/recover_early/winners.json` as a DISCLOSED separate pool; `trial_placements.json`
and every frozen set are untouched. Training use requires its own pre-registration.

## 5. Errors made, and the one signal that would have caught them

Run 1 seeded the can at the tool xy from the tape, which is in THAT DAY'S ARM FRAME, and placed it in
the sim WORLD frame without applying the yaw. The sim replays joint angles through a yawed mount, so
the tool lands at R(yaw) * p_tape -- 8.5 cm away at 9.7 deg, 16.6 cm at 19.2 deg. That single line
explains the whole run-1 shortfall.

**The signal was in run 1's own output and I misread it three times:** every winner sat on the search
grid boundary (median 4.0 cm on 12-16; all four 12-17 winners at exactly 5.7 cm, the corner). A search
whose solutions pile up at the edge of its grid has been stopped by the fence, not found the answer.
Instead I diagnosed, in order: (a) a missing mount translation -- the rigid fit's translation term was
real arithmetic but the goal-error check later showed rotation-only was as good or better (12-17:
1.1 cm rotation-only vs 1.8 cm rigid), so it was dropped; (b) a misplaced goal at "4.7 cm" -- that
number was my own arithmetic error, subtracting a 12-18-frame offset vector inside the early-day frame
without rotating it; corrected, the goal error is 2.5 cm (12-16) and 1.1 cm (12-17), and the goal had
been at its correct world position all along. Both wrong diagnoses were reported to the user before
the real cause was found.

Also disclosed: the run-1 stage labels disagreed with an independent render replay on 3 of 31 trials
(140, 148, 177 -- search said nested, replay said contact), a ~10 % reproducibility gap between two
separately-written execution paths over the same tape and can position. Run 2's labels have not yet
been checked the same way.

## 6. Not done

- uid 183 unsolved; 12-15 (3 trials, different camera rig) excluded throughout.
- The `_episodes.npy` tapes for the 120 are written; no `demo_manifest` grading, no dataset build.
- The run-2 search-vs-replay agreement check.
- Whether these trials share a demonstrator with session 4 is unknown and unknowable from the files.
