# Demo recovery results — 2026-08-31 (evening session)

Goal: recover as many unsolved demos as possible, ideally through nested.
Method: `cpu_research.py` wide grids (upright only, per protocol) anchored at FK seed +
grasp-closure position, CPU-verified winners, ×2 reproduction (idle box), safe merge.
All shards/logs: session scratchpad `rescue/`; merged into `trial_placements.json`
(backup `.bak_0831_1741`). Search tag: `cpu_research_recovery_0831`.

## Headline
**13 of the 14 unsolved successes recovered** (the 12 never-searched + 255), plus
fail-trial 240 as a bonus — **7 through NESTED**, the full task. Solved successes:
61 → **74 of 75**. Only 303 remains (control-limited: 0/32 candidates lift the can,
max_z flat at 0.105 everywhere in a 6 cm ring; no video exists to aim better).

| uid | real stage | recovered stage | winner xy | note |
|---|---|---|---|---|
| 233 | nested | **nested** | (0.4673, −0.0294) | the user-validated "perfect" demo; winner = FK seed verbatim |
| 255 | (success; replay was no-pick) | **nested** | (0.4445, −0.0331) | goal-disturbed at t0 in real video, still nests vs static goal |
| 262 | nested | **nested** | (0.4575, −0.0338) | |
| 267 | nested | **nested** | (0.4231, −0.1789) | drag demo (FK 12 cm from close; close-anchored candidate won) |
| 275 | nested | **nested** | (0.4302, −0.1509) | |
| 301 | nested | **nested** | (0.4862, 0.0995) | arm-base edge start; contact basin ≥ ±6 cm wide |
| 321 | nested | **nested** | (0.4243, 0.0161) | |
| 259 | placed | placed | (0.4381, −0.2) | |
| 266 | placed | placed | (0.4502, −0.0361) | |
| 278 | placed | placed | (0.4451, −0.1343) | |
| 333 | placed | placed | (0.4381, −0.2) | |
| 319 | placed | picked | (0.3827, 0.0212) | drag demo; round-2 upgrade attempt: see below |
| 329 | placed | picked | (0.4578, −0.022) | round-2 upgrade attempt: see below |
| 240 | picked (fail label) | picked | (0.4273, −0.1259) | fails-v2 pool candidate (PREREG amendment required before any arm use) |
| 303 | (success; replay no-pick) | UNSOLVED | — | no video; grid exhausted; control-limited |

Striking correspondence: every recovered stage ≤ its real stage, and **all six real
nested demos re-nest** (233, 262, 267, 275, 301, 321) + 255. The old "fk-seed" label
on these trials meant *never searched*, not *unsearchable* — for several the FK seed
itself wins (233 exactly).

## Validation honesty
- Winners are CPU-verified in the scoring basin by construction ('ok' class).
- ×2 reproduction: all 14 winners re-ran once more, **0/14 stage flips** (box idle;
  consistent with the 07-20 determinism finding). A third rep was launched and the
  task was stopped externally mid-run (rep 2 completed; rep 3 did not start) — if the
  official record wants strict ×3, rerun: `rescue/driver_val.sh` (rep 3 only).
- Merge preserved the existing world block (`merge_and_validate.py`'s hardcoded WORLD
  is STALE — finger_kp 100 vs current 40 — do not use its `merge` path).
- These placements are search fits validated by replay, like all ok-class entries; the
  camera-audit instrument (`can_pos_recovery/camera_audit/`) can score them at
  close-time once the PREREG lands (`CAMERA_GATE_2026-08-31.md` §4).

## What did NOT happen (scope discipline)
- No dataset/matched-set changes: the paper's frozen sets are untouched. New tapes
  (e.g., re-collected 233/255/262/... episodes) would go to a disclosed v2 pool only.
- (1b) fails arms not rescored; 240 is merely available for a future fails-v2.
- The 4 no-pick fails (268, 282, 288, 324) stay unsolved — nothing observable
  validates a placement for a demo that never touches the can via search; their
  placements are a camera-audit deliverable (t0 = grasp-free, so t0 measurement IS
  valid for them — unlike success demos).

## v2 tape pool COLLECTED (same evening): `baselines/episodes_recovered_v2/`
All 14 recovered trials re-collected in the TRAINING-ENV basin (collect_all_classified,
`--ee`, env.max_steps override in place): **14/14 tapes, 23,191 transitions, integrity
verified** (states/actions/n consistent, states_ee present). Stage manifest
`_stage.json`. Env-basin stages: **nested 7** (233, 255, 262, 267, 275, 301, 321),
**placed 5** (259, 266, 278, 333, **319**), **contact 1** (**329**), picked 1 (240).
Basin finding: env-path ≥ replay-basin for EVERY tape — 319 picked→placed and 329
picked→contact IMPROVE in the basin the tapes actually live in (the round-2 "flake"
below was basin difference, not noise). This pool is DISCLOSED-V2 only: it does not
touch the frozen matched sets; any training use needs a PREREG amendment.
Eyeball videos: `can_pos_recovery/videos/<uid>_*.mp4` (render_videos, replay basin;
outcome suffixes from this batch use the LEGACY hard-contact nested scorer — the
script is now patched to the proximity metric, and mislabeled files were renamed).

## Rounds 3+ (late evening): the no-pick decomposition and the ok_batch sweep
Listing the 24 success demos whose tapes were no-pick exposed three classes:
(a) **11 stale tapes** — `ok` placements whose July episodes_all tapes predate their
solve (collected at FK seeds); (b) **12 `ok_batch`** — GPU-batch winners never
CPU-verified (the documented transfer cliff); (c) 303.

**Stale re-collection: 11/11 improved, none stayed no-pick** — nested 247, 256, 320,
330 (the 07-20 re-search winners whose tapes were never refreshed); contact 295;
placed 246, 250, 286; picked 234, 293, 318.

**ok_batch CPU re-search (round 3, wide grids + rings around batch positions):
11/12 solved**, ×3-validated at low load (0 flips), merged
(`cpu_research_recovery_0831b`, backup `.bak_0831_2045`): nested 294, 299, 300
(the max_steps-bug flagship), 328; placed 239, 244, 245, 287, 331; picked 236, 315.
**237 = the one refusal, convicted by camera**: the projected seeds sit exactly on its
visible can (far-right table edge, cross-body reach) and 0/31 candidates lift with
best max_z 0.118 — control-limited, not placement-limited (`calib/preclose_237.jpg`).

Re-collection into the v2 pool: 10/11 transferred at or above their replay stage
(236 picked→NESTED); **294 flipped nested→no-pick in the env basin** — the two-basins
gap in the harmful direction; targeted `--env-path` re-search run (result below).

## FINAL FUNNEL (training-env basin, episodes_all + v2 overlays, single-collection)
| phase | 75 successes | all 91 non-stub |
|---|---|---|
| picked | **72/75 = 0.96** | 80/91 = 0.88 |
| placed | **67/75 = 0.89** | 72/91 = 0.79 |
| contact | **33/75 = 0.44** | 34/91 = 0.37 |
| nested | **25/75 = 0.33** | 26/91 = 0.29 |

(Session start: picked 0.67, placed 0.63, contact 0.29, nested 0.21 on the 75.)
Remaining no-pick successes: 237 (camera-convicted control-limited), 303 (no video),
294 (pending env-path search). v2 pool: 36 tapes / 91,650 transitions, all `--ee`.
Caveat: single-collection stages, not idle-×3 official rates.

## Round 2 (319/329 upgrade attempt): no upgrades in the replay basin — and a flake
that was really a basin gap
Fine ±1.5 cm grids around both winners + a 6-point bridge along 319's 13 cm drag line
toward its lift closure. 319 stays picked everywhere in the REPLAY basin (its
drag-carry is open-loop-hard there). 329 produced ONE "contact" candidate under 7-way
load that came back **picked** on both solo validation reps — rejected for the
replay-basin record; 329 keeps its round-1 winner. Post-hoc, the v2 collection showed
both trials do BETTER in the env-path basin (319 placed, 329 contact), so the
borderline behavior was the two-basins gap surfacing, compounded by load sensitivity
(07-20 finding). The reproduction protocol still stands — it is what caught this.
