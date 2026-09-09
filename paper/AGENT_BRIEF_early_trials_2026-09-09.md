# Agent brief: the 120 recovered "early" trials — state, tools, and where to push next

Hand this to a fresh agent with `paper/EARLY_TRIALS_ROTATION_2026-09-08.md` (the full account) and
`paper/CONFOUNDS.md` (the project's living ledger — check every claim against it). This file is the
operational version: what exists, how to run it, what is known-broken, and what is worth trying.

**September 9 follow-up — world provenance must be repaired before reuse:** the recovery and
rendering scripts omit the variant post-build hook. They apply yaw/riser/shelf geometry but retain
base arm gains and omit the goal spawn-height adjustment. Gravity compensation is already applied by the pre-hook. A full-length
uid-118 paired diagnostic reproduces picked=true in the archived pre-only setup and picked=false
with the full hook, with identical source tape and can position. Thus the counts below describe
the archived path, not independently verified full w3+yaw demonstrations. See CONFOUNDS 56 and
[`eef_recovery_2026-09-09/README.md`](eef_recovery_2026-09-09/README.md). Existing winners are preserved;
do not resume a corrected search into the same result files or mix the two world setups.

## 1. What this is

The project compares human vs machine demonstrations for imitation learning on a Kinova gen3-lite
pick-and-place-and-slide task (pick a can off the table, set it on a shelf, push it against a goal
can). The human corpus in every frozen set is **74 trials, all from one afternoon (2024-12-18)**.

On 2026-09-08 it turned out that of 224 raw trial recordings, **120 (uids 110–231, recorded 12-15,
12-16, 12-17) had never been extracted at all** — the "224 → 93" was non-ingestion, not quality
attrition. Those 120 are now extracted and placement recovery was attempted. The archived search
labels 37 as contact or nested; these are candidates for full-task recovery, not 37 verified complete
demonstrations in the fully configured world (see the September 9 follow-up above).

## 2. State as of 2026-09-09 03:00

| | |
|---|---|
| tapes extracted | 120/120 (`inthewild_trials/<uid>_{episodes,cartesian}.npy`, uids 110–231) |
| clean candidates | 89 (lift ≥ 15 cm and released at shelf height; 12-15's 3 trials excluded, different camera rig) |
| **recovered** | **88 / 89** (only uid 183 unsolved) |
| by stage | 23 nested, 14 contact, 51 picked-only |
| **full-task** | **37** (nested + contact) |
| videos | 88/88 in `can_pos_recovery/videos_recovered/<uid>_<day>_<stage>.mp4` (real \| sim, verdicts stamped) |
| winners | `~/wm_fix_2026-09-03/recover_early/winners.json` — **disclosed separate pool**; `trial_placements.json` and all frozen sets UNTOUCHED |

Outcome of the 88 replays: 37 completed · 31 on the shelf but short of the goal · 18 lifted to shelf
height then fell · 2 never got up. Of the 51 picked-only, 39 % end below the shelf top and **95 % of
those had been carried above shelf height first** — i.e. mid-carry drops, which is the known w3
in-hand ejection mechanism (CONFOUNDS row 49), not necessarily a bad demonstration.

## 3. The one physical fact you must not lose

**The arm sat at a different yaw relative to the table/shelf/goal on the early days.** Measured from
camera-independent tool poses (pick AND place bearings shift together, which a moved goal cannot
produce; cam0 is bolted to the stand and barely changes, so the stand stayed put):

| day | yaw vs 12-18 | variant |
|---|---|---|
| 12-16 | −9.7° | `gc_kp4_riser3_shelf6_yaw16` |
| 12-17 | −19.2° | `gc_kp4_riser3_shelf6_yaw17` |

Each variant differs from w3 (`gc_kp4_riser3_shelf6`) by **exactly one key**, `yaw`. The scene is
authored in base-frame coordinates, so rotating the MOUNT realizes "the arm is turned relative to the
scene". Goal, shelf and table keep their w3 positions and are correct — do not move them.

**Consequence that cost a whole run:** a tape's tool pose is in THAT DAY'S ARM FRAME. The sim replays
joint angles through a yawed mount, so the tool lands at `R(yaw) · p_tape` in world coordinates. Any
seed, IC or can position derived from a tape must be rotated by `R(yaw)` before being placed in the
sim world. `recover_early.seeds()` does this (`YAW_DEG`); anything new must too.

## 4. Commands

```bash
PY=~/workspace/genesis_sim2real/venv/bin/python           # sim venv (Genesis)
EV=.venv-eval/bin/python                                  # has `rosbags` (bag reading); sim venv does NOT

# extract a bag -> tapes  (ROS1-free port; --verify diffs against the 96 pre-existing tapes)
$EV trial_reader_episodes_rosbags.py --verify
$EV trial_reader_episodes_rosbags.py --early
$EV trial_reader_cartesian.py <uids...>

# placement recovery (plan -> parallel rollouts -> winners); ONE VARIANT PER PROCESS
$PY can_pos_recovery/recover_early.py plan --out $D
$PY can_pos_recovery/recover_early.py work --out $D --slice K N --variant gc_kp4_riser3_shelf6_yaw16
$PY can_pos_recovery/recover_early.py pick --out $D

# videos (real | sim, both stage verdicts stamped per frame)
$PY can_pos_recovery/render_recovered.py --out $D --variant <V> [--shard K N] [--stages ...]

# review sheets for raw trials (frames + gripper/tool-z signals)
$PY can_pos_recovery/review_early_trials.py --per-sheet 10

# world selftest — ALWAYS run after touching sim_variants
$PY baselines/sim_variants.py --selftest gc_kp4_riser3_shelf6_yaw16
```

`$D = ~/wm_fix_2026-09-03/recover_early`. Everything checkpoints per rollout and skips completed work,
so any of it can be killed and resumed.

## 5. Traps that have already bitten (do not re-learn these)

1. **Never `pgrep -f` / `pkill -f` a pattern that appears in your own command line.** It matches the
   shell running it. This killed two shells and a running chain in one session. Kill by PID.
2. **Seeds must be rotated into the world frame** (§3). Run 1 skipped this and got 40/89 instead of
   88/89. The tell was in its own output: *every winner sat on the search-grid boundary*. A search
   whose solutions pile up at the edge of its grid has been stopped by the fence.
3. **The renderer OOM'd twice** by accumulating frames. It now streams to disk and caps output at 400
   frames per video; workers sit at ~2.5 GB. Do not raise the cap without watching RSS.
4. **Genesis morphs ignore `euler` set after construction** — `quat` is derived at `__init__` and used
   thereafter. Set `quat`. The selftest reads the BUILT base orientation back, which is the only
   reason this was caught.
5. **`rosbags` is not in the sim venv.** Bag work goes in `.venv-eval`.
6. Do not trust a tape's `stage` field: `_nested()` only runs when an episode hits `max_steps`, and
   most tapes end `adapter_exhausted`. Use the honest re-score json.

## 6. Open threads, roughly in value order

1. **Verify run-2 stage labels against an independent replay.** In run 1 the search and the renderer
   disagreed on 3 of 31 (search said nested, replay said contact) — a ~10 % gap between two
   separately-written execution paths on the same tape and can position. Run 2 has NOT been checked.
   Do this before anyone trains on the pool; it directly changes the "37 full-task" number.
2. **The 51 picked-only trials.** 18 are mid-carry drops (likely w3's in-hand ejection, CONFOUNDS 49);
   31 reach the shelf but stop short. Widening the candidate grid or refitting yaw PER TRIAL (rather
   than per day) is untested and cheap — the current grid is 25 points at 1/2/4 cm around one seed.
3. **uid 183** — the single unsolved trial.
4. **Grading and dataset build.** These tapes have no `demo_manifest` entry and no dataset. Any
   training use needs its own pre-registration (the pool is disclosed and separate by design).
5. **Lineage caution.** The July placements for the 74 were recovered in the BASE world; these were
   recovered in w3+yaw. If the two pools are ever merged, that difference must be disclosed.
6. **12-15** (3 trials, different camera rig) was excluded throughout and never attempted.

## 7. Things that are NOT problems (already chased and closed)

- **The goal position is correct.** A "4.7 cm misplaced goal" I reported was my own arithmetic error
  (an offset vector not rotated into the early-day frame). Correctly computed the goal error is 2.5 cm
  on 12-16 and 1.1 cm on 12-17, both within the estimate's noise.
- **No mount translation is needed.** A rotation+translation rigid fit scored *worse* than
  rotation-only on 12-17 (1.8 cm vs 1.1 cm). Dropped.
- **The early trials are genuine**, not junk: 101/120 pick-like, median lift 19.1 cm vs 19.9 cm for the
  known-good set, identical bag topics and rates.
- **Participant identity is not recoverable.** No field anywhere (`config.yaml` byte-identical across
  all 224 except `data_dir`; `user_NNN` is a trial counter). Four sessions on four consecutive days is
  the only grouping the data supports.

## Active full-world follow-up

The separate `paper/eef_recovery_2026-09-09/early_yaw_pool/` now declares all 89
early candidates for timestamp-based EEF replay with both hooks and built-world
yaw/gain/shelf readback. This is distinct from the 74-trial December-18 timing
census. Old winner poses remain outcome-fitted; old labels are not inherited.
