# `slide_success` smoke test on {RLPD} POLICY rollouts (Lane 9, 2026-09-11)

42 clips, 5.0 MB. Every clip is a **policy rollout** under the unified ladder — not a demonstration tape. The overlay is the one from `can_pos_recovery/videos_ladder_2026-09-11/`, drawn by the same `annotate_demos._draw_panel` / `_terminal_card`, so the conventions are the ones you already know: **yellow = fired within the last 6 decisions, green = granted, grey = not yet**, `d<N>` under a lit chip is the decision it fired on, and no chip or timeline tick shows a grant that has not happened yet.

**One counting convention to know** (it is inherited from the demonstration clips, so the two sets agree): a chip's `d<N>` is the **0-based index** of the decision that granted the stage, while the diagnostics line's `d<N>/<M>` counts **decisions taken**. The frame on which a chip labelled `d122` first lights therefore reads `d123/…` on the line below it. Same event, two bases.

```
rollouts : baselines/eval_e2e_annot.py --kind sac --mode sample --ladder staged
           --ic-file baselines/eval_ics.json --ic-set rnd|hold --max-steps 1200 --threads 2
driver   : baselines/diagnostics/slide_smoke_rollouts.sh
index    : baselines/diagnostics/slide_smoke_index.py
ladder   : staged
stamp    : unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 | max_return=8 | terminal=slide_success+tipped | shaping=off | full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 | git=known-good-2026-08-27-842-ga1d8fd0-dirty / unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 | max_return=8 | terminal=slide_success+tipped | shaping=off | full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 | git=known-good-2026-08-27-843-gfc0148c-dirty
```

## Episode budget

| checkpoint | start set | seeds | episodes |
|---|---|---:|---:|
| `dH_s901` | hold | 0,1 | 30 |
| `dH_s901` | rnd | 0,1,2 | 90 |
| `dH_s940` | hold | 0,1 | 30 |
| `dH_s940` | rnd | 0,1,2 | 90 |
| `dH_s941_c040` | hold | 0 | 15 |
| `dH_s941_c040` | rnd | 0,1 | 60 |

Seeds vary the ACTION SAMPLING, not the starts: each seed replays the same 30 (`rnd`) / 15 (`hold`) starts. So the episode counts are draws, not independent starts.

## Class census (checkpoint x start set)

| checkpoint | set | n | `held_press_timeout` | `nested_drop` | `nopick` | `picked_only` | `placed_only` | `push_no_nest` | `pushed_no_contact` | `reset_artifact` | `slide` | `tipped_after_release` | `tipped_before_release` |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `dH_s901` | hold | 30 | 0 | 1 | 3 | 0 | 1 | 1 | 9 | 0 | 5 | 6 | 4 |
| `dH_s901` | rnd | 90 | 1 | 1 | 3 | 2 | 10 | 2 | 13 | 4 | 15 | 10 | 29 |
| `dH_s940` | hold | 30 | 0 | 1 | 15 | 3 | 1 | 1 | 0 | 0 | 2 | 1 | 6 |
| `dH_s940` | rnd | 90 | 0 | 0 | 49 | 2 | 3 | 1 | 1 | 3 | 1 | 3 | 27 |
| `dH_s941_c040` | hold | 15 | 0 | 0 | 8 | 5 | 0 | 0 | 0 | 0 | 0 | 0 | 2 |
| `dH_s941_c040` | rnd | 60 | 0 | 0 | 30 | 6 | 0 | 0 | 0 | 4 | 0 | 0 | 20 |

| checkpoint | episodes | `slide_success` | `nested_v2` (sticky) | `nested_honest` (settled) | `contact_push` | `pushed` | `placed_v2` | `picked` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `dH_s901` | 120 | **20** | 22 | 24 | 25 | 56 | 78 | 85 |
| `dH_s940` | 120 | **3** | 4 | 4 | 6 | 5 | 18 | 28 |
| `dH_s941_c040` | 75 | **0** | 0 | 0 | 0 | 0 | 4 | 19 |

## How long after the release does `pushed` fire?

`pushed` is one of the three clauses of `slide_success`, and its accumulator starts at the FIRST `placed_v2` grant. So the gap between the two grants says where the 10 mm of "goalward progress" came from. A gap of a few decisions is the can drifting as the fingers open and the tool withdraws — the set-down settle — not a push.

| checkpoint | episodes with `pushed` | 0 = within the release decision | 1-3 | 4-10 | 11-30 | > 30 | median gap | also `contact_push` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `dH_s901` | 56 | 7 | 36 | 7 | 5 | 1 | 1 | 20 |
| `dH_s940` | 5 | 2 | 1 | 0 | 1 | 1 | 1 | 3 |
| `dH_s941_c040` | 0 | 0 | 0 | 0 | 0 | 0 | n/a | 0 |

## What the smoke test found

These are read off the 315 episodes above; each is checkable on the clips listed.

**1. `pushed` is mostly the set-down transient, not a push.** Of the 23 positives, **20 have `pushed` firing within 3 decisions of the release** and **5 fire inside the release decision itself** (4 env frames, ~0.1 s). Across every episode that ever fired it, 46 of 61 land within 3 decisions. The accumulator opens at the first `placed_v2` grant, so the can drifting goalward as the fingers open and the tool withdraws clears the 10 mm threshold on its own. Watch `dH_s940_hold_ep4_slide.mp4`: the arm carries the can to 10.3 cm from the goal with the grip already commanded open, `placed_v2` grants at d122, and the can is at 6.9 cm one decision later — 34 mm in a single decision, three times the threshold, before the arm could have completed a stroke.

**2. 8 of the 23 positives never fired `contact_push` at all** — the ladder paid its top rung (+4, terminal) on episodes with no frame where the tool was demonstrably behind the can touching the goal. `slide_success` = released AND pushed AND `nested_v2`, and none of those three requires a contact frame, so a release that drifts home pays the same as a push.

**3. 2 positives sit within 5 mm of the `in_hand` threshold.** `HELD_LEVER_M` is 0.025 and Lane 1 measured the held tail out to 32.3 mm, recommending 0.030. At 0.030 those episodes read as still-in-hand, so `nested_v2` and with it `slide_success` flip and the +4 is not paid. The constant is load-bearing for the top rung on this evidence, which it was not on the demonstration tapes.

**4. `nested_v2` held up; the settled reference is the one that misfires.** `nested_v2` 26, `nested_honest` 28, agreeing on 26: **no `nested_v2` firing lacked a settled nest** (precision 1.000) and the 2 settles it missed BOTH had `placed_v2` never granted — the robot pushed the can home with the fingers closed and never satisfied the grip-command release clause, and the post-episode settle then scored a can the robot had not released. That is the known "settled nested nests a HELD can" defect, and `nested_v2` refusing them is the better answer. Legacy `nested_proxy` fired 26 times with 14 real settled nests behind them (precision 0.538).

**5. The D2 release-first precondition holds structurally:** 0 episodes granted `contact_push` before or without `placed_v2` (prediction P2 is 0).

**6. Reading for the ladder redesign.** Every defect above is the same shape: no clause requires the TOOL to be near the can while the can moves. The Ladder N proposal in `LADDER_UNIFY_BRIEF_2026-09-10` already fixes exactly this — `farside` requires `|tool_xy - can_xy| <= 0.08 m` on the frame, and `slide_gain` accumulates only on frames where `farside` holds. On this evidence that change is the difference between paying for a slide and paying for a set-down, and it is worth more than raising the slide reward.

## Clips

Every `slide_success` positive is here, uncapped. The failure classes are sampled to a quota, spread over checkpoints where the class occurs on more than one. 6 of the 20 episodes flagged BORDERLINE (just outside a clause) are in this set.

Classes that OCCUR in the census but have no clip here (the budget went to the disputed classes; the counts are in the table above, so an empty row below is not a count of zero): `nopick`, `picked_only`. Every episode is still on disk — see *Where everything lives*.

### `dH_s901_hold_ep7_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 7 (uid 294)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d19, placed_v2 d27, released d27, pushed d31, contact_push d31, nested_v2 d36, slide_success d36, contact_push_legacy d31, nested d31
- **end** — slide_success at decision 37 of 37; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.080 m, gain 17.9 mm, lever 110.8 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_hold_ep0_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 1) — **episode** 0 (uid 252)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d15, placed_v2 d23, released d23, pushed d24, nested_v2 d34, slide_success d34
- **end** — slide_success at decision 35 of 35; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.070 m, gain 102.2 mm, lever 25.3 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_hold_ep3_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 1) — **episode** 3 (uid 265)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d44, placed_v2 d53, released d53, pushed d53, contact_push d55, nested_v2 d59, slide_success d59, contact_push_legacy d55
- **end** — slide_success at decision 60 of 60; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.065 m, gain 55.3 mm, lever 96.5 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_hold_ep8_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 1) — **episode** 8 (uid 295)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d17, placed_v2 d31, released d31, pushed d31, nested_v2 d87, slide_success d87
- **end** — slide_success at decision 88 of 88; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.070 m, gain 87.9 mm, lever 56.7 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_hold_ep10_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 1) — **episode** 10 (uid 311)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d15, placed_v2 d26, released d26, pushed d30, contact_push d31, nested_v2 d39, slide_success d39, contact_push_legacy d31, nested d31
- **end** — slide_success at decision 40 of 40; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.074 m, gain 96.1 mm, lever 74.6 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep4_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 4 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d124, placed_v2 d132, released d132, pushed d133, contact_push d137, nested_v2 d142, slide_success d142, contact_push_legacy d137, nested d137
- **end** — slide_success at decision 143 of 143; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 104.6 mm, lever 53.7 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep20_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 20 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d13, placed_v2 d23, released d23, pushed d25, contact_push d27, nested_v2 d30, slide_success d30, contact_push_legacy d27
- **end** — slide_success at decision 31 of 31; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.065 m, gain 109.4 mm, lever 88.5 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep23_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 23 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d15, placed_v2 d27, released d27, pushed d28, contact_push d33, nested_v2 d49, slide_success d49, contact_push_legacy d33
- **end** — slide_success at decision 50 of 50; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.065 m, gain 115.8 mm, lever 63.8 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.1 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep27_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 27 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d14, placed_v2 d25, released d25, pushed d26, contact_push d61, nested_v2 d72, slide_success d72, contact_push_legacy d61, nested d63
- **end** — slide_success at decision 73 of 73; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.066 m, gain 62.3 mm, lever 27.4 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep1_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 1 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d19, placed_v2 d27, released d27, pushed d28, nested_v2 d33, slide_success d33
- **end** — slide_success at decision 34 of 34; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.078 m, gain 115.0 mm, lever 107.7 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep3_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 3 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d17, placed_v2 d25, released d25, pushed d26, nested_v2 d31, slide_success d31
- **end** — slide_success at decision 32 of 32; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.072 m, gain 68.9 mm, lever 125.6 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep11_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 11 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d21, placed_v2 d27, released d27, pushed d28, nested_v2 d200, slide_success d200
- **end** — slide_success at decision 201 of 201; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.076 m, gain 136.5 mm, lever 39.2 mm, in_hand 0, at_rest 1, can tilt 0.1 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep12_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 12 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d15, placed_v2 d21, released d21, pushed d22, contact_push d57, nested_v2 d62, slide_success d62, contact_push_legacy d57
- **end** — slide_success at decision 63 of 63; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.065 m, gain 128.0 mm, lever 61.3 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep15_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 15 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d23, placed_v2 d31, released d31, pushed d32, nested_v2 d39, slide_success d39, nested d35
- **end** — slide_success at decision 40 of 40; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.070 m, gain 112.6 mm, lever 96.3 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep29_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 29 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d49, placed_v2 d119, released d119, pushed d132, contact_push d123, nested_v2 d138, slide_success d138, contact_push_legacy d123, nested d123
- **end** — slide_success at decision 139 of 139; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 18.5 mm, lever 79.0 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_s2_ep4_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 4 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d63, placed_v2 d71, released d71, pushed d71, contact_push d80, nested_v2 d87, slide_success d87, contact_push_legacy d80, nested d81
- **end** — slide_success at decision 88 of 88; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.066 m, gain 18.5 mm, lever 51.2 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep17_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 17 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d16, placed_v2 d30, released d30, pushed d32, contact_push d62, nested_v2 d65, slide_success d65, contact_push_legacy d62
- **end** — slide_success at decision 66 of 66; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.067 m, gain 130.7 mm, lever 60.2 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_ep18_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 18 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d43, placed_v2 d52, released d52, pushed d55, nested_v2 d60, slide_success d60, nested d58
- **end** — slide_success at decision 61 of 61; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 62.5 mm, lever 164.7 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_s2_ep27_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 27 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d14, placed_v2 d28, released d28, pushed d29, nested_v2 d115, slide_success d115
- **end** — slide_success at decision 116 of 116; reward 6.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.077 m, gain 134.4 mm, lever 38.8 mm, in_hand 0, at_rest 1, can tilt 0.9 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_rnd_s2_ep29_slide.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 29 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d16, placed_v2 d25, released d25, pushed d26, contact_push d27, nested_v2 d35, slide_success d35, contact_push_legacy d27
- **end** — slide_success at decision 36 of 36; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.065 m, gain 58.0 mm, lever 96.4 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s940_hold_ep4_slide.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 0) — **episode** 4 (uid 273)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d25, placed_v2 d122, released d122, pushed d122, contact_push d124, nested_v2 d129, slide_success d129, contact_push_legacy d124, nested d124
- **end** — slide_success at decision 130 of 130; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 27.1 mm, lever 31.2 mm, in_hand 0, at_rest 1, can tilt 0.1 deg, goal tilt 0.1 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s940_hold_ep14_slide.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 1) — **episode** 14 (uid 335)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d34, placed_v2 d195, released d195, pushed d195, contact_push d195, nested_v2 d200, slide_success d200, contact_push_legacy d195, nested d195
- **end** — slide_success at decision 201 of 201; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 17.9 mm, lever 95.6 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s940_rnd_ep16_slide.mp4`

- **checkpoint** `dH_s940` — **start set** `rnd` (seed 1) — **episode** 16 (random start)
- **class** `slide` — POSITIVE. `slide_success` fired: released, >= 10 mm of goalward gain after release, and `nested_v2` true on that frame. It is the ladder TERMINAL, so the clip ENDS on the firing frame -- there is no "after" footage by construction, and the settled verdict on the card comes from the one post-episode settle.
- **first fire** — picked d25, placed_v2 d32, released d32, pushed d33, contact_push d36, nested_v2 d43, slide_success d43, contact_push_legacy d36, nested d42
- **end** — slide_success at decision 44 of 44; reward 8.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.065 m, gain 27.2 mm, lever 75.4 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — Watch the frame the SLIDE chip lights. Did the gripper actually PUSH the can home, or did the can arrive some other way and the three clauses happened to line up? Check the `pushed` decision on the card against the `placed_v2` one: a gap of a couple of decisions means the 10 mm came from the set-down settle, not a stroke.

### `dH_s901_hold_ep3_tipped_after_release.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 3 (uid 265)
- **class** `tipped_after_release` — The tip rule ended the episode after `placed_v2` had been granted.
- **first fire** — picked d35, placed_v2 d47, released d47, pushed d48
- **end** — tipped at decision 53 of 53; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.091 m, gain 13.7 mm, lever 78.0 mm, in_hand 0, at_rest 0, can tilt 64.5 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **BORDERLINE** — dist 90.8 mm (nest needs <= 81)
- **what to check** — The can was set down and then knocked over. Was the release genuine before the tip?

### `dH_s901_hold_ep6_nested_drop.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 6 (uid 284)
- **class** `nested_drop` — `nested_v2` fired with `pushed` FALSE -- the can arrived and settled without 10 mm of post-release goalward travel.
- **first fire** — picked d14, placed_v2 d27, released d27, contact_push d27, nested_v2 d50, contact_push_legacy d27, nested d29
- **end** — truncated at decision 300 of 300; reward 4.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.078 m, gain 9.4 mm, lever 87.8 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **BORDERLINE** — gain 9.4 mm (push needs 10)
- **what to check** — This is the drop route. Confirm the can was placed/dropped into position rather than pushed: if it visibly slid home, `pushed` missed a real push.

### `dH_s901_hold_ep11_tipped_after_release.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 11 (uid 325)
- **class** `tipped_after_release` — The tip rule ended the episode after `placed_v2` had been granted.
- **first fire** — picked d34, placed_v2 d43, released d43, pushed d47, contact_push d48, contact_push_legacy d48
- **end** — tipped at decision 50 of 50; reward 4.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.093 m, gain 69.0 mm, lever 48.5 mm, in_hand 0, at_rest 0, can tilt 72.7 deg, goal tilt 22.5 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **BORDERLINE** — dist 93.1 mm (nest needs <= 81)
- **what to check** — The can was set down and then knocked over. Was the release genuine before the tip?

### `dH_s901_rnd_ep10_nested_drop.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 2) — **episode** 10 (random start)
- **class** `nested_drop` — `nested_v2` fired with `pushed` FALSE -- the can arrived and settled without 10 mm of post-release goalward travel.
- **first fire** — picked d23, placed_v2 d66, released d66, contact_push d112, nested_v2 d68, contact_push_legacy d112, nested d64
- **end** — truncated at decision 300 of 300; reward 4.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.066 m, gain 0.0 mm, lever 52.5 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — This is the drop route. Confirm the can was placed/dropped into position rather than pushed: if it visibly slid home, `pushed` missed a real push.

### `dH_s940_hold_ep6_nested_drop.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 0) — **episode** 6 (uid 284)
- **class** `nested_drop` — `nested_v2` fired with `pushed` FALSE -- the can arrived and settled without 10 mm of post-release goalward travel.
- **first fire** — picked d20, placed_v2 d111, released d111, nested_v2 d124, nested d115
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.072 m, gain 0.0 mm, lever 71.4 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 1 / final-frame 1
- **what to check** — This is the drop route. Confirm the can was placed/dropped into position rather than pushed: if it visibly slid home, `pushed` missed a real push.

### `dH_s901_hold_ep13_push_no_nest.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 13 (uid 331)
- **class** `push_no_nest` — `contact_push` fired (release first, can touching the goal, tool on the far side) but `nested_v2` never did.
- **first fire** — picked d16, placed_v2 d31, released d31, contact_push d34, contact_push_legacy d29
- **end** — truncated at decision 300 of 300; reward 4.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.089 m, gain 5.7 mm, lever 126.9 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 90.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **BORDERLINE** — gain 5.7 mm (push needs 10); dist 89.0 mm (nest needs <= 81)
- **what to check** — The near miss. The diagnostics say which clause failed -- distance just over 81 mm, `at_rest` never holding, or the can tipped. Is the failure the right call?

### `dH_s940_hold_ep4_push_no_nest.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 1) — **episode** 4 (uid 273)
- **class** `push_no_nest` — `contact_push` fired (release first, can touching the goal, tool on the far side) but `nested_v2` never did.
- **first fire** — picked d36, placed_v2 d90, released d90, contact_push d90, contact_push_legacy d90, nested d97
- **end** — truncated at decision 300 of 300; reward 4.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.100 m, gain 0.0 mm, lever 87.6 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **BORDERLINE** — dist 99.8 mm (nest needs <= 81)
- **what to check** — The near miss. The diagnostics say which clause failed -- distance just over 81 mm, `at_rest` never holding, or the can tipped. Is the failure the right call?

### `dH_s901_rnd_ep10_push_no_nest.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 10 (random start)
- **class** `push_no_nest` — `contact_push` fired (release first, can touching the goal, tool on the far side) but `nested_v2` never did.
- **first fire** — picked d19, placed_v2 d68, released d68, pushed d80, contact_push d81, contact_push_legacy d81, nested d93
- **end** — truncated at decision 300 of 300; reward 4.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.095 m, gain 52.6 mm, lever 72.3 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **BORDERLINE** — dist 95.4 mm (nest needs <= 81)
- **what to check** — The near miss. The diagnostics say which clause failed -- distance just over 81 mm, `at_rest` never holding, or the can tipped. Is the failure the right call?

### `dH_s901_hold_ep0_pushed_no_contact.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 0 (uid 252)
- **class** `pushed_no_contact` — `pushed` fired (>= 10 mm goalward after release) but `contact_push` never did -- goalward travel with no far-side solver contact frame.
- **first fire** — picked d20, placed_v2 d27, released d27, pushed d28
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.271 m, gain 57.5 mm, lever 154.5 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 0
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Is this a real push the contact clause missed, or is the "gain" just the can settling / rolling as the gripper withdraws? If the latter, `pushed` is too cheap.

### `dH_s940_rnd_ep28_pushed_no_contact.mp4`

- **checkpoint** `dH_s940` — **start set** `rnd` (seed 0) — **episode** 28 (random start)
- **class** `pushed_no_contact` — `pushed` fired (>= 10 mm goalward after release) but `contact_push` never did -- goalward travel with no far-side solver contact frame.
- **first fire** — picked d30, placed_v2 d36, released d36, pushed d60, nested d63
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.117 m, gain 16.7 mm, lever 197.6 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 90.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Is this a real push the contact clause missed, or is the "gain" just the can settling / rolling as the gripper withdraws? If the latter, `pushed` is too cheap.

### `dH_s901_hold_ep5_pushed_no_contact.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 5 (uid 276)
- **class** `pushed_no_contact` — `pushed` fired (>= 10 mm goalward after release) but `contact_push` never did -- goalward travel with no far-side solver contact frame.
- **first fire** — picked d66, placed_v2 d73, released d73, pushed d74
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.136 m, gain 78.1 mm, lever 178.1 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Is this a real push the contact clause missed, or is the "gain" just the can settling / rolling as the gripper withdraws? If the latter, `pushed` is too cheap.

### `dH_s901_rnd_ep16_held_press_timeout.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 1) — **episode** 16 (random start)
- **class** `held_press_timeout` — `contact_push_legacy` fired without a release: the arm presses the HELD can into the goal and runs to the horizon.
- **first fire** — picked d16, contact_push_legacy d125
- **end** — truncated at decision 300 of 300; reward 1.0 of 8
- **settle verdict** — `nested_honest` 1 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.066 m, gain 0.0 mm, lever 100.2 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — The behaviour the release-first clause was written to stop paying. Confirm the can is still in the hand throughout (hand=1 on the diagnostics line).

### `dH_s901_hold_ep9_tipped_after_release.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 9 (uid 302)
- **class** `tipped_after_release` — The tip rule ended the episode after `placed_v2` had been granted.
- **first fire** — picked d15, placed_v2 d22, released d22, pushed d26
- **end** — tipped at decision 29 of 29; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.134 m, gain 17.3 mm, lever 130.2 mm, in_hand 0, at_rest 0, can tilt 68.7 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — The can was set down and then knocked over. Was the release genuine before the tip?

### `dH_s940_hold_ep13_tipped_after_release.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 1) — **episode** 13 (uid 331)
- **class** `tipped_after_release` — The tip rule ended the episode after `placed_v2` had been granted.
- **first fire** — picked d94, placed_v2 d108, released d108, contact_push d108, contact_push_legacy d107, nested d107
- **end** — tipped at decision 118 of 118; reward 4.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 1
- **diagnostics at the end** — dist 0.292 m, gain 0.0 mm, lever 187.2 mm, in_hand 0, at_rest 0, can tilt 90.0 deg, goal tilt 0.0 deg, in_band 0
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — The can was set down and then knocked over. Was the release genuine before the tip?

### `dH_s901_hold_ep2_tipped_before_release.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 0) — **episode** 2 (uid 256)
- **class** `tipped_before_release` — The tip rule ended the episode while the can had never been released.
- **first fire** — picked d16
- **end** — tipped at decision 94 of 94; reward 1.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.219 m, gain 0.0 mm, lever 173.7 mm, in_hand 0, at_rest 0, can tilt 90.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Did the can really fall over, and does it fall before any placement?

### `dH_s940_hold_ep1_tipped_before_release.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 0) — **episode** 1 (uid 254)
- **class** `tipped_before_release` — The tip rule ended the episode while the can had never been released.
- **first fire** — picked d107
- **end** — tipped at decision 134 of 134; reward 1.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.453 m, gain 0.0 mm, lever 52.1 mm, in_hand 0, at_rest 0, can tilt 89.8 deg, goal tilt 0.0 deg, in_band 0
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Did the can really fall over, and does it fall before any placement?

### `dH_s901_hold_ep4_placed_only.mp4`

- **checkpoint** `dH_s901` — **start set** `hold` (seed 1) — **episode** 4 (uid 273)
- **class** `placed_only` — `placed_v2` granted after a real pick, and nothing above it.
- **first fire** — picked d17, placed_v2 d28, released d28
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.161 m, gain 0.0 mm, lever 50.5 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Is the can genuinely set down, open-gripper, on the shelf, at the decision PLACE lights?

### `dH_s940_hold_ep7_placed_only.mp4`

- **checkpoint** `dH_s940` — **start set** `hold` (seed 0) — **episode** 7 (uid 294)
- **class** `placed_only` — `placed_v2` granted after a real pick, and nothing above it.
- **first fire** — picked d27, placed_v2 d59, released d59
- **end** — truncated at decision 300 of 300; reward 2.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.141 m, gain 0.0 mm, lever 100.4 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Is the can genuinely set down, open-gripper, on the shelf, at the decision PLACE lights?

### `dH_s901_rnd_ep13_reset_artifact.mp4`

- **checkpoint** `dH_s901` — **start set** `rnd` (seed 0) — **episode** 13 (random start)
- **class** `reset_artifact` — `placed_v2` granted with `picked` FALSE -- the start already put the can inside the shelf footprint, so the rung is true at reset with the arm at home (CONFOUNDS row 82).
- **first fire** — placed_v2 d10, released d10
- **end** — truncated at decision 300 of 300; reward 1.0 of 8
- **settle verdict** — `nested_honest` 0 (the post-episode settle, the reference `nested_v2` is checked against); legacy `nested_proxy` 0
- **diagnostics at the end** — dist 0.399 m, gain 0.0 mm, lever 331.3 mm, in_hand 0, at_rest 1, can tilt 0.0 deg, goal tilt 0.0 deg, in_band 1
- **`nested_v2`** sticky 0 / final-frame 0
- **what to check** — Confirm the arm never touches the can. This is a task-setup fact, not a predicate bug.

## Caveats

1. **Shared-process protocol.** Each (checkpoint, start set, seed) ran its episodes in ONE Genesis world, in order. Full-scope episodes are order-dependent, so these counts are a smoke test, not cells of record; the isolated protocol (`--ic-index`, one process per episode) costs 2.05x and is what a cell uses.
2. **Sampled actions.** The mode cells contain no slides at all; sampling is both the training-time statistic and the only setting that has ever produced one.
3. **`nested_v2` in the stage table is STICKY** (read off `env._granted`), so both it and the final-frame value are listed per clip. Lane 1 measured the sticky read at precision 0.571 against the settle on its 60 episodes and recommended reporting the final frame (`NESTED_V2_PREDICATE_2026-09-10` §5.4). **On these 315 episodes the sticky read scores 1.000**, and every sticky firing also has the final-frame value set — under this ladder `slide_success` is TERMINAL, so an episode ends at the first `nested_v2` frame and the sticky/final split that Lane 1 saw has no room to open. The disagreement is a property of the terminal rule, not of the predicate; do not carry the 0.571 into a staged-ladder table without re-measuring.
4. **An absent value is printed `n/a`, never 0** — a tracker that never ran a full-scope frame has no diagnostics, and that is different from a zero reading.
5. **The two start sets are not what their names suggest.** `hold` is 15 DEMONSTRATION starts and is NOT held out — 14 of the 15 are training starts (`REVIEW_GUIDE_2026-09-07` §8 item 7). `rnd` is the random box, out of distribution, and 4 of its 30 starts put the can INSIDE the shelf footprint, so `placed_v2` is true at reset with the arm at home (CONFOUNDS row 82); those episodes are labelled `reset_artifact` here rather than `placed_only`, and they are a task-setup fact, not a predicate bug.
6. **`dH_s901` was trained on the OLD ladder** (picked/contact/nested, 250k decisions) and is SCORED here under the unified `staged` one. Its sidecar records no ladder, so the evaluator falls back to `staged` and says so. It is included because it is the only checkpoint on record that places and pushes often enough to be positive-rich — not as a like-for-like arm against the two pilot checkpoints.

## Where everything lives

The 42 clips here are a SELECTION. Every one of the 315 episodes was rendered and all of them are kept (not committed — `*.mp4` is gitignored):

```
rollouts, all mp4s, per-cell metrics.json : /tmp/claude-1000/-home-j-workspace-genesis-pickaplace/14e7a131-c656-4890-a0e7-bcea8665917d/scratchpad/slide_smoke
  <ckpt>_<ic-set>_s<seed>/ep<N>_<uid|rnd>_<class>.mp4
  <ckpt>_<ic-set>_s<seed>/metrics.json  -- per_episode[] carries, for EVERY episode:
      grants{}    the first-fire DECISION of every reported stage
      end_diag{}  lever_m, dist_xy_m, in_hand, at_rest, goalward_gain_m, pushed,
                  released, can_tilt_deg, goal_tilt_deg, in_band, nested_v2_now/_ever
      end_reason, end_decision, klass, borderline, stages{}, ladder_provenance
logs                                     : /tmp/claude-1000/-home-j-workspace-genesis-pickaplace/14e7a131-c656-4890-a0e7-bcea8665917d/scratchpad/slide_smoke/logs/
checkpoints (fetched read-only)          : <scratchpad>/ckpt/{dH_s940,dH_s941_c040,dH_s901}
```

The scratchpad is session-local and not backed up. Anything that must survive should be copied out before the session ends.
