# Phase-annotated DEMONSTRATIONS — human and machine (Lane 5, 2026-09-11)

23 clips (11 human, 12 machine), 5 MB total. Every clip is a **demonstration tape re-executed
through the unified `FullTaskEnv(scope='full', ladder='staged')`** — the same code path a
training run uses, the same world (`gc_kp4_riser3_shelf6`), the same action semantics. Nothing
here is a policy rollout; these are the tapes the next batch will train on.

Built by `baselines/annotate_demos.py`. It computes **no predicate of its own**: every chip is
read from `info` / `env._granted` / `env.tracker`, so the video is evidence about the ladder,
not about the renderer. The re-execution was run twice independently (an 8-process census and a
6-process render) and the two agree on every stage flag, grant decision and reward for all 23
tapes — the annotation is reproducible on this machine.

```
census : baselines/annotate_demos.py census --in <set> --src-dir <src> --out <json> --procs 8
render : baselines/annotate_demos.py render --in <set> --src-dir <src> --out-dir <dir> --procs 6 --tapes ...
sets   : human   $W/demos_state_full/dHfull_all      (74 tapes)
         machine $W/demos_state_full/dDPfull_first   (72 tapes)
ladder : unified-2026-09-10 | staged | picked=1 placed_v2=1 contact_push=2 slide_success=4
         max_return=8 | terminal=slide_success+tipped | shaping=off
         full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7
         git=known-good-2026-08-27-812-g24979f3
```

## How to read the overlay

**LADDER row** (bright) — the rungs that pay under `staged`. A chip lights **on the frame its
stage is granted**, shows **YELLOW** for the 6 decisions after the grant (the "just fired" flash;
the code comment says cyan, but OpenCV's BGR order renders that tuple yellow), then turns
**GREEN** and stays green — green and yellow both mean "granted"; yellow only adds "within the
last 6 decisions". Grey outline = not granted yet. Each lit chip carries `d<N>`, the decision
it fired on. A chip never shows a grant that has not happened yet (no spoilers), and the timeline ticks
at the bottom likewise only appear once passed.

| chip | stage | pays | definition |
|---|---|---|---|
| `PICK` | `picked` | +1 | the env's hardened held-can flag |
| `PLACE` | `placed_v2` | +1 | grip cmd < 0.45 ∧ shelf footprint ∧ z-band ∧ tilt < 20°, sustained 10 frames |
| `PUSH` | `contact_push` | +2 | **`placed_v2` already granted** ∧ can↔goal contact ∧ tool on the far side ∧ gripper clear of the goal |
| `NEST2` | `nested_v2` | 0 | picked ∧ placed_v2 granted ∧ dist_xy ≤ 0.081 ∧ both upright ∧ z-band ∧ **not in hand** ∧ **at rest** |
| `SLIDE` | `slide_success` | +4 | released ∧ pushed ∧ nested_v2 — the only non-tip terminal |

**legacy row** (dim, behind a rule) — pays nothing, terminates nothing, kept so old rows stay
readable: `nestP` = the withdrawn `nested` proxy, `pushL` = `contact_push_legacy` (the (g)
geometry with **no** release requirement), `slidL` = `slide_success_legacy` (the (l) predicate
with the grip < 0.3 clause that (p) withdrew).

**Diagnostics line** — `d<i>/<N>`, `lever` = |tool_xy − can_xy| (m), `hand` = `in_hand`
(lever < 0.025), `rest` = `at_rest` (≤ 2 mm of can xy travel over 12 env frames), `gain` =
`goalward_gain_m`, `push` = `pushed` (gain ≥ 10 mm after release), `dist` = can-to-goal xy (m),
`grip` = the COMMANDED grip 0…1. **No predicate on this ladder reads `grip`** — it is shown
because it is the thing amendments (l)/(p)/(x) got wrong, and the user should be able to see it.

**Reward line** — `staged paid` is what the env actually paid, cumulative, out of 8.
`sparse would pay` is the counterfactual for `ladder='sparse'` (nested_v2 = 1, terminal) **on
this same trajectory**: 0 until the first `nested_v2` frame, 1.0 from it, with `(ENDS here)`
marking where a sparse episode would have stopped. It is a counterfactual, not a second run.

**Terminal card** (held ~1.5 s at the end) — end reason, every grant decision, the settled
`nested_honest` from one `end_of_episode()` settle, and the tape's **recorded** (old-ladder)
reward beside the **re-executed** one.

Frames: ≤ 400 per clip. The body is uniformly subsampled (the note says `subsampled k/N`) but
**every frame on which a stage is granted is force-included**, with its neighbours, so a
subsample can never hide the event the clip exists to show. Playback speed preserves wall-clock
(7.5 decisions/s), so "the can is at rest" looks like rest.

## Class counts over the FULL sets (not just the clips)

Re-executed under the unified ladder; `classify()` is ordered, first match wins.

| class | {human} 74 | {machine} 72 |
|---|---|---|
| `slide` (slide_success) | 13 | 14 |
| `nested_drop` (nested_v2, never `pushed`) | **1** | 2 |
| `push_no_nest` (contact_push, never nested) | 6 | 4 |
| `carry_in` (contact reached with no prior placed_v2) | 7 | 6 |
| `placed_only` | 16 | 18 |
| `picked_only` | 11 | 12 |
| `tipped` | 11 | 8 |
| `nopick` | 9 | 8 |
| tapes granting `placed_v2` | 42 | 44 |
| tapes granting `contact_push` | 10 | 14 |
| tapes granting `nested_v2` | 14 | 16 |
| settled `nested_honest` | 17 | 18 |
| Σ reward recorded (old ladder) → re-executed (staged) | 118 → **179** | 131 → **192** |
| Σ reward under `sparse` | 14 | 16 |

`nested_drop` exists only **once** in the human set, so the brief's "2 human nested-by-drop"
could not be met — the class has one member (299) and it is included. The machine set has two
(295, 300) and both are included.

## The clips

`set_<trial uid>_<class>.mp4`. The uid is the real trial the start came from (the segment tapes
carry no uid; it is resolved from the contract-v1 source tape's `ic_uid`, with the tape length
asserted to match). Grant columns are decision indices; `rec→new` is the tape's recorded
(old-ladder) reward → what the unified ladder paid on re-execution.

### {human demonstrations} — `dHfull_all`, 11 clips

| clip | class | decisions | grants (decision) | end | rec→new | what it should show / what you are checking |
|---|---|---|---|---|---|---|
| `human_232_slide.mp4` | slide | 255 | pick 70, place 121, pushed 194, **nest2 201, SLIDE 201** | slide_success @202 | 3→6 | The clean complete demonstration. **Does `NEST2` light only after the can is released and at rest?** Watch `hand` go 1→0 and `rest` go 0→1 before the chip lights. Note `grip 0.41` at the slide frame — the human re-closed the fingers to a fist and pushed; `PLACE` still counts it as released (threshold 0.45). **Also: `PUSH` never lights** — the can arrives 7.3 cm from the goal, inside the 8.1 cm `nested_v2` radius but with no solver contact, so the +4 rung pays without the +2 rung. |
| `human_233_slide.mp4` | slide | 260 | pick 69, place 174, **push 209**, pushed 207, nest2 212, **SLIDE 212** | slide_success @213 | 3→8 | The only shape that pays the whole ladder (8/8). `PUSH` lights 3 decisions before `NEST2`. **Check the ordering is the one the ladder claims:** release → tool behind the can → contact → arrival. |
| `human_242_carry_in.mp4` | carry_in | 144 | pick 74, contact 117, **pushL 117**, place 143, nestP 143 | stream_exhausted @144 | 7→2 | The legacy row firing where the ladder row correctly does not. At d117 the can is **still in the gripper** and pressed into the goal: `pushL` lights, `PUSH` stays dark. The recording then ends one decision after the release. **Check: is the can in the hand when `pushL` lights?** The old ladder paid 7 here; the new one pays 2. |
| `human_255_push_no_nest.mp4` | push_no_nest | 315 | pick 146, place 196, pushed 288, **push 314**, nestP 314 | stream_exhausted @315 | 7→4 | **Known miss #1** (NESTED_V2_PREDICATE §4.2). The settle says `nested_honest=1`; `nested_v2` says 0. `PUSH` lights on decision **314 of 315** — the recording ends while the can is still moving, so `rest` never reaches 1. **Check the terminal card: is the can still drifting when the clip stops?** If so the predicate is right and the tape is short. |
| `human_262_tipped.mp4` | tipped | 228 | pick 96, place 141 | **tipped** @228 | 1→2 | A real release (`PLACE` lights at d141) followed by a tip. **Check the tip rule fires on a genuinely fallen can**, and that `PLACE` stays lit afterwards (grants are sticky). |
| `human_277_placed_only.mp4` | placed_only | 240 | pick 70, place 127, pushed 190 | stream_exhausted @240 | 1→2 | `placed_v2` only, but `push` goes to 1 at d190 — the can moved ≥ 10 mm goalward after release without ever touching the goal. **Check `push=1` in the diagnostics while `PUSH` stays dark**: `pushed` is displacement, `contact_push` needs contact. They are different things and this clip separates them. |
| `human_278_nopick.mp4` | nopick | 201 | — | stream_exhausted @201 | 0→0 | A demonstration that never picks. Every chip stays dark, reward 0. **The negative control:** if any chip lights here, a predicate is firing on nothing. |
| `human_299_nested_drop.mp4` | nested_drop | 600 | pick 442, contact 529, pushL 529, place 558, **push 558, nest2 561** | truncated @600 | 3→4 | The **only** human nesting that is not a slide. `push` stays 0 (no 10 mm of goalward gain after release) so `SLIDE` never lights even though `NEST2` does. **Check: did the can arrive by being set down, or by being pushed?** Under `sparse` this tape would pay its single +1 at d561 and end. |
| `human_305_push_no_nest.mp4` | push_no_nest | 289 | pick 102, place 171, pushed 281, **push 288**, nestP 288 | stream_exhausted @289 | 7→4 | **Known miss #2.** Same shape as 255: `PUSH` at decision 288 of 289, `nested_honest=1`, `nested_v2=0`. **Same question: is the can still moving at the cut?** |
| `human_308_carry_in.mp4` | carry_in | 364 | pick 93, contact 166, **pushL 166** | stream_exhausted @364 | 3→1 | **Known miss #3, and it does NOT reproduce on this lineage.** Lane 1 (census tapes) has 308 as a settle-nest blocked by `placed_v2`. Here `placed_v2` is indeed never granted — but `nested_honest` is **0**, so on `dHfull_all` 308 is not a nest at all. **Check by eye which account the video supports:** does the human ever let go? |
| `human_328_carry_in.mp4` | carry_in | 600 | pick 405, contact 540, **pushL 540** | truncated @600 | 3→1 | A long carry-in: the can is driven into the goal still held, `pushL` lights, `PUSH` and `PLACE` never do. **Check that the gripper is closed the whole time.** Old ladder 3, new ladder 1. |

### {machine demonstrations} — `dDPfull_first`, 12 clips

Machine tapes mostly run to the 601-frame cap, so most end `truncated @600`.

| clip | class | decisions | grants (decision) | end | rec→new | what it should show / what you are checking |
|---|---|---|---|---|---|---|
| `machine_233_slide.mp4` | slide | 356 | pick 86, place 173, pushed 206, **push 211**, nest2 216, **SLIDE 216** | slide_success @217 | 7→8 | The machine's full-ladder demonstration — **reaches the goal**. Same ordering check as human 233. |
| `machine_243_slide.mp4` | slide | 600 | pick 77, place 154, pushed 197, **nest2 211, SLIDE 211** | slide_success @212 | 1→6 | The second **reaches the goal** clip, and the machine analogue of human 232: arrives inside 8.1 cm with **no** `contact_push`, so 6/8 not 8/8. |
| `machine_277_push_no_nest.mp4` | push_no_nest | 401 | pick 229, place 303, pushed 390, **push 400**, nestP 400 | stream_exhausted @401 | 7→4 | The machine copy of the 255/305 shape: `PUSH` at decision **400 of 401**, `nested_honest=1`, `nested_v2=0`. **Check whether the arm is mid-push when the tape ends.** |
| `machine_280_placed_only.mp4` | placed_only | 600 | pick 57, place 116 | truncated @600 | 1→2 | The machine set's dominant class (18/72): release on the shelf, then 484 decisions of nothing. **Check `PLACE` lights on a genuine set-down**, and watch how long the tape runs after it. |
| `machine_283_carry_in.mp4` | carry_in | 600 | pick 309, contact 568, **pushL 568** | truncated @600 | 3→1 | **Presses the held can.** The gripper never opens (`PLACE` never lights) and the can is driven into the goal still clamped. **AND `nested_honest` = 1** — the settled reference nests a can that is in the hand, because `_nested()` has no release clause. `nested_v2` rejects it on `not in_hand`. **Check the terminal card: `lever 0.014 hand=1 grip 0.63 dist 0.065` beside `nested_honest (settled) 1`.** This is the one tape in either set where that happens. |
| `machine_286_nopick.mp4` | nopick | 600 | — | truncated @600 | 0→0 | 600 decisions, nothing granted. The machine negative control. |
| `machine_295_nested_drop.mp4` | nested_drop | 481 | pick 304, contact 404, pushL 404, **place 446, nest2 446**, nestP 480 | stream_exhausted @481 | 3→2 | **`place` and `nest2` fire on the SAME decision.** `placed_v2` needs 10 sustained frames, so by the time it grants the can has already been sitting still beside the goal and `nested_v2` is true immediately. `push` = 0 — a pure drop. **Under `sparse` this pays its only rung the instant the release completes, for a trajectory with no push at all.** Check whether that is the behaviour the sparse arm is meant to reward. |
| `machine_298_push_no_nest.mp4` | push_no_nest | 189 | pick 47, place 108, pushed 136, **push 145**, nestP 188 | stream_exhausted @189 | 7→4 | The **proxy disagreement**: `nestP` lights (old ladder paid 7) but `nested_v2` and `nested_honest` are both 0. **Check by eye whether the can is nested at the end.** If it is not, the proxy is what it is claimed to be. |
| `machine_300_nested_drop.mp4` | nested_drop | 534 | pick 375, **place 485, nest2 490**, nestP 533 | stream_exhausted @534 | 7→2 | Second drop-route nesting, 5 decisions between release and arrival, `push` = 0. Old ladder paid 7, unified pays 2, sparse would pay 1 at d490. |
| `machine_301_carry_in.mp4` | carry_in | 600 | pick 102, contact 176, **pushL 176** | truncated @600 | 1→1 | **Presses the held can**, second example, for 424 decisions after contact. **Check the gripper never opens** — this is the behaviour the release gate on `contact_push` exists to stop paying. |
| `machine_308_tipped.mp4` | tipped | 229 | pick 68, place 153, pushed 213, **push 221** | **tipped** @227 | 3→4 | A real release, a real push (+2 paid at d221), then the can tips 6 decisions later. **Check the tip is genuine** and that the +2 was earned before it. |
| `machine_331_carry_in.mp4` | carry_in | 281 | pick 153, contact 198, **pushL 198**, place 259, nestP 280 | stream_exhausted @281 | 7→2 | Contact reached **before** any release (d198 vs d259), then a late `placed_v2`, then the proxy fires on the last decision. Old ladder 7, unified 2. **Check the order of events against the chips.** |

## What looked wrong to me on the video (report, not interpretation)

1. **`nested_honest` — the column `nested_v2` is being validated against — nests a can that is
   still in the gripper.** `machine_283`, terminal card, last frame (d600): `lever 0.014
   hand=1 rest=1 dist 0.065 grip 0.63`, `placed_v2 0`, `nested_v2 0` — and
   `nested_honest (settled) 1`. The can is 1.4 cm from the tool point with the fingers
   commanded to 0.63; it is being held, and the settle says it is nested.
   `GenesisCanEnv._nested()` is `picked ∧ proximity ∧ both upright`; it has no release and no
   not-in-hand clause, so a carried can parked next to the goal passes. One tape in 146, and
   `nested_v2` correctly rejects it — but it means the reference is not a clean gold standard
   in the carry-in class, and it fails in the direction that flatters carrying.
2. **The ladder is not monotone: `slide_success` (+4) pays without `contact_push` (+2) on 10 of
   13 human slides and 6 of 14 machine slides.** `nested_v2`'s radius is 0.081 m while two cans
   touch at 0.066 m, so a can can "arrive" 1.5 cm clear of the goal with no solver contact. A
   policy can therefore collect the top rung having never satisfied the rung below it.
3. **Under `sparse`, `nested_v2` can be granted on the same decision `placed_v2` is** (machine
   295, and 5 decisions later in machine 300), because `placed_v2`'s 10-frame sustain means the
   can has already been at rest when the release is confirmed. In both of those tapes `pushed`
   is False. The sparse arm's single rung is reachable by a pure set-down at the moment of
   release — the P8 route question is already answerable on the demonstrations.
4. **`contact_push` fires on the last decision of 4 tapes** (human 255 @314/315 and 305
   @288/289, machine 277 @400/401 and 257 @222/224). These are recordings that stop mid-push.
   This is Lane 1's `at_rest` diagnosis reproduced on a different tape lineage; it is a property
   of the recordings, not of the predicate.
5. **`placed_v2` at grip < 0.45 admits a fist.** Human 232 completes its slide with the
   commanded grip at 0.41 on the final frame (d202, one decision after `nested_v2`) — the
   fingers are part-closed while the ladder treats the can as released. It is the behaviour
   amendment (p) deliberately allows; it is worth the user confirming it reads as "released"
   on the video.
6. **`dHfull_all` is not the census lineage, and 308 differs.** On these tapes 308 has
   `nested_honest = 0`, not 1. The two sets are different recordings of the same starts
   (SLIDE_CLAUSE5_LINEAGE §7: actions differ on 24 of 74). Everything in this index is measured
   on `dHfull_all` / `dDPfull_first` — the sets the pilot will actually train on.

## Not verified here

- The re-execution was run only on this machine (32-core). The project's core-count sensitivity
  (CONFOUNDS; 36-core boxes diverge) is untested for these numbers; two independent passes on
  THIS box agree exactly, which is reproducibility, not portability.
- `slide_success_legacy` (`slidL`) never lit on any of the 23 clips. That is consistent with the
  (l) grip < 0.3 clause passing 2 of 74 human tapes, but 23 clips is not a test of it.
- No clip here is a policy rollout. The brief's optional {RLPD} checkpoint episodes were not
  rendered.
