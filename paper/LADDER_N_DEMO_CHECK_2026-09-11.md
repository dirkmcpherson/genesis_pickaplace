# Ladder N on the demonstrations — calibrating the slide (Lane 7, 2026-09-11)

**The question.** The user asked for a reward that requires the behaviour the paper is about:
*"nest such that max reward (or sparse reward) comes from release, moving the gripper to the
opposite side of the can, and sliding it towards the goal can"*, then *"a version of sparse
that enforces the slide"*, then — the instruction that decides everything below —
**"characterize this so that as many human demos do it as possible; the important part is that
they put the can down, move to the opposite side of the can as the goal, and slide it."**

So the constants of the slide predicate are a **measurement on the 74 human tapes**, not a
choice. This document is that measurement.

**Headline.** At the calibrated constants the paid rung `home` fires on **13 of 13** human
sim-slide tapes and **14 of 14** machine sim-slide tapes, and on **no other tape in either
set** — 100 % recall and 100 % precision against Lane 5's independent behaviour
classification. The brief's proposed constants scored 5 of 13. Two of the brief's three
proposed constants had to move, and the third (`far_release`) turns out to be **unnecessary
and arm-asymmetric**; the evidence says leave it off.

**Scope.** Predicates, ladders, the scoring path, the demonstration-side measurement. No
cluster job was submitted, no cluster tree was touched, the local GPU was not used.

Code: genesis_pickaplace `ladder-unify-2026-09-11` (this commit), r2dreamer `f8743e0`+.

---

## 0. What is defined

```
slide_event  :=  released                      # the can was put DOWN on the shelf
                 ∧ settled_after_release       # ...and came to rest: the set-down finished
                 ∧ farside                     # the tool went to the OPPOSITE side of it
                 ∧ slide_gain_m ≥ 0.01         # ...and the can TRAVELLED goalward from there
home         :=  slide_event ∧ nested_v2       # and it arrived, settled and upright
```

| constant | brief | **calibrated** | what moved it |
|---|---|---|---|
| `FARSIDE_REACH_M` | 0.08 m | **0.10 m** | at 0.08 only 5 of 13 human slides are seen; at 0.10, 13 of 13 |
| `FARSIDE_CONE_DEG` | 90° (`dot < 0`) | **60°** | same 13 slides, one fewer non-slide tape — the tighter cone is free |
| `SLIDE_GAIN_MIN_M` | 0.01 m | **0.01 m** | unchanged; 0.005 adds a non-slide, 0.02 loses a slide, 0.03 loses four |
| set-down latch | not in the brief | **on** | Lane 9: the old `pushed` was mostly the release transient |
| `FAR_RELEASE_DIST_M` | 0.10 m, ON | **0.08 m, and OFF** | the clause buys nothing and costs 2 machine slides (§4) |

`farside` is `released ∧ ¬in_hand ∧ (angle of the tool off the away-from-goal ray ≤ cone) ∧
|tool_xy − can_xy| ≤ reach`. **No solver-contact term**, deliberately: 10 of the 13 human
sim-slides never make a can↔goal solver contact, so a contact clause would score the world
rather than the human. That is the one clause separating it from the (g) `contact_push`.

`slide_gain_m` is metres of goalward progress credited only on **new minima** of
dist_xy(can, goal) since the release, only on frames where the farside condition holds *that
frame*, and only after the set-down latch. The running minimum tracks **every** frame after
the release, so progress made by CARRYING the can lowers the bar without paying. It is
therefore monotone, which is what lets the `nested_ramp` ramp be paid incrementally with no
clawback and makes oscillation unprofitable.

The two ladders:

| rung | `nested_sparse` | `nested_ramp` | requires |
|---|---|---|---|
| `picked` | — | +1 | — |
| `placed_v2` | — | +1 | `picked` |
| `farside` | — | +1 | `placed_v2` |
| slide ramp | — | +2 × min(1, `slide_gain_m`/0.10), incremental | `farside` |
| `home` | **+1**, terminal | **+4**, terminal | `slide_event` |
| max return / r2dreamer `return_clamp` | 1 | 9 | |

`slide_event` is **logged under every ladder and paid by none**, so the paper can count the
slides that did not arrive as well as the ones that did. `tipped` terminates with its existing
(zero) penalty; nothing else terminates.

---

## 1. Method: record once, score many

The user asked whether this re-copies the demonstrations for every variant. It does not.

A reward column does not steer the arm, so the only things a ladder changes about a
demonstration are **where the episode stops** and **what it pays**. Each tape is re-executed
through `FullTaskEnv(scope='full')` **once**, with termination suppressed
(`env.never_terminate`, an instance knob nothing else sets), writing a **per-env-frame stage
record**: poses, tool point, commanded grip, solver contacts, the env-owned stage flags — every
input `StageTracker` and the reward loop consume — plus one end-of-episode settle
(`nested_honest`). Every candidate ladder is then applied **offline**, in pure numpy, with no
Genesis and no simulation.

```
# record, once per set  (both sets: 16 MB, 146 tapes, 264,220 env frames)
python baselines/rl/relabel_reward.py --in $D/demos_state_full/<set> \
       --records-out $D/stage_records/<set> --procs 8
# score, once per ladder × far_release
python baselines/rl/relabel_reward.py --in $D/demos_state_full/<set> \
       --out $D/relabel_dev/<set>_rnr --from-records $D/stage_records/<set> \
       --ladder nested_ramp [--far-release]
# and the calibration itself needs no relabel at all
python baselines/diagnostics/ladder_n_slide_calibration.py --records $D/stage_records \
       --census <lane5>/can_pos_recovery/videos_ladder_2026-09-11
```

Two things make this safe rather than merely convenient.

**One reward loop, two call sites.** The loop moved out of `_step_once` into
`full_env.LadderAccountant`. The env calls it once per frame; the offline scorer calls the same
object. They cannot drift, because there is nothing to drift from.

**It is verified, not assumed.** `--verify-against` scores a direct re-execution and an offline
replay of the same tapes and asserts the reward columns **and** the terminal decisions are
equal element for element.

| check | result |
|---|---|
| `--verify-against`, human, `staged`, 2 tapes | 2/2 identical (reward column and terminal decision) |
| `--verify-against`, human, `nested_ramp --far-release`, 3 tapes | 3/3 identical |
| offline `staged` vs **Lane 5's independent direct re-execution**, all 74 human tapes | Σ 179.0 = 179.0, and every rung count, every end reason identical |
| offline `sparse` vs the same census, all 74 | Σ 14.0 = 14.0 |

The third row is the strongest of these: Lane 5 built their census by direct re-execution with
their own script, and the offline path reproduces it tape for tape.

**Wall clock, measured on this box.**

| path | cost |
|---|---|
| record one set (74 tapes / 116,884 frames, 8 processes) | 431 s wall (~46 s CPU per tape) |
| record the other (72 / 147,336) | 535 s wall |
| direct re-execution, per tape | 12–21 s |
| **offline replay, per tape** | **1.2–2.3 s (≈ 10× faster than direct)** |
| offline replay + rewriting a whole relabelled set | ~4–5 s per tape (the npz re-write dominates) |
| **the 160-cell calibration grid over all 146 tapes** | **~13 min, one pass** |

Scoring the four ladder variants by re-execution would have cost ≈ 7.5 CPU-hours of
simulation. Record-once cost 1.9 CPU-hours and every additional variant costs minutes — which
is what made a 160-cell calibration affordable at all.

---

## 2. The calibration

Full grid: `baselines/diagnostics/ladder_n_slide_calibration.py`. Cells below are
**human sim-slides admitted / human tapes from the non-slide classes admitted**
(`carry_in` + `nested_drop` + `placed_only`, 24 human tapes), at cone 60°, set-down latch on,
no release clause. 13 sim-slides is the ceiling.

| reach \ gain | ≥ 0.5 cm | ≥ 1 cm | ≥ 2 cm | ≥ 3 cm | ≥ 5 cm |
|---|---|---|---|---|---|
| 6 cm | 5/4 | 3/3 | 3/1 | 3/1 | 3/0 |
| 8 cm  *(the brief)* | 5/7 | **5**/5 | 5/3 | 5/2 | 4/0 |
| **10 cm** | 13/9 | **13/8** | 12/6 | 9/4 | 5/1 |
| 12 cm | 13/9 | 13/8 | 12/6 | 9/4 | 5/1 |

**The reach is the binding constant, and the brief's value was wrong by a factor that costs
8 of 13 slides.** The reason is geometric: the tool point sits about 5 cm behind whatever
surface actually contacts the can. On the first two slide tapes examined frame by frame,
**100 % of the goalward motion after the release happens with the tool 8.0–9.4 cm from the can
centre** — just outside the proposed 0.08 m. "Can radius plus a finger" was the wrong model of
where the tool point is during a push; Lane 1's independently measured fist-contact
distribution runs to 11.17 cm (p99), which 0.10 m sits inside.

10 cm and 12 cm are equivalent on slides, so the **tighter** value is taken. The 60° cone and
the 1 cm gain are each free at the knee: 90° admits one more non-slide, 0.5 cm admits one more,
2 cm costs a slide and 3 cm costs four.

### The set-down latch is free on demonstrations and load-bearing on policies

Lane 9 measured, on 315 sampled {RLPD} rollouts, that the old `pushed` predicate was mostly the
**release transient**: 20 of 23 `slide_success` positives fired it within 3 decisions of the
release, 5 inside the release decision itself, with the can rolling up to 34 mm out of the
*opening* hand while the tool lever *grew*. So "put the can down" became a clause: no goalward
millimetre counts until the can has been at rest once since the release.

On the demonstrations it costs **nothing**:

| arm | slides latched | unlatched | non-slide admitted latched | unlatched | credit the latch excluded (p50 / p90 / max) |
|---|---|---|---|---|---|
| {human demonstrations} | 13 | 13 | 8 | 8 | 0.00 / 0.00 / 0.00 mm |
| {machine demonstrations} | 14 | 14 | — | — | 0.00 / 0.00 / 0.00 mm |

Zero excluded credit, because on a human tape the hand is still at the can during the
transient, so the geometry clause already excludes it. **The latch removes an artefact that
policies exploit and a behaviour that humans do not have.** `gain_during_release_m` reports
what it excluded on every episode, so the exclusion is never silent.

---

## 3. What the calibrated predicate pays — the acceptance test

At reach 0.10 m, cone 60°, gain ≥ 1 cm, latch on, **no release clause**:

| arm | tapes | `slide_event` | of which `slide` / `push_no_nest` / other | **`home`** | `home` by class | recall on sim-slides | precision |
|---|---|---|---|---|---|---|---|
| {human demonstrations} | 74 | 26 | 13 / 5 / 8 `placed_only` | **13** | slide 13 | **13/13** | **13/13** |
| {machine demonstrations} | 72 | 24 | 14 / 4 / 3 `placed_only`, 1 `carry_in`, 2 `tipped` | **14** | slide 14 | **14/14** | **14/14** |

human `home` uids: 232, 233, 237, 247, 251, 256, 259, 273, 275, 302, 304, 316, 317
machine `home` uids: 233, 242, 243, 251, 255, 259, 262, 263, 265, 281, 302, 305, 311, 321

**The acceptance test the brief set is met.** Every human tape that completes a slide in
simulation earns the top rung, and no tape that does not, does. Under `nested_ramp` those 13
human tapes earn the full ladder; under `nested_sparse` they are the 13 points the set carries.

`slide_event` is deliberately looser than `home`, and its extra firings are informative rather
than wrong: 5 human `push_no_nest` tapes are slides that did not arrive (exactly what the flag
is for), and 8 `placed_only` tapes pushed ≥ 1 cm from the far side without arriving. Lane 5's
`placed_only` / `push_no_nest` boundary was drawn with the `pushed` predicate that Lane 9 has
since shown to be a transient detector, so those 8 are **not** established false positives —
they are the class a human should check on video. None of them reaches `home`, so none is paid.

### Per-clause failure, on every sim-slide tape

With **no release clause**, no human or machine sim-slide fails any clause — the table is
empty, which is the result. With the brief's release clause ON (§4) exactly two machine tapes
fail, both on `release_too_close`:

| arm | uid | release (cm) | farside frames | slide_gain (cm) | `nested_v2` frames | failing clause |
|---|---|---|---|---|---|---|
| {machine demonstrations} | 255 | 7.9 | 0 | 0.00 | 1449 | `release_too_close` |
| {machine demonstrations} | 302 | 7.8 | 0 | 0.00 | 235 | `release_too_close` |

Human sim-slide margins, for reference — every one clears every clause with room:

| uid | release (cm) | farside frames | slide_gain (cm) | `nested_v2` frames |
|---|---|---|---|---|
| 232 | 10.1 | 259 | 3.66 | 175 |
| 233 | 8.8 | 242 | 2.12 | 191 |
| 237 | 8.4 | 231 | 1.94 | 163 |
| 247 | 13.8 | 223 | 6.35 | 131 |
| 251 | 10.4 | 258 | 2.76 | 234 |
| 256 | 10.8 | 364 | 3.92 | 262 |
| 259 | 13.9 | 504 | 6.36 | 3 |
| 273 | 14.9 | 668 | 5.80 | 355 |
| 275 | 13.9 | 508 | 6.22 | 220 |
| 302 | 8.5 | 295 | 2.42 | 155 |
| 304 | 11.2 | 1093 | 3.44 | 203 |
| 316 | 13.0 | 744 | 4.93 | 125 |
| 317 | 12.1 | 698 | 5.62 | 303 |

The thinnest margin is uid 237 at 1.94 cm of gain (the threshold is 1 cm) and uid 259 with only
3 frames of `nested_v2` — the can arrives and the recording ends almost immediately.

---

## 4. The release clause should be OFF — measured, against the prior

The coordinator's instruction was that the user's intent is best served with `far_release=True`
(the release that counts must be ≥ 0.10 m from the goal, so a set-down-and-nudge cannot pay).
**The data does not support the clause at any threshold, for two independent reasons.**

**(1) Release distance does not separate a slide from a set-down.** It was assumed to; it does
the opposite of what the assumption needs.

| arm | class | tapes | release p50 (cm) | ≥ 8 cm |
|---|---|---|---|---|
| {human demonstrations} | `placed_only` (a non-slide class) | 16 | **13.1** | 16/16 |
| {human demonstrations} | `slide` | 13 | **11.2** | 13/13 |
| {human demonstrations} | `push_no_nest` | 6 | 11.9 | 6/6 |
| {human demonstrations} | `carry_in` | 7 (3 release) | 7.9 | 1/3 |
| {human demonstrations} | `nested_drop` | 1 | 6.6 | 0/1 |
| {machine demonstrations} | `placed_only` | 18 | 12.9 | 18/18 |
| {machine demonstrations} | `slide` | 14 | 11.2 | 12/14 |
| {machine demonstrations} | `nested_drop` | 2 | 6.7 | 0/2 |

Humans **set the can down FARTHER from the goal when they are not going to slide it**
(13.1 cm) than when they are (11.2 cm). A release-distance threshold therefore cannot exclude
`placed_only`; the only classes it excludes are the drop classes (`nested_drop` at 6.6 cm,
`carry_in` at 7.9 cm).

**(2) The drop classes are already excluded, by the slide clauses themselves.** With the clause
OFF, `home` fires on 13 human and 14 machine tapes and **every one is class `slide`** — no
`nested_drop`, no `carry_in`, no `placed_only`. A drop-and-nudge at the goal does not produce
1 cm of far-side goalward travel *after the can has come to rest*, which is precisely what the
set-down latch plus the farside geometry ask for. The release clause is redundant.

**And it is not free.** Switching it on costs:

| arm | `home` with the clause OFF | ON (≥ 8 cm) | lost |
|---|---|---|---|
| {human demonstrations} | 13 | 13 | 0 |
| {machine demonstrations} | 14 | 12 | 2 (uids 255, 302) |

**The clause removes 2 machine slides and 0 human slides.** Since the machine-vs-human
comparison is the paper, a clause that trims one arm and not the other is a confound
introduced by the reward definition — for no measured benefit. At the brief's 0.10 m it is
worse still: 3 human slides are lost as well.

**Recommendation: `far_release` stays OFF (the constructor default).** It remains implemented,
measured on every episode as `release_far`, and available as a switch; `FAR_RELEASE_DIST_M` is
set to 0.08 m so that if it is ever turned on it costs the human arm nothing. Do not turn it on
without re-checking the arm asymmetry above.

---

## 5. What the ladders pay both sets

All four variants, scored offline from the same stage records, at the calibrated constants.
`staged` and `sparse` are included to show the pilot's ladders are unmoved.

| arm | ladder | far_release | Σ reward | picked | placed_v2 | farside | slide_event | **home** | nested_v2 |
|---|---|---|---|---|---|---|---|---|---|
| {human demonstrations} (74) | `staged` | off | 179.0 | 65 | 42 | 39 | 25 | 12 | 14 |
| {human demonstrations} | `sparse` | off | 14.0 | 65 | 42 | 39 | 25 | 12 | 14 |
| {human demonstrations} | **`nested_sparse`** | off | **13.0** | 65 | 42 | 39 | **26** | **13** | 14 |
| {human demonstrations} | `nested_sparse` | on | 13.0 | 65 | 42 | 37 | 26 | 13 | 14 |
| {human demonstrations} | **`nested_ramp`** | off | **219.2** | 65 | 42 | 39 | **26** | **13** | 14 |
| {human demonstrations} | `nested_ramp` | on | 217.2 | 65 | 42 | 37 | 26 | 13 | 14 |
| {machine demonstrations} (72) | `staged` | off | 192.0 | 64 | 44 | — | — | — | 16 |
| {machine demonstrations} | `sparse` | off | 16.0 | 64 | 44 | — | — | — | 16 |
| {machine demonstrations} | **`nested_sparse`** | off | **14.0** | 64 | 44 | — | **24** | **14** | 16 |
| {machine demonstrations} | **`nested_ramp`** | off | (manifest) | 64 | 44 | — | **24** | **14** | 16 |

The `staged` / `sparse` Σ and their `picked` / `placed_v2` / `nested_v2` counts reproduce Lane
5's independent census tape for tape. Note that their `slide_event` and `home` columns read one
LOWER (25/12 rather than 26/13): those ladders **terminate earlier** — `staged` on
`slide_success` and `sparse` on `nested_v2` — so one tape's episode ends before its slide
finishes. That is not a disagreement about the predicate; it is the correct observation that a
logged flag is only observable up to the ladder's own terminal, and it is the reason `home`
must be read from a run of the ladder that pays it. Rows marked "—" are in the manifests.

**The statistic this buys the paper.** The pilot's `sparse` ladder pays `nested_v2`: 14 human
tapes and **16** machine tapes. `nested_sparse` pays `home`: 13 human and 14 machine. The
difference is the **drop route** — 1 human and 2 machine tapes arrive without sliding. The
machine set contains more drop-route arrivals than the human set, and the nested ladder is what
makes that visible as a reward difference rather than an annotation.

`slide_gain_m` over the human set: p50 0.0 cm (most tapes never slide), p90 4.7 cm, max
10.1 cm. **The ramp's 0.10 m span is therefore never saturated by a human demonstration** —
a typical human slide of 3–6 cm earns 0.6–1.2 of the 2 available. If the ramp is meant to pay a
human slide near its maximum, the span should be **0.05 m**, not 0.10 m; at 0.05 m a 3 cm slide
earns 1.2 and a 5 cm slide saturates. Recorded as an open decision, not changed unilaterally —
the brief fixed the span, and unlike the reach it is not falsified by the coverage test.

---

## 6. Where the specification had to change, and why

1. **`FARSIDE_REACH_M` 0.08 → 0.10 m.** The brief's value sees 5 of 13 human slides. §2.
2. **`FARSIDE_CONE_DEG` 90° → 60°.** Same coverage, one fewer non-slide tape. Free.
3. **The set-down latch is new.** Lane 9's evidence; free on demonstrations. §2.
4. **`far_release` OFF, `FAR_RELEASE_DIST_M` 0.10 → 0.08 m.** Redundant and arm-asymmetric. §4.
5. **`home` is now `slide_event ∧ nested_v2`,** with `slide_event` a named, logged, sticky flag
   — the user's three clauses in one place, countable without arrival.
6. **`released` now requires `picked`.** 4 of the 30 `rnd30` evaluation starts satisfy
   `placed_v2` AT RESET with the arm at home (CONFOUNDS row 82), and `released` used to latch
   on frame 0 there. A real behaviour change, reachable in reverse via
   `released_requires_picked=False`, and identical frame-for-frame on any history where the
   pick precedes the release — every real demonstration.
7. **A drop earns `farside`, which the brief did not anticipate.** A gripper withdrawing
   straight back from a set-down passes through the far-side band, so under `nested_ramp` a
   drop at the goal earns picked + placed_v2 + farside = 3 of 9, not the 2 the brief predicted.
   No clause in the brief's definition excludes a withdrawal and inventing one in a test would
   be a new predicate. What carries the contrast is the pair above it: the ramp needs goalward
   motion made from that pose after the set-down, and `home` needs that plus arrival. A drop
   earns **0** of both, under both variants. (For reference the pilot's `staged` ladder pays
   the same drop 4 of its 8.) A withdrawal to the SIDE does not grant `farside` at all, which
   is the control that the clause is geometry and not a rubber stamp.

---

## 7. Caveats

**Hardware — the one that decides whether a number here is citable.** These records were made
on this development box (pop-os, AMD Ryzen 9 5950X, 32 cores, AVX2), not the cluster's 64-core
class. Genesis is not bit-identical across CPU classes: the same human set re-executed on the
cluster paid Σ 171 under `staged` where this box pays Σ 179. **Records of record must be made
on the 64-core class.** These are development artefacts — adequate for choosing a ladder on the
demonstration side, not numbers for the paper. Every record and manifest stamps its node, core
count and ISA. Regenerating costs one recording pass (~9 min wall on 8 cores per set); every
variant after that is seconds.

**Re-execution fidelity.** The re-execution diverges from the original recording by more than
1 cm of can position on 36 of 74 human and 38 of 72 machine tapes (p50 9.1 / 12.9 mm). This is
the known non-determinism of this tape lineage (SLIDE_CLAUSE5_LINEAGE §7), it affects Lane 5's
census identically, and it is the reason the hardware caveat above is not a formality.

**The behaviour classes are a weak control.** `slide` / `placed_only` / `push_no_nest` come
from Lane 5's census, which was built with the `pushed` predicate that Lane 9 has since shown
to be mostly a release-transient detector. The `slide` class itself is robust (it is the staged
ladder's top rung, which needs arrival), so the **recall** figure is sound. The `placed_only`
vs `push_no_nest` boundary is not, so the 8 human tapes counted as "false positives" of
`slide_event` should be read as "unadjudicated", and checked on video before anyone calls them
errors. `home` admits none of them either way.

**The policy-side check could not be run.** Lane 9's 315 rollouts are stored as per-episode
outcomes, ICs and videos (`slide_smoke/*/metrics.json`) — **no per-frame trajectories**, so
their 23 positives cannot be re-scored under the calibrated predicate without re-running the
rollouts. Their ICs and the checkpoints are both available, so the re-run is possible but was
not attempted here. **Recommendation: give `eval_e2e.py` the same `--records-out` stage
recording this script has**, and the question becomes a 2-second offline re-score for every
future evaluation instead of a re-simulation.

**A demonstration is not a policy.** These numbers say what a ladder *would have paid* a
demonstration, which is what the demo buffer will carry. They say nothing about whether a
learner can reach those rungs online. That is the pilot's question.

**An absent value is not a zero.** Every count is over all 74 / 72 tapes, all of which were
scored; per-tape rows are in each manifest under `per_tape`.

---

## 8. Draft amendment (aa), for registration BEFORE any Ladder-N job

### (aa) Ladder N — the nested ladders

**Motivation (user, 2026-09-11).** Both ladders of the pilot pay their top rung for an outcome
that a DROP at the goal satisfies; neither requires the behaviour the paper is about. Ladder N
pays the behaviour: *the can is put down, the gripper moves to the opposite side of it from the
goal, and the can is slid home.*

**Definitions** are `baselines/stage_predicates.py` at the commit stamped in every run's
`ladder_provenance.json`; `slide_event` and `home` are as in §0 above, with the constants
**calibrated on the 74 human tapes** (§2), not chosen:
`FARSIDE_REACH_M` 0.10, `FARSIDE_CONE_DEG` 60, `SLIDE_GAIN_MIN_M` 0.01, set-down latch ON,
`far_release` **OFF** (`FAR_RELEASE_DIST_M` 0.08 if ever enabled).

**The two variants**, one chosen by the pilot's sparse arm:

| ladder | pays | terminal | max return / r2d clamp |
|---|---|---|---|
| `nested_sparse` | `home` 1.0 | `home` | 1 / 1.0 |
| `nested_ramp` | picked 1 → placed_v2 1 → farside 1 → ramp 2 → `home` 4 | `home` | 9 / 9.0 |

**P-aa-1 (demonstration side; MEASURED, recorded so the training numbers read against it).**
Under the calibrated constants `home` fires on **13/74 human** and **14/72 machine** tapes,
which are exactly the 13 and 14 tapes that complete a slide in simulation — 100 % recall and
100 % precision against Lane 5's behaviour classes. `slide_event` fires on 26 human and 24
machine tapes. `nested_sparse` sets therefore carry Σ 13.0 (human) and Σ 14.0 (machine).
Compare the pilot's `sparse` (`nested_v2`): 14 and 16 — the difference is the drop route.

**P-aa-2.** Every `home` grant in training has a prior `slide_event`, `farside` and `placed_v2`
grant in the same episode (structural; count of violations = 0).

**P-aa-3.** Under `nested_ramp`, by 50 % of budget at least one seed per arm shows
`slide_gain_m > 0` — the rung neither pilot ladder could express. Disconfirm → that arm is
rerun with the D7 goalward shaping ON, disclosed.

**P-aa-4 (the route census).** Of the `nested_v2` events in training rollouts, the fraction
that are also `home` is the slide share. Prediction: **under 50 % in every arm at every
milestone** (the demonstrations themselves sit at 13/14 human and 14/16 machine, and policies
have so far nested only by dropping). A slide share above 50 % in any arm would mean paying the
outcome alone induces the slide.

**P-aa-5.** Max episode return ≤ 9 (`nested_ramp`) / ≤ 1 (`nested_sparse`), and the r2dreamer
`return_clamp` equals the ladder's max return in every run's stamp.

**Decision rule.** If the pilot's sparse arm reaches `nested_v2` in ≥ 1 seed per arm, the
16v16 runs `nested_sparse`; otherwise `nested_ramp`. Either way the demonstration-side counts
above are registered first and the demo sets are built by re-execution (D5) under the same
ladder the runs train on.

**Disclosures.**
1. `released` requiring `picked` changes `released`/`pushed`/`contact_push` on any trajectory
   where `placed_v2` precedes the pick. On the 146 demonstrations it moves nothing; on the
   `rnd30` evaluation starts it removes 4 spurious firings per arm.
2. `far_release` is implemented and measured but **OFF**: it excludes no drop the slide clauses
   do not already exclude, and it removes 2 machine slides and 0 human ones (§4).
3. The ramp span of 0.10 m is never saturated by a human demonstration (p90 gain 4.7 cm); at
   the registered span a typical human slide earns 0.6–1.2 of the 2 available. Open decision.
4. The demonstration-side numbers here were produced on a 32-core AVX2 box. The stage records
   must be regenerated on the 64-core class before any of them is quoted in the paper.
5. Lane 5's `placed_only` / `push_no_nest` boundary was drawn with a predicate now known to be
   a transient detector, so the 8 human `slide_event` firings called false positives above are
   unadjudicated, not established errors.
