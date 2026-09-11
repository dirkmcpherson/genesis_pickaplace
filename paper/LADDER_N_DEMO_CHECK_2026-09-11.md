# Ladder N on the demonstrations (Lane 7, 2026-09-11)

**What this answers.** `LADDER_UNIFY_BRIEF_2026-09-10` §"Ladder N" proposes two new reward
ladders and asks, before either is registered: *what would they pay the demonstrations?* The
brief sets the bar itself — "the human set should earn near the maximum on its 13 sim-slides;
if it does not, the definition is wrong, not the humans."

**Scope.** Predicates, ladders, the scoring path and the demonstration-side measurement. No
cluster job was submitted, no cluster tree was touched, and the local GPU was not used.

Code: genesis_pickaplace `cf66293` (branch `ladder-unify-2026-09-11`), r2dreamer `f8743e0`.

---

## 0. The ladders, as implemented

| rung | `nested_sparse` | `nested_ramp` | requires |
|---|---|---|---|
| `picked` | — | +1 | — |
| `placed_v2` | — | +1 | `picked` |
| `farside` | — | +1 | `placed_v2` |
| slide ramp | — | +2 × min(1, `slide_gain_m` / 0.10), paid incrementally | `farside` |
| `home` | **+1**, terminal | **+4**, terminal | `farside` ∧ `slide_gain_m` ≥ 0.01 ∧ `nested_v2` |
| max return | 1 | 9 | |

`tipped` terminates with its existing (zero) penalty. Nothing else terminates. Every other
stage — `contact_push`, `slide_success`, `nested_v2`, the legacy proxies — is computed and
logged under both, and pays nothing.

The predicates (`baselines/stage_predicates.py`):

- **`farside`** (sticky) — `released` ∧ not `in_hand` ∧ dot(tool − can, goal − can) < 0 in xy ∧
  |tool_xy − can_xy| ≤ 0.08 m. No solver-contact term, deliberately: 10 of the 13 human
  sim-slides never make a can↔goal solver contact, so a contact clause would score the world
  rather than the human (that is the difference from the (g) `contact_push`).
- **`slide_gain_m`** — metres of goalward progress made on frames where the farside condition
  holds *this frame* and the can is not in hand, credited only on **new minima** of
  dist_xy(can, goal) since the release. The running minimum tracks *every* frame after the
  release, so progress made by CARRYING the can lowers the bar without paying; credit does not,
  so oscillation cannot farm the ramp. Monotone non-decreasing, which is what lets the ramp be
  paid incrementally with no clawback.
- **`home`** (sticky, terminal) — `nested_v2` ∧ `farside` granted ∧ `slide_gain_m` ≥ 0.01.
- **`release_far`** — whether the release that granted `released` happened at ≥ 0.10 m from the
  goal. Measured on every tracker; it *gates* `farside`/`slide_gain`/`home` only when the
  tracker is built with `far_release=True`.
- **`released` now requires `picked`.** 4 of the 30 `rnd30` starts satisfy `placed_v2` **at
  reset** with the arm at home and the can never touched (CONFOUNDS row 82), and `released`
  used to latch on frame 0 of those episodes. This is a real behaviour change, not a no-op; it
  is reachable in reverse via `released_requires_picked=False`, and on any history where the
  pick precedes the release — every real demonstration — the flags are identical frame for
  frame (`test_released_requires_picked_changes_nothing_when_the_pick_comes_first`).

---

## 1. Method: record once, score many

The user asked whether this re-copies the demonstrations for every variant. It does not.

A reward column does not steer the arm, so the only things a ladder changes about a
demonstration are **where the episode stops** and **what it pays**. So each tape is
re-executed through `FullTaskEnv(scope='full')` **once**, with termination suppressed
(`env.never_terminate`, an instance knob nothing else sets), and a **per-env-frame stage
record** is written: the poses, the tool point, the commanded grip, the solver contacts, the
env-owned stage flags — every input `StageTracker` and the reward loop consume — plus one
end-of-episode settle (`nested_honest`). Each candidate ladder is then applied **offline**,
in pure numpy, with no Genesis and no simulation.

```
# record (once per set)
python baselines/rl/relabel_reward.py --in  $D/demos_state_full/<set> \
                                      --records-out $D/stage_records/<set> --procs 8
# score (once per ladder x far_release)
python baselines/rl/relabel_reward.py --in  $D/demos_state_full/<set> \
       --out $D/relabel_dev/<set>_rnr --from-records $D/stage_records/<set> \
       --ladder nested_ramp [--far-release]
```

Two things make this safe rather than merely convenient:

1. **One reward loop, two call sites.** The loop moved out of `_step_once` into
   `full_env.LadderAccountant`. The env calls it per frame; the offline scorer calls the same
   object. They cannot drift, because there is nothing to drift from.
2. **It is verified, not assumed.** `--verify-against` scores a direct re-execution and an
   offline replay of the same tapes and asserts the reward columns and the terminal decisions
   are equal element for element.

<!--VERIFY-->

---

## 2. What the ladders pay the demonstrations

<!--TABLES-->

---

## 3. Where the specification had to change

<!--DEVIATIONS-->

---

## 4. Caveats

**Hardware.** These records were made on this development box (pop-os, AMD Ryzen 9 5950X,
32 cores, AVX2), not on the cluster's 64-core class. Genesis is not bit-identical across CPU
classes, and this box is known to disagree with the cluster on 6 of 74 human and 8 of 72
machine tapes after the first contact. **Records of record must be produced on the 64-core
class**; these are development artefacts, adequate for choosing a ladder on the demonstration
side and not a number for the paper. Every record and every manifest stamps its node, core
count and ISA. The re-run on the cluster costs one recording pass (~20 min on 8 cores) and
then scores every variant in seconds.

**A demonstration is not a policy.** These numbers say what the ladder *would have paid* a
demonstration, which is what the demo buffer will carry. They say nothing about whether a
learner can reach those rungs online. That is the pilot's question, not this one.

**An absent value is not a zero.** Every 0 below is a count over 74 or 72 tapes that were all
scored; the tape lists are in the manifests under `per_tape`.

---

## 5. Draft amendment (aa), for registration BEFORE any Ladder-N job

<!--AMENDMENT-->
