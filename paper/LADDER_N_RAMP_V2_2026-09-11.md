# `nested_ramp` v2 — folding `farside` into the slide ramp (revision of amendment (aa))

**What changed, in one line.** `farside` no longer pays its own 1.0; it moved into the
ramp's `requires`, and the vacated 1.0 moved onto the ramp's `scale` (2.0 → 3.0). Max
return is unchanged at 9.0. Everything else about `nested_ramp` — `picked`, `placed_v2`,
`home`, the `requires` chain, `far_release`, the tip guard — is untouched, and the other
three ladders (`staged`, `sparse`, `nested_sparse`) are proved byte-identical below.

Code: `baselines/rl/full_env.py`. Tests: `baselines/tests/test_ladder_unified.py`. Branch
`ladder-unify-2026-09-11`, this lane's worktree (12c).

---

## 0. Why

Lane 11's `paper/PILOT_RESCORE_2026-09-11.md` scored 600 policy rollouts (4 checkpoints ×
150 episodes) offline under `nested_ramp` and found that the checkpoint with the highest
mean return, `dDPfirst_s920` (2.14 of 9 over its 150 episodes), never once arrives at the
goal. The reason is geometric, not statistical: a gripper that sets the can down and then
withdraws **straight back** crosses the far-side band on its way out — Lane 5's census
tapes put it 2.5–8 cm behind the can, the opposite side from the goal — so `farside`
GRANTS on a retreat exactly as it would on an approach. Under the original `nested_ramp`
that was worth `picked(1) + placed_v2(1) + farside(1) = 3` of 9, for a policy that never
pushes anything (measured directly: `test_10b`'s construction reproduces the mechanism
exactly, and Lane 11's own record shows `dDPfirst_s920`'s `home` count is 0 in 390
episodes, `paper/E2E_AUDIT_BRIEF_2026-09-10.md`).

The coordinator's decision (registered here as the (aa) revision): `farside` becomes a
**logged prerequisite that pays nothing**; its 1.0 is folded into the slide ramp, which now
pays `3.0 × min(1, slide_gain_m / 0.05)` on new minima and **requires `farside`** to have
been reached at all. A withdrawal now earns exactly `picked + placed_v2 = 2`, whichever way
the tool retreats. The first credited centimetre of a *real* far-side push pays
`3.0 × 0.01/0.05 = 0.6`.

---

## 1. The code change

`baselines/rl/full_env.py`, the `'nested_ramp'` entry of `LADDERS` (line 154; `ramp=` at
line 162):

```python
# BEFORE
'nested_ramp': dict(stage_reward=dict(picked=1.0, placed_v2=1.0, farside=1.0, home=4.0),
                    requires=dict(placed_v2='picked', farside='placed_v2',
                                  slide_event='farside', home='slide_event'),
                    ramp=dict(key='slide_gain_m', scale=2.0, span=0.05, requires='farside'),
                    terminal=('home',)),

# AFTER
'nested_ramp': dict(stage_reward=dict(picked=1.0, placed_v2=1.0, home=4.0),
                    requires=dict(placed_v2='picked', farside='placed_v2',
                                  slide_event='farside', home='slide_event'),
                    ramp=dict(key='slide_gain_m', scale=3.0, span=0.05, requires='farside'),
                    terminal=('home',)),
```

`farside=1.0` is removed from `stage_reward` (line 154); nothing else in that line moves.
`ramp['scale']` goes from `2.0` to `3.0` (line 162, so `max_return('nested_ramp')` — the
sum of `stage_reward.values()` plus `ramp['scale']` — is `1 + 1 + 4 + 3 = 9`, unchanged.

**The `requires` dict is kept exactly as it was.** `farside: 'placed_v2'` is no longer
consulted by the payment loop (`LadderAccountant.frame` only looks up `requires[stage]` for
a `stage` that is a key of `stage_reward`, and `farside` is not one any more), but it still
documents the geometric chain a reader checks, and `slide_event: 'farside'` /
`home: 'slide_event'` are exactly as load-bearing as before — `home`'s payment is still
gated on `slide_event` having been reached, and `slide_event` itself can only become true
once the tracker's own `self.farside` is set (`stage_predicates.py:474`). Nothing here
needed to change for "`home` still needs `farside` → `slide_event`" to remain true.

`LadderAccountant` itself is untouched: it already treats `requires` and `ramp` as generic
structures (`baselines/rl/full_env.py:412-445`, method `frame`), so a rung that is *removed* from
`stage_reward` simply stops being looked at by the payment loop while remaining fully
tracked in `granted`/`LOGGED_STAGES` — which is exactly the "logged prerequisite, pays
nothing" behaviour the revision asks for, with no new code path.

`nested_sparse`, `staged`, `sparse` are untouched (different dict entries; the accompanying
comment block above `LADDERS` was extended to explain the revision but no other ladder's
values changed).

---

## 2. Tests

`baselines/tests/test_ladder_unified.py`, tests 10a–10f updated to the new numbers, plus a
new test 10g. All exercise the REAL `StageTracker` + `LadderAccountant` (no fakes) per the
file's existing convention for the Ladder-N section.

| test | before | after |
|---|---|---|
| 10a (full slide, 10 cm push) | pays 9 = 1+1+**1**+**2**+4; `paid` includes `farside` | pays 9 = 1+1+**3**+4; `paid` = `{picked, placed_v2, home}`, `farside` granted but never paid; `ramp_paid` saturates at 3.0 |
| 10b (drop, straight-back withdrawal) | pays 3 = 1+1+1 (farside cheap but non-zero); a sideways withdrawal pays 2 — the two retreat directions **disagreed** | pays 2 = 1+1 **either way** — the retreat direction no longer matters, because `farside`'s payment is gone, not detected-and-excluded |
| 10c (3 cm net push, ramp only) | `ramp_paid` = 2×30/50 = 1.2; total 1+1+1+1.2 = 4.2 | `ramp_paid` = 3×30/50 = 1.8; total 1+1+1.8 = 3.8 (no farside term) |
| 10d (far_release gates a close release) | unaffected — the gate blocks `farside` itself, so the gated case already paid exactly 2 and the ungated case already summed to 9 (2.0/9.0/9.0 unchanged; not modified) | same |
| 10e (`requires` defers an out-of-order rung) | first half pays 0 then 2 (no `farside`/ramp involved); second half (same-frame chain incl. `farside`/`slide_gain_m=0.10`) totals 9 | numerically unaffected — `farside`'s lost 1.0 and the ramp's gained 1.0 (saturated, scale 2→3 over the same 0.10 m value) cancel exactly, so the total stays 9 |
| 10f (staged/sparse unchanged; provenance) | asserted `ramp.scale == 2.0` | asserted `stage_reward` has no `farside` key, `stage_reward == {picked:1, placed_v2:1, home:4}`, `ramp.scale == 3.0`, `ramp.requires == 'farside'`, and the stamp text contains `ramp:slide_gain_m=3/0.05m` and no `farside=` term |
| **10g (new)** | — | three cases from one family of episodes (`_release_farside_push_home` with different `end_d`): a set-down + straight-back withdrawal through the far-side band pays **exactly 2.0**; the same episode with a 3 cm far-side push after the set-down latch pays **2 + 3·0.6 = 3.8**; with a 5 cm push and arrival, **9.0** (the ladder's `max_return`) |

Verified directly before writing the test (not just asserted inside it):

```
$ ~/workspace/genesis_sim2real/venv/bin/python - <<'EOF'
... _release_farside_push_home(start_d=0.105, end_d=0.105, n_push=40) -> reward=2.0
... _release_farside_push_home(start_d=0.115, end_d=0.085, n_push=40) -> reward=3.8000000000000016
... _release_farside_push_home(start_d=0.115, end_d=0.065, n_push=40) -> reward=9.0, home=True
EOF
```

Full suite, this venv:

```
$ ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_ladder_unified.py     # ALL OK (24 sub-tests)
$ ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_record_demos_contract.py   # ALL OK
$ ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_stage_predicates.py        # 39/39 passed
$ ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_terminal_guard.py           # ALL OK
```

No other ladder's or predicate's constant was touched. `pytest` was not used to run these
(project convention: plain `python <file>.py`), but the files are pytest-discoverable and
`pytest` is installed in this venv if a caller prefers `-m pytest baselines/tests -q`.

---

## 3. Demo side: the 74 human / 72 machine end-to-end tapes

Source: the stage records built by Lane 7
(`/home/j/data/genesis_pickaplace/stage_records/{dHfull_all,dDPfull_first}`, one row per
env frame, termination suppressed, ladder-independent). Rescored offline under the revised
`nested_ramp`, tip_guard `not_in_hand`, `far_release` off (default):

```bash
PY=~/workspace/genesis_sim2real/venv/bin/python
D=/home/j/data/genesis_pickaplace

$PY baselines/rl/relabel_reward.py \
    --in            $D/demos_state_full/dHfull_all \
    --from-records  $D/stage_records/dHfull_all \
    --out           <scratch>/dHfull_all_rnrh \
    --ladder nested_ramp --tip-guard not_in_hand

$PY baselines/rl/relabel_reward.py \
    --in            $D/demos_state_full/dDPfull_first \
    --from-records  $D/stage_records/dDPfull_first \
    --out           <scratch>/dDPfull_first_rnrh \
    --ladder nested_ramp --tip-guard not_in_hand
```

**Disclosure.** These were written to this lane's scratch, not to the shared
`$D/demos_state_full/{dHfull_all,dDPfull_first}_rnrh` — those two directories already exist
(built by Lane 7 under the *pre-revision* ladder, `farside=1.0`/`scale=2.0`, `full_env` sha
`a8894b00e460` stamped in their old `manifest.json`, vs `23fe428f222f` for this revision's
rebuild) and are shared filesystem state other lanes may be reading; this session did not
overwrite them. Whoever adopts (aa) v2 for training should regenerate those two in place
with the commands above (same tip_guard, same source, ~5-8 min each single-process on this
box).

Census join: `can_pos_recovery/videos_ladder_2026-09-11/census_{human,machine}.json` (Lane
5's independent behaviour classification, joined on the tape filename — same set, disjoint
method, so this is a genuine cross-check, not the reward loop grading itself).

### Per-set total

| set | tapes | Σ reward, old ladder as recorded | Σ reward, `nested_ramp` v2 (this revision) |
|---|---|---|---|
| human `dHfull_all` | 74 | 118.0 | **215.89** |
| machine `dDPfull_first` | 72 | 131.0 | **217.83** |

(For reference, the *pre-revision* `nested_ramp` — `farside` paid, ramp scale 2.0 — summed
to 235.93 on the human set per its persisted `manifest.json`; the drop to 215.89 is 39
tapes' worth of `farside`'s vacated 1.0 minus the ramp-scale increase's partial offset on
the tapes that push, which is exactly the redistribution the revision intends.)

### The 13 human / 14 machine sim-slide tapes (census `klass == 'slide'`)

Every one of these reaches `picked`, `placed_v2`, `farside`, `slide_event` and `home` — 13/13
and 14/14, matching Lane 7's calibration precision/recall exactly (`LADDER_N_DEMO_CHECK`
§"Headline").

| set | n | mean pay (of 9) | min | max | n at max (9.0) |
|---|---|---|---|---|---|
| human | 13 | **8.198** | 6.631 (`genesis-105000-052-234`, ic_uid 237) | 9.000 | 4 of 13 |
| machine | 14 | **8.078** | 6.761 (`genesis-107014-069-601`, ic_uid 311) | 9.000 | 6 of 14 |

(Compare Lane 7's registered span-0.05 measurement of the *old* ramp on the human set,
8.39 mean, 4 of 13 at max — the v2 numbers are close but not identical because that number
included `farside`'s flat +1 on every one of the 13, which the ramp-only scale-3 credit
does not exactly reproduce tape-for-tape; both report the same 13/13 coverage and the same
count at saturation.)

### Rungs reached, sim-slide tapes only (both sets 13/13 and 14/14 on every rung — unaffected by the revision, since these are tracker flags, not payments)

| rung | human (of 13) | machine (of 14) |
|---|---|---|
| `picked` | 13 | 14 |
| `placed_v2` | 13 | 14 |
| `farside` | 13 | 14 |
| `slide_event` | 13 | 14 |
| `home` | 13 | 14 |

### The drop/withdrawal census classes

| set | class | n | mean pay | min | max |
|---|---|---|---|---|---|
| human | `nested_drop` | 1 | 2.000 | 2.000 | 2.000 |
| human | `placed_only` | 16 | 2.969 | 2.000 | 5.000 |
| machine | `nested_drop` | 2 | 2.000 | 2.000 | 2.000 |
| machine | `placed_only` | 18 | 2.431 | 2.000 | 4.867 |

Every `nested_drop` tape pays exactly 2.0 — `picked + placed_v2`, nothing more, which is
precisely test_10b's construction reproduced on real demonstrations: a drop-and-retreat
tape reaches `farside` (it is in the class definition's own false-positive check,
Lane 7's grid) but the revision pays it nothing for that.

**Not asked for, but measured because the join made it visible and it bears on how tight
Lane 5's census classes are as a control:** 10 of 16 human and 5 of 18 machine `placed_only`
tapes (Lane 5's non-slide class, built from the older `pushed` predicate) reach `farside`
with **non-trivial, non-zero `slide_gain_m`** — up to 57.5 mm (human,
`genesis-102004-033-241`) and 47.8 mm (machine, `genesis-105019-056-601`), each paying up to
5.0/4.867 of 9. These are real partial far-side pushes that never arrived (no `home`), which
`LADDER_N_DEMO_CHECK`'s own §"the behaviour classes are a weak control" already flags as a
limitation of `placed_only`/`push_no_nest` as a boundary; the rest of `placed_only` (6/16
human, 13/18 machine) pay exactly 2.0, i.e. never reach `farside` or reach it with zero net
progress. None of this is new behaviour from the revision — the ramp's crediting of partial
progress on non-arriving tapes is unchanged; only the reward this credit sits on top of
(2.0 instead of 3.0 flat) changed.

---

## 4. Pilot side: re-scoring Lane 11's 600 policy records

Records: `paper/PILOT_RESCORE_2026-09-11.md` (600 episodes, 28 cells, 4 checkpoints ×
{rnd, hold} × seeds; SB3 SAC, `--ladder staged` live, `not_in_hand` was **not** used here —
`pilot_rescore.py` calls `offline_episode` with the module's tip-guard default, matching how
Lane 11 built the batch; unrelated to this revision). Located in this lane's own scratch,
which turned out to already hold Lane 11's artefacts (shared scratchpad on this box).

```bash
PY=~/workspace/genesis_sim2real/venv/bin/python
$PY baselines/diagnostics/pilot_rescore.py --records <scratch>/lane11/rec \
    --out <scratch>/lane11/rescore_aa.json > <scratch>/lane11/rescore_aa.md
```

To isolate the effect of *this* revision from Lane 7's earlier, separately-landed ramp-span
recalibration (0.10 m → 0.05 m, which reached this branch via the same merge before this
lane started and is NOT part of this change), the same 600 records were also scored against
the merged-but-pre-revision code (`git stash` the one-line `full_env.py` edit, rerun, `git
stash pop`) to get a genuine "before this revision" baseline on identical inputs:

```bash
git stash push -- baselines/rl/full_env.py
$PY baselines/diagnostics/pilot_rescore.py --records <scratch>/lane11/rec \
    --out <scratch>/lane11/rescore_before_aa.json > <scratch>/lane11/rescore_before_aa.md
git stash pop
```

**Verification first.** Sections 1 (`picked`/`placed_v2`/`farside`/`slide_event`/`home`/…
grant counts) and 3 (rungs each ladder GRANTED, `end=terminal` counts) of the two runs'
markdown are **byte-identical** — the revision changes only `nested_ramp`'s own reward
column (Section 2), never which predicate fires, when, or where any episode terminates.
That is the intended scope and it is exact, not approximate.

### Per-checkpoint mean return under `nested_ramp`, before vs after (n=150 each, rnd+hold combined)

| checkpoint | Σ before | mean before | n paying > placed level, before | Σ after | mean after | n paying > placed level, after | `home` count |
|---|---|---|---|---|---|---|---|
| `dDPfirst_s920` (machine, 250k) | 330.8 | 2.205 | 98 | **242.2** | **1.614** | **37** | 0 |
| `dH_s901` (human, 250k) | 300.5 | 2.004 | 66 | 240.8 | 1.605 | 14 | 4 |
| `dH_s940` (human, 100k pilot) | 62.2 | 0.414 | 10 | 52.2 | 0.348 | 1 | 0 |
| `dH_s941_c040` (human, 40k pilot) | 45.0 | 0.300 | 0 | 45.0 | 0.300 | 0 | 0 |

**The exploit is confirmed gone.** `dDPfirst_s920`'s clean reference ("picked+placed
level") is `Σ staged / n = (109.0 + 116.0) / 150 = 1.500` — under this checkpoint,
`contact_push` and `slide_success` are both 0 (Section 1 of both runs), so `staged`'s sum
*is* the pure picked+placed_v2 sum. The pre-revision mean (2.205) sat 0.705 above that
purely from `farside` touches on retreat; the post-revision mean (1.614) sits only 0.114
above it, and that residual is genuine partial-ramp credit (37 of 150 episodes still pay
above 2.0, each with measured nonzero `slide_gain_m`, none of them reaching `home`).
`dH_s940` — the pilot's *own* 100k checkpoint — shows the identical pattern at smaller
scale: 10 episodes exploited `farside` on retreat before the revision (Σ drops from 62.2 to
52.2, `n` beyond placed 10→1). `dH_s941_c040` never reaches `farside` at all (Section 1: 0
of 150 in both runs), so its sum is untouched to the decimal (45.0 = 45.0) — the cleanest
possible negative control that this revision touches nothing it should not.

**`dH_s901`'s four `home` episodes: MEASURED, not the number I was asked to confirm.** The
brief's expectation was that these four should "still pay 9". They do not — under this
revision they pay **8.509, 7.385, 6.831 and 7.542** of 9, and — checked directly against the
pre-revision code on the identical records — they did not pay 9 before the revision either
(8.673, 7.923, 7.554, 8.028). This is not a defect introduced by the (aa) revision; it is
how `home` and the ramp interact by construction. `home` requires only `slide_event`
(released ∧ farside ∧ `slide_gain_m ≥ SLIDE_GAIN_MIN_M = 0.01`) **and** `nested_v2`
(arrival) — a threshold condition that can fire, and empirically does fire, well before the
can has travelled the ramp's full 5 cm span. `home` is `terminal`, so the moment it fires
the episode — and the ramp's accumulation — stops:

| episode | `slide_gain_m` at `home` | ramp span | ramp fraction walked | pay before | pay after |
|---|---|---|---|---|---|
| `dH_s901_hold_s1/ep8_uid295` | 41.8 mm | 50 mm | 84% | 8.673 | 8.509 |
| `dH_s901_hold_s3/ep14_uid335` | 23.1 mm | 50 mm | 46% | 7.923 | 7.385 |
| `dH_s901_rnd_s0/ep27_uidrnd` | 13.9 mm | 50 mm | 28% | 7.554 | 6.831 |
| `dH_s901_rnd_s2/ep17_uidrnd` | 25.7 mm | 50 mm | 51% | 8.028 | 7.542 |

Every one arrives with room to spare above the 1 cm `SLIDE_GAIN_MIN_M` gate but below the
5 cm span, so `home`'s +4 lands on top of a partial ramp, never the saturated one. This is
identical before and after the revision (same `slide_gain_m`, same terminal frame in both
runs — Section 1/3 parity above already proved the frame doesn't move); what the revision
changes is only the ramp's per-mm rate (`ramp_paid` scales by exactly 3/2 between the two
runs, e.g. `0.923 × 1.5 = 1.385`), and the loss of `farside`'s flat 1.0 nets out to a small
decrease (−0.164 to −0.723) rather than the "unchanged at 9" the brief assumed. **The
correct summary is: `home` reliably pays its full +4 on all four episodes, both before and
after; none of the four reaches the reward ceiling, and that has nothing to do with this
revision.**

**Caveat carried from the source document, unaffected by this revision but relevant to
reading the table above:** 12–13 of each `dH_s901` cell's 150 episodes are *censored* for
`home` (the live run's own `staged` terminal, `slide_success`, ended the record before a
`nested_ramp` `home` could be observed one way or the other — `paper/PILOT_RESCORE
_2026-09-11.md` §"WHAT IS AND IS NOT MEASURABLE"). `dDPfirst_s920`'s numbers carry no such
censoring (0 of 150 in both runs — it never reaches `slide_success` under `staged`), so its
mean-drop confirmation above is on fully-observed data.

---

## 5. Draft paragraph — revision of PHASE_PLAN amendment (aa)

> **Revision of (aa), 2026-09-11.** `nested_ramp`'s `farside` rung is withdrawn as a paid
> stage: Lane 11's offline re-score of 600 policy rollouts under the registered ladder found
> that a gripper which sets the can down and withdraws straight back crosses the far-side
> band on its way out, collecting `farside`'s +1 (and, on the pilot's own 100k checkpoint,
> doing so on 10 of 150 episodes) without ever pushing the can — the highest-scoring
> checkpoint in that batch (mean 2.14 of 9) never once arrived at the goal. `farside`
> remains a logged, sticky flag (unchanged tracker semantics, `stage_predicates.py`) and a
> hard prerequisite for the slide ramp and for `slide_event`/`home`, but it pays nothing of
> its own; its vacated 1.0 is folded into the ramp's scale (2.0 → 3.0 over the existing
> 0.05 m span), so `max_return('nested_ramp')` is unchanged at 9.0 and a genuine far-side
> push is credited from its very first centimetre (3.0 × 1 cm / 5 cm = 0.6) rather than only
> once it clears a flat gate. Re-scoring the 74 human / 72 machine end-to-end demonstration
> tapes under the revision confirms the 13/13 and 14/14 sim-slide coverage measured in
> `LADDER_N_DEMO_CHECK_2026-09-11.md` §"Headline" is untouched (a tracker-level result, not a
> reward-level one) and that every drop/withdrawal tape in Lane 5's census now pays exactly
> `picked + placed_v2 = 2`, matched exactly by `dDPfirst_s920`'s post-revision mean return
> (1.614) collapsing onto its own picked+placed reference level (1.500) on the same 600
> rollouts. Four `dH_s901` episodes reach `home` in that batch; measured directly, none of
> the four reaches the ladder's 9.0 ceiling either before or after this revision, because
> `home` is a threshold condition (arrival plus ≥1 cm of credited slide) that can and does
> fire before the ramp's 5 cm span is walked — an existing property of the ladder, not a
> consequence of this change, and worth stating precisely rather than as "still pays 9".

---

## 6. What is committed, what is not

Committed in this lane's worktree: `baselines/rl/full_env.py` (the `LADDERS['nested_ramp']`
entry and its comment block), `baselines/tests/test_ladder_unified.py` (tests 10a/10b/10c/10f
updated, 10g added), this document.

Not committed / not touched: the shared `$D/demos_state_full/{dHfull_all,dDPfull_first}_rnrh`
relabelled sets (still reflect the pre-revision ladder — see §3's disclosure), Lane 11's
`<scratch>/lane11/rescore*.json` (development artefacts, this box's AVX2/32-core class, not
the cluster's 64-core class of record — same caveat `PILOT_RESCORE_2026-09-11.md` and
`stage_records/README.md` already carry for every number of this kind), any other ladder or
predicate constant.
