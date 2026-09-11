# The tip guard, implemented (Lane 12a, 2026-09-11)

PHASE_PLAN amendment (aa), paragraph "Tip rule", from Lane 6's measurement
(`paper/TIP_RULE_2026-09-11.md` §5 and §10). **Threshold, termination and penalty are
UNCHANGED** — `TIP_DEG = 60.0`, terminate, `TIP_PENALTY = 0.0`. Only the GUARD moves, and
only when a caller asks for it.

| | rule of record (`tip_guard='grip'`) | amendment (aa) (`tip_guard='not_in_hand'`) |
|---|---|---|
| guard | commanded grip `a_phys[6] < GRIP_OPEN (0.3)` | the StageTracker's `not in_hand` (`\|tool_xy − can_xy\| >= HELD_LEVER_M = 0.025 m`), **no gripper term** |
| sustain | 1 env frame | **4 consecutive env frames** (1 decision at `action_repeat=4`) |
| threshold | `tilt_deg(can) > 60` | same |
| terminates | yes | same |
| penalty | 0.0 (−0.25/−0.1 in `scope='place'` only) | same |

**The class default stays `'grip'`.** Nothing that does not name a guard changes behaviour:
the pilot's semantics, every evaluator, every recorder and every phase scope are
bit-identical (`'grip'` keeps sustain 1, so even the sustain counter is a no-op there).

Branch `worktree-agent-a1b8a1ff26ca40700` off `ladder-unify-2026-09-11` @ `330a9b1`.
r2dreamer tree `~/workspace/r2dreamer`, branch `main`, commit **`16e03cb`** (parent
`2c0a686`).

Everything below is LOCAL and CPU-only: the VPN is down, so nothing was submitted, nothing on
the cluster was read or written, and no GPU was used.

---

## 1. What changed

### `baselines/stage_predicates.py`
Two exported helpers, `lever_m(tool_xy, can_xy)` and `is_in_hand(tool_xy, can_xy,
held_lever_m=HELD_LEVER_M)`, and `StageTracker.update` now CALLS them instead of inlining the
same two lines. One definition of `in_hand`, read by the tracker, by the tip guard in
`FullTaskEnv` (indirectly, through the tracker's own per-frame flag) and by
`CartesianFullTaskEnv` (directly). The tracker's arithmetic is unchanged —
`test_stage_predicates.py` 39/39 still passes.

### `baselines/rl/full_env.py`
* `TIP_GUARD_CHOICES = ('grip', 'not_in_hand')`, `TIP_GUARD_DEFAULT = 'grip'`,
  `TIP_GUARD_SUSTAIN = {'grip': 1, 'not_in_hand': 4}` — module constants beside the ladder
  table, with the measurement that motivates them in the comment.
* `FullTaskEnv.__init__(..., tip_guard='grip')`. A constructor argument, never an env var.
  `not_in_hand` is REFUSED outside `scope='full'` (see §2). `self._tip_free_run` is the
  per-episode sustain counter, reset in `_reset_tracker()` and at both phase-reset sites.
* The tip site in `FullTaskEnv._step_once` selects the guard, then counts consecutive frames
  of **the conjunction** `(tilt > TIP_DEG) AND guard`, then fires at `>= tip_guard_sustain`.
  With `'grip'` at sustain 1 the arithmetic reduces to the previous expression exactly,
  including its short-circuit order (the can quaternion is still read only when the guard
  already holds).
* The second tip site, `CartesianFullTaskEnv`, takes the same argument with the same default
  and answers `not_in_hand` with `stage_predicates.is_in_hand` (that class runs no tracker;
  `in_hand` is instantaneous, so nothing is lost).
* `ladder_provenance()` gained `tip_guard`, `tip_guard_sustain_frames`, `tip_deg` and
  `tip_penalty`; `ladder_stamp()` gained a `tip=tilt>60deg&<guard>@<n>f` field. An unknown
  guard name RAISES rather than falling back.
* `refuse_legacy_gates()` now also refuses `FULLENV_TIP_GUARD`, so a tree that someone
  expects to read an env var fails loudly instead of silently running the rule of record.
* `terminal_from_tape(..., tip_guard=...)` RAISES `NotImplementedError` for anything but
  `'grip'` — see §5(c).

### Trainers and launchers — the guard is REQUIRED, never defaulted
| file | how |
|---|---|
| `baselines/rl/train_rlpd.py` | `--tip-guard {grip,not_in_hand}`, `required=True`; asserted against `env.tip_guard`; refused outside `scope=full`; written into the checkpoint sidecar and into `ladder_provenance.json` (via `env.provenance()`) |
| `cluster/sbatch_rlpd_e2e.sh` | `TIP_GUARD=${TIP_GUARD:?...}`, validated, passed as `--tip-guard`, added to the run-registry knobs, and printed in the pre-training `[ladder]` stamp |
| `cluster/wmfix_full.sbatch` | same, passed to the adapter as `env.tip_guard=$TIP_GUARD` |
| `~/workspace/r2dreamer/envs/genesis.py` | `GenesisPick(tip_guard=...)` **raises** on `None` — no default |
| `~/workspace/r2dreamer/envs/__init__.py` | `tip_guard=config.get("tip_guard", None)` — passes the absence through so the adapter can refuse it |
| `~/workspace/r2dreamer/configs/env/genesis_full_state.yaml` | `tip_guard: null` (the launcher must set it) |
| `~/workspace/r2dreamer/train.py` | reads `env.tip_guard` for the logdir provenance and **raises** if unset |
| `cluster/relabel_e2e_sets.sbatch` | `TIP_GUARD=${TIP_GUARD:?...}`; appends `h` to the output-set suffix |

### Evaluators
`baselines/eval_e2e.py` and `~/workspace/r2dreamer/eval_genesis.py` take the guard from the
**run's own record** — the {RLPD} checkpoint sidecar and the r2dreamer run config — exactly
as they already take the ladder. `--tip-guard` on `eval_e2e.py` is an optional ASSERTION and
a disagreement is fatal. A checkpoint or config that names no guard predates the amendment,
so it resolves to `'grip'` and the evaluator PRINTS that it did. `env.provenance()` goes into
`metrics.json`, so every eval cell now records the guard it ran.

### Relabel and annotation
* `baselines/rl/relabel_reward.py --tip-guard {grip,not_in_hand}`, `required=True`, honoured
  by all four modes (direct re-execution, `--records-out`, `--from-records`,
  `--verify-against`), carried into every worker subprocess, stamped per tape
  (`rz_tip_guard`), into `manifest.json` (`tip_guard`, `tip_guard_sustain_frames`, and inside
  `ladder_provenance` / `ladder_stamp`) and into `repeat.json` (`relabel.tip_guard`), which is
  the file the launchers gate on. The output-set suffix gains `h` for `not_in_hand`
  (`_rz` → `_rzh`, `_rnrf` → `_rnrfh`), and a mismatched `--out` is refused.
* A STAGE RECORD is guard-independent — nothing terminates during a `--records-out` pass and
  the record holds both guards' inputs (`grip_cmd`, `tool_xy`, `can_pos`, `can_quat`) — so
  one record scores either guard. The record manifest stamps `recording_env_tip_guard` as
  provenance only, plus `tip_guard_applicable: [grip, not_in_hand]`.
* `baselines/annotate_demos.py --tip-guard`, `required=True`. The `--mark-tip` red marker
  stays the RULE OF RECORD, so a clip rendered under the new guard shows both: where the old
  rule would have fired, and where the live one did. The terminal card gained a
  `LIVE guard: <guard>, sustain N env frames` line; every census row is stamped.

### New diagnostics
* `baselines/diagnostics/tip_guard_demo_check.py` — Lane 6's §5 guard table, recomputed from
  Lane 7's stage records (§3).
* `baselines/diagnostics/tip_guard_ladder_diff.py` — what the guard costs a LADDER, per tape
  (§4). Both call `stage_predicates` / `full_env` / `relabel_reward` rather than
  re-implementing a predicate or a reward.

## 2. What happens in the other scopes

`not_in_hand` is refused outside `scope='full'` and the phase scopes stay on `'grip'`.
The reason is not caution, it is that the guard is not computable there: `FullTaskEnv`
builds a `StageTracker` only for `scope='full'` (`__init__`: `self.tracker = None` otherwise),
so `self._track` is empty in `place` / `contact` / `carrycontact` / `pick` / `reach` /
`touchgoal` / `reach_goal` and there is no `in_hand` to read. A silent fall-back to the old
guard for a caller who asked for the new one is the defect class this amendment exists to
remove, so the constructor asserts instead:

    tip_guard='not_in_hand' needs the StageTracker's in_hand, which only scope='full' runs

Nothing about the phase scopes changes: with `'grip'` at sustain 1 the new code path is the
old expression. `scope='place'`'s `PLACE_TIP_PENALTY` branch is untouched.

`CartesianFullTaskEnv` is the exception: it has no tracker either, but it is a separate class
with its own poses to hand, so it answers `not_in_hand` from `stage_predicates.is_in_hand`
directly. It has no live runs; the argument exists so the two tip sites cannot drift apart
again. Its default is `'grip'` and its behaviour under that default is unchanged.

## 3. Demo-side counts — Lane 6's numbers reproduce exactly

**The measurement is independent of Lane 6's.** Lane 6 measured from its own probe
(`baselines/tip_rule_probe.py`, which disables the rule with `env.TIP_DEG = 1e9` on the
instance). These numbers come from **Lane 7's stage records** — a different re-execution of
the same tapes, with termination suppressed by `env.never_terminate` — scored by a different
script. The `grip` rows are the cross-check that the two re-executions are the same
trajectories; had they disagreed, nothing below would be comparable.

```bash
V=~/workspace/genesis_sim2real/venv/bin/python ; D=/home/j/data/genesis_pickaplace
$V baselines/diagnostics/tip_guard_demo_check.py \
   --records $D/stage_records/dHfull_all     --set-name human \
   --census can_pos_recovery/videos_ladder_2026-09-11/census_human.json \
   --records $D/stage_records/dDPfull_first  --set-name machine \
   --census can_pos_recovery/videos_ladder_2026-09-11/census_machine.json
```

**All 146 tapes** ({human tapes} `dHfull_all` 74 + {machine tapes} `dDPfull_first` 72):

| guard | sustain | terminates | fires while IN HAND | recovers later | free-flat cans MISSED | Lane 6 §5 |
|---|---|---|---|---|---|---|
| `grip` (of record) | 1 | **25** | **6** | 0 | **29** | 25 / 6 / 0 / 29 ✅ |
| `grip` | 4 | 21 | 3 | 0 | 30 | 21 / 3 / 0 / 30 ✅ |
| `not_in_hand` | 1 | 52 | 0 | **1** | 0 | 52 / 0 / 1 / 0 ✅ |
| **`not_in_hand` (adopted)** | **4** | **46** | **0** | **0** | **2** | 46 / 0 / 0 / 2 ✅ |
| `not_in_hand` | 12 | 39 | 0 | 0 | 9 | 39 / 0 / 0 / 9 ✅ |

Every registered number reproduces: **in-hand firings 6 → 0, missed free-horizontal cans
29 → 2, recoveries lost 0, tapes ending `tipped` 25 → 46.**

Per set, at the adopted setting:

| set | tapes | `grip` terminates | `not_in_hand@4` terminates | in-hand firings (grip → new) | free-flat missed (grip → new) | median decisions left at the fire |
|---|---|---|---|---|---|---|
| {human tapes} `dHfull_all` | 74 | 13 | **26** | 3 → 0 | 16 → 2 | 0 → 65 |
| {machine tapes} `dDPfull_first` | 72 | 12 | **20** | 3 → 0 | 13 → 0 | 0 → 277 |

**29 tapes gain a tip termination; 8 lose one.** The 8 that lose it are the 6 in-hand firings
(human 235, 239, 262; machine 239, 250, 252) plus 2 whose can is free but whose flat spell is
shorter than the 4-frame sustain at the moment the old rule fired (machine 275, 316). The 2
free-flat cans the amendment still misses are **human 235 and human 239** — the same two
in-hand releases, whose can does go flat later but not for 4 consecutive frames while free.

The rule of record's six in-hand firings, with the tool-to-can lever on the firing frame:
human 262 (0.021 m), human 235 (0.018 m), human 239 (0.018 m), machine 239 (0.002 m),
machine 250 (0.020 m), machine 252 (0.013 m) — all inside `HELD_LEVER_M = 0.025 m`, i.e. the
can was demonstrably in the gripper. This is the thing the user saw in the clips.

### Hardware caveat, stated before anyone quotes these
The stage records were made on **pop-os, AMD Ryzen 9 5950X, 32 cores, AVX2** — NOT the
cluster's 64-core class. Genesis is not bit-identical across CPU classes (the same human set
pays Σ 171 on the cluster and Σ 179 here). These are **development artefacts**: adequate for
deciding a guard, not a number for the paper. The counts of record must be regenerated from
records made on the 64-core class. Amendment (aa)'s **P-aa-7** already registers this
expectation for the `home` counts (± 1 tape per set); nothing here contradicts it, and nothing
here tests it.

## 4. What the guard costs a LADDER

```bash
$V baselines/diagnostics/tip_guard_ladder_diff.py \
   --records $D/stage_records/dHfull_all    --set-name human \
   --census can_pos_recovery/videos_ladder_2026-09-11/census_human.json \
   --records $D/stage_records/dDPfull_first --set-name machine \
   --census can_pos_recovery/videos_ladder_2026-09-11/census_machine.json \
   --ladder staged --ladder nested_ramp:far
```

Every stage record scored twice under one ladder, through `relabel_reward.offline_episode` —
the function `--from-records` calls, which shares `full_env.LadderAccountant` with the live
env. This is the in-memory form of building the `_rz` and `_rzh` sets; the CLI build of those
two sets is §5's corroboration.

### `staged` (the ladder Lane 6's reward table uses; the (aa) CONTROL arm)

| | {human tapes} n=74 | | {machine tapes} n=72 | |
|---|---|---|---|---|
| guard | `grip` | `not_in_hand@4` | `grip` | `not_in_hand@4` |
| **Σ reward** | **179.0** | **177.0 (−2.0)** | **192.0** | **192.0 (±0)** |
| end `tipped` | 13 | **26** | 12 | **20** |
| end `slide_success` | 13 | 13 | 14 | 14 |
| end `stream_exhausted` | 31 | 23 | 7 | 12 |
| end `truncated` | 17 | 12 | 39 | 26 |
| `picked` | 65 | 65 | 64 | 64 |
| `placed_v2` | 42 | 42 | 44 | 44 |
| `contact_push` | 10 | **9** | 14 | 14 |
| `slide_success` | 13 | 13 | 14 | 14 |
| `slide_event` | 25 | 25 | 24 | 24 |
| `nested_v2` | 14 | 14 | 16 | 16 |
| `home` | 12 | 12 | 14 | 14 |

**Exactly one tape in 146 pays differently: human 274, −2, losing `contact_push`.** Lane 6's
registered cost reproduces exactly. That tape's can is horizontal and free from decision ~448
with the fingers commanded shut (grip 1.00), so its `contact_push` is a FLAT can shoved into
the goal — `contact_push` has no tilt clause, which is a separate predicate hole, not a reason
to keep the guard. Full row:

```
human 274 staged      grip        : reward 4.0  end stream_exhausted@503
human 274 staged      not_in_hand : reward 2.0  end tipped@449            (lost contact_push)
human 274 nested_ramp grip        : reward 5.0  end stream_exhausted@503
human 274 nested_ramp not_in_hand : reward 5.0  end tipped@449            (no cost: contact_push is not a Ladder-N rung)
```

### `nested_ramp --far-release` (the (aa) batch's Ladder-N arm)

| | {human tapes} n=74 | | {machine tapes} n=72 | |
|---|---|---|---|---|
| guard | `grip` | `not_in_hand@4` | `grip` | `not_in_hand@4` |
| **Σ reward** | **233.9** | **233.9 (±0)** | **227.8** | **227.8 (±0)** |
| end `home` | 13 | 13 | 12 | 12 |
| end `tipped` | 13 | **26** | 12 | **20** |
| `picked` / `placed_v2` / `farside` | 65 / 42 / 37 | same | 64 / 44 / 37 | same |
| `slide_event` | 26 | 26 | 22 | 22 |
| **`home`** | **13** | **13** | **12** | **12** |

**The registered prediction holds: no `home`-paying tape changes its grants under the new
guard — the sets are identical tape for tape, not merely equal in count, and Σ reward does
not move at all.** The one tape whose grants move anywhere (human 274) pays no `home` and no
`nested_v2` under either guard.

### The horizon this buys back
Tapes whose END REASON moves: **19 of 74 human** (median **81** decisions saved, max 248) and
**18 of 72 machine** (median **318**, max 452). One machine tape ends 3 decisions LATER — its
old firing frame needed 4 sustained frames under the new rule.

### One number here disagrees with `LADDER_N_DEMO_CHECK_2026-09-11.md`, and it is not the guard
That document reports `home` firing on **13 human / 14 machine** sim-slides. Under
`nested_ramp --far-release` I measure **13 human / 12 machine**; under `staged`
(`far_release` off) I measure 12 human / **14** machine. Both guards give the same numbers, so
**this is not a tip-guard effect** — it is `far_release` (which re-gates `farside` →
`slide_event` → `home`) and the 32-core-vs-64-core record class. Whoever quotes the `home`
census must say which `far_release` setting and which hardware class produced it. Flagged, not
investigated — the Ladder-N predicate is not this lane's to move, and the discrepancy does not
touch the guard decision, which is identical either way.

## 5. Offline == direct, under the new guard

The offline scorer applies the guard from the record's own columns; the env applies it from
live solver reads. That those agree is asserted, not assumed — `--verify-against` re-executes
each tape through Genesis AND scores its stage record offline, then compares the reward column
element for element and the terminal decision:

```bash
$V baselines/rl/relabel_reward.py --in $D/demos_state_full/dHfull_all \
   --verify-against $D/stage_records/dHfull_all \
   --ladder staged --tip-guard not_in_hand --limit 14
```

```
VERIFY ladder=staged far_release=False tip_guard=not_in_hand:
    14/14 tapes identical (reward column AND terminal decision)
wall clock: direct re-execution 325.1s, offline replay 37.61s (9x)
```

**14 of 14, with the discriminating cases inside the sample** — this is not 14 tapes on which
the guard never fires:

| # | tape | reward | terminal | why it discriminates |
|---|---|---|---|---|
| 2 | `genesis-100001-014-171` | 2.0 | `tipped@89` | fires ONLY under the new guard (offline predicted decision 88 → end 89) |
| 4 | `genesis-100003-016-229` | 2.0 | `stream_exhausted@228` | **human 262** — the `grip` guard's in-hand firing at d227 is GONE, and the tape runs to its end |
| 8 | `genesis-100007-020-466` | 1.0 | `tipped@217` | fires only under the new guard (offline: 216) |
| 10 | `genesis-100009-022-601` | 1.0 | `tipped@383` | fires only under the new guard (offline: 382) |
| 1, 5, 11 | — | 6.0 / 6.0 / 8.0 | `slide_success` | the guard does not cut a successful slide short |
| 7 | `genesis-100006-019-601` | 4.0 | `truncated@600` | a 600-decision tape the guard never fires on |

Three new-guard firings and one removed old-guard firing, all reproduced by BOTH paths at the
same decision. The end decisions also match the offline predictions of §3 exactly (89 = 88+1,
217 = 216+1, 383 = 382+1).

### The CLI build agrees with the in-memory diff
The four human sets §4's numbers describe were also built through the real CLI
(`--from-records`), and their manifests reproduce the diff exactly:

| set | ladder | far | guard | Σ reward | end reasons |
|---|---|---|---|---|---|
| `dHfull_all_rz` | staged | 0 | grip | 179.0 | slide_success 13, stream_exhausted 31, **tipped 13**, truncated 17 |
| `dHfull_all_rzh` | staged | 0 | **not_in_hand** | **177.0** | slide_success 13, **tipped 26**, stream_exhausted 23, truncated 12 |
| `dHfull_all_rnrf` | nested_ramp | 1 | grip | 233.9 | home 13, stream_exhausted 31, **tipped 13**, truncated 17 |
| `dHfull_all_rnrfh` | nested_ramp | 1 | **not_in_hand** | **233.9** | home 13, **tipped 26**, stream_exhausted 23, truncated 12 |

**The action streams are untouched by the guard**: aggregate `actions_sha256`
`77bc48750f1d975b` and all 74 per-tape shas identical between `_rz` and `_rzh`. `manifest.json`
carries `tip_guard` / `tip_guard_sustain_frames` and the guard inside `ladder_provenance` /
`ladder_stamp`; `repeat.json` — the file both launchers gate on — carries
`relabel.tip_guard`. The machine half of the CLI build was cancelled for CPU (the in-memory
diff of §4 covers it and calls the same function); it is one command to reproduce.

### The refusals fire
```
$ train_rlpd.py --steps 1
train_rlpd.py: error: the following arguments are required: --tip-guard
$ relabel_reward.py --in ... --out x_rz --ladder staged --tip-guard not_in_hand --dry-run
FATAL: --ladder staged --tip-guard not_in_hand writes a _rzh set, but --out is 'x_rz'. ...
$ ARM=dH SEED=0 LADDER=staged DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh
sbatch_rlpd_e2e.sh: line 88: TIP_GUARD: set TIP_GUARD (grip | not_in_hand) -- ... PHASE_PLAN (aa)
$ ... TIP_GUARD=hand DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh
FATAL: TIP_GUARD must be grip | not_in_hand (got hand)
```
The RLPD launcher's dry run cannot go further on this box — the next gate is the cluster disk
floor, which reads `? GB` off a path that does not exist here. That the flag reaches the
trainer is checked by reading `TRAIN_ARGS` and by `train_rlpd.py --help` accepting it, not by
running the launcher to completion.

## 6. Tests

`baselines/tests/test_ladder_unified.py` gained four cases. All four test files still pass —
`test_ladder_unified` ALL OK (21 cases including the new four), `test_stage_predicates` 39/39,
`test_terminal_guard` ALL OK, `test_record_demos_contract` ALL OK.

```bash
~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_ladder_unified.py   # ALL OK
~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_stage_predicates.py # 39/39
~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_terminal_guard.py   # ALL OK
~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_record_demos_contract.py
```

| test | what it pins |
|---|---|
| `test_aa1_a_held_can_tilted_past_60_with_the_fingers_open` | Lane 6 defect 1. Can at 90°, lever 0.010 m (inside `HELD_LEVER_M`), fingers COMMANDED OPEN: terminates under `grip`, does NOT terminate under `not_in_hand` over 8 frames. |
| `test_aa2_a_free_flat_can_with_the_fingers_in_a_fist` | Lane 6 defect 2. Can at 90°, lever 0.30 m, fingers commanded **0.41** (the amendment-(p) fist): terminates under `not_in_hand`, does NOT under `grip`. |
| `test_aa3_the_four_frame_sustain` | 3 free frames then back in hand → no termination and the run resets to 0; 4 consecutive → fires on the 4th. Also: an UPRIGHT free can never accumulates (the tilt clause is inside the conjunction), `TIP_GUARD_SUSTAIN` is `{grip: 1, not_in_hand: 4}`, `TIP_GUARD_DEFAULT == 'grip'`, `TIP_DEG == 60.0`, `TIP_PENALTY == 0.0`. |
| `test_aa4_the_guard_is_in_the_stamp_and_is_never_an_env_var` | The stamp differs between the guards (`tip=tilt>60deg&grip@1f` vs `tip=tilt>60deg&not_in_hand@4f`); the provenance dict carries `tip_guard` / `tip_guard_sustain_frames` / `tip_deg` / `tip_penalty`; a typo in the guard name RAISES; the bare stamp is the rule of record; `full_env.py` contains no read of a `TIP_GUARD` environment variable; both env classes take the constructor argument. |

The fake-world rig (`make_env`, which bypasses `__init__` on purpose) now sets `tip_guard`,
`tip_guard_sustain` and `_tip_free_run`, so a future attribute added to the guard cannot go
missing there and pass vacuously.

## 7. What the amendment text should correct

**(a) The sustain is on the CONJUNCTION, and the text does not say so.** Amendment (aa) reads
"the tracker's `not in_hand` sustained 4 env frames", which parses as "sustain the guard, read
the tilt now". Lane 6's registered numbers come from
`tip_rule_probe._first_sustained((tilt > T) & guard, k)` — the guard AND the tilt, together,
for 4 frames. **The env implements the conjunction**, because that is what reproduces the
registered 46. Measured cost of the ambiguity, both readings on all 146 tapes:

| reading | terminates | in hand | recovers | free-flat missed |
|---|---|---|---|---|
| conjunction sustained 4 (IMPLEMENTED, = Lane 6) | 46 | 0 | 0 | 2 |
| guard alone sustained 4, tilt instantaneous | 47 | 0 | 0 | 2 |

One machine tape of 146. Worth one clause in the amendment, not a re-registration:
`tilt > TIP_DEG AND not in_hand, both holding for 4 consecutive env frames`.
(`tip_guard_demo_check.py --guards not_in_hand_gonly:4` reproduces the second row.)

**(b) Demo-set naming.** The brief calls the new sets `_rh` / `_rn`. The suffix this
implementation writes composes from the existing map: ladder suffix + `f` for
`--far-release` + `h` for `not_in_hand`, so the (aa) batch's sets are **`_rzh`** (staged +
new guard, the control arm) and **`_rnrfh`** / **`_rnsfh`** (Ladder N + far_release + new
guard). `relabel_reward.py` REFUSES an `--out` whose basename does not end in the computed
suffix and names the expected one, so this is discoverable rather than silent — but if the
coordinator wants `_rh`/`_rn` the map in `relabel_reward.LADDER_SUFFIX` /
`TIP_GUARD_SUFFIX` is the single place to change, and `cluster/relabel_e2e_sets.sbatch`'s
`case` must change with it. **I did not guess the intended names.**

**(c) `terminal_from_tape` cannot carry the new guard, and now says so.** The demo tape
layouts hold no tool point: a legacy stride-1 tape has only `states`/`actions`, and a
contract-v1 tape's `tipped` column is whatever guard its RECORDER ran. So
`full_env.terminal_from_tape(tip_guard='not_in_hand')` raises `NotImplementedError` pointing
at the stage-record path. This does not touch the end-to-end batch: `train_rlpd
--demo-format segment` reads `is_terminal` off the tape
(`full_demos.segment_transitions_full`) and never calls it; the pick / SACfD / legacy routes
that do call it stay on `'grip'`, which is correct for them.

**(d) The relabel does NOT move a tape's `is_terminal` column, and the new guard widens the
gap.** `relabel_reward` overwrites `reward` and records the re-execution's terminal as
`rz_end_reason` / `rz_end_decision`, but `is_terminal` is inherited verbatim from the source
tape (`write_repeat_json`, LADDER_IMPL_NOTES §4). Measured on the source sets: only
**18 of 74 human and 22 of 72 machine** tapes carry a terminal row at all, while the
re-execution terminates 25 tapes under `grip` and **46** under `not_in_hand`, a median of
88 decisions earlier. So under the new guard, 46 tapes have a reward column that stops at the
tip and a terminal flag that does not — the RLPD critic bootstraps through states the env
would never continue from, which is the AUDIT_impl F1 defect `--demo-terminal-guard` exists
to prevent. **This is pre-existing and not mine to decide**, but it is bigger under (aa) than
under the rule of record and the coordinator should rule on it before the `_rzh` sets train.

```bash
# the count above
~/workspace/genesis_sim2real/venv/bin/python - <<'PY'
import numpy as np, glob
for s in ('dHfull_all','dDPfull_first'):
    fs=sorted(glob.glob(f'/home/j/data/genesis_pickaplace/demos_state_full/{s}/*.npz'))
    print(s, len(fs), sum(int(np.asarray(np.load(f,allow_pickle=True)['is_terminal'],bool).any()) for f in fs))
PY
```

**(e) Two provenance defects found while wiring this, both fixed in `~/workspace/r2dreamer`.**
`train.py::_write_ladder_provenance` called `full_env.ladder_provenance(ladder)` with no
`far_release`, so a `nested_ramp` run launched with `FAR_RELEASE=1` wrote `far_release: false`
into its own `ladder_provenance.json` and printed a `far_release=off` `[ladder]` line — the
same "the artefact does not record the objective" failure D6 exists to end, one level down.
It now reads both `far_release` and `tip_guard` from the run's own config and REFUSES to start
if `tip_guard` is unset.

**(f) NOT FIXED, reported: `eval_genesis.py` never passes `far_release` to the adapter.**
`GenesisPick(...)` there is built without it, so every {r2dreamer} Ladder-N cell is scored
with `far_release=False` whatever the run trained with. It is one argument, in the same call I
edited for `tip_guard`, and I deliberately left it: it changes the SCORE of registered cells,
which is the coordinator's call and not this lane's. If the (aa) batch's Ladder-N arms are
scored with `eval_genesis.py` as it stands, their `farside` / `slide_event` / `home` cells
will not be the predicate their training used.

**(g) Two evaluator COPIES still take the class default.**
`baselines/eval_e2e_annot.py` now reads the guard from the checkpoint sidecar like
`eval_e2e.py`. `baselines/eval_e2e_stagerec.py` does NOT — it passes neither ladder nor guard
and runs the defaults. That is harmless for what it produces (a stage record is
guard-independent: nothing terminates and the record holds both guards' inputs) but its own
`[ladder]` line will say `grip`. Left as is, and said here rather than left to be discovered.

## 8. What was NOT done

* **The r2dreamer changes are syntax-checked, not executed.** This box has no r2dreamer
  environment (`import envs.genesis` fails on `tensordict`), and the VPN is down so nothing
  could be smoke-tested on the cluster. Every edit there is small and of the same shape as its
  neighbour, but **the first cluster smoke must confirm that `env.tip_guard=<g>` reaches
  `FullTaskEnv` and appears in the `[ladder]` stamp** before any (aa) job is submitted. The
  stamp line is the check: it must read `tip=tilt>60deg&not_in_hand@4f`.
* **No policy was trained or evaluated under the new guard.** Everything here is
  demonstrations and unit tests. Amendment (aa)'s P-aa-6 (the control arm's `tipped` rate
  within ±0.05 of the pilot's staged arm) is the registered way to settle what the guard does
  to a LEARNER, and it needs the control runs.
* **No demo set of record was built.** The sets under
  `/tmp/.../scratchpad/{dHfull_all,dDPfull_first}_{rz,rzh,rnrf,rnrfh}` are development
  artefacts on 32-core AVX2. The `_rzh` / `_rnrfh` sets of record must be built on the
  64-core class with `cluster/relabel_e2e_sets.sbatch TIP_GUARD=not_in_hand`.

