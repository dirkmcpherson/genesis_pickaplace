# Do the policies slide? {RLPD} rollouts re-scored under the calibrated predicate (Lane 11, 2026-09-11)

**The answer.** No. Across **600 sampled {RLPD} rollouts** on four checkpoints, the calibrated
`home` rung fires **4 times**, and all four belong to `dH_s901` — the 250k-decision long-run
human checkpoint, not the pilot. The two **pilot** checkpoints (`dH_s940`, the staged 100k
final, and `dH_s941_c040` at 40k) produce **0 `home` and 0 `slide_event` in 300 episodes**,
while the old `slide_success` rung fires 3 times on them. Lane 9's suspicion is confirmed and
sharpened: of the **32** old `slide_success` positives in this batch, only **4 survive** as
calibrated slides; the other **28** fail because the can travelled **0.0 mm** goalward from a
far-side tool pose after it had come to rest (27 of the 28 are exactly zero, the largest is
3.3 mm against a 10 mm threshold), while a median-0 / p90-62 mm / max-123 mm of goalward drift
that the old `pushed` accumulator *did* count was drift during the set-down, excluded by the
latch. The predicate change is not cosmetic: **8** of those 28 never put the tool on the far
side of the can at all. Separately, the calibrated predicate finds **21 slides the old rung
missed entirely** — pushes from the far side that never arrived — **15 of them from the machine
arm `dDPfirst_s920`**, which sets the can down and pushes it a median 21.6 mm goalward in 15 of
150 episodes and reaches `nested_v2` **zero** times. So the behaviour the paper is about does
occur in these policies, at a rate of 25 in 600 and almost never with arrival — and the old rung
was measuring something else.

---

## 0. What was run, and what could not be

**Learner: {RLPD} (SB3 SAC `.zip`), scope e2e (full task), action mode SAMPLED, `--ladder staged`,
`gc_kp4_riser3_shelf6`, `--max-steps 1200` (300 decisions at repeat 4), `--threads 2`, one
Genesis world per process (the shared-process protocol of PHASE_RESULTS §5.1), CPU only.**

The brief named four pilot checkpoints. **Two of them do not exist on this box and the cluster is
unreachable (VPN down), so they were not scored.** What was on disk, and what each one actually is:

| tag | file | arm | trained under | budget | pilot? |
|---|---|---|---|---|---|
| `dH_s940` | `rlpd_final.zip` | human `dHfull_all_rz` | sidecar says `ladder: staged`, `max_return 8.0` | 100k decisions, FINAL | **yes** |
| `dH_s941_c040` | `rlpd_ckpt.zip` | human `dHfull_all_rz` | sidecar says `ladder: staged` | **40k** decisions (`ckpt_frac 0.4`) — the only s941 artefact present | **yes**, but not the final |
| `dH_s901` | `rlpd_final.zip` | human `dHfull_all_rx` | sidecar records **no** ladder (predates the argument) — the OLD gp_e2e ladder | 250k decisions, FINAL | no (long run) |
| `dDPfirst_s920` | `rlpd_final.zip` | **machine** `dDPfull_first_rx` | sidecar records **no** ladder — the OLD ladder | 250k decisions, FINAL | no (long run) |

- `dDPfirst_s960` and `dDPfirst_s961` (the brief's machine pilot seeds) **were never fetched to
  this box**; `dDPfirst_s920` is the only machine checkpoint here and is included so the batch is
  not single-arm. It is a long-run checkpoint, not a pilot one.
- `dH_s941`'s FINAL checkpoint is not on this box either; `ckpt_040` is.
- For `dH_s901` / `dDPfirst_s920`, `--ladder staged` is a **scoring** ladder, not the training
  objective. The evaluator says so on stdout (`ladder 'staged' from the fallback (sidecar records
  none)`), and it is the same choice Lane 9 made, so the two batches are comparable.

**Episode budget** — seeds vary the ACTION SAMPLING, not the starts, so these are draws, not
independent starts:

| checkpoint | `rnd` (30 starts) | `hold` (15 starts) | total |
|---|---|---|---|
| each of the four | seeds 0,1,2 → **90** | seeds 0,1,2,3 → **60** | **150** |
| | | | **600 episodes** |

**Hardware.** `pop-os`, **AMD Ryzen 9 5950X, 16 physical cores / 32 logical threads, AVX2**.
This is **not** the cluster's 64-core class of record. Genesis is not bit-identical across CPU
classes, so every number here is a development artefact: adequate for deciding whether the
policies slide, not a cell for the paper. Every episode and every record stamps its node, core
count and ISA.

**Ladder stamp on all 28 cells** (identical, checked by the table builder's own rule):

```
unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4
| max_return=8 | terminal=slide_success+tipped | shaping=off | far_release=off
| full_env=23447a5dbbb4 genesis_can_env=40544bf73c8c stage_predicates=1e4912119bf2
| git=known-good-2026-08-27-860-g2272659-dirty
```

**Commands.**

```bash
PY=~/workspace/genesis_pickaplace/.venv-eval/bin/python \
OUT=<scratch>/lane11/roll REC=<scratch>/lane11/rec CKPTS=<scratch>/ckpt PAR=8 THREADS=2 \
  bash baselines/diagnostics/pilot_rescore_rollouts.sh          # 600 episodes, 28 cells, ~35 min

$PY baselines/diagnostics/pilot_rescore.py --records <scratch>/lane11/rec \
    --out <scratch>/lane11/rescore.json                          # four-ladder offline score, ~7 min

$PY baselines/diagnostics/pilot_repro_check.py \
    --a <scratch>/slide_smoke --b <scratch>/lane11/roll --label-a Lane9 --label-b Lane11
```

---

## 1. The code: `--records-out`

`baselines/eval_e2e.py` (and `baselines/eval_e2e_annot.py`) now take `--records-out <dir>` and
write **one per-env-frame stage record per episode**, in the format
`baselines/rl/relabel_reward.py --records-out` writes for demonstration tapes. Lane 7's
`FrameRecorder`, `grip_phys_from_action` and `_cpu_stamp` are **imported, not copied**, so there
is one format and one reader: `relabel_reward.offline_episode` scores a policy episode and a
demonstration tape by the same code path. Cost measured on this batch: **600 records, 36.6 MB,
mean 59 KB / median 74 KB / max 89 KB per episode** (the brief's ~50 KB estimate was close).

**Off, it changes nothing — proven, not asserted.** The same checkpoint / start set / seed /
limit was run twice, once with `--records-out` and once without, and the two `metrics.json`
compared field for field after stripping only what is volatile by construction (wall clock, pid,
and the record path itself):

```
top-level keys compared : 52 (volatile stripped: ['node', 'pids', 'records_out', 'seconds'])
per-episode keys stripped: ['pid', 'record', 'seconds']
RESULT: IDENTICAL -- every remaining field matches, including all per-episode stages.
```

The `record` key and the `records_out` key are **absent entirely** when the flag is not given, so
a cell produced without it is what it was before the option existed. The project's four test
files still pass (`test_ladder_unified` ALL OK, `test_stage_predicates` 39/39,
`test_terminal_guard` ALL OK, `test_record_demos_contract` ALL OK).

### `--never-terminate`: NOT added, and the brief's premise needs one correction

The brief said to add it only if the record needs post-terminal frames, and that it does not for
re-scoring the ladder that ran. **That is true for the ladder that ran, and false for a ladder
whose terminal comes later** — which is two of the four I was asked to score. Measured rather
than argued: **28 of 600 episodes** ended at `staged`'s own paid terminal (`slide_success`)
without `home`, so under `nested_sparse` / `nested_ramp` their `home` is **unknown, not zero**
(§5b). I did not add the plumbing, for a reason that is not convenience:

- a policy is closed-loop, so a recorded trajectory **cannot be extended offline**; and
- a never-terminate re-roll is **not the same episodes plus more frames**. Suppressing the
  terminal makes the first affected episode consume more action draws, which shifts the RNG
  stream for every later episode in that process. It measures a different sample, not a longer
  version of this one. (Isolating one episode with `--ic-index` does not fix this either: that
  episode then starts from the fresh seed instead of the shared-process RNG state.)

What the censored episodes look like is in §5b, and it bounds the damage: all 28 already satisfy
`nested_v2`, and 27 of them had **exactly 0.0 mm** of credited slide gain at the cut. They were
not on the edge of `home`.

Two other decisions worth recording. The recorder wraps `FullTaskEnv._step_once` **on the
instance** and only reads post-step state, so `full_env` itself is untouched — a recorder needing
a hook inside the env would be a lever the training path could trip over. And a first run of
`pilot_rescore.py` was projected at **134 minutes**: `np.load` on a compressed npz is lazy, and
`offline_episode` indexes `rec['can_pos'][i]` once per frame, re-decompressing the whole array
each time. Materialising the record once first took it to **6.8 minutes — a 20× difference** on
exactly the same arithmetic.

---

## 2. Verification: the offline replay reproduces the live run

Every episode was scored offline under the ladder it ran, and compared with the live run's own
stage columns and reward:

| check | result |
|---|---|
| episodes with a live stamp | **600 of 600** |
| flag disagreements (`picked`, `placed_v2`, `contact_push`, `slide_success`, `nested_v2`, `farside`, `slide_event`, `home`) | **0** |
| reward disagreements (episode return, tolerance 1e-4) | **0** |

This is the same guarantee Lane 7 established for tapes (`--verify-against`, 146/146), now for
policies: the record is not a summary of the rollout, it is a substrate the rollout can be
recomputed from.

---

## 3. Lane 9's 315 episodes reproduce exactly — with one flag change, and it is the predicted one

The seed sets for `dH_s940` / `dH_s941_c040` / `dH_s901` on `rnd` (0,1,2) and `hold` (0,1) are
the ones Lane 9 ran, so 13 cells are directly comparable.

| | Lane 9 | Lane 11 |
|---|---|---|
| episodes compared | 315 | 315 |
| outcome agreement | — | **315 / 315** |
| `slide_success` positives | **23** | **23** |
| `slide_success` agreement | — | **315 / 315** |
| per-flag disagreements over 11 shared stage columns | — | **1** |

The single disagreement is `dH_s901_rnd_s0` **ep19**, `contact_push`: Lane 9 = 1, Lane 11 = 0.
It is **not** noise, and it is not the recorder:

```
Lane9  ep19 ic can_pos=[0.5753, -0.0847, 0.113]  outcome=tipped  steps=26  reward=3.0
       stages: {'placed_v2': 1, 'contact_push': 1}
Lane11 ep19 ic can_pos=[0.5753, -0.0847, 0.113]  outcome=tipped  steps=26  reward=1.0
       stages: {'placed_v2': 1}
```

Same start, same 26 decisions, same `tipped` end — `picked` never granted. This start is one of
the CONFOUNDS-row-82 reset artefacts: it begins inside the shelf footprint, so `placed_v2` is
true at reset with the arm at home. Lane 7's change making `released` require `picked`
(disclosure 6) removes the spurious `released` latch on frame 0, and with it the `contact_push`
that used to follow — **worth −2.0 reward on this episode**. The reset-artefact starts were
measured on this batch rather than taken on report: over the 12 `rnd` cells, episodes with
`placed_v2` and no `picked` occur on **ICs 6, 13, 19 and 26 only** (29 of 360 `rnd` episodes),
confirming CONFOUNDS row 82's list exactly.

**Three things fall out of 315/315 at once**: the mp4 render does not perturb the sim, the
`FrameRecorder` wrapper does not perturb the sim, and Lane 7's env changes (the accountant
refactor, `never_terminate`, `far_release`, the same-frame terminal fix) changed no dynamics —
only the one predicate they were meant to change.

---

## 4. Stage rates, per checkpoint × start set

All flags read off the stage records. {RLPD}, sampled, `--ladder staged`, n as shown.

| checkpoint | set | n | `picked` | `placed_v2` | `contact_push` | `pushed` (old) | `farside` | **`slide_event`** | **`home`** | `nested_v2` | `nested_honest` | `tipped` | `slide_success` (old) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `dH_s940` *(pilot, 100k)* | `hold` | 60 | 22 | 9 | 6 | 3 | 6 | **0** | **0** | 4 | 4 | 13 | 2 |
| `dH_s940` *(pilot, 100k)* | `rnd` | 90 | 17 | 12 | 2 | 3 | 4 | **0** | **0** | 1 | 1 | 30 | 1 |
| `dH_s941_c040` *(pilot, 40k)* | `hold` | 60 | 26 | 0 | 0 | 0 | 0 | **0** | **0** | 0 | 0 | 14 | 0 |
| `dH_s941_c040` *(pilot, 40k)* | `rnd` | 90 | 19 | 6 | 0 | 0 | 0 | **0** | **0** | 0 | 0 | 30 | 0 |
| `dH_s901` *(long run, 250k)* | `hold` | 60 | 55 | 45 | 14 | 37 | 32 | **6** | **2** | 15 | 16 | 17 | 14 |
| `dH_s901` *(long run, 250k)* | `rnd` | 90 | 59 | 55 | 17 | 35 | 34 | **4** | **2** | 16 | 18 | 39 | 15 |
| `dDPfirst_s920` *(long run, 250k, machine)* | `hold` | 60 | 59 | 50 | 0 | 15 | 50 | **12** | **0** | 0 | 0 | 5 | 0 |
| `dDPfirst_s920` *(long run, 250k, machine)* | `rnd` | 90 | 57 | 59 | 0 | 5 | 48 | **3** | **0** | 0 | 1 | 24 | 0 |
| **all** | | **600** | 314 | 236 | 39 | 98 | 174 | **25** | **4** | 36 | 40 | 172 | 32 |

Read three rows of this table together and the picture is not the one either rung alone gives.

- **The pilot checkpoints do not slide and mostly do not place.** `dH_s940` picks in 39 of 150
  and places in 21; `dH_s941_c040` picks in 45 of 150 and places in 6. Neither ever earns
  `slide_event`. At 100k and 40k decisions these are early policies — the comparison with the
  250k `dH_s901` is a **budget** comparison as much as anything else, and nothing here separates
  "the pilot ladder does not induce sliding" from "100k decisions is not enough to pick reliably".
- **`dH_s901` is the only checkpoint that ever reaches `home`**, 4 times in 150 episodes (2.7 %),
  against 29 old `slide_success` positives (19.3 %).
- **The machine arm `dDPfirst_s920` slides 15 times and arrives 0 times.** It places (109 of 150),
  goes far-side (98 of 150), pushes the can a median 21.6 mm goalward from there (max 58.5 mm)
  — and never satisfies `nested_v2`, never earns `contact_push`, and earns `nested_honest` once.
  It is the most *slide-like* policy in the batch by the calibrated predicate and scores zero on
  every arrival rung.

---

## 5. The confusion: old `slide_success` against calibrated `slide_event` / `home`

Over all **600** episodes:

| | count |
|---|---:|
| old `slide_success` positives | **32** |
| calibrated `slide_event` | **25** |
| calibrated `home` | **4** |
| old positives that survive as `slide_event` | **4 / 32** |
| old positives that survive as `home` | **4 / 32** |
| `home` with **no** old positive | 0 |
| `slide_event` with **no** old positive | **21** |

**Why the 28 non-survivors fail.** `settled_after_release` is *not* the discriminator here — all
32 reached it. Two clauses do the work:

| failure mode | episodes |
|---|---:|
| `farside` **never granted** — the tool was never on the opposite side of the can within 10 cm | **8** |
| `farside` granted, but under 10 mm of goalward travel credited from there | **20** |
| credited `slide_gain` on all 28 | max **3.3 mm**; **27 are exactly 0.0 mm** |
| goalward drift the **set-down latch excluded** on those 28 | p50 0.0, p90 **61.7**, max **122.6 mm**; **9 of 28** clear the old 10 mm `pushed` threshold on that drift alone |

So the old rung fired on two distinct non-slides: **8 of the 28** never had the tool behind the
can at all, and for a further **9** the 10 mm of "goalward progress" was the can drifting out of
the opening hand before it came to rest — Lane 9's finding, now with a number on it (up to
12.3 cm of drift on one episode).

**The four survivors**, all `dH_s901`, all `nested_honest` 1:

| cell | episode | slide gain | release dist | `nested_ramp` | `nested_sparse` |
|---|---|---:|---:|---:|---:|
| `dH_s901_hold_s1` | `ep8_uid295` | 41.8 mm | 19.4 cm | 7.84 / 9 | 1 / 1 |
| `dH_s901_hold_s3` | `ep14_uid335` | 23.1 mm | 21.5 cm | 7.46 / 9 | 1 / 1 |
| `dH_s901_rnd_s0` | `ep27_uidrnd` | 13.9 mm | 15.1 cm | 7.28 / 9 | 1 / 1 |
| `dH_s901_rnd_s2` | `ep17_uidrnd` | 25.7 mm | 21.4 cm | 7.51 / 9 | 1 / 1 |

Their mean `nested_ramp` return is **7.52 of 9**, against **7.80** for the 13 human sim-slide
demonstrations (LADDER_N_DEMO_CHECK §5). The gap is the ramp span, not the policy: like the
demonstrations, none of them saturates the registered 0.10 m span — which is the same evidence
for a 0.05 m span that Lane 7 left as an open decision.

**The 21 `slide_event` firings the old rung missed** are slides that did not arrive: 15 from
`dDPfirst_s920` (median gain 21.6 mm) and 6 from `dH_s901`. This is the column `slide_event` was
introduced for, and on policies it is not empty — unlike `home`.

---

## 5b. Right-censoring for `home`

**28 of 600** episodes ended at `staged`'s own paid terminal without `home`, so under the two
nested ladders their `home` is **unknown, not zero** — those ladders would not have stopped the
episode there (25 on `dH_s901`, 3 on `dH_s940`). Bounding it:

- all **28 of 28** already satisfy `nested_v2` — the can was already settled in the goal;
- `farside` was never granted in the recorded frames on **8** of them;
- credited `slide_gain` at the cut was **exactly 0.0 mm on 27 of 28** (max 3.3 mm).

So none of them was near the `home` clause when the record stopped, and the headline count of 4
is a floor that is very unlikely to move. Settling it properly needs a never-terminate rollout,
which is a **different sample** (§1), not a longer version of this one.

---

## 6. What each ladder would have paid these trajectories

Σ episode return, scored offline from the same records. The record ladder is `staged` in every
cell, so the `staged` column is the return the run actually collected.

| checkpoint | set | n | Σ `staged` | Σ `sparse` | Σ `nested_sparse` | Σ `nested_ramp` |
|---|---|---:|---:|---:|---:|---:|
| `dH_s940` | `hold` | 60 | 51.0 | 4.0 | 0.0 | 37.0 |
| `dH_s940` | `rnd` | 90 | 37.0 | 1.0 | 0.0 | 25.1 |
| `dH_s941_c040` | `hold` | 60 | 26.0 | 0.0 | 0.0 | 26.0 |
| `dH_s941_c040` | `rnd` | 90 | 25.0 | 0.0 | 0.0 | 19.0 |
| `dH_s901` | `hold` | 60 | 184.0 | 15.0 | 2.0 | 144.7 |
| `dH_s901` | `rnd` | 90 | 208.0 | 16.0 | 2.0 | 150.2 |
| `dDPfirst_s920` | `hold` | 60 | 109.0 | 0.0 | 0.0 | **165.7** |
| `dDPfirst_s920` | `rnd` | 90 | 116.0 | 0.0 | 0.0 | **155.8** |
| **all 600** | | | **756.0** | **36.0** | **4.0** | **723.6** |
| max single episode | | | 8.00 | 1.00 | 1.00 | 7.84 |

**The reward-design observation this batch forces.** Under `nested_ramp`, the highest-scoring
checkpoint in the batch is **`dDPfirst_s920` — 321.5 over 150 episodes (2.14 of 9 per episode)
— and it never once arrives**. `dH_s901`, the only checkpoint that reaches `home` at all, scores
294.9 (1.97 per episode). The ramp's first three rungs (picked + placed_v2 + farside = 3 of 9)
are exactly what a policy that sets the can down and withdraws collects, and Lane 7 flagged this
in disclosure 7: a withdrawal from a set-down passes through the far-side band and earns
`farside`. On demonstrations that was harmless because the set that earns it also slides. On
**policies it is not harmless**: it makes "place and back off" the highest-return behaviour under
the ramp, ahead of the only arm that ever completes the task. `nested_sparse` has the opposite
property — it pays 4.0 over 600 episodes, all four to genuine slides, and 0.0 to every other
checkpoint including the one that pushes most. It is a correct signal and an almost absent one.

---

## 7. Caveats

1. **Hardware.** 16-core / 32-thread AVX2 desktop, not the 64-core cluster class. Development
   artefacts; regenerate on the class of record before any number here is quoted in the paper.
   (The `--threads 2` pin is stamped on every episode, alongside node, cores and ISA.)
2. **Two of the four pilot checkpoints named in the brief were not scored** — `dDPfirst_s960` and
   `dDPfirst_s961` are not on this box and the VPN is down. The machine arm here is a **long-run**
   checkpoint under the **old** ladder, so no human-vs-machine contrast in this document is a
   pilot contrast, and the cross-checkpoint comparisons mix training budgets (40k / 100k / 250k)
   and ladders. **Do not read §4 as a source comparison.**
3. **`dH_s901` and `dDPfirst_s920` were scored under a ladder they did not train under.** Their
   sidecars record no ladder; the evaluator falls back to `staged` and says so. This is the same
   choice Lane 9 made, so the two batches are comparable, but it is a re-score.
4. **`dH_s941_c040` is a 40k checkpoint, not a final.** Its zeros are the zeros of an early
   policy; they are not evidence about the pilot's final behaviour.
5. **Seeds are draws, not starts.** 90 `rnd` episodes are 30 starts × 3 action-sampling draws and
   60 `hold` episodes are 15 × 4. Treating 600 as 600 independent trials would overstate the
   precision of every rate above.
6. **Shared-process protocol.** Episodes within a cell run in one Genesis world in order, so they
   are not independent (PHASE_RESULTS §5.1, coordinator 2026-09-07). The `_iso` protocol exists
   and was not used; the point of this batch was comparability with Lane 9.
7. **An absent value is not a zero.** Every count is over all 600 episodes, all of which were
   scored; the 28 censored episodes are called out in §5b rather than counted as `home` = 0.
8. **`dH_s901`'s `contact_push` numbers are not comparable with Lane 9's** on reset-artefact
   starts, for the reason in §3. Everything else is.

---

## 8. Artefacts

| what | where |
|---|---|
| `--records-out` | `baselines/eval_e2e.py`, `baselines/eval_e2e_annot.py` |
| rollout driver (28 cells, 600 episodes) | `baselines/diagnostics/pilot_rescore_rollouts.sh` |
| four-ladder offline scorer | `baselines/diagnostics/pilot_rescore.py` |
| batch-vs-batch reproducibility | `baselines/diagnostics/pilot_repro_check.py` |
| the records themselves (45 MB, 600 npz, **not committed**) | `<scratchpad>/lane11/rec/<cell>/ep<N>_uid<U>.npz` |
| the cells (**not committed**) | `<scratchpad>/lane11/roll/<cell>/metrics.json` |
| full per-episode scoring, all four ladders | `<scratchpad>/lane11/rescore.json` |

The scratchpad is session-local and not backed up. The records are the thing worth copying out:
they make every future predicate question on these 600 episodes a seven-minute offline re-score
instead of a re-simulation.
