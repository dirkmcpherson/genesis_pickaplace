# Review guide — where the project stands and how to read the null results (2026-09-07)

*Purpose: a single document to review against. Every number below is from a fresh-process evaluation file and is also in the docs of record (`RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md`, `PHASE_RESULTS_2026-09-05.md`, `PHASE_PLAN_2026-09-04.md`, `MORNING_TABLE_2026-09-04.md`, `ROBOMIMIC_LOG_2026-09-06.md`, `DISCRETE_ACTION_REPLAY_2026-09-06.md`, `DV3_DEBUG_2026-09-05.md`, `WM_FIX_LOG_2026-09-03.md`). Where a number is not in my working set I point to the doc instead of restating it. Videos: 16 per-condition reels already sent (random draws across seeds, in-distribution row over out-of-distribution row); matched-start reels (both arms on the same starts, stratified by joint outcome) are being built and sent separately with a joint-outcome index (`REVIEW_REELS_INDEX_2026-09-07.md`).*

## 1. The claim under review

**For learners that interact with the environment (RL from demonstrations, world models), demonstration source — messy in-the-wild human versus clean machine — does not change what is learned, at matched demonstration counts; pure imitation is the source-sensitive learner.** Every experiment below is a test of one piece of that sentence.

## 2. Results of record, with the precision each test had

The statistic everywhere: LAST checkpoint, fresh process, deterministic actions ("mode"; sampled actions reported alongside), per-seed success counts, exact two-sided permutation test. "MDE" = the smallest true difference the test would have detected with 80 % power at α 0.05, computed from the observed per-seed spread; "CI" = 95 % half-width of the observed difference. A null is only as strong as its MDE.

### 2.1 Pick stage (Genesis, corrected world w3, 66 human raw vs 58 machine tapes on the same starts)

| learner | human | machine | Δ | p | n |
|---|---|---|---|---|---|
| World model r2dreamer, rnd30 | 0.617 | 0.608 | +0.009 | 0.875 | 8 v 8 |
| World model, rnd300 retest | 0.602 | 0.618 | −0.017 | 0.546 | 8 v 8 |
| RLPD, rnd30 | 0.600 | 0.517 | +0.083 | 0.485 | 8 v 8 |
| DP (pruned human vs machine), rnd30 | 0.520 | 0.467 | +0.053 | 0.123 | 10 v 10 |

Per-seed counts and the MDEs for these cells are in `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §3 and `MORNING_TABLE_2026-09-04.md` §1; the RLPD arm's seed spread is the widest of the three (one dead seed per arm in the all-data variant), so its null is the weakest. Also of record: the world model's in-distribution deficit for human demos (holdv2 p 0.035, all-demo p 0.044) — small, significant, and the only significant source effect in the project; out of distribution it vanishes.

### 2.2 Human failures added (dHv2all = 106 tapes vs raw 66; registered prediction: gain)

World model 0.588 vs 0.617 (p 0.579); RLPD 0.554 vs 0.600 (p 0.567). Prediction not met; no gain, no significant loss. Unmatched counts by design.

### 2.3 Clock control (repeat-1, one decision per simulator step)

| cell | human | machine | Δ | p | MDE | CI |
|---|---|---|---|---|---|---|
| rnd30 MODE, 4 v 4 | 0.642 | 0.650 | −0.008 | 1.000 | 0.128 | ±0.093 |

Every seed learned. The action-repeat hold was not masking an effect larger than ≈ 0.13.

### 2.4 Place phase (entry = pick-grant state; success = placed anywhere on the shelf; 39 vs 39 demos, policy-generated starts)

| cell | human | machine | Δ | p | MDE | CI |
|---|---|---|---|---|---|---|
| polE MODE, 8 v 8 | 0.703 ± 0.091 | 0.647 ± 0.084 | +0.056 | 0.227 | 0.132 | ±0.094 |
| unmatched machine (63 demos) | — | 0.709 | −0.006 | 0.878 | | |
| symmetric bank (machine-policy starts) | 0.681 | 0.720 (63 demos) | −0.039 | 0.374 | | |

Note the two machine numbers: 24 extra demonstrations were worth +0.062 to the machine arm — a demo-COUNT effect of the size the source effect is not.

### 2.5 Contact after release (entry = placed state; 11 vs 11 demos, both below the registered 20-demo floor)

| cell | human | machine | Δ | p | MDE | CI |
|---|---|---|---|---|---|---|
| polE_contact MODE (160 policy-generated placed states), 8 v 8 | 0.593 ± 0.091 | 0.602 ± 0.059 | −0.009 | 0.841 | 0.116 | ±0.082 |
| holdE (11 human placed states, at ceiling) | 1.000 | 0.943 | +0.057 | 0.026 | | |
| machine at its own yield (25 demos) | — | 0.609 | −0.016 | 0.711 | | |

The holdE line is the project's other significant difference: 88/88 vs 83/88 on eleven starts where the can is already touching-distance from the goal. It does not carry to the discriminating bank.

### 2.6 Carrycontact (entry = pick-grant state; contact by any route; 21 vs 21)

| cell | human | machine | Δ | p | MDE | CI |
|---|---|---|---|---|---|---|
| polE MODE, 8 v 8 | 0.807 ± 0.023 | 0.796 ± 0.019 | +0.011 | 0.348 | 0.032 | ±0.023 |

The tightest null in the project: the seeds barely vary, so a true difference above ≈ 0.03 would have shown. Five of 148 entries fail to restore in this scope for both arms (counted as failures, symmetric).

### 2.7 End-to-end from scratch (staged reward; 74 human tapes incl. 10 no-picks vs machine best-per-start 72; 2e6 steps; 8 v 8)

| stage, rnd30 MODE | human | machine | Δ | p | MDE | CI |
|---|---|---|---|---|---|---|
| picked | 0.500 ± 0.159 | 0.537 ± 0.125 | −0.037 | 0.643 | 0.216 | ±0.154 |
| contact | 0.379 ± 0.156 | 0.338 ± 0.161 | +0.042 | 0.639 | 0.239 | ±0.170 |
| nested | 0.163 ± 0.110 | 0.192 ± 0.090 | −0.029 | 0.621 | 0.152 | ±0.108 |

This is the weakest null: seed variance is huge (8–19 picks of 30 within an arm), so only a difference above ≈ 0.2 would have been detected. The wave-1 (4 v 4) pattern "human ahead at every stage, contact +0.117" was seed noise. The policies solve the task by carrying the held can into the goal (the release-based `placed` stage is granted 0–4 times in 240 episodes); the human demonstrations mostly release first.

### 2.8 Second world model (dv3)

Gate 1 (reach proxy) passed 15/15 on all four seeds once the return clamp was ported — the same collapse trigger as r2dreamer, no other lever needed. Gate 2/3 (pick from the same human and machine demos, 2 seeds each) reads out ≈ 19:00 today; three of four runs had ignited by 340k, the fourth (machine s1) had not by 500k.

### 2.9 Positive controls that show the design can detect an effect

- **Imitation is source-sensitive on an independent generator:** robomimic Can, BC-RNN at the last checkpoint on our 50-start bank: single-human 0.92, mixed-skill humans 0.927, SAC-generated 0.393 (3 seeds each). The online-learner matrix on the same arms is built and gated on your go.
- **Imitation is idle-sensitive here:** DP on raw human demos collapses (rnd 0.24–0.35) vs pruned 0.55–0.60 (RESULTS §2); the raw set idles 45 % of the time vs 4 % for the machine set (characterization doc).
- **The world-model fix is not a tuning accident:** the same clamp resurrects both ports.

## 3. What the nulls do and do not establish

1. **Established at ≈ 0.1 precision:** no source effect for the world model at pick, place, contact and carrycontact, and for RLPD at pick, with demonstration counts matched. The carrycontact null is precise to ≈ 0.03.
2. **Established only at ≈ 0.2 precision:** the end-to-end null; the RLPD null is between the two.
3. **Directionally consistent, never significant:** where there is any lean it is toward human demos out of distribution (place +0.056, RLPD +0.083) and toward machine demos in distribution (the WM holdv2/all-demo deficit).
4. **Not established:** anything about images (state observations only), about a second task (Genesis only until robomimic runs), about a generator independent of the human data (the machine demos are a distillation), or about pure imitation on THIS task at matched idle structure (DP was only ever run on pruned human vs machine).

## 4. Threats to validity, ranked by how much they could move the conclusion

1. **Machine demos are distilled from the human demos** (a DP teacher trained on pruned human data, run on the same starts). Mitigation in flight: robomimic MG (SAC rollouts) — the BC-RNN control already shows that generator is different enough to break imitation.
2. **Demonstration count was unmatched in the original phase runs** (place 39 v 63, contact 11 v 25, carrycontact 21 v 36); fixed by the matched reruns, which are now the numbers of record; the unmatched ones are disclosed secondaries. The place case showed count matters (+0.06 for 24 demos).
3. **The world model needed a modification to learn at all** (value-target clamp at the maximum return). Applied identically to both arms; reproduced on dv3; disclosed. Reviewers may still ask whether an unmodified DreamerV3 would show a source effect — the honest answer is that an unmodified one learns nothing here.
4. **Stage definitions.** `placed` = anywhere on the shelf footprint; `nested` does not require release; both registered in advance, both visible in the reels. A reviewer could reasonably want "placed near the goal" — that would be a new registered predicate, not a rescoring.
5. **Entry-bank artefacts.** Five policy-generated entries never restore in the carrycontact scope; a held-can state cannot be reconstructed reliably for ≈ 3 % of entries; counted as failures for both arms; mechanism unresolved (two diagnostic attempts withdrawn, logged).
6. **Simulator noise floor.** A 1 mm initial-condition jitter flips ≈ 7 of 74 replayed demonstrations at the contact stage (CONFOUNDS row 47); evaluation banks use fixed states, but single-seed differences of a few episodes are within this floor.
7. **Protocol choices** that could be second-guessed: LAST checkpoint (not best-of); deterministic actions as the statistic with sampled alongside; rnd30 as the primary OOD set (rnd300 agreed where run); n = 8 seeds (MDE ≈ 0.12–0.24 depending on the cell).
8. **A recorder change on the simulator-fidelity side** (fast-open release for human tapes, CONFOUNDS row 50) exists but is not used in any result here; if adopted later the machine arm needs the same treatment.
9. **The all-data human arm** mixes 40 failure/no-pick tapes into 66 successes; its null is against an unmatched count by design.

## 5. What would change the conclusion

- A robomimic result where RLPD or the world model drop by ≥ 0.15 on MG vs PH (the registered falsifier) — source-indifference would then be a Genesis-specific finding.
- A matched-count end-to-end result at 16 v 16 showing a ≥ 0.15 stage gap (the current MDE is ≈ 0.2).
- An unmodified world model that learns and shows a source effect (no such learner exists at present).
- Evidence in the matched-start reels that the two arms reach the same success rate by different behaviours (e.g. one arm nudging, the other lifting cleanly) — that would not contradict the null on success, but it would change how it should be described.

## 6. Review checklist (what to look at, in order)

1. `PHASE_RESULTS_2026-09-05.md` §2.y, §3, §4, §5.1 — the four matched-count tables and their per-seed lists.
2. The matched-start reels and `REVIEW_REELS_INDEX_2026-09-07.md` — the joint-outcome counts (both / human-only / machine-only / both-fail) per comparison: interchangeable arms give human-only ≈ machine-only.
3. `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §6–§7 — the fix and why it is not a confound.
4. `PHASE_PLAN_2026-09-04.md` — every registration and its verdict, in order, including the ones that were not met (all-data gain; wave-1 end-to-end predictions).
5. `WM_FIX_LOG_2026-09-03.md` 2026-09-05..07 — every slip and withdrawal, dated.
6. `DEMO_CHARACTERIZATION_dDP_vs_dHv2raw_2026-09-04.md` — how the two demo sets actually differ (idle 45 % vs 4 %, speed, nudges, identical approach geometry).
7. `ROBOMIMIC_PLAN_2026-09-05.md` + `ROBOMIMIC_LOG_2026-09-06.md` — the bar-raiser design, its gates passed so far, and the BC-RNN control.
8. `DISCRETE_ACTION_REPLAY_2026-09-06.md` — the joystick-command analysis (ternary code faithful; raw twist a weak label).

## 7. Decisions the review should produce

- Robomimic matrix: go / no-go (72 runs, ≈ 240 GPU-h, launcher gated on `GO=1`).
- Whether to raise end-to-end to 16 v 16 (another ≈ 100 GPU-h) or report it at its current precision.
- Whether DP gets a raw-human vs machine cell on this task (to state the imitation contrast under the same protocol rather than only the pruned contrast).
- Whether the paper's framing is "source-indifferent online learners vs source-sensitive imitation" (recommended) and whether the in-distribution WM deficit is reported as a finding or a footnote.
- Whether dv3, if it passes today, runs the full 8 v 8 pick comparison as a second world model.
