# Review guide — where the project stands and how to read the null results (2026-09-07)

> **Terminology — "mode" vs "deterministic" (corrected 2026-09-07).** `--mode mode` selects the *mode* of the action distribution rather than sampling it. For RLPD this is genuinely deterministic: the policy returns `tanh(mean)` and repeats exactly. For the world models (r2dreamer, dv3) it is **not** run-to-run deterministic, because the agent samples its stochastic latent inside `act` and there is no per-episode reseed; a cell reproduces exactly only when the whole episode sequence is replayed with the same RNG stream (verified: 0/30 differences on the §5.1 sequence check). No comparison is biased by this — both arms are evaluated identically — but "deterministic" overstates it for world-model cells, and the word is used below in that looser sense.


*Purpose: a single document to review against. Every number below is from a fresh-process evaluation file and is also in the docs of record (`RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md`, `PHASE_RESULTS_2026-09-05.md`, `PHASE_PLAN_2026-09-04.md`, `MORNING_TABLE_2026-09-04.md`, `ROBOMIMIC_LOG_2026-09-06.md`, `DISCRETE_ACTION_REPLAY_2026-09-06.md`, `DV3_DEBUG_2026-09-05.md`, `WM_FIX_LOG_2026-09-03.md`). Where a number is not in my working set I point to the doc instead of restating it. Videos: 16 per-condition reels already sent (random draws across seeds, in-distribution row over out-of-distribution row); matched-start reels (both arms on the same starts, stratified by joint outcome) are being built and sent separately with a joint-outcome index (`REVIEW_REELS_INDEX_2026-09-07.md`).*

## 1. The claim under review

**For learners that interact with the environment (RL from demonstrations, world models), demonstration source — messy in-the-wild human versus clean machine — does not change what is learned, at matched demonstration counts; pure imitation is the source-sensitive learner.** Every experiment below is a test of one piece of that sentence.

## 2. Results of record, with the precision each test had

The statistic everywhere: LAST checkpoint, fresh process, deterministic actions ("mode"; sampled actions reported alongside), per-seed success counts, exact two-sided permutation test. "MDE" = the smallest true difference the test would have detected with 80 % power at α 0.05, computed from the observed per-seed spread; "CI" = 95 % half-width of the observed difference. A null is only as strong as its MDE.

### 2.1 Pick stage (Genesis, corrected world w3, 66 human raw vs 58 machine tapes on the same starts)

| learner | human | machine | Δ | p | n |
|---|---|---|---|---|---|
| World model r2dreamer, rnd30 | 0.617 ± 0.062 | 0.608 ± 0.039 | +0.009 | 0.875 | 8 v 8; MDE 0.078, CI ±0.055 |
| World model, rnd300 retest | 0.602 | 0.618 | −0.017 | 0.546 | 8 v 8 |
| RLPD, rnd30 | 0.600 | 0.517 | +0.083 | 0.485 | 8 v 8 |
| DP (pruned human vs machine), rnd30 | 0.520 | 0.467 | +0.053 | 0.123 | 10 v 10 |

World-model per-seed counts: human [15, 19, 19, 19, 21, 17, 20, 18] vs machine [20, 18, 18, 18, 16, 19, 19, 18] of 30 — a null precise to ≈ 0.08. RLPD and DP per-seed lists are in `CROSS_LEARNER_CONDITIONS_2026-09-03.md` §1–2 and `MORNING_TABLE_2026-09-04.md` §1; the RLPD arm's seed spread is the widest of the three (one dead seed per arm in the all-data variant), so its null is the weakest. Also of record: the world model's in-distribution deficit for human demos (holdv2 p 0.035, all-demo p 0.044) — small, significant, and the only significant source effect in the project; out of distribution it vanishes.

**DP pruned-human "0.80 vs 0.52" resolved (2026-09-07, `DP_PRUNED_GAP_2026-09-07.md`):** the 0.80 is an in-distribution cell (held-out human starts; the pruned-human DP is at 0.913 there now) and 0.52 is the random-start cell. Ten of the 30 random starts are picked 0 of 280 times across every DP seed of both arms because they lie beyond the farthest training can position (x ≥ 0.524 vs ≤ 0.513 in every pruned set); on the other 20 starts DP picks 0.78 (0.78 × 20/30 = 0.52). RLPD reaches 5 and the world model 4 of those ten. Not a regression; a support-extrapolation limit of the imitator. Reporting change to adopt: stratify rnd30 by support (20 in / 10 out) in the cross-learner table.

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

1. **Established at ≈ 0.1 precision or better:** no source effect for the world model at pick (≈ 0.08), place (≈ 0.13), contact (≈ 0.12) and carrycontact (≈ 0.03), and for RLPD at pick, with demonstration counts matched.
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

**Update 2026-09-07 13:30 — the first falsifier below appeared to FIRE for RLPD. IT DID NOT SURVIVE ITS OWN REGISTERED CONTROL; see the 2026-09-08 addendum at the end of this document, which supersedes this paragraph.** robomimic Can, 8 seeds, 50 shared starts, last checkpoint, deterministic: RLPD from mixed-human demos 0.455 vs from SAC-generated demos 0.147, Δ +0.307, p 0.008 (sampled 0.458 vs 0.168, p 0.011). On an independent machine generator RLPD is source-sensitive, in the same direction as imitation (BC-RNN 0.927 vs 0.393). Two confounds are being controlled before this is read as a source effect: demonstration quantity (MH200 ≈ 41k transitions vs MG200s ≈ 16.5k, because SAC rollouts are short) via MGall and an all-successes MG arm (718 tapes, ≈ 59k rows), and the RLPD budget (100k → 300k decisions, the registered extension; the MG arm had 0/8 seeds ≥ 0.5). The world-model and DP arms are still running. Whatever the controls show, the Genesis result now reads as "source-indifferent on THIS task with distilled machine demos", and the paper's framing must be decided on the robomimic outcome.


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

---

## 8. CORRECTIONS AFTER THE ADVERSARIAL REVIEWS (2026-09-07 17:00) — these supersede the statements above where they conflict

Sources: `ADVERSARIAL_REVIEW_statistics_2026-09-07.md` (dd826f6), `..._robomimic_ops_...` (8bc7927), `..._eval_env_...` (87a6dba), `..._data_...` (f6d46e4). Each item is CONFIRMED by the reviewer unless marked.

**Numbers and claims of record that change or are re-labelled**
1. **RLPD pick null precision (§2.1, §3.1):** MDE 0.345, CI ±0.245 (per-seed [23,20,18,20,18,2,24,19] v [18,19,17,19,20,0,20,11], one dead seed per arm). It is the WEAKEST null in the project, not a ≈ 0.1 one.
2. **The significant in-distribution deficit (§2.1)** belongs to the all-data human arm (106 tapes) vs machine, not to the matched raw pair (whose in-distribution cells are p 0.106 / 0.238). And every p < 0.05 Genesis cell is deterministic-only: sampled p 0.139 / 0.118 / 0.467. The sampled-actions statistic was chosen post hoc (user, 2026-09-07 15:20); disclosed; both modes reported. ≈ 60 uncorrected tests; 3 at p < 0.05 is the null expectation.
3. **End-to-end `nested` (§2.7, §5.1) is the TRAINING PROXY for the world model** (sticky contact ∧ grip commanded open ∧ both upright, terminating), not the settled proximity test, because the adapter disables the env's settle; DP/RLPD report the honest settled predicate — the learners' nested columns are different quantities until the world-model cells are re-scored with the honest predicate (queued). The "carry-without-release" reading in §2.7 was BACKWARDS: the proxy requires release, and every world-model nested success commanded the gripper open. The `placed` stage that motivated it uses a stale base-world height band (0.12–0.18 m vs a 0.17 m shelf top in w3) and is unusable in the full scope.
4. **Place cells (§2.4) evaluated bank entries WITH REPLACEMENT:** in the place scope the evaluator does not pin the entry uid, so each episode draws uniformly from the bank (~94 of 148 distinct entries per cell) and silently substitutes any entry that fails to restore; carrycontact pins the uid, which is why the same 5 entries fail there and never in place (the "history-dependent restore" chase of 2026-09-05 is closed). Both arms drew the same way, so the comparison is unbiased but noisier than "148 entries once" implies; the place cells are being re-scored with pinned entries.
5. **Policy-generated banks carry a units bug:** `--dump-entries` stores the policy's grip command in [−1, 1] while the restore reads physical [0, 1]; 35/148 machine-policy place entries restore with the fingers commanded open (should be 5) and all 160 `polE_contact` entries are clipped fully open. Banks are being rebuilt with physical grip; contact-after-release and carrycontact cells will be re-scored.
6. **`contact`'s "gripper behind the can" clause is vacuous** (`ee` is the wrist, 14.5 cm behind the tool; true on 332/332 banked states), and the first `contact_push` draft inherited it (fires on 132/148 held states); corrected to the tool point before its re-score reads out (69/148 with the tool point).
7. **`hold15` is not held-out:** 14 of its 15 starts are training starts of `dHv2raw` (12/15 for dH and dDP; 15/15 for the full-task set). The label "held-out" is false and was pre-registered against on 2026-08-23; numbers unaffected. Read every "hold" cell as in-distribution.
8. **End-to-end arms are matched on tapes but not on content:** Σ reward 118 vs 206, and 3 vs 16 demonstrated nested completions, because `--one-per-ic-best` picks the best of up to 3 attempts on 89 % of machine starts (+66 % reward per kept tape). This is a selection asymmetry in the machine arm's favour on top of the count matching.
9. **Robomimic falsifier:** registered on PH200 v MG200s; re-pointed to MH200 v MG200s after the readout (the user's instruction predates the readout; disclosed). The MG demo actions differ from MH in ways NO queued control removes — 4.6× the mean |action|, 6× the step-to-step roughness, a continuous gripper vs binary — so the registered decision rule would label an action-process effect a "source effect"; an action-statistics control (amendment A4) is required before the word "source" is used. The registered no-demo gate G2b was never run (submitted now). The caveat "MG starts come from SAC's own reset distribution" is WRONG (shared placement distribution, measured); struck. MH200 over-weights the two best operators (68/66/66). The bootstrap CI of the gap is [0.13, 0.48].
10. **Precision of an operational claim:** 16 of the 72 mis-fired robomimic jobs ran for ~2 s before cancellation (no outputs, no contamination); "none started" was wrong.

**Weaknesses now disclosed (S2)**
- MDEs above use all entries; excluding starts both arms always fail raises them (pick 0.078 → 0.101; e2e nested 0.152 → 0.207).
- "Matched-N" matches tapes only, never rows (pick 14 323 v 7 285; contact 978 v 569) and every place run trains from 64 human vs 179 machine entry states; the rule can only cut the machine arm.
- 11 of 13 `holdE_place` entries are starts of segments in the human training set vs 6/13 machine (in-distribution cell only; the contact holdE cell is clean at 2/11 v 1/11).
- `make_phase_banks.py` mixes post-decision flag indices with a state index: banked "pick grant" states are one decision early (can 1.1 cm lower), the place +1 one decision late; consistent across arms.
- The place symmetry control existed only for the superseded 63-demo arm; carrycontact had none (amendment (i) submitted 16:40, registered first).
- Run logs show demonstrations being evicted from the world-model buffer mid-run in some configurations, contradicting "prefill never evicted" — to be quantified per arm.
- The stride-1 re-encoding disagrees with the executed action on 1.26 % of human rows and 0 % of machine rows (the leash bites only the human follower).
- `--requeue` was dead code in the wmfix launchers (fixed for new submissions; no run of record was preempted); the G0 gate in `submit_primary.sh` passes at 4/10 where 4/5 is registered; registration commits trailed submissions by seconds (rule: commit before submit).

**What the reviewers tried to break and could not:** every published p/Δ/MDE reproduced to 3 dp (20 cells); the permutation test is exact and two-sided; `restore_failed` counts as failure symmetrically; `polE_contact` is balanced 80/80 by source; `matched_n.py` is a faithful seeded subsample; contract-v1 invariants hold (0 violations over 6 sets); og4 is absent from every phase/full set (three locks); the 8-v-8 end-to-end pool is budget-clean; repo and cluster env copies differ only by the registered scope patch; horizons are matched; the robomimic RLPD numbers reproduce from the run dirs with a clean bank sha and no preemption; the three robomimic env variants are physically identical; the QOS move lost no flags.

### Addendum, 2026-09-07 22:xx — the robomimic no-demo control (G2b) read out, and it narrows the claim

RLPD with a genuinely empty demonstration half (verified per run: `demo_batch 0`, `demo "none"`, budget reached, same bank sha), 8 seeds, 100k decisions, LAST checkpoint: **0/50 on every seed, both action modes — 0.000**. The registered clause (no-demo RLPD ≤ 0.10) passes.

Consequence for wording: MG200s scores 0.147, which is **above** the demo-free floor, so the SAC-generated demonstrations do help RLPD. Of the two readings that were live after the 13:30 result, only one survives: **"MG helps RLPD less than MH does"**, not "MG is worthless to RLPD". Any draft sentence asserting the latter must be struck. This control was registered in the plan and had never been run until tonight; it was found missing by the robomimic/ops adversarial review.

### Addendum, 2026-09-07 23:40 — four corrections from the overnight work, two of which withdraw statements made earlier the same day

**1. The reproducibility control is PHYSICAL CORE COUNT; the instruction-set question is unresolved, not settled.** My earlier AVX2/AVX-512 attribution is withdrawn, but so is the stronger counter-claim that CPU family is ruled out — both rested on Slurm's CPU-family labels, which this cluster reports incorrectly (a node advertising Broadwell is a Cascade Lake part). Only the processor *counts* are trustworthy, and on those the result is clean: All 53 same-core-count comparisons are bit-identical; all 19 differing pairs have a 36-core machine on exactly one side. An independent instance: a 6-core desktop re-executes a tape recorded elsewhere bit-exactly on 2 of 74, while its own locally-recorded census is bit-exact. The underlying threat is unchanged — hardware is partially confounded with arm in the published 8 v 8 — but the operational rule becomes "the bank, the scorer and the runs must share a physical core count", which is checkable in advance and available on any node. A thread-pinning sweep that would confirm the mechanism was running at the VPN drop; **no re-score should be launched before it reports.**

**2. `contact` does not score the behaviour its name implies, in either population.** With the corrected predicate (tool point rather than wrist, tool required on the far side of the pick-can from the goal), **84–86 % of policy grants are carry-ins** — the can is carried in and parked against the goal, never released. The human demonstrations are contaminated too, at 12 of 26 (46 %). Provisional corrected cells, 8 v 8, unpinned: contact-after-release 0.346 v 0.366 (p 0.308) against 0.593/0.602; carrycontact 0.285 v 0.250 (p 0.546) against 0.807/0.796. Release-scored `slide_success` ≈ 0 in both phases. The registered wrong-side disconfirm branch has FIRED. **The demonstration-source null is unaffected** (both |Δ| < 0.10; arms fail alike to within 0.05). Caveat blocks are now attached to §3 and §4 of `PHASE_RESULTS_2026-09-05.md`.

**3. The learners never acquired the slide at all**, which changes what the human demonstration set is *for*. Humans slide on 15 of 74 tapes (20 %); policies manage 0–2 episodes per ~1200 (~0.1 %). The human tapes are therefore the only source of the behaviour, not a supplement to something the learner partly has — the decisive argument against trimming the Slide human set to the 15 tapes this world reproduces.

**4. dv3 now has a working configuration** (state input + the r2dreamer return clamp; gate passed 0.700/0.700 against a registered 0.5). The claim in `RESULTS` §7 item 9 and `CONFOUNDS` row 44 that no configuration exists is corrected. The point that matters for review is that the clamp diagnosis reproduces on an independent port, so the world-model arm is no longer a single-implementation result. Rung 3 of row 44 — reference-task validation on a standard benchmark — remains OPEN for both ports; "validated implementation" must still not be claimed.

**Terminology.** "Deterministic" is accurate for RLPD (the policy returns the tanh of the mean) but not for the world models, which sample a latent inside the policy call with no per-episode reseed; those cells reproduce as whole sequences under the same RNG stream (verified 0/30 on §5.1). No comparison is biased. The four documents of record now carry this note.


### Addendum, 2026-09-08 — the robomimic source claim is withdrawn by its own registered control

The quantity control read out at 8 seeds and reverses the headline. RLPD from machine demonstrations scores 0.147 on the 200-tape subsample, 0.475 on all 718 successes and **0.610 on all 3,900 rollouts**, against 0.455 from human demonstrations. MG718s is statistically indistinguishable from the human arm (Δ +0.020, p 0.886) and MGall is above it. Within the machine arms, MGall beats MG200s by 0.463 (p 0.0002). **The registered "quantity/coverage, not source" rule fires, so the RLPD ordering is not a demonstration-source effect.**

Three consequences. First, the robomimic leg no longer contradicts the Genesis null — it **agrees** with it on an independent task and an independent generator, which is a cleaner result than the one we thought we had. Second, the Diffusion Policy and BC-RNN separations rest on the same 200-tape draw and are not safe to report as source effects until run on the larger machine arms. Third, the budget confound is settled: tripling the decision budget moves the subsample arm only +0.140 (p 0.115) and does not close the gap.

**What is still open.** The 200-tape draw was not neutral — 89 % of its rows come from the last four SAC checkpoint blocks. So "at matched tape count the human draw wins" may reflect a pathological sample or a real per-demonstration quality difference. A neutral 200-tape draw from the 718 successes separates these for about 8 GPU-hours and should be run before the leg is written up either way.


### Addendum, 2026-09-08 — the hardware axis, stated correctly at last

**The hardware axis is a TWO-CLASS SPLIT, and the two candidate causes cannot be separated on this cluster.** A census of 120 machines settles the shape of it: **all 24 AVX2 machines have exactly 36 cores, and every other size (32, 40, 48, 64, 96) is AVX-512, with zero overlap.** So "the 36-core machines differ" and "the AVX2 machines differ" are literally the same statement here, which is why both accounts fitted the evidence and why neither I nor the agents were being careless in preferring one.

What breaks the tie between them is a different observation: **40-core and 64-core machines are different sizes on the same instruction set, and they agree bit-for-bit.** A rule based on core count per se predicts those should diverge, so the pure per-size account is falsified. What fits every observation is a two-class split — one class of machines behaves differently from all the others — while the mechanism behind the class boundary (instruction set, microarchitecture, memory configuration, or something else co-varying with all three) is **not identifiable on this hardware**. Both of my earlier statements — that it was AVX2 versus AVX-512, and that it was core count — should be read as two descriptions of the same unresolved boundary.

None of this changes what we do: avoid the minority class, and every comparison is internally consistent. Pinning satisfies either account.

**Cost, now measured rather than guessed.** On the pinned class, RLPD runs 27.7 s per episode and Diffusion Policy 425 s — the latter 1.46× faster than the unpinned login-node figure that had made this look expensive. The full pinned re-score is ≈69 CPU-hours for RLPD and ≈352 for Diffusion Policy, and with 55 machines available at that size all 32 runs fit concurrently: **≈8 hours wall, ordinary parallel work rather than a scheduling problem.** One irreducible constraint: a shared-process 60-start Diffusion Policy cell is 6.5 hours serial and cannot be sharded without becoming the isolated protocol, so for that learner the isolated cells are actually faster.
