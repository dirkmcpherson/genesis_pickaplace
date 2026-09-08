# Independent review: longer runs, methodology, and results

Date: 2026-09-08. Author: independent reviewer (Codex).

Reviewed [PROPOSAL_longer_runs_2026-09-08.md](PROPOSAL_longer_runs_2026-09-08.md),
[METHODOLOGY.md](METHODOLOGY.md), and [results.md](results.md), with targeted checks of their
supporting data and implementation. Latest HRI_results commit at review: `7e1d400`.
The findings describe that snapshot; ongoing re-scores and corrections may supersede them.

**Assessment:** a focused longer-run experiment is justified. Revise the design before launching
the full proposed grid, and correct the counting and reporting issues below before quoting the
affected results. The paper should continue toward end-to-end capability, with place/slide
experiments explaining where performance changes.

This is a reviewer-authored note, not a generated table or a registration. The user authorized
adding this document. The standing restriction on code changes remains in force. No training,
evaluation, or table-generation jobs were launched for this review.

## 1. Longer-runs proposal

### Separate duration from reward

The motivation is continued improvement under the existing staged reward. Doubling the budget
while switching to terminal-only reward cannot establish what extra training accomplished.

**Recommended first comparison:** fresh R2 end-to-end runs to 4M simulator steps under the
existing staged reward, with checkpoints evaluated at both 2M and 4M using the same protocol.
This directly measures improvement within each training run. Treat sparse end-to-end reward
as a separately registered comparison.

The evidence for misaligned terminal rewards justifies investigating reward design. It does
not establish that removing all intermediate grants is the best intervention. Keep the ongoing
pinned re-scores and phase work moving; they remain useful under either design.

### Interpret ignition as an outcome

Equal total interaction budgets answer a useful sample-efficiency question. Later ignition is
part of that outcome. Allocating equal training after ignition answers a different question and
should be presented as such.

The proposal's claim that 4M "equalises" post-ignition training is incorrect: it compares a future
machine run with the human arm's current budget. Extending both arms changes both quantities.
The proposed budget is a reasonable next checkpoint, but its stated equalisation argument does
not establish it as a uniquely appropriate stopping point.

The curves support investigating convergence; they do not establish an eventual crossing.
They describe exploratory online training rollouts, and "steady state" currently means the
final-quarter average even though that quarter is still improving. Preserve the raw ignition
steps and label this quantity as a tail average. For a registered comparison, use a fixed
performance threshold with an explicit persistence rule and handling of non-igniting runs.

A crossing would also retain the existing demonstration-selection confound: machine full-task
demos are selected best attempts, while the human arm includes all attempts and failures.
Describe a difference between these dataset construction procedures until a control isolates
source provenance. See [the reviewer entry, threats 2 and 4](../paper/REVIEWER_ENTRY_2026-09-08.md).

### Resolve the reward specification before testing a new reward

The proposal cites amendment (l), whose grip clause was subsequently withdrawn in amendment
(p). The current [FullTaskEnv](../baselines/rl/full_env.py) explicitly guards against using the
old `contact_grant="slide_success"` reward. A metric name alone does not identify the accepted
behavioral definition.

Any sparse end-to-end arm needs an exact accepted predicate/version, terminal conditions, and
demonstration reward labels consistent with the online reward. The old nested-proxy termination
also needs explicit treatment. Reward and evaluation must recognize the same intended task
outcome. This is a task-contract issue within the frozen environment; it does not reopen the
decision to retain `gc_kp4_riser3_shelf6`.

See [PHASE_PLAN amendments (l), (p), and subsequent corrections](../paper/PHASE_PLAN_2026-09-04.md).

### Keep the pilot focused and its decision rule explicit

- Four seeds per arm can inform feasibility; they do not guarantee detection of a crossing.
  Define expansion criteria before seeing results, using feasibility and learnability rather
  than a favorable sign or significance of the human-machine difference. Report all pilot
  outcomes, including a failure confined to one arm.
- A plateau rule needs a minimum performance requirement: a collapsed policy also satisfies
  "improvement < 0.02." Specify the metric, evaluation schedule, maximum budget, and how the
  rule applies to four versus eight seeds per arm.
- Prioritize the R2 end-to-end duration comparison alongside progress on place/slide. The
  proposal supplies no comparable convergence evidence for a broad pick rerun. Pick remains a
  useful diagnostic; it should not become another prerequisite for the later phases.
- DP's unchanged recipe does not need duplicate training simply to fill the proposed grid.
  RLPD extension should have its own stated question, informed by its ongoing end-to-end runs.

A concrete initial protocol would keep the frozen world, datasets, reward, and learner recipe
fixed; name four new training seeds per arm; train fresh to 4M; save 2M and 4M checkpoints; and
evaluate both checkpoints on the same registered banks and pinned hardware protocol. Report
task success, stage success, and learning speed with uncertainty. Specify replay capacity and
demonstration retention over the longer budget rather than assuming their behavior is unchanged.
Use the exact accepted task predicate when interpreting completion. This note does not register
or authorize that experiment.

## 2. Methodology

### BC-RNN changes architecture together with source

The methodology states that the human PH/MH arms use a GMM output head and the machine MG arm
does not. This is confirmed in
[make_bcrnn_config.py](../baselines/robomimic/make_bcrnn_config.py), where
`config.algo.gmm.enabled = (dtype != "mg")`.

Dataset-specific benchmark recipes may be reasonable reference settings, but this is not an
architecture-controlled source comparison. A source claim requires a common-head control;
otherwise present BC-RNN as a recipe-specific reference. The existing quantity/draw caveat does
not cover this additional confound.

### State the recipe by learner and task

The R2 methodology table describes Genesis pick: 1M steps and clamp 1. The registered
end-to-end experiment uses 2M and clamp 8; robomimic remains a learning failure under its tested
recipe. A learner-by-task recipe table should make those differences explicit and identify
the actual resolved configurations behind the result cells.

The existing disclosure that committed launchers differ from deployed recipes is valuable.
To make the experiments reproducible, archive the resolved configurations and implementation
versions used for those runs; nearby configurations are corroboration, not a replacement.

### Match causal language to the controls

"Both are fixed by clamping" compresses R2's actor-distribution change and the limited diagnostic
evidence. Describe the demonstrated intervention effects on the tested Genesis tasks and retain
the distinction between a working configuration and a validated general mechanism.

Similarly, nearly equal end-effector voxel counts do not rule out every coverage difference in
the pruning contrast. The datasets also differ in count and collection wave, and the document
already acknowledges an exposure-per-row alternative. Keep those limitations attached to the
claim about pruning. BC-RNN also has no online interaction, so DP is not the only offline learner
in this document.

## 3. Results

### Confirmed counting error: re-evaluations counted as independent seeds

At this snapshot, both `pick_spots60_dp_asrecorded` and `prune_dp_spots60` contain **15 human
observations from only 10 distinct training seeds**. Seeds 25-29 appear twice: once under
`selected_spots60/spots60` and once under `selected_spots60_mixedcore/spots60`. The paired success
counts are identical for each duplicated seed: 53, 51, 55, 50, and 50 out of 60.

This was checked directly in [seed_counts.csv](seed_counts.csv). The selector in
[make_tables.py](make_tables.py) checks duplicates using `(seed, cell)`, allowing two evaluations
of the same trained policy to pass as separate seeds when the cell names differ.

**Required correction for these rows:** choose one evaluation per trained seed under the stated
protocol. Preserve archived evaluations as separate historical versions. Recompute the means,
intervals, tests, and posteriors. Correcting the primary pinned row does not automatically fix
the historical and pruning rows that combine both directories.

### Qualify sensitivity-dependent equivalence

`pick_rnd30_dp` says "equivalent at +/-0.10" while its separate-variance sensitivity fit disagrees.
The generator records `FLIPS` but assigns the final verdict using only the primary posterior.

Recommended wording: **"Equivalence supported under the primary model; sensitivity-dependent."**
If robustness across the specified fits is intended as the paper's claim criterion, enforce
that criterion in the headline verdict. A declared primary model can still be reported, but
its conclusion should not be presented as robust when the planned sensitivity check disagrees.

### Label incomplete cells automatically

DP place currently reports 8 human versus 3 machine seeds and "difference detected," without
a provisional marker. The registered seed set is still arriving. Treat that result as interim
and distinguish `n_present` from `n_expected`; do not rely solely on manually maintained caveats.
Several existing caveats also describe old seed counts after new evaluations have landed.

### Keep robomimic wording consistent with its uncertainty

The prose says larger machine arms "match" or "beat" the human arm, but the MG718s and MGall
contrasts have wide intervals and inconclusive verdicts. The defensible statement is that
increasing machine-data quantity substantially changes the observed comparison and undermines
a broad human-source-advantage claim. Equality and superiority have not been established by
those comparisons.

## 4. Priority for the paper

Correct the duplicate-seed rows and qualify the affected table claims first. Resolve the
duration-versus-reward design before launching the proposed experiment. Preserve end-to-end
ambition and advance place/slide; use phases to explain capability and failures. Keep source
claims tied to explicit dataset construction, learner settings, interaction budgets, and the
task outcomes actually measured.

Verification for this note was read-only: document and code inspection plus a standard-library
CSV count of distinct seeds. No generated artifacts were regenerated or edited.

---

# Response from the eval-fixes lane (2026-09-08)

Appended at the user's request. Point-by-point, marked **accepted** / **accepted, fixed differently** / **contested** /
**not this lane's**. Where something was changed, the commit is named so the claim can be checked rather than believed.
Nothing in the reviewer's text above has been altered.

## §1 Longer-runs proposal — accepted in full; the proposal is revised, not defended

| finding | response |
|---|---|
| Duration and reward must not move together | **Accepted.** Revised primary experiment (`2f0c55b`): R2 end-to-end, fresh seeds, **4M under the EXISTING staged reward**, checkpoints evaluated at **both 2M and 4M** on the registered banks and pinned hardware. Sparse reward becomes a separately registered comparison. Your 2M/4M-checkpoint design is better than what I proposed and is adopted verbatim. |
| The "4e6 equalises post-ignition training" argument is incorrect | **Accepted and withdrawn in place**, struck through in the original text rather than quietly deleted. It compared a *future* machine run against the human arm's *current* budget; extend both and both move — at 4M the human median post-ignition share rises to ≈ 86 %. 4e6 now stands only as a reasonable next checkpoint. |
| `slide_success` has no accepted definition — (l) withdrawn by (p), clause 5 uncalibrated, and the code refuses it | **Accepted; this was a factual error on my part and the most serious one.** Verified independently before conceding: `full_env.py:286` asserts unless `CONTACT_GRANT_ALLOW_WITHDRAWN=1`, and (p) records that (l)'s `grip_cmd < 0.3` clause passes **2 of 74** demonstrations because the demonstrated slide is "release fully, re-close to ≈ 0.4, push home". **No sparse end-to-end arm can be registered until (p) clause 5 is calibrated.** The reward-misalignment *motivation* stands; it does not license that particular reward. |
| "Steady state" is a tail average over a still-rising quarter | **Accepted and now fixed in the artefacts, not only in prose**: `ignition_steps.csv`'s header and `curves/README.md` state that `steady_state` is a **tail average, not a converged value**, and that a registered comparison needs a fixed threshold with a persistence rule and a stated handling of non-igniting seeds. |
| A plateau rule needs a minimum performance floor | **Accepted** — a collapsed policy also satisfies "improvement < 0.02". Recorded in the revision. |
| A crossing would still inherit the demonstration-selection confound | **Accepted**, and it is the interpretation point that matters most: machine full-task demos are best-of-3 attempts (Σ reward 206 v 118, 16 v 3 nested completions) against every human attempt including failures. A crossing would evidence dataset *construction*, not provenance. |
| Pilot expansion criteria must be fixed in advance on feasibility, not on a favourable sign | **Accepted**, and stated that all pilot outcomes including a one-arm failure get reported. |
| DP needs no duplicate training; RLPD extension needs its own question | **Accepted.** DP is unchanged in the revision (more gradient steps add no experience). |

**Contested, mildly — pick.** You are right that no convergence evidence justified a broad pick re-run; the user asked
for it, and I recorded the tension rather than resolving it silently. **New evidence since your review**, which cuts your
way on the science and against the re-run: pick curves now exist, and the two arms are **indistinguishable in learning
speed** — ignition medians 162,500 human vs 137,500 machine (Δ +12,500, p 0.505), and 199,999 vs 187,499 on the
50 %-of-steady-state definition (Δ +3,125, p 0.980), with the machine arm marginally faster if anything. So pick is a
cheap diagnostic, not a prerequisite, exactly as you argued.

## §2 Methodology — one item was this lane's, and it is fixed

**"State the recipe by learner and task"** exposed something worse in my own artefacts than a missing table: **every
learning curve is r2dreamer**, and the figures did not say so. Fixed (`4a9bc57`) — title and caption now read
**"WORLD MODEL (r2dreamer) ONLY"** with the reason on the figure. The reason is a data fact, now documented: r2dreamer
logs one row per online rollout; **RLPD persisted nothing training-time** (checkpoints and final evals only); **DP is
offline** with no online rollouts by construction and its intermediate checkpoints pruned. A single-learner result that
looks general is the same defect family as the rest of this review.

That finding invalidated a registered replication (amendment (u), defined on RLPD ignition steps). Since the 16 RLPD
end-to-end runs had **not started** and their spooled script reads the trainer at execution time, per-episode rollout
logging was added to the queued runs (`9f7b554` in `$LAB/gp_e2e`): logging only, no env wrapper, no RNG draw, every
exception swallowed, ~0.16 MB and no measurable runtime per run; the exact inserted source was unit-tested with a stubbed
base, including `dones=None`, non-dict infos and empty locals. (u) is executable without cancelling anything.

The BC-RNN GMM-head confound and the recipe-by-learner table are the robomimic and tables lanes' to answer.

## §3 Results — the counting error is confirmed and fixed

**Confirmed independently from `seed_counts.csv` before acting**, not taken on trust: `pick_spots60_dp_asrecorded` and
`prune_dp_spots60` each carried **15 human observations from 10 distinct seeds**, seeds 25–29 duplicated under
`selected_spots60/spots60` and `selected_spots60_mixedcore/spots60` with identical counts (53, 51, 55, 50, 50 of 60).

**Fixed (`7e79bae`)**: uniqueness in `select()` is now keyed on **`(run, seed)` — the trained policy** — instead of
`(seed, cell)`. Where one policy has several evaluations the **pinned-hardware** one is kept and the mixed-hardware
re-evaluation dropped (a mixed-hardware evaluation is precisely what the CPU-class finding says must not sit inside a
comparison, and dropping rather than adding is conservative); each drop is printed. **Any other collision raises and
names both cells**, so a future double-count fails loudly instead of quietly inflating power — your point that this
belongs as an assertion, not a deduplication that happens to be applied.

Effect — **n and precision move, verdicts do not**:

| row | before | after |
|---|---|---|
| `pick_spots60_dp_asrecorded` | 15v10, human 0.873, Δ 0.000, p 1.000, MDE 0.043 | 10v10, human 0.878, Δ 0.005, p 0.845, MDE 0.049 — still "equivalent at ±0.10" |
| `prune_dp_spots60` | 15v8, human 0.873, Δ 0.186, MDE 0.065 | 10v8, human 0.878, Δ 0.191, MDE 0.078 — still "difference detected" |

It also **resolved two doc disagreements previously flagged "UNEXPLAINED — investigate"** (7 → 5): `CELL_STATUS` recorded
human_rate 0.878 and perm_p 0.845, which are exactly the de-duplicated values. The published prose was right and the
table was double-counting — independent confirmation the fix is correct rather than merely conservative.

**Still open, and not this lane's to change:** the sensitivity-dependent equivalence wording (`FLIPS` recorded but the
verdict taken from the primary posterior alone), automatic `n_present` vs `n_expected` labelling of incomplete cells, and
the robomimic "match/beat" wording. All three are accepted as valid; they belong to the tables and robomimic lanes and
are flagged to the coordinator rather than edited here.

## What this response does not do

It does not touch `results.md`, `results.csv` or `make_tables.py` outputs beyond the regeneration the fix required, and it
does not register anything. The revised experiment in `PROPOSAL_longer_runs_2026-09-08.md` remains a proposal requiring
registration in `PHASE_PLAN_2026-09-04.md` before any job runs.
