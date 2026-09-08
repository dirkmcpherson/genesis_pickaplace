# Proposal: longer runs, sparse phase rewards, and the end-to-end reward question (2026-09-08)

> **REVISED after `INDEPENDENT_REVIEW_2026-09-08.md` (Codex).** The reviewer found a factual error and a design error in
> the first draft. Both are conceded and the design below is changed accordingly; the original text is kept underneath the
> revision so the change is auditable.
>
> **Conceded — factual.** The first draft recommended making end-to-end sparse **on `slide_success`**. That predicate as
> defined in amendment (l) was **WITHDRAWN by amendment (p)**: its `grip_cmd < 0.3` clause passes **2 of 74**
> demonstrations, because the demonstrated slide is "release fully, re-close to ≈ 0.4, push the can home". The code
> already refuses it — `full_env.py:286` asserts unless `CONTACT_GRANT_ALLOW_WITHDRAWN=1`, and (p)'s replacement
> predicate has **clause 5 uncalibrated**. So the draft proposed training on a reward with no currently accepted
> definition. **No sparse end-to-end arm can be registered until (p) clause 5 is calibrated and accepted.**
>
> **Conceded — design.** (a) The draft bundled two interventions, longer training *and* a different reward; with both
> changed, neither is attributable. (b) The claim that 4e6 "equalises post-ignition training" is **wrong**: it compared a
> future machine run at 4M against the human arm's *current* 2M budget. Extend both arms and both quantities move — at 4M
> the human median post-ignition share rises to ≈ 86 %, so the gap persists. 4e6 is a reasonable next checkpoint, not a
> principled equalisation point, and I withdraw that argument. (c) "Steady state" in `curves/` is a **tail average over a
> quarter that is itself still improving**; it is a descriptive label, not a converged value, and a registered comparison
> needs a fixed threshold with a persistence rule and a stated handling of non-igniting seeds. (d) A plateau rule needs a
> **minimum performance floor** — a collapsed policy also satisfies "improvement < 0.02". (e) Even a crossing would
> inherit the **demonstration-selection confound** (machine full-task demos are best-of-3 attempts, Σ reward 206 v 118;
> the human arm keeps every attempt including failures), so it would evidence a dataset-construction difference, not
> source provenance, until a control isolates it.
>
> **Contested, mildly.** The reviewer notes no convergence evidence justifies a broad *pick* re-run. I agree on the
> evidence — the user asked for pick alongside end-to-end, and I record the tension rather than resolve it silently: pick
> is cheap and diagnostic, but it should not gate place and slide.
>
> ### Revised primary experiment (replaces §2's grid as the first thing to run)
> **R2 end-to-end, fresh seeds, 4M simulator steps, under the EXISTING staged reward**, with checkpoints saved and
> evaluated at **both 2M and 4M** on the same registered banks and the pinned hardware protocol. That measures what extra
> training buys *within* each run and against its own 2M point, with one variable moved. 4 seeds per arm as a feasibility
> pilot with expansion criteria fixed in advance on feasibility/learnability, not on the sign or significance of the
> human-machine difference; report every pilot outcome including a one-arm failure. Sparse end-to-end is a **separately
> registered** comparison, blocked on (p) clause 5. DP needs no duplicate training (more gradient steps add no
> experience); any RLPD extension needs its own stated question, informed by its in-flight end-to-end runs. Specify
> replay capacity and demonstration retention over the longer budget rather than assuming they are unchanged.

---

## Original draft (superseded where the revision says so)

**Status: proposal, nothing launched.** Written by the eval-fixes lane at the user's request. It does not modify any
generated artefact in this directory. Registration must land in `paper/PHASE_PLAN_2026-09-04.md` **before** any job.

## 1. Why change anything: the current end-to-end comparison is measured mid-flight

From the training logs of the eight seeds per arm behind `PHASE_RESULTS` §5.1 (`curves/`, per-seed, binned on a fixed
step grid). Comparing each seed's final quarter of training against the quarter before it:

| stage | human | machine |
|---|---|---|
| picked | +0.055, 5/8 seeds still improving | **+0.110, 7/8 still improving** |
| contact | +0.099, 5/8 | **+0.127, 7/8** |
| nested_proxy | +0.090, 6/8 | **+0.109, 7/8** |

Neither arm has converged at 2e6, and the machine arm is improving about twice as fast. It also ignites later — median
step to 50 % of its own steady state is 974,996 versus 724,997 for the human arm on `picked` (Δ −243,749, permutation
p = 0.032) and 1,149,996 versus 999,996 on `contact` (p = 0.044). Because the budget is a fixed step count, later
ignition means **less training after ignition**: median 60 % remaining for the machine arm against 73 % for the human
arm, and 31 % for the slowest machine seed.

So "equivalent at 2e6" is a statement about a budget that the two arms consumed differently, taken while the gap is
closing. That is the case for lengthening — not that longer runs are generally nicer.

*(Caveat, stated once and inherited by everything below: these are online training rollouts, and the ignition thresholds
and grid were chosen after seeing the data — exploratory, not a registered test. See `curves/README.md`.)*

## 2. What to run

**Order: end-to-end and pick first, then place and slide.** End-to-end is where the evidence of non-convergence is;
pick is the cheapest scope and the one every later phase depends on.

**4 seeds per arm** for the trial, all three learners — **r2dreamer (R2), RLPD, Diffusion Policy**. Four seeds is enough
to see whether the machine arm's faster climb continues into a crossing; it is *not* enough to publish an equivalence
claim (at n=4 the exact permutation floor is p = 0.029 and the MDE is wide). Treat this as a pilot whose output is a
go/no-go on the full 8-seed re-run, and say so in the registration.

**"Longer" means something different per learner, and that must be explicit:**

| learner | budget now | proposed | note |
|---|---|---|---|
| r2dreamer (R2) | 2e6 sim steps | **4e6** | the arm with the convergence evidence |
| RLPD | 250k decisions (e2e) | **500k** | online, same doubling logic |
| Diffusion Policy | 100k grad steps | 100k (unchanged) unless it shows the same pattern | offline: more gradient steps do not add experience, so this is not the same intervention |

~~4e6 is chosen, not doubled arbitrarily: the slowest machine seed ignited at 1.37M, so 4e6 leaves it 66 % of training
post-ignition, matching the human arm's *current* median of 73 %. It equalises the thing that is currently unequal.~~
**WITHDRAWN (see revision): this compares a future machine run against the human arm's current budget; extending both
arms moves both quantities. 4e6 stands only as a reasonable next checkpoint.**

**Re-run fresh; do not resume.** The checkpoints contain `agent_state_dict`, `optims_state_dict`, `step` and **no replay
buffer**, so resuming would be a warm policy with a cold buffer — a different experiment, not a continuation.

## 3. Reward: phases sparse (already true), end-to-end is the real question

**Phases are already sparse and should stay that way.** `FullTaskEnv(phase_sparse=True)` pays exactly one +1 at that
scope's own grant and terminates; tips terminate with no penalty. Pick likewise pays +1 at the pick grant. Nothing to
change — worth stating so it is not "fixed" by accident.

**End-to-end is currently NOT sparse**, and there is now a specific reason to change it. It trains on the staged ladder
picked 1 / placed 1 / contact 2 / **nested 4**, whose top rung is the **training proxy** — sticky contact ∧ grip
commanded open ∧ both upright. Amendment (j) measured that proxy against the settled predicate on a reproduced §5.1
cell: **0.333 proxy versus 0.133 honest, a 2.5× over-count**; and amendment (l) has since made **`slide_success`** the
statistic of record. The end-to-end runs are therefore optimising a quantity we have shown to be wrong and no longer
score against.

~~**Recommendation: make end-to-end sparse on `slide_success`**, so the reward pays what the paper scores.~~
**WITHDRAWN (see revision): `slide_success` per (l) was withdrawn by (p) and the code refuses it; (p)'s replacement has
an uncalibrated clause 5. The misalignment argument below still stands as motivation for investigating reward design,
but it does not license this particular reward.** With the
clamp set to the reward magnitude, as the clamp finding requires (`return_clamp` = maximum attainable return: 1.0 for a
+1 terminal, 100 for a +100 one).

**The risk, stated honestly:** removing the ladder removes the intermediate signal that currently gets the agent to the
pick at 550–800k steps. Sparse end-to-end has never been run here; it may not ignite within 4e6 even with demo prefill.
Mitigations, in preference order:
1. **Keep the existing 2e6 staged cells as the published comparison** — they are not invalidated by this proposal, and
   the pinned re-score now in flight corrects them for the hardware confound.
2. **Register the sparse arm's failure as a reportable outcome**, not a false start: "the full task is not learnable from
   sparse reward at 4e6 under this recipe" is a legitimate result, provided it is predicted in advance.
3. If a hedge is wanted, run **2 seeds staged alongside 2 sparse** rather than 4 sparse — but that answers neither
   question well, and I would not do it.

## 4. What must be registered before launching

- The per-learner budgets above, in each learner's own units, with the disclosure that they are not commensurable.
- **A plateau criterion**, so the next budget question is settled by data rather than another heuristic: e.g. stop when
  the trailing-quarter improvement is < 0.02 in ≥ 6/8 seeds per arm, reported whether or not it is reached.
- **A prediction for the crossing.** We now know the machine arm climbs faster; if it overtakes at 4e6 that is a source
  effect in the *opposite* direction to any human advantage, and how we read it must be fixed in advance.
- That this is a **4-seed pilot**, with the 8-seed run gated on its outcome.

## 5. Cost

At 5–10 h wall clock per 2e6 world-model run, 4e6 is roughly 12–20 h per run. Per scope: 4 seeds × 2 arms × 3 learners
= 24 runs. End-to-end plus pick ≈ 48 runs; place plus slide a further 48. Diffusion Policy is much cheaper than the two
online learners and RLPD sits between. Order the work so end-to-end and pick land first, as proposed above.

## 6. What this proposal deliberately does not touch

The pinned end-to-end re-score and the phase re-score now in flight. Those correct the *published* numbers for the
entry-bank and hardware defects and are needed regardless of whether any longer run happens.
