# Overnight state — MORNING BRIEF (2026-09-08 06:20)

*Everything below this brief is the chronological record, including entries later corrected. Read the brief; dip into the chronology only for provenance. Nothing was published, trained on a wrong target, or lost.*

## 1. Decisions waiting for you (nothing else is blocked)

1. **Which human tapes the Slide phase trains on**: 15 (the simulation completed the slide) or ~30 (you performed one, the sim fell 2–4 cm short on 16 of them). Full argument in `PHASE_PLAN` amendment (q), including the asymmetry that machine demonstrations are sim-achievable by construction. Slide's 32 jobs are held pending this.
2. **Whether to raise end-to-end to 16 v 16** (its detectable-effect threshold is ≈ 0.21, the project's widest).
3. **Robomimic framing** — but wait for the controls (see §3); the day's strongest result may not survive them.

## 2. Results you already knew that CHANGED overnight

- **The published end-to-end `nested` over-counts by 2.5×.** It is the training proxy (0.333 on the checked cell); the honest settled predicate gives 0.133. DP and RLPD report the honest one, so the three-learner table cannot mix them. The cell itself reproduces exactly (0/30 differences), so this is a definition problem, not a measurement error.
- **The "policies carry the can without releasing" reading of §5.1 was wrong.** Measured: 8 of 30 episodes did release onto the shelf. I had corrected this once already on weaker evidence; it is now settled.
- **Hardware class is partially confounded with ARM in the published 8 v 8** — the machine arm is 8/8 AVX-512, the human arm has 2 AVX2 cells. This is the most consequential finding of the night and is why all 64 end-to-end cells are being re-scored on one pinned CPU model (amendment (t)).
- **My Slide success predicate was wrong and is withdrawn** (amendment (p)): it required an open gripper at contact, which passes **2 of 74** demonstrations, because you slide with the gripper half-closed. The reward change that would have matched it was stopped before landing. The phase's *entry* gate had the same defect and is also corrected (amendment (r)).
- **"Deterministic mode" is deterministic only given the RNG stream** — the world model samples its latent inside the policy. No comparison is biased, but the word is wrong in four documents.
- **The robomimic "MG is worthless" reading is dead**: the no-demo control passed at 0.000, so machine demonstrations do help, just less.

## 3. What is new and worth knowing

- **A three-learner robomimic result with the ordering our thesis predicts** — imitators lose most on machine data (BC-RNN 0.53, DP 0.77), online RL least (RLPD 0.31) — **but the quantity control is contradicting the premise**: RLPD on all 3,900 rollouts scores 0.74/0.60 on two seeds, *above* the human arm. If that holds, all three gaps are about the 200-tape subsample, not machine provenance. Decisive controls still landing.
- **dv3, the second world model, works.** Gate passed 0.700/0.700 human vs 0.633/0.633 machine. The clamp fix reproduces on both ports. `RESULTS` §7 item 9 and `CONFOUNDS` row 44 now say the opposite of the truth and need updating.
- **A reproducibility finding worth publishing**: our contact-rich results are bit-reproducible within an instruction-set class (53/53) and can flip across AVX2/AVX-512 over long horizons (agreement to 1e-9 at decision 12, outcome flip by 32). Slurm's own feature labels misreport CPUs, so pinning must read `/proc/cpuinfo`.
- **The carrycontact null survives every predicate** (contact, geometric push, release-based) — a real robustness check — while confirming those policies essentially never release (3 releases in 2368 episodes).
- **The count reconciliation is closed.** 21/26/16/11/7/14/15/30 are one lineage difference plus one predicate change plus one design choice; the 11+7=18 arithmetic was a coincidence that does not survive a lineage change.

## 4. Running / held

Running: robomimic recovery (72 runs, DP arm complete, controls landing), place 32, end-to-end 32, dv3 4-seed comparison, contact re-score 192 cells, in-distribution evaluations. Held deliberately: Slide 32 (predicate), end-to-end re-score 320 cells (launching now that pinning is prepared).

## 5. Two problems found and fixed that could have cost us quietly

A silent data-loss defect (completed episodes discarded, presenting as absent data — 6 cells recovered, none recomputed) and dead seeds being averaged into arm means without flagging (one shifts an arm by ~0.07, which is the concrete reason the RLPD pick null is the weakest at 0.345).

---

# Overnight state, 2026-09-07 ~23:00 → morning of 09-08

*Read `WHAT_EACH_PHASE_TRAINS_ON_2026-09-07.md` first (what Pick/Place/Slide actually optimise), then this. Corrections of record live in `REVIEW_GUIDE_2026-09-07.md` §8 + addendum.*

## Decisions waiting for you

1. **Slide reward.** The phase pays and terminates on bare `contact` (which a policy collects while still gripping) but is scored on release-based `slide_success`. Its 32 training jobs are **held**, and the place agent is registering amendment (o) to make the reward match the score — this completes your "train for contact with the can placed on the shelf and not held" instruction rather than being a new decision. Release when you have seen it and the count reconciliation below.
2. **How many human tapes actually slide — ANSWERED, and it is a design choice, not a count.** Under the corrected predicate the answer is **14** on the census lineage, not 18. Your ~18 was right too: it measures something different. 16 further tapes show the human releasing and pushing correctly while the SIMULATED can stops 2–4 cm short — a systematic control-limited shortfall, not a bad demonstration. So 14 = "the slide succeeded in simulation", ~30 = "the human performed a good slide", and your eyeball count off the real footage sits between. **The decision is which set the phase trains on** (amendment (q), full argument there). My recommendation: train on the 14 and disclose the 16 with their cause — because the machine demonstrations are harvested from policies acting in this simulator and are therefore 100 % sim-achievable by construction, so an unfiltered human set would hand the human arm demonstrations that cannot succeed in the environment both arms are scored in, hurting the imitation learner worst. Filtering has its own cost: it selects for the geometry this world reproduces.
3. Whether to widen end-to-end to 16 v 16 (current minimum detectable effect ≈ 0.21).

## Open problem (nothing is being published against it)

The evaluator does not reproduce one end-to-end cell of record: `dHfull_all_s3` rnd30 episode 0, identical IC/checkpoint/mode/seed/horizon, record = nested in 32 steps, rerun = timeout at 300 with no pick. Two facts now in hand:
- **Our phase evaluations are bit-exact across nodes** — 3488 episodes, 23 cells, zero differences in steps/outcome/contact, aggregate 0.7314 vs 0.7314 (`$W/repro_check.py`). This **contradicts** the earlier "up to 62 % of episodes differ" claim, which the contact_push agent has been asked to trace to its cells and withdraw if it came from its superseded wrist-based run. One of the two must be struck, not softened.
- **Full-scope episodes look order-dependent**: the same start reproduces exactly when run first and diverges when run second — phase scopes cannot do this because each episode teleports a banked entry.

Running now: isolation 3355832 (current×2, (j)+(l)-reverted, history arm, rnd30 ep0) and **sequence check 3355868**, a 30-episode rerun in the record's own order diffed per-episode *and* on aggregate. **That job decides whether `PHASE_RESULTS` §5.1's 8v8 end-to-end numbers stand as published.** The 320-cell re-score stays held until it reads out.

## Results that landed today

- **Robomimic RLPD:** mixed-human 0.455 vs SAC-generated 0.147, p 0.008 — the registered falsifier fired. **No-demo control ran tonight and passed at 0.000**, so the defensible sentence is "MG helps less than MH", not "MG is worthless". Action-statistics arms are queued; note the SAC demos cannot be smoothed at all (0/200 survive re-execution), so that control is a bounded dose test, not a clean separation.
- **Robomimic DP (partial):** mixed-human 41–46/50, SAC-generated 1–8/50. Contrast withheld until three retrains finish.
- **dv3 second world model:** G2 passed — 0.700/0.700 human, 0.633/0.633 machine on random starts, hold 15/15. The clamp fix works on both ports. G3 four-seed comparison lands ~17:00 today. **`RESULTS` §7 item 9 and CONFOUNDS row 44 say dv3 has no working configuration and must be updated.**
- **Slide predicate, measured:** carrycontact policies never release — contact 124/148, geometric push test 27/148, `slide_success` 0/148; slide-phase smokes fail with reason `grip_closed` on every checked episode. Your suspicion about what "contact" was rewarding is confirmed at the mechanism level.

## Running overnight

Place 32 (DP+RLPD) · end-to-end 32 (DP+RLPD, amendment (n)) · robomimic recovery 72 across four batches · dv3 G3 4 · contact/slide re-score 192 cells · in-distribution `spots60` evals (12 held to free CPU quota, resubmit in the morning) · machine-first teachers (the fork's, after their disk-full loss).

**Held deliberately:** 32 Slide runs (reward), 320-cell re-score (reproduction), 12 spots60 evals (quota).

## Infrastructure lessons now enforced

Filesystem hit 100 % and killed every job; 434 GB freed, all finals/selected checkpoints/evals/datasets kept. Launchers now keep only final checkpoints, refuse to start under 100 GB free, and refuse to score a checkpoint whose step ≠ the budget. Registrations are committed before submission. Short CPU jobs must use `--qos=preempt -p preempt` (the standard quota caps at 250 cores).

## Late additions (23:40–00:10) — read these with the sections above

**The Slide predicate I registered was wrong, and it is withdrawn (amendment (p)).** `slide_success` as written in (l) required the gripper commanded open (`< 0.3`) at contact. Scored against the demonstrations it passes **2 of 74**, with **44 failures on the grip clause alone**: the human sets the can down, opens fully, then re-closes to ≈ 0.4 and pushes the can home half-closed (user's own words, "sometimes its easier to push with the gripper closed"; worked example uid 232, in our 11, fails on grip alone). Median last-commanded grip across the 74 is 0.39; only 27 of 74 ever go below 0.3. Amendment (o), which would have made the training reward match that predicate, was **stopped before landing** — it would have trained the arm away from the demonstrated solution. Replacement registered: prior release onto the shelf, plus contact, footprint and tilt, plus a not-clamped clause whose threshold must be **calibrated from the demonstration traces** with a pre-stated acceptance test (must pass 232, must reject carry-in, should land near the ~18 counted by eye — and if it does not, report that rather than tune to it). 32 Slide jobs stay held.

**Two anomalies, now properly separated** (I conflated them earlier; corrected here):
- *Recording-lineage divergence*: `dHfull_w3` and the census recording of the same 74 ICs differ on 24 of 74 action streams and 11 contact flags. This explains why our four counts (26 / 21 / 16 / 11) never reconciled — it is a lineage gap, not a predicate gap. **Whichever lineage the phase bank is built from must be the one that is scored**; nothing is re-cut until the slide session says which.
- *Evaluator reproduction failure*: a replay of ONE checkpoint, record on cluster node pax109, rerun on pax154, same lineage. Not the same mechanism. Today's patches are exonerated (reverted and current are identical at every step; 3488 episodes bit-exact post-freeze). Under test now: the same episode on pax109 and two other nodes. If pax109 reproduces and the others do not, the axis is the node; if pax109 also fails, the record is not reproducible on its own hardware and the cause is unnamed.
- The peer's earlier same-node divergence evidence came from **pre-freeze** cells (trees being edited, disk filling), so node and run-condition are confounded there; only the running comparison separates them.

**Load-bearing job to look for first in the morning: sequence check 3355868** — a 30-episode rerun in the record's own order, diffed per episode and on aggregate. It decides whether `PHASE_RESULTS` §5.1's 8v8 end-to-end numbers stand as published, independently of which explanation wins.

## 00:20 — the robomimic headline may reverse; watch this first alongside the sequence check

The A2 quantity control **MGall** (all 3,900 SAC rollouts *including* the 3,182 failures; 536,522 transitions) is scoring **0.74 and 0.60** on its first two seeds — **above MH200's 0.455**, and far above MG200s' 0.147. Provenance verified per run before recording (arm, demo sha, γ, demo_batch, budget reached, 50 episodes, same bank sha).

Registered prediction P2 said MGall would fall **below** MH200 by ≥ 0.15. The first two seeds go the other way, and the registered A2 decision rule is explicit: *if MG718s or MGall reaches MH200 − 0.10, the effect was quantity/coverage, not source.* If that holds at n = 8, then today's headline — "RLPD learns much worse from SAC-generated demonstrations, Δ +0.307, p 0.008" — describes a property of the **200-tape MG subsample** (16.5k rows, drawn only from late checkpoints), not of machine provenance. The falsifier would not have fired against source at all.

**MG718s becomes the decisive arm**: 718 tapes / 59k rows, i.e. *more* rows than MH200's 41k but the same generator, so it separates quantity from provenance where MGall (13× MH200's data) cannot.

Nothing is computed until all 8 seeds of both arms land. But the sensible expectation for the morning is that the robomimic story is **not** settled and the day's strongest-sounding result is the one most likely to move. That is the control working as designed — it was registered before the result, and it is now contradicting it.

## 01:15 — SOLVED: the end-to-end reproduction failure is NODE sensitivity, not a bug

Read this before the sections above that describe it as open.

**Result.** The same checkpoint, IC, mode, seed and horizon replayed on four cluster nodes: **pax109 — the node that produced the record — returns `nested` in 32 steps, 3/3 repeats. pax001, pax030 and pax154 all return `timeout` at 300 steps.** Each node is self-consistent; the nodes disagree with each other.

**Ruled out, with the evidence:**
- *Our patches* — (j)+(l) reverted vs current: reset state and every action and state identical through 12 decisions, and the same full-episode outcome.
- *Job geometry* (the competing hypothesis): the record ran `-n 8` with sequential evals; the pax109 rerun ran `-n 4`, different geometry, and still reproduced exactly, while pax001/pax030/pax154 under that same 4-core geometry all failed to. Geometry is neither necessary for reproduction nor sufficient for divergence.
- *CPU family*: pax109 and pax154 are both broadwell, 36 cores, same memory — and they disagree; pax030 is sapphirerapids and agrees with pax154.
- *Episode order*: the order-dependence hypothesis is dead — uid254 in-sequence still mismatches, and the artefact that suggested it came from inside the disk-full window.
- *Non-determinism*: the pipeline is deterministic within a node (12/12 decisions identical on repeats; 3488 phase episodes bit-exact).

**Mechanism.** Agreement to 1e-9 at decision 12 with an outcome flip by decision 32 is chaotic amplification of a sub-1e-9 numerical difference over 300 decisions of contact-rich physics. Short phase episodes do not have the horizon to amplify it, which is exactly why 3488 phase episodes reproduced bit-exactly across nodes while long full-scope episodes do not.

**What this means for us.** Not a code fault, not a physics fault, but a real constraint: **long-horizon end-to-end episodes are node-sensitive, short phase episodes are not.** End-to-end cells must record the node that produced them, and any end-to-end comparison should either hold the node fixed or treat node as a variance source. It plausibly contributes to why the end-to-end arm already had the project's widest minimum detectable effect (≈ 0.21) while the phase arms were tight (carrycontact ≈ 0.03). Whether the *cell-level aggregates* move — and therefore whether `PHASE_RESULTS` §5.1's 8v8 numbers stand as published — is answered by sequence check 3355868, still running; individual episodes flipping does not by itself move an 8-seed aggregate, and both arms' seeds were spread across nodes, so this is noise rather than bias.

**Also settled tonight:** the contact_push agent's geometry hypothesis loses to this one — its own data has the same shape (reruns agree with each other across nodes and code versions; the *record* is the outlier because of where it was made).

## 02:10 — CORRECTION to the 01:15 entry: the axis is PHYSICAL CORE COUNT, and there may be a one-line fix

The 01:15 section above says the axis is "the node" and that CPU family is excluded. **Both statements are wrong** and are superseded here. The agent had not verified the original smoke's node and assumed one; with provenance checked, the split is by **physical core count**:

| nodes | cores / arch | ep0 result |
|---|---|---|
| pax109, pax154 | **36**, broadwell | `nested`/32 — reproduces the record (6 runs, both allocations) |
| pax001 | 40, broadwell | `timeout`/300 |
| pax030, pax004 | 64, sapphirerapids | `timeout`/300 |

Consequences: the earlier "pax109 and pax154 are both broadwell and disagree" argument dissolves — they agree, and both are 36-core. The order-dependence artefact is explained too: that record came from a 64-core node and was re-run on a 36-core one. And it fits the contact_push agent's cells, whose records came from a 36-core node while its re-runs used 48-, 64- and 40-core nodes.

**Likely mechanism and a cheap fix.** The Genesis/taichi CPU thread count tracks physical cores, so different machines execute different parallel reduction orders, which changes floating-point summation at the 1e-9 level, which a 300-decision contact-rich episode amplifies into an outcome flip. If that is it, **pinning `TI_NUM_THREADS` makes results hardware-independent** and every evaluator simply exports it. A sweep over {4, 8, 36} on 40- and 64-core nodes versus the 36-core baseline is running now. If a pinned value makes a 64-core node return `nested`/32, this entire class of problem goes away — and note it would also mean the contact_push agent's thread-count intuition was right in substance while its "job allocation geometry" version was wrong.

**Order dependence is a separate, still-live effect** on its own clean evidence (same node, same code, r=1.0 standalone vs r=3.0 as episode 2). A direct measurement is running that dumps the full post-reset state for both conditions and diffs the following 20 decisions, returning one of three verdicts: incomplete reset (naming the first differing field), hidden solver state, or clean.

**And it reaches the new arms.** `eval_e2e.py` builds one env and loops episodes in sequence, structurally the same as the world-model evaluator on this axis, so the 32 DP/RLPD end-to-end cells inherit any leakage — and worse for comparability, the two arms' policies produce different episode lengths, so the residual pattern differs *between arms within the same table*. If the verdict is "incomplete reset", those 32 evaluations are held, the reset is fixed, and they are re-run.

## 03:10 — core-count mechanism CONFIRMED at scale (128 comparisons)

Independent audit by the contact_push agent, joining every re-scored cell to the SLURM node that produced its record and its re-run (`$W/core_audit2.py`, committed 082c55f):

| record vs re-run | comparisons | bit-identical | differing |
|---|---|---|---|
| **same physical core count** | 53 | **53** | **0** |
| different core count | 75 | 56 | 19 |

- **Same core count is always bit-identical** — across different nodes, different CPU families, and different code versions.
- **CPU family is ruled out directly**: broadwell-40c agrees with sapphirerapids-64c, and both disagree with broadwell-36c. Every one of the 19 differing comparisons has a 36-core machine on exactly one side; no two non-36-core machines ever disagreed.
- A core-count change does not *force* divergence (56 of 75 unequal pairs still match) — only episodes near a decision boundary amplify it, which is why the contact scope shows it and carrycontact never does.
- The earlier "four cells on one node, three differing, one identical" anomaly **dissolves**: those cells' records came from two different machines, and same-core identity holds throughout.

Both the node-identity attribution and the job-geometry framing are now formally withdrawn in that agent's doc. Every current cell is **unpinned**; if the `TI_NUM_THREADS` sweep confirms pinning as the fix, the affected sections are regenerated from pinned cells before any number is quoted.

**Worth keeping for the paper's methods section regardless of the fix:** contact-rich physics results here are bit-reproducible given the same thread count, and can flip outcomes across machines with different core counts over long horizons. That is a concrete, quantified reproducibility statement most robot-learning papers cannot make, and we can make it because the failure was chased rather than absorbed into seed variance.

---

# STATE AT 03:30 ON THE REPRODUCTION QUESTION — this supersedes every earlier entry in this file

Two distinct effects, both now explained. Neither is a bug in our environment or our patches.

## Effect 1: hardware class. Real, and NOT fixable by pinning threads.
- **Same physical core count ⇒ bit-identical, always**: 53/53 comparisons, across different nodes, CPU families and code versions.
- Different core count ⇒ 19 of 75 differ, and **every one has a 36-core machine on exactly one side**. CPU family is ruled out directly (broadwell-40c agrees with sapphirerapids-64c; both disagree with broadwell-36c).
- **Thread pinning does NOT fix it** — the hoped-for one-line fix is dead. A 40-core node returns `timeout`/300 at `TI_NUM_THREADS` 4, 8 and 36; a 64-core node likewise at 36; the 36-core node returns `nested`/32 at 4 and 36. So the divergence is arithmetic-level, not thread-count-level.
- **Consequence:** record the node (and its core count) in every cell; hold node class fixed within any comparison. Only episodes near a decision boundary amplify it, which is why long full-scope episodes show it and short phase episodes never did.

## Effect 2: what I called "episode-order dependence" is the POLICY'S RNG, not the environment.
- The reset probe refutes its own headline: post-reset state is **bit-identical in all 21 fields** (`max|diff| = 0.000e+00` on qpos, qvel, both cans' pose/velocity/angular velocity, obs vector, contact counts, grants, delta targets), yet **the action at t = 0 differs by 5.8e-2, before any physics runs**. An identical observation producing a different action is the policy, not the solver.
- Cause: Dreamer samples its posterior latent inside `act`, and the global torch RNG is never re-seeded per episode, so episode *k*'s actions depend on how many decisions preceded it. (Note this applies to deterministic-action cells too: the latent is sampled even when action selection is not.)
- **The environment resets correctly. The "incomplete reset" framing is withdrawn.**
- **Whole-cell re-runs still reproduce** (the RNG stream is identical from process start) — which is exactly why 3488 phase episodes were bit-exact and why episode 0 always reproduced. Only *subset* re-runs, pulling one episode out of a sequence, diverge.
- Fix is cheap (`torch.manual_seed(seed + ep)` per episode) and makes episodes independent, but it changes numbers relative to the records, so it goes in a registered amendment rather than a silent patch.
- `eval_e2e.py` shares the structure, so the 32 DP/RLPD end-to-end cells inherit it as a **reproducibility property, not a correctness bug**, provided each cell is produced as one sequence.

## What is still outstanding
The two sequence checks (pax109 and pax154, 30 episodes each) remain the load-bearing answer for whether `PHASE_RESULTS` §5.1's cell-level aggregates stand as published. Everything else on this question is settled.

## 04:05 — investigation closed on mechanism, and one wording consequence for every world-model number

The evaluator agent has closed the investigation (committed 14e0cc9). Two independent causes, neither of them our patches, and it lists what it withdrew with the evidence that killed each: "episode-order dependence of the env" (it was the policy RNG), "hidden solver state" (all 21 post-reset fields bit-identical), and "not a CPU-family split" (rested on an unverified node assumption). Measured confirmation of cause 2: on an identical observation, a re-seeded action is **bit-identical** while an RNG-advanced action differs by **3.75e-2**.

**Wording consequence worth carrying into the paper.** Because `Dreamer.act` samples the RSSM posterior latent even at `eval=True`, our `--mode mode` cells are **deterministic given the RNG stream**, not unconditionally deterministic. Everything we have published as "deterministic actions" is better described as "greedy action selection over a sampled latent, reproducible when the cell is re-run as a whole sequence from process start". This does not bias any comparison — both arms use the same protocol, seed and ordering, and phase cells reproduce 3488/3488 — but "deterministic" as an unqualified word is wrong in `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md`, `MORNING_TABLE_2026-09-04.md`, `PHASE_RESULTS_2026-09-05.md` and `REVIEW_GUIDE_2026-09-07.md`, and should be corrected there before submission.

**Rules that now follow for end-to-end cells:** re-run whole sequences, never single episodes pulled from a sequence; record the node; hold CPU class fixed within a comparison. A per-episode re-seed would make episodes independent but changes every existing number, so it needs its own registration (amendment (s) already produces isolated cells alongside shared ones, which gets the same information without invalidating anything).

**Still outstanding, unchanged:** the paired 30-episode sequence checks on a 36-core and a 64-core node, which decide whether `PHASE_RESULTS` §5.1's aggregates stand. The 320-cell re-score stays held until then.

## 04:30 — mechanism pinned exactly: AVX2 vs AVX-512 (and a trap in Slurm's labels)

**It is the instruction set, not the core count.** pax109 and pax154 carry the *same* CPU — Xeon E5-2695 v4, Broadwell, **AVX2** — and both reproduce the record bit-for-bit. pax001 (Xeon Gold 6248, Cascade Lake) and pax030 (Xeon Gold 6438M, Sapphire Rapids) are **AVX-512** parts and both diverge, at every thread pinning tested. Different vectorised kernels in the Genesis CPU backend produce different rounding, amplified over a 300-decision contact-rich horizon. Core count correlated with this only because it happened to track the part.

**Operational trap, worth carrying beyond this project: Slurm's feature labels lie.** `pax001` advertises `AvailableFeatures=broadwell` but is a Cascade Lake Gold 6248. So `--constraint=broadwell` does **not** guarantee a reproducing node. Pin by CPU model read from `/proc/cpuinfo`, or by explicit `--nodelist`. (Our GPU launchers constrain on GPU type, not CPU features, so they are unaffected — but any CPU-class pinning we add must not rely on the labels.)

**Scope, which is reassuring:** phase cells are immune to both effects — short episodes, bank-restored starts, 3488/3488 bit-exact — so the place, carrycontact and contact re-scores are safe to run on any node, and that is most of the held 320 cells. Only long full-scope end-to-end cells need "whole sequence, same CPU model".

**For the paper's methods section:** we can state precisely that contact-rich simulation outcomes here are bit-reproducible within an instruction-set class and can flip across AVX2/AVX-512 over long horizons, with the crossover quantified (agreement to 1e-9 at decision 12, outcome flip by decision 32). Few papers in this area can say anything this specific about their own reproducibility.

## 04:50 — carrycontact re-score COMPLETE (16/16 cells), and the null survives every predicate

Reproduces the record exactly (955/955 human and 942/942 machine contact episodes — a clean validation of the re-score pipeline on a phase scope, as expected since phase cells are immune to the AVX effect).

| predicate | human (21 demos) | machine (21) | Δ | p |
|---|---|---|---|---|
| `contact` (published) | 0.807 | 0.796 | +0.011 | 0.348 |
| `contact_push` (geometric far-side test) | 0.285 | 0.250 | +0.035 | 0.55 |
| `slide_success` (release-based) | **1/1184** | **2/1184** | — | — |

Two readings. **The demo-source null holds under all three predicates**, so tightening the definition does not create an effect where there was none — that is a real robustness check on the phase result, not a restatement of it. And **the carrycontact policies essentially never release the can**: 3 successful releases in 2368 episodes. That phase measures carrying a held can into the goal, which is exactly why the definition needed changing, and it explains why its null was the tightest in the project (MDE ≈ 0.03) — both arms were being scored on a near-trivial variant.

## 05:00 — a silent data-loss defect found and fixed, and a ceiling warning on the new in-distribution set

**Defect (fixed, nothing lost).** Finished pick re-evaluations were reporting a cell as MISSING while all of its per-episode results were present and valid: the sweep's parent shell was being killed between finishing the episodes and aggregating them, with no error and no trap line, so computed episodes were being silently discarded. The sweep is now self-healing (re-invokes the aggregation once if the summary is absent; the underlying sweep is resume-safe so nothing is recomputed), and a repair pass recovered **6 cells, with 33 already complete, 0 failures and no episode recomputed**. Worth noting the failure mode: a lost cell presented as *absent data*, not as an error, which is the kind of thing that quietly shrinks an n without anyone noticing.

**Ceiling warning on `spots60`.** The first 9 finished pick re-evaluations sit at **56–59/60 sampled and 58–60/60 deterministic** on the new in-training-distribution set, against 17–21/30 on the random-box set. That validates `spots60` as a genuine in-distribution measure — the gap is large and in the expected direction — but it also means **the in-distribution cell is close to saturated, so it will have little power to separate human from machine demonstrations**, the same problem that made the `holdE` contact bank uninformative. Expect the in-distribution row to be a null by ceiling rather than a null by measurement, and read the out-of-distribution row as the discriminating one. (A known-dead seed reproduces as dead — 0/15 and 4/60 — so the set is not simply generous.)

## 05:20 — dead seeds were being averaged in silently; now flagged, and it explains the RLPD null's weakness

The pick table was including dead seeds in arm means without saying so. They are now detected and labelled explicitly, while remaining in the aggregates because the registered statistic is the LAST checkpoint. The detector independently found exactly the two documented in `CONFOUNDS` — `dHv2raw` s65 and `dDPv2` s55 — which is evidence it keys on the right property rather than on a threshold fitted to the data.

**Why this matters for a result already on the table.** One dead seed moves an arm's random-start mean by about 0.07. The RLPD pick comparison (human 0.600 vs machine 0.517, Δ +0.083, p 0.485) has **exactly one dead seed in each arm**, so the point estimate is roughly unaffected — but the variance is not, and this is the concrete reason that null carries the project's widest minimum detectable effect (0.345, per `REVIEW_GUIDE` §8 item 1). The honest framing for the paper is that the RLPD pick cell is dominated by seed mortality rather than by any demonstration-source effect, and that reporting it as "no difference" without the detectable-effect figure would overstate it.

Also validated in passing: the within-arm sampled-versus-deterministic difference is about one episode in thirty (human −1.00, machine −0.88), so the statistic switch the user requested costs almost nothing in level and does not disturb the comparisons.

# 05:45 — THE ANSWER FOR §5.1, AND THE NIGHT'S MOST CONSEQUENTIAL FINDING

**Good news first: the published end-to-end cell reproduces exactly.** 0/30 episode differences, aggregates identical (picked 19/30, contact 13/30, nested 10/30). Whole-sequence reproduction on a matched CPU model holds.

**But two things about that cell change what it means:**

1. **The published `nested` over-counts by 2.5×.** It is the training proxy (10/30 = 0.333); the honest settled predicate gives **4/30 = 0.133**, with 6 of the 10 proxy-only. DP and RLPD report the honest one, so the three-learner table cannot mix them without correction. Related: `placed_v2` reads 8/30, so **8 episodes did release the can onto the shelf** — this finally settles the "policies carry without releasing" reading in §5.1/§2.7, which was wrong (I corrected it once already on weaker evidence; this measurement closes it).

2. **Hardware class is partially confounded with ARM in the published 8 v 8.** The machine arm is 8/8 AVX-512 sapphirerapids; the human arm has two cells on AVX2 broadwell and one on graniterapids. Given that instruction set demonstrably flips long-horizon outcomes, arm and hardware are not independent in that comparison. This was invisible until cells were mapped to nodes, and it is the single most consequential thing found tonight.

**Remedy, registered as amendment (t) and approved:** re-score all 64 end-to-end cells on one pinned CPU model, pinned by `/proc/cpuinfo` model string or explicit node list (never by Slurm label, which misreports). Phase cells need no pinning. Published §5.1 stands as a record of what was measured, superseded by the pinned re-score for the three-learner table, with both reported and the difference attributed.

## 06:10 — the robomimic DP arm is complete: total separation, and the three-learner picture

DP (LAST checkpoint, sampled, 50-state bank, 8 v 8): mixed-human [41,44,39,46,45,42,43,45] = **0.863** (sd 0.047) versus SAC-generated [6,1,8,2,4,6,6,5] = **0.095** (sd 0.046). Δ **+0.767**, exact permutation **p = 0.00016** — the minimum attainable at this n, i.e. complete separation between the arms.

**All three learners on identical arms:**

| learner | mixed-human | SAC-generated (200 tapes) | Δ |
|---|---|---|---|
| BC-RNN (robomimic's own) | 0.927 | 0.393 | 0.53 |
| Diffusion Policy | 0.863 | 0.095 | **0.77** |
| RLPD (online) | 0.455 | 0.147 | 0.31 |

Two readings, and they pull in opposite directions. **The ordering is what our Genesis framing predicts**: the two pure imitators lose most (0.53, 0.77) and the learner that interacts with the environment loses least (0.31) — source sensitivity decreasing with online interaction. **But the quantity control is already contradicting the premise**: MGall (all 3,900 rollouts, 536k transitions) is scoring 0.74 and 0.60 on its first two seeds, *above* the human arm, which would make all three gaps a property of the 200-tape subsample rather than of machine provenance.

So the honest state is: a large, clean, three-learner effect on the 200-tape arms, whose cause is not yet established, with the decisive evidence (MGall and MG718s at 8 seeds, plus the action-statistics dose pairs) still landing. Do not write the ordering up as a finding until those controls read out — the control was registered before the result precisely so it can overturn it.

---

## VPN DOWN 22:37 EDT (recorded 22:47)

The Tufts tunnel dropped. `ip -br addr` shows only wifi and tailscale; `login.pax.tufts.edu` no longer resolves, because the tunnel supplies that DNS. Last successful cluster access was 22:37.

**Nothing is lost.** Everything already submitted keeps running on the cluster: robomimic recovery and its controls, the place runs, the end-to-end runs, dv3, the contact re-score, the in-distribution evaluations. Slurm does not care that this box went away, and the launchers carry `--requeue`.

**What is blocked until you reconnect:** monitoring, any new submission, and rsync. Concretely that means the pinned end-to-end re-score under amendment (t) had been *prepared* but not launched, and the ISA probe that establishes which nodes are AVX2 was still running when the link dropped.

**On reconnect, in this order:**
1. Re-run the ISA probe if its log is incomplete, since the pinned nodelist must come from `/proc/cpuinfo` and not from Slurm's feature labels, which are wrong on this cluster.
2. Launch the 256 phase cells on any nodes (they are hardware-insensitive) and the 64 end-to-end cells on the verified nodelist.
3. Collect the robomimic controls, which are the results that decide whether the three-learner ordering survives.

The end-to-end agent has been told to stop attempting cluster access and to leave its exact command list under this heading.
