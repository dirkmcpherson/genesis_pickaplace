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
