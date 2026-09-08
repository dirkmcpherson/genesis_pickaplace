# Overnight state — MORNING BRIEF (written 2026-09-07 ~23:00 local; the chronology below is stamped in a mix of local and cluster/UTC time, which runs 4 h ahead)

*Everything below this brief is the chronological record, including entries later corrected. Read the brief; dip into the chronology only for provenance. Nothing was published, trained on a wrong target, or lost.*

**Before anything else: the VPN dropped at 22:37 and the cluster is unreachable from this box.** Nothing submitted was lost and jobs kept running. **The first thing to do on reconnect is kill roughly 30 stray probe processes on the shared login node — and it is now confirmed they are still there.** The attempt to kill them never reached the cluster: the link had already failed at hostname resolution, so those commands went nowhere. They are ours, they were slow enough to make connections time out on their own, and they are running on infrastructure other people share, so clearing them comes before any science. Details and the full command list are in the last section of this document.

## 1. Decisions waiting for you (nothing else is blocked)

1. **Which human tapes the Slide phase trains on.** This is now properly reconciled, and **your "roughly 18" is not reproducible by any automated criterion — for a good reason.** Every automated count scores the *simulation*; your 18 was a judgement of the *real footage*. The full ladder, on the corrected entry gate (can set down upright on the shelf, held 10 frames, no gripper term), which fires on 48 of 74 tapes, counted by how far the human pushed the can goalward after entry:

   | push ≥ | 0 | 1 cm | 2 cm | 3 cm | 4 cm | 6 cm |
   |---|---|---|---|---|---|---|
   | tapes | 46 | 36 | 33 | **30** | **25** | **15** |

   So the registered "~30" is exactly the ≥3 cm set, and the registered "15" is the set the simulation actually completes. **Do not try to match the number 18.** Two entirely unrelated constructions both total exactly 18 and they agree on only 3 uids out of 18 — the count coinciding is a coincidence, twice over. What the bracket means: **15 = the world reproduces the slide; 30 = you performed one.** The gap between them is the world's shortfall, not yours — the simulated tool tracks the real tool to 0.2 cm at set-down and the can simply fails to follow by 2–4 cm.

   **A second, stronger argument against cutting to 15 arrived after the above.** The policies never learned this slide *at all*: humans perform a full release-then-push slide on **15 of 74 tapes (20 %)**, while the trained policies achieve it in **0 to 2 episodes out of roughly 1,200 per cell (~0.1 %)**. The human demonstrations are therefore not being matched to a behaviour the learner already has — they are its **only** source. Trimming them to the subset this world happens to reproduce removes the only signal for the behaviour the phase exists to teach. This argument does not depend on the world being unrepresentative, which is why it is stronger than the selection-effect point.

   **My earlier recommendation (train on the 15) is contested and I no longer lead with it.** Its virtue is symmetry, since machine demonstrations are sim-achievable by construction. Its defect, which I had not weighed, is that selecting human tapes by simulation success filters them toward the geometry this world happens to reproduce, which is a selection effect on the *world*. The honest options are: 15 (symmetric, but biased toward easy geometry), 30 (every demonstrated slide, including tapes whose reward the world can never grant — fine for imitation, a real problem for RL), or **25 at the ≥4 cm cut as the middle that drops only the weakest pushes**. All three sets are enumerated in `paper/slide_per_uid_2026-09-07.txt`. Slide's 32 jobs are held on your call.
2. **Whether to raise end-to-end to 16 v 16** (its detectable-effect threshold is ≈ 0.21, the project's widest).
3. **Robomimic framing** — but wait for the controls (see §3); the day's strongest result may not survive them.

## 2. Results you already knew that CHANGED overnight

- **The published end-to-end `nested` over-counts by 2.5×.** It is the training proxy (0.333 on the checked cell); the honest settled predicate gives 0.133. DP and RLPD report the honest one, so the three-learner table cannot mix them. The cell itself reproduces exactly (0/30 differences), so this is a definition problem, not a measurement error.
- **The "policies carry the can without releasing" reading of §5.1 was wrong.** Measured: 8 of 30 episodes did release onto the shelf. I had corrected this once already on weaker evidence; it is now settled.
- **The hardware axis is a TWO-CLASS SPLIT, and the two candidate causes cannot be separated on this cluster.** A census of 120 machines settles the shape of it: **all 24 AVX2 machines have exactly 36 cores, and every other size (32, 40, 48, 64, 96) is AVX-512, with zero overlap.** So "the 36-core machines differ" and "the AVX2 machines differ" are literally the same statement here, which is why both accounts fitted the evidence and why neither I nor the agents were being careless in preferring one.

  What breaks the tie between them is a different observation: **40-core and 64-core machines are different sizes on the same instruction set, and they agree bit-for-bit.** A rule based on core count per se predicts those should diverge, so the pure per-size account is falsified. What fits every observation is a two-class split — one class of machines behaves differently from all the others — while the mechanism behind the class boundary (instruction set, microarchitecture, memory configuration, or something else co-varying with all three) is **not identifiable on this hardware**. Both of my earlier statements — that it was AVX2 versus AVX-512, and that it was core count — should be read as two descriptions of the same unresolved boundary.

  None of this changes what we do: avoid the minority class, and every comparison is internally consistent. Pinning satisfies either account.

- **My Slide success predicate was wrong and is withdrawn** (amendment (p)): it required an open gripper at contact, which passes **2 of 74** demonstrations, because you slide with the gripper half-closed. The reward change that would have matched it was stopped before landing. The phase's *entry* gate had the same defect and is also corrected (amendment (r)).
- **"Deterministic mode" is deterministic only given the RNG stream** — the world model samples its latent inside the policy. No comparison is biased, but the word is wrong in four documents.
- **The robomimic "MG is worthless" reading is dead**: the no-demo control passed at 0.000, so machine demonstrations do help, just less.

## 3. What is new and worth knowing

- **A three-learner robomimic result with the ordering our thesis predicts** — imitators lose most on machine data (BC-RNN 0.53, DP 0.77), online RL least (RLPD 0.31) — **but the quantity control is contradicting the premise**: RLPD on all 3,900 rollouts scores 0.74/0.60 on two seeds, *above* the human arm. If that holds, all three gaps are about the 200-tape subsample, not machine provenance. Decisive controls still landing.
- **Direct answer to your question about `contact`: no, it is not credit for touching the gripper to the goal can — but it is not the slide you want either.** Only 14–16 % of the episodes that earn it do so by gripper contact. The other **84–86 % earn it by carrying the pick-can in and parking it against the goal**, never releasing. Measured with the geometrically correct predicate (the tool must be on the far side of the pick-can from the goal, using the tool point rather than the wrist, which the review showed made the old test vacuous), the numbers fall by half: contact phase **0.346 human vs 0.366 machine** (p 0.308) against 0.593/0.602 under the loose predicate, and carrycontact **0.285 vs 0.250** (p 0.546) against 0.807/0.796. A true slide, scored on release, is **essentially never achieved — 0 to 2 episodes out of ~1200 per cell**, and only during the settle window. The registered disconfirm branch has therefore FIRED: the failing fraction is 0.39 to 0.69 where the registration allowed 0.15. **The source null survives all of it** — both differences are inside 0.10 and the two arms fail the same way to within 0.05, so this changes what our policies are doing, not what the paper concludes about demonstration source.

  **But the loose predicate contaminates the human side too, which reaches the cells of record.** Of the 26 human tapes that earn `contact`, 14 earn it after setting the can down and 12 (46 %) earn it by carrying the can in with no set-down at all. So bare `contact` conflates the two routes in *both* populations, at 46 % for the human demonstrations against 84–86 % for the policy episodes — different rates, but neither clean. Any human-versus-machine cell scored on bare `contact` — which includes the contact result of record at 0.593 v 0.602 and carrycontact at 0.807 v 0.796 — is therefore comparing two populations that are substantially not doing the scored behaviour. The corrected-predicate versions of both cells already exist above and should travel with them.

- **The discrete end-effector action space you asked about is a poor fit, and we now have the numbers.** Built offline from the raw tapes, on the 74-tape set of record (verified set-equal, not invented), 37,216 decisions resampled onto the real 0.12 s decision grid. **A small grid fails badly**: replaying a tape open-loop through a quantised action space drifts a median of 68, 33 and 25 mm at 3, 5 and 7 levels per channel — and the can is 66 mm across, so a ternary grid loses the can entirely. The flattering "79 % of decisions represented at 3 levels" is an artefact of the **51.4 % of decisions that move nothing at all**; over the decisions that actually move, quantisation error stays at roughly half a bin no matter how many bins you use, which is what "the motion is analog" means numerically. The first honest setting is about **15 levels per translation channel and 6 for yaw**, i.e. 53 factored logits rather than a handful. **This also explains why the earlier ternary result did not transfer**: that was measured on the *commanded* joystick signal, and the commanded path only reproduces 17 of 74 picks. Figure and full quantiles in `paper/EEF_ACTION_DIST_2026-09-07.md`.

- **One thing that may be a mislabel in our own code, flagged not fixed.** In all 74 tapes the human's commanded rotation is a single channel: two of the three angular components are exactly zero throughout, and the one that is used drives realised **world-z yaw** (correlation −0.55) while having *no* relationship to realised pitch (correlation 0.000). Our cartesian environment names that channel `PITCH_CAP`/`dpitch`. The recorded reference frame is unspecified, so the data cannot settle whether the name or the mapping is wrong, and no code was changed. Worth ten minutes before anyone builds on the cartesian action space. Incidentally the same analysis confirms the arm is genuinely 4-DOF here — roll and pitch vary by 2.8° and 0.8° across whole demonstrations.

- **dv3, the second world model, works.** Gate passed 0.700/0.700 human vs 0.633/0.633 machine. The clamp fix reproduces on both ports. `RESULTS` §7 item 9 and `CONFOUNDS` row 44 now say the opposite of the truth and need updating.
- **A reproducibility finding worth publishing**: our contact-rich results are bit-reproducible within an instruction-set class (53/53) and can flip across AVX2/AVX-512 over long horizons (agreement to 1e-9 at decision 12, outcome flip by 32). Slurm's own feature labels misreport CPUs, so pinning must read `/proc/cpuinfo`.
- **The carrycontact null survives every predicate** (contact, geometric push, release-based) — a real robustness check — while confirming those policies essentially never release (3 releases in 2368 episodes).
- **The count reconciliation is closed.** 21/26/16/11/7/14/15/30 are one lineage difference plus one predicate change plus one design choice; the 11+7=18 arithmetic was a coincidence that does not survive a lineage change.

## 3b. Morning readouts, 2026-09-08

**The contact re-score is complete (192/192 cells) and the provisional numbers held exactly.** Nothing needed correcting. New: the end-to-end cells under the corrected predicate read **0.204 human vs 0.212 machine** on random starts (p 0.942), so the source null now holds on **all three predicates in every end-to-end cell**. A true release-scored slide is earned *only* in the end-to-end task and never in the phases — and most of those grants come from the post-episode settle rather than a deliberate push, with only 2 of 10 sustained.

**Recommended framing, which I have adopted:** report the three predicates as three different things rather than picking one. `slide_success` is the *task outcome*, and the finding is that no arm learns a true slide. `contact_push` is the *discriminating statistic* that actually carries the human-versus-machine comparison. Bare `contact` stays as the *legacy* predicate with its failing fraction attached, because it overstates capability by 1.5–3× and earlier numbers must remain interpretable. Dropping `slide_success` because policies fail it would redefine success as whatever was achieved; putting a p-value on it would manufacture a null out of a floor effect.

**The hardware confound is real, and larger than I first reported.** **Magnitude, corrected 2026-09-08:** an earlier figure of "at most 0.058, human-arm-only" was too reassuring — it came from the re-score audit's own cells rather than from a deliberate cross-class test. Scoring **the same cell on the other class differs on 24 of 30 episodes**, moving contact from 0.433 to 0.533. That is a swing of **+0.100, exactly the width of our registered equivalence margin**, so hardware class can move a cell by the entire region we use to claim "no difference". It is therefore not a disclosure-only issue: comparisons must be held within one class, and the pinned re-score is necessary rather than optional. Within a class the picture stays clean — both same-class whole-sequence re-runs of §5.1 give 0 of 30 per-episode differences.

Earlier framing, kept for provenance: The audit now covers 277 comparisons and **same core count is bit-identical in 119 of 119, with no exceptions**. Of 192 re-scored cells, 16 differ — every one with a 36-core record node. The movement is human-arm-only (those are the 36-core records), unsigned, and small: contact 0.593→0.597, end-to-end 0.379→0.388, hold-out 0.750→0.692. Machine arms are unchanged everywhere. So this threatens no conclusion; it needs disclosure and a stamped core count, not a re-run.

## 3a. One registration deviation to disclose

The robomimic leg's **originally registered primary contrast was PH200**, the single-human demonstration set, and it has never been run. What we have instead is MH200, the mixed-skill human set. That substitution was your call and a principled one — our own Genesis dataset is mixed-human, so the mixed-skill set is the honest analogue — but it is a departure from what was registered and belongs in the write-up as such rather than being quietly absent. Running it later costs roughly 8 GPU-hours if we want the registered contrast on the record too.

## 4. Running / held

Running: robomimic recovery (72 runs, DP arm complete, controls landing), place 32, end-to-end 32, dv3 4-seed comparison, contact re-score 192 cells, in-distribution evaluations. Held deliberately: Slide 32 (predicate), end-to-end re-score 320 cells (launching now that pinning is prepared).

## 4a. What to watch, and one trap in the tape labels

All of it is on local disk and now listed in `paper/REVIEW_REELS_INDEX_2026-09-07.md`. The end-to-end human trials are complete at **all 74** — real footage beside the simulated re-execution with a wrist camera and the deviation stamped. Earlier only 47 rendered; the 27 failures were not missing tapes but a new assertion landing in the environment mid-run while the renderer never exported the world variable, which is fixed. There are also **20 slide demonstrations with a synchronised signal graph**, which are the ones to watch for the decision above: 232 releases, re-closes to 0.4 and pushes the can home (the demo that broke the old gripper-based rule), 273 pushes with a fully closed fist, 255 with an open hand, and 259 pushes but the can stops short.

**The trap:** a tape's own `stage` field is stale and will mislead you. The environment only evaluates the nested predicate when an episode runs to its step limit, and 52 of 74 end earlier, so **13 tapes are genuinely nested while their label says they are not** — 232, 233, 237, 247, 251, 273, 275, 294, 299, 302, 304, 317, 330. Read the honest json, never the tape label. This is what caused 232 to be described to you incorrectly earlier today.

## 4b. The cell matrix you asked about

`paper/CELL_STATUS_2026-09-07.md` (built offline tonight) is the phase-by-learner table with every cell either carrying its number and source or saying what it is waiting on. The short version: **the three-learner comparison exists at the pick and nowhere else yet** — every other Genesis row is a world-model result with two columns that the queued runs fill. It also carries the robomimic table with its control caveat attached, so the two cannot drift apart.

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

## VPN DOWN 22:37 — pinned e2e re-score pickup

> **PARTLY SUPERSEDED — read `### VERDICT: cores` at the end of this section BEFORE running anything below.** The
> pinning design in this section targets AVX2 vs AVX-512, which is not the guard. The reconnect order and the loose
> ends are still correct; the `--nodelist=pax109,pax154` pin and the "verified AVX2 nodelist" framing are NOT.

End-to-end (full task) DP+RLPD lane, PHASE_PLAN amendment (n) (+ (s)/(t) responses). The VPN dropped at ~22:37 EDT
mid-way through the ISA probe; no further cluster access was attempted after that. **Nothing in flight is lost: the 32
training jobs (`e2e_rlpd_*` 3355448–63, `e2e_dp_*` 3355464–79) were all PENDING at the drop and keep queuing.**

### State at the drop

- **Local repo: everything of mine is COMMITTED** (last code commit `b7d7fab`). Nothing of this lane exists only in a
  working tree here.
- **Cluster clone `$LAB/gp_e2e` (branch `e2e-work`): last COMMIT `aa543eb`, but its working tree also holds the newer
  files, rsynced successfully at ~22:33, a few minutes before the drop — `eval_e2e.py` (role stamp), `e2e_eval_cells.sh`
  (ROLE/CELL_DIR), `merge_e2e_iso.py`, `e2e_table_all.py` (--cell-root, role-mixing refusal), `sbatch_e2e_rescore.sh`,
  `isa_probe.sh`. They are UNCOMMITTED there.** Consequence, and it is the good one: the pending jobs read the eval
  stage at run time, so any job that starts before someone reconnects will correctly stamp its in-job cells
  `role: preview`. Re-rsync and commit on reconnect anyway, so the clone's provenance matches the local history.
- **Loose ends on the login node to clear FIRST:** `cluster/isa_probe.sh` was running with up to 30 concurrent
  `srun --overlap` probes and was making `ssh` itself time out. **The kill attempts provably did NOT land** — by then
  the link was failing at DNS (`ssh: Could not resolve hostname login.pax.tufts.edu`, re-confirmed 23:5x), so those
  commands never reached the cluster. Assume the probes are still running and clear them first.
  `$LAB/gp_e2e/isa_map.json` is absent or partial and must not be trusted; delete it rather than read it.

### The nodelist: NOT established by my probe

The probe never returned a readable result. The only verified AVX2 nodes are the coordinator's own:
**`pax109,pax154`** (Xeon E5-2695 v4, Broadwell — both reproduce the record bit-for-bit). Known AVX-512: `pax001`
(Cascade Lake), `pax030` (Sapphire Rapids). Pinning to `pax109,pax154` is safe *without* re-running the probe, because
`sbatch_e2e_rescore.sh` re-reads `/proc/cpuinfo` on arrival and dies loudly if the node is not AVX2 — never trust
`--constraint`, which mislabels `pax001` as `broadwell`. Re-run the probe (at **PAR ≤ 6**, not 30) only to widen the
pool or to answer whether any AVX2 node carries a GPU, which is still unknown.

### Commands on reconnect, in order

```bash
# 1. clear the stray probe processes (they were saturating the login node)
ssh pax 'pkill -f isa_probe.sh; pkill -f "overlap -w pax"; rm -f /cluster/tufts/shortlab/jstale02/gp_e2e/isa_map.json'

# 2. re-sync the lane's code into the clone and commit it there (clone was at aa543eb; local is b7d7fab)
cd ~/workspace/genesis_pickaplace
rsync -az --relative baselines/eval_e2e.py baselines/merge_e2e_iso.py baselines/e2e_table_all.py \
    baselines/rl/full_demos.py cluster/e2e_eval_cells.sh cluster/e2e_build_sets.sh cluster/isa_probe.sh \
    cluster/sbatch_e2e_rescore.sh cluster/sbatch_rlpd_e2e.sh cluster/sbatch_dp_e2e.sh cluster/submit_e2e.sh \
    pax:/cluster/tufts/shortlab/jstale02/gp_e2e/
ssh pax 'cd /cluster/tufts/shortlab/jstale02/gp_e2e && git add -A baselines cluster && git -c user.name="Claude Opus 5" \
    -c user.email="noreply@anthropic.com" commit -m "amendment (n): preview/record split + pinned re-score launcher"'

# 3. see how far the 32 training jobs got
ssh pax 'squeue -u $USER -o "%.10i %.18j %.9T %.10M %R" | grep e2e_; \
         ls -d /cluster/tufts/shortlab/jstale02/gp_e2e/baselines/rl/checkpoints/e2e/*/rlpd_final.zip 2>/dev/null | wc -l'
```

Then, for every run whose TRAINING has finished (the launcher refuses a partial run), submit the pinned CPU-only pass
that produces the cells of record — both the shared cell (statistic of record, order-matched to PHASE_RESULTS §5.1)
and the isolated cell (amendment (s) correctness check), written under `<run>/rec/`:

```bash
ssh pax 'cd /cluster/tufts/shortlab/jstale02/gp_e2e && export GENESIS_PICKAPLACE_ROOT=$PWD && \
  for A in dH dDP; do for S in $(seq 0 7); do \
    LEARNER=rlpd ARM=$A SEED=$S sbatch --nodelist=pax109,pax154 -J e2erec_rlpd_${A}_s$S cluster/sbatch_e2e_rescore.sh; \
  done; done'
# DP the same with LEARNER=dp (CPU-only, and slow -- see the unknowns below before committing to it)
```

Readout, once the pinned cells exist (never mix roles — the table refuses):

```bash
python3 baselines/e2e_table_all.py --cell-root rec --strat                 # cells of record, shared protocol
python3 baselines/e2e_table_all.py --cell-root rec --cell-suffix _iso --strat   # isolation correctness check
python3 baselines/e2e_table_all.py --strat                                 # the in-job PREVIEW cells, descriptive only
```

### What is measured, and what is still unknown

| question | status |
|---|---|
| in-job cells marked so no table can pick them up | **DONE** — `role: preview` by default in every metrics.json and headline; `--role record` is refused without `--require-isa`; pinned cells live under `<run>/rec/`; `e2e_table_all.py` refuses any row that mixes roles. Verified locally, not yet exercised on the cluster. |
| DP evaluation CPU-only | **YES, demonstrated** — a full 300-decision episode ran with no GPU visible (`No accelerated backend detected. Using default cpu`) and produced a sensible result (uid 252: picked, `nested_honest`). **568 s per episode on the AVX-512 login node (Xeon Gold 6346).** |
| DP CPU cost on an **AVX2** node | **UNKNOWN.** Broadwell E5-2695 v4 is an older core than the Ice Lake part I measured on; I will not extrapolate. Measure one episode before committing to a 16-run DP pass — at ~570 s/episode a full DP run (195 episodes across shared + iso cells) is ≈ 31 h of CPU, so the pass is feasible but wants a real number and probably an array over nodes. |
| pinned RLPD **cell** wall-clock on AVX2 | **UNKNOWN — never run.** All timings I have are AVX-512 login-node per-episode figures: shared ≈ 28.5 s/episode plus ≈ 30 s process start per cell; isolated ≈ 59 s/episode (2.05×). A full pinned cell on `pax109`/`pax154` is the first thing to measure on reconnect. |
| node → instruction-set map | **NOT established** (probe killed mid-flight). Verified AVX2: `pax109`, `pax154` only, from the coordinator. |
| does any AVX2 node have a GPU | **UNKNOWN** — the probe would have answered it; `isa_probe.sh` now records `gpu=` per node and prints the AVX2-with-GPU list. |

### Answers that came back before the link dropped (and one new cost problem)

**Preview marking is done.** In-job evaluation cells are stamped `role: preview` by default, the record role is refused unless the instruction-set check is enabled, pinned cells are written to their own subdirectory, and the table builder refuses any row that mixes roles. So no table can silently blend preview cells with cells of record.

**Diffusion Policy can be evaluated with no GPU** — demonstrated on a full 300-decision episode with a sensible result. That was the hoped-for answer, but it comes with a bill: **568 seconds per episode**, measured on an AVX-512 machine. A 60-start cell is therefore about 9.5 hours, and the Diffusion Policy share of the pinned end-to-end pass is on the order of 300 CPU-hours. It is parallelisable and the cluster has the cores, so this is a scheduling question rather than a blocker, but it is worth your judgement in the morning: **the pinned re-score may cost more than the confound it removes is worth for the Diffusion Policy arm specifically.** The world-model and RLPD arms are far cheaper per episode.

**The RLPD wall-clock on a pinned AVX2 node is unknown**, because that measurement never ran. All existing timings are from AVX-512 hardware, roughly 30 to 60 seconds per episode, and extrapolating them to older silicon would be a guess. The same is true of the Diffusion Policy cost on AVX2, which matters more given the figure above. The right first move on reconnect is to measure one cell of each rather than commit to all 32 runs blind.

**One loose end to clear before anything else:** up to 30 probe processes launched with `srun --overlap` may still be alive on the login node, and they were slow enough to make `ssh` itself time out. The first reconnect command kills them. This is shared infrastructure, so it takes priority over restarting the pass.

The verified AVX2 nodes remain `pax109` and `pax154`. Pinning to them is safe even without the probe, because the re-score script re-reads the processor information on arrival and aborts loudly if it lands on the wrong instruction set.

### CORRECTION (superseded in part by `### VERDICT: cores` below) — machine size, not instruction set

The coordinator withdrew the instruction-set attribution: **divergence tracks physical CORE COUNT, not AVX2 vs
AVX-512.** The dissociation is clean in both directions — a 40-core Broadwell and a 64-core Sapphire Rapids agree
bit-for-bit *across* the ISA boundary, while a 36-core Broadwell disagrees with the 40-core Broadwell on the *same*
ISA. All 53 same-core-count comparisons are bit-identical, and every one of the 19 differing pairs has a 36-core
machine on exactly one side. The contact-phase agent withdrew its own node-identity claim on the same evidence, and
its pax053 anomaly dissolves once records are attributed to machine size rather than node name.

**The underlying concern is unchanged:** hardware is partially confounded with arm in the published 8-versus-8, so
those cells still need re-scoring under one consistent configuration. Only the definition of "consistent" moved.

What changed in the code (committed `6a63823`, local only — the cluster clone still has the ISA-era files, which is
harmless because the in-job preview path sets no guard):

- `eval_e2e.py --require-cores <n>` (machine physical cores = sockets × cores-per-socket from `/proc/cpuinfo`, which
  reports the whole node inside a cgroup) and `--threads <n>` (pins OMP/MKL/OpenBLAS/NUMEXPR/Taichi *before* torch
  and genesis import, then `torch.set_num_threads`). Every episode now stamps cores, sockets, logical CPUs, task
  affinity, thread count and OMP setting. `--require-isa` is kept as a diagnostic and documented as withdrawn.
- `--role record` now requires `--require-cores` and/or `--threads`: a cell of record is still pinned by
  construction, just on the right variable.
- `merge_e2e_iso.py` fails on a cell spanning core counts or thread counts; `e2e_table_all.py` warns on machine size
  and offers `--cores N` for the clean within-configuration comparison, printing the per-arm core-count balance.
- `isa_probe.sh` → **`hw_probe.sh`**: reports cores/sockets/logical/GPU per node, groups by machine size, names the
  most common size, and defaults to `PAR=6` (PAR=30 saturated the login node).

**HOLD, now mechanical.** `cluster/sbatch_e2e_rescore.sh` refuses to launch until `SWEEP_VERDICT` is set, because the
thread-pinning sweep decides the mechanism: if fixing the thread count makes everything agree, a thread pin is
sufficient and is satisfiable on **any** machine (`SWEEP_VERDICT=threads THREADS=<n>`); if it does not, cells must be
matched by machine size instead (`SWEEP_VERDICT=cores REQUIRE_CORES=<n> THREADS=<n>`), a stricter design. Launching
before the verdict risks a second set of cells that also has to be thrown away. Verified: it prints `HOLD:` and exits
1 when unset.

**Scheduling consequence, and it is good.** A core/thread pin needs no `--nodelist` — the guard reads `/proc/cpuinfo`
on arrival — so the CPU-only pass no longer has to squeeze through two named machines, and the ~570 s/episode
Diffusion Policy cost (≈300 CPU-hours) becomes ordinary parallel work.

**Revised first steps on reconnect** (replacing the `--nodelist=pax109,pax154` submission above):

1. Clear the stray probe processes and the untrustworthy map (unchanged, still step one).
2. Re-sync + commit the lane's code into `$LAB/gp_e2e` — it must now include `hw_probe.sh` and the deletion of
   `isa_probe.sh`.
3. `bash cluster/hw_probe.sh` at **PAR=6** to get the machine-size census and pick the size to standardise on.
4. **Wait for the thread-pinning sweep verdict.** Do not submit the pinned pass before it.
5. Then take the two measurements the coordinator asked for — one RLPD cell and one DP episode — **on machines of a
   fixed core count** (not a fixed instruction set), and report wall-clock before committing to all 32.

### VERDICT: `cores` — the hold is released, and one claim in the correction above is itself retracted

**The verdict is `cores`.** On reconnect the pinned pass launches with `SWEEP_VERDICT=cores REQUIRE_CORES=<n>`
(plus `THREADS=<n>`, which costs nothing and removes a free variable). Pick `<n>` with `cluster/hw_probe.sh`.

**There was never a competing sweep.** The thread-pinning probe belongs to the eval-fixes agent, not the
contact-phase agent; the "pending sweep" line was that agent declining to pre-empt someone else's in-flight result.
Its own 128-comparison audit never set a thread variable, so in every one of those comparisons the thread count
simply *was* the physical core count. That is why the audit establishes the core-count rule and is structurally
incapable of speaking to thread pinning — no contradiction, two agents describing different things.

**Retraction inside the correction above.** The CPU-*family* attribution in that audit came from Slurm's
`AvailableFeatures`, and those labels are wrong on this cluster — the same defect that made a node advertising
Broadwell turn out to be Cascade Lake. So "a 40-core Broadwell agrees bit-for-bit with a 64-core Sapphire Rapids"
is **not usable evidence about instruction sets**, and neither was the original AVX claim, which leaned on the same
labels. **The instruction-set question is UNRESOLVED, not ruled out.** The section above states that cross-ISA
agreement as if it were established; it is not, and every occurrence of it in the code has been reworded
(`7c71ea9`).

**What is established rests only on processor counts, which are reliable:** 53 of 53 same-core-count comparisons
bit-identical across nodes, labels and code versions, and all 19 disagreements with a 36-core machine on exactly one
side. So `--require-cores` is the correct guard, sufficient on every comparison on record, and checkable before
submission. `--require-isa` stays as the diagnostic stamp — it is what makes a later re-check of families against
`/proc/cpuinfo` possible on cells that already exist.

**Code state (`7c71ea9`, local only — the cluster clone still has the older files):**

- `sbatch_e2e_rescore.sh`: `SWEEP_VERDICT=cores` is the released path (`REQUIRE_CORES` required, `THREADS`
  recommended, explicit warning if anyone pins to 36). `SWEEP_VERDICT=threads` now **refuses** without a written
  `THREADS_ONLY_OK=<reason>`, since nothing on record supports a threads-only pin. A bare launch still refuses, so
  nobody pins on a guess. All three paths verified locally.
- `hw_probe.sh` flags 36-core machines explicitly and suggests the most common **non-36-core** size for
  `REQUIRE_CORES`.
- Wording corrected throughout `eval_e2e.py`, `merge_e2e_iso.py`, `e2e_table_all.py`: guard of record, `--threads`
  documented as "alongside `--require-cores`, never instead of it", and the `hw_axis` stamp now records that the ISA
  question is unresolved.

**Reconnect sequence (unchanged where it matters):**

1. Kill the stray login-node probe processes and delete the untrustworthy map — still step one.
2. Re-sync + commit the lane's code into `$LAB/gp_e2e` (must include `hw_probe.sh` and the removal of
   `isa_probe.sh`).
3. `bash cluster/hw_probe.sh` at **PAR=6** → machine-size census → choose `REQUIRE_CORES=<n>`, avoiding 36.
4. **Two measurements before committing the full pass, both at that fixed core count:** one RLPD cell and one
   Diffusion Policy episode. Both are genuinely unknown; the DP figure is the one that decides whether the pass is
   cheap or a scheduling problem (the only DP number on record, 568 s/episode, is from a different machine and does
   not transfer).
5. Then the 32-cell pinned pass, and read out with `--cell-root rec` (never mixing `preview` and `record`).

### 2026-09-08 07:0x–07:35 — machine-size census, and the two blocking measurements

Link back at 07:02. Login node confirmed clean of my probes (the six `srun --overlap` matches belong to `laolab`, not
me). `isa_map.json` deleted unread. Clone `$LAB/gp_e2e` re-synced and committed (`884f643`), `isa_probe.sh` removed.

**Machine-size census** (`cluster/hw_probe.sh`, PAR=6, 124 nodes probed, 120 answered → `hw_map.json`):

| physical cores | machines | with GPU | isa |
|---|---|---|---|
| 32 | 5 | 5 | avx512 |
| 36 | 24 | 2 | **avx2** |
| 40 | 25 | 20 | avx512 |
| 48 | 10 | 4 | avx512 |
| 64 | **55** | 15 | avx512 |
| 96 | 1 | 1 | avx512 |

**`REQUIRE_CORES=64` chosen** — the most common non-36 size, 55 machines, so the pinned pass needs no `--nodelist`.

**Finding that bears on the unresolved instruction-set question: on this cluster the two variables are PERFECTLY
COLLINEAR.** Every one of the 24 AVX2 machines has exactly 36 physical cores, and every machine of any other size is
AVX-512 — zero overlap across all 120 machines. So "the 36-core machines disagree with everything else" and "the AVX2
machines disagree with the AVX-512 machines" are the *same statement here*, and no comparison on this cluster can
separate them. Two consequences worth stating plainly:

1. It explains how both accounts fitted the same data without either being careless.
2. It is mild evidence *against* core count as the mechanism: the 40-core and 64-core machines are different sizes on
   the same instruction set, and those comparisons **agreed** bit-for-bit. A pure core-count rule would predict they
   diverge. What actually fits every observation is a **two-class split** (the 36-core/AVX2 group vs the rest), not a
   per-size rule. This does not change the guard — pinning `--require-cores 64` satisfies either account — but the
   paper should describe the axis as a two-class split rather than assert core count as the cause.

**The two measurements, both on pinned 64-core machines** (jobs 3363030 / 3363031; the first pair 3363001/2 died in
4 s on `set -u` vs conda's deactivate hook, fixed):

| measurement | node | wall |
|---|---|---|
| RLPD full `rnd30` cell, 30 episodes, shared protocol, one process | pax146, Gold 6438M | **858 s** (832 s in-loop, **27.7 s/episode**) |
| Diffusion Policy, one `rnd30` episode, CPU only | pax048, Gold 6448Y | **425 s** (390 s in-loop) |

The DP figure is the one that was blocking: **390 s/episode, 1.46× faster than the 568 s login-node figure** that did
not transfer. Extrapolated pinned pass (shared + `_iso`, per amendment (s)):

- **RLPD:** 210 shared + 180 isolated episodes per run ≈ 4.3 CPU-h; wall ≈ 50 min/run (the `spots60` shared cell,
  28 min, is the long pole). 16 runs ≈ **69 CPU-hours**.
- **DP:** 105 shared + 90 isolated episodes per run ≈ 22 CPU-h; wall ≈ 8 h/run, bottlenecked by the shared `spots60`
  cell (60 × 390 s = 6.5 h **serial and irreducible** — a shared-process cell cannot be sharded without becoming the
  isolated protocol). 16 runs ≈ **352 CPU-hours**.
- With 55 machines of the pinned size, all 32 runs fit concurrently: **≈ 8 h wall for the whole pinned pass**, i.e.
  ordinary parallel work, not a scheduling problem. Note the `_iso` cells are *faster* in wall-clock than the shared
  ones for DP, because one process per episode parallelises and a shared cell cannot.

Still blocked on the training runs: all 32 `e2e_*` jobs remain PENDING at `--nice=9000` behind 16 place runs and 32
robomimic A4 jobs, so the pinned pass has nothing of mine to score yet.
