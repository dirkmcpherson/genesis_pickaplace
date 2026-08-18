# Algorithm state, ignition view, and the dense-reward expectation (2026-08-18)

Style: short sentences, one claim each. Sources: paper/CLUSTER_ROUND_2026-08-17.md
(RESULTS + amendments), paper/RESULTS_MATRIX_2026-08-15.md, paper/figs/
FIGNOTES_ignition_20260818.md, the four AUDIT_*_2026-08-17.md reports, and
FABLE_HANDOFF §31-36. Every number below traces to one of those.

Figures: paper/figs/overview_ignition_20260818.png (all cells, one dot per seed)
and the per-source panels. Style rule: shape = demo source, color = algorithm.

---

## 1. Training state per algorithm

### DP (diffusion policy, BC)
- **State: DONE, publish-ready.** dH 0.62 in-dist (8/8 seeds 0.40-0.80); dDP
  0.80 (8/8 seeds 0.67-0.93). Random-IC 0.23 both. P(model>human)=0.994 in-dist.
- Seed audit: DP is CLEAN (lerobot seeds correctly; per-seed curves differ from
  step 1). Untouched by the RLPD/dv3 defects.
- Not run: dR2D_DP (DP on champion clean demos). Trivial to add Thursday.
- Ignition: **none — DP has no lottery.** Every seed lands in a 0.4-0.93 band.

### RLPD (SAC + immutable 50% demo batches, ensemble critics, UTD 10)
- **State: PRIMARY measurement DONE** (cluster n=16 x 2 arms, 08-17/18, post
  demo-RNG fix, in-job fresh-process sweeps): dH **8/16 ignited**, pooled pick
  0.221; dR2D **10/16**, pooled 0.200. Contrast p=0.72 — a clean null on the
  demo-set question. Best seeds 0.60 / 0.53 at 100k; median seed 2.5-3/15.
- dDP: only the superseded pair wave (0/3, pre-fix). Needs the n=16 treatment
  Thursday.
- Bugs found and fixed this week: demo buffer ignored --seed (all seeds shared
  one curriculum, fixed 2fbed2a); pair-dH wave was a re-execution (struck);
  a 5-line scoring bug scored one sweep 0/90 (caught by audit); four silent-
  default CRITICALs (grip-column 6th sighting etc.) fixed 08-17.
- Controls: our RLPD and the AUTHORS' reference RLPD both fail sparse
  ManiSkill PickCube at 300k identically (0/3, grasp 0). Dense ManiSkill:
  reference grasps 0.3-0.6, ours ~0, neither completes. Trainer is not the
  distinguishing defect; sparse-at-300k is hard for everyone.
- Running now: dense-reward assessment (3 seeds, local; §4).

### r2dreamer (decoder-free world model, our fork)
- **State: dH DONE across 3 waves + local; dDP DONE (0/20).** Pooled dH
  ignition 8/34 (0.24). Champion 0.91 sampled / 1.00 mode; wave 3 (08-18)
  added s36 at 1.00 and s31 at 0.93 (fresh evals). dH-vs-dDP contrast p~0.02,
  direction never violated.
- **MS positive control PROVISIONALLY PASSED** (08-18): two seeds >=0.8 at
  the 50k decision point, one at 1.0 by 61k — ~2x faster than reference dv3
  on the same recipe. Confirmation at 100k/150k pending. If it holds, the
  fork is sound and bounded_normal comes off the suspect list.
- Seed audit: r2dreamer FULLY CLEAN.
- Structural fact: igniting seeds COLLAPSE — both wave-3 igniters read 0 at
  their final 3M checkpoint. Best-checkpoint protocol stands. This is the
  bistability the multi-policy proposal targets (§5).
- Not run: dR2D_R2D (r2dreamer as its own student). Cheap to add.

### dv3 (dreamerv3-torch, world model)
- **State: NULL on genesis** (msrecipe: entropy fingerprint 3/3, takeoff 0/3
  at 3e5). Every genesis dv3 seed label was FICTIONAL until yesterday
  (config.seed overwritten by OS entropy since Jan 16 — audit F3); the
  minibatch curriculum was hard-seeded 0 in every run (F2). Both fixed
  (dreamerv3-torch 43dcc6c). Independence of the 3 msrecipe draws holds
  (they were random); reproducibility of any specific run does not.
- **MS-at-HEAD spot check RUNNING** (2 seeds, March demo recipe, seed-fixed
  code): ~55k steps, 0.0 so far — expected, takeoff window is 110-137k.
  Verdict ~tomorrow morning. A degraded HEAD result would undermine the
  msrecipe-null reading; a clean 1.0 keeps dv3's genesis null as a task-x-
  recipe property.
- Not run: dDP_DV3, dR2D_DV3. Both gated on the HEAD verdict.

---

## 2. My view on ignition rates

1. **Ignition is algorithm-shaped, not just low.** r2dreamer is cleanly
   bimodal (0 or 0.5-1.0, nothing between). RLPD is broad-and-low (6/16 dH
   seeds at exactly 0, the rest smeared 0.07-0.70). DP has no lottery. dv3
   never leaves 0 on genesis. Same task, same demos, four different
   distributions — the lottery is a property of the learner x task, not of
   the task alone.
2. **The rates are more trustworthy than they were 48h ago.** The RLPD n=16
   wave is the first with independent demo curricula per seed and its
   ignition DOUBLED (0.33 -> 0.50-0.62). Part of that may be the cluster
   machine class (G2/G3: cross-machine physics differences are real); the
   within-wave contrast is clean regardless.
3. **The remaining suspicion is legitimate but now bounded.** Four audits
   (RNG, silent-default, run-identity, normalization) closed the code-side
   causes I can find. What remains is task structure: sparse pick reward at
   ~500 sim steps to the first +1, a grasp basin that random exploration
   enters rarely, and demo tapes whose action labels are 13% unrepresentable
   in the training action space (normalization audit) — none of which is a
   bug, all of which lower ignition.
4. **The performance ceiling is fine; the variance is the finding.** Ignited
   RLPD seeds reach DP-comparable in-dist rates (0.53-0.60). Ignited
   r2dreamer seeds reach 0.93-1.00. The paper's honest RL claim is "high
   ceiling, seed lottery, collapse after ignition" — and that claim now has
   three independent sightings of collapse (r2d champion, r2d wave 3, RLPD
   clean-long 9/15 -> 0/15).

---

## 3. Expected difference from dense reward

The lever (built + gated 08-18): `pick_shaping` — potential-based approach
term r += γφ(s') − φ(s), φ = −2·‖eef − can‖, γ matched to the agent's 0.998.
Training-only; the eval metric is unchanged (sparse fresh-process pick).

What it should and should not do, registered before data:
1. **Should raise IGNITION, not the ceiling.** Potential shaping is
   policy-invariant (Ng 1999): the optimal policy is unchanged, so an
   ignited seed's endpoint should not move. What changes is basin entry —
   the gradient toward the can exists from step 0 instead of appearing only
   after a lucky grasp. Prediction: RLPD ignition rises from ~0.5 toward
   0.7-0.9 IF the lottery is basin-entry-limited (the leading hypothesis);
   stays ~0.5 if it is credit-assignment- or plasticity-limited.
2. **Should NOT create hover-farming.** The audit C1 hazard (dense +
   terminate-on-success + high γ) applies to RAW dense reward, not potential
   shaping: here the per-step leak is (1−γ)·2·d ≈ 0.002·d, ≤ 0.001/step at
   any reachable distance, and it telescopes to a bounded total. Gate showed
   ~0.004/step against a +1 terminal. Completion still dominates.
3. **Should reduce seed variance, not eliminate it.** If ignition becomes
   near-universal, the remaining spread is the collapse dynamics (§2.4),
   which shaping does not address.
4. **Cross-algorithm expectation.** RLPD: the assessment running now (bar in
   CLUSTER_ROUND: ≥2/3 seeds ≥3/15 AND pooled ≥0.16). r2dreamer: the same
   term can be added to the genesis env config; expected effect smaller
   because r2dreamer already ignites via imagination and its failure mode is
   post-ignition collapse. dv3: potentially the LARGEST beneficiary — its
   only positive anywhere was a dense-reward result, and its genesis
   fingerprint-without-takeoff is exactly "actor commits but never finds
   reward." DP: unaffected (no reward).
5. **The paper framing if it works**: "under sparse reward, ignition is a
   lottery for every RL/WM learner; under potential-shaped reward, does the
   demo-source effect appear?" — a cleaner H4 test than more sparse seeds.
   If it does NOT work: dense stays out of the round robin and the sparse
   story stands unamended.

---

## 4. Running right now (08-18 13:00)
- dv3-MS-at-HEAD, 2 seeds, March recipe (local GPU, ~55k of 200k).
- RLPD dense dH s1 (local); s0/s2 memory-gated behind it (chain running).
- r2d-MS control, 3 seeds (cluster; provisional pass at 50k).
- Figures/style rule/audits all pushed through 88f5f85.

---

## 5. Multi-policy-per-world-model: yes, and here is the plan

The user's proposal (08-15) is the right next lever for the world-model arm,
and the evidence for it grew this week: r2dreamer's world model keeps
improving in non-igniting seeds; ignited seeds collapse actor-side; the
lottery lives in the actor. So the plan, gated on the dv3-MS-HEAD verdict:

1. **r2dreamer first, not dv3.** r2dreamer has 8 igniting dH seeds with
   saved world models and 26 non-igniting ones. Freeze a trained WM (from
   an ignited AND a non-ignited seed), keep its replay buffer, and re-draw
   the actor+critic K times (K=5-8) on shared imagination. Select by
   imagined return; confirm by fresh-process eval. Two registered questions:
   (a) does a NON-igniting seed's WM support an igniting actor on re-draw?
   (that is the whole hypothesis: the WM is fine, the actor draw is the
   lottery); (b) does re-drawing from an IGNITED seed's WM reproduce
   ignition at rate >> 0.24?
2. **dv3 second, only after MS-HEAD is clean.** Same K-actor mechanism, but
   there is no ignited dv3 genesis WM to start from — so it rides the dense
   lever (§3.4) or a fresh post-seed-fix wave. If dv3-MS-HEAD is degraded,
   dv3 gets a bisect before any of this.
3. **RLPD analog in parallel (cheap):** periodic actor-critic RESETS keeping
   the buffer (Nikishin 2022 / D'Oro 2023 primacy-bias resets) — the model-
   free version of "K draws per shared data." Also directly targets the
   9/15 -> 0/15 collapse.
Cost: one WM training run + K imagination-only actor trainings ≈ 1.3x a
single run for K lottery tickets.
WM freezing (user note, 08-18): fully freezing the WM is the FIRST variant
to try — it is much faster (no WM gradients, actor-only updates on
imagination) and isolates the actor-lottery hypothesis cleanly. But a
fully frozen WM can be a disadvantage: the re-drawn actor visits states
the WM never fit, and a TINY WM learning rate (e.g. 1e-2 to 1e-1 of the
training lr) lets it track those without re-opening the lottery. Registered
order: (i) frozen WM, K draws; (ii) only if (i) ignites less than expected
or actor returns diverge from imagined returns, repeat with tiny-lr WM.
Never start with (ii): it is slower and confounds 'actor draw' with 'WM
continued training'. Build estimate: 1 day for r2dreamer (the
checkpoint/replay plumbing exists), then a 3-WM x K=6 local pilot before it
goes on the cluster.
