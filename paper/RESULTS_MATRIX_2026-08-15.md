# d{source}_{algorithm} RESULTS MATRIX — publish-readiness (2026-08-15)

One row per condition. Best current number, its protocol, and a readiness verdict.
Style: short sentences, one claim each. Sources: PAPER_PLAN decision log,
paper/RUN_LEDGER_2026-08-14.md, FABLE_HANDOFF_2026-08-13.md §12-22.

## The matrix

| condition | best number | protocol | verdict |
|---|---|---|---|
| dH_DP | 0.62 in-dist (0.40-0.80, n=8) / 0.23 random | hardened predicate, sequence evals, P2-robustness-checked | **READY** |
| dDP_DP | 0.80 in-dist (0.67-0.93, n=8) / 0.23 random | same, matched-N, matched rule | **READY** |
| dH vs dDP (BC claim) | P(model>human) = 0.994 in-dist; random arms tie | Bayesian, n=8/arm | **READY** with the P2 methods paragraph |
| dDP2_DP (gen-1) | 0.87 lineage; chain 0.67 -> 0.87 -> 0.87 | older protocol | descriptive/appendix only |
| dH_SACfD | 0 (16 seeds) | honest predicate | **REFRAME** — confounded by the entropy-backup defect (stock SB3). Publish as the bug narrative, not as "SACfD fails" |
| dDP_SACfD | 0 (16 seeds) | same | same reframe |
| dH_RLPD | stable signature: 1/3 seeds ignite, 4-6 picks/45 at 100k; best single ckpt 0.40 (s0@150k, fresh-process replicated) | fresh-process, registered bars, 4 independent waves | **HOLD** — see credibility gap below |
| dDP_RLPD | 0/3 seeds at bar, 2/45 pooled | matched pair, same machine/wave | ready ONLY as the conditional claim: "no source effect detectable at n=3/100k" (p=0.27) |
| dH_R2D (r2dreamer) | champion 0.91 sampled / 1.00 mode (n=45+15) | best-checkpoint + independent confirmation | **READY** with the lottery protocol stated (ignition 4/20 cumulative; bistability documented; clamp fix refuted) |
| dDP_R2D | 0 ignitions in 20 seeds (FINAL — firming wave completed 08-15, all seeds ~1M steps) | two waves, dilution-fixed | **READY as the contrast**: pooled dH 6/24 vs dDP 0/20, Fisher one-sided p=0.019. Caveats: pooling crosses the E2 boundary (per-wave: dH 2/4, 3/10, 1/10 vs dDP 0/10, 0/10); the firming wave alone is underpowered (1/10 vs 0/10, p=0.5). Direction never violated in any wave |
| dH_DV3 | **NULL, final (08-15)**: all 3 seeds finished the 3e5 budget with return 0 / picked 0. Entropy fingerprint fired 3/3 but takeoff never followed (s1/s2 rebounded to +2.3/+3.2; s0 stayed committed at -0.61 to the end) | msrecipe port, registered budget = 2.2-2.7x the reference MS takeoff window (110-137k) | **READY as a null** — publish as: the MS-shaped recipe transfers the entropy-collapse precursor but not takeoff. Fingerprint = necessary, not sufficient. Option HELD for user: resume s0 (still collapsed at budget end) with extended budget |
| dDP_DV3 | not run | moot under the dH null (no source comparison without ignition) | **CLOSED** unless dH_DV3 reopens |

## Positive controls
| control | status |
|---|---|
| joint DP 0.67 in-dist (audit-replicated) | standing |
| MS_RLPD-ctl (our RLPD on ManiSkill PickCube) | **FAILED, final (08-15)**: 0/3 seeds. Flat 0.00 at every 50k decision point through 300k (single transient: s0 success 0.10 / grasp 0.10 at 150k). Grasp rate ~0 throughout — the agent never enters the grasp basin on THEIR task either. End-state diagnostics (s0): ent_coef collapsed to 4.6e-4, actor_q_mean 0.39 vs +100 terminal reward, critic_loss ~5e-4. Registered reading: the implementation (or its config) does not reproduce published RLPD-class results on the reference benchmark → the genesis RLPD rows are NOT evidence about the task or dataset until this is explained. The four-wave invariance now has a fifth candidate cause: a defect or config gap shared by all waves |
| cell B (clean in-sim champion demos -> our RLPD) | **DEMOS BUILT + GATED; wave launch-ready, held behind the MS control.** 66 demos, median 131 frames (human 746), density 0.724% (human 0.079%, ManiSkill 1.45%). Guards: negctl 0/54, open-loop 4/5 (human gate 4/5 same session), provenance stamped. Bars pre-registered. See paper/cell_b_clean_demos_2026-08-15.md |

## The RLPD credibility gap (the user's concern, stated plainly)
1. The numbers are low: ~1/3 ignition, peaks 0.27-0.40. Reviewers will suspect the implementation.
2. The defense under construction: the MS positive control (same code, their benchmark, published rates) + the intervention story (one deleted term restored ignition; the formula matched every log window).
3. The four-wave invariance is a RESULT: density, diversity, source, and action-consistency all fail to move the signature. Budget is the untested lever.
4. Decision pending: 200k x n>=5, continuation training, or take the row as measured.

## What must resolve before a full run
1. ~~MS positive control verdict~~ RESOLVED 08-15: FAILED 0/3 (see controls
   table). Implementation credibility is now the OPEN question, inverted:
   the burden moved from "is the dataset bad" to "is the trainer sound".
2. Cell B verdict (dataset-vs-method). LAUNCHED 08-15 (dR2D_RLPD-clean_s{0,1,2},
   startup gates passed: 66 eps / 9118 transitions / 66 rewarded, x3).
   Interpretation caveat now that cell A failed: >=2/3 ignite would show the
   trainer CAN work (and isolate the MS-control failure to its wrapper/config);
   <=1/3 firms a shared methods defect. The pre-registered bars stand.
3. ~~msrecipe verdict~~ RESOLVED 08-15: NULL at budget. dv3 stays out of the
   matrix as a learner; the fingerprint-without-takeoff result joins the
   actor-side-lottery evidence. s0-resume option held for user.
4. The RLPD budget decision.
5. ~~Firming-wave completion~~ RESOLVED 08-15: wave done at ~1M steps/seed.
   dH 1/10 (s22: training-loop 0.4; fresh evals 0.067/0.133 @972k, 0 @1M —
   checkpoint bistability again). dDP 0/10. Final contrast: pooled p=0.019.

## Cross-cutting caveats that apply to every row
1. P2: sequence evals are not independent draws. Paper numbers use fresh-process or carry the methods paragraph.
2. Env fixes changed baselines twice (E1, E2). Rows never pool across those boundaries (RUN_LEDGER maps every run).
3. Eval horizons: RLPD rows at 400 steps, DP/SACfD at 1200. Measured effect on the strongest checkpoint: nil. Stated per table.

## IGNITION STATISTICS (added 08-15, user request)

Ignition = a seed produces a policy above the noise floor (RLPD: >=3/15 fresh-
process demo-IC at the fixed checkpoint; r2dreamer: nonzero best-checkpoint eval).

### Measured rates

| arm | config | ignited / seeds | rate | rough 95% CI |
|---|---|---|---|---|
| RLPD nb | fixed trainer, 100k | 1/3 | 0.33 | — |
| RLPD pair-dH | same | 1/3 | 0.33 | — |
| RLPD hold | +25x density | 1/3 | 0.33 | — |
| RLPD mref | measured-ref demos | 1/3 | 0.33 | — |
| **RLPD pooled (dH-class)** | 4 waves | **4/12** | **0.33** | [0.10, 0.65] |
| RLPD pair-dDP | model demos | 0/3 | 0.00 | [0, 0.71] |
| r2dreamer dH local | delta recipe | 2/4 | 0.50 | — |
| r2dreamer dH cluster w1 | 1M steps | 3/10 | 0.30 | — |
| r2dreamer dH cluster w2 | post-fix env | 1/10 | 0.10 | — |
| **r2dreamer dH pooled** | | **6/24** | **0.25** | [0.10, 0.47] |
| r2dreamer dDP | both waves (FINAL 08-15) | 0/20 | 0.00 | [0, 0.17] |
| dv3 historical | any config | 0/all | 0.00 | — |
| dv3 msrecipe | MS-shaped task (FINAL 08-15) | entropy 3/3, takeoff 0/3 | 0.00 | [0, 0.71] |
| SACfD | defective trainer | 0/30+ | 0.00 | confounded (T1) |
| same dv3 code on ManiSkill | their task | ~always | ~1.0 | dozens of seeds |

### Structural facts
1. The RLPD rate is EXACTLY one seed per wave, four waves running. Pooled picks
   5-7/45 every wave. The rate is invariant to reward density (25x), critic
   diversity, demo source, and action-space consistency.
2. Within an igniting r2dreamer run there is a SECOND lottery: checkpoint
   bistability (~1-in-7 checkpoints good on the champion run).
3. WHICH demos a seed unlocks is seed-dependent (two 4-pick seeds share 1 of 8
   uids). The COUNT is what is stable. Ignition is a basin-entry event.
4. The same algorithms ignite ~always on ManiSkill. The lottery is a property
   of THIS task's exploration/credit landscape (pending cells A/B confirmation).

### Interpretation and the multi-policy-per-world-model proposal (user, 08-15)
The lottery does NOT live in the learned models of the world:
- r2dreamer's world model keeps improving in non-igniting runs; the failures are
  actor-side (entropy-collapse cycles, lambda-return explosion).
- Direct prior evidence (2026-08-11 bc-run diagnostic, already in the record):
  giving the recipe an action prior made it learn QUICKLY -> "the WM bottleneck
  is exploration/credit assignment, not world-model capacity."
- The four-wave RLPD invariance says the same thing model-free: the value/reward
  plumbing is fine; the seed decides whether exploration enters the grasp basin.

PROPOSAL (user): train MULTIPLE POLICIES against ONE world model. The WM trains
on all experience regardless of actor luck; actors are cheap; N actor draws cost
~1 WM training run plus imagination. Two concrete forms:
1. r2dreamer ACTOR RE-DRAWS: periodically re-init actor+critic (keep WM +
   buffer), or train K parallel actor heads on shared imagination; select by
   imagined return, confirm by eval. K lottery tickets per WM.
2. RLPD analog: periodic actor-critic RESETS keeping the replay buffer —
   literature-anchored (primacy bias: Nikishin et al. 2022; high-replay resets:
   D'Oro et al. 2023 — resets at high UTD recover plasticity and are the
   standard fix for exactly this pathology class).
This is now the leading candidate lever, ahead of brute-force budget (200k x n5):
it attacks the measured mechanism (seed-level basin entry) instead of buying
more draws at the same rate, and both forms reuse trained components.

### Code-era caveats on the ignition table (added after user challenge, 08-15)

1. **The dv3-on-MS reference (~1.0) is HISTORICAL — March 2026 code.** ~200 wandb
   runs, Feb 27 - Mar 2. Nobody has re-run MS on the current HEAD, which has
   since gained the vec facade, scope plumbing, eval ingestion, and the msrecipe
   port. The 08-10 audit verified CONFIG parity, not code-era parity. The claim
   "same code both sides" is really "same code as of March". PENDING CONTROL
   (queued, not launched): 1-2 seed MS PickCube spot-check on current HEAD with
   the March config. Cheap; closes the gap.
2. **The genesis side of dv3 IS current-HEAD measured**: the msrecipe runs on
   the cluster right now are today's code (entropy collapse 3/3). The stale
   side is the POSITIVE reference, not the null.
3. **r2dreamer pooled rates (6/24) cross the E2 env boundary**: local + wave 1
   pre-reset-fix, wave 2 post-fix. Per-wave rates: 2/4, 3/10, 1/10. The pooled
   figure is indicative; per-wave is citable. (Already the ledger's rule;
   restated here because the pooled number appears above.)
4. **The four RLPD waves ran on four successive commits** (nb f906ee6 -> pair
   -> hold f612c55 -> mref 398e078). Pooling is justified BY CONSTRUCTION, not
   assumption: every intermediate change shipped with a byte-identity gate on
   the default path (proven, incl. 28/28 policy tensors), and each wave's lever
   was the only behavioral delta. This is what the gate discipline buys.
5. A null replicated across many code states (dv3-genesis, historical) is
   robust to this caveat. A positive measured at one old state (dv3-MS) is not.
