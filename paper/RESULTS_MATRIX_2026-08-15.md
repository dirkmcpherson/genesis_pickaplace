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
| dDP_R2D | 0 ignitions in ~20 seeds | two waves, dilution-fixed | **NEAR-READY** — the contrast dH 4/20 vs dDP 0/20 is p~0.05; direction never violated |
| dH_DV3 | null historically (instrumented); msrecipe RUNNING: entropy fingerprint FIRED on 3/3 seeds, takeoff pending | in-flight | **WAIT** — resolves within the 3e5 budget |
| dDP_DV3 | not run under msrecipe | data local (genesis_m1all), converter needs a dreamer-format reader | **BLOCKED** on dH_DV3 verdict |

## Positive controls
| control | status |
|---|---|
| joint DP 0.67 in-dist (audit-replicated) | standing |
| MS_RLPD-ctl (our RLPD on ManiSkill PickCube) | RUNNING — bar: >=2/3 seeds >=0.5 by 300k |
| cell B (clean in-sim champion demos -> our RLPD) | **DEMOS BUILT + GATED; wave launch-ready, held behind the MS control.** 66 demos, median 131 frames (human 746), density 0.724% (human 0.079%, ManiSkill 1.45%). Guards: negctl 0/54, open-loop 4/5 (human gate 4/5 same session), provenance stamped. Bars pre-registered. See paper/cell_b_clean_demos_2026-08-15.md |

## The RLPD credibility gap (the user's concern, stated plainly)
1. The numbers are low: ~1/3 ignition, peaks 0.27-0.40. Reviewers will suspect the implementation.
2. The defense under construction: the MS positive control (same code, their benchmark, published rates) + the intervention story (one deleted term restored ignition; the formula matched every log window).
3. The four-wave invariance is a RESULT: density, diversity, source, and action-consistency all fail to move the signature. Budget is the untested lever.
4. Decision pending: 200k x n>=5, continuation training, or take the row as measured.

## What must resolve before a full run
1. MS positive control verdict (implementation credibility).
2. Cell B verdict (dataset-vs-method).
3. msrecipe verdict (does dv3 join the matrix or stay a null).
4. The RLPD budget decision.
5. Firming-wave completion (last dDP stragglers) -> final ignition-contrast p.

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
| r2dreamer dDP | both waves | 0/~20 | 0.00 | [0, 0.17] |
| dv3 historical | any config | 0/all | 0.00 | — |
| dv3 msrecipe | MS-shaped task | entropy 3/3, takeoff TBD | — | — |
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
