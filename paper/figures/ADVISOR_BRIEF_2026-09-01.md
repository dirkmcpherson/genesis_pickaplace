# The corrected-world triple gradient — full explanation for the advisor briefing

Companion to `fig9_advisor_triple.png` (this directory). Analysis: `analysis/bayes_triple_2026-09-01.py`.
All per-seed numbers of record: `paper/RESULTS_for_writing_2026-08-30.md` §1, §2.3, §3.1. Everything here is
from pre-registered blocks (`paper/PREREG_final_round_robin_2026-08-23.md`, amendments A16–A28).

## 1. The result in one sentence

In a single simulated world, with per-IC-matched demonstration sets, the effect of demonstration source
(human teleoperation vs a converged DP policy's rollouts) **scales with how much the learner uses the
demonstrations beyond imitation**: Diffusion Policy is (nearly) indifferent, RLPD shows a pre-registered
null, and the world model strongly prefers human demonstrations.

## 2. The central table (corrected world; success = pick on 30 fixed random ICs)

| learner | how it uses demos | human | machine | Δ | perm p | Δ posterior [95% CrI] | P(Δ>0) | P(ROPE) |
|---|---|---|---|---|---|---|---|---|
| DP (n=10v10, selected ckpt) | imitation only | 0.547 | 0.487 | +0.06 | 0.042 | +0.06 [−0.05, +0.16] | 0.85 | 0.42 |
| RLPD (n=8v8, LAST ckpt) | off-policy value backup | 0.496 | 0.517 | −0.02 | 0.98 | −0.03 [−0.25, +0.20] | 0.39 | 0.33 |
| WM/r2dreamer (n=8v8, BEST ckpt) | learns dynamics, dreams | 0.554 | 0.308 | +0.25 | 0.133 | +0.20 [−0.05, +0.42] | 0.95 | 0.08 |

**Read the WM row honestly:** at n=8v8 the mean statistic is directional-strong but NOT significant
(P(Δ>0)=0.95, CrI touches zero) — one dead human seed (s84) moved it below the line, and an earlier draft
of this brief (n=7v8, before s84's re-score) overstated it as CrI [+0.07,+0.49]; this row is the correction.
The ignition view — **human 6/8 vs machine 3/8** under the registered BEST-hold ≥ 8/15 criterion (Fisher p 0.315; P(ignition_H > ignition_M) 0.926; Δignition +0.30 [−0.12, +0.67]; corrected 09-02 from a mis-quoted 7/8 v 3/8 that used BEST rnd, see RESULTS §3.1 / PREREG A35) — is suggestive only, at n=8. **The confirmatory
test is A27 (n=12v12, ignition pre-defined as co-primary), reading out ~09-03.** Until then the WM claim is
"consistently directional across every view of the data, pending confirmation," not "established."

Checkpoint rules differ per learner **by pre-registration**, matching each learner's failure mode: DP is
stable (selected ckpt); RLPD's registered statistic is the LAST checkpoint (A16 — selection can hide
late divergence); r2dreamer is checkpoint-bistable in BOTH arms, so BEST-of-K on a selection set is
load-bearing and disclosed (LAST checkpoints are mostly dead in both arms).

### 2b. Checkpoint-rule sensitivity (added 09-02 — a reviewer will ask)
The three rows use three checkpoint rules (pre-registered per learner). Same-rule re-read, corrected world:

| Δ (human − machine) | selected / BEST | LAST |
|---|---|---|
| DP, rnd | +0.06 | +0.05 (final ckpts 0.52 vs 0.47) |
| RLPD, rnd | +0.04 (perm p 0.66) | −0.02 (p 0.98) |
| WM, hold | +0.22 (BEST) | **−0.07** (LAST 0.29 vs 0.36; both arms mostly dead) |

DP and RLPD are rule-invariant. **The WM effect exists only under BEST-of-K** — the honest statement of the
bistability disclosure: the world model REACHES a working policy from human demos more often (6/8 vs 3/8
seeds, corrected 09-02), but neither arm KEEPS it to the end of training. The WM claim is about learnability under selection,
not about the trained endpoint, and must be worded that way.

## 3. The context that gives the table meaning: the old world

| RLPD, old world (γ 0.99 sparse) | human | machine | Δ | p |
|---|---|---|---|---|
| success-only sets (n=7v7, LAST rnd) | 0.700 | 0.495 | **+0.21** | 0.002 |
| + own failure tapes (n=8v8) | 0.746 | 0.537 | **+0.21** | <0.001 |
| + duplicated-success control (n=8v8) | 0.688 | 0.475 | **+0.21** | 0.013 |
| divergence (max critic loss ≥ 1) | 1/8 | 6/8 | | Fisher 0.041 |

The same RLPD recipe shows a large, thrice-replicated human advantage in the *old* (lower-fidelity) world —
and a divergence asymmetry with a measured mechanism: the machine demos there are **narrow** (−21% EEF
workspace coverage vs human, fig6), and a bootstrapped critic extrapolates badly outside demo support. In
the corrected world the coverage deficit closes to −5% (the stiffer, gravity-compensated arm makes the DP
teacher's rollouts nearly human-broad) — and the RLPD effect vanishes, exactly as the coverage account
predicts. Both the old-world effect (A16) and the corrected-world null (A20) were pre-registered, with
predictions met and failed respectively, and both are reported.

**"Null" vs "indifferent":** DP-indifferent is an *invariance of the learner* — bounded-small effect in
both worlds, pruned or raw tapes. RLPD-null is *one cell of a learner × world interaction* — the learner
demonstrably cares one world over; this world removed the property (coverage narrowness) it cares about.
Without the old world these would look like the same fact; the old world is what splits them.

## 4. What the statistics are, and why these ones

- **Seed is the unit everywhere.** One training run = one observation. Episodes within a seed are not
  independent evidence about the *learner* (they share weights), so all inference is over seeds.
- **Exact permutation test** (primary frequentist): relabel the seed outcomes across arms, count how often
  the mean difference is at least as extreme. No distributional assumptions; exact at these n. Welch t
  (with Welch–Satterthwaite df) is the companion for confidence intervals.
- **Hierarchical Beta-Binomial** (Bayesian): seed k in arm a scores y_ak ~ Binomial(30, θ_ak) with
  θ_ak ~ Beta(μ_a·κ, (1−μ_a)·κ). The hierarchy absorbs seed-level overdispersion (real: RLPD and WM seeds
  are bimodal) instead of pretending 240 pooled episodes are 240 independent Bernoullis — which would give
  absurdly overconfident intervals. Priors are weak (μ uniform, κ ~ Gamma mean 20). Report: posterior of
  Δ = μ_H − μ_M, its 95% credible interval (CrI: "given data and model, 95% probability Δ lies here" —
  the statement an advisor actually wants, vs the CI's repeated-sampling guarantee), and P(Δ>0).
- **ROPE** (region of practical equivalence, |Δ| < 0.05): the Bayesian tool for *arguing a null*. P(ROPE)
  is the posterior mass on "practically zero." RLPD's 0.33 means: consistent with zero, but n=8 can't
  *prove* equivalence — we claim "no large effect + failed registered prediction," not "exactly equal."
- **Ignition rate + Fisher exact** (WM co-primary, pre-defined in A27): the WM's failure mode is
  not-igniting (a seed either learns the task or stays near zero), so arm means mix two populations and
  their variance is ugly. Ignition (BEST hold ≥ 8/15) is the statistic matched to that structure; Fisher
  tests the 6/8 vs 3/8 split exactly (p 0.315). Threshold fixed before the confirmatory block (A27) runs, disclosed
  as informed by this data.

## 5. Mechanism status

- **Why RLPD's effect is world-dependent: measured.** Coverage (fig6, fig7) — the corrected world both
  *retains more of the real human signal* (+20%) and *produces broader machine demos* (−21% → −5% deficit).
- **Why the WM still cares when coverage equalizes: hypothesis with a measurement.** Screening 38 tape
  metrics with a pre-declared criterion (separates sources in the corrected world MORE than in the old —
  i.e., not just coverage again), the only surviving family is **temporal burstiness** — and a robustness re-derivation (09-01 pm)
  sharpened its operationalization: the discriminating quantity is the **full-stop fraction** (decisions
  with EEF speed < 0.5 mm): human 0.37 vs machine 0.22, d = +1.16, threshold-tight — *humans stop,
  the machine creeps*. (The screen's original pause_frac used a tape-relative threshold that is fragile
  when ~half the frames are slow — at a loose 2 mm cut the arms are indistinguishable, 50% vs 48% —
  this sensitivity is disclosed, and fig10 shows the robust version.) Interpretation: human demos contain
  genuine rest states and stop→start transients — dynamics regimes the creeping machine rollouts never
  exhibit; a world model must model them, a critic only needs the visited states. This is explicitly *hypothesis-generating* (38 comparisons, one dataset — stated in
  `paper/WM_METRIC_2026-09-01.md`) and its causal test is pre-registered (A28): machine demos re-timed to
  human dwell statistics (geometry unchanged), human demos smoothed, and a DART-noise control that raises
  bandwidth *without* stop-go structure — if the noise arm ignites too, the mechanism is bandwidth, not
  structure. Sets in construction with a required manipulation check.

## 6. Caveats to volunteer

1. WM arm deviations, disclosed — and CORRECTED 09-02: the return clamp (100) is mis-set 5–50× below the shaped
   returns actually attained (median ≈ 500–1000), which explains the checkpoint bistability in both arms (critic
   saturated → advantage noise → entropy collapse; fig13). The earlier "unclamped r2dreamer runs away" verdict was judged
   against the wrong ceiling and is withdrawn (dv3's runaway, re-checked against its own returns, STANDS); correctly scaled pilots are pre-registered (A32). Dense reward (sparse
   never ignites) and BEST-of-K selection remain disclosed; endpoint (LAST) claims are suspended pending A32.
2. DP's p = 0.041 is one unadjusted comparison; treat as "at most a small human edge."
3. RLPD corrected-world n=8 → wide CrI; the v2 full-pool block (n=8v8, training now) adds power.
4. The frozen demo sets under-represent the hardest human demos (recovered only on 08-31); the v2 blocks
   (full 66–69-demo pool, fresh validated placements, raw unpruned human tapes) re-test everything —
   DP+RLPD results ~09-02, WM n=12v12 (A27) ~09-03, all with registered predictions.
5. The world-fidelity claim rests on two worlds; "fidelity" is one axis of many possible world differences
   (though the sim2real corrections were bag-measured, not arbitrary).

## 7. What is running right now (all pre-registered)

| block | question | n | ETA |
|---|---|---|---|
| A25/A26 v2 DP+RLPD, both worlds | do the DP/RLPD rows replicate on the full raw pool? | 8v8 each | ~09-02 |
| A27 v2 WM, corrected world | does the WM row replicate with power? | 12v12 + ignition co-primary | ~09-03 |
| A28 burstiness ablation | is burstiness causal for WM ignition? | 3 arms × 4 seeds | ~09-04 |
