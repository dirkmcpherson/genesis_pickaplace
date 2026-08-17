# Cluster round 2026-08-17 — jobs in flight and what each one decides

Written at submit time (jobs running, no results yet). Companion to
RESULTS_MATRIX_2026-08-15.md (current rates) and FABLE_HANDOFF §31-34
(seed-audit corrections these designs answer to).

## The jobs

| family | n | config | where | ETA |
|---|---|---|---|---|
| RLPD n=8, dH arm | 8 seeds x 100k | nb config, post-demo-RNG-fix (>=2fbed2a), episodes_pick_phase_all | cluster, sbatch_rlpd.sh | first results ~3-4h, all by tonight |
| RLPD n=8, dR2D arm | 8 seeds x 100k | same, one variable: episodes_champion_pick | cluster | same |
| r2dreamer wave 3, dH | 10 seeds x 1M | champion recipe (genesis_pick_v5d4c_delta), seeds 30-39 | cluster | ~1-2 days (queue-gated) |
| r2d-on-MS positive control | 3 seeds x 300k | our fork, their benchmark, dense +100, all genesis levers stock | cluster (r2d_ms_venv) | ~1 day after install |
| dv3-MS-at-HEAD spot check | 1 seed x 300k (s1 follows if s0 ambiguous) | March maniskill config, today's code | local GPU | ~1-2 days |

Every RLPD job self-gates (git ancestor check for the demo-RNG fix; demo-dir
filename-pattern provenance check) and runs its own 30 fresh-process evals,
emitting one `SWEEP-RESULT` line. Aggregation is `grep -h SWEEP-RESULT
rlpd_*.out` — no JSON key path in the loop.

## What each family decides

### 1. RLPD dH vs dR2D at n=8/arm — the demo-source question, done properly
Every prior estimate of this contrast was compromised: the demo buffer ignored
--seed (all seeds shared one demo curriculum), and one dH wave was a literal
re-execution of another. These are the first cluster-scale seeds with none of
that. Current best estimates from the corrected local data: both arms ~0.33
ignition, pick-rate 0.143 vs 0.143 — indistinguishable.
- EXPECT under the null (our registered lean): ~2-3/8 at bar in EACH arm,
  similar pooled pick counts. Result: the paper's RLfD row becomes "no demo-
  source effect at n=8/arm," stated with real independence for the first time.
- A large effect (e.g. 5/8 vs 1/8) would be marginal (Fisher p~0.06) — the
  design anticipates a TOP-UP to n=20/arm (independent jobs, higher seeds,
  no rerun needed) if the point estimates separate.
- Either way: 16 training curves at known-independent seeds feed the
  instability analysis (the 9/15->0/15 collapse pattern from clean-long).
Registered before results: no point prediction on the contrast; the null is
the lean based on pooled 0.143 vs 0.143.

### 2. r2dreamer wave 3 (dH seeds 30-39) — firming the 6/24 lottery rate
The champion (0.91) rests on an ignition rate estimated from 24 seeds across
an env-fix boundary. Ten more post-fix seeds at the champion recipe:
- EXPECT 2-3/10 ignitions if the pooled 0.25 rate holds.
- 0/10 would drag the pooled rate toward 0.17 and force a "wave-2-era rate is
  lower" investigation (post-fix baseline shift, per the ledger's rule).
- Also feeds the dH_R2D vs dDP_R2D contrast (currently p=0.019) with same-era
  dH seeds.

### 3. r2d-on-MS control — is the r2dreamer FORK sound on a proven benchmark?
Our fork, their task, dense +100 reward (the setting where reference dv3
ignites ~always, takeoff 110-137k). All genesis recipe levers set to stock;
architecture (decoder-free, size12M, bounded_normal) is what's under test.
- Bar (pre-registered): >=2/3 seeds >=0.50 success by 300k, 50k decision
  points only.
- PASS: the fork is sound; genesis's 6/24 lottery is a property of task
  x recipe, and the multi-policy/reset lever is the right next investment.
- FAIL: the fork itself is suspect; pre-named suspect #1 is bounded_normal
  (samples to +/-4 outside [-1,1], left stock deliberately).

### 4. dv3-MS-at-HEAD — closing the code-era caveat
The "~1.0 on ManiSkill" reference that anchors every dv3-genesis comparison
is March 2026 code. Nobody has run MS on HEAD since the vec facade, scope
plumbing, eval ingestion, and msrecipe port landed.
- EXPECT success ~1.0 with takeoff 110-137k (config parity was audited 08-10;
  only code-era is open).
- A degraded HEAD result would UNDERMINE the msrecipe-null interpretation
  (the null could be a HEAD regression, not a task property) and would be the
  most consequential surprise available this round.

## What this round does NOT decide
1. The RLPD instability mechanism (collapse at 300k) — needs the reset/
   multi-policy build, not more seeds.
2. Anything about SACfD (retired to the bug narrative) or DP (already READY).
3. The sparse-MS question — closed as "too hard for everyone at 300k"
   (swap-test verdict, 08-16).

## Aggregation commands (for the .out files, tonight)
```
grep -l FATAL rlpd_*.out                        # must be empty
grep -h '\[demos\]' rlpd_*.out | sort | uniq -c # exactly two demo lines
grep -h SWEEP-RESULT rlpd_*.out | sort          # the whole wave, one line/seed
```
Paste the SWEEP-RESULT block into the session; the contrast, CIs, and the
top-up decision come back computed against the registered expectations above.

## AMENDMENT (08-17, registered BEFORE any SWEEP-RESULT was read)

The run-identity audit (AUDIT_run_identity_2026-08-17.md) found that the n=8
wave's seeds 0-7 largely RE-EXECUTE seeds already run locally (dH s3-7 = exp
wave; dR2D s3-6 = exp; s0 both arms = nb/clean; code unchanged between waves,
curves track to cross-hardware noise). To prevent a pair-dH-style double count
at 5x scale, the following counting rule is REGISTERED pre-data:

1. THE CLUSTER n=8 WAVE IS THE PRIMARY MEASUREMENT for the RLPD demo-source
   contrast: one machine class, one code state (post demo-RNG fix), in-job
   sweeps, internally same-machine.
2. All prior local RLPD waves (nb, hold, mref, exp, cell-B 100k) are hereby
   SUPERSEDED for pooled-rate purposes: development-era corroboration only.
   No pooled statistic may count a local seed and its cluster re-execution.
3. Any top-up extends with seeds 8-19 (never 0-7 again).
4. Tooling gap recorded: sidecars do not stamp the seed and nothing refuses
   seed re-use. RUN_REGISTRY proposal (in the audit doc) is the next build,
   AFTER this round: identity key incl. seed + demo fingerprint; refusal
   needs --duplicate-ok; git-only differences WARN (every confirmed duplicate
   crossed a doc-only commit).
