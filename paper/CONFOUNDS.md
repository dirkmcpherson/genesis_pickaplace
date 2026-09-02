# CONFOUNDS — living ledger (started 2026-09-02; reference on EVERY claim edit)

Rule: every result sentence in RESULTS/brief/paper must be checkable against this table. A confound is
"closed" only when a within-table control exists; "disclosed" means it is stated with numbers but not
removed; "open" means an experiment is running. Update the status column whenever a block reads out.

| # | confound | threatens | control / status | evidence / where |
|---|---|---|---|---|
| 1 | **World fidelity** (old vs corrected) | any cross-world comparison; the RLPD headline | CLOSED for within-learner tables (A19: one world per table); cross-learner figure is all-corrected-world; RLPD run in BOTH worlds → world-dependence is a *finding* (A16 met, A20 null) | RESULTS §2.1/§2.3, fig1/fig5, fig6 (coverage mechanism) |
| 2 | **Checkpoint rule per learner** (DP selected / RLPD LAST / WM BEST) | the cross-learner gradient | DISCLOSED + sensitivity table: DP, RLPD rule-invariant; **WM effect exists only under BEST-of-K** → WM claim = learnability under selection, not endpoint | brief §2b; A32 tests endpoint stability |
| 3 | **Reward density** (DP none / RLPD sparse / WM dense) | the cross-learner gradient | OPEN: A33 (RLPD-dense, corrected world, frozen sets, n=8v8) + A32 SPARSE-RS1 (WM sparse pilot). Prior: old-world RLPD-dense n=6v6 +0.08 ns (shaping hurt both) | PREREG A33/A32; RESULTS §2.2 dense |
| 4 | **Human tape format** (pruned vs raw) per learner | DP rows; any "human" label | DISCLOSED + quantified: frozen blocks ALL pruned (inconsistency, A24/A25); v2: DP pruned (`human*`), RLPD/WM raw; raw costs DP +0.16/+0.27 (fig11); pruning removes time not space (fig12) | fig11/fig12, `human*` convention (colstyle), RESULTS §1 |
| 5 | **Demo-set N / IC support** | every human-vs-machine contrast | CLOSED within contrasts (per-IC, per-N matching: make_matched_sets / make_v2_matched); v2 raw 69/66 vs pruned 60/60 handled by two machine views (dDPv2 vs raw base; dDPv2p vs pruned base, A31) | manifests (shas), V2_BUILD |
| 6 | **Frozen-set selection bias** (recoverable-only trials) | generality of frozen results | DISCLOSED; removed in v2 (full 74/75 pool, validated placements) | DEMO_RECOVERY_RESULTS, CRITIQUE_demo_recovery |
| 7 | **Machine-demo generator identity** (dDP = different DP teacher per world; teacher saw all ICs) | "machine demos" as one thing; per-seed holdouts | DISCLOSED: teacher must train on PRUNED human (A31, corrected after A25 error); generator exposure symmetric with the human demonstrator (A22 note, withdrawn design) | PREREG A25/A31, fig6e |
| 8 | **Row count / dataset size** | fails-arm effect; raw-vs-pruned | CLOSED for fails (row-matched dup controls, A17 rule); DISCLOSED for raw (2× rows, fig12a) | RESULTS §2.2, fig12 |
| 9 | **Training-IC placement drift** (frozen tapes vs recovered truth) | "re-executions of real trials" framing | DISCLOSED: median 2 cm, 5/19 > 3 cm, max 13 cm (319); common-mode across arms → no reruns | RESULTS claim 10 |
| 10 | **Infrastructure incidents** (NFS-handle kills from wandb_cache purge; bad node pax007; OOM packs) | seed integrity | HANDLED: fresh seed ids on relaunch (A9); re-eval from archived ckpts (no retrain); pax007 excluded; wandb_cache removed from purge | SESSION_LOG 09-02 |
| 11 | **Multiple comparisons** | DP +0.06 (one unadjusted test); 38-metric screen | DISCLOSED; screen is hypothesis-generating; causal test A28 | brief §6, WM_METRIC |
| 12 | **Eval-IC overlap** (frozen hold-15 ⊂ training ICs) | "generalization" wording | DISCLOSED (A8/A16: hold never the statistic); rnd-30 fixed and shared across ALL blocks; v2 hold = all training ICs, labelled in-dist (A23) | PREREG A16/A23 |
| 13 | **WM critic-target scale** (return_clamp=100 vs shaped returns ~500–1000) | all WM ENDPOINT claims (both arms equally) | OPEN: A32 (C2000, RS1, SPARSE-RS1). Ignition asymmetry (7/8 v 3/8) is within-block and unaffected | RESULTS §3.2, fig13 |
| 14 | **Load / machine sensitivity of replay** (borderline demos flip under load; official numbers idle box ×3) | placement funnel numbers | DISCLOSED: funnel stages are single-collection; learner evals are fresh-process per episode | 07-20 finding; DEMO_RECOVERY_RESULTS |
| 15 | **Metric operationalization** (pause_frac fragile; idle_frac gripper-column bug; act_hf_frac demoted) | mechanism claims | HANDLED: strict-stop fraction (threshold-robust) is the headline metric; both bugs disclosed in FIGURES notes | WM_METRIC addenda, FIGURES fig6/7/8 notes |

## Standing rules
1. Never compare across rows of this table without naming the confound in the caption/text.
2. A new block gets a PREREG amendment BEFORE readout, and a row here if it changes any control.
3. When a status changes (OPEN → CLOSED/DISCLOSED), edit this file in the same commit as the result.
