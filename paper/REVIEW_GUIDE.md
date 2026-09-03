# REVIEW GUIDE — cold-start instructions for reviewing this project's results
(written 2026-08-19; assume the local box and its session context are GONE and
you have only this repo + wandb access as jambotime)

## What this project is
Human-vs-model demonstrations for imitation/RL on a Kinova pick task in Genesis.
Conditions are d{source}_{algorithm}: sources dH (human), dHpruned (human,
idle-frames pruned — DP only), dDP (DP-teacher harvests), dR2D (r2dreamer-
champion harvests); algorithms DP, RLPD, r2dreamer (R2D), dv3 (DV3).

## Read in this order
1. paper/RESULTS_MATRIX_2026-08-15.md — every condition's best number, protocol,
   readiness verdict, and every correction (corrections are dated, never silent).
2. paper/ROUND_ROBIN_2026-08-20.md — the final training round: what was
   submitted, the smoke proofs, the Thursday submit map, caption caveats.
3. paper/CLUSTER_ROUND_2026-08-17.md — the prior round's results + the two
   pre-data amendments (counting rule; fidelity confound).
4. paper/ALGORITHM_STATE_2026-08-18.md — per-algorithm state, the ignition
   interpretation, dense-reward expectations (§3, registered pre-data), and
   the multi-policy-per-WM plan (§5).
5. FABLE_HANDOFF_2026-08-13.md §25-36 — the session ledger: every bug, fix,
   audit, and decision with dates and commits.
6. paper/AUDIT_*_2026-08-17.md + AUDIT_ms_chain_2026-08-16.md — four adversarial
   audits; their findings drove the corrections in (1).
7. paper/RUN_LEDGER_2026-08-14.md — which runs cross which env-fix boundaries
   (E1/E2/E3); pooling rules.

## Where results land (all automatic, no box needed)
- wandb entity: jambotime. Projects: genesis_paper (RLPD, DP, incl. evals),
  r2dreamer_genesis (r2d train + -eval-step runs), dreamer_v3_maniskill and
  dreamer_v3 (dv3).
- RLPD: fresh-process sweep numbers are PUSHED to each run's wandb summary
  (sweep/demoIC, sweep/randomIC, sweep/n) — these, not in-train evals, are the
  paper numbers. Run names {ARM}_RLPD-n20_s{SEED}[-shaped].
- DP: eval runs named {ARM}_DP...-eval; metric = eval_indist/picked + random.
- r2dreamer: -eval-step runs carry eval/picked; BEST-checkpoint protocol (see
  matrix row) — the final checkpoint often reads 0 on igniting seeds
  (bistability); never judge by the last checkpoint alone.
- dv3: eval_success_rate / log_picked in the -joint runs.
- Cluster .out files (if reachable): one greppable line per job — SWEEP-RESULT,
  DP-RESULT, R2D-RESULT, DV3-RESULT. cluster/RUN_REGISTRY.jsonl lists every
  launched run's identity key (script, arm, seed, git, knobs, demo fingerprint).

## How to aggregate honestly (the rules that got learned the hard way)
1. Fresh-process numbers only for paper claims; in-train eval curves are
   monitoring. RLPD: use the sweep/ summary fields.
2. Unit of analysis = the SEED. Ignition = >=3/15 fresh demo-IC (RLPD) or
   >=0.20 best-checkpoint eval (r2d). Never pool a seed with its re-execution
   (RUN_REGISTRY warns on these); never pool across E-boundaries (run ledger).
3. Read any json metric as ['metrics']['eval/picked'] — a top-level .get()
   default silently scored 90 episodes as 0 once (matrix cell-B correction).
4. Before trusting any all-zero aggregate, open ONE stdout log and ONE json.
5. Figures: shape = demo source, color = algorithm (paper/figs/STYLE_RULE.md);
   aggregation scripts in analysis/ (make_ignition_figs, pull/assemble).
6. Known caveats per cell: ROUND_ROBIN §4 (dR2D is 52 pixel demos for WM arms
   vs 66 state tapes for RLPD/DP; dDP DP-row trains success-only; dH = unpruned
   for RL/WM, dHpruned for DP; cluster-vs-local same-machine rule).

## Registered bars still open at write time
- RLPD dense (pick_shaping): CLOSED 08-19 — PASS. s0 0/15, s1 4/15, s2 5/15
  demo-IC; pooled 9/45 = 0.20 >= 0.16; artifacts in
  paper/dense_verdict_2026-08-19/. Dense blocks submitted for RLPD/r2d/dv3
  (ROUND_ROBIN FINAL THURSDAY SUBMIT BLOCK). Expectation held (ALGORITHM_STATE
  §3: raises ignition odds, not the ceiling).
- r2d/dv3 dense variants (if submitted): same qualitative expectation; their
  shaped configs are genesis_pick_v5d4c_delta_shaped (r2dreamer, gamma 0.999)
  and genesis_pick_msrecipe_shaped overlay / SHAPED=1 (dv3, gamma 0.997).
- dv3 round-robin ignition = nonzero picked at eval; the MS-HEAD control
  (0.9 x 2 seeds, wandb dreamer_v3_maniskill msHEAD_demo_s{0,1}) carries the
  implementation's credibility.
- r2d-MS control: PASS (two seeds >=0.8 by 51-61k; CLUSTER_ROUND doc).

## The paper's current headline claims (with their evidence)
1. BC: model demos beat human in-dist at matched N (dDP_DP 0.80 vs dHpruned_DP
   0.62, P=0.994, n=8/arm); no generalization gain (random 0.23 both).
2. RLfD (RLPD): no demo-set effect at n=16/arm (0.50 vs 0.62 ignition, p=0.72);
   high per-seed variance and post-ignition collapse are the phenomenon.
3. World models: r2dreamer reaches 0.91-1.00 on human demos via a seed+checkpoint
   lottery (8/34 ignition); model-demo arm 0/20 (p~0.02); dv3 null on genesis
   with BOTH MS controls positive -> task-x-recipe, not code.
4. Every RL/WM implementation is positively controlled on ManiSkill; the two
   RLPD MS failures are sparse-reward-specific (reference RLPD fails
   identically — swap test).
5. Collapse-after-ignition observed independently in r2dreamer (2x waves) and
   RLPD (clean-long) -> motivates the multi-policy/reset program (ALGORITHM_STATE §5).

## If you must relaunch or extend
Use the four hardened sbatches (cluster/sbatch_{rlpd,dp,r2dreamer}.sh,
dreamerv3-torch/sbatch_genesis_multi.sh). Every one: DRYRUN=1 first; gates
refuse wrong arms/datasets/stale env vars; RUN_REGISTRY refuses duplicate
(config,seed,demo) keys unless DUPLICATE_OK="reason". Datasets move by rsync
ONLY (never git). NEVER conda-install into the cluster genesis env. The
Wednesday smoke transcript (what passing looks like) is in ROUND_ROBIN §2 and
the 08-18/19 handoff entries.
