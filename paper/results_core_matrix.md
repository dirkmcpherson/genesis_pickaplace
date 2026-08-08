# Core matrix results (pick phase)

Numbers pulled fresh from wandb on 2026-08-08 (entity `jambotime`). Official
matrix numbers come from the `*-eval` runs in project `genesis_paper`
(keys `eval_indist/*`, `eval_random/*`, 15 episodes per eval); lineage numbers
from `genesis_pickaplace_ouro` (gen-0 provenance flagged below). All source
runs were created 2026-07-31 or later, i.e. after the 2026-07-30 eval-semantics
change — no pre-change numbers are used.

Metric shown is `picked` (pick-phase scope). `eval_indist` = demo-IC evaluation
(the ~3 demonstrated can positions; the headline). `eval_random` =
support-random ICs (generalization). The two are never mixed or averaged
together.

---

## Figure 1 — `figs/bc_central_picked.png`: BC head-to-head (dH_DP vs dDP_DP)

Diffusion Policy trained on N=66 demos per source under matched everything
(same pick-grant truncation rule, same recipe, same eval ICs), x3 seeds.

| condition | seed | eval_indist picked | eval_random picked | wandb run |
|---|---|---|---|---|
| dH_DP  | s0 | 0.800 | 0.133 | `dH_DP_s0-eval` |
| dH_DP  | s1 | 0.600 | 0.333 | `dH_DP_s1-eval` |
| dH_DP  | s2 | 0.600 | 0.200 | `dH_DP_s2-eval` |
| **dH_DP mean (range)** | | **0.67 (0.60–0.80)** | **0.22 (0.13–0.33)** | |
| dDP_DP | s0 | 0.867 | 0.267 | `dDP_DP_s0-eval` |
| dDP_DP | s1 | 0.733 | 0.200 | `dDP_DP_s1-eval` |
| dDP_DP | s2 | 0.667 | 0.333 | `dDP_DP_s2-eval` |
| **dDP_DP mean (range)** | | **0.76 (0.67–0.87)** | **0.27 (0.20–0.33)** | |

What it shows: model demonstrations beat human demonstrations on both eval
distributions at the pick phase (H1 inverted). The seed ranges overlap
(dH 0.60–0.80 vs dDP 0.67–0.87 in-dist), so at n=3 this is a consistent
ordering of means, not a separation of ranges.

Caveats:
- n=3 seeds; ranges shown, 15 episodes per eval.
- The dH_DP numbers depend on idle-frame pruning (29.6% of frames dropped);
  the unpruned same-frames control sat at 0.27/0.13 — a ~2.5x preprocessing
  effect. Model harvests contain no idle frames by construction (closed-loop
  teachers), which is itself a property of the demo source, not a matched
  preprocessing step.
- Pick-phase scope only. Downstream metrics are near zero for all DP seeds
  (placed reached 0.60 once, `dDP_DP_s1-eval` in-dist; contact/nested ~0) —
  descriptive context, not matched-protocol full-task results.

## Figure 2 — `figs/sacfd_picked.png`: SACfD rows (dH_SACfD vs dDP_SACfD)

RL-from-demos; unlike DP, SACfD also consumes failed demos as zero-reward
negatives (human set 66+25; harvests keep-fails 30). Same evals as above.

| condition | seed | eval_indist picked | eval_random picked | wandb run |
|---|---|---|---|---|
| dH_SACfD  | s0 | 0.400 | 0.333 | `dH_SACfD_s0-eval` |
| dH_SACfD  | s1 | 0.533 | 0.333 | `dH_SACfD_s1-eval` |
| dH_SACfD  | s2 | 0.133 | 0.067 | `dH_SACfD_s2-eval` |
| **dH_SACfD range** | | **0.13–0.53** | **0.07–0.33** | |
| dDP_SACfD | s0 | 0.067 | 0.000 | `dDP_SACfD_s0-eval` |
| dDP_SACfD | s1 | 0.133 | 0.200 | `dDP_SACfD_s1-eval` |
| dDP_SACfD | s2 | 0.600 | 0.200 | `dDP_SACfD_s2-eval` |
| **dDP_SACfD range** | | **0.07–0.60** | **0.00–0.20** | |

What it shows: no resolvable source effect for RL-from-demos — seed variance
(0.07–0.60 in-dist across both sources) dwarfs any human-vs-model difference.
Report ranges, never bare means, for these rows. The figure draws the mean bar
for scale only.

Secondary metrics (context): SACfD is the only learner with nonzero placed —
in-dist placed 0.20/0.20/0.07 (dH s0/s1/s2) and 0.00/0.13/0.27 (dDP s0/s1/s2);
contact ≤ 0.13 everywhere; nested 0 everywhere.

Caveats:
- n=3 seeds, 15 episodes per eval; SACfD seed spread is wide (0.07–0.60) —
  the dominant uncertainty in these rows. N=5 would sharpen the comparison.
- SACfD trains on unpruned data (idle frames are honest dynamics/negative data
  for RL), so the pruning caveat above does not apply to these rows; the
  no-idle-frames property of model harvests still does.
- Pick-phase scope only.

## Figure 3 — `figs/lineage_picked.png`: generational lineage (gen 0 → 1 → 2)

Joint DP self-distillation: gen-0 trained on the 66 human pruned pick-phase
demos; each later generation trained on a success-filtered harvest from the
previous generation's policy.

| generation | eval_indist picked | eval_random picked | wandb run (project) |
|---|---|---|---|
| gen 0 | 0.667 | 0.267 | `audit_joint_rebuilt-eval` (`genesis_pickaplace`) — see provenance flag |
| gen 1 | 0.867 | 0.333 | `ouro_dp_joint_dp_gen1-eval` (`genesis_pickaplace_ouro`) |
| gen 2 | 0.867 | 0.333 | `ouro_dp_joint_dp_gen2-eval` (`genesis_pickaplace_ouro`) |

What it shows: self-distillation amplifies once (0.67 → 0.87 in-dist,
0.27 → 0.33 random) then plateaus exactly — no model collapse through two
generations, and no further gain either. The gen-1 value is independently
replicated by the matrix condition `dDP_DP_s0` (0.867 in-dist, cap-1200
harvest, fresh seed).

Caveats:
- Single seed per generation, cap-600 harvests; the matrix rows use cap-1200.
  These numbers are comparable within the lineage only — do not place them in
  the same table as the x3-seed matrix rows.
- 15 episodes per eval.
- Pick-phase scope only.

---

## Discrepancies and provenance flags (wandb vs PAPER_PLAN)

1. **Lineage gen-0 provenance (FLAG).** There is NO gen-0 eval run in
   `genesis_pickaplace_ouro` (the gen-0 eval originally went to the wrong
   project during the poisoned-env window). The gen-0 value 0.667/0.267 that
   PAPER_PLAN cites traces to `audit_joint_rebuilt-eval` (project
   `genesis_pickaplace`, 2026-07-31), which evaluates checkpoint
   `baselines/outputs/audit_joint_rebuilt` — a seed-0 joint DP trained on the
   same rebuilt pruned pick-phase dataset with the same recipe as the lineage
   gen-0, but NOT the literal `ouroboros/ouro_dp_joint/gen0/dp` checkpoint.
   The `ouro_dp_joint_dp_gen0` training run itself carries no eval summary.
   If a reviewer asks, the honest statement is: gen-0's eval is a same-recipe
   replicate, not a post-hoc eval of the very checkpoint that generated the
   gen-1 harvest. A direct eval of that checkpoint would close this gap.
2. **Everything else matches.** All 12 matrix eval numbers (dH_DP, dDP_DP,
   dH_SACfD, dDP_SACfD; in-dist and random; 3 seeds each) pulled from wandb
   agree with the values recorded in PAPER_PLAN's 2026-08-02/08-03 decision-log
   entries to the stated precision.
3. Run-history noise, for the record: three `dH_SACfD_s*` runs failed on
   2026-08-01 and were superseded by the finished 2026-08-02 runs;
   `ouro_dp_joint_dp_gen1` crashed once (2026-07-31) before the finished
   2026-08-01 run. Only finished runs' evals are used here.
