# Week Summary — 2026-08-01 → 08-07 (for review)

One-line: **the paper's core experiment is complete with an inverted-H1 headline;
r2dreamer learned pick but place resisted nine designs; several load-bearing bugs
found and fixed along the way.**

## 1. Paper results (all ×3 seeds, matched protocol, wandb `genesis_paper`)

**Central table — BC at the pick phase** (N=66, same truncation rule, demo ICs,
same recipe):

| condition | picked in-dist | picked random |
|---|---|---|
| dH_DP (human demos)    | 0.80 / 0.60 / 0.60 (mean **0.67**) | 0.13 / 0.33 / 0.20 (0.22) |
| dDP_DP (model demos)   | 0.87 / 0.73 / 0.67 (mean **0.76**) | 0.27 / 0.20 / 0.33 (0.27) |

**Model demonstrations beat human demonstrations on both distributions. H1 is
refuted (inverted) at the grasp phase.**

- **SACfD (RLfD)**: dH 0.40/0.53/0.13 vs dDP 0.07/0.13/0.60 — source effect
  within seed noise; RL cares about its seed, not the demo source. Notably the
  smallest generalization gaps in the project (dH_SACfD_s0 gap 0.07).
- **Generational (archival lineage)**: 0.67 → 0.87 → 0.87 — self-distillation
  amplifies once then plateaus; **no model collapse** through two generations.
- **dv3 (H4 arm)**: 4 seeds × ~4-5M steps, both sources. Terminal evals 0/6;
  lifetime = recurring 1-2/6 flickers on 3 seeds that never consolidate.
  Honest H4 statement: at these budgets neither source produces a working dv3
  policy — an H4-consistent null (both sources equally slow), not a comparison.
  (s0/s1 stopped at ~3.9M at their 2-day wall; resubmittable but the story is set.)
- **Not launched**: wave-2 dSACfD source (needs `launch_paper_week.sh` rerun on
  the cluster — one command, teachers now exist), wave-3 dDV3 source (gated on a
  working dv3 teacher: moot).
- **Preprocessing note recorded for the paper**: DP requires idle-frame pruning
  (0.67-0.80 pruned vs 0.27 unpruned same-frames) — a 2.5× effect; model
  harvests have no idle frames (a source property H2's analysis should quantify).

## 2. The cartesian-BC mystery: SOLVED (July30th_Fable.md §2)

The obs×action 2×2, rebuilt on provably-clean data (bitwise-equal to the
positive control), answered the week-old question:

|  | joint actions | abs6 actions |
|---|---|---|
| joint obs | 0.27 | 0.07 |
| **ee obs** | **0.53** | 0.00 |

**The action representation is the killer; ee observations actually help.**
Derived abs6 actions execute the task perfectly open-loop — the failure is
isolated to DP's learned action head for absolute-pose targets. En route: the
original 2×2 was voided (its source alone drops joint DP 0.73→0.07 — the
dual-replay's inconsistent time base), and the eval-mapping bug that would have
produced a confident wrong answer was caught before any number was read.

## 3. r2dreamer (the local mandate: pick → place → full task)

- **Port**: py3.11 venv coexisting with genesis, env adapter, demo prefill into
  their torchrl buffer, periodic checkpoints, standalone eval with videos —
  built and verified in ~1 day. ~2× dv3's wall-clock throughput.
- **Pick**: trained 3.1M steps; training pick-rate grew 2.4%→~20% (survived
  demo eviction). Policy eval: ~0-1/15 — same train-eval gap as dv3, but with a
  live learning curve rather than a flatline.
- **Place (from banked held-can entry states): nine designs, no sustained
  placing.** The escalation ladder and what each taught:
  1-2. bare / +re-injection+tip-penalty → zero (demo eviction; reward invisible)
  3. +shaping+oversampling → 3 successes (gradient exists, optimum wrong)
  4-5. +dense entry curriculum (1790 entries) → ~50 successes then collapse
     (**quick-tip beat holding**: step cost -3 vs tip -0.25)
  6. tip -1 → zero (**grip-opening globally suppressed**)
  7. graded tips + attempt bonus → 1 (**bonus farmable**)
  8. no step cost → 1 (**hold-forever attractor**)
  9. **actor-BC** (imitation term, λ 1→0.1) → 7 total, 0 late — best yet;
     imitation likely reconquered by value-max as λ decayed. Final checkpoint
     eval running now; a λ=1.0-throughout rerun is the one config-only
     escalation left before calling it.
- **Bonus find in #9**: demo actions were fed ONE STEP STALE into the world
  model in every prior demo run (npz vs buffer row convention) — fixed.
- The pre-collapse checkpoint is a *perfect holder* (15/15 held, 0 tips) that
  never releases — the cleanest artifact of the "commitment problem."
- **Place-phase assets banked for any future attempt**: entry banks (66 + 1790
  dense), place demos with images (41), placed_v2 release predicate, shaping
  infra, BC-actor infra.

## 4. Bug ledger additions this week (all fixed & pushed)

| bug | blast radius |
|---|---|
| eval-ingest float() on video path | killed s2's training at 2.2M |
| FullTaskEnv missing scope/constants | ALL joint dv3 periodic evals silently failing |
| grip column ×2 more (joint tip rule, 6-DOF cartesian tip rule) | 4th & 5th family sightings |
| hardcoded .venv-eval python in eval callback | killed all 3 dH_SACfD on cluster |
| lerobot resume contract (config_path) | killed the DP lineage gen-1; would have hit every preemption |
| 2×2 eval env keyed off obs half | confident-wrong-answer to the central question (caught pre-read) |
| demo actions one step stale in r2dreamer | all demo-prefill runs before 08-04 |
| pgrep self-matches (×3 incidents) + orphaned workers (81 procs, 42GB) | wasted GPU-hours, OOM'd runs |

## 5. Operational honesty

- Cluster queue mystery diagnosed: priority weighting + no backfill for long
  walltimes; jobs flowed via preempt slot-recycling (~4 concurrent).
- **The BC run ended 08-04 and local monitors failed to wake me — the GPU sat
  idle ~2.5 days.** The watcher chain that carried the week broke unattended.
- Quota: within budget all week; storage ~as estimated.

## 6. Decisions on your desk

1. **Place**: one λ=1.0 rerun, or call it and write the commitment-problem
   finding (my lean: one rerun — config-only, overnight, then call it either way).
2. **dv3 s0/s1**: resubmit for the last ~1M to 5M, or accept ~3.9M as terminal.
3. **Wave-2 dSACfD**: rerun `launch_paper_week.sh` on the cluster (one command)
   to fill the third source row.
4. **N=5** on the cheap lerobot conditions for the paper's error bars.
5. Paper assembly: analysis scripts + skeleton were pre-empted by the place
   campaign — next block of work unless you redirect.
