# Amendment (af) — cluster pixel batch: running tally (started 2026-09-14 00:35)

Design, predictions P-af-1…5 and the launch record: `paper/PHASE_PLAN_2026-09-04.md` amendment (af) + rev 1 + the
status notes of 2026-09-13 21:28 / 22:00 / 22:30. Every world-model run: pixels (top ++ wrist 64×64) + 8-dim proprio,
`image_aug shift4`, `tip_guard not_in_hand`, fresh world model, 1M online steps, ONE seed per GPU (preempt QOS), trees
`$LAB/gp_px` @ 9b50280 (2f33ac6 for the two resubmitted seeds: launcher cache path only) + `$W/r2dreamer_px` @ 0b1b9d8.
Cells: the milestone sweep at 0.5M and 1M online (`$W/ln_milestone_cells/<run>/online_<N>/<cell>/metrics.json`), fresh
process, pinned 64/64-core nodes, training world stamped; submitted through the preempt QOS (status 22:30). **A cell
is one checkpoint; the policy oscillates and collapses on short timescales (R2D_LIVE_VS_RELOAD, the (ae) dead
checkpoints), so quote both milestones together, never one.** Every number below is copied from the cells' own
`headline_stages` as printed by the hourly check; rates are over 30 (rnd30) or 15 (hold15) episodes.

## 0.5M online — `home` rate (rnd30 MODE | hold15 MODE | rnd30 SAMPLED), read 2026-09-14 00:35

### {dv3 local recipe on the cluster = DreamerV3 losses in the r2dreamer chassis}, `nested_sparse10`

| seed | human `dHfull_all_rns10h_img` | machine `dDPfull_first_rns10h_img` |
|---|---|---|
| s4 | 0.37 (11/30) \| 0.60 (9/15) \| (cell pending) | 0.63 (19/30) \| 1.00 (15/15) \| 0.60 |
| s5 | **0.00 (0/30; picked 0.70)** \| (cell pending) \| (cell pending) | 0.57 (17/30) \| 1.00 (15/15) \| 0.67 |
| s6 | 0.43 (13/30) \| 0.67 (10/15) \| 0.50 | (milestone reached 00:20; cells queued) |
| s7 | (milestone reached; cells queued) | (resubmitted seed, at 0.57M counter; milestone imminent) |

### {r2dreamer = the port's contrastive representation loss}, pixels, `nested_sparse10` — P-af-2

| seed | human | machine |
|---|---|---|
| s0 | 0.57 (17/30) \| 1.00 (15/15) \| 0.40 | 0.50 (15/30) \| 1.00 (15/15) \| 0.50 |
| s1 | 0.63 (19/30) \| 0.60 (9/15) \| 0.57 | 0.40 (12/30) \| 0.47 (7/15) \| 0.43 |
| s2 | 0.60 (18/30) \| 0.47 (7/15) \| 0.57 | (milestone reached; cells queued) |

### ramp control (`nested_ramp`, dreamer losses) — P-af-3: machine s0 cells queued 00:34 (job 3686231); human s0
(resubmitted) at 0.57M counter, milestone not yet reached.

### {RLPD} pixels — P-af-5: 47–51k of 250k decisions at 00:35 (~4 decisions/s, half the smoke's rate; four jobs on
two shared nodes); finals and their dependent evals ≈ Monday afternoon.

## Reading at 00:35 (descriptive; 0.5M is the FIRST of two milestones)

- **The contrastive (r2dreamer) loss also solves the task from pixels**: 5/5 scored seeds reach `home` on 40–63 % of
  random starts at 0.5M, both arms — the pixel observation + augmentation, not the reconstruction loss, is what the
  state-based runs lacked (P-af-2's "R2 loss works from pixels" branch, so far).
- **Machine seeds ignite on the cluster too** (dreamer s4/s5 0.63/0.57; r2dreamer s0/s1 0.50/0.40): 4/4 machine seeds
  scored, consistent with the local (ae) machine seeds (P-af-1 / P-ae-1 direction).
- Human dreamer s5 reads 0/30 at 0.5M while picking 21/30 — one cell; the 1M cell and the local seeds' pattern (dead
  checkpoints right after take-off) say wait for the second milestone before reading it as a non-igniting seed.
- Nothing here is a contrast yet: one milestone per seed, cells still landing. The 1M cells (~03:30) complete the
  registered readout.

## Local (ae) beside it — {dv3 local} machine s2 (started 21:14 09-13)

Series `home` (hold15 | rnd30 MODE): 0.1M 0|0, 0.2M 0|1, 0.3M 0|0 (dead: picked 0/15, 0/30), 0.4M 0|0 (dead: picked
0/15, 2/30), **0.5M 11/15 | 13/30**. Training last-30 at 0.55M: picked 1.00, home 0.90, tipped 0.07; 169 training homes
by 722 episodes. Two consecutive dead checkpoints (0.3M, 0.4M) right at take-off — the largest collapse window seen so
far; the post-hoc sampled series will size it.
