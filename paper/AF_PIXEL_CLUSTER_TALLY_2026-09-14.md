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
| s5 | **0.00 (0/30; picked 0.70)** \| **0.00 (0/15; picked 1.00)** \| (cell pending) | 0.57 (17/30) \| 1.00 (15/15) \| 0.67 |
| s6 | 0.43 (13/30) \| 0.67 (10/15) \| 0.50 | 0.33 (10/30) \| 0.73 (11/15) \| (pending) — read 01:35 |
| s7 | 0.50 (15/30) \| 0.80 (12/15) \| (pending) — read 01:35 | **0.07 (2/30; picked 0.73)** \| **0.13 (2/15; picked 1.00)** \| (pending) — resubmitted seed, read 02:35 |

## 1M online — `home` rate (rnd30 MODE | hold15 MODE), landing from 02:35 (jobs COMPLETED 0:0 after 4.1–5.1 h)

### {dreamer losses}, `nested_sparse10`

| seed | human | machine |
|---|---|---|
| s4 | 0.43 (13/30) \| 0.47 (7/15) | 0.30 (9/30) \| **0.07 (1/15)** — read 03:47 |
| s5 | **0.50 (15/30) \| 0.60 (9/15)** — recovered from 0/30, 0/15 at 0.5M | 0.53 (16/30) \| 0.73 (11/15) — read 03:47 |
| s6 | 0.33 (10/30) \| 0.53 (8/15) — read 04:47 | 0.27 (8/30) \| 0.40 (6/15) — read 04:47 |
| s7 | **0.07 (2/30; picked 0.60)** \| 0.13 (2/15) — read 04:47/05:47 (0.5M was 15/30) | 0.33 (10/30) \| 0.67 (10/15) — read 05:47 |

**Dreamer-loss sparse10, 4 v 4, both milestones in (05:47).** Two-milestone mean of rnd30 MODE `home` (0.5M, 1M), per
seed: human s4 12.0, s5 7.5, s6 11.5, s7 8.5 of 30 (mean 9.9/30 = **0.329**); machine s4 14.0, s5 16.5, s6 9.0, s7 6.0
of 30 (mean 11.4/30 = **0.379**). Arm gap 0.05 in favour of machine; within-arm ranges 4.5/30 (human) and 10.5/30
(machine). Beside the local (ae) 3 v 3 at the same recipe (ten-point series: human 0.339, machine 0.267), the two
batches lean opposite ways by less than either's within-arm spread — P-af-1's "same picture as local" reading: no arm
separation at this n, on either machine class. Ignition (any `home` cell ≥ 5/30 at either milestone): 4/4 and 4/4.

### {r2dreamer loss} at 1M (read 05:47)

| seed | human | machine |
|---|---|---|
| s0 | **0.67 (20/30) \| 0.93 (14/15)** | 0.53 (16/30) \| 0.73 (11/15) |
| s1 | (queued) | (queued) |
| s2 | (queued) | (queued) |

### ramp control at 1M: **machine s0 0.40 (12/30) \| 0.40 (6/15)** (read 03:47; up from 1/30 at 0.5M); **human s0
0.07 (2/30; picked 0.67) \| 0.13 (2/15)** (read 04:47/05:47; down from 13/30 at 0.5M). The two ramp seeds swap places
between the milestones — the checkpoint lottery of `R2D_LIVE_VS_RELOAD` at full size; per seed, the two-milestone
mean is 6.5/30 (machine) and 7.5/30 (human), both below the sparse10 seeds' two-milestone means so far (human s4 12,
s5 7.5, s6 11.5, s7 8.5; machine s4 14, s5 16.5, s6 9 of 30). P-af-3 is read on those means once every cell exists.

**Note on the 1M cells (04:47):** of the seven dreamer sparse10 seeds with both milestones, six read LOWER at 1M than
at 0.5M (human s7 15 → 2, machine s4 19 → 9 the largest drops); the local (ae) series show the same run-to-run swings
(machine s2: 21/30 at 0.8M, 11/30 at 1.0M) with no trend over 0.5–1.0M, so this is the oscillation sampled at two
points, not a decline — but two points per seed is all this batch has, which is why the (ae) local series (ten
points per seed) remains the statistic the human-v-machine question is read on.

{r2dreamer loss}: all six jobs COMPLETED, 1M cells queued (8 `lnms_` pending, 8 running at 03:47). The `final`
cells are the same checkpoint as `online_1000000` (the run ends at 1M) and read identically — count them once.
Jobs: **all 16 world-model runs COMPLETED 0:0 by 05:47** (4.1–5.1 h each; no preemption in the whole batch); all 32
0.5M cells exist; 1M rnd30 MODE cells exist for 12 of 16 runs at 05:47 (r2dreamer s1/s2 both arms queued).

### {r2dreamer = the port's contrastive representation loss}, pixels, `nested_sparse10` — P-af-2

| seed | human | machine |
|---|---|---|
| s0 | 0.57 (17/30) \| 1.00 (15/15) \| 0.40 | 0.50 (15/30) \| 1.00 (15/15) \| 0.50 |
| s1 | 0.63 (19/30) \| 0.60 (9/15) \| 0.57 | 0.40 (12/30) \| 0.47 (7/15) \| 0.43 |
| s2 | 0.60 (18/30) \| 0.47 (7/15) \| 0.57 | 0.60 (18/30) \| 1.00 (15/15) \| (pending) — read 01:35 |

### ramp control (`nested_ramp`, dreamer losses) — P-af-3
Machine s0 at 0.5M (read 01:35): rnd30 MODE `home` **0.03 (1/30)**, hold15 MODE **0.07 (1/15)**, picked 0.70 / 1.00 —
the pixel world model picks and places under the ramp but barely arrives home, where every sparse10 seed but two is at
0.33–0.63 at the same milestone (one seed, one milestone). **Human s0 at 0.5M (read 02:35): rnd30 MODE 0.43 (13/30),
hold15 MODE 0.60 (9/15)** — the human ramp seed reaches home at the sparse10 seeds' rate while the machine ramp seed
does not (1 v 1, one milestone; the 1M cells decide whether this is the ramp or the seed).

### {RLPD} pixels — P-af-5: 47–51k of 250k decisions at 00:35 (~4 decisions/s, half the smoke's rate; four jobs on
two shared nodes); finals and their dependent evals ≈ Monday afternoon. **01:35 — Q-WATCHDOG on all four runs** (7
firings each, every 10k decisions from ~10k): mean actor-state Q 20–39 against a maximum task return of 10 (dH s0
38.1 → 38.3, dH s1 38.8 → 27.0, dDPfirst s0 21.3 → 28.2, dDPfirst s1 28.6 → 20.0 over the last two checks). It is a
warning, not an abort — the runs continue — and the same class of value inflation as the local γ0.998 RLPD failures of
09-01 (Q → 3 082 there), far milder here and not monotonic. Read the 250k finals with this in hand; a policy that still
solves the task at the end is the result of record, an exploding Q is the disconfirm branch of P-af-5's "RLPD from
pixels" clause.

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
0/15, 2/30), **0.5M 11/15 | 13/30, 0.6M 13/15 | 17/30, 0.7M 0/15 | 0/30 (dead: picked 0/15, 4/30), 0.8M 14/15 |
21/30** (read 02:35; 21/30 is the best rnd30 cell of any local seed so far — the previous best was machine s1's 20/30).
Three dead checkpoints in one run (0.3M, 0.4M, 0.7M), each between cells at 0.43–0.70. Training last-30 at 0.55M: picked 1.00, home 0.90, tipped 0.07; 169 training homes
by 722 episodes. Two consecutive dead checkpoints (0.3M, 0.4M) right at take-off — the largest collapse window seen so
far; the post-hoc sampled series will size it.
