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
| s1 | 0.67 (20/30) \| 0.93 (14/15) — read 06:47 | 0.53 (16/30) \| 0.93 (14/15) — read 06:47 |
| s2 | 0.60 (18/30) \| 0.93 (14/15) — read 06:47 | 0.63 (19/30) \| 0.80 (12/15) — read 06:47 |

**r2dreamer-loss sparse10, 3 v 3, both milestones in (06:47).** Two-milestone mean of rnd30 MODE `home` (0.5M, 1M):
human s0 18.5, s1 19.5, s2 18.0 of 30 (mean 18.7/30 = **0.622**); machine s0 15.5, s1 14.0, s2 18.5 of 30 (mean
16.0/30 = **0.533**). Gap 0.09 toward human; within-arm ranges 1.5/30 (human) and 4.5/30 (machine). Every one of the
twelve cells is ≥ 12/30 and every hold15 cell ≥ 7/15 — no dead checkpoint among the six seeds, where the dreamer-loss
seeds show 2/30, 0/30 and 1/15 cells. Ignition 3/3 and 3/3.

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

## (af) READOUT — 2026-09-14 06:50: every world-model run has both milestones scored (16 runs × 0.5M/1M × rnd30/hold15 MODE)

**Rise time on the TRAINING record** (first online step at which the rolling-30 `train_ep_home` ≥ 0.5; the (ae)
"rise time of record"; parsed from each run's `console.log`, origin = the run's first counter line):

| learner / arm | seeds → rise (online steps) | first `home` episode |
|---|---|---|
| {dreamer} human s4 s5 s6 s7 | **507k, 526k**, 258k, 299k | 222k, 324k, 182k, 284k |
| {dreamer} machine s4 s5 s6 s7 | 343k, 300k, 338k, 447k | 225k, 222k, 278k, 237k |
| {r2dreamer} human s0 s1 s2 | 293k, 257k, 290k | 154k, 169k, 238k |
| {r2dreamer} machine s0 s1 s2 | 258k, 308k, 240k | 178k, 151k, 154k |
| ramp human s0 / machine s0 | 448k / 341k | 307k / 165k |

- **P-af-1 (cross-machine, dreamer losses): MET.** 8/8 cluster seeds ignite by 1M; 6/8 inside the registered 0.2–0.5M
  window (human s4/s5 at 0.507M/0.526M are the exceptions, 1–3 rolling windows past it). The ≥ 3-of-8 pooling
  condition holds, so the local and cluster dreamer seeds may be pooled for P-af-4 — with the protocol difference
  stated: the local per-seed statistic averages EIGHT series cells (0.3–1.0M), the cluster one TWO (0.5M, 1M).
  Local rise times were 0.30–0.36M (five seeds); the cluster human seeds spread wider (0.26–0.53M).
- **P-af-2 (contrastive loss from pixels): MET, 3/3 and 3/3** (criterion ≥ 2 of 3 per arm by 1M). The r2dreamer-loss
  seeds are the best pixel policies of the batch: two-milestone `home` 0.622 (human) / 0.533 (machine) v 0.329 / 0.379
  for the dreamer losses, rise 240–308k on all six, no cell below 12/30. The pixel result is NOT specific to the
  reconstruction loss; report as "world model from pixels, both representation losses".
- **P-af-3 (ramp control, 2 seeds under rev 1 instead of the 4 the clause was written for): PARTLY met, and the
  reading it was written to give is "both ingredients".** Cells (slide_event | home | tipped): human s0 0.5M hold15
  9/15 | 9/15 | 3/15, rnd30 14/30 | 13/30 | 11/30; 1M hold15 **15/15 | 2/15 | 0/15** (13 timeouts), rnd30 20/30 | 2/30 |
  8/30. Machine s0 0.5M hold15 15/15 | 1/15 | **14/15**, rnd30 21/30 | 1/30 | **26/30**; 1M hold15 6/15 | 6/15 | 1/15,
  rnd30 12/30 | 12/30 | 8/30. Clause (i) `slide_event` ≥ 0.5 on hold15 by 1M: 2/2 seeds at some milestone, 1/2 on the
  1M cell itself. Clause (ii) tipped above the sparse10 seeds (sparse10 1M rnd30 tipped: dreamer human 0.40, machine
  0.28; r2dreamer 0.23 / 0.28): the machine ramp seed at 0.5M tips 26/30 = 0.87 — the state-ramp signature — then
  reads 0.27 at 1M; the human ramp seed never exceeds the band. So under pixels the ramp DOES reach `home` (13/30,
  12/30 at its better milestone, inside the dreamer-sparse10 band), which says "+10 alone" is not the active
  ingredient; and it also shows what the state ramp showed — sliding far without nesting (human 1M: 20/30 slides,
  2/30 home) and a tipping checkpoint (machine 0.5M) — which the sparse10 seeds do not. Two-milestone `home` means
  6.5/30 and 7.5/30 sit at the bottom of the sparse10 range (6.0–16.5). n = 1 per arm, 2 cells each: the ramp is
  learnable from pixels but less reliably; pixels are what fixed the perception, and the terminal +10 is what makes
  the finish consistent. Not a source contrast.
- **P-af-4 (pooled 8 v 8, ±0.15 margin, exact permutation): waits for the local 4 v 4** (human s3 running, machine s3
  next; ≈ 17:00 09-14). Cluster dreamer 4 v 4 alone: human 0.329 v machine 0.379 (two-milestone means).
- **P-af-5 (RLPD pixels): pending** (165k of 250k at 06:47; Q-watchdog 20–27 on all four).

## P-af-5 — {RLPD} pixels, the (af) 2 v 2 at 250k decisions (final checkpoint; read 2026-09-14 15:35)

`home` (hold15 MODE | rnd30 MODE | rnd30 SAMPLED), outcomes from the cells' `per_episode`:

| seed | human `dH` | machine `dDPfirst` |
|---|---|---|
| s0 | 1/15 \| 0/30 \| 0/30 (picked 6/15, 4/30; 11 tipped on rnd30) | 0/15 \| (cell running) \| 0/30 (picked 0/15, 0/30) |
| s1 | 0/15 \| 0/30 \| 0/30 (picked 1/15, 3/30) | **10/15 \| 16/30 \| 19/30** (picked 15/15, 20/30; 9 tipped) |

Verdict on the registered letter (ignition = ≥ 1 `home` in any final cell, ≥ 1 of 2 seeds per arm): **met on
both arms (human s0 by a single hold15 episode; machine s1 outright).** Substance: ONE of four RLPD-from-pixels seeds
solves the task at 250k decisions — machine s1 reaches `home` on 53–63 % of random starts, on par with the world
models at 1M — while the other three barely pick (4–6 of 30). RLPD from pixels is learnable at this budget with this
encoder, and it is the least reliable of the three learners here (1/4 v 21/21 world-model seeds); the Q-watchdog
(Q 20–40 against a max return of 10, all four runs) did not separate the igniting seed from the others (its last
value 20.0, human s0's 20.2). The 28 (ag) seeds with 25k-step checkpoints will say whether the three quiet seeds
ignite later or never. (The merged `rnd30_mode_iso/metrics.json` headline for machine s1 reads picked 1.0 / home 1.0
while its per-episode outcomes are the shared cell's 16 home / 9 tipped / 5 timeout — the merged headline field is
wrong or means something else; use the shared cell and the per-episode counts.)

## (ag) cells — 2M runs, milestones 0.5M / 1M / 1.5M / 2M (rnd30 MODE `home`, count of 30; read 15:35 09-14)

{dreamer losses}, `nested_sparse10`, the 16 new seeds:

| seed | human 0.5M → 1M | machine 0.5M → 1M |
|---|---|---|
| seed | human 0.5M → 1M → 1.5M → 2M | machine 0.5M → 1M → 1.5M → 2M |
| s8 | 1 → 4 → 8 → 7 | 18 → 9 → 8 → 16 |
| s9 | 8 → 10 → 9 → 6 | 12 → 7 → · → · |
| s10 | 10 → 10 → 0 (picked 20) → 3 | **0 (picked 0)** → 15 → 5 → 13 |
| s11 | 0 (picked 10) → 12 → 12 → 12 | 6 → 3 → 4 → 18 |
| s12 | 12 → 2 → 8 → **0 (picked 17)** | **0 (picked 0)** → 9 → 10 → 14 |
| s13 | 1 (picked 2) → 9 → 13 → 17 | 13 → 17 → 16 → 17 |
| s14 | 11 → 3 → 16 → · | 9 → 6 → 6 → **0 (picked 19)** |
| s15 | 8 → 13 → 17 → 5 | 18 → 4 → 16 → 16 |

(read 21:50 09-14; s9 human 2M 6, s14 human 2M 13, s9 machine 2M 10 landed 21:47; the `final` cell = the 2M
checkpoint, counted once; all 16 dreamer-loss runs COMPLETED 0:0 by 20:40, 8.6–9.5 h each; machine s9's 1.5M cell
is the one cell still unscored.)

### {dreamer losses} — READOUT of the (ag) 8 v 8 at 2M and the DREAMER 16 v 16 (2026-09-14 21:50)

| statistic (per seed, rnd30 MODE `home`) | human mean | machine mean | Δ (H−M) | exact two-sided permutation p |
|---|---|---|---|---|
| (ag) 8 v 8, single 2M cell | 0.263 | 0.433 | −0.171 | 0.106 |
| (ag) 8 v 8, pooled 0.5M + 1M cells (the registered pooled statistic) | 0.238 | 0.304 | −0.067 | 0.266 |
| (ag) 8 v 8, four-milestone mean (machine s9 over 3 cells) | 0.271 | 0.338 | −0.067 | 0.207 |
| **DREAMER 16 v 16, pooled 0.5M + 1M** (local s0–3 + (af) s4–7 + (ag) s8–15) | **0.306** | **0.304** | **+0.002** | **0.98** (Monte-Carlo, 200k) |

P-ag-1 (pooled dreamer-loss 16 v 16 inside ±0.15): **MET** — Δ +0.002 on the statistic every seed has; the three
batches lean human (local), machine ((af), (ag)) and cancel. The 2M single-cell read leans machine at p 0.11 and the
(ae) local series leaned human at p 0.40 — same learner, same recipe, opposite signs; neither is a finding, and the
hardware difference of the local seeds (user: to be noted in the paper) does not change the pooled null. P-ag-3
(2M ≥ 1M on average): 2M mean 0.348 v 1M mean 0.283 over the 16 (ag) seeds — holds, weakly, and with cells swinging
0 ↔ 17 on the same seed. Nothing below n = 16 v 16 separates the demonstration sources for the pixel world model.

{r2dreamer loss}, `nested_sparse10`, the 13 new seeds per arm (same columns):

| seed | human 0.5M → 1M → 1.5M → 2M | machine 0.5M → 1M → 1.5M → 2M |
|---|---|---|
| s3 | 20 → 20 → 18 → 18 | 17 → 21 → 16 → 16 |
| s4 | 16 → 17 → 20 → 10 | 10 → 20 → 22 → 16 |
| s5 | 17 → 17 → 14 → 17 | 14 → 10 → 20 → 18 |
| s6 | 19 → 18 → 21 → 19 | 16 → 19 → 17 → 17 |
| s7 | 17 → 18 → 20 → 15 | 18 → 18 → 9 → 11 |
| s8 | 18 → 15 → 20 → 19 | 14 → 18 → 18 → 17 |
| s9 | · → 10 → · → · | · (cells queued) |
| s10 | 5 → 18 → 20 → 17 | 11 → 19 → 18 → 16 |
| s11 | 20 → 21 → 19 → 16 | 21 → 19 → 19 → 9 |
| s12 | 21 → 19 → 16 → 16 | 9 → 10 → 19 → 18 |
| s13 | 20 → 19 → 20 → 18 | 14 → 15 → 16 → 21 |
| s14 | 8 → 17 → 13 → 19 | 16 → 17 → 15 → 17 |
| s15 | 20 → 22 → 16 → · | 15 → 17 → 17 → 22 |

(`·` = cell not yet scored; read 12:25 09-15; all 26 r2dreamer (ag) runs COMPLETED 0:0, 0 preemptions.)

### {r2dreamer loss} — READOUT MOMENT at 15 v 15 (2026-09-15 12:25; s9 both arms pending → 16 v 16 next check)

| statistic (per seed, rnd30 MODE `home`) | human mean | machine mean | Δ (H−M) | permutation p (MC 300k) |
|---|---|---|---|---|
| **pooled 0.5M + 1M cells, (af) s0–2 + (ag) s3–8, s10–15** | **0.593** (0.383–0.700) | **0.527** (0.317–0.667) | **+0.067** | **0.059** |
| (ag) single 2M cell, 11 v 12 | 0.558 | 0.550 | +0.008 | 0.90 |

Reading: on the pooled statistic the human arm reads 0.07 higher at p 0.06 — inside the registered ±0.15 margin
(P-ag-2), so "no effect detectable" is the sentence, with the honest note that this is the one condition where the
sign is stable and the p small. On the 2M cells the arms are identical (0.558 v 0.550). The two arms' seed ranges
overlap almost entirely; the machine arm's lowest seeds (s12 0.317, s5 0.40) are seeds whose 0.5M or 1M cell caught a
dip (s12: 9, 10 at 0.5M/1M then 19, 18 at 1.5M/2M) — the checkpoint lottery, not a non-igniting seed (rise time 240–308k
on all). Every one of the 30 seeds ignites (P-ag-2's ignition clause met). Both arms sit on a 15–21/30 plateau from
1M on; the contrastive loss is the learner to lead the pixel table with. The two machine seeds that read 0 with no picks at 0.5M read 15/30
and 9/30 at 1M — late rise, not dead seeds; and s12 human / s15 machine drop 12 → 2 and 18 → 4 between milestones —
the oscillation again. {r2dreamer loss} s3–s15: first 0.5M milestones scored from ~17:00.

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
