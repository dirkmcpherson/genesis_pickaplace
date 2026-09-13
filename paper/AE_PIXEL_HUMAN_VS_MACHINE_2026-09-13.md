# Amendment (ae) — human vs machine demonstrations for the pixel world model: running tally (started 2026-09-13 01:55)

Design and predictions: `paper/PHASE_PLAN_2026-09-04.md` amendment (ae). Learner: {dv3 local} = DreamerV3 losses in
the r2dreamer chassis, pixels (top ++ wrist 64×64) + 8-dim proprioception, `image_aug shift4`, `nested_sparse10`,
`tip_guard not_in_hand`, fresh world model per seed, 1M online steps, one run at a time on pop-os (~6.5 h each).
Sets: human `dHfull_all_rns10h_img` (74 tapes, Σ 130, 13 `home`), machine `dDPfull_first_rns10h_img` (72 tapes, Σ 140,
14 `home`; DP teacher trained on the pruned human set, first attempt per start; both built from the LOCAL stage records).

**Statistic of record (per seed):** mean `home` rate over the rnd30 MODE series cells at 0.3M–1.0M online steps
(8 cells × 30 episodes; fresh process, deterministic actions, training world, `[sim-variant]` on every cell).
Secondary: hold15 MODE the same way; ignition snapshot; tipped fraction. The number is the SERIES — never one
checkpoint (`R2D_LIVE_VS_RELOAD_2026-09-12.md`).

## Series, `home` per cell (snapshots at ~0.1M … 1.0M online steps; MODE cells)

| seed | arm | hold15 (of 15) | rnd30 (of 30) | statistic of record (rnd30 0.3–1.0M) | hold15 mean 0.3–1.0M |
|---|---|---|---|---|---|
| s0 | human | 0, 0, 5, 1, 12, 13, 11, 6, 12, 15 | 0, 0, 7, 3, 18, 16, 10, 8, 11, 16 | **89/240 = 0.371** | 75/120 = 0.625 |
| s0 | machine | 0, 0, 4, 8, 4, 10, 5, 0, 0, 2 | 1, 0, 6, 5, 6, 8, 4, 0, 0, 1 | **30/240 = 0.125** | 33/120 = 0.275 |
| s1 | human | (launched 01:56, PID 1374701) | | | |
| s1 | machine | | | | |
| s2 | human | | | | |
| s2 | machine | | | | |
| s3 | human | | | | |
| s3 | machine | | | | |

Tipped (rnd30 MODE cells, 0.3–0.9M): human s0 9, 7, 7, 9, ?, ?, ?; machine s0 7, 7, 18, 9, 13, 15, 24 — the machine
seed's slides end in a tip far more often (at 0.5M: 12 of 18 slides tipped v 0 of 18 for the human seed).

## Milestone cells, parent protocol (SAMPLED actions)

| seed | arm | 0.5M hold15 / rnd30 `home` | 1M hold15 / rnd30 `home` |
|---|---|---|---|
| s0 | human | 11/15 (4 tipped) / 14/30 (11 tipped, 5 timeout) | **15/15** / 17/30 (9 tipped, 4 timeout) |
| s0 | machine | 11/15 (3 tipped, 1 timeout) / 10/30 (15 tipped, 5 timeout) | (queued) |

## Reading so far (n = 1 v 1 — descriptive only)

- Both seeds ignite on the same schedule (first `home` cells at the 0.3M snapshot: 5/15 v 4/15, 7/30 v 6/30) —
  consistent with P-ae-1 and with P-ae-3's "no later" clause so far.
- After ignition the human seed climbs to a 12–15/15 plateau with a single trough at 0.8M; the machine seed peaks at
  10/15 (0.6M) and then collapses to tipping (0/15, 0/30 at 0.8–0.9M; 24 of 30 random starts tipped at 0.9M) before a
  partial recovery at 1.0M (hold15 15/15 picked and placed, 2/15 `home`, 0 tipped — it stops pushing).
- Machine s0's 1.0M cell: hold15 15/15 picked, placed and far side but only 2/15 slides (0 tipped); rnd30 picked 25/30,
  `home` 1/30, 22 timeouts, 7 tipped — the policy has stopped pushing rather than pushing into tips.
- Difference in the statistic of record at n = 1 v 1: 0.371 − 0.125 = 0.246 in favour of human, outside P-ae-2's ±0.15 — this
  is a direction to test, not a finding; the same learner's own seed-to-seed swings (this run's 0.8M trough) are of the
  same size as the arm gap. Seeds 1–3 decide.
- The behavioural difference worth watching across seeds is the finish of the slide: same slide frequency, different
  tip rate. If it holds, the candidate mechanism is in the demonstrations' push geometry (DP-teacher pushes v human
  pushes), which the stage records can quantify offline (`slide_gain_m`, tool–can lever at set-down) without any new run.

## Records

Runs: `~/runs_dv3_local/dv3px_sparse10_<set>_rlDreamer_s<seed>/` with `LAUNCH_px_ae_<arm>_s<seed>.txt`; series
`~/runs_dv3_local/dv3px_sparse10_series_<arm>_s<seed>/ck_<counter>/` (human s0: `dv3px_sparse10_series/`) with
`record_last60_before.txt` beside each; milestone cells `dv3px_sparse10_eval_m{500k,1000k}[_dM_s0]/`; chain log
`~/runs_dv3_local/ae_chain.log`; sequencer log `ae_queue.log`.
