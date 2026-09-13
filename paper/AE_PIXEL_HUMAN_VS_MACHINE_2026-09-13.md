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
| s1 | human | 0, 0, 0, 0, 14, 13, 6, 8, 9, 10 | 0, 0, 0, 0, 18, 12, 9, 6, 14, 10 | **69/240 = 0.288** | 60/120 = 0.500 |
| s1 | machine | 0, 1, 2, 5, 0, 14, 15, 7, 7, 14 | 0, 0, 4, 11, 0, 14, 20, 7, 16, 17 | **89/240 = 0.371** | 64/120 = 0.533 |
| s2 | human | (launched 14:46, PID 1638622) | | | |
| s2 | machine | | | | |
| s3 | human | | | | |
| s3 | machine | | | | |

Tipped (rnd30 MODE cells, 0.3–0.9M): human s0 9, 7, 7, 9, ?, ?, ?; machine s0 7, 7, 18, 9, 13, 15, 24 — the machine
seed's slides end in a tip far more often (at 0.5M: 12 of 18 slides tipped v 0 of 18 for the human seed).

### Interim at 2 v 2 (2026-09-13 14:50, descriptive)

Statistic of record: **human 0.371, 0.288 (mean 0.33) · machine 0.125, 0.371 (mean 0.25)**. The arm difference
(0.08) is smaller than the within-arm spread (human 0.08, machine 0.25); machine s1's series is indistinguishable
from human s0's (both 0.371; final cells 14/15 and 17/30 v 15/15 and 16/30). Machine s0 — the tip-then-stop seed —
is the only one of four outside the band so far. P-ae-1 (ignition, ≥ 3/4 seeds per arm): 2/2 and 2/2 so far. P-ae-3
(machine ignites no later): first hold15 `home` snapshot — human 0.3M, 0.5M; machine 0.3M, 0.2M — holds so far.
Nothing here is a claim: two seeds per arm, and the collapse/recovery swings within a run are as large as any gap.

## Rise time (user's ask, 2026-09-13 17:00: "how early a seed takes off")

Online steps (counter minus the prefill origin) at which the TRAINING record's rolling-30-episode rate first reaches
the threshold (dense in time, sampled actions, the policy's own starts), and the first SERIES snapshot whose cell
meets the condition (sparse, 100k apart, MODE actions). Script: `baselines/diagnostics/ae_rise_time.py`.
**Rise time of record = first online step with rolling-30 `home` ≥ 0.5** (the sustained take-off); the others are
the companions. Registered as a secondary in (ae) ("ignition snapshot") and the quantity P-ae-3 is about.

| seed | picked ≥ 0.5 | home ≥ 0.1 | home ≥ 0.3 | **home ≥ 0.5** | first `home` episode | series: first rnd30 `home` ≥ 1 | series: first hold15 `home` ≥ 5 |
|---|---|---|---|---|---|---|---|
| human s0 | 227k | 189k | 244k | **329k** | 188k | 304k | 304k |
| human s1 | 229k | 241k | 327k | **354k** | 222k | 502k | 502k |
| human s2 | 158k | 216k | 337k | **360k** | 159k | (running) | (running) |
| machine s0 | 208k | 254k | 315k | **321k** | 236k | 102k | 403k |
| machine s1 | 169k | 212k | 296k | **303k** | 205k | 303k | 403k |

Reading at 3 v 2: every seed takes off between 0.30M and 0.36M online steps (rolling `home` ≥ 0.5), inside a 60k
band; the machine seeds are 20–50k earlier on that measure and 20–60k earlier to sustained picking, the human seeds
20–80k earlier to the first `home` episode. Differences of that size are one or two rolling windows and well inside
the seed spread — P-ae-3 ("machine no later") holds so far but is not a finding. The series ignition column lags the
training measure by 0–150k because the 100k grid and the collapse windows (human s1's 0.3–0.4M) fall where they fall.

## Milestone cells, parent protocol (SAMPLED actions)

| seed | arm | 0.5M hold15 / rnd30 `home` | 1M hold15 / rnd30 `home` |
|---|---|---|---|
| s0 | human | 11/15 (4 tipped) / 14/30 (11 tipped, 5 timeout) | **15/15** / 17/30 (9 tipped, 4 timeout) |
| s0 | machine | 11/15 (3 tipped, 1 timeout) / 10/30 (15 tipped, 5 timeout) | 1/15 (12 timeout, 2 tipped) / 3/30 (21 timeout, 6 tipped) |
| s1 | human | 13/15 (2 tipped) / 13/30 (13 tipped, 4 timeout) | 7/15 (7 tipped, 1 timeout) / 11/30 (12 tipped, 7 timeout) |
| s1 | machine | 0/15 (15 timeout) / 0/30 (23 timeout, 7 tipped) — a dead checkpoint, see the note | 12/15 (2 tipped, 1 timeout) / 16/30 (8 tipped, 6 timeout) |

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

## Dead checkpoints (a protocol note, 2026-09-13 12:00, corrected 13:00)

Twice so far a series checkpoint's hold15 MODE cell read **0/15 with all 15 episodes timing out** while its own
60-episode record window read picked 0.63 / home 0.35 (human s1 `ck_517892`, 0.4M) and picked 0.65 / home 0.52
(machine s1 `ck_653443`, 0.5M). My first reading ("the deterministic action is degenerate while the stochastic policy
performs") is **withdrawn** for the machine case: the 0.5M MILESTONE checkpoint (counter 649 952, 3.5k frames before
`ck_653443`) evaluated with SAMPLED actions also reads picked 0/15 (15 timeouts) and 4/30 (23 timeouts, 7 tipped),
`home` 0 — the policy at that checkpoint does nothing under either action mode. So these are short, complete
collapses (a few tens of episodes — invisible in a 30-episode rolling window at 0.40, decisive in a checkpoint) of the
kind `R2D_LIVE_VS_RELOAD` documented at longer timescales; the run recovers within ~50 episodes (machine s1's next
snapshots: 14/15 and 15/15). The statistic of record stays as registered (MODE series; both arms carry the same
exposure). **Planned post hoc, symmetric across all eight seeds, after the chain finishes:** sampled-action cells on every
series checkpoint (all saved), reported beside the MODE series as a secondary — not in place of it — and the
collapse count per seed as a descriptive.

## Records

Runs: `~/runs_dv3_local/dv3px_sparse10_<set>_rlDreamer_s<seed>/` with `LAUNCH_px_ae_<arm>_s<seed>.txt`; series
`~/runs_dv3_local/dv3px_sparse10_series_<arm>_s<seed>/ck_<counter>/` (human s0: `dv3px_sparse10_series/`) with
`record_last60_before.txt` beside each; milestone cells `dv3px_sparse10_eval_m{500k,1000k}[_dM_s0]/`; chain log
`~/runs_dv3_local/ae_chain.log`; sequencer log `ae_queue.log`.
