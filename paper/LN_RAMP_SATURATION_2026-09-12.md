# Ladder N readout, 2026-09-12 ~05:00 — {RLPD} ignites on `nested_ramp` (human arm); {r2dreamer} saturates the ramp and stops

Read from the cells that exist at 04:45 EDT. Every cell below carries the stamp
`ladder=nested_ramp | picked=1 placed_v2=1 home=4 ramp:slide_gain_m=3/0.05m | max_return=9 | tip=tilt>60deg&not_in_hand@4f`
(or the `staged` stamp for the control seeds). Census script: `cluster/ln_cell_census.py`
(§6 has the commands). Absent cells are listed as absent, not as zero.

**Bottom line.** Same ladder, same demonstration sets, same guard, same horizon (300 decisions):

- **{RLPD} `nested_ramp`, human `dHfull_all_rnrh`, 250k decisions, final checkpoint:** seed s950 reaches `home`
  in **all six** of its cells — hold15 MODE **2/15**, hold15 sampled 1/15, rnd30 MODE 1/30, rnd30 sampled 1/30,
  spots60 MODE 7/60, spots60 sampled 6/60 (12 `home` in 210 episodes, 11 of them with reward ≥ 7.9). Seed s951:
  3 `home` in 210 episodes, all in spots60 (MODE 2/60, sampled 1/60), none in hold15/rnd30. Both human seeds
  satisfy the rev-3 "≥ 1 `home` in an evaluation cell" clause.
- **{RLPD} `nested_ramp`, machine `dDPfull_first_rnrh`, 250k:** s970 `home` 0 and `slide_event` 0 in all six cells
  (farside up to 8/15). s971 still training (8.6 h in) — ABSENT.
- **{r2dreamer} `nested_ramp`, 2M online steps:** `home` 0 in every cell of every seed (pooled: s950 0/120,
  s951 0/150, s970 0/150, s971 0/225). But the 2M policies DO slide: s950 (human) hold15 MODE `slide_event`
  **12/15**, s971 (machine) rnd30 MODE 11/30. **Their episode reward is pinned at exactly 5.000** — picked 1 +
  placed_v2 1 + the full ramp 3 — in 34 of the 38 `slide_event` episodes across those four cells. The ramp pays
  3 × min(1, gain/0.05 m): five centimetres of goalward gain collects everything it will ever pay, and the
  policies stop there. The human seed then idles to timeout (10 of 12 hold15 slides end `timeout` at 300
  decisions); the machine seed tips the can (8 of 11 rnd30 slides end `tipped`).
- **{dv3, local: DreamerV3 losses in the r2dreamer chassis} `nested_ramp`, human, 2M:** 1 `home` in 90 episodes
  (rnd30 MODE ep 29, reward 7.69; `paper/DV3_LOCAL_E2E_COLLAPSE_2026-09-11.md`).

So the world-model learners are not failing to find the slide any more — they find it, take the ramp to
saturation, and never take the last 5–7 cm that pay the +4. {RLPD} does, on the human seeds. §4 says what I
think that means and what NOT to conclude from it.

## 1. {RLPD} 250k finals — `home` per cell (final checkpoint `rlpd_final.zip`, in-job `preview` cells)

`home / n` with `slide_event / n` in parentheses. Seeds s95x = human `dHfull_all_*`, s97x = machine
`dDPfull_first_*`. The two `ctl` seeds per arm are `staged` (picked 1 / placed_v2 1 / contact_push 2 /
slide_success 4) with the same `not_in_hand` guard — rev 3's guard control.

| seed | arm | ladder | hold15 MODE | hold15 sampled | rnd30 MODE | rnd30 sampled | spots60 MODE | spots60 sampled |
|---|---|---|---|---|---|---|---|---|
| s950 | human | nested_ramp | **2/15** (9) | 1/15 (4) | 1/30 (6) | 1/30 (7) | 7/60 (17) | 6/60 (27) |
| s951 | human | nested_ramp | 0/15 (0) | 0/15 (1) | 0/30 (0) | 0/30 (0) | 2/60 (3) | 1/60 (2) |
| s970 | machine | nested_ramp | 0/15 (0) | 0/15 (0) | 0/30 (0) | 0/30 (0) | 0/60 (0) | 0/60 (0) |
| s971 | machine | nested_ramp | ABSENT (training, 8.6 h) | | | | | |
| s958 | human | staged (ctl) | 0/15 (0) | 0/15 (0) | 0/30 (0) | 0/30 (0) | 0/60 (0) | 0/60 (0) |
| s959 | human | staged (ctl) | 0/15 (1) | 0/15 (0) | 0/30 (0) | 0/30 (0) | 0/60 (0) | 0/60 (0) |
| s978 | machine | staged (ctl) | 0/15 (0) | 0/15 (0) | 0/30 (0) | 0/30 (0) | 0/60 (0) | 0/60 (0) |
| s979 | machine | staged (ctl) | 0/15 (0) | 0/15 (0) | 0/30 (0) | 0/30 (0) | 0/60 (0) | 0/60 (0) |
| s955, s956 | human | nested_sparse | ABSENT (training, 8.3–8.6 h; land within the hour) | | | | | |
| s975, s976 | machine | nested_sparse | ABSENT (training, 8.3 h) | | | | | |
| s952, s953 / s972, s973 | human / machine | nested_ramp, 500k (rev 2) | ABSENT (4–5 h of ~17) | | | | | |
| s957, s958 / s977, s978 (`e2e_rev3/`) | human / machine | nested_sparse, 500k (rev 3) | ABSENT (0.7–3.9 h) | | | | | |

Other stages on the ramp seeds (hold15 MODE): s950 picked 15/15, placed_v2 14/15, farside 14/15; s951 15/15,
14/15, 13/15; s970 14/15, 9/15, 8/15. s970 gets to the far side of the can half the time and never gains a
centimetre from there.

The staged control that matters: **s959 (human, staged) nests 9/15 on hold15 MODE (`nested_v2` = `nested_honest`
= 0.60) with `slide_event` 1/15 and `contact_push` 7/15** — the carry-and-drop route the old ladder pays and
Ladder N refuses. Its rnd30 MODE cell contains one reward-8.0 episode (`slide_success` paid once). s958 barely
places (`placed_v2` ≤ 0.10 in every cell). So under the new guard the staged ladder still produces "nesting"
without a slide on one seed of two, and the ramp ladder produces slides-then-`home` on both human seeds. That is
the contrast Ladder N was built to create.

Reward histograms for the s950 cells are spread — hold15 MODE: 1.0 ×1, 2.0 ×5, 2.88, 3.17, 3.43, 3.68 ×2, 3.75,
5.0 ×1, 9.0 ×2. Partial ramp values (2 < r < 5) are the common case: {RLPD}'s slides are mostly SHORT of the 5 cm
that saturates the ramp, and a few go all the way. Compare §2.

Caveats on these cells: they are the launcher's in-job `preview` cells (role stamped in `metrics.json`), one
shared process per cell on the job's node (pax048 for s950, 64-core class by the launcher's guard). The cells
OF RECORD come from the pinned re-score (`cluster/sbatch_e2e_rescore.sh`, `<run>/rec/`), which has not been
run for this batch. For an ignition read (is `home` reachable at all) the preview cells are sufficient; for a
rate they are not.

## 2. {r2dreamer} 2M cells — the ramp saturates

From `$W/ln_milestone_cells/` (`python3 $W/ln14_milestone_table.py`), all on 64/64-core nodes.

| run | milestone | cell | picked | placed_v2 | farside | slide_event | home | tipped | reward = 5.000 among slide_event eps |
|---|---|---|---|---|---|---|---|---|---|
| dHfull_all_rnrh s950 | final (2M) | hold15 MODE | 15/15 | 14/15 | 14/15 | **12/15** | 0/15 | 3/15 | 12 of 12 |
| dHfull_all_rnrh s950 | final (2M) | rnd30 MODE | 16/30 | 12/30 | 11/30 | 9/30 | 0/30 | 15/30 | 6 of 9 (others 4.18, 3.91, 3.63) |
| dDPfull_first_rnrh s971 | online_2000000 | hold15 MODE | 15/15 | 15/15 | 15/15 | 6/15 | 0/15 | 14/15 | 6 of 6 |
| dDPfull_first_rnrh s971 | online_2000000 | rnd30 MODE | 19/30 | 16/30 | 15/30 | 11/30 | 0/30 | 28/30 | 10 of 11 (other 4.17) |
| dDPfull_first_rnrh s971 | online_2000000 | rnd30 sampled | 17/30 | 16/30 | 15/30 | 9/30 | 0/30 | 29/30 | — |
| dHfull_all_rnrh s951 | online_1000000 | hold15 MODE | 7/15 | 4/15 | 2/15 | 0/15 | 0/15 | 10/15 | — |
| dDPfull_first_rnrh s970 | online_1000000 | hold15 MODE | 13/15 | 12/15 | 12/15 | 0/15 | 0/15 | 8/15 | — |

Outcome of the slide episodes: s950 hold15 MODE 10 `timeout` + 2 `tipped`; s971 rnd30 MODE 8 `tipped` + 2
`timeout` + 1 `tipped` at 4.17. The evaluator's legacy `slide_fail_reason` reads `no_contact` (18) / `grip_closed`
(5) on these — that field scores the withdrawn `slide_success` predicate and says nothing about the ramp.

What "reward exactly 5.000" means mechanically: `slide_gain_m` is paid on new minima of the can–goal distance
while `farside` holds, scaled 3/0.05 m, capped at 3. A policy that pushes 5 cm and stops has collected the
whole ramp. The set-down-to-goal distance in the human tapes is 10.7–12.3 cm (SLIDE_ANATOMY), and the
`nested_v2` threshold is centre distance ≤ 8.1 cm, so after a 5 cm push there are typically 2–7 cm left on
which the ladder pays NOTHING until the terminal +4. On the world-model side that +4 has never been
experienced online (0 `home` in every training and evaluation cell), so the imagined return of pushing
further is the return of pushing 5 cm — zero marginal value, and the tip risk of pushing further is real
(s971 tips 28/30). The policy behaves exactly as the reward surface tells it to.

Why {RLPD} is different is not established here. The plausible account: RLPD trains on 50 % demonstration
batches for the whole run, and the human set contains the +4 (13 human tapes reach `home` under this ladder,
`paper/LADDER_N_DEMO_CHECK_2026-09-11.md`), so its critic sees the terminal every batch; r2dreamer sees the demonstrations only in prefill, and the
demonstration fraction of its buffer falls through the run (memory note `dv3-demo-mixing-experiment`). That is
a hypothesis; the test would be demo re-injection (`env.demo_reinject_every`, already plumbed in the trainer)
or a longer-span ramp, and neither has been run.

## 3. Ignition tally against the rev-3 procedure (PHASE_PLAN (aa) REVISION 3)

"Ignited" = ≥ 2 of 4 seeds with ≥ 1 `home` in an evaluation cell, per arm per learner, at the recipe's budget.

| learner | ladder | human | machine | status |
|---|---|---|---|---|
| {RLPD} 250k | nested_ramp | **2 of 2 read** (s950 12/210, s951 3/210); s952/s953 (500k) pending | 0 of 1 read (s970); s971, s972/s973 pending | human arm IGNITED; machine open |
| {RLPD} 250k | nested_sparse | 0 read (s955/s956 land ~05:30) | 0 read (s975/s976) | open |
| {RLPD} 250k | staged + new guard (ctl) | 0 of 2 (s958, s959) | 0 of 2 (s978, s979) | not ignited |
| {r2dreamer} 2M | nested_ramp | 0 of 2 read (s950 final, s951 1M) | 0 of 2 (s970 1M, s971 2M) | not ignited at 2M; 4M extensions running |
| {r2dreamer} 2M | nested_sparse | s956 cells absent (sweep running) | s975 0/75 at 1M | open |
| {dv3 local} 2M | nested_ramp | 1 of 1 (1/90) | — | single seed |

The decision the user asked for ("which of these reward variants has the best chance", then ≥ 16 per arm) is
between `nested_ramp` and `nested_sparse`; the sparse RLPD finals are due within the hour and should be read
before anything is recommended. What is already decidable: the `staged` control does not reach `home` on any
of four seeds and produces drop-route nesting on one of them, so it is not a candidate.

## 4. What this does and does not say

- It does NOT say r2dreamer cannot slide. It slides on 12/15 in-distribution starts at 2M. It says the ramp as
  registered gives a world model no reason to slide past 5 cm, and that the +4 is too far past the saturation
  point for the model to discover it online at 2M.
- It does NOT rank the learners on demonstration source: RLPD human 2/2 vs machine 0/1 is n = 1 on the machine
  side, and s970 does not slide at all (0 `slide_event` in 210 episodes), so the machine RLPD seeds may simply
  not have reached the slide by 250k.
- The 4M r2dreamer extensions (`ln_r2_ramp4M_*`, running) cannot fix a flat reward surface by running longer;
  they can only make the saturation cleaner. Their milestone cells will say whether the tip rate on the
  machine seed improves (that part is learnable).
- Candidate amendment, NOT registered, for the user: pay the ramp to arrival — `span` = the set-down-to-threshold
  distance measured per episode (or a fixed 0.12 m), so that every centimetre up to `nested_v2` pays — and/or
  demo re-injection for the world-model learners. Either changes the reward recipe and needs an amendment
  before any job. I recommend the per-episode span: it keeps the ceiling at 9, keeps the saturation point AT
  the terminal instead of 5–7 cm before it, and changes nothing for a policy that arrives.

## 5. Videos

Two {RLPD} s950 `home` episodes render (the rnd30 cells write mp4s; hold15/spots60 cells do not):
`can_pos_recovery/videos_ln_home_2026-09-12/rlpd_ramp_dH_s950_250k_rnd30_mode_ep0_home.mp4` (reward 7.92,
175 decisions) and `..._rnd30_sample_ep16_home.mp4` (reward 9.0, 276 decisions). One saturated-slide example
per {r2dreamer} arm, same directory: `r2dreamer_ramp_dH_s950_2M_hold15_mode_ep0_slide5cm_timeout.mp4` (uid 252,
reward 5.000, 300 decisions) and `r2dreamer_ramp_dM_s971_2M_rnd30_mode_ep0_slide5cm_tipped.mp4` (reward 5.000,
233 decisions, ends `tipped`). All four sent to the user 2026-09-12 ~04:50.

## 6. Reproduce

```
# {RLPD} cells (in-job preview cells, all 20 Ladder N RLPD runs)
ssh jstale02@login.pax.tufts.edu "python3 - \$(ls -d /cluster/tufts/shortlab/jstale02/gp_ladderN/baselines/rl/checkpoints/e2e/e2e_rlpd_* /cluster/tufts/shortlab/jstale02/gp_ladderN/baselines/rl/checkpoints/e2e_rev3/e2e_rlpd_*)" < cluster/ln_cell_census.py
# {r2dreamer} milestone cells (table + per-episode reward histograms)
ssh jstale02@login.pax.tufts.edu 'python3 /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/ln14_milestone_table.py'
C=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/ln_milestone_cells
ssh jstale02@login.pax.tufts.edu python3 - $C/full_r2d_state_dDPfull_first_rnrh_s971/online_2000000/rnd30_mode/metrics.json $C/full_r2d_state_dHfull_all_rnrh_s950/final/hold15_mode/metrics.json < cluster/ln_cell_census.py
```
