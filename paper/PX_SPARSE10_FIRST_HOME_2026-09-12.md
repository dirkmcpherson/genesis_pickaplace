# {dv3 local} pixels + proprio, shift4, `nested_sparse10` — `home` 5/15 on hold15 MODE at 0.3M online steps (2026-09-12 ~14:50)

**Run:** `~/runs_dv3_local/dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s0` (PHASE_PLAN amendment (ad), registered
12:3x, launched 12:42; `LAUNCH_px_sparse10_s0.txt`). DreamerV3 losses (`rep_loss=dreamer`) in the r2dreamer chassis;
observation = top ++ wrist 64×64×6 pixels + the 8-dim proprioception (q[:6], gripper motor, grip effort) — no can pose,
no goal pose; `image_aug=shift4`; ladder `nested_sparse10` (only `home` pays, +10, terminal `home+tipped`, tip guard
`not_in_hand`); human set `dHfull_all_rns10h_img` (74 tapes, Σ 130, 13 `home`, real renders); fresh world model; seed 0;
2M online budget; throughput 42–47 fps.

## The cell

Series checkpoint `dv3px_sparse10_series/ck_421321/` (counter 421 321 = 303 697 online frames; `latest.pt` at its 300k
rewrite; sha in `snapshot.json`), evaluated by `eval_genesis.py` in a fresh process, MODE (deterministic) actions, the
training world (`[sim-variant] gc_kp4_riser3_shelf6` in the log), 15 demonstration starts (hold15):

| cell | n | picked | placed_v2 | farside | slide_event | home |
|---|---|---|---|---|---|---|
| ck_421321 hold15 MODE | 15 | 14/15 | 14/15 | 14/15 | 5/15 | **5/15** (eps 0, 1, 4, 9, 12) |
| ck_421321 rnd30 MODE | 30 | (running at the time of writing; see the addendum) | | | | |
| ck_317776 hold15 / rnd30 MODE | 15 / 30 | 0/15 / 2/30 | 0 / 3* | 0 / 0 | 0 / 0 | 0 / 0 |
| ck_219022 hold15 / rnd30 MODE | 15 / 30 | 0/15 / 0/30 | 0 / 3* | 0 / 0 | 0 / 0 | 0 / 0 |

\* rnd30 `placed_v2` on 3 in-footprint starts is the reset-time grant (CONFOUNDS row 82), not a placement.

Every slide that started arrived: `slide_event` 5/15 = `home` 5/15. The five `home` episodes (uid 252, 254, 273, 302,
327) each scored exactly 10.0 in 62–126 decisions; the other ten: 7 timeouts, 3 tipped (`outcomes`), rewards 0.0.
The training record's 60 episodes logged before the capture (`record_last60_before.txt`): picked 0.82, placed_v2 0.80,
farside 0.80, slide_event 0.40, home 0.37, tipped 0.17 — the cell's 5/15 = 0.33 agrees with the matching window, as
`R2D_LIVE_VS_RELOAD_2026-09-12.md` requires. The record kept rising after the capture: the last-30 window at counter
461 505 reads picked 1.00, placed_v2 0.97, farside 0.97, slide_event 0.73, home 0.73, tipped 0.20; 97 training `home`s of
456 episodes so far; the first `home` appeared between counters 317 776 and 421 321. Videos: the five `ep*_uid*_home.mp4`
and `rollouts_grid.mp4` (all 15) under `fresh_eval_hold15_mode/`.

## What it means, and what it does not

- **P-ad-1 met at 0.3M** (hold15 MODE `picked` 14/15 ≥ 0.5, registered for 1M). **P-ad-2 met at 0.3M** (≥ 1 series cell
  with `home`, registered for 2M). Both were floors, not rates.
- This is the first policy from any learner in this project that reaches `home` — release, far side, slide, nest — at a
  RATE in an evaluation cell (1/3 of in-distribution starts) rather than as a lone episode (previous best: the state-based
  ramp checkpoint at 3.4M steps, `home` 1/30 rnd30, tipping 14/15 after a saturated 5 cm slide; the cluster's state-based
  r2dreamer arms: `home` 0/270 at 2M).
- **n = 1 seed, and three things changed at once** relative to the state-based runs: pixel observations with augmentation
  (the user's directive), the +10 sparse terminal (amendment (ac)), and the removal of the privileged can/goal pose from
  the policy input. Nothing here attributes the ignition to one of them. The cluster's state-based `nested_sparse10` arm
  ((ac), running) is the +10-alone comparison; a pixels + `nested_ramp` run would be the pixels-alone comparison; neither
  exists yet. Do not write "pixels fixed it" or "+10 fixed it".
- hold15 starts are training starts (14 of 15 are in the demonstration set). The rnd30 cell (random starts in the box)
  is the generalisation read; see the addendum when it lands.
- Tip rate in the training window is 0.20 — far below the state-based ramp policy's 0.9 — with no tip penalty; the
  sparse ladder pays nothing for pushing past arrival, so there is nothing to over-push for.

## The series so far (every `latest.pt` rewrite, 100k counter steps apart; MODE cells; record window = the 60 training episodes before the capture)

| checkpoint (counter) | online steps | record-60: picked / home / tipped | hold15 MODE picked / home | rnd30 MODE picked / home |
|---|---|---|---|---|
| ck_219022 | 101k | 0.00 / 0.00 / — | 0/15 / 0/15 | 0/30 / 0/30 |
| ck_317776 | 200k | (first picks) | 0/15 / 0/15 | 2/30 / 0/30 |
| ck_421321 | 304k | 0.82 / 0.37 / 0.17 | 14/15 / **5/15** | 16/30 / **7/30** |
| ck_519849 | 402k | 0.68 / 0.50 / 0.47 | 2/15 / 1/15 | 6/30 / 3/30 |
| ck_619574 | 502k | 0.92 / 0.73 / 0.22 | 15/15 / **12/15** (3 tipped; nested_v2 = home, no drop nests) | 21/30 / **18/30** (7 tipped, 5 timeout; nested_v2 20/30) |

| ck_718981 | 601k (post-eviction) | 0.97 / 0.73 / 0.10 | 15/15 / **13/15** (2 tipped) | 20/30 / **16/30** |
| ck_818916 | 701k | (see record file) | 15/15 / 11/15 (slide 14/15) | 20/30 / 10/30 (slide 19/30) |
| ck_921211 | 804k | (see record file) | 15/15 / 6/15 (farside 12/15, slide 6/15) | 19/30 / 8/30 |
| ck_1019364 | 902k | (see record file) | 15/15 / 12/15 (slide 15/15; three slides did not nest) | 16/30 / 11/30 |
| ck_1119904 | 1 002k (the 1M point; run stopped 19:01 as human s0 of amendment (ae)) | (see record file) | 15/15 / **15/15** | 20/30 / **16/30** |

**Human seed 0, complete series (0.1M → 1.0M), MODE cells:** hold15 `home` 0, 0, 5, 1, 12, 13, 11, 6, 12, **15** of 15;
rnd30 `home` 0, 0, 7, 3, 18, 16, 10, 8, 11, **16** of 30. **Statistic of record (amendment (ae)) = mean rnd30 MODE `home`
over the 0.3–1.0M cells = 89/240 = 0.371** (hold15: 75/120 = 0.625). The run ended on its best hold15 cell (15/15) and
its second-best rnd30 cell; the 0.8M trough (6/15, 8/30) sits inside the same series.

**1M MILESTONE, parent protocol (SAMPLED actions, `online_1000000.pt`, `dv3px_sparse10_eval_m1000k/`, 19:29):** hold15
picked 15/15, placed 15/15, farside 15/15, slide 15/15, **`home` 15/15** (outcomes: 15 home); rnd30 picked 19/30, placed
21/30, farside 18/30, slide 17/30, **`home` 17/30** (9 tipped, 4 timeout). Sampled and MODE agree at this checkpoint
(15/15 v 15/15; 17/30 v 16/30).

Post-eviction trend at 0.7–0.8M: slides keep starting (hold15 14/15, rnd30 19/30 at 0.7M) but fewer arrive
(`home` 11/15 → 6/15 on hold15; 16/30 → 10/30 on rnd30) — a drift in the slide's finish, not in the approach. The
series to 1M decides whether this is the oscillation seen earlier or a post-eviction decline.

**0.5M MILESTONE, parent protocol (SAMPLED actions, `online_500000.pt`, `dv3px_sparse10_eval_m500k/`):** hold15 picked
13/15, placed 11/15, farside 11/15, slide 11/15, `home` **11/15** (4 tipped); rnd30 picked 23/30, placed 21/30, farside
18/30, slide 16/30, `home` **14/30** (11 tipped, 5 timeout). Sampled actions tip more than MODE (11 v 7 on rnd30) and
nest slightly less — the same ordering the state-based cells showed.

**ck_619574 (0.50M online, 16:20):** `home` 12/15 on demonstration starts and **18/30 on random starts**, deterministic
actions, fresh process, training world — the full task (pick, place, release, far side, slide, nest) from pixels +
proprioception on 60 % of random starts at half a million online steps. Matches its record window (home 0.73). Grid
videos of both cells sent to the user. The same caveats hold (n = 1 seed, no attribution, an oscillating policy — the
next checkpoint may read lower, as ck_519849 did), which is exactly why the series continues to 2M.

ck_519849 is the caution: its 60-episode record window reads picked 0.68 / home 0.50 while the reloaded checkpoint
reads picked 2/15 / home 1/15 on hold15 MODE. The rolling-30 training `home` rate through the run
(`0.00 ×6, 0.13, 0.03, 0.30, 0.30, 0.40, 0.30, 0.30, 0.60, 0.77, 0.50, 0.13, 0.33, 0.50, 0.57, 0.37, 0.10, 0.73, 0.80,
0.70, 0.80, 0.70, 0.77, 0.73, 0.70`) shows the policy swinging between ~0.1 and ~0.8 within a few hundred episodes; a
60-episode window straddles such swings, and ck_519849 was captured in a trough that even its window averages over.
Same mechanism as `R2D_LIVE_VS_RELOAD_2026-09-12.md`, one notch more volatile — `act_entropy` 3e-5 (the e2e recipe of
record; DreamerV3's default is 3e-4) is the obvious suspect for an actor that can swing this far, and is a registered
change, not a knob to turn on this run. Consequence for reporting: quote the SERIES, never one checkpoint; a "best
checkpoint" is a selection and must be labelled as such.

## Reproduce

```
# the cell (fresh process; the exports are REQUIRED, see memory r2dreamer-eval-world-env-var)
cd ~/workspace/r2dreamer && export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
  R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 MUJOCO_GL=egl
D=/home/j/runs_dv3_local/dv3px_sparse10_series/ck_421321
.venv/bin/python eval_genesis.py --checkpoint $D/latest.pt --episodes 15 --mode mode --max-steps 1200 \
  --ic-file /home/j/workspace/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --seed 0 --device cuda --out $D/fresh_eval_hold15_mode
# the record window: $D/record_last60_before.txt (the 60 training episodes logged before the capture)
```

## Addendum — rnd30 MODE of the same checkpoint (landed 14:58)

| cell | n | picked | placed_v2 | farside | slide_event | nested_v2 | home | outcomes |
|---|---|---|---|---|---|---|---|---|
| ck_421321 rnd30 MODE | 30 | 16/30 | 19/30* | 16/30 | 7/30 | 13/30 | **7/30** (eps 3, 11, 14, 16, 19, 21, 23; 68–271 decisions, reward 10.0 each) | 14 timeout, 9 tipped, 7 home |

\* includes the 3 in-footprint reset grants (CONFOUNDS row 82). `nested_v2` 13/30 > `home` 7/30: six episodes nest by
the drop/carry route that `nested_sparse10` does not pay — the ladder shapes the policy toward the slide but does not
eliminate the other route at 0.3M.

So on random starts, one in four episodes ends `home`, one in two picks. For reference on the same cell: {RLPD}
`nested_ramp` human s950 (250k) `home` 1/30; the state-based {dv3 local} ramp checkpoint at 3.4M steps `home` 1/30;
every cluster {r2dreamer} 2M cell 0/30. Same caveats as above: n = 1 seed, no attribution among pixels, +10 and the
removed pose, one checkpoint of a policy that `R2D_LIVE_VS_RELOAD` showed can move within ~1k updates — the series
continues at every 100k-step save.
