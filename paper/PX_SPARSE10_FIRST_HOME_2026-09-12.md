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

## Addendum — rnd30 MODE of the same checkpoint

(to be filled from `$D/fresh_eval_rnd30_mode/metrics.json` when the watcher finishes it)
