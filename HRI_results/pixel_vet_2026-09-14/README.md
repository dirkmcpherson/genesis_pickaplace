# Vetting the pixel-observation claim (2026-09-14)

The other bot's claim: add pixel observations + data augmentation and success rises sharply. This directory holds
what is needed to check it: training curves, evaluation cells, and episode videos, all read from the cluster
run directories and cells (nothing re-rendered, nothing re-scored). Runs are the other bot's amendment (af); the
readout of record is its `paper/AF_PIXEL_CLUSTER_TALLY_2026-09-14.md`.

## Files

| file | what |
|---|---|
| `curves_pixel_training.png/.pdf` | training rollouts, mean ± SE across seeds, one row per config (dreamer port n=4/arm; r2dreamer port n=3/arm; ramp control n=1/arm), with the STATE-based `nested_sparse`(+1) arm's first 1M overlaid dashed |
| `eval_pixel_milestones.png/.pdf` | evaluation cells (fresh process, mode actions, 30 random starts) at 0.5M and 1M, mean ± SE |
| `eval_pixel_per_seed.md` | every seed's rnd30-mode cell at 0.5M and 1M (picked / slide_event / home) |
| `eval_cells_pixel.tsv` | raw cell table (all three cells per milestone) |
| `records/`, `extract_records_px.py`, `plot_pixel.py` | per-episode records and the scripts that regenerate the figures |
| `episodes/reel_*.mp4` | up to 9 episodes per selected cell (home, tipped, timeout), labelled, ~2× slow; `episodes/<cell>/` holds the raw per-episode mp4s and that cell's `metrics.json` |

## What the data say

**Evaluation cells of record (rnd30 MODE, `home` rate, mean ± SE over seeds):**

| config | arm | n | 0.5M | 1M |
|---|---|---|---|---|
| pixels, nested_sparse10, r2dreamer port | human | 3 | 0.60 ± 0.02 | **0.64 ± 0.02** |
| pixels, nested_sparse10, r2dreamer port | machine | 3 | 0.50 ± 0.06 | **0.57 ± 0.03** |
| pixels, nested_sparse10, dreamer port | human | 4 | 0.33 ± 0.11 | 0.33 ± 0.10 |
| pixels, nested_sparse10, dreamer port | machine | 4 | 0.40 ± 0.13 | 0.36 ± 0.06 |
| pixels, nested_ramp, dreamer port | human | 1 | 0.43 | 0.07 |
| pixels, nested_ramp, dreamer port | machine | 1 | 0.03 | 0.40 |
| STATE, nested_sparse(+1), r2dreamer (for reference) | human | 4 | 0.00 | 0.00 (0.41 at 4M) |
| STATE, nested_sparse(+1), r2dreamer | machine | 4 | 0.00 | 0.00 (0.00 at 4M) |
| STATE, nested_sparse10, r2dreamer | both | 2+2 | 0.00 | 0.00 (0.00 at 4M) |

1. **The effect is real and large.** Every pixel seed (14/14 on sparse10) reaches `home` in its 1M evaluation cell;
   the state-based arm reaches it on 0/16 seeds at 1M and only 3/16 at 4M. The training curves agree: pixel runs
   pick at > 0.9 by 250k and reach 0.6–0.9 training `home` by 1M on both ports; the state arm's `picked` never
   exceeds 0.3 in its first 1M.
2. **No source effect from pixels.** Human v machine is within SE on both ports (r2dreamer port 0.64 v 0.57 at 1M,
   n = 3 v 3; dreamer port 0.33 v 0.36, n = 4 v 4). The machine arm, dead on every state-based ladder, is alive
   here from 0.5M.
3. **Oscillation is real on the dreamer port.** s7 human 15/30 → 2/30, s5 human 0/30 → 15/30, s4 machine 19 → 9,
   between 0.5M and 1M. The r2dreamer port is stable (every seed within ±2 between milestones). A single-milestone
   number on the dreamer port is a lottery draw; quote the series or the r2dreamer port.
4. **Tipping falls as `home` rises** on the pixel runs (training `tipped` 0.5 → 0.1), while the state arm stays at
   0.4–0.5 tipped throughout. The pixel policies learn to set down without knocking the can over; the state
   policies do not.

## What to vet before adopting it — three things changed at once

Relative to the state-based `nested_sparse10` arm (which is DEAD: 0 picks at 4M on 2 v 2), the pixel runs change
**three** things together, and the pixel lane's own registrations say so:

- (a) observation: 64×64 image (+ proprioception only, `env.state_slice 8`) instead of the 17-d state that
  includes the ground-truth can pose and goal xy;
- (b) `image_aug shift4` random-shift augmentation on the image;
- (c) the encoder/decoder path (CNN) instead of the MLP state path.

The +10 reward is NOT a candidate: the state-based +10 arm (ac) is as dead as the +1 arm. So the candidates are
pixels+aug (the other bot's reading) or the REMOVAL of the privileged can/goal pose from the observation (a
state encoding that makes the sparse critic hard to fit). These are separable with two cheap controls, neither of
which exists:
- **state-only with the same proprio-only slice** (drop the can/goal pose, keep the MLP): if it ignites, the
  pose was the problem, not the absence of pixels;
- **pixels without augmentation**: if it ignites, augmentation is not the lever.

Also to check by eye in the reels: that `home` episodes are pushes into contact, not drops (the `home` predicate
requires `slide_event`, so a drop cannot score, but a nudge after a near-goal set-down can); and that the top
camera in the 64×64 observation actually resolves the goal can — the wrist view carries most of the information.

## Reels (episodes/, rnd30 MODE, labelled, ~2× slow)

| reel | why |
|---|---|
| `reel_A_human_r2dreamer_s0_1M` | best case: 20/30 home, 0 tipped |
| `reel_B1_human_dreamer_s7_0.5M`, `reel_B2_…_1M` | the oscillation case: 15/30 then 2/30 |
| `reel_C_ramp_human_dreamer_s0_1M` | ramp: 20/30 slide_event but 2/30 home — slides without nesting |
| `reel_D_ramp_machine_dreamer_s0_0.5M` | the tipping checkpoint: 26/30 tipped |
| `reel_E_machine_dreamer_s4_0.5M` | machine best case: 19/30 home at 0.5M |

Budget: every pixel run is 1M online steps (the state arms are 4M); demo eviction from the FIFO buffer at 0.5M
applies to both. Pixel throughput 46.7 fps v 81.7 state-only (`paper/PX_PIXEL_CONFIG_2026-09-12.md`).
