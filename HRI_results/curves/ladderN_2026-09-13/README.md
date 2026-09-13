# Ladder-N training curves — human v machine, mean ± SE across seeds (2026-09-13)

Built from the per-episode stage records of every Ladder-N run on the cluster as of 2026-09-13 ~12:00
(`records/INDEX.tsv` lists every run, seed, ladder, budget and source path). Regenerate with
`python3 extract_records.py <out>` on the login node, rsync `<out>/` to `records/`, then `python3 plot_curves.py`.
Refresh `eval_cells_r2dreamer.tsv` from `$W/ln14_milestone_table.py` when new milestone cells land.

## What each figure is

| file | content |
|---|---|
| `curves_<learner>_<ladder>.png/.pdf` | one panel per stage; human (blue) v machine (orange); thick line = mean across seeds, band = ±1 SE (std/√n, ddof 1), faint lines = individual seeds; the small numbers above each panel = seeds contributing to that bin (a run that has not reached that step drops out) |
| `home_by_ladder_<learner>.png/.pdf` | the terminal stage for every ladder on one axis (`home` for r2dreamer; `nested_v2` for RLPD, whose logger does not record `home`), with each curve's legend stating how far its runs reached |
| `eval_milestones_r2dreamer.png/.pdf` | the EVALUATION protocol: fresh-process rnd30 mode cells at 0.5/1/2/4M, mean ± SE across seeds, n per point |
| `binned_<learner>.csv` | the per-seed binned rates behind the training-curve figures (bin = 100k online steps r2dreamer, 12.5k decisions RLPD; a bin needs ≥ 5 episodes) |
| `eval_cells_*.tsv` | the raw cell tables the eval figure and the results tables are drawn from |

**Training curves are online rollouts of the exploring policy** (sampled actions; r2dreamer resets from the demo
starts, RLPD from its own reset distribution). They are not the evaluation protocol and an endpoint is not a table
number. The eval figure is the protocol of record (mode actions, 30 random starts per seed).

## Seeds per curve

| learner | ladder | human seeds | machine seeds | steps reached |
|---|---|---|---|---|
| {r2dreamer} | nested_ramp | 950 951 952 953 (rev 3) + 954 959 960 961 (rev 4, running) | 970 971 972 973 + 974 979 980 981 (running) | 950/951/970/971: 2M; 952/953/972/973: 4M; rev 4: ~1.7M and rising |
| {r2dreamer} | nested_sparse | 955 956 957 958 | 975 976 977 978 | 4M (976 stalled at 3.93M) |
| {r2dreamer} | nested_sparse10 | 957 958 (running) | 977 978 (running) | ~3.2M / ~2.8M, target 4M |
| {RLPD} | nested_ramp | 950 951 (250k) 952 953 (500k) | 970 971 (250k) 972 973 (500k) | mixed budgets |
| {RLPD} | nested_sparse | 955 956 (250k) 957 958 (500k) | 975 976 (250k) 977 978 (500k) | mixed budgets |
| {RLPD} | nested_sparse10 | 957 958 | 977 978 | 500k |
| {RLPD} | staged (control) | 958 959 | 978 979 | 100k |

RLPD's per-episode record carries picked / placed_v2 / nested_v2 / slide_success / tipped only — the Ladder-N RLPD
logger predates `farside` / `slide_event` / `home`, and no episode return is logged, so `home` is not recoverable
for RLPD from training records. Its intermediate checkpoints (16 %, 40 %) exist unevaluated.

## The reward-function × budget confound, stated plainly

The ladders were NOT run to a common budget, and within a ladder the seeds were not either:

- r2dreamer `nested_sparse` has 4 v 4 seeds at 4M. `nested_ramp` has 8 v 8 seeds but only 2 v 2 of them at 4M; the
  rest are at 2M or still at ~1.7M. `nested_sparse10` is 2 v 2 and still training.
- RLPD ramp and sparse each mix two 250k seeds with two 500k seeds per arm; sparse10 is 500k; the control is 100k.

So "sparse beats ramp" for the world model is only a fair statement at MATCHED x. From `binned_r2dreamer.csv`,
training-episode `home` rate (mean ± SE, n):

| ladder | arm | at 1.9M | at 4.0M |
|---|---|---|---|
| nested_sparse | human | 0.111 ± 0.067 (4) | **0.612 ± 0.206 (4)** |
| nested_sparse | machine | 0.000 (4) | 0.000 (4) |
| nested_ramp | human | 0.018 ± 0.007 (4) | 0.044 ± 0.039 (**2**) |
| nested_ramp | machine | 0.012 ± 0.006 (4) | 0.000 (**2**) |
| nested_sparse10 | human | 0.000 (2) | — |
| nested_sparse10 | machine | 0.000 (2) | — |

At the matched 2M point sparse-human is already ahead of ramp-human (0.11 v 0.02), and at 4M the ramp comparison
rests on 2 seeds. The 4 v 4 ramp-at-4M read arrives when the rev-4 packs finish (~09-14 00:00); the sparse10 4M
read ~09-13 afternoon. Until then the only budget-matched, seed-matched human-v-machine contrast at 4M is
**nested_sparse, 4 v 4**, and there human ignited 3/4 seeds v machine 0/4 (Fisher p 0.14).

The eval-cell figure shows the same picture under the evaluation protocol: sparse-human `home` 0 → 0 → 0.08 → 0.41
(n = 4 at every point), every other curve at ≤ 0.03.

## What the curves show that the tables do not

1. **Bimodality.** The faint per-seed lines in `curves_r2dreamer_nested_sparse.png`: three human seeds climb to
   `home` 0.6–0.9 of training episodes, one (s956) never leaves zero and loses `picked` after 2M; all four machine
   seeds sit at zero on every stage past `picked`, and their `picked` decays to zero by ~2M. The SE band on the
   human curve is wide because it spans two populations, not because the ignited seeds disagree.
2. **The ramp gets both arms to the far side and sliding** (`farside`, `slide_event` panels of the ramp figure rise
   for BOTH arms after ~1.2M) **but not home** — consistent with the 5 cm ramp saturating (`LN_RAMP_SATURATION`).
3. **Machine `picked` under sparse never establishes.** Machine sparse seeds peak below 0.1 picked and decay; the
   human sparse seeds hold 0.2 from the start (the human demos' prefill) and climb after 1.5M. Whether that is the
   demonstration source or the un-ignited-seed collapse is exactly what (ac) is testing.
4. **RLPD** (`curves_rlpd_*.png`): human above machine on `picked`/`placed_v2` under every ladder, no `nested_v2`
   beyond noise for either arm within the budgets run; the 250k/500k seeds are visibly different lengths.
