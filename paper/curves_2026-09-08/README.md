# Learning curves (2026-09-08) — ONLINE TRAINING ROLLOUTS, not evaluation

Built by `learning_curves.py` (cluster, reads every run's `metrics.jsonl`) + `plot_curves.py` (plots + ignition test).
Data: `curves_2026-09-08.csv` (per family/arm/stage/step-bin: mean, SE, n_seeds, per-seed values) and
`curves_2026-09-08_seeds.csv` (per-seed ignition step, peak, tail mean, classification).

## Read this before using a number from here
These are **online training rollouts**: the exploring policy, resetting from the **training** bank. They are **not** the
evaluation protocol, which uses mode/sampled actions on the polE / rnd30 start sets and exists **only at the final
checkpoint**. A curve's endpoint is therefore **not** the table number and must not be compared with
`PHASE_RESULTS` §2.y / §3 / §5.1 or the `HRI_results` tables. The caveat is printed on both figures.

`e2e` `nested_proxy` is the **training proxy** (sticky contact + grip commanded open + both upright); it over-counts the
settled predicate by ~2.5× (`EVAL_FIXES` §7.5) and is never labelled `nested`.

## Files
| file | what |
|---|---|
| `learning_curves_mean.png/.pdf` | mean ± SE across 8 seeds, per arm, per stage |
| `learning_curves_seeds.png/.pdf` | per-seed traces (seed spread is the story; the mean hides it) |
| `curves_2026-09-08.csv` | the curve data, including each seed's value per bin |
| `curves_2026-09-08_seeds.csv` | ignition step / peak / tail / classification per seed |

## Two findings

**1. Time to ignition differs on the end-to-end task, where the final level does not.** First binned step at which a
seed's online success reaches 0.2, per-seed, exact-style permutation on the 8 v 8 ignition steps:

| family | stage | human median | machine median | Δ mean | perm p |
|---|---|---|---|---|---|
| **e2e** | **picked** | **549,998** | **799,996** | **−262,499 steps** | **0.018** |
| place | placed_v2 | 75,000 | 87,500 | −9,375 | 0.414 |
| slide | contact | 12,500 | 12,500 | −3,125 | 1.000 |

Human seeds ignite at [424998, 424998, 474998, 474998, 624998, 624998, 674997, 724997]; machine at
[524998, 574998, 624998, 724997, 874996, 924996, 924996, 1374994]. So on the long-horizon task the human-demo arm reaches
the pick roughly **260k steps sooner**, while the published final-checkpoint comparison for that same arm pair is a null —
a difference in learning *speed* that every table in the project is blind to, because they are all final-checkpoint.

**NOT a registered test.** The 0.2 threshold, the 40-bin grid and the choice of stage are all mine, chosen after seeing the
data; three families were tested and only this one is significant (Bonferroni over 3 → 0.054). Treat it as an exploratory
observation worth registering and re-testing, not as a result.

**2. No dead seeds in these families.** On each family's own target stage every seed ignited and held: place `placed_v2`
8/8, slide `contact` 8/8, e2e `picked` 8/8, both arms. (The `contact` column *within place training* shows seeds never
igniting or collapsing, but contact is not a target of the place task — do not read that as a dead seed.) The dead seeds
flagged elsewhere in the project belong to the RLPD and pick families, which are not plotted here.
