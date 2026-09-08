# Learning curves (2026-09-08) — WORLD MODEL (r2dreamer) ONLY, ONLINE TRAINING ROLLOUTS, not evaluation

*Placed here at the user's request (2026-09-08). Written by the eval-fixes lane, not by `make_tables.py`: these files are
self-contained and **nothing here regenerates or edits `results.md`, `results.csv` or `make_tables.py`**, so the tables
lane keeps sole ownership of those. Regenerate with `learning_curves.py` (on the cluster) then `plot_curves.py` (local).*

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

## CORRECTION (2026-09-08, self-found): the end-to-end series was measuring termination, not the stage

The first version of these curves built every series from the `episode/train_*` flags. For `scope='full'` that is wrong.
The adapter writes those flags only when an episode terminates **inside** it (nested proxy or tip); a TimeLimit
truncation happens outside, so a **truncated episode logs all-zero flags even when it picked**. Measured on
`dHfull_all` s3: **1198 of 2911 episodes all-zero, every one of length exactly 300** (the horizon), and **608 of them
scored ≥ 1** — they picked. `picked` read **0.480 by flag against 0.688 by score**.

The tell was visible in the output and I missed it first time: flag-based `picked`, `contact` and `nested` all reported
**identical** ignition steps (−262,499, p 0.018), because all three were really measuring "terminated having reached the
stage", not the stage.

**Fixed:** end-to-end stages are now derived from `episode/score`, which accumulates the reward stream and survives
truncation. The staged ladder is picked +1 / placed +1 (stale band, ~never earned) / contact +2 / nested +4, so
score ≥ 1 → picked, ≥ 3 → contact, ≥ 7 → nested. **`pick`, `place` and `slide` are unaffected** — each terminates on its
own stage, so a truncated episode there genuinely lacks it and the flag is exact.

**The finding survives the correction, with honest magnitudes** (absolute-0.2 crossing; 8 v 8 per-seed permutation):

| stage | human | machine | Δ | p | was (contaminated) |
|---|---|---|---|---|---|
| picked | 424,998 | 574,998 | −156,249 | **0.019** | −262,499, p 0.018 |
| contact | 824,997 | 949,996 | −187,499 | **0.044** | identical to picked |
| nested_proxy | 1,024,996 | 1,224,995 | −249,999 | 0.059 | identical to picked |

On the 50 %-of-tail-average definition: picked −156,250 (p 0.033), contact −150,000 (p 0.052).

Online final-quarter levels by score, which the flag series also understated: picked human 0.823 v machine 0.760;
contact 0.674 v 0.572; nested_proxy 0.355 v 0.292.

## Standing measurement note (user, 2026-09-08): track ignition STEP, not only ignition likelihood

Every learning-speed claim must be reported from the **raw per-seed step**, not from a collapsed pass/fail rate. Whether a
seed ignited is a much weaker statement than when it ignited, and the two arms in this project differ on the second while
being equivalent on the first (below). `ignition_steps.csv` therefore keeps the raw columns — steady-state value, peak,
final, bin width, and the step at which each seed first reaches **25 / 50 / 75 / 90 % of its own steady state**, plus the
absolute-0.2 crossing — so any threshold ("step at which we hit 50 % of steady-state operation", or any other) can be
reported later without recomputing, and a different definition can be applied to that file directly. Do not collapse it
to a single threshold in the source.

`steady_state` is a **tail average** — the mean of the final quarter of that seed's non-empty bins — and for the
end-to-end runs that quarter is **still improving**, so it is not a converged value (independent review, 2026-09-08). A
registered comparison should use a fixed performance threshold with an explicit persistence rule and a stated handling of
non-igniting seeds. `bin_width_steps` records the resolution, which bounds how finely any of these steps can be read
(e2e 50,000 steps per bin).

### Step to 50 % of steady state (per-seed medians, 8 v 8)

| family | stage | human | machine | Δ | perm p |
|---|---|---|---|---|---|
| place | placed_v2 | 87,500 | 112,500 | −25,000 | 0.075 |
| slide | contact | 37,500 | 37,500 | 0 | 1.000 |
| **e2e** | **picked** | **549,998** | **774,997** | **−156,250** | **0.033** |
| **e2e** | **contact** | **874,996** | **1,024,996** | **−150,000** | **0.052** |

**Pick shows no learning-speed difference** (added at the user's request, 2026-09-08): on the absolute-0.2 crossing the
ignition medians are 162,500 human vs 137,500 machine (Δ mean +12,500, p 0.505); on the 50 %-of-steady-state definition
199,999 vs 187,499 (Δ mean +3,125, p 0.980). If anything the machine arm is marginally faster, and neither is
distinguishable. So the end-to-end effect is
not a general property of the human demonstrations: it appears on the long-horizon task and not on the single-stage one.

The end-to-end effect survives the change of definition (absolute 0.2 threshold → 50 % of each seed's own steady state)
and appears at two stages. Still **not a registered test**: thresholds, grid and stages were chosen after seeing the data,
and several were examined — it is an exploratory observation to register and re-test, not a result to cite.

## Consequence for the training budgets: the arms did not get equal post-ignition training

The budgets were not chosen from these curves — they predate them. Phase runs (place / slide / carrycontact / pick) ran
**1e6 sim steps** (81 launches at `env.steps=1000000`); end-to-end ran **2e6**, registered in `PHASE_PLAN` amendment (d)
as *"2× the single-phase budget; the pick alone needed ≈ 3–5e5"*. That is a doubling heuristic anchored on how long the
pick took, not a convergence criterion, and nothing in the registration set a stopping rule.

What the ignition data now shows, at the fixed 2e6 end-to-end budget:

| stage | arm | training left after first reaching the stage (median / worst seed) |
|---|---|---|
| picked | human | 73 % / 64 % |
| picked | machine | **60 % / 31 %** |
| contact | human | 56 % / 41 % |
| contact | machine | **43 % / 26 %** |

Because the machine arm ignites later, a *fixed* step budget hands it systematically less post-ignition training — for the
slowest machine seed, under a third of the run remained after it first picked. The published end-to-end comparison is a
null, so this did not manufacture a human advantage; but it does mean **the equivalence is stated at a budget the two arms
consumed differently**, and it runs in the direction of understating the machine arm rather than the human one. Any future
end-to-end run should either budget by post-ignition steps, adopt an explicit stopping rule, or report learning speed
alongside the final level — which is the reason for the standing note above.

## Why there is no per-learner breakdown (user, 2026-09-08)

Every curve here is **r2dreamer**. That is not a plotting choice — it is what the three learners persist:

| learner | training-time record on disk | curve possible? |
|---|---|---|
| **r2dreamer (R2)** | `metrics.jsonl`, one row per online rollout episode with `episode/train_{picked,placed_v2,contact,nested}` | **yes — this is what is plotted** |
| **RLPD** | none. The place runs keep `ckpt_040`, `ckpt_100`, `rlpd_final.zip` and final eval dirs; there is no `monitor.csv`, `progress.csv` or wandb run directory on the RLPD side | only by **evaluating saved checkpoints** — `ckpt_040` (100k decisions) and `ckpt_100` (LAST) exist for place, so a **2-point** curve per seed is reachable at real compute cost (16 extra evaluations per phase) |
| **Diffusion Policy** | wandb offline log with the **training loss**; checkpoints pruned to `100000` and `last` to save disk | **no success curve.** DP is offline: there are no online rollouts by construction, and the intermediate checkpoints that would give success-vs-gradient-step were deleted. Loss-vs-step is available but is not comparable with the other two |

So the three learners cannot share an x-axis even in principle — sim steps, decisions and gradient steps — which is the same
non-commensurability already disclosed for their budgets. The per-learner comparison **does** exist for the final numbers
(`results.md`: `place_r2d`, `place_rlpd`, `place_dp` and their pick/slide equivalents); it is the *curves* that are
single-learner, and the figures now say so in the title and caption rather than implying generality.

If a per-learner learning-speed comparison is wanted for the paper, the cheapest honest version is the RLPD 2-point curve
(ckpt_040 vs ckpt_100) alongside R2 evaluated at the matching two points — stated in each learner's own budget units, with
DP represented by its single final point and no interpolation.
