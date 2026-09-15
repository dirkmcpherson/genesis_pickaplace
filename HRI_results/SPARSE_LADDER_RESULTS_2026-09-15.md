# Sparse-ladder human v machine — every cell (2026-09-15)

## 1. Purpose

This document collects **every** human-vs-machine evaluation cell trained under either sparse reward ladder
(`nested_sparse` = `home` pays +1, `nested_sparse10` = `home` pays +10) that exists on the cluster or locally as
of 2026-09-15, across every observation space (17-d state vs 64×64×6 pixels + 8-d proprio) and every learner
({r2dreamer}/{dv3-in-r2dreamer-chassis} with two representation-loss ports, {RLPD}, {Diffusion Policy}). The
evaluation cell of record throughout is **rnd30 MODE** (30 random starts, deterministic/mode action selection,
fresh-process isolation) at the **latest milestone each seed has reached**, read off directly from the cluster
(no re-scoring, no re-simulation). The unit of analysis is the per-seed `home` count out of 30; per-arm
statistics are ignition count (seeds with ≥ 1 `home`), pooled home rate, and an exact two-sided Fisher test on
ignition (human v machine), plus a hierarchical (seed-random-effects) Bayesian read using the project's own
`HRI_results/hri_stats.py`.

**Commands used to pull the raw numbers:**
- {r2dreamer}/{dv3} state + pixel cells: `ssh pax "cd /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03 && python3 ln14_milestone_table.py"` (TSV of `run, milestone, cell, n, ladder, picked, placed_v2, farside, slide_event, home, nested_v2, nested_honest, tipped, node, cores`; filtered locally to `cell == rnd30_mode`, `ladder in (nested_sparse, nested_sparse10)`, excluding `_rnrh`/`_rzh` (ramp/staged) runs and `final`/`smoke` milestones, keeping each run's max `online_N` milestone).
- {RLPD} state: `ladder_provenance.json` + `fresh_eval_rnd30_mode/metrics.json` under `/cluster/tufts/shortlab/jstale02/gp_ladderN/baselines/rl/checkpoints/{e2e,e2e_rev3}/e2e_rlpd_*` and `/cluster/tufts/shortlab/jstale02/gp_ac/baselines/rl/checkpoints/e2e_ac/e2e_rlpd_*`.
- {RLPD} pixels: same pattern under `/cluster/tufts/shortlab/jstale02/gp_pxr/baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_*`.
- {Diffusion Policy} pixels: `/cluster/tufts/shortlab/jstale02/gp_ah/baselines/outputs/dp_px/` — directory does not exist (nothing trained yet, see §4/§5).
- Fisher exact: `scipy.stats.fisher_exact` (available locally, used directly). Bayesian columns: `HRI_results/hri_stats.py:bayes_equivalence(a_k, 30, b_k, 30, prior='primary')` and (for the per-arm rate table) the same module's grid/likelihood internals (`_binom_loglik_grid`, `_log_sigmoid`, `PRIORS['primary']`) integrated per arm alone.

## 2. Results

### 2a. Cells of record (rnd30 MODE, latest milestone per seed)

| learner | observation | ladder | budget/milestone | arm | n | per-seed `home` (of 30) | ignited (≥1 home) | pooled home rate | Fisher p (h v m) | P(Δ>0) | P(Δ∈ROPE ±0.10) | BF01 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r2dreamer | state | nested_sparse | 2M–4M (mixed) | human | 8 | 20,0,16,13,0,0,3,0 | 4/8 | 0.217 | 0.077 | 0.986 | 0.166 | 0.72 |
| r2dreamer | state | nested_sparse | 2M–4M (mixed) | machine | 8 | 0,0,0,0,0,0,0,0 | 0/8 | 0.000 | 0.077 | 0.986 | 0.166 | 0.72 |
| dreamer (port) | pixels | nested_sparse10 | 1M–2M (mixed) | human | 12 | 13,15,10,2,7,6,3,12,0,17,13,5 | 11/12 | 0.286 | 1.000 | 0.061 | 0.410 | 2.49 |
| dreamer (port) | pixels | nested_sparse10 | 1M–2M (mixed) | machine | 12 | 9,16,8,10,16,10,13,18,14,17,0,16 | 11/12 | 0.408 | 1.000 | 0.061 | 0.410 | 2.49 |
| r2dreamer (port) | pixels | nested_sparse10 | 1M–2M (mixed) | human | 16 | 20,20,18,18,10,17,19,15,19,10,17,16,16,18,19,16 | 16/16 | 0.558 | 1.000 | 0.448 | 0.993 | 539 |
| r2dreamer (port) | pixels | nested_sparse10 | 1M–2M (mixed) | machine | 15 | 16,16,19,16,16,18,17,11,17,16,9,18,21,17,22 | 15/15 | 0.553 | 1.000 | 0.448 | 0.993 | 539 |
| r2dreamer | state | nested_sparse10 | 4M | human | 2 | 0,0 | 0/2 | 0.000 | 1.000 | 0.488 | 0.710 | 8.79 |
| r2dreamer | state | nested_sparse10 | 4M | machine | 2 | 0,0 | 0/2 | 0.000 | 1.000 | 0.488 | 0.710 | 8.79 |
| RLPD | state | nested_sparse | 250k/500k (mixed) | human | 4 | 0,0,0,0 | 0/4 | 0.000 | 1.000 | 0.485 | 0.950 | 68.8 |
| RLPD | state | nested_sparse | 250k/500k (mixed) | machine | 4 | 0,0,0,0 | 0/4 | 0.000 | 1.000 | 0.485 | 0.950 | 68.8 |
| RLPD | state | nested_sparse10 | 500k | human | 2 | 0,0 | 0/2 | 0.000 | 1.000 | 0.488 | 0.710 | 8.79 |
| RLPD | state | nested_sparse10 | 500k | machine | 2 | 0,0 | 0/2 | 0.000 | 1.000 | 0.488 | 0.710 | 8.79 |
| RLPD | pixels | nested_sparse10 | 250k | human | 2 | 0,0 | 0/2 | 0.000 | 1.000 | 0.163 | 0.286 | 1.44 |
| RLPD | pixels | nested_sparse10 | 250k | machine | 2 | 0,16 | 1/2 | 0.267 | 1.000 | 0.163 | 0.286 | 1.44 |

### 2b. Model-based per-arm rate (hierarchical seed-random-effects posterior, `prior='primary'`)

The pair-level columns above (`P(Δ>0)`, `P(Δ∈ROPE)`, `BF01`) describe the *difference*; this table gives the
population-averaged rate for each arm alone, from the same posterior grid (mean and 95% credible interval),
which the main table omitted to keep its width fixed.

| learner | observation | ladder | arm | posterior mean home rate | 95% CI |
|---|---|---|---|---|---|
| r2dreamer | state | nested_sparse | human | 0.235 | [0.102, 0.418] |
| r2dreamer | state | nested_sparse | machine | 0.013 | [0.002, 0.049] |
| dreamer (port) | pixels | nested_sparse10 | human | 0.299 | [0.198, 0.423] |
| dreamer (port) | pixels | nested_sparse10 | machine | 0.409 | [0.303, 0.518] |
| r2dreamer (port) | pixels | nested_sparse10 | human | 0.558 | [0.500, 0.614] |
| r2dreamer (port) | pixels | nested_sparse10 | machine | 0.553 | [0.500, 0.613] |
| r2dreamer | state | nested_sparse10 | human | 0.091 | [0.007, 0.382] |
| r2dreamer | state | nested_sparse10 | machine | 0.091 | [0.007, 0.382] |
| RLPD | state | nested_sparse | human | 0.036 | [0.004, 0.179] |
| RLPD | state | nested_sparse | machine | 0.036 | [0.004, 0.179] |
| RLPD | state | nested_sparse10 | human | 0.091 | [0.007, 0.382] |
| RLPD | state | nested_sparse10 | machine | 0.091 | [0.007, 0.382] |
| RLPD | pixels | nested_sparse10 | human | 0.091 | [0.007, 0.382] |
| RLPD | pixels | nested_sparse10 | machine | 0.328 | [0.103, 0.626] |

(n = 2 arms and all-zero arms share the same weakly-informative-prior posterior — e.g. every "n=2, both 0/30"
row above collapses to the identical [0.007, 0.382] interval regardless of learner/ladder, because the
likelihood at k=0,0 is the same and the prior dominates at this n; do not read these as learner-specific.)

## 3. Caveats

- Bayesian columns use `hri_stats.bayes_equivalence(...)` (`HRI_results/hri_stats.py`), default `prior='primary'`, ROPE ±0.10; Fisher p via `scipy.stats.fisher_exact` (available locally). The model ran without error on every pair, including the all-zero-vs-all-zero and n=2 pairs, so no Beta-Binomial fallback was needed.
- Milestones are mixed within several arms (2M vs 4M for r2dreamer state; 1M/1.5M/2M for pixel r2dreamer/dreamer ports; 250k vs 500k for RLPD state nested_sparse) because seeds have reached different online-step counts; "latest per seed" was used, so counts are not from a common checkpoint.
- Diffusion Policy: no sparse-ladder cells exist anywhere (state or pixels) — `gp_ah/baselines/outputs/dp_px` does not exist on the cluster; DP is absent from this table entirely.
- RLPD pixels: only seeds 0/1 per arm have a completed `fresh_eval_rnd30_mode` (10+ other RLPD-px dirs exist per arm but their metrics are MISSING — still training or unevaluated, budgets ranging 250k–2M); n=2v2 is a severe undersample.
- r2dreamer-port pixel machine arm is missing seed 9 entirely (no cells written at any milestone), so that pair is 16 v 15, not 16 v 16.
- dreamer-port pixel cells only exist for seeds 4–15 (no s0–s3), while r2dreamer-port pixel cells span s0–s15 (minus s9 machine) — the two "pixel" ports are not the same seed population, so don't average them together.
- State-based nested_sparse10 has only 2 seeds per arm for both learners (r2dreamer and RLPD) — essentially a smoke-sized read, both arms dead (home=0 throughout).
- RLPD state nested_sparse and nested_sparse10, and r2dreamer state nested_sparse10, are 0/0 machine and mostly-0 human — the "ignited" and BF01 numbers there reflect near-total non-ignition, not a resolved null; treat P(ROPE) as a statement about tiny-n symmetric zeros, not evidence of true equivalence.
- Pooled home rate (§2a) is a naive sum-of-successes/sum-of-trials statistic; §2b gives the hierarchical model's population-averaged rate and CI per arm instead.

## 4. Design differences between the two result sets, disclosed

The state arm (r2dreamer, `nested_sparse`, +1, 4M budget, n=8v8, directional human advantage) and the pixel
arm (both ports, `nested_sparse10`, +10, 1–2M budget, n up to 16v15, near-exact human/machine equivalence) are
**not a controlled contrast on reward magnitude alone** — they differ in several respects simultaneously, all
disclosed in the repo:

- **Reward magnitude.** State ladder pays `home` +1 (`nested_sparse`); pixel ladder pays `home` +10
  (`nested_sparse10`). But magnitude is *not* the active ingredient by itself: the **state-based `nested_sparse10`
  arm is also dead** (0/2 v 0/2 at 4M, §2a) — same +10 reward, same r2dreamer chassis, state observation, and it
  never ignites. (`paper/PHASE_PLAN_2026-09-04.md` amendment (ac); `HRI_results/pixel_vet_2026-09-14/README.md`
  row "STATE, nested_sparse10, r2dreamer | both | 2+2 | 0.00 | 0.00 (0.00 at 4M)".)
- **Observation.** State arm: 17-d vector including the **ground-truth can pose and goal xy** (a privileged
  state encoding). Pixel arm: 64×64×6 uint8 image (top camera ++ wrist camera) plus **only the first 8 dims of
  state** (`env.state_slice: 8` = 6 joint positions, gripper motor, grip effort) — can pose and goal xy are
  **not observed**; the policy must read them off the pixels. (`paper/PHASE_PLAN_2026-09-04.md` amendment (ad):
  "the can pose/quaternion and goal xy are NOT observed"; `paper/PX_PIXEL_CONFIG_2026-09-12.md` line 26.)
- **`shift4` augmentation.** Pixel arm applies DrQ-style random shift (replicate-pad 4 px, one integer offset
  per sequence, nearest/bilinear per config) to the image before *both* the encoder and the decoder target — a
  data augmentation, not a loss term. The state arm has no analogous augmentation (nothing to augment).
  (`paper/PX_PIXEL_CONFIG_2026-09-12.md` §"What `shift4` is, and what it is not"; amendment (ad).)
- **Encoder/decoder path.** Pixel arm adds a CNN encoder + CNN decoder (136 tensors vs the state-only config's
  93 — "the extra 43 tensors are the CNN encoder and CNN decoder"); state arm is MLP-only.
  (`paper/PX_PIXEL_CONFIG_2026-09-12.md` line 130.)
- **Budget.** State: 4M online steps (r2dreamer `nested_sparse`/`nested_sparse10`). Pixel: 1–2M online steps
  (local (ad) runs to 2M, cluster (af) runs to 1M). (`paper/PHASE_PLAN_2026-09-04.md` amendments (ad), (af);
  `HRI_results/pixel_vet_2026-09-14/README.md`: "every pixel run is 1M online steps (the state arms are 4M)".)
- **Throughput.** Pixel ≈ 46.7 fps settled; state-only ≈ 81.7 fps settled — pixels cost roughly 43–44% of
  throughput on the same box, both well above the 25 fps floor. (`paper/PX_PIXEL_CONFIG_2026-09-12.md` lines
  182, 220, 237–239; `HRI_results/pixel_vet_2026-09-14/README.md` final line.)
- **Demonstration eviction.** Both arms use a FIFO replay buffer (`buffer.max_size=5e5`) and demo frames are
  gone from replay by ~500k online steps in **both** the state and pixel configurations — so for the state arm's
  4M budget, ≥ 87% of training happens with no demonstration in replay, and for the pixel arm's 1M budget, ~50%
  does. (`paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` §7.1: "all demo frames gone by 500000 online env
  steps... Ignition happens at 2M+, so for ≥ 87% of training neither arm has any demonstration in replay";
  `HRI_results/pixel_vet_2026-09-14/README.md`: "demo eviction from the FIFO buffer at 0.5M applies to both".)
- **Representation-loss port and seed ranges.** Two independent world-model implementations were run on
  pixels: `rep_loss=dreamer` (reconstruction loss, "dreamer" port in this table, seeds 4–15, 12 per arm) and
  `rep_loss=r2dreamer` (the port's own contrastive loss, no decoder; "r2dreamer" port, seeds 0–8/0–15, up to
  16 v 15). The state arm only ever used the r2dreamer-port chassis (`model.rep_loss` default, i.e. no
  `rep_loss=dreamer` state cells exist). (`paper/PHASE_PLAN_2026-09-04.md` amendment (af): "R2 = `rep_loss=r2dreamer`
  (the port's own contrastive loss, no decoder... )"; seed assignments per §2a of this document, and
  `paper/AF_PIXEL_CLUSTER_TALLY_2026-09-14.md`.)

`HRI_results/pixel_vet_2026-09-14/README.md` names this explicitly as an open confound: "Relative to the
state-based `nested_sparse10` arm... the pixel runs change **three** things together... observation, `image_aug
shift4`... and the encoder/decoder path (CNN) instead of the MLP state path," and proposes the two controls
listed in §5 below, neither of which has been run.

## 5. What can and cannot be claimed

On the **state** observation, the only comparison with enough seeds to say anything (r2dreamer, `nested_sparse`,
n=8v8) shows a **directional human advantage in ignition** (4/8 human seeds reach `home` at least once vs 0/8
machine, Fisher p=0.077, P(Δ>0)=0.986) whose **magnitude is unresolved** at this n (P(Δ∈ROPE)=0.166, BF01=0.72 —
inconclusive, not evidence for or against a ≤0.10 effect). On **pixels**, the best-powered arm (r2dreamer port,
n=16v15) shows near-exact **human/machine equivalence** (P(Δ∈ROPE)=0.99, BF01=539, pooled rates 0.558 v 0.553),
i.e. demonstration source stops mattering once pixels replace the privileged state. These two findings are
**not a clean state-vs-pixels contrast**: as §4 details, the pixel arm differs from the state arm in
observation, augmentation, encoder, reward magnitude, training budget, and (for the state-based +10 control)
still fails to ignite even with the larger reward — so "pixels equalize the arms" is confounded with "removing
the privileged can/goal pose equalizes the arms" and with "adding `shift4` augmentation equalizes the arms."
The two controls that would separate these are registered as missing in `HRI_results/pixel_vet_2026-09-14/README.md`:
a **state-only run with the proprio-only 8-d slice** (drop can/goal pose, keep the MLP — if it ignites, the pose
was the problem, not the absence of pixels) and a **pixels-without-augmentation** run (if it ignites, `shift4` is
not the lever). Diffusion-Policy-on-pixels and most of RLPD-on-pixels are simply not run yet: DP-pixel has no
trained checkpoints anywhere (`gp_ah/baselines/outputs/dp_px` does not exist), and RLPD-pixel has only 2 of its
seeds per arm evaluated (§2a, §3), so neither learner supports a pixels claim beyond {r2dreamer}/{dv3}.
