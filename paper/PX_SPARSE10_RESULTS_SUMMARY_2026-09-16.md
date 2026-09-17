# `nested_sparse10` PIXEL results — summary of record, 2026-09-16 (21:45)

Human demonstrations (`dHfull_all_rns10h_img`, 74 tapes, 13 `home`) v machine demonstrations
(`dDPfull_first_rns10h_img`, 72 tapes, 14 `home`). Observation: two 64×64 RGB cameras (top, wrist) plus 8-dim
proprioception, no object or goal pose; `image_aug shift4`; `tip_guard not_in_hand`; terminal `home` pays +10.
Every number below comes from `paper/figures/px_phase_2026-09-14/*.csv`, regenerated today from a fresh rsync of the
cluster (run records, milestone cells, RLPD records) plus the local seeds. Methods: `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md` §7.
Tests are exact two-sided permutation tests on per-seed values (Monte-Carlo, 300 000 relabelings, when the exact
count exceeds 1.5 M). Margin of the standing null: ±0.15.

## 1. Headline table (evaluation cells: deterministic MODE actions, 30 fixed random starts, fresh process)

| learner | budget | n human v machine | statistic | human | machine | Δ (h − m) | p | seeds reaching `home` |
|---|---|---|---|---|---|---|---|---|
| {DreamerV3 losses} r2dreamer chassis | 1M / 2M | 16 v 16 | mean `home`, 0.5M + 1M cells | 0.306 | 0.303 | +0.003 | 0.97 | 16/16 v 16/16 |
| same, (ag) seeds only | 2M | 8 v 8 | `home`, single 2M cell | 0.263 | 0.433 | −0.171 | 0.11 | — |
| {r2dreamer contrastive loss} | 1M / 2M | 16 v 16 | mean `home`, 0.5M + 1M cells | 0.588 | 0.530 | +0.057 | 0.09 | 16/16 v 16/16 |
| same, (ag) seeds only | 2M | 13 v 13 | `home`, single 2M cell | 0.572 | 0.556 | +0.015 | 0.76 | — |
| {RLPD} | 250k decisions | 2 v 2 scored | `home`, final cell | 0/30, 0/30 | 0/30, 16/30 | — | — | 0/2 v 1/2 |
| {Diffusion Policy} (ah) | 100k updates | 4 v 4 | `home`, final rnd30 SAMPLED cell, total of 120 | 3/120 | 2/120 | — | 1.00 | 2/4 v 1/4 |
| {Diffusion Policy} (ah) | 100k updates | 4 v 4 | `picked`, same cells, total of 120 | 41/120 | 37/120 | +0.033 | 0.54 | — |

The DreamerV3-loss 16 = 4 local seeds (pop-os workstation, amendment (ae)) + 12 cluster seeds ((af) s4–7 at 1M, (ag)
s8–15 at 2M). The local-only 4 v 4 statistic of record (ten-cell series, 0.3–1.0M) was 0.306 v 0.237, p 0.40.

**Reading.** Both world-model losses learn the full task from pixels on every seed, 64 of 64. Neither shows a
demonstration-source effect on the evaluation statistic: both differences sit well inside the ±0.15 margin. The
contrastive loss is the stronger pixel learner (about 0.56 v 0.30 `home`). The single 2M cells lean machine for the
DreamerV3 losses and do not agree with the pooled statistic; a single checkpoint swings between 0 and 17/30 on one
seed, so the pooled statistic is the number of record. Diffusion Policy from pixels picks but almost never finishes
(`home` 5 of 240 episodes), with no source difference.

## 2. {RLPD} pixels — the one source difference, and a budget defect

**Training record, last 50k of the 250k-decision budget (sampled actions, the policy's own starts, 9 v 8 seeds):**

| phase | human | machine | Δ | exact p |
|---|---|---|---|---|
| picked | 0.44 | 0.78 | −0.34 | < 0.001 |
| placed_v2 | 0.16 | 0.51 | −0.35 | < 0.001 |
| tipped | 0.45 | 0.38 | +0.07 | 0.36 |

Both arms pick at about 0.9 by 50k decisions. The human arm then **loses** the pick (0.9 → 0.44) while the machine arm
holds it (`fig_learning_curves_rlpd_cluster.png`). Separation is complete on picked: every machine seed (0.67–0.95) is
above every human seed except one (0.24–0.65). This is a post hoc read of the training record, not the registered
statistic (final rnd30 MODE `home`), and RLPD's record carries no `home` flag. Report it as a directional finding
with that label until the evaluation cells exist.

**Budget defect.** The 13 (ag) RLPD jobs did not stop at `--steps 250000`: they trained to the 30 h wall clock
(400k–592k decisions) and were killed at 08:46 09-16 before their final evaluation. Every one saved
`rlpd_250000_steps.zip`, the registered checkpoint, so they need evaluation only, not retraining. The analysis now cuts
each RLPD record at 250k decisions (1M frames). The pilot seeds s0/s1 stopped correctly; the difference is in the
(ag) launcher path (`CKPT_EVERY=25000`) and must be found before any further RLPD pixel job is submitted. 15 further
RLPD seeds (s8 machine, s9–15) were cancelled while held and have no data.

**REGISTERED READOUT, 2026-09-17 02:30 — {RLPD} pixels, rnd30 MODE `home` at the 250k-decision checkpoint.**
All human and machine cells are in (7 human + 6 machine scored overnight, jobs 3773759–71).

| seeds | human `home` per seed (of 30) | machine `home` per seed (of 30) | mean human | mean machine | exact p |
|---|---|---|---|---|---|
| design s0–7, 8 v 8 | 0,0,0,0,0,0,0,0 | 0,16,0,0,0,0,11,3 | 0.000 | 0.125 | 0.200 |
| incl. human s8, 9 v 8 | + 0 | same | 0.000 | 0.125 | 0.082 |

`picked` in the same cells: human 0.117, machine 0.371 (8 v 8). Seeds reaching `home`: human 0/9, machine 3/8.
No human-demonstration RLPD seed finishes the task from pixels. Three machine seeds do. The registered test does not
reach significance at this n, but the direction matches the training-record result above. The limitation stated by the
user applies: RLPD ran without a frame stack.

## 3. Controls and ignition

- **Ignition (`fig_ignition.png`).** Training-record criterion (rolling-30 `home` ≥ 0.5) and evaluation criterion
  (≥ 1 `home` in any rnd30 cell) both hold on 4/4 + 4/4 local, 12/12 + 12/12 DreamerV3-loss cluster, and 16/16 + 16/16
  contrastive-loss seeds.
- **State observation (amendment (ai), local, 1 seed, 2M).** The same DreamerV3-loss recipe on the 17-dim state with
  privileged can and goal pose reached `home` in 0 of 2054 training episodes and 0 in all 17 evaluation snapshots.
  The observation is the active ingredient; pixels and `shift4` augmentation are not separated.
- **`nested_ramp` control (1 v 1).** Reaches `home` from pixels on both seeds.

## 4. Rise time (`paper/figures/px_rise_time_by_phase.tex`, `fig_rise_lag_thresh0p5/0p1.png`)

Median online step (thousands) at which the rolling-30 training rate crosses 0.5:

| learner | arm | picked | placed | home |
|---|---|---|---|---|
| {DreamerV3 losses} local | human / machine | 211 / 181 | 229 / 287 | 350 / 320 |
| {DreamerV3 losses} cluster | human / machine | 192 / 213 | 268 / 279 | 443 / 337 |
| {r2dreamer} cluster | human / machine | 131 / 156 | 217 / 204 | 264 / 258 |
| {RLPD} cluster (frames) | human / machine | 53 / 61 | 86 / 90 | not recorded |

Phases rise together under the sparse reward; `home` follows `picked` by roughly 100–260k steps. No arm is
consistently faster.

## 5. What is still open

1. Evaluate the 13 RLPD `rlpd_250000_steps.zip` checkpoints (hold15/rnd30, MODE/SAMPLED) → the registered RLPD 9 v 8.
2. Find why the (ag) RLPD launcher ignored its step budget.
3. The planner-dataset cohorts (`planner72_*`, `r2teacher_*`, other agent) are not in this summary; they are a
   separate source arm and still training.
4. `pixels` vs `shift4` augmentation is untested.

## 6. Files

Figures and CSVs: `paper/figures/px_phase_2026-09-14/` (learning curves per learner, steady state, ignition, rise-time
lag at 0.5 and 0.1, per-seed CSVs, `px_run_census.csv`). LaTeX: `paper/figures/px_rise_time_by_phase.tex`.
Regenerate: `HRI_results/HOWTO_pull_and_plot_pixel_results_2026-09-15.md`. Mirror:
`~/data/genesis_pickaplace/px_analysis_2026-09-14`; the six deleted RLPD seeds and the stale 09-15 RLPD mirror are in
`~/data/genesis_pickaplace/px_analysis_quarantine_deleted_rlpd_2026-09-16/`, out of every figure.
