# FIGNOTES — ignition distribution + performance-with-SE figures (2026-08-18)

Scope: **pick phase only**. Every number below was pulled fresh from wandb on
2026-08-18 by `analysis/assemble_ignition_20260818.py` (raw dump:
`paper/figs/ignition_numbers_20260818.json`) and rendered by
`analysis/make_ignition_figs_20260818.py`. Nothing is quoted from memory or
from PAPER_PLAN; where a pulled number disagrees with a project doc, the
disagreement is stated in §6 rather than resolved silently.

Interpreter: `/home/j/workspace/dreamerv3-torch/venv/bin/python`.
Entity `jambotime`; projects `genesis_paper`, `r2dreamer_genesis`, `dreamer_v3`.

## 0. Files

| file | family | what it is |
|---|---|---|
| `ignition_bimodal_dH_20260818.png` | (A) | per-seed final metric, strip+dot, human-demo panel |
| `ignition_bimodal_dDP_20260818.png` | (A) | same, DP-model-demo panel |
| `ignition_bimodal_dR2D_20260818.png` | (A) | same, r2dreamer-champion-demo panel |
| `perf_all_dH_20260818.png` | (B) | pick rate vs step, mean +/- SE, ALL seeds |
| `perf_all_dDP_20260818.png` | (B) | same |
| `perf_all_dR2D_20260818.png` | (B) | same |
| `perf_ignited_dH_20260818.png` | (C) | same, ignited seeds only |
| `perf_ignited_dDP_20260818.png` | (C) | same |
| `perf_ignited_dR2D_20260818.png` | (C) | same |
| `overview_ignition_20260818.png` | combined | every source x learner cell that exists, one dot per seed, plus explicit NOT-RUN rows |
| `ignition_numbers_20260818.json` | data | raw per-seed pull backing all ten figures |

## 1. THE BIG CAVEAT — the metrics are not protocol-matched

Each series uses the best metric that exists for that learner. They are all
"pick rate on demo initial conditions", but they are **not the same
measurement** and must never be differenced across algorithms without saying
so:

| learner | metric plotted | episodes | horizon | process |
|---|---|---|---|---|
| DP | `eval_indist/picked` from the `*-eval` run | 15 | 1200 | fresh eval process |
| RLPD | `eval_indist/picked` logged during training | 10 | 400 | **in-train sequence eval** |
| r2dreamer | (A): best `eval/picked` over `*-eval-step*` runs; (B)/(C): in-loop `eval/picked` | 15 | recipe default | (A) fresh process, (B)/(C) in-loop |
| dv3 | (A): best `policy_eval/picked` from `dH_msr_ar4_s*-eval-step*`; (B)/(C): in-loop `eval/picked` | 6 | recipe default | (A) fresh process, (B)/(C) in-loop |

Consequences that are stated on every figure:
1. `eval_indist` = **demo-IC** (the ~3 demo can positions; the headline number).
   `eval_random` = support-random ICs (generalization). The figures plot
   demo-IC only. Random-IC numbers are in T1/T2/T3 and are never mixed in.
2. Sequence evals are not independent draws (the standing P2 caveat). The DP
   row and the r2dreamer/dv3 (A) points are fresh-process; the RLPD (A) points
   and every (B)/(C) curve are **in-train / in-loop** and carry P2.
3. Budgets differ by ~30x (RLPD 1e5, dv3 ~3.2e5, r2dreamer 1e6-3e6). The
   (B)/(C) x-axis is therefore **log-scaled**. y always starts at 0.
4. Eval horizons differ (RLPD 400 steps, DP 1200). Measured effect on the
   strongest checkpoint was nil, but it is not zero by construction.
5. No smoothing is applied anywhere. Every seed is drawn: a dot in (A), a thin
   translucent line in (B)/(C).

## 2. Ignition definitions actually used

- Plotted bar: **0.20**, drawn as a dashed line on every panel.
- RLPD registered bar = **>= 3/15 fresh-process demo-IC** = the 0.20 rate. The
  figures apply 0.20 to the **in-train n=10** final eval, so the bar is a
  rate-equivalent, not the registered count. See §6.1.
- r2dreamer registered definition in RESULTS_MATRIX is "**nonzero**
  best-checkpoint eval". The task spec for these figures pinned ">= 0.2 on any
  fresh eval". The two disagree on exactly one seed. See §6.2.
- DP has no ignition lottery (all 16 seeds across both sources land 0.40-0.93).
  It is drawn in (A) precisely so the contrast with RL bimodality is visible.
- dv3: 0 ignitions by any definition.

## 3. Every run family plotted

| panel | series | run names | n | era / code state | protocol |
|---|---|---|---|---|---|
| dH | DP | `dH_DP_s0..s7-eval` | 8 | trained 08-01..08-09; eval-of-record 08-02..08-09; hardened re-eval 08-10/11 | fresh process, 15 ep |
| dH | RLPD | `dH_RLPD-n20_s0..s15` | 16 | cluster, 08-17, **post** demo-RNG fix (>= 2fbed2a) — the PRIMARY RLPD measurement | in-train, 10 ep |
| dH | R2D local | `pick_delta25d4_s0`, `pick_d4clamp_s0`, `pick_delta25d4_s1`, `pick_delta25d4_s2` | 4 | local box, 08-11/12, pre-E1/E2 | fresh ckpt evals, 15 ep |
| dH | R2D wave 1 | `dH_R2Dshort_s10..s19` | 10 | cluster, 08-12/13, **pre**-E1/E2 env | fresh ckpt evals, 15 ep |
| dH | R2D wave 2 (firming) | `dH_R2Dshort_s20..s29` | 10 | cluster, 08-14/15, **post**-E1/E2 env | fresh ckpt evals, 15 ep |
| dH | R2D wave 3 | `pick_v5d4c_delta_s30..s39` | 10 | cluster, 08-17/18, champion recipe, **3M** steps | fresh ckpt evals, 15 ep |
| dH | R2D 3M wave (SUPERSEDED) | `dH_R2D_s0..s3` | 4 | cluster, 08-11/12; superseded by the `*short*` waves; excluded from the registered 6/24 pooling | mixed |
| dH | DV3 | `genesis_pixels_dH_msr_ar4_s0..s2-joint` + `dH_msr_ar4_s*-eval-step*` | 3 | cluster, 08-14, ~3.2e5 steps | fresh evals, 6 ep |
| dDP | DP | `dDP_DP_s0..s7-eval` | 8 | same as dH_DP | fresh process, 15 ep |
| dDP | RLPD | `dDP_RLPD-pair_s0..s2` | 3 | local, 08-14, **PRE** demo-RNG fix, superseded era | in-train, 10 ep |
| dDP | R2D wave 1 / wave 2 | `dDP_R2Dshort_s10..s19` / `s20..s29` | 10 / 10 | 08-12/13 and 08-14/15 | fresh ckpt evals, 15 ep |
| dDP | R2D 3M wave (SUPERSEDED) | `dDP_R2D_s0..s4` | 5 | 08-11/12; carries the dDP prefill/reward-dilution defect (~40% low reward frames) — this is WHY the `*short*` waves were re-run | mixed |
| dDP | DV3 | — | 0 | **NOT RUN** (closed under the dH_DV3 null) | — |
| dR2D | RLPD | `dR2D_RLPD-n20_s0..s15` | 16 | cluster, 08-17, post demo-RNG fix — PRIMARY | in-train, 10 ep |
| dR2D | DP / R2D / DV3 | — | 0 | **NOT RUN** (dR2D demos come FROM r2dreamer, so dR2D_R2D is not a meaningful cell) | — |

### Deliberately NOT plotted
- All prior **local** RLPD waves (`dH_RLPD-nb/-hold/-mref/-ln/-ar4/-ar8/-exp`,
  `dH_RLPD_s0..s6`, `dR2D_RLPD-clean`, `dR2D_RLPD-clean-long`). The counting
  rule registered pre-data on 08-17 (CLUSTER_ROUND Amendment 1) supersedes
  every local RLPD wave for pooled-rate purposes: the cluster wave largely
  RE-EXECUTES those seeds, and no statistic may count a local seed and its
  cluster re-execution. They remain development-era corroboration.
- `pick_delta_s0` r2dreamer evals (dated before the 2026-08-11T03:00 delta-eval
  cutoff) — VOID by the existing rule.
- SACfD (all rows 0, retired to the bug narrative; the entropy-backup defect in
  stock SB3 confounds it) and the ManiSkill positive/negative controls.

## 4. Exact numbers behind each panel

### T1. RLPD — per-seed in-train eval, demo ICs (`eval_indist/picked`, n=10 episodes, 400-step horizon)

| seed | dH 25k | dH 50k | dH 75k | **dH 100k final** | dH randomIC 100k | dR2D 25k | dR2D 50k | dR2D 75k | **dR2D 100k final** | dR2D randomIC 100k |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.00 | 0.20 | 0.20 | 0.40 | 0.10 | 0.00 | 0.50 | 0.10 | 0.00 | 0.10 |
| 1 | 0.00 | 0.00 | 0.20 | 0.30 | 0.20 | 0.00 | 0.20 | 0.40 | 0.20 | 0.10 |
| 2 | 0.00 | 0.00 | 0.10 | 0.70 | 0.50 | 0.00 | 0.00 | 0.10 | 0.30 | 0.00 |
| 3 | 0.00 | 0.00 | 0.00 | 0.20 | 0.00 | 0.00 | 0.00 | 0.30 | 0.10 | 0.10 |
| 4 | 0.00 | 0.00 | 0.30 | 0.10 | 0.10 | 0.00 | 0.40 | 0.50 | 0.10 | 0.20 |
| 5 | 0.00 | 0.00 | 0.00 | 0.30 | 0.00 | 0.00 | 0.00 | 0.20 | 0.00 | 0.00 |
| 6 | 0.00 | 0.00 | 0.40 | 0.10 | 0.00 | 0.10 | 0.40 | 0.00 | 0.30 | 0.20 |
| 7 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 |
| 8 | 0.00 | 0.00 | 0.00 | 0.20 | 0.00 | 0.10 | 0.10 | 0.10 | 0.30 | 0.50 |
| 9 | 0.00 | 0.00 | 0.10 | 0.40 | 0.20 | 0.00 | 0.70 | 0.00 | 0.30 | 0.30 |
| 10 | 0.00 | 0.00 | 0.10 | 0.30 | 0.00 | 0.00 | 0.00 | 0.70 | 0.00 | 0.00 |
| 11 | 0.00 | 0.00 | 0.30 | 0.10 | 0.10 | 0.00 | 0.30 | 0.40 | 0.00 | 0.00 |
| 12 | 0.00 | 0.20 | 0.20 | 0.00 | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| 13 | 0.00 | 0.20 | 0.20 | 0.20 | 0.00 | 0.00 | 0.00 | 0.10 | 0.20 | 0.10 |
| 14 | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 | 0.20 | 0.00 | 0.00 | 0.40 | 0.20 |
| 15 | 0.00 | 0.20 | 0.10 | 0.30 | 0.10 | 0.00 | 0.10 | 0.00 | 0.10 | 0.00 |

- **dH_RLPD-n20** final demo-IC: mean 0.231, SE 0.044, range 0.00-0.70, n=16; at bar (>=0.20) 10/16. Final random-IC mean 0.088 (range 0.00-0.50).

- **dR2D_RLPD-n20** final demo-IC: mean 0.144, SE 0.035, range 0.00-0.40, n=16; at bar (>=0.20) 7/16. Final random-IC mean 0.113 (range 0.00-0.50).

### T2. RLPD dDP — pair wave (n=3, PRE demo-RNG fix, superseded era)

| seed | 25k | 50k | 75k | **100k final** | randomIC 100k |
|---|---|---|---|---|---|
| 0 | 0.00 | 0.00 | 0.20 | 0.00 | 0.00 |
| 1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| 2 | 0.00 | 0.00 | 0.00 | 0.10 | 0.10 |

- mean 0.033, range 0.00-0.10, n=3, at bar 0/3.

### T3. DP — per-seed demo-IC pick rate (`eval_indist/picked`, n=15 episodes, 1200-step horizon)

| seed | dH record | dH hardened re-eval | dH randomIC (record) | dDP record | dDP hardened re-eval | dDP randomIC (record) |
|---|---|---|---|---|---|---|
| 0 | 0.80 | 0.80 | 0.13 | 0.87 | 0.73 | 0.27 |
| 1 | 0.60 | 0.60 | 0.33 | 0.73 | 0.73 | 0.20 |
| 2 | 0.60 | 0.60 | 0.20 | 0.67 | 0.73 | 0.33 |
| 3 | 0.60 | 0.80 | 0.27 | 0.80 | 0.80 | 0.13 |
| 4 | 0.40 | 0.47 | 0.20 | 0.67 | 0.67 | 0.13 |
| 5 | 0.60 | 0.53 | 0.40 | 0.80 | 0.80 | 0.20 |
| 6 | 0.67 | 0.67 | 0.27 | 0.93 | 0.93 | 0.20 |
| 7 | 0.67 | 0.60 | 0.07 | 0.93 | 0.93 | 0.33 |

- **dH_DP** record wave: mean 0.617, SE 0.039, range 0.40-0.80, n=8 | hardened re-eval mean 0.633 (range 0.47-0.80) | random-IC mean 0.233 (range 0.07-0.40).

- **dDP_DP** record wave: mean 0.800, SE 0.038, range 0.67-0.93, n=8 | hardened re-eval mean 0.792 (range 0.67-0.93) | random-IC mean 0.225 (range 0.13-0.33).

### T4. r2dreamer — per-seed BEST fresh-process checkpoint eval (15 episodes/eval, demo ICs, sampled actions)

| source | wave | run name | best fresh eval | # fresh evals | best in-loop eval | env steps reached | created |
|---|---|---|---|---|---|---|---|
| dDP | cluster 3M wave (SUPERSEDED) | `dDP_R2D_s0` | 0.00 | 5 | 0.00 | 2,999,744 | 2026-08-11 |
| dDP | cluster 3M wave (SUPERSEDED) | `dDP_R2D_s1` | 0.00 | 5 | 0.00 | 2,999,936 | 2026-08-11 |
| dDP | cluster 3M wave (SUPERSEDED) | `dDP_R2D_s2` | 0.00 | 5 | 0.00 | 2,999,640 | 2026-08-11 |
| dDP | cluster 3M wave (SUPERSEDED) | `dDP_R2D_s3` | 0.00 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.00 | 2,008,016 | 2026-08-11 |
| dDP | cluster 3M wave (SUPERSEDED) | `dDP_R2D_s4` | 0.00 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.00 | 1,687,960 | 2026-08-11 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s10` | 0.00 | 5 | 0.00 | 999,770 | 2026-08-12 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s11` | 0.00 | 5 | 0.00 | 999,931 | 2026-08-12 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s12` | 0.00 | 5 | 0.00 | 999,765 | 2026-08-12 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s13` | 0.00 | 5 | 0.00 | 999,399 | 2026-08-12 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s14` | 0.00 | 5 | 0.00 | 999,830 | 2026-08-12 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s15` | 0.00 | 5 | 0.00 | 999,815 | 2026-08-13 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s16` | 0.00 | 5 | 0.00 | 999,881 | 2026-08-13 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s17` | 0.00 | 5 | 0.00 | 999,023 | 2026-08-13 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s18` | 0.00 | 5 | 0.00 | 999,793 | 2026-08-13 |
| dDP | cluster wave 1 | `dDP_R2Dshort_s19` | 0.00 | 5 | 0.00 | 999,787 | 2026-08-13 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s20` | 0.00 | 5 | 0.00 | 999,814 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s21` | 0.00 | 5 | 0.00 | 999,739 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s22` | 0.00 | 5 | 0.00 | 999,641 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s23` | 0.00 | 5 | 0.00 | 999,356 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s24` | 0.00 | 5 | 0.00 | 999,864 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s25` | 0.00 | 5 | 0.00 | 999,957 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s26` | 0.00 | 5 | 0.00 | 999,554 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s27` | 0.00 | 5 | 0.00 | 999,820 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s28` | 0.00 | 5 | 0.00 | 999,739 | 2026-08-14 |
| dDP | cluster wave 2 (firming) | `dDP_R2Dshort_s29` | 0.00 | 5 | 0.00 | 999,598 | 2026-08-14 |
| dH | cluster 3M wave (SUPERSEDED) | `dH_R2D_s0` | 0.00 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.00 | 2,493,299 | 2026-08-11 |
| dH | cluster 3M wave (SUPERSEDED) | `dH_R2D_s1` | 0.00 | 5 | 0.00 | 2,999,532 | 2026-08-11 |
| dH | cluster 3M wave (SUPERSEDED) | `dH_R2D_s2` | 0.00 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.00 | 2,675,048 | 2026-08-11 |
| dH | cluster 3M wave (SUPERSEDED) | `dH_R2D_s3` | 0.07 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.07 | 2,494,055 | 2026-08-11 |
| dH | cluster wave 1 | `dH_R2Dshort_s10` | 0.93 | 5 | 0.67 | 999,855 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s11` | 0.00 | 5 | 0.00 | 999,625 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s12` | 0.00 | 5 | 0.00 | 999,417 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s13` | 0.27 | 5 | 0.27 | 999,972 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s14` | 0.00 | 5 | 0.00 | 999,972 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s15` | 0.53 | 5 | 0.40 | 999,994 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s16` | 0.13 *(in-loop fallback: no fresh-eval run exists)* | 0 | 0.13 | 514,380 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s17` | 0.00 | 5 | 0.00 | 972,424 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s18` | 0.00 | 5 | 0.00 | 999,132 | 2026-08-12 |
| dH | cluster wave 1 | `dH_R2Dshort_s19` | 0.00 | 5 | 0.00 | 999,301 | 2026-08-12 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s20` | 0.00 | 5 | 0.00 | 999,873 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s21` | 0.00 | 5 | 0.00 | 972,432 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s22` | 0.13 | 5 | 0.40 | 999,713 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s23` | 0.00 | 5 | 0.00 | 999,681 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s24` | 0.00 | 5 | 0.00 | 999,845 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s25` | 0.00 | 5 | 0.00 | 999,654 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s26` | 0.00 | 5 | 0.00 | 999,669 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s27` | 0.00 | 5 | 0.00 | 999,760 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s28` | 0.00 | 5 | 0.00 | 998,969 | 2026-08-14 |
| dH | cluster wave 2 (firming) | `dH_R2Dshort_s29` | 0.00 | 5 | 0.00 | 999,616 | 2026-08-14 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s30` | 0.00 | 5 | 0.00 | 2,999,837 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s31` | 0.93 | 5 | 0.93 | 2,999,584 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s32` | 0.00 | 5 | 0.00 | 2,999,516 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s33` | 0.00 | 5 | 0.00 | 2,999,813 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s34` | 0.00 | 1 | 0.07 | 2,999,970 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s35` | 0.00 | 5 | 0.00 | 2,999,823 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s36` | 1.00 | 5 | 0.93 | 2,999,843 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s37` | 0.00 | 5 | 0.07 | 2,999,894 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s38` | 0.00 | 5 | 0.00 | 2,999,861 | 2026-08-17 |
| dH | cluster wave 3 | `pick_v5d4c_delta_s39` | 0.00 | 5 | 0.00 | 2,999,906 | 2026-08-17 |
| dH | local (delta recipe) | `pick_d4clamp_s0` | 1.00 | 23 | — | — | None |
| dH | local (delta recipe) | `pick_delta25d4_s0` | 1.00 | 34 | — | 2,999,977 | 2026-08-11 |
| dH | local (delta recipe) | `pick_delta25d4_s1` | 0.00 | 10 | — | — | None |
| dH | local (delta recipe) | `pick_delta25d4_s2` | 0.07 | 30 | — | — | None |

### T5. dv3 msrecipe (dH only; the only dv3 cell that exists)

| run | in-loop `eval/picked` | fresh `policy_eval/picked` (step: value) |
|---|---|---|
| `genesis_pixels_dH_msr_ar4_s0-joint` | 67,100: 0, 176,316: 0, 271,716: 0 | `dH_msr_ar4_s0-eval-step50000` 0 (n=6), `dH_msr_ar4_s0-eval-step157112` 0 (n=6), `dH_msr_ar4_s0-eval-step260000` 0 (n=6) |
| `genesis_pixels_dH_msr_ar4_s1-joint` | 68,244: 0, 177,192: 0, 272,332: 0 | `dH_msr_ar4_s1-eval-step60000` 0 (n=6), `dH_msr_ar4_s1-eval-step160000` 0 (n=6), `dH_msr_ar4_s1-eval-step260000` 0 (n=6) |
| `genesis_pixels_dH_msr_ar4_s2-joint` | 85,376: 0, 193,048: 0, 284,644: 0 | `dH_msr_ar4_s2-eval-step67100` 0 (n=6), `dH_msr_ar4_s2-eval-step170000` 0 (n=6), `dH_msr_ar4_s2-eval-step270000` 0 (n=6) |

### T6. Ignition counts as plotted (per panel column)

| panel | series | n | ignited (>= 0.20) | mean of per-seed metric |
|---|---|---|---|---|
| dH | DP (eval-of-record) | 8 | 8/8 | 0.617 |
| dH | RLPD n=16 (in-train final) | 16 | 10/16 | 0.231 |
| dH | R2D local | 4 | 2/4 | 0.517 |
| dH | R2D cluster wave 1 | 10 | 3/10 | 0.187 |
| dH | R2D cluster wave 2 (firming) | 10 | 0/10 | 0.013 |
| dH | R2D cluster wave 3 | 10 | 2/10 | 0.193 |
| dH | R2D cluster 3M wave (SUPERSEDED) | 4 | 0/4 | 0.017 |
| dH | DV3 msrecipe | 3 | 0/3 | 0.000 |
| dDP | DP (eval-of-record) | 8 | 8/8 | 0.800 |
| dDP | RLPD pair (n=3, pre-fix) | 3 | 0/3 | 0.033 |
| dDP | R2D cluster wave 1 | 10 | 0/10 | 0.000 |
| dDP | R2D cluster wave 2 (firming) | 10 | 0/10 | 0.000 |
| dDP | R2D cluster 3M wave (SUPERSEDED) | 5 | 0/5 | 0.000 |
| dDP | DV3 | 0 | — | **NOT RUN** |
| dR2D | RLPD n=16 (in-train final) | 16 | 7/16 | 0.144 |
| dR2D | DP / R2D / DV3 | 0 | — | **NOT RUN** |

Under the project's "**nonzero** best checkpoint" r2dreamer rule instead of the
0.20 bar the r2d dH counts become: local 3/4, wave 1 4/10, wave 2 1/10,
wave 3 2/10, 3M wave 1/4. dDP stays 0 in every wave under both rules.

## 5. One-line reading of each panel

- **(A) dH**: DP is unimodal and high (0.40-0.80, no lottery). RLPD is a broad
  low spread (0 to 0.70) with 6/16 seeds at exactly 0. r2dreamer is the clean
  bimodal case: seeds are either 0.00 or 0.53-1.00, nothing in between except
  two sub-bar blips (0.067, 0.133). dv3 is flat 0.
- **(A) dDP**: DP is the best cell in the whole study (0.67-0.93, 8/8) while
  every RL/world-model learner on the same demos is at or near 0 — the source
  effect is entirely learner-conditional.
- **(A) dR2D**: only RLPD exists; 7/16 seeds over the 0.20 bar, best 0.40, and
  three whole cells are NOT RUN.
- **(B) dH**: RLPD rises to 0.23 by 100k; the r2d wave means never exceed ~0.18
  because the non-ignited majority holds them down; dv3 is flat 0; DP's
  final-checkpoint band at 0.62 sits far above everything.
- **(B) dDP**: everything except DP is flat at or near 0 across the whole
  budget range.
- **(B) dR2D**: RLPD mean over all 16 seeds runs 0.031 / 0.169 / 0.181 / 0.144
  at 25k / 50k / 75k / 100k — it peaks at 75k and DIPS at 100k, with individual
  seeds swinging 0 to 0.70 between adjacent evals. (dH by contrast rises
  monotonically: 0.000 / 0.050 / 0.138 / 0.231.)
- **(C) dH**: given ignition, RLPD reaches 0.330 mean at 100k (n=10/16) and r2dreamer's
  igniting seeds oscillate violently between ~0.9 and 0 — the checkpoint
  bistability, visible as the mean crashing to 0 by ~2M despite peaks near
  0.9. dv3 and r2d wave 2 drop out of the figure entirely (0 ignited seeds),
  which is annotated in the legend.
- **(C) dDP**: only DP remains; every RL/WM series is annotated "n=0/N ignited
  — no series".
- **(C) dR2D**: RLPD-only, n=7/16 ignited, mean 0.057 / 0.200 / 0.100 / 0.286
  at 25k / 50k / 75k / 100k — non-monotone even among ignited seeds. The dH
  ignited-only counterpart (n=10/16) is 0.000 / 0.060 / 0.100 / 0.330.
- **overview**: the whole grid at a glance, including the four NOT-RUN cells.

## 6. DISAGREEMENTS WITH THE PROJECT DOCS — read this before citing anything

### 6.1 RLPD (A) points are NOT the registered fresh-process sweep numbers
`paper/CLUSTER_ROUND_2026-08-17.md` RESULTS reports the PRIMARY RLPD
measurement as the in-job fresh-process sweep (15 demo-IC episodes per seed,
one fresh process per episode):

| arm | doc: ignited (>= 3/15) | doc: pooled demo-IC | **this figure: in-train n=10 final** | **this figure: at 0.20 bar** |
|---|---|---|---|---|
| dH | 8/16 (0.50) | 53/240 = 0.221 | mean 0.231 | 10/16 (0.62) |
| dR2D | 10/16 (0.62) | 48/240 = 0.200 | mean 0.144 | 7/16 (0.44) |

**The per-seed `SWEEP-RESULT` lines could not be sourced.** They are emitted
only to the cluster's `rlpd_*.out` files (`cluster/sbatch_rlpd.sh` line 105);
they were never written to wandb, never committed, and the cluster is not
reachable from this machine. Commit `e6f11f8` recorded only the aggregate
table. Figures (A)/(B)/(C) therefore use the **in-train `eval_indist/picked`**
that IS in wandb, and the legends/titles say so.

Two things follow and both must be reported:
1. The **pooled means agree well** (dH 0.231 vs 0.221) but the **dR2D arm does
   not** (0.144 in-train vs 0.200 fresh-process).
2. The **direction of the arm contrast flips**: in-train says dH > dR2D
   (0.231 vs 0.144, 10/16 vs 7/16); the registered fresh-process sweep says
   dR2D >= dH on ignition (10/16 vs 8/16) and dH marginally ahead on pooled
   rate. Neither difference is significant (the doc reports Fisher p=0.72 on
   ignition, p=0.65 pooled, MWU 0.51), so the honest statement is **"clean
   null either way"** — but do NOT quote the in-train ordering as if it were
   the sweep result. **The verdict of record stays the doc's: no demo-SET
   effect at n=16/arm.**
3. Cause of the gap is known in kind, not in magnitude: 10-episode in-train
   sequence evals vs 15-episode fresh-process evals, plus the P2
   non-independence of sequence evals. Fresh-process picks have previously
   been shown to be a superset of sequential picks (AUDIT_ms_chain C3).

### 6.2 r2dreamer ignition definition is inconsistent in the record
- `RESULTS_MATRIX_2026-08-15.md` scores ignition as "nonzero best-checkpoint
  eval" and counts wave 2 as **1/10** on `dH_R2Dshort_s22` (best fresh eval
  **0.133** = 2/15).
- The same doc scores `pick_delta25d4_s2` (best fresh eval **0.067** = 1/15) as
  **NOT** a discovery ("single episode, dv3-blip magnitude").
- The spec for these figures pinned ">= 0.2 on any fresh eval".
These three rules do not agree. The figures use **>= 0.20** (so wave 2 shows
0/10, not 1/10) and T6 gives the nonzero counts alongside. Pooled dH ignition
is therefore **7/34** at the 0.20 bar vs **8/34** under "nonzero" — against the
doc's per-wave 2/4, 3/10, 1/10 (= 6/24 before wave 3 existed). This needs a
one-line registration decision before publication.

### 6.3 DP: the matrix headline is the FIRST eval wave, not the re-eval
`RESULTS_MATRIX_2026-08-15.md` gives dH_DP 0.62 (0.40-0.80) and dDP_DP 0.80
(0.67-0.93). Those are the **eval-of-record wave** (08-02..08-09): pulled
values 0.617 and 0.800 exactly. The 08-10/11 hardened re-eval of the same
checkpoints gives 0.633 (0.47-0.80) and 0.792 (0.67-0.93). The two waves
agree within seed noise; the figures plot the **record wave** and T3 lists
both. This is not a contradiction, but the earlier figure script
`analysis/make_matrix_figs_20260812.py` labelled the 08-10 cutoff as "wave 2 =
hardened re-eval", which reads as if the later wave were primary. It is not.

### 6.4 r2dreamer wave counts confirmed, with one bookkeeping note
Pulled per-wave dH ignition (nonzero rule) reproduces the doc exactly: local
2/4 at the 0.20 bar (3/4 nonzero), wave 1 3/10, wave 2 1/10 nonzero, wave 3
2/10 — and dDP 0/20 across waves 1-2. `dH_R2Dshort_s16` has **no**
`-eval-step` runs in wandb (its training run stopped at 514k env steps); the
figure uses its best in-loop eval (0.133, a triangle marker) so the column
still shows n=10. `dH_R2D_s0/s2/s3` and `dDP_R2D_s3/s4` likewise fall back to
in-loop; that whole 3M family is superseded anyway.

## 7. Caveats that apply to specific series

1. **n=3 seeds unless stated.** Only `dDP_RLPD-pair` (n=3) and `dH_DV3` (n=3)
   are at that floor here; everything else is n=4 to n=16. Ranges are given in
   every summary line, never bare means. (SACfD, whose seed spread is
   0.07-0.60, is not plotted at all.)
2. **Pre/post demo-RNG fix.** Before commit `2fbed2a` (08-17) the RLPD
   immutable demo buffer sampled with a generator hard-seeded to 0 in every
   run, so all "independent" seeds shared one demo curriculum. The
   `dDP_RLPD-pair` series is **pre-fix**: its 3 seeds are positively correlated
   through that channel and its CI is optimistic. The n=16 dH/dR2D waves are
   post-fix and are the first genuinely independent RLPD seeds.
3. **Superseded local RLPD waves** are excluded per the pre-registered
   counting rule (§3). Any pooled statistic that added them would double-count
   cluster re-executions of the same seeds.
4. **The dH-vs-dR2D RLPD arms are not single-variable.** CLUSTER_ROUND
   Amendment 2: dH tapes carry 13.3% of frames with delta-joint label error
   > 1e-3 (66/66 picked episodes break from the demonstrated command before
   the grasp) while dR2D tapes are exact (0.000%); demo reward density differs
   9.2x; dH contains 33% no-pick tapes and dR2D 0%. The wave measures "which
   demo SET works better in this trainer as-encoded", not a pure demo-source
   causal effect. (The confound is moot for the verdict because nothing
   separates.) **dH-vs-dDP comparisons are unconfounded on this axis** (both
   ~11-14% label error), and the DP rows are unaffected entirely (DP trains on
   raw absolute commands).
5. **Human-DP numbers depend on idle-frame pruning** (0.67-0.80 pruned vs 0.27
   unpruned control). Model harvests contain no idle frames at all — that is a
   property of the source, not a preprocessing choice. The `dH_DP` column is a
   pruned-demo number.
6. **r2dreamer checkpoint bistability.** On igniting seeds the FINAL checkpoint
   frequently reads 0.00 — true for both wave-3 ignitions (`s31` 0.93 best at
   1.18M, 0.00 at 3M; `s36` 1.00 best at 982k, 0.00 at 3M) and for the
   champion run. (A) therefore plots the BEST fresh checkpoint; (B)/(C) plot
   the raw curve, which is why the (C) dH r2d means collapse to 0 past ~2M.
   Within an igniting run only ~1 checkpoint in 7 is good.
7. **r2dreamer pooled rates cross the E2 env boundary** (local + wave 1
   pre-reset-fix; waves 2/3 post-fix). Per-wave rates are citable, the pooled
   figure is indicative. The panels keep the waves as separate series for
   exactly this reason.
8. **dv3 / r2dreamer eval-harness fixes** landed 2026-08-01 (scope restore) and
   2026-08-08 (action conditioning). Every run plotted here was created
   2026-08-11 or later, so **no pre-fix numbers are used**. Likewise every run
   post-dates the 2026-07-30 old-eval-semantics boundary.
9. **dv3 episode counts are n=6** per fresh eval (periodic protocol), against
   10 for RLPD and 15 for DP/r2dreamer. A 0/6 is much weaker evidence of a
   zero than a 0/15; the dv3 null rests on 3 seeds x 3 evals x 6 episodes plus
   the in-loop curve, all exactly 0.
10. **Pick-phase scope only.** Nothing here is a full-task number; full-task
    figures from the full-demo predecessors are descriptive context, not
    matched protocol.
11. **uid 331** (banked placement, instant-pick IC artifact) is excluded from
    any per-episode analysis. These figures aggregate rates only and do not
    touch per-uid outcomes; the RLPD sweep uid list in `cluster/sbatch_rlpd.sh`
    does not contain 331.
12. **The generational-lineage rows** (single-seed, cap-600 harvests) are NOT
    in these figures; the matrix rows here are cap-1200. Do not cross-read.
13. Local r2dreamer run directories (`~/workspace/r2dreamer/runs/*/policy_eval/
    metrics.json`) were **not** needed: every series plotted has its evals in
    wandb. No TensorBoard-sourced number appears in these figures.

## 8. Reproduce

```
V=/home/j/workspace/dreamerv3-torch/venv/bin/python
$V analysis/pull_ignition_wandb_20260818.py  <scratchdir>   # ~10 min of wandb API
$V analysis/assemble_ignition_20260818.py    <scratchdir>   # -> ignition_numbers_20260818.json
$V analysis/make_ignition_figs_20260818.py                  # -> the ten PNGs
```
