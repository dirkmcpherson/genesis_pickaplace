# DP on the pruned human set: why 0.52 when we remember ~80 %? (forensic account, 2026-09-07)

Question (James, 09-07): "assess why DP is doing so poorly on the pruned set. I thought we had it up near 80% successful."
Scope: every Diffusion Policy pick-stage number on record, its exact cell, where the ~0.80 lives, and a quantified
decomposition of the gap to the 0.520 of record (`paper/MORNING_TABLE_2026-09-04.md` §1: DP dH-pruned **0.520**, LAST
checkpoint, rnd-30, n = 10, corrected world). Nothing was trained; no GPU job was run. Every number below is read from an
evaluation file, a HEADLINE line, a wandb summary, or a cited document; per-IC counts were tallied from the existing
per-episode `sweep.json` records with `sel_ckpts.py` / `peric.py` (throwaway, in the session scratchpad). Path legend:
`C:` = cluster `/cluster/tufts/shortlab/jstale02/genesis_pickaplace`, `L:` = this repo, `W:` = wandb entity `jambotime`.

## 0. Verdict in ten lines

1. **No DP-on-pruned-human cell has ever scored 0.80 on random starts.** The best single seed on random starts is 0.63
   (old world, selected ckpt); the best arm mean is 0.589 (old world, LAST, n = 6). The 0.520 is not a fall from 0.80.
2. **The ~0.80 is an in-distribution (demo-start) number**: the July positive control `hdp_s2` 0.80 and the August
   `dH_DP_s0` 0.80 are single seeds on the 15 demo starts (arm means 0.67 and 0.62); the machine arm `dDP_DP` averaged
   0.80 there; in the current design the same arm scores **0.913 on hold-15** and **0.853 on the same 15 demo starts
   the August 0.62 was measured on** — in-distribution DP is *better* than it has ever been.
3. On random starts the arm went 0.23 (August, stride-1 recipe) → 0.57 (repeat-4 recipe, old world) → 0.52 (corrected
   world): the recipe of 08-23 more than doubled the random-start number; the corrected world cost about 0.05.
4. **The remaining gap is the IC set, and it has a mechanism**: 10 of the 30 rnd ICs are never picked by any DP seed in
   the corrected world (0/10 dH, 0/10 dDP, 0/8 dHv2). 9 of those 10 lie at can x ≥ 0.52 m, beyond the farthest training
   can of the pruned set (x ≤ 0.513); their nearest training start is 7.8 cm away (median) vs 3.8 cm for the 20 live ICs.
   On the 20 ICs inside the pruned set's support DP picks **156/200 = 0.78**. 0.78 × 20/30 = 0.52.
5. Those far ICs are reachable: RLPD picks 5 of the 10 and the world model 4 (online exploration); DP, a pure imitator on
   absolute joint targets, extrapolates to none of them. This is a *property of the evaluation set relative to the
   pruned support*, not a DP regression, and it is the same for the machine arm (0.467 = 140/200 = 0.70 on the live 20).
6. Checkpoint rule costs 0.027 (selected 0.547 → LAST 0.520); dataset version (frozen dH 58 vs dHv2 60) costs ≤ 0.04;
   predicate hardening, horizon, sampling and process protocol cost nothing measurable (same 1200-step horizon and
   sampled actions since July; the 08-09 predicate moved the August seeds by ≤ 0.025).
7. Seed variance is now small (LAST rnd SD 0.072, range 0.40–0.60, n = 10) — the July "0.00–0.53" spread belonged to
   38-demo, stride-1, single-box runs and no longer describes the learner.
8. **So: a different cell, and within that cell a support-limited ceiling — not a regression.** The pruned human DP is
   at its historical best on every in-distribution cell and on the reachable random starts.
9. Nothing needs re-running to settle the memory question; the like-for-like cells exist (§3.1, §3.8).
10. What *would* sharpen the paper's DP row: report rnd-30 stratified by training support (20 in / 10 out), and draw
    the queued `rnd2` (CONFOUNDS row 31) from the *success-tape* support box, not the all-solved box (§5).

## 1. Every DP pick-stage number on record

Predicate = `picked` throughout (hardened lift since 08-09, `L:baselines/genesis_can_env.py:48-60,258-265`; the
pre-08-09 rows were re-evaluated under it, row 7). Horizon = 1200 sim steps in every row (wandb config `max_steps 1200`
for the July/August evals; `--max-steps 1200` in `L:cluster/eval_sweep.sh:37`). DP actions are sampled at eval in every
row (`L:baselines/wandb_eval.py:105-109`; seeded per IC since 080cb73). "in-dist-15" = `ic_sampling.demo_ics(env)[:15]`
= the 15 uids that became `eval_ics.json` `sel` (its `sel_note`); "random-15" = `sample_support_ics(env, 15, seed=0)` =
the first 15 of `eval_ics.json` `rnd` (its `rnd_note`); `hold` = 15 held-out demo uids; `rnd` = 30 fixed random starts;
`hold-66/69` = all training ICs of the v2 sets (`eval_ics_v2_w3.json` `hold_note`).

### 1.1 The pruned-human lineage (the arm in question)

| # | date | dataset (set, N tapes, world recorded) | eval world | IC set | ckpt | seeds | picked | source |
|---|---|---|---|---|---|---|---|---|
| 1 | 07-06..13 | DP-v3, 58 solved demos, full task, pre-goal-fix world | base (old goal) | demo starts | last | 1 | in-dist 0.49 | `L:CLAUDE.md:44`; panel 07-08 random-IC 0.34 (`:46`) |
| 2 | 07-18 | `lerobot_dataset_pick` 38 success demos, UNpruned | base | random-15 (pre-07-30 `eval/*` = random) | last | 3 | 0.00 / 0.07 / 0.47 (mean 0.18) | `L:CLAUDE.md:36`; `W:genesis_pickaplace/dp_pick_0720_1754-eval` 0/15 |
| 3 | 07-20 | `lerobot_dataset_pick_pruned` (make_dp_pruned, south goal) | base | random-15 | last | 3 | 0.40 / 0.07 / 0.53 (mean **0.33**) | `L:CLAUDE.md:34`; `W:genesis_pickaplace/dp_pick_pruned_0720_2117-eval` eval/picked 0.333 n=15 |
| 4 | 07-31 | `lerobot_dataset_pick_pruned` rebuilt 66/66 (the earlier 0.67 had trained on 60/66, `L:July30th_Fable.md:195-197`) | base | in-dist-15 / random-15 | last | 3 | **0.667 / 0.267** (`audit_joint_rebuilt`), **0.733 / 0.133** (`hdp_s1`), **0.800 / 0.067** (`hdp_s2`) | `W:genesis_pickaplace/{audit_joint_rebuilt,hdp_s1,hdp_s2}-eval`; `L:PAPER_PLAN.md:401-405` ("pruned human DP = 0.67/0.73/0.80 picked in-dist") |
| 5 | 08-01 | `lerobot_x2x2v2_jobs_jact` — same frames UNpruned (control) | base | in-dist-15 / random-15 | last | 1 | 0.27 / 0.13 | `L:PAPER_PLAN.md:403-404`; `L:July30th_Fable.md:246-250` |
| 6 | 08-02..09 | `lerobot_dH_pick`, 66 pruned pick-phase tapes, stride 1 (30 Hz) | base | in-dist-15 / random-15 | last | 8 | in-dist **0.617** (.80 .60 .60 .60 .40 .60 .667 .667), random **0.233** (.13 .33 .20 .27 .20 .40 .27 .07) | `W:genesis_paper/dH_DP_s{0..7}-eval` (created 08-02..09; train cfg `dataset.root=baselines/lerobot_dH_pick`); `L:paper/results_core_matrix.md:26-34` |
| 7 | 08-10/11 | same 8 checkpoints re-evaluated under the hardened predicate | base | same | last | 8 | 0.633 (0.47–0.80) / 0.208 (0.13–0.33) | `W:` same run names, later `created`; `L:paper/results_matrix_2026-08-11.md:52-56` |
| 8 | 08-23 | `matched_v1_pilot/dH`, 51 tapes, contract-v1 recorder (repeat 4, follower re-execution of the pruned tapes), old world | base | sel / hold / rnd | selected (LAST) | 2 | s0 sel 13 hold 11 rnd 15/30 (LAST 14, 16); s1 14/13/15 (LAST 14, 17) | `C:baselines/outputs/dp_pilot/dH_DP_s{0,1}/sweep/HEADLINE.txt`; `L:PAPER_PLAN.md:252-253` ("DP-r4 pilots … rnd 15/30 vs stride-1 dHpruned_DP 0.62/0.23") |
| 9 | 08-24..25 | `matched_v2/dH`, 56 tapes (`--arrival either`), old world | base | hold / rnd | selected | 5 (+s19) | hold 61/75 = **0.813**, rnd 85/150 = **0.567** (17 16 18 15 19); with s19: rnd 102/180 = 0.567 | `C:baselines/outputs/dp_final/dH_DP_s{10..14,19}/sweep/HEADLINE.txt`; `L:paper/RESULTS_TABLE_2026-08-25.md`, `paper/UPDATE_2026-08-25.md:41-43` (s15–18 never scored: no sweep dir) |
| 9b | " | " | base | hold / rnd / sel | **LAST** | 6 | hold 75/90 = 0.833, rnd 106/180 = **0.589** (17 18 18 16 19 18), sel-15 72/90 = **0.800** | same HEADLINEs (`final_*`); `sweep/100000/sweep.json` |
| 10 | 08-24 | `matched_w2_pilot/dH` (w3 pilot, the dDP_w2 teacher) | w3 | hold / rnd | selected = LAST (100k) | 2 | hold 14/15, rnd 18/30 (both seeds) | `C:…/dp_pilotw2/dH_DP_s{0,1}/sweep/HEADLINE.txt`; `L:paper/METHODS_draft_2026-08-28.md:232` |
| 11 | 08-24..26 | **`matched_w3/dH`, 58 tapes, w3** (the frozen pruned human set; sha 07ce3f36) | w3 | hold / rnd | selected | 10 | hold 133/150 = **0.887**, rnd 164/300 = **0.547** (18 19 18 14 13 15 16 16 18 17) | `C:…/dp_w2final/dH_DP_s{20..29}/sweep/HEADLINE.txt`; `L:paper/RESULTS_for_writing_2026-08-30.md:16`; `paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` §2.1 |
| **11b** | " | " | w3 | hold / rnd / sel | **LAST (100k)** | 10 | hold 137/150 = **0.913**, rnd 156/300 = **0.520** (18 18 18 14 13 15 12 16 17 15), sel-15 128/150 = **0.853** | same HEADLINEs (`final_*`); `sweep/100000/sweep.json`; **= the 0.520 of record** (`MORNING_TABLE` §1) |
| 12 | 08-30/31 | `matched_v2/dHunpruned`, 52 tapes (unpruned control), old world | base | hold / rnd | selected | 3 | rnd 13 17 16 /30 = 0.511, hold 13 12 13; LAST rnd 16 10 16 | `C:…/dp_final/dHunpruned_DP_s{32,33,34}/sweep/HEADLINE.txt`; `RESULTS:22-30` |
| 13 | 08-26.. | `matched_w3/dHallpruned_1e3` / `_1e2` (density controls, w3) | w3 | hold / rnd | selected | 3 / 3 | rnd 18 17 17 / 17 18 17 (0.578 / 0.578), hold 13–14/15 | `C:…/dp_density/dHallpruned_*_DP_s2?/sweep/HEADLINE.txt` |
| 14 | 08-27.. | `matched_w3/dH_A` half (N14), w3; `dH_A` v2 halves | w3 | hold / rnd | selected | 2 (+2) | rnd 11, 17 (LAST 15, 17); v2 halves 9, 10 | `C:…/dp_splithalf/dH_A_DP_s4{0,1}`, `dp_splithalf_v2/dH_A_DP_s5{0,2}` (partial blocks) |
| 15 | 08-31 | local `lerobot_dataset_pick_v2r`, 73 tapes of the recovered corpus, stride 1, local box | base | in-dist-15 / random-15 | last | 1 | 0.40 / 0.07 | `L:CLAUDE.md:23`; `W:genesis_pickaplace/dp_v2r_0831` |
| 16 | 09-01 | `matched_v2/dHv2` (v2 pruned, old world) pilots; `matched_w3/dHv2` pilots | base / w3 | hold-69 / rnd; hold-66 / rnd | selected | 2 / 2 | old: rnd 15, 12; hold 69/69, 64/69. w3: rnd 16, 13 (LAST 18, 13); hold 63/66, 66/66 | `C:…/dp_pilotv2P/`, `dp_pilotv2Pw3/dHv2_DP_s{0,1}/sweep/HEADLINE.txt` (the dDPv2p teacher = w3 s0 ckpt 080000, `TEACHER_SELECTED.txt`) |
| 17 | 09-02 | `matched_v2/dHv2`, 60 tapes, old world | base | hold-69 / rnd | selected | 8 | rnd 123/240 = **0.512** (18 15 12 17 15 15 15 16); hold 62–67/69 | `C:…/dp_v2fullP/dHv2_DP_s5?/sweep/HEADLINE.txt`; `RESULTS:33` |
| 18 | 09-03 | **`matched_w3/dHv2`, 60 tapes, w3** (dHpruned of record for v2) | w3 | hold-66 / rnd | selected | 8 | rnd 122/240 = **0.508**, hold 514/528 = 0.973 | `C:…/dp_v2fullPw3/dHv2_DP_s5?/sweep/HEADLINE.txt`; `CROSS_LEARNER` §2.2 |
| 18b | " | " | w3 | " | LAST | 8 | rnd 121/240 = **0.504** (15 13 15 19 15 15 13 16), hold 507/528 = 0.960 | same |
| 19 | 09-04 | `matched_w3/dHfull_pruned`, 64 full-task tapes pruned (`prune_full_v1`), w3 — the phase-plan teacher | w3 | sel / hold / rnd | 100k (= selected = LAST) | 1 | sel 14/15, **hold 14/15 = 0.933**, rnd 14/30 = 0.467 | `C:…/dp_phase/dHfull_pruned_DP_s0/sweep/HEADLINE.txt`; sidecar `checkpoints/100000/dp_sidecar.json` |
| — | 08-24 | `matched_w4_pilot/dH` (ts5_w4 pilots) | w4 | — | — | 2 | **no HEADLINE** (never scored) | `C:…/dp_pilotw4/dH_DP_s{0,1}` |

Raw-human DP rows (disclosed within-source legs, not "the pruned set"): `dHv2raw` old world sel rnd 84/240 = 0.350
(`dp_v2full`), w3 sel 57/240 = 0.237 / LAST 49/240 = 0.204 (`dp_v2fullw3`), pilots 5–8/30 — `RESULTS:33-34` (A29 met).

### 1.2 Machine / other DP arms (context for the "0.80" search)

| date | arm (set, N, world) | IC set | ckpt | seeds | picked | source |
|---|---|---|---|---|---|---|
| 08-02..09 | `dDP_DP` (`m1all_harvest_succ_lerobot`, 66 cap-1200 DP-teacher tapes, base) | in-dist-15 / random-15 | last | 8 | **0.800** (0.67–0.93) / 0.225 (0.13–0.33); hardened re-eval 0.792 / 0.217 | `W:genesis_paper/dDP_DP_s{0..7}-eval`; `results_core_matrix.md:35-43` |
| 08-02 | ouroboros lineage gen-1 / gen-2 (cap-600 harvests, single seed) | in-dist-15 / random-15 | last | 1 | 0.867 / 0.333 (both) | `W:genesis_pickaplace_ouro`; `results_core_matrix.md:130-131` |
| 08-19 | `dR2D_DP` (`lerobot_dR2D_pick`, 66 champion tapes, base) | in-dist-15 / random-15 | last | 3 | **0.96** / **0.76** | `L:paper/ROUND_ROBIN_RESULTS_2026-08-22.md:30,47` |
| 08-24..25 | `dDP` old world (`matched_v2/dDP`, 56) | hold / rnd | selected | 5 (+s19) | hold 0.893 (14 14 14 14 11), rnd 83/150 = 0.553 (16 18 16 17 16); LAST rnd 0.553 | `C:…/dp_final/dDP_DP_s*/sweep/HEADLINE.txt` |
| 08-24..25 | `dR2D` old world (`matched_v2/dR2D`, 56) | hold / rnd | selected | 5 | hold 0.827, rnd 91/150 = **0.607** (17 21 20 16 17) | `C:…/dp_final/dR2D_DP_s1?/sweep/HEADLINE.txt` |
| 08-24..26 | `dDP` w3 (`matched_w3/dDP`, 58) | hold / rnd | selected / LAST | 10 | 0.873 / 0.487; LAST 0.920 / **0.467** (17 14 13 15 12 15 11 15 16 12) | `C:…/dp_w2final/dDP_DP_s2?/sweep/HEADLINE.txt` |
| 09-02/03 | `dDPv2p` w3 (60) | hold-66 / rnd | selected / LAST | 8 | 0.962 / 0.492; LAST 0.943 / 0.525 | `C:…/dp_v2fullPw3/dDPv2p_DP_s5?/sweep/HEADLINE.txt` |
| 08-26.. | `dDPallpruned_1e3/1e2` w3 | hold / rnd | selected | 2 / 2 | rnd 17 14 / 13 16 | `C:…/dp_density/` |

## 2. Where the ~0.80 exists

Exactly these cells, all **in-distribution (demo starts)**, none on random starts:

| cell | value | what it is |
|---|---|---|
| `hdp_s2-eval` 07-31 | 0.800 in-dist (12/15) — random 0.067 | single seed, `lerobot_dataset_pick_pruned`, old world, 15 demo starts; the third of the "0.67/0.73/0.80" positive-control trio (`PAPER_PLAN.md:401-405`, `:111`) |
| `dH_DP_s0-eval` 08-02 (and 08-10 re-eval) | 0.800 in-dist (12/15) — random 0.133 | the best of 8 seeds whose arm mean is **0.617**; `dH_DP_s3` also hit 0.80 on the 08-11 re-eval |
| `dDP_DP` arm 08-09 | mean **0.800** in-dist (0.67–0.93), n = 8 — random 0.225 | the **machine-demo** arm of the August BC matrix ("BC in-dist P(model > human) = 0.994") — a different data source |
| `dH` w3 hold-15, LAST | **0.913** (137/150), n = 10; selected 0.887 | the current design's in-distribution cell (`MORNING_TABLE` reports only rnd); ceiling is 14/15 = 0.933 because hold uid 331 is not in any pruned training set and is picked 0/10 |
| `dH` w3 sel-15, LAST | **0.853** (128/150), n = 10 | the *same 15 uids* as the August 0.617 in-dist cell (`eval_ics.json` `sel_note`); ceiling 14/15 (uid 234 lying can, 0/430 ever, `METHODS_draft:433`) |
| `dHfull_pruned_DP_s0` 09-04 | hold **14/15 = 0.933**, rnd 14/30 = 0.467 | the full-task DP teacher of the phase plan — "hold 14/15" in the 09-04/05 status lines |
| `dHv2` w3 hold-66 | 0.973 selected / 0.960 LAST, n = 8 | v2 pruned set scored on its own 66 training starts |

There is no DP-on-pruned-human random-start cell above 0.63 in any single seed (old world s14, 19/30) or 0.589 as an
arm mean (old world, LAST, n = 6); in the corrected world the best seed is 18/30 = 0.60.

## 3. Decomposition of the gap to 0.520

Two chains are needed because the "0.80" and the 0.520 differ in almost every column. Chain A stays inside the August
block (where the 0.80 seed lives); chain B stays inside the frozen corrected-world block (where the 0.520 lives). The
cross-block terms (recipe, world, dataset version) are §3.4–3.6.

### 3.1 IC set — the dominant term (≈ −0.39 in both blocks, with a measured mechanism)

| block | in-distribution | random | Δ |
|---|---|---|---|
| August (`lerobot_dH_pick`, base, LAST, n = 8) | 0.617 (in-dist-15) | 0.233 (random-15) | **−0.38** |
| frozen w3 (`matched_w3/dH`, LAST, n = 10) | 0.913 (hold-15) / 0.853 (sel-15) | 0.520 (rnd-30) | **−0.39 / −0.33** |
| frozen w3, rnd split | — | rnd 0–14 (= the August random-15) **0.400** (60/150); rnd 15–29 **0.640** (96/150) | |

Per-IC tally at the LAST checkpoint (picks out of seeds; DP from `dp_w2final/*/sweep/{final,selected}/sweep.json`
per-episode records, RLPD from `rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s6?/sweep/*/sweep.json`, WM from
`$LAB/wm_fix_2026-09-03/runs/s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s?/fresh_eval_rnd30_mode/metrics.json`
`per_episode[].outcome`). "nearest train" = distance from the rnd can xy to the nearest of the 58 `matched_w3/dH`
training can positions (`C:matched_w3/dH/manifest.json` `ic_uid_histogram` × `L:can_pos_recovery/trial_placements.json`);
the pruned set's training cans span x ∈ [0.383, 0.513], y ∈ [−0.203, 0.141]; the rnd sampler's box is
[0.3395, 0.592] × [−0.237, 0.189] (`eval_ics.json` `support_box`, = all *solved* uids' bounding box + 1 cm,
`L:baselines/ic_sampling.py:14-31`).

| rnd k | can x | can y | nearest train (cm) | DP dH /10 | DP dDP /10 | DP dHv2 /8 | RLPD dHv2raw /8 | WM dHv2raw /8 |
|---|---|---|---|---|---|---|---|---|
| 0 | 0.500 | −0.122 | 5.7 | 8 | 8 | 7 | 5 | 7 |
| 1 | 0.350 | −0.230 | 4.9 | 1 | 1 | 4 | 6 | 3 |
| **2** | **0.545** | 0.152 | 4.6 | **0** | 0 | 0 | 0 | 0 |
| 3 | 0.493 | 0.074 | 1.3 | 9 | 8 | 8 | 8 | 7 |
| 4 | 0.477 | 0.161 | 2.2 | 9 | 10 | 7 | 7 | 8 |
| **5** | **0.546** | −0.236 | 8.3 | **0** | 0 | 0 | 0 | 0 |
| **6** | **0.556** | −0.223 | 8.4 | **0** | 0 | 0 | 1 | 0 |
| **7** | **0.524** | −0.162 | 4.6 | **0** | 0 | 0 | **6** | **8** |
| **8** | **0.557** | −0.006 | 7.1 | **0** | 0 | 0 | **3** | **4** |
| 9 | 0.415 | −0.057 | 4.1 | 4 | 5 | 6 | 6 | 5 |
| **10** | 0.347 | −0.184 | 4.8 | **0** | 0 | 0 | **3** | 0 |
| 11 | 0.509 | 0.039 | 4.8 | 10 | 7 | 8 | 7 | 7 |
| 12 | 0.495 | −0.074 | 2.6 | 9 | 10 | 7 | 7 | 7 |
| **13** | **0.591** | 0.181 | 10.0 | **0** | 0 | 0 | **2** | **5** |
| 14 | 0.513 | 0.040 | 5.1 | 10 | 6 | 8 | 7 | 8 |
| 15 | 0.513 | −0.071 | 3.7 | 7 | 7 | 7 | 6 | 7 |
| 16 | 0.374 | 0.070 | 5.0 | 3 | 1 | 3 | 3 | 1 |
| 17 | 0.472 | −0.105 | 4.0 | 9 | 8 | 2 | 7 | 8 |
| 18 | 0.462 | 0.142 | 0.6 | 10 | 9 | 8 | 7 | 7 |
| **19** | **0.575** | −0.085 | 9.3 | **0** | 0 | 0 | **3** | **2** |
| 20 | 0.484 | −0.100 | 4.9 | 10 | 9 | 4 | 7 | 8 |
| 21 | 0.490 | −0.093 | 4.3 | 10 | 10 | 6 | 7 | 8 |
| 22 | 0.438 | 0.142 | 1.8 | 9 | 10 | 7 | 7 | 6 |
| 23 | 0.397 | 0.028 | 1.6 | 8 | 1 | 2 | 7 | 8 |
| 24 | 0.361 | 0.117 | 7.9 | 5 | 9 | 4 | 0 | 0 |
| **25** | **0.538** | −0.135 | 7.4 | **0** | 0 | 0 | 0 | 0 |
| **26** | **0.561** | −0.212 | 8.4 | **0** | 0 | 0 | 1 | 0 |
| 27 | 0.424 | −0.173 | 0.6 | 10 | 10 | 8 | 7 | 8 |
| 28 | 0.453 | 0.102 | 0.6 | 9 | 10 | 7 | 7 | 8 |
| 29 | 0.398 | −0.215 | 1.3 | 6 | 1 | 8 | 7 | 8 |
| **total** | | | | **156/300 = 0.520** | 140/300 = 0.467 | 121/240 = 0.504 | 144/240 = 0.600 | 148/240 = 0.617 |

- **Ten rnd ICs are dead for every DP arm** in the corrected world: {2, 5, 6, 7, 8, 10, 13, 19, 25, 26} — 0/10 dH,
  0/10 dDP, 0/8 dHv2 (28 seeds, 280 episodes, 0 picks). **Nine of the ten have can x ≥ 0.524 m — beyond the farthest
  training can (0.513)**; the tenth (k = 10) sits 4.8 cm from the nearest training can at the near-base y-edge. Median
  nearest-training distance: dead 7.8 cm, live 3.8 cm. No live IC has x ≥ 0.52.
- On the 20 live ICs the pruned human DP picks **156/200 = 0.780**; the machine arm 140/200 = 0.700. **0.780 × 20/30 =
  0.520** — the number of record *is* the ~0.8 the user remembers, diluted by a third of the evaluation set that lies
  outside the pruned set's support.
- The dead ICs are not unreachable: RLPD (dHv2raw, LAST) picks 5 of them (k = 7, 8, 10, 13, 19 at 2–6 of 8) and the
  world model 4 (k = 7 at 8/8, 8 at 4/8, 13 at 5/8, 19 at 2/8); only k = 2, 5, 25 (and 6, 26 at ≤ 1/8) are dead for
  every learner, while k = 24 is the reverse case (DP 5/10, RLPD/WM 0/8). Online
  learners extrapolate to far-x cans that the imitator never does — consistent with DP learning absolute joint targets
  from a support that ends at x = 0.513.
- Old world, same rnd file (`dp_final/dH`, LAST, n = 6): dead set {5, 7, 9, 10, 13, 16, 19, 22} (8 ICs), i.e. the far-x
  ICs 2, 6, 8, 25, 26 were still pickable in the base world (1–6 of 6) and three near/mid ICs (9, 16, 22) were not. The
  corrected world (3 cm riser, kp×4, gravity compensation) moved the reachable set outward at the base and inward at far
  x for DP — the world term of §3.5 is concentrated on rnd 0–14 (0.533 old → 0.400 w3) while rnd 15–29 is unchanged
  (0.644 → 0.640).
- This refines CONFOUNDS row 31 ("rnd is an in-support sample"): the box is the *all-solved-uid* box (61 uids including
  solved fails and the lying-can trials), and one third of it lies outside the *success-tape* support that any DP arm
  trains on. Not registered here; flagged for the ledger (§5).

### 3.2 Seed selection (max-of-seeds vs arm mean): −0.13 to −0.18

The 0.80s are the best seed of their arms: July trio 0.667/0.733/0.800 → mean 0.733 (n = 3); August 0.80 vs mean 0.617
(n = 8, SD 0.111). In the frozen block the best seed is 0.60 on rnd (s20/21/22 18/30) vs mean 0.520 (SD 0.072).

### 3.3 Checkpoint rule: −0.027 on rnd, +0.027 on hold

Frozen w3 dH: selected 0.547 → LAST 0.520 on rnd (per seed: 18 19 18 14 13 15 16 16 18 17 → 18 18 18 14 13 15 12 16 17
15); hold 0.887 → 0.913 (selection on `sel` does not transfer to hold). Old world: selected 0.567 → LAST 0.589 (n = 6).
The August rows were `last` only. Rule-invariant within ±0.03, as `ADVISOR_BRIEF` §2b states.

### 3.4 Recipe + dataset re-execution (stride-1 → contract-v1 repeat-4 sets, 08-23): **+0.30 on random starts**

Same 15 random ICs, same world (base), same 100k-step lerobot DP: August `lerobot_dH_pick` stride-1 LAST random-15
**0.233** (n = 8) → `matched_v1_pilot/dH` repeat-4 pilot selected rnd-30 0.50 (n = 2, `PAPER_PLAN:252`) →
`matched_v2/dH` repeat-4 LAST rnd 0–14 **0.533** (n = 6, `dp_final`). In-distribution on the same 15 uids: 0.617 → 0.800
(LAST sel-15, n = 6). The 08-23 change (decision-rate 7.5 fps dataset, hold-4 delta integrator at eval,
`L:cluster/sbatch_dp.sh:5-9`, `L:baselines/wandb_eval.py:298-320`; follower-re-executed tapes, `METHODS_draft` §4.1) is
the single largest *improvement* in the DP lineage. (Dataset-content changes are entangled with the recipe here: 66
stride-1 tapes vs 51/56 re-executed tapes; the v1 pilot's 51 tapes already show the jump.)

### 3.5 World (base → `gc_kp4_riser3_shelf6`): about −0.05 to −0.07 on rnd, +0.05 on sel/hold

LAST rnd: old 0.589 (n = 6, SD 0.034) → w3 0.520 (n = 10, SD 0.072), Δ −0.069; selected 0.567 → 0.547, Δ −0.020.
LAST sel-15: 0.800 → 0.853; LAST hold: 0.833 → 0.913. The rnd loss is entirely the far-x ICs (§3.1, last bullet); the
sets differ (56 vs 58 tapes, `--arrival either` both) so this is world + set-difference, n = 6 v 10, inside ~1 seed SD.

### 3.6 Dataset version within the corrected world: ≤ −0.02

`matched_w3/dH` (58, frozen) LAST rnd 0.520 → `matched_w3/dHv2` (60, recovered corpus) LAST 0.504 / selected 0.508
(Δ −0.016 / −0.039, n = 10 v 8); density controls `dHallpruned_1e3/1e2` selected 0.578 (n = 3 each); unpruned control
old world 0.511 selected (n = 3) vs pruned old-world 0.567. The same ten rnd ICs are dead for dHv2 (table above) — the
recovered corpus adds no far-x training can (dHv2 lacks 245 246 286 293 295 300 331 too, `CONFOUNDS` row 26).
`dHfull_pruned` (64 full-task tapes) rnd 14/30 = 0.467 (n = 1) — a full-task teacher, not a pick-scope arm.

### 3.7 Evaluation protocol: ≈ 0

- Horizon: 1200 sim steps in every row since July (`W:` eval configs `max_steps 1200`; `eval_sweep.sh:37`).
- Predicate: hardened 08-09 (`aa762ac`, `genesis_can_env.py:48-60`); re-evaluating the August 16 checkpoints moved
  dH in-dist 0.617 → 0.633 and random 0.233 → 0.208 (`results_matrix_2026-08-11.md:52-56`, "no seed by more than ±0.20,
  no mean by more than 0.025"). The 08-28 change (`d2c391f`, pick scope pays one terminal) is reward-side and does not
  touch DP. No tip termination at eval in any row (`genesis_can_env.py:275`).
- Action selection: sampled in every row; seeded per IC since 08-17 (`080cb73`); no deterministic DP re-score exists
  (`CROSS_LEARNER` §3 item 3) — sampling noise is inside the per-seed spread, not a bias.
- Process: one fresh process per episode since 08-23 (`eval_sweep.sh:110-131`) vs one process per IC block before;
  unquantified for DP pick, bounded by the July observation that only borderline episodes flip under load (`CLAUDE.md`
  07-20 gates) — same harness for both arms of every contrast.
- Dual-IC semantics flip of 07-30 (`3bf4d91`, `July30th_Fable.md:200-205`): `eval/*` meant random before, demo starts
  after — the reason a July "0.13" and a July "0.67" can be the same policy. Rows 2–3 above are random-start numbers.

### 3.8 Seed variance: then 0.00–0.53, now 0.40–0.60

Random-start per-seed spread: 07-18 (38 demos, stride 1) 0.00/0.07/0.47; 07-20 0.40/0.07/0.53; August n = 8 0.07–0.40
(SD 0.107); old-world repeat-4 n = 6 0.53–0.63 (SD 0.034); **frozen w3 n = 10 0.40–0.60 (SD 0.072)**; dHv2 n = 8
0.43–0.63 (SD 0.063). The historical "0.00–0.53" spread describes the July recipe on tiny sets; the current arm's
95 % interval on its mean is roughly 0.52 ± 0.05.

### 3.9 Reconciliation

Chain A (August block, old world, stride-1): 0.800 (best seed, 15 demo starts) → 0.617 (arm mean, −0.18) → 0.233 (the
same seeds on 15 random starts, −0.38).
Chain B (frozen w3 block): 0.913 hold-15 LAST → 0.853 on the August in-dist uids (−0.06; 7 of those 15 are not in the
58-tape set, incl. lying-can 234) → 0.780 on the 20 rnd ICs inside the training support (−0.07) → **0.520 on all 30**
(−0.26, ten out-of-support ICs picked 0/280) → 0.547 if the selected checkpoint is used instead (+0.027).
Cross-block: August random-15 0.233 → frozen w3 rnd 0–14 0.400 (+0.17 net of recipe +0.30 and world −0.13 on those ICs).

## 4. Verdict

0.52 is a different cell from every ~0.80 on record — random starts vs demo starts — and inside that cell it is a
support-limited ceiling, not a regression: the pruned human DP has never picked a can placed beyond x ≈ 0.52 m from any
of 28 seeds, and one third of the fixed rnd-30 set is placed there. On every cell that existed when the 0.80s were
recorded, the arm is now higher (in-dist-15: 0.62 → 0.85; random-15: 0.23 → 0.40; hold-15: 0.91 ≈ ceiling 0.93). The
one genuine loss since August is the corrected world's −0.07 on random starts, itself concentrated on the far-x ICs,
and it is shared by the machine arm (0.467 = 0.70 on the live 20). DP's human-vs-machine contrast (+0.053 LAST,
+0.06 selected) is unaffected: both arms carry the same ten dead ICs.

## 5. What, if anything, to rerun

Nothing is required to settle the question: the like-for-like cells exist and are cited above. Three cheap follow-ups
would make the DP row easier to read (none run here; register nothing new by this doc):
1. **Report rnd-30 stratified by pruned-set support** (20 in / 10 out) next to every DP number in `MORNING_TABLE` §1 —
   pure re-tabulation of existing `sweep.json` files (this doc's §3.1 is that tabulation for the three w3 arms).
2. The queued `rnd2` set (CONFOUNDS row 31) should be drawn from the *success-tape* support box, or reported with the
   same stratification; otherwise it will re-import the same 1/3 extrapolation share. Suggested ledger entry: "rnd-30
   support box is the all-solved-uid box; 10/30 ICs (9 at x ≥ 0.52) lie outside every pruned set's training support and
   are picked 0/280 by DP; RLPD/WM pick 5 of them."
3. Optional, ≤ 2 GPU jobs if ever wanted: a deterministic (`mode`) re-score of the 10 frozen dH LAST checkpoints on rnd-30
   (CONFOUNDS row 33) would bound diffusion-sampling noise; it cannot move the dead-IC term (0/280) and is not needed for
   this question.

## 6. Sources consulted

Docs: `paper/MORNING_TABLE_2026-09-04.md` §1; `paper/RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §2, §4.3;
`paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` §1–3; `paper/CONFOUNDS.md` rows 2, 4, 25, 26, 31, 33, 34, 40;
`paper/figures/ADVISOR_BRIEF_2026-09-01.md` §2–2b; `paper/RESULTS_for_writing_2026-08-30.md` §1;
`paper/RESULTS_TABLE_2026-08-25.md`; `paper/UPDATE_2026-08-25.md` §3a; `paper/results_core_matrix.md`;
`paper/results_matrix_2026-08-11.md`; `paper/RESULTS_MATRIX_2026-08-15.md`; `paper/ROUND_ROBIN_RESULTS_2026-08-22.md`;
`paper/METHODS_draft_2026-08-28.md` §4.1, §4.5, §5.1, ceiling row; `paper/PREREG_final_round_robin_2026-08-23.md` A29;
`PAPER_PLAN.md:97-120, 236-330, 395-470`; `July30th_Fable.md` §2, §5–6, 08-01 addenda; `CLAUDE.md` Agent Status lines
23, 31, 34, 36, 44, 46.
Code: `cluster/sbatch_dp.sh`, `cluster/dp_select_confirm.sh`, `cluster/eval_sweep.sh`, `baselines/wandb_eval.py`,
`baselines/ic_sampling.py`, `baselines/make_dp_pruned.py`, `baselines/genesis_can_env.py`, `baselines/eval_ics.json`,
`baselines/eval_ics_v2_w3.json`, `can_pos_recovery/trial_placements.json`; git log of `genesis_can_env.py` /
`wandb_eval.py` (`aa762ac`, `3bf4d91`, `d53a5b1`, `080cb73`).
Cluster (read-only): every `baselines/outputs/dp_*/*/sweep/HEADLINE.txt` (184 DP registry rows,
`cluster/RUN_REGISTRY.jsonl`), `sweep/{020000..100000,selected,final}/sweep.json` per-episode records for `dp_w2final`,
`dp_final`, `dp_v2fullPw3`, `dp_v2fullw3`, `dp_pilot*`, `dp_phase`; `dp_sidecar.json` of `dp_w2final/dH_DP_s20` and
`dp_phase/dHfull_pruned_DP_s0`; `matched_w3/{dH,dHv2,dDP,dHfull_pruned}/manifest.json`, `matched_v2/{dH,dHunpruned}`,
`matched_v1_pilot/dH`; `rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s6?/`, `rlpd_g99w3_{dH,dDP}_s4?/` sweep records;
`$LAB/wm_fix_2026-09-03/runs/s2_r2d_pick_state_{dHv2raw,dDP}_bnormclamp1ent5_s?/fresh_eval_rnd30_mode/metrics.json`.
wandb (read-only, `jambotime`): `genesis_paper` `d{H,DP}_DP_s{0..7}-eval` (summaries + configs, both eval waves),
`dH_DP_s0` train config; `genesis_pickaplace` `audit_joint_rebuilt-eval`, `hdp_s{1,2}-eval`,
`dp_pick_0720_1754-eval`, `dp_pick_pruned_0720_2117-eval`, `dp_v2r_0831` configs.
