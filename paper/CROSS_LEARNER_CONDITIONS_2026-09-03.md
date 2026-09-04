# CROSS-LEARNER CONDITIONS — DP and RLPD in the corrected world (`gc_kp4_riser3_shelf6`, "w3"), read-only audit 2026-09-03 ~23:30

Purpose: every condition and number a like-to-like comparison against the world-model (WM) arm needs, each cell cited or `NOT FOUND`.
Nothing here was recomputed by training; sums/means below are arithmetic over the cited per-seed counts. Path legend: `L:` = this repo
(`/home/james/workspace/genesis_pickaplace`), `C:` = cluster copy `/cluster/tufts/shortlab/jstale02/genesis_pickaplace` (read via `ssh pax`).
Newest number sources: `C:baselines/outputs/dp_*/<arm>_DP_s<seed>/sweep/HEADLINE.txt` (DP), `C:rlpd_<job>.out` / `C:rlpdsel_<job>.out`
`SWEEP-HEADLINE` lines (RLPD). Where a results doc and a HEADLINE disagree, both are given.

## 0. WM reference (the side DP/RLPD are matched against — as stated by the WM-fix session, not re-verified here)
| item | WM value | cite |
|---|---|---|
| world | `gc_kp4_riser3_shelf6`, `[sim-variant]` line asserted in every eval | `L:paper/WM_FIX_LOG_2026-09-03.md:880-882` |
| demo sets | `demos_state/{dH,dDP}` from `matched_w3/{dH,dDP}` (58/58 tapes) via `to_dreamer_native.py --with-state --repeat 4 --terminal-reward 1`; raw human `demos_state/dHv2raw` (66 tapes) | `WM_FIX_LOG:237-240,931-933`; `WM_FIX_PLAN_2026-09-03.md:376-377` |
| obs | 17-dim state (`genesis_can_env._obs()['state']`), `mlp_keys 'state'`, `cnn_keys '$^'` | `WM_FIX_LOG:810`; `WM_FIX_PLAN:157-158` |
| action | delta_joint cap 0.025 / leash 5×, action_repeat 4 | `WM_FIX_LOG:812` |
| reward | sparse +1 on demo AND online rows, reward_scale 1, return_clamp 1.0, act_entropy 3e-5, stock `bounded_normal` actor | `WM_FIX_LOG:810-811` |
| budget / horizon | "1M env steps per run" (unit sim-steps vs decisions: NOT FOUND); `time_limit 1200`; buffer 5e5, no re-injection | `WM_FIX_LOG:811-812,882`; `WM_FIX_PLAN:163` |
| eval | fresh-process `eval_genesis.py`, END-OF-RUN ckpt (LAST), 1200 sim steps, `baselines/eval_ics.json` hold15 + rnd30 (+ alldemo74), `picked`, sample AND mode; one process per IC set (CONFOUNDS row 40 wording) | `WM_FIX_LOG:880-882,991-993`; `L:paper/CONFOUNDS.md:48` |
| seeds / stat | gate n=2/arm (dH s0,s2; dDP s0,s1); expansion to n=8/arm submitted 23:05; registered statistic "LAST ckpt, rnd30 MODE, n=8 vs 8, exact permutation" | `WM_FIX_LOG:884-890,985-989` |
| WM numbers (n=2) | dH hold 15/15,13/15 (sample) rnd 18/30,18/30; dDP hold 15/15,14/15 rnd 20/30,17/30 (mode within 1 episode) | `WM_FIX_LOG:886-889` |
| NB | the FROZEN WM block (s80-87) was pixels-only, TL 400, dense ×100, BEST-of-K — a different learner from the stage-2 recipe | `CONFOUNDS.md:26,33,52` |

## 1. Conditions
### 1.1 Learner-wide conditions (apply to every arm row of that learner; cited once)
| item | DP (lerobot Diffusion Policy) | RLPD (SB3 SAC subclass) |
|---|---|---|
| world: how set | `SIM_VARIANT` env var → exported `SIM_VARIANT_FOR_SIDECAR` → `dp_sidecar.json.sim_variant` (`L:cluster/sbatch_dp.sh:195-196,475`); manifest `sim_variant` gate (`:300-303`); eval resolves CLI > sidecar > `base` (`L:baselines/wandb_eval.py:123-140`). **Registry knobs carry NO sim_variant** (`sbatch_dp.sh:352-354`) — world is only in the sidecar/dataset path. Verified sidecar: `C:baselines/outputs/dp_w2final/dH_DP_s20/checkpoints/100000/dp_sidecar.json` `"sim_variant": "gc_kp4_riser3_shelf6"` | `SIM_VARIANT` env var → `--sim-variant` to `train_rlpd.py` + registry knob (`L:cluster/sbatch_rlpd.sh:87,190,209`); manifest gate (`:169-172`); sidecar carries it (`C:baselines/rl/checkpoints/rlpd_g99w3_dH_s40/ckpt_100/rlpd_ckpt.action_mode.json` `"sim_variant": "gc_kp4_riser3_shelf6"`); `rlpd_select_confirm.sh` asserts IC-file vs sidecar world (`L:cluster/rlpd_select_confirm.sh:80-91`) |
| observation | 17 numbers = `observation.state` (8: q[6], grip motor, grip effort) + `observation.environment_state` (9: can xyz, can quat, goal xy); no images (`L:baselines/convert_to_lerobot.py:78-90`; `L:cluster/sbatch_dp.sh:82-85`; `L:baselines/dp_runner.py:7-8,62-65`); state source `L:baselines/genesis_can_env.py:296-313` | 17-dim `state` vector, `STATE_DIM = 17` (`L:baselines/rl/pick_env.py:36`; `L:baselines/rl/full_env.py:402`); same `_obs` (`genesis_can_env.py:296-313`) |
| action space | TRAIN: 7 = 6 absolute joint targets (window-end) + grip 0..1, dataset fps 7.5 (`L:paper/METHODS_draft_2026-08-28.md:304`; `sbatch_dp.sh:330-337`). EVAL: hold-4 — q* → `clip((q*−target)/(4·0.025))` integrated 0.025/step on the running target, leash 0.125, `delta_ref='target'` (`L:baselines/wandb_eval.py:298-320`); chunk n_action_steps 8 lerobot default (`METHODS_draft:306`) | delta_joint `[-1,1]^7`, `--action-mode delta_joint --delta-ref target` (`sbatch_rlpd.sh:196`), cap 0.025 rad/sim-step, leash 5×0.025 = 0.125 (`full_env.py:275-276,317-318`); eval integrator mirrors it (`wandb_eval.py:222-256`) |
| training budget | 100 000 grad steps, batch 64, `BUDGET_UNIT=grad_steps` (`sbatch_dp.sh:187,198,452`); K=5 ckpts at 20/40/60/80/100k (`:203-204`) | 100 000 DECISIONS = 400 000 sim steps at repeat 4, `BUDGET_UNIT=decisions` (`sbatch_rlpd.sh:33,80-81,89`); UTD 10, E=10, Z=2, demo batch 128 (`:198-199`); γ 0.99 UTD 10 for every w3 row of record (registry, §1.2); sidecar `"steps": 100000, "steps_unit": "decisions"` |
| action_repeat | 4 (`sbatch_dp.sh:190`; sidecar `action_repeat: 4`; eval refuses a mismatch `wandb_eval.py:293-296`) | 4 (`sbatch_rlpd.sh:81`; sidecar `action_repeat: 4`) |
| episode horizon | train: n/a (offline); eval 1200 sim steps (`sbatch_dp.sh:191`; `L:cluster/eval_sweep.sh:37,128`) | train 1200 sim steps = 300 decisions (`sbatch_rlpd.sh:82`; sidecar `train_max_steps: 1200`); eval 1200 (`:83`) |
| eval harness | `cluster/eval_sweep.sh` → `wandb_eval.py`, ONE FRESH PROCESS PER EPISODE, sim CPU, policy GPU if visible (`eval_sweep.sh:23-25,110-131`); per-episode json, denominators asserted, missing ≠ 0 (`:138-166`) | same harness, `--kind sac`, policy CPU (`eval_sweep.sh:126`; `sbatch_rlpd.sh:211-212,268-282`) |
| success predicate | `picked` = hardened lift (z > pick_z ∧ grip closed ∧ eef–can < 0.20, sustained 10 frames) (`genesis_can_env.py:48-60,258-265`); aggregated as `picked` (`eval_sweep.sh:150`); no tip termination at eval, `done = t ≥ max_steps` (`genesis_can_env.py:275`) | same |
| action selection at eval | SAMPLED (diffusion noise seeded per episode `--seed k`) (`wandb_eval.py:105-109,436`; `eval_sweep.sh:118-119,128`) | DETERMINISTIC `predict(deterministic=True)` (`wandb_eval.py:236,263,436`) |
| checkpoint rule | 5 ckpts scored on `sel`; best (ties → later) CONFIRMED on hold+rnd; final (100k) also on hold+rnd; headline = selected (`L:cluster/dp_select_confirm.sh:5-6,22-38`; PREREG §5/A7 `L:paper/PREREG_final_round_robin_2026-08-23.md:164-167,276`) | identical selection (`sbatch_rlpd.sh:248-282`); registered STATISTIC = LAST ckpt rnd-30 (A16/A20 `PREREG:350,382`); both selected and final reported in every headline |
| statistic | `analysis/stats.py`: seed is the unit, Welch + exact permutation, min attainable p (`L:paper/RESULTS_for_writing_2026-08-30.md:5`); PREREG §7 says paired permutation but stats.py is unpaired (`CONFOUNDS.md:36` (c)) | same |

### 1.2 Per-arm cells (corrected world only; N = tapes; rows = decision rows from the manifest / lerobot `total_frames`)
| learner × arm | demo set (path, N, rows) | IC file → sel/hold/rnd | seeds (n) | wave / jobs | run in w3? |
|---|---|---|---|---|---|
| DP × dH (frozen pruned human) | `C:baselines/matched_w3/dH/lerobot`, N=58, 6927 rows, fps 7.5, sha 07ce3f36… (`C:matched_w3/dH/manifest.json`, `lerobot/meta/info.json`; registry row dataset_root) | `baselines/eval_ics.json` → 15/15/30 (`L:baselines/eval_ics.json` keys; `sbatch_dp.sh:197`) | 20–29 (10) (`C:cluster/RUN_REGISTRY.jsonl` sbatch_dp.sh arm=dH wave=w2final) | wave **`w2final`** (name is misleading: dataset is matched_w3) — `C:baselines/outputs/dp_w2final/dH_DP_s2?/`; sidecar git 2061f08, 2026-08-24 | yes |
| DP × dDP (frozen pruned-matched machine) | `matched_w3/dDP/lerobot`, N=58, 7285 rows, sha d3bf95f6… | `eval_ics.json` 15/15/30 | 20–29 (10) | `w2final`, `dp_w2final/dDP_DP_s2?/` | yes |
| DP × dHpruned = dHv2 (v2 pruned human) | `matched_w3/dHv2/lerobot`, N=60, 7129 rows, sha aaa88a81… | `baselines/eval_ics_v2_w3.json` → sel 15 / **hold 66 (= ALL training ICs, sel ⊂ hold)** / rnd 30 (byte-copy of eval_ics rnd) (`C:baselines/eval_ics_v2_w3.json` notes; `PREREG:414-420` A23; `L:cluster/a31_chain/common.sh:18` via `C:`) | 50–57 (8) (registry) | `v2fullPw3`, `dp_v2fullPw3/dHv2_DP_s5?/` | yes |
| DP × dDPpruned = dDPv2p (v2 machine, matched to pruned base) | `matched_w3/dDPv2p/lerobot`, N=60, 7067 rows, sha 18fa94f3… | `eval_ics_v2_w3.json` 15/66/30 | 50–57 (8) | `v2fullPw3`, `dp_v2fullPw3/dDPv2p_DP_s5?/` (a31 chain `common.sh:18`, `submit_learners.sh:21`) | yes |
| DP × dHv2raw (raw human, DISCLOSED within-source leg) | `matched_w3/dHv2raw/lerobot`, N=66, 14 323 rows, sha bd77c9c3… | `eval_ics_v2_w3.json` 15/66/30 | 50–57 (8) | `v2fullw3`, `dp_v2fullw3/dHv2raw_DP_s5?/` | yes |
| DP × dDPv2 (raw-matched machine) | — | — | — | — | **NOT RUN for DP** (no `sbatch_dp.sh` registry row for arm dDPv2; A31 makes dDPv2p the DP machine arm `PREREG:512-514`) |
| RLPD × dH (frozen pruned human), sparse | `C:baselines/matched_w3/dH` (native npz), N=58, sha 07ce3f36… (sidecar `demo_dir`, `demo_n_eps: 58`) | `eval_ics.json` 15/15/30 (`sbatch_rlpd.sh:88`) | 40–47 (8) (registry: g99w3, γ 0.99, UTD 10) | `g99w3` (A20 `PREREG:376-386`), `C:rlpd_3085499-3085514.out`, `C:baselines/rl/checkpoints/rlpd_g99w3_dH_s4?/` | yes |
| RLPD × dDP (frozen machine), sparse | `matched_w3/dDP`, N=58, sha d3bf95f6… | `eval_ics.json` 15/15/30 | 40–47 (8) | `g99w3`, same .out set | yes |
| RLPD × dH / dDP, DENSE (A33) | same sets | `eval_ics.json` | 60–67 (8 v 8) | `g99w3dense`, `C:rlpd_3163620-3163636.out` (`RESULTS:165-167`) | yes (collapsed, §2) |
| RLPD × dHv2raw (raw human), sparse | `matched_w3/dHv2raw`, N=66, 14 323 rows, sha bd77c9c3… (registry demo_sha) | `eval_ics_v2_w3.json` 15/66/30 (`L:cluster/rlpd_select_confirm.sh:31,58`) | **60–67 (8) = arm of record** (`RESULTS:178-180`); re-scored from archived ckpts on `-p batch` CPU (`rlpd_select_confirm.sh:39-46`; `C:rlpdsel_3162545…3223232.out`). ALSO registry rows s50–57 same wave (jobs 3118794…3118822, 09-01, FAILED at final flush; only s51's .out carries a headline: rnd 23/30) — not cited in any results doc | `g99v2fullw3` | yes |
| RLPD × dDPv2 (raw-matched machine), sparse | `matched_w3/dDPv2`, N=66, 7826 rows, sha 3e4baae8… | `eval_ics_v2_w3.json` 15/66/30 (`submit_learners.sh:22`) | 50–57 (8) | `g99v2fullw3`, in-job eval on `-p gpu`; `C:rlpd_3170398/402/406/410/414/418/422/426.out` | yes (all 8 headlines present 09-03; NOT yet in RESULTS) |
| RLPD × dHpruned (dHv2) / dDPpruned (dDPv2p) | — | — | — | — | **NOT RUN for RLPD** (no `sbatch_rlpd.sh` registry rows; A31: RLPD human arm = dHv2raw `PREREG:514`) |

## 2. Numbers (corrected world; per seed `hold/rnd` = SELECTED ckpt, then `LAST` = final ckpt_100/100k; counts x/15 or x/66 and x/30)
### 2.1 DP, frozen block, `eval_ics.json` — source `C:baselines/outputs/dp_w2final/<arm>_DP_s<seed>/sweep/HEADLINE.txt` (also `RESULTS:14-17`)
| arm | seed: selected hold / rnd | LAST hold / rnd | means (selected) | means (LAST) |
|---|---|---|---|---|
| dH | s20 13/15 18/30 (sel 80k) · s21 13 19 (80k) · s22 14 18 (40k) · s23 14 14 · s24 14 13 · s25 13 15 · s26 14 16 (40k) · s27 13 16 · s28 11 18 (60k) · s29 14 17 (60k) | 14 18 · 13 18 · 14 18 · 14 14 · 14 13 · 13 15 · 14 12 · 13 16 · 14 17 · 14 15 | hold 133/150 = **0.887**, rnd 164/300 = **0.547** (n=10; = `RESULTS:16`) | hold 137/150 = 0.913, rnd 156/300 = **0.520** |
| dDP | s20 13 15 (80k) · s21 14 14 · s22 12 14 (80k) · s23 14 15 · s24 11 16 (60k) · s25 13 15 · s26 13 11 · s27 14 15 · s28 14 16 · s29 13 15 (80k) | 14 17 · 14 14 · 14 13 · 14 15 · 14 12 · 13 15 · 13 11 · 14 15 · 14 16 · 14 12 | hold 131/150 = **0.873**, rnd 146/300 = **0.487** (= `RESULTS:17`) | hold 138/150 = 0.920, rnd 140/300 = **0.467** |
Stat (selected rnd, matched N=56 subset per `RESULTS:19-20`): dH−dDP +0.06, Welch CI [+0.006, +0.114], exact-perm p 0.041 (one unadjusted test; Holm → 0.082 per `CONFOUNDS.md:36`). LAST-ckpt contrast: NOT FOUND in any doc. dH s20 ran on pax007 (later excluded node, `CONFOUNDS.md:40`).

### 2.2 DP, v2 pool, `eval_ics_v2_w3.json` (hold = 66 training ICs) — source `C:baselines/outputs/dp_v2fullPw3/…/HEADLINE.txt`, `dp_v2fullw3/…`
| arm | seed: selected hold/66 · rnd/30 | LAST hold · rnd | means (selected) | means (LAST) |
|---|---|---|---|---|
| dHpruned (dHv2) | s50 65 17 (80k) · s51 65 13 · s52 62 16 (40k) · s53 65 19 · s54 64 13 (80k) · s55 65 15 · s56 64 13 · s57 64 16 | 62 15 · 65 13 · 62 15 · 65 19 · 60 15 · 65 15 · 64 13 · 64 16 | hold 514/528 = 0.973, rnd 122/240 = **0.508** (= `RESULTS:44`) | hold 507/528 = 0.960, rnd 121/240 = **0.504** |
| dDPpruned (dDPv2p) | s50 58 15 (60k) · s51 66 14 (80k) · s52 65 16 · s53 64 16 (60k) · s54 64 14 (20k) · s55 62 13 (20k) · s56 65 15 (40k) · s57 64 15 (80k) | 62 17 · 60 15 · 65 16 · 61 17 · 59 13 · 64 16 · 64 16 · 63 16 | hold 508/528 = 0.962, rnd 118/240 = **0.492** (n=8; `RESULTS:45` still says 0.489 at n=6, s56/57 were pending) | hold 498/528 = 0.943, rnd 126/240 = **0.525** |
| dHv2raw (raw human, disclosed) | s50 55 9 (40k) · s51 56 9 (80k) · s52 51 7 · s53 59 5 (40k) · s54 57 10 (60k) · s55 53 5 · s56 50 5 · s57 55 7 | 50 6 · 52 7 · 51 7 · 55 6 · 47 6 · 53 5 · 50 5 · 55 7 | hold 436/528 = 0.826, rnd 57/240 = **0.237** (= `RESULTS:34`) | hold 413/528 = 0.782, rnd 49/240 = 0.204 |
Stats: dHpruned − dDPpruned Δ +0.02, no p stated (`RESULTS:44-46`); pruned − raw +0.27 [+0.19, +0.35] perm p < 0.001 (`RESULTS:33-34`, A29).

### 2.3 RLPD sparse, frozen block (A20), `eval_ics.json` — source `C:rlpd_3085499-3085514.out` SWEEP-HEADLINE (also `RESULTS:142-149`)
| arm | seed: selected hold/15 · rnd/30 (ckpt) | LAST hold · rnd | means selected | means LAST (A20 statistic) |
|---|---|---|---|---|
| dH | s40 15 19 · s41 8 13 (ckpt_040) · s42 15 19 · s43 14 19 · s44 9 8 (ckpt_040) · s45 15 **18/29 (exp 30, INCOMPLETE cell)** (ckpt_080) · s46 15 20 · s47 15 19 | 15 19 · 0 1 · 15 19 · 14 19 · 0 1 · 14 21 · 15 20 · 15 19 | rnd 0.565 as mean of per-seed rates incl. 18/29 (`RESULTS:146`); raw counts 135/239 | hold 88/120 = 0.733; rnd 119/240 = **0.496** (= `RESULTS:145,148`) |
| dDP | s40 12 18 · s41 15 19 · s42 15 17 · s43 15 19 · s44 15 20 · s45 1 1 (ckpt_080) · s46 15 20 · s47 9 11 | 12 18 · 15 19 · 15 17 · 15 19 · 15 20 · 0 0 · 15 20 · 9 11 | rnd 125/240 = **0.521** (`RESULTS:146`) | hold 96/120 = 0.800; rnd 124/240 = **0.517** (`RESULTS:145,149`) |
Stat (`RESULTS:143-147`): LAST rnd dH−dDP −0.021, CI [−0.301, +0.259], Welch p 0.875, exact perm p 0.983; selected rnd +0.044, p 0.640/0.665; divergence 3/8 v 3/8 Fisher 1.0. A20 predictions FAILED.

### 2.4 RLPD sparse, v2 raw pair, `eval_ics_v2_w3.json` (hold = 66 training ICs)
| arm | seed: selected hold/66 · rnd/30 (ckpt) | LAST hold · rnd | means selected | means LAST |
|---|---|---|---|---|
| dHv2raw s60–67 (`C:rlpdsel_*.out`; `C:…/rlpd_g99v2fullw3_dHv2raw_s6?/sweep/HEADLINE.txt`; `RESULTS:178-180`) | s60 66 23 · s61 66 20 · s62 66 18 · s63 65 20 · s64 66 18 · s65 66 19 (ckpt_080) · s66 66 24 · s67 66 19 | 66 23 · 66 20 · 66 18 · 65 20 · 66 18 · **1 2** · 66 24 · 66 19 | hold 527/528; rnd 161/240 = **0.671** (= `RESULTS:180`) | hold 462/528 = 0.875; rnd 144/240 = **0.600** (7/8 alive) |
| dDPv2 s50–57 (`C:rlpd_3170398/402/406/410/414/418/422/426.out`; NOT in RESULTS yet) | s50 66 19 (ckpt_080) · s51 66 20 · s52 66 19 · s53 66 20 · s54 65 19 · s55 65 16 (ckpt_080) · s56 66 20 · s57 66 20 | 62 18 · 66 20 · 66 19 · 66 20 · 65 19 · **0 0** · 66 20 · 66 20 | hold 526/528; rnd 153/240 = **0.638** | hold 457/528 = 0.866; rnd 136/240 = **0.567** (7/8 alive) |
Stat: NOT FOUND (no document compares dHv2raw vs dDPv2 for RLPD; A21 registered |diff| < 0.10 prediction for the corrected world, `PREREG:396-397`; row-count/horizon asymmetries `CONFOUNDS.md:27,34`).

### 2.5 RLPD DENSE, frozen sets (A33), `eval_ics.json` — `C:rlpd_3163620-3163636.out`; `RESULTS:164-171`
dH selected hold/rnd: s60 5/4 · s61 2/4 · s62 4/4 · s63 0/1 · s64 0/1 · s65 9/8 · s66 1/5 · s67 2/2 → rnd 29/240 = 0.12; LAST rnd 4,2,4,1,1,0,0,1 = 13/240 = 0.05.
dDP selected: s60 0/0 · s61 0/1 · s62 0/0 · s63 7/11 · s64 5/7 · s65 3/1 · s66 5/1 · s67 0/0 → rnd 21/240 = 0.09; LAST rnd 0,1,0,2,1,0,1,0 = 5/240 = 0.02. Δ +0.03, no p stated (`RESULTS:171`).

### 2.6 OLD WORLD (matched_v2, `base`) — sensitivity only per A34 (`PREREG:550-555`); not extracted per seed here
RLPD sparse s30–37: LAST rnd dH 0.700 (n=7) v dDP 0.495 (n=7), +0.205, perm p 0.002 (`RESULTS:92-103`). DP s10–19 (registry, n=10/arm; `RESULTS:20` "n=5 replicate: no spread" — seed count conflict, NOT RESOLVED).

## 3. MISMATCHES vs the WM arm (one line each; "match" rows kept where the check was made)
1. **Action parametrization at training**: DP trains on ABSOLUTE window-end joint targets and is only converted to the delta MDP at eval (hold-4 integrator, `wandb_eval.py:298-320`); RLPD and WM train natively on delta_joint. (RLPD ↔ WM match: cap 0.025, leash 0.125, repeat 4.)
2. **Budget unit**: DP 100k grad steps (batch 64, so raw-pool arms see each row ~half as often, `CONFOUNDS.md:42`); RLPD 100k decisions (400k sim steps, UTD 10); WM "1M env steps" — whether sim steps or decisions is NOT FOUND, so RLPD:WM env-interaction ratio is unresolved.
3. **Eval action selection**: RLPD deterministic; DP sampled (seed k) with NO deterministic re-score; WM reports sample AND mode. A DP "mode" cell does not exist.
4. **Checkpoint rule of record**: DP headline = selected-of-5 (LAST also recorded, §2.1–2.2); RLPD statistic = LAST (selected also recorded); WM = end-of-run (LAST). LAST numbers therefore exist for all three; DP's registered headline is not LAST.
5. **hold IC file differs for v2 arms**: DP v2 and RLPD raw-pair rows use `eval_ics_v2_w3.json` (hold = 66 training ICs, in-distribution) while WM uses `eval_ics.json` hold-15 (+ alldemo74); rnd-30 is byte-identical across files (`eval_ics_v2_w3.json` rnd_note; `PREREG:568-571`). Only rnd is cross-learner comparable for v2 rows.
6. **Machine comparator for the raw-human comparison**: WM stage 2b pairs dHv2raw (66) against the FROZEN dDP (58, `WM_FIX_LOG:985-986`); RLPD pairs dHv2raw (66) against dDPv2 (66, per-IC matched). Different machine sets, different N and IC support.
7. **Seeds**: DP n=10 (frozen) / 8 (v2); RLPD n=8; WM n=2 per arm at the time of this audit (n=8 expansion running, `WM_FIX_LOG:987-989`).
8. **Eval process granularity**: DP/RLPD one fresh process per EPISODE (`eval_sweep.sh:110-131`); WM `eval_genesis.py` one process per IC set with in-process resets (`CONFOUNDS.md:48` row 40 — which also mis-describes DP/RLPD as "one process per block"; the script is per-episode).
9. **Eval partition / node**: RLPD dHv2raw re-scored on `-p batch` CPU nodes, dDPv2 in-job on `-p gpu` (`CONFOUNDS.md:40` row 32); DP in-job with GPU policy; WM node class NOT FOUND.
10. **Reward**: RLPD sparse +1, γ 0.99 (sidecar) ↔ WM sparse +1 reward_scale 1 — match; WM discount/λ NOT FOUND in the sections read; DP has no reward. RLPD dense arm is dead (§2.5) so no dense pairing is possible.
11. **Demo rows fed to the learner**: RLPD samples demo transitions 50/50 per batch from an immutable buffer (`sbatch_rlpd.sh:199`, `METHODS_draft:322`); WM buffer 5e5 with demos never evicted, no duplication (`WM_FIX_LOG:811`); DP epochs over rows — three different exposure regimes (`CONFOUNDS.md:44` row 36).
12. **Observation**: MATCH on content (same 17 numbers; DP splits them 8+9) — but only for the stage-2 WM recipe; every frozen-block WM number is pixels-only (`CONFOUNDS.md:52`).
13. **Horizon**: RLPD train/eval 1200 = WM time_limit/eval 1200 — match (the frozen WM block trained at 400, `CONFOUNDS.md:26`).
14. **Predicate**: all three use `picked` (hardened lift) — match (`genesis_can_env.py:258-265`; `WM_FIX_LOG:882`).
15. **Incomplete cell**: RLPD dH s45 selected rnd is 18/29 (one episode missing); its mean was taken as a rate (`CONFOUNDS.md:30` row 22).
16. **Registry blind spot**: DP registry rows carry no `sim_variant` and the corrected-world DP wave is named `w2final`; world identity for DP rests on `dp_sidecar.json` + dataset path only.
17. **Stale results doc**: `RESULTS:45` (dDPv2p n=6, 0.489) and `RESULTS:180-181` (dDPv2 "training") are superseded by the n=8 HEADLINEs in §2.2/§2.4.

## 4. Sources consulted
Local (`L:`): `paper/RESULTS_for_writing_2026-08-30.md` (§1, §2, §2.3–2.5, §3.1 line 226); `paper/CONFOUNDS.md` (rows 1–4, 8, 12, 17–19, 22–23, 25–26, 31–36, 40, 42–44);
`PAPER_PLAN.md:97-160`; `paper/PREREG_final_round_robin_2026-08-23.md` (§0–2, §5–7, A1–A8, A16, A17, A20–A36); `paper/METHODS_draft_2026-08-28.md` (§3.1–3.4, §5.1, §5.2, §5.6, §6.1–6.5);
`paper/V2_BUILD_2026-09-01.md` (§4, §B, §D, addenda); `paper/WM_FIX_LOG_2026-09-03.md:805-1002`; `paper/WM_FIX_PLAN_2026-09-03.md` (§1, §3–7 grep);
`baselines/eval_ics.json`; `cluster/sbatch_dp.sh`; `cluster/sbatch_rlpd.sh`; `cluster/eval_sweep.sh`; `cluster/dp_select_confirm.sh`; `cluster/rlpd_select_confirm.sh`;
`baselines/wandb_eval.py`; `baselines/eval_core.py`; `baselines/dp_runner.py`; `baselines/convert_to_lerobot.py:40-92`; `baselines/genesis_can_env.py`; `baselines/rl/train_rlpd.py:47-262`; `baselines/rl/full_env.py:270-404`; `baselines/rl/pick_env.py:36-91`.
Cluster (`C:`, read-only): `paper/harvest_2026-09-03_0554.md` (RLPD headlines block lines 604-664); `paper/harvest_2026-09-02_1754.md` (grep only); `cluster/RUN_REGISTRY.jsonl` (583 rows);
`baselines/eval_ics{,_v2,_v2_w3}.json`; `baselines/matched_w3/{dH,dDP,dHv2,dDPv2,dHv2raw,dDPv2p}/manifest.json` + `lerobot/meta/info.json`; `cluster/a31_chain/{common.sh,submit_learners.sh}`;
`baselines/outputs/dp_{w2final,v2fullPw3,v2fullw3,density,pilotv2Pw3,pilotv2fullw3}/*/sweep/HEADLINE.txt`; `baselines/outputs/dp_w2final/dH_DP_s20/checkpoints/100000/dp_sidecar.json`;
`baselines/rl/checkpoints/rlpd_g99w3_dH_s40/ckpt_100/rlpd_ckpt.action_mode.json`; `rlpd_3085499-3085514.out`, `rlpd_3163620-3163636.out`, `rlpd_3170398…3170426.out`, `rlpdsel_3162545…3223232.out` (SWEEP-HEADLINE lines); `analysis/results_table.py` (header only).
Not found anywhere: a DP×dDPv2 or RLPD×dHv2/dDPv2p run; a deterministic (mode) DP re-score; a LAST-ckpt DP contrast; the WM budget unit and discount; an RLPD dHv2raw-vs-dDPv2 test.
