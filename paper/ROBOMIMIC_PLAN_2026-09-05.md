# ROBOMIMIC PH / MH / MG leg for RLPD and r2dreamer — plan (written 2026-09-05, before any install or run)

*Purpose: the "cheapest bar-raiser" of `POSITIONING_demo_source_null_2026-09-05.md` §3(a)/(b) — test the demonstration-source null
(RLPD, world model) on a public task with an INDEPENDENT machine generator, so reviewer objection (4) ("distillation ≠ machine
data") and (1) ("one tuned task") get an answer. Protocol mirrors `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §5 and the gate
style of `PHASE_PLAN_2026-09-04.md` §4. Marks: **V** = verified this session against the URL, the file, or a probe of the
downloaded hdf5; **UNVERIFIED** = not checked. Nothing here has been run; nothing was installed on the cluster.*

## 1. Datasets (robomimic v0.1 study data, re-hosted for robosuite v1.5.1)
*Amendment 2026-09-05: the h5py probe of the 1.1 GB `can/mg/low_dim_sparse_v15.hdf5` landed (sha256 `1348033e86dea8bb…`, 1,098,749,512 bytes); the MG bullet below and the §5 row-count disclosure now carry measured values (V); one §3 sentence that assumed a per-checkpoint key was corrected.*
| item | fact | source |
|---|---|---|
| which tasks ship which sources | PH: Lift, Can, Square, Transport, ToolHang. MH: Lift, Can, Square, Transport. **MG: Lift and Can ONLY** (Square/Transport/ToolHang have no MG) **V** | https://robomimic.github.io/docs/datasets/robomimic_v0.1.html |
| PH | 1 operator (RoboTurk), 200 successful trajectories per task **V** | same page; paper https://arxiv.org/abs/2108.03298 (CoRL 2021) |
| MH | 6 operators × 50 = 300 successful trajectories; 2 "worse", 2 "okay", 2 "better" **V** | same |
| MG | SAC (RLkit) trained 2.4M (Lift) / 7.2M (Can) env steps with 150-step episodes; a checkpoint every 600k steps = 5 (Lift) / 13 (Can); per checkpoint 300 rollouts of FIXED length 150; "most checkpoints achieved 0 % average task success rate but the last few checkpoints reached ∼70−80 %"; Can = 585k transitions (3,900 rollouts), sparse reward + done at success (a `_dense` twin exists) **V** | https://ar5iv.labs.arxiv.org/html/2108.03298 (appendix, verbatim) ; docs page above |
| Can-Paired | one operator, 100 initial states × (one good + one bad trajectory) = 200 demos **V**; probe (V): 19,795 rows, length 55/97/147, **exactly 100/200 tapes have reward > 0 — the "bad" half are FAILED attempts** by the same operator at the same starts (no good/bad mask key; success = `rewards.sum() > 0`) | ar5iv; h5py probe |
| hosting | HF repo `robomimic/robomimic_datasets`, paths `v1.5/<task>/<type>/low_dim_v15.hdf5`, MG `low_dim_sparse_v15.hdf5` / `low_dim_dense_v15.hdf5`; raw `demo_v15.hdf5`; **`image_v15.hdf5` is NOT hosted (HTTP 404)** — images must be regenerated from raw **V** | https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/__init__.py (registry + `HF_REPO_ID`); HEAD requests this session |
| file sizes (HEAD, bytes) | can/ph 46.9 M, can/mh 112.8 M, can/mg sparse 1,098.7 M, can/paired 41.7 M, lift/mg 326.2 M, square/ph 51.1 M, square/mh 123.3 M **V** | HF HEAD this session |

**Probe of the downloaded Can files (h5py, this session — V):**
- `env_args`: `PickPlaceCan`, robot Panda, `env_version 1.5.1`, `control_freq 20`, controller `OSC_POSE` (`control_delta true`, output caps ±0.05 m / ±0.5 rad per step), `reward_shaping false`, `ignore_done true`, `use_object_obs true`.
- obs keys/dims: `object` 14, `robot0_eef_pos` 3, `robot0_eef_quat` 4, `robot0_eef_quat_site` 4, `robot0_gripper_qpos` 2, `robot0_gripper_qvel` 2, `robot0_joint_pos` 7 (+cos/sin 7/7), `robot0_joint_vel` 7; `states` = 71-dim flattened MuJoCo state; `next_obs` group present; `actions` (T,7) in [−1,1], gripper column ∈ {−1,+1}.
  The study's low-dim input = `object + eef_pos + eef_quat + gripper_qpos` = **23 dims** (keys per `generate_paper_configs.py`, https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/scripts/generate_paper_configs.py).
- PH Can: 200 demos, **23,207 rows**, length min/med/max 82/115/151, 200 distinct can placements at t0 (x ∈ [−0.28, 0.03], y ∈ [−0.18, −0.05] in `object[0:3]`).
- MH Can: 300 demos, **62,756 rows**, length 98/178/1050 (14 tapes > 400, 10 > 500); per group: worse 30,439 rows (med 270, max 1050), okay 18,053 (med 176), better 14,264 (med 136); mask keys `worse/okay/better`, `worse_okay`, `okay_better`, `worse_better`, per-operator, `train/valid`, `20_percent`, `50_percent`.
- Reward/done convention in the files: `rewards` = 1.0 on EVERY row from the first success to the end (5–6 rows), `dones` = 1 on those same rows; tapes continue 4–5 rows past first success. So "+1 once, terminal" is a conversion choice we make, not the file's.
- MG Can (probe, **V**): 3,900 rollouts, every one exactly 150 rows, **585,000 rows** (`total` attr; matches the paper); `env_version 1.5.1`, 20 Hz, `reward_shaping false`; same obs keys as PH/MH. **Successful rollouts (reward > 0): 718 / 3,900 = 18.4 %.** Rows-to-success (rows up to and including the first success row) over the 718: min 45 / p10 59 / median 76 / mean 82.5 / p90 114 / max 150 (sum 59,222 rows); rollouts continue after success (median 74 more rows, max 105) with reward = 1 and done = 1 on every later row. **The file has NO `mask` group and NO per-checkpoint key** — checkpoint membership is not recorded. Counting successes per consecutive block of 300 rollouts in index order gives 0, 0, 0, 0, 0, 0, 3, 24, 31, 130, 132, 195, 203 — consistent with the paper's "most checkpoints 0 %, last few ∼70 %" (65–68 % in the last two blocks), but the index-order → checkpoint mapping is an inference, not a file fact. MG gripper actions are continuous (values between −1 and +1), unlike the human ±1 toggles.

**Reference numbers (robomimic Table 1, low-dim, MAX success over checkpoints, 3 seeds, 50 rollouts, `offline_study` physics — V via ar5iv, two extraction passes agree on the PH/MG rows):**
Can PH: BC 95.3 / BC-RNN 100.0 / BCQ 88.7 / CQL 38.0 / HBC 100.0 / IRIS 100.0. Can MH: 86.0 / 100.0 / 62.7 / 22.0 / 91.3 / 92.7. Can MG: 64.7 / 68.7 / 75.3 / 1.3 / 40.7 / 48.0.
Lift MG: 65.3 / 70.7 / 91.3 / 64.0 / 47.3 / 96.0. Quote: "BCQ in particular performs strongly on our agent-generated MG datasets … neither BCQ nor CQL performs particularly well on these human-generated datasets."
Read-out for our framing: on Can, BC's big loss is on **MG** (−27 to −31 pts), MH costs plain BC −9 and BC-RNN nothing. "BC degrades on MH" is a Square/Transport statement, not a Can one.

**Target choice: Can PH / MH / MG.** Only Lift and Can have MG (V). Lift is at ceiling for every learner on PH/MH (100 %) and BCQ 91 % on MG — no headroom to see a source effect. Square has no MG (V). Can is the one task with all three sources and a non-trivial success rate. Lift is a possible cheap secondary (326 MB MG) only if Can passes its gates.

## 2. Environment for the online learners (RLPD, r2dreamer)
- Stack: robosuite + DeepMind `mujoco` bindings. The hosted datasets carry `env_version 1.5.1` (V probe); robosuite 1.5.1 requires `mujoco>=3.2.3` and `mink` (V https://github.com/ARISE-Initiative/robosuite/blob/v1.5.1/setup.py); 1.4.1 required `mujoco>=2.3.0` (V .../v1.4.1/setup.py). History: CoRL-2021 data = mujoco-py `offline_study` branch (robomimic v0.2); v1.4.1 data = robomimic v0.3 (2022-07); v1.5 data + HF hosting = robomimic v0.4 (2023-03) (V https://github.com/ARISE-Initiative/robomimic/releases). PyPI has robosuite up to 1.5.2 (2025-12-24) but **robomimic only to 0.3.0** — robomimic v0.5 must be installed from GitHub source (V `pip index versions` on the cluster). Pin of record: **robosuite==1.5.1, robomimic @ github v0.5.0 tag (UNVERIFIED that the tag installs cleanly), mujoco 3.3.7 (already on both cluster envs, V)**.
- State env = `robomimic.utils.env_utils.create_env_from_metadata(env_meta)` → `EnvRobosuite`, which forces `ignore_done=True`, `use_object_obs=True`, exposes `is_success()["task"]` (= robosuite `_check_success`: can inside its target bin) and `reset_to({"states": ...})` (V https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/envs/env_robosuite.py). Same controller config as the data ⇒ demo actions are valid env actions. Sparse reward: 1.0 per object in its bin, `reward_shaping=False` (V https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/environments/manipulation/pick_place.py). Because `ignore_done=True` and robosuite's own `horizon` is 1000 (V base.py), OUR wrapper owns termination: +1 and terminate on the first success, truncate at 400 decisions.
- Clock: native 20 Hz control (V), **action_repeat 1**, one decision per env step; robomimic eval horizon 400 (PH/MG) and 500 (MH) per the registry (V). We fix ONE horizon = **400 decisions** for training and eval of all arms (disclose the 14 MH tapes > 400 rows; they remain training data, nothing is truncated).
- Initial conditions: robosuite's `UniformRandomSampler` draws from the GLOBAL `np.random` (V https://github.com/ARISE-Initiative/robosuite/blob/v1.5.1/robosuite/utils/placement_samplers.py); `MujocoEnv(seed=)` seeds `self.rng` but `reset()` does not route placement through it (V base.py). So the matched-IC analogue of our `rnd30` is a **stored bank of 50 initial MuJoCo states** (`bank_can50.npz`, generated once by `np.random.seed(1000+k); env.reset(); state = env.get_state()["states"]`) replayed through `reset_to` for every learner, seed and checkpoint (V: `EnvRobosuite.get_state()` returns `{"model", "states"}`; `reset_to` calls `sim.set_state_from_flattened` + `sim.forward()`; `EnvRobosuite.is_done()` is always False — "robosuite envs always rollout to fixed horizon" — so the wrapper owns termination). sha256 of the bank goes in every metrics.json.
- Rendering only for the standing eval-video directive: `MUJOCO_GL=egl`, robosuite `renderer="mujoco"` for v1.5 (V env_robosuite); EGL on cluster nodes UNVERIFIED.

## 3. Data conversion (hdf5 → our contracts; no re-execution, no re-encoding)
- One row per decision at 20 Hz (the file's rows ARE decisions). Cut every tape at k = first row with `rewards > 0`; rows 0..k, reward +1 exactly once at k, `terminated` True at k; `next_obs` from the file's `next_obs` group. Tapes with no success (only in MG) keep reward 0, `terminated` False at the cap (value bootstraps) — same rule as `to_dreamer_native.py` for fails.
- State vector = the 23-dim study obs in fixed key order; actions verbatim (7-dim, already in [−1,1]). Provenance: sha256 of the hdf5 + filter key + cut rule in a `manifest.json`.
- RLPD: new loader `robomimic_demo_transitions(hdf5, filter_key, cut)` → the `(obs, a, r, next_obs, done)` tuples `DemoData` already consumes (V `baselines/rl/rlpd_sac.py:153`). Not the contract-v1 path (`native_demo_transitions` asserts Genesis stamps: sim_variant, delta_cap, leash — V `train_sacfd_full.py:494`).
- r2dreamer: writer `baselines/rl/robomimic_to_dreamer.py` emitting the `to_dreamer_native.py` layout: `image` zeros (T,64,64,6) u8 (loader asserts that shape — V `demo_prefill.py:83`), `state` (T,23), `action` backward-shifted with `action[0]=0` (asserted, V :89), `reward`, `is_first/is_last/is_terminal`, `discount`, `logprob`, plus `repeat.json {action_repeat 1, terminal_reward 1, src_sha}` (both asserted, V :183/:191). **One loader edit is unavoidable: `demo_prefill.py:82` hard-codes `state.shape == (T, 17)`** → parametrise by config.
- Arms ("raw vs pruned" translated): robomimic ships no failed HUMAN tapes and PH is already success-only, single operator (≈ our pruned human without idle removal). Matched-N primary arms, **200 tapes each**: `PH200` (all), `MH200` (fixed subsample: 33–34 per operator, seed 0, listed in the manifest), `MG200s` (200 of the 718 successful rollouts, drawn uniformly at random with seed 0; there is NO per-checkpoint key in the file (V), so no checkpoint stratification is possible — the manifest lists the chosen demo indices and their 300-block histogram; by index order the successes sit in the last ~7 blocks, so MG200s is de facto late-checkpoint data). Secondary all-data arms: `MH300`; `MGall` (3,900 incl. failures, the set as published); and **`PH200+Pbad100`** = PH200 plus the 100 FAILED Can-Paired tapes — the closest public analogue of `dHv2all` (same-operator human failures). Optional quality ladder if time permits: `MH-worse/okay/better` (100 each) and Can-Paired good/bad.
- Matched ICs across arms are impossible (every source has its own random placements: PH 200 distinct, MH 300 distinct — V). Substitute: the shared 50-state eval bank (§2) plus a registered covariate — can-xy at t0 per source (2-D histogram overlap), the analogue of the EEF-coverage figure.
- Descriptives per arm, same script as `DEMO_CHARACTERIZATION_dDP_vs_dHv2raw_2026-09-04.md`: rows, tape length, idle fraction (|a[:6]| < 0.05 while gripper unchanged), mean |a|, rows-to-success; MG rollouts run 150 steps flat, so MG tapes will be the shortest after the cut.

## 4. Learners, adapters, budgets, compute
- **RLPD**: `baselines/rl/train_rlpd_robosuite.py` (new, thin) = env `RobosuiteCanEnv(gym.Env)` (obs Box(23), act Box(7), success-terminate, 400-cap) + `make_rlpd` unchanged (SAC, UTD 10, E10/Z2, LN critics, γ 0.99, 50/50 demo batches) + the new loader; `cluster/sbatch_rlpd_robosuite.sh` (the Genesis launcher's ARM allow-list / contract-v1 / sim_variant gates do not apply — V `sbatch_rlpd.sh`). Budget of record **100k decisions** (= our recipe; 250 episodes of ≤400 steps), one registered extension to 300k if gate G3 fails. Wall time on the cluster for 100k decisions in Genesis: 9,014–12,079 s = 2.5–3.4 h (V `rlpd_3258287–91.out`); MuJoCo steps are cheaper than Genesis, gradient work dominates → **~3 h/run** (UNVERIFIED until the pilot).
- **r2dreamer**: `envs/robosuite.py` returning `{"state": f32[23], "image": zeros u8[64,64,3], is_first/is_last/is_terminal}` + a `suite == "robosuite"` branch in `envs/__init__.py` (V structure) + `configs/env/robosuite_can_state.yaml` = `genesis_pick_state.yaml` with `action_repeat 1`, `time_limit 400` (the `TimeLimit` wrapper counts decisions — V `wrappers.py:8`), `demo_downsample 1`, and the fixed recipe (`bounded_normal`, `return_clamp 1.0`, `act_entropy 3e-5`, `reward_scale 1`, mlp encoder/decoder on `state`); `eval_robosuite.py` replacing `eval_genesis.py` (bank + `--mode mode|sample`). Budget of record **500k decisions** (1,250 episodes; our pick runs used 250k decisions), one registered extension to 1M. Wall time: Genesis 1M sim steps/250k decisions = 2.9–3.3 h (V RESULTS §4.1); at 500k decisions with `train_ratio 512` expect **~4–6 h/run** (UNVERIFIED).
- **BC reference row (cheap, needed so the BC contrast is measured under OUR protocol, not only cited)**: robomimic's own BC-RNN low-dim config (`seq_length 10`, hidden 400 — V generate_paper_configs.py) on PH200 / MH200 / MG200s, 3 seeds, LAST checkpoint, same bank. ~1 GPU-h each.
- Compute: 3 sources × 2 learners × 8 seeds = 48 runs ≈ 24×3 h + 24×5 h ≈ **190 GPU-h**; pilots (6 runs), no-demo controls (8+8), BC row (9), all-data arms (2×2×8 = 32 runs ≈ 130 GPU-h, secondary) → **≈ 220 GPU-h primary, ≈ 350 GPU-h with secondaries**. The cluster gpu partition has ~120 GPUs across a100/h200/l40s/b200 nodes (V `sinfo`) but 49 of our jobs were queued at the time of writing (V `squeue`); $LAB has 318 GB free (V) — the Can files need ~1.3 GB.
- Environments: never pip into `condaenv/genesis` or `r2d_venv` (both lack robosuite/robomimic/h5py — V). Build two fresh venvs on the login node (PyPI and HF reachable there — V): `$LAB/robo_rlpd_venv` (py3.10, torch 2.7.0+cu126, sb3 2.8.0, gymnasium, robosuite 1.5.1, robomimic v0.5, h5py) and `$LAB/r2d_venv_robo` (overlay on the r2d_venv recipe as the WM-fix session did for DMC). Lab prior art exists — `$LAB/fastrl/envs/robotenv.py` + `robomimic_data.ipynb` (2025, mujoco 3.2.5) and DEMO3's `robosuite.yaml` (mujoco 3.1.5) — with different pins; read, do not reuse blindly.

## 5. Registration (to be frozen in this file BEFORE the first full-seed submission; pilots are exempt but reported)
- **Statistic of record**: success on the 50-state bank, LAST checkpoint, deterministic actions (`mode`) for RLPD and r2dreamer (sampled as secondary), per-seed counts, exact two-sided permutation test on seed means, **n = 8 v 8**, contrasts in this order: PH200 v MG200s (primary, the source axis), PH200 v MH200 (quality axis), MH200 v MG200s. Add a registered TOST equivalence test with margin **±0.10** (the positioning note's ask). Secondary: MH300 v PH200; MGall v MG200s and PH200+Pbad100 v PH200 (the failed-tape axis, machine and human).
- **Predictions**: RLPD and r2dreamer |Δ| < 0.10 on all three primary contrasts, and TOST passes at n = 8 if per-seed sd ≤ 0.08 (as in our Genesis WM arm; the RLPD arm's sd was larger and may not pass — say so). A directional secondary prediction from robomimic's BCQ result: if anything, **MG ≥ PH** for the value-based learners. BC-RNN (our row): MG200s below PH200 by ≥ 0.15; MH200 within 0.10 of PH200 on Can.
- **Gates**: G0 install/compatibility — replay 5 PH tapes' actions from `reset_to(states[0])` in the installed env: final can position within 1 cm of the file's `next_obs` and the same success flag on ≥ 4/5 (physics/version match; failure = version hell, stop). G1 positive control — BC-RNN on PH200 ≥ 0.90 (paper: 100 on old physics; v1.5 numbers are not re-benchmarked by robomimic — V model-zoo page — hence 0.90). G2 negative controls before any readout — random policy on the bank ≤ 2/50; RLPD and r2dreamer WITHOUT demos at the budget of record ≤ 0.10 (the paper's SAC needed 7.2M env steps on Can to reach 70–80 % — V — so ≈0 at 100k–500k decisions is expected). G3 learnability (PHASE_PLAN §4 rule) — every arm ≥ 0.5 on the bank in ≥ 3/8 seeds at LAST; otherwise the single registered budget extension; otherwise "not learnable at this budget".
- **Falsifiers of the paper's framing**: (a) PH200 ≫ MG200s for RLPD or r2dreamer (Δ ≥ 0.15, p < 0.05) — source-indifference fails on an independent generator; the F1 coverage story then has to carry it via the t0-placement covariate. (b) MG200s ≫ PH200 for both — robomimic's offline finding extends online, i.e. not a null either. (c) MGall ≫ MG200s or PH200+Pbad100 ≫ PH200 — failures help here but did not in Genesis (`dHv2all`), which undercuts the "online learners make their own failures" mechanism. (d) G3 fails everywhere — no claim.
- Disclosures by construction: PH is one operator; MG is 150-step fixed rollouts with a different start distribution; tape counts matched but rows not (after the first-success cut: PH200 ≈ 22.2k rows, MH200 ≈ 41k (V-derived from the per-group row counts), **MG200s ≈ 200 × 82.5 = 16.5k rows** (V: mean rows-to-success 82.5 over the 718 successes), MGall = 585k rows of which 718 rollouts carry a +1 (V)); state input (23-dim privileged object pose), no images; our LAST-checkpoint protocol vs robomimic's max-over-checkpoints; physics v1.5.1 vs the paper's `offline_study`.

## 6. Timeline and top risks
Timeline (calendar days, cluster-queue permitting): D1 venvs + dataset download on the login node + G0; D2–3 converters, adapters, bank, smoke runs (2k steps), BC-RNN control queued, THIS FILE frozen; D4 pilots (1 seed × 3 sources × 2 learners) → G2/G3 readout and wall-time measurement; D5–7 the 48 primary runs over two nights (+ secondaries if the queue allows); D8 statistics (`morning_table.py` port), videos, write-up. ≈ 8 working days; +50 % slack for the queue (49 jobs already ours).
Risks, in order: (1) **Install/version hell** — robosuite 1.5.1 on mujoco 3.3.7 with `mink`, robomimic from source, EGL for videos, py3.10 vs 3.11 across our two stacks; mitigated by fresh venvs, G0, and doing nothing in the shared envs. (2) **MG being "easy" or "different"** — SAC rollouts are Markov, smooth, and start from SAC's own reset distribution; a value learner may prefer them (robomimic's BCQ did); that is a finding, not a null, and the t0-placement covariate is registered to read it. (3) **MG exists for Can (and Lift) only** — one task again; Square/Transport cannot join; the MH groups and Can-Paired are the only extra quality axes. (4) **Image vs state** — image data are not hosted for v1.5 (404); we stay state-based, so the pixel-policy objection (positioning §3 (1)) is untouched. (5) **Compute/queue** — ~220–350 GPU-h behind our own queued waves; secondaries are the first to drop. (6) Eval nondeterminism across node classes — one node class per eval, fresh process per eval, as now.

## 7. Summary (10 lines)
1. Run RLPD and r2dreamer on robomimic **Can** with the public PH (200, one operator), MH (300, six operators, three skill groups) and MG (3,900 SAC-checkpoint rollouts) sets; Lift is at ceiling and Square has no MG (V).
2. MG is an independent generator (SAC/RLkit, 7.2M steps, checkpoints every 600k, 300 × 150-step rollouts each, failures included, most checkpoints at 0 %) — exactly the "not a distillation" machine data the positioning note asks for.
3. Files: HF `robomimic/robomimic_datasets` `v1.5/can/{ph,mh,mg}/low_dim*_v15.hdf5`, 47 MB / 113 MB / 1.1 GB; probed: 23-dim study state, 7-dim OSC_POSE actions in [−1,1], 20 Hz, `env_version 1.5.1`, success = can in target bin (V).
4. Online env = robomimic's `EnvRobosuite` from the file's `env_meta` (robosuite 1.5.1, mujoco ≥ 3.2.3; cluster already has mujoco 3.3.7 but no robosuite/robomimic/h5py — new venvs only).
5. Conversion = cut each tape at first success, +1 once, terminal; 23-dim state; `action_repeat 1`, `time_limit 400`; new loaders for both learners; one hard-coded `(T,17)` assert in the WM prefill must become configurable.
6. Arms: matched-N 200 tapes each (PH200 / MH200 / MG200s) primary; MH300 and MGall (with failures) secondary; shared 50-state eval bank via `reset_to` replaces matched ICs; t0-placement overlap is the registered covariate.
7. Statistic: success on 50 fixed states, LAST checkpoint, deterministic, exact permutation n = 8 v 8, plus TOST ±0.10; predictions |Δ| < 0.10 for RLPD and WM, BC-RNN drops ≥ 0.15 on MG (robomimic: BC 95→65, BC-RNN 100→69; MH costs Can BC-RNN nothing).
8. Gates: G0 replay-consistency of 5 PH tapes (version match), G1 BC-RNN ≥ 0.90 on PH, G2 random ≤ 2/50 and no-demo learners ≤ 0.10, G3 learnability ≥ 0.5 in ≥ 3/8 seeds; one registered budget extension each (RLPD 100k→300k decisions, WM 500k→1M).
9. Cost ≈ 220 GPU-h primary (48 runs at ~3 h RLPD / ~5 h WM, times UNVERIFIED for MuJoCo), ≈ 350 GPU-h with secondaries; ~8 working days plus queue slack.
10. Falsifiers are registered in both directions (PH ≫ MG breaks source-indifference; MG ≫ PH extends robomimic's offline result online); either is reportable, and "not learnable at this budget" is the third honest outcome.

## A1. Amendment 2026-09-06 (registered before any DP run): Diffusion Policy as the third learner
*Requested by the user 2026-09-06 ("our three algorithm families"). Written at preparation time, before any DP smoke
or seed on the robomimic data; the registered text above is unchanged.*
- **Learner**: lerobot Diffusion Policy, STATE-ONLY inputs — `observation.state` = eef_pos + eef_quat + gripper_qpos (9),
  `observation.environment_state` = object (14), action = the file's 7-dim OSC_POSE + gripper in [−1, 1] executed directly
  (the env's own action space; no integrator, no action repeat). Recipe of record = `cluster/sbatch_dp.sh`'s: 100k gradient
  steps, batch 64, lerobot 0.4.5 (fork `genesis-fixes`) defaults otherwise (horizon 16, n_obs_steps 2, n_action_steps 8);
  dataset fps 20 (one row per decision, identical cut rule to the other learners). Launcher `cluster/robomimic/sbatch_dp_robo.sh`,
  converter `baselines/robomimic/convert_arms.py --targets lerobot`, evaluator `baselines/robomimic/eval_dp_robosuite.py`.
- **Statistic**: success on the same 50-state bank, LAST checkpoint, SAMPLED actions (DP is stochastic; per-episode torch seed
  = bank index, as `cluster/eval_sweep.sh` does), n = 8 v 8, the same three contrasts and the same exact permutation + TOST
  (±0.10) as §5. DP has no "mode" cell; the sampled cell is its statistic of record.
- **Gate G3-DP** (learnability, same rule as §5 G3): every arm ≥ 0.5 on the bank in ≥ 3/8 seeds at LAST; one registered
  extension to 300k gradient steps; otherwise "not learnable at this budget". No no-demo control is meaningful for BC.
- **Prediction (from robomimic Table 1 BC/BC-RNN rows and our Genesis DP results, `RESULTS_WM_HUMAN_VS_MACHINE §2`)**:
  DP on MG200s BELOW PH200 by ≥ 0.15 (the offline learners lose on MG: BC 95→65, BC-RNN 100→69); MH200 within 0.10 of PH200
  (Can: MH costs BC −9, BC-RNN 0). I.e. the BC-family gradient the paper claims for BC should reappear for DP, while §5
  predicts |Δ| < 0.10 for RLPD and r2dreamer on the same three sets — that contrast (DP source-sensitive, online learners
  source-indifferent) is the pre-registered pattern; DP source-INdifference (|Δ| < 0.10 on PH v MG) would falsify it and
  would say the robomimic BC loss on MG is a BC-RNN/MLP artefact, not a demo-source property.
- **Disclosures**: DP trains only on success-cut rows (rows after the first success are dropped for every learner, so the
  DP sets are exactly the RLPD/WM demo sets); MG200s ≈ 16.5k rows vs PH200 ≈ 22.2k vs MH200 ≈ 41k (tape-matched, not
  row-matched); lerobot's default normalisation (min-max over the dataset) is learner-internal; our LAST-checkpoint /
  sampled-action protocol vs robomimic's max-over-checkpoints.
- **Environment note (fact, not a change)**: the runs use the NEW `$LAB/robo_venv` (RLPD, DP, BC-RNN) and
  `$LAB/r2d_venv_robo` (r2dreamer overlay on r2d_venv) — `cluster/robomimic/verify_env.sh` prints the pins.
- **Budget accounting for r2dreamer (clarification of §4)**: "500k decisions" = ONLINE decisions; the launcher sets
  `env.steps = prefill rows + 500k` because the demo prefill spends `env.steps` by construction (`demo_prefill.py`), and
  `buffer.max_size = 2·rows + 500k` so no demo row is evicted (the same "nothing evicted" choice as the Genesis WM arm).
- **Bank restore (clarification of §2)**: entries are restored with `reset_to({model: xml, states})` (robomimic's
  playback convention; the model xml is stored alongside the flattened state), not states-only.

## A2. Amendment 2026-09-07 (registered BEFORE any control run): RLPD source-effect controls — quantity, failures, budget
*Trigger (coordinator, 09-07): the RLPD arm of the primary matrix read out with falsifier (a) firing on the quality/source axis —
`eval_bank50_mode` LAST, n = 8 v 8: MH200 [27, 44, 30, 1, 20, 9, 24, 27] = 0.455 vs MG200s [10, 2, 2, 13, 6, 12, 9, 5] = 0.147,
Δ = +0.307, exact two-sided p = 0.008 (sample: 0.458 vs 0.168, p = 0.011). Before that is read as a demonstration-SOURCE
effect, three controls separate tape count / row count, failure rows and budget from source. Written before any of them
is built or submitted; the primary-matrix protocol (bank_can50, LAST checkpoint, mode primary / sample secondary,
exact permutation on per-seed counts, 8 seeds, launchers unchanged) applies verbatim.*
- **C1 — MGall** (the registered §3 secondary): all 3,900 SAC rollouts incl. 3,182 failures, 536,522 rows after the cut
  (718 rewarded terminals; reward density 0.13 % vs MG200s 1.2 %, MH200 0.49 %), RLPD 100k decisions, 8 seeds (`ARM=MGall`).
- **C2 — MG718s** (new arm, same converters/manifests): EVERY successful MG rollout, 718 tapes, ≈ 59k rows (> MH200's
  41,134 rows and 3.6× its tape count; MG200s ⊂ MG718s), RLPD 100k decisions, 8 seeds (`ARM=MG718s`). Row- and
  tape-superior to MH200, so a remaining deficit cannot be quantity.
- **C3 — MG200s @ 300k decisions** = the §4/§5 registered budget extension for the arm that failed G3 (MG200s LAST
  ≥ 0.5 in 0/8 seeds at 100k), 8 seeds (`STEPS=300000 TAG=ext300k`). Same launcher, K = 5 checkpoints archived.
- **Statistic**: MODE success on the bank at LAST per seed; exact two-sided permutation, n = 8 v 8, against the primary
  cells MG200s@100k (0.147) and MH200@100k (0.455); TOST ±0.10 where equivalence is claimed. Sample as secondary.
- **Predictions (falsifiable):** P1 (C2): MG718s > MG200s by ≥ 0.10 (more rewarded tapes help an off-policy learner) but
  MG718s < MH200 by ≥ 0.15 with p < 0.05 — the source effect survives row-matching. P2 (C1): |MGall − MG718s| < 0.10
  (failure rows neither rescue nor sink it — the Genesis `dHv2all` precedent), hence MGall < MH200 by ≥ 0.15. P3 (C3):
  MG200s@300k improves over MG200s@100k by ≥ 0.10 but stays below MH200@100k by ≥ 0.15; G3 at 300k (≥ 0.5 in ≥ 3/8 seeds)
  NOT met. **Decision rules:** if MG718s or MGall reaches MH200 − 0.10 → the effect was quantity/coverage, not source;
  if MG200s@300k reaches MH200@100k − 0.10 → it was budget (MG data are slower to exploit, not worse); if all three stay
  ≥ 0.15 below MH200 (p < 0.05) → falsifier (a) stands as a source effect on RLPD, to be read with the t0-placement
  covariate (`CAN_POS`) and the per-checkpoint-block composition recorded in the log.
- **Disclosures:** MG718s and MGall share every MG200s tape (nested arms, not independent samples); the SAC checkpoint
  block of each tape is inferred from index order (no per-checkpoint key in the file); C3 is a longer budget on the SAME
  demos; all three are RLPD-only (the r2d/dp arms of the matrix are still running and are not extended here).

### Amendment A3 — primary contrast changed to MH200 v MG200s (user, 2026-09-07 10:45; recorded 13:45)

User: "drop single-human for now since our own dataset is just mixed-human." The PH200 arms were removed from the submitted matrix (the 72-job mis-fire at 10:26 was cancelled before any job started; the 48-run matrix submitted at 10:50 is MH200 + MG200s × rlpd/r2d/dp × 8 seeds). The primary source contrast is therefore **MH200 v MG200s** for every learner; PH200 v MG200s and PH200 v MH200 remain registered secondaries to be run only on a later go. The BC-RNN control keeps all three (PH 0.92 / MH 0.927 / MG 0.393). Falsifier (a) is read on MH200 v MG200s.

## A4. Amendment 2026-09-07 (registered BEFORE any run; runs held until the coordinator confirms): action-process control for the RLPD MH200 v MG200s gap
*Trigger: adversarial review S1-1 — MG demo actions differ from human ones in three non-source ways that none of the A2
controls (MGall, MG718s, MG200s@300k, all drawn from the same SAC rollouts) removes. Recomputed on the arms' rlpd
transitions (`robomimic_data/arms/action_stats.json`, arm dims 0–5): mean |a| MG200s 0.664 vs MH200 0.144 (PH200 0.236);
mean |a_t − a_{t−1}| 0.274 vs 0.043 (0.050); gripper: MG 1,903 distinct values (29 % at |g| ≥ 0.99, 1 % exactly ±1)
vs binary ±1 on every human row; MG drives all six OSC dims incl. the three rotations (per-dim |a| 0.58–0.78 vs human
0.02–0.06). Any RLPD deficit on MG data is equally consistent with "bang-bang, high-frequency demonstrations" as with
"machine source".*
- **Design (re-execution pairs; the honest form — the actions in every tuple are the ones that produced s′):** each
  arm is built by replaying a modified action sequence open-loop in the installed env from `reset_to({model xml,
  states[0]})` of the ORIGINAL tape, recording the fresh observation each step (`get_observation()`), for the tape's own
  length T (no extra actions exist), keeping the tape iff `is_success()` fires within T, cutting at first success,
  +1 once, terminal. Each modified arm is paired with a **re-execution control** built by the same pipeline with the
  UNMODIFIED actions, and both arms are restricted to the tapes that succeed in BOTH re-executions (identical tape
  sets; removes survivorship and replay-drift asymmetry). Yields are reported before any training.
  - **R0 MG200s_re** (control): MG200s, original actions, re-executed. **R1 MG200s_sm** (treatment): arm dims passed
    through a causal EMA a′_t = β a′_{t−1} + (1−β) a_t (a′_0 = a_0) with ONE β for the whole arm, found by bisection so
    that the arm's mean |Δa′| equals MH200's 0.043; gripper binarised at 0 (g′ = +1 if g > 0 else −1, robosuite's
    close/open convention). The EMA also lowers |a| on the sign-flipping components (reported, not matched separately);
    a magnitude-restored variant (per-tape rescale of a′ to the tape's original mean |a|, then clip) is registered as
    **R1m** and built only if R1's yield < 100 tapes.
  - **R2 MH200_re** (control): MH200, original actions, re-executed. **R3 MH200_rough** (treatment): a′_t = clip(a_t +
    n_t, −1, 1), n_t i.i.d. uniform per dim with half-width ε set by bisection so that the arm's mean |Δa′| equals
    MG200s's 0.274 (seeded, fixed); gripper unchanged (binary — MG's continuous gripper has no human mirror). Resulting
    mean |a| reported (it rises but cannot be matched to 0.664 without saturating every row).
  - Optional, zero build cost (review S1-1 (b)): **Can-Paired good 100 v bad 100** (one operator, one action process,
    success vs failure) as a within-process quality contrast; RLPD 8 seeds each if the A4 pairs are run.
- **Learner / statistic:** RLPD recipe of record, 100k decisions, 8 seeds per arm, LAST on bank_can50, mode primary /
  sample secondary, exact two-sided permutation on per-seed counts, contrasts R1 v R0 and R3 v R2 (within-pair, same
  tapes), plus R1 v MH200@100k. TOST ±0.10 where equivalence is claimed.
- **Predictions (falsifiable):** P-A4-1: **R1 − R0 ≥ +0.15 (p < 0.05)** — smoothing + binarising the SAC actions
  recovers a large part of the gap, and R1 ≥ MH200 − 0.10 → the "source" effect is (mostly) the action process.
  P-A4-2: **R3 − R2 ≤ −0.15 (p < 0.05)** — roughened human actions hurt RLPD. P-A4-3: R0 ≈ MG200s and R2 ≈ MH200
  (|Δ| < 0.10; re-execution itself is neutral — the G0 result makes this likely for MH; for MG it is the yield that
  is uncertain). **Decision rules:** if P-A4-1 holds → the paper's sentence is "RLPD learns worse from bang-bang,
  high-frequency demonstrations; the SAC generator produces them" (mechanism named), not "source-sensitive";
  if |R1 − R0| < 0.10 AND |R3 − R2| < 0.10 → the action process is not the mechanism and the A2 decision rule stands;
  mixed outcomes (one pair moves, the other not) → report both, no source claim; if R1's yield < 50 tapes even with
  R1m → the control is "not buildable by open-loop replay" and only R3 v R2 is read.
- **Counts (upper bounds; the measured yields replace them in the log before training):** R0/R1 ≤ 200 tapes, ≤ 16,501
  rows (MG200s re-cut at the re-execution's first success); R2/R3 ≤ 200 tapes, ≤ 41,134 rows; Paired 100 + 100 tapes,
  ≈ 9.9k rows (19,795 rows in the file, bad half uncut). Cost ≈ 32 runs × ~1.5–2 GPU-h ≈ 48–64 GPU-h (R0–R3);
  Paired +16 runs. Build (CPU, login node, ~200 replays × ~3 s per arm) needs no GPU.
- **Disclosures:** re-executed tapes are new sim trajectories (the plan's "no re-execution" rule is suspended for this
  control only, by design); β/ε are arm-level constants; yields select tapes (identical sets within a pair, but the
  pair's set is a non-random subset of the arm); the EMA changes both smoothness and magnitude; the t=0 artefact
  (review S2-6) is ABSENT from re-executed arms (fresh observations) — a further difference from the file-based arms,
  disclosed. **Nothing in A4 is submitted until the coordinator confirms.**

### A4 addendum — 2026-09-07 14:30 (registered BEFORE any A4 training; measured yields decide the arms)
*A4 registered "the largest roughness that keeps ≥ 100 paired tapes" as the fallback when the registered target is not
open-loop executable. Measured (builder `baselines/robomimic/build_reexec_arms.py`, all yields out of the 200 MH200 /
200 MG200s source tapes, control = original actions re-executed, treatment = modified, kept iff BOTH succeed):*
- **MG pair — NOT BUILDABLE, as the registered rule anticipated.** R0 control 114/200 (57 %); R1 (EMA β = 0.8712 →
  |Δa| 0.0431, gripper binarised) **0/200**; R1m (magnitude restored, |a| 0.577, |Δa| 0.0522) **0/200**; β-ladder on 40
  tapes (control 20/40): β 0.30 → 3/40, β 0.50/0.70/0.86 → 0/40. Per A4's rule ("R1 yield < 50 even with R1m → not
  buildable by open-loop replay; only R3 v R2 is read"), **the MG treatment arm is abandoned**; P-A4-1 is NOT testable
  and is withdrawn, unrun. The 114/200 control yield is reported as a fact about MG tapes (they are open-loop
  reproducible about half the time; any modification of their actions destroys that).
- **MH pair — built at a reduced roughness (dose ladder), because the registered ε is not executable either.** At the
  registered ε = 0.4056 (|Δa| 0.2741 = MG's): control 185/200, treatment 5/200 → 5-tape arms, unusable (written as
  provenance only, never trained). Ladder (40 tapes, control 37/40): ε 0.05 → both 33, 0.10 → 29, 0.15 → 25, 0.20 → 19,
  0.30 → 4. Full builds: **ε = 0.15 → control 185/200, treatment 98/200, BOTH 95** (`MH200_re15` 95 tapes / 15,702 rows,
  |a| 0.159, |Δa| 0.0455; `MH200_rough15` 95 tapes / 15,681 rows, |a| 0.189, **|Δa| 0.1177 = 2.6× MH200's 0.0431, 43 % of
  MG200s's 0.2741**); **ε = 0.20 → BOTH 61** (`MH200_re20` 61 / 9,806, |Δa| 0.0439; `MH200_rough20` 61 / 9,781,
  **|Δa| 0.1459 = 53 % of MG's**). 95 < the registered 100 by five tapes — disclosed, and the reason both doses are run:
  the pair is now a **dose-response** design (ε 0.15 primary by yield, ε 0.20 secondary), not a single matched arm.
- **Registered statistic and predictions for what IS run** (32 RLPD runs = 4 arms × 8 seeds, 100k decisions, LAST on
  bank_can50, mode primary / sample secondary, exact two-sided permutation on per-seed counts, n = 8 v 8):
  **P-A4-2′ (replaces P-A4-2 at the executable dose): rough15 − re15 ≤ −0.10 (p < 0.05)**, and rough20 − re20 ≤ rough15 −
  re15 (monotone dose-response). **P-A4-3′: re15 and re20 are within 0.10 of MH200@100k** (re-execution + tape selection
  are neutral). **Decision rules:** if P-A4-2′ holds → roughness alone degrades RLPD on human demonstrations, so the
  MH200-v-MG200s gap cannot be attributed to source without also holding action statistics fixed — the paper's sentence
  names the action process as a live mechanism; if |rough − re| < 0.10 at BOTH doses → roughness up to 53 % of MG's is
  not sufficient to explain the gap (magnitude, gripper coding and source remain jointly confounded, stated as such);
  if re15/re20 differ from MH200 by ≥ 0.10 → the pair is read only within itself (the selection is not neutral).
- **Disclosures:** the treatment is a *partial* dose (43 % / 53 % of MG's roughness), so a null is bounded, not general;
  the kept tapes are the easier 95 (or 61) of 200 (both arms identical sets); |a| rises with ε (0.159 → 0.189) so
  roughness and magnitude move together; the gripper stays binary in both arms (MG's continuous gripper has no human
  mirror and remains untested); re-executed arms carry no t=0 observation artefact. **Can-Paired good-v-bad is NOT run**
  (it needs a build; the coordinator restricted it to "no build").

## A5. Amendment 2026-09-08 (registered BEFORE the runs): fixed-head control for the BC-RNN row
*Trigger: the adversarial-review question "does the GMM head interact with MG's action statistics?" — checked against the
configs that actually ran and the `config` blob inside each `model_epoch_2000.pth`. The answer is stronger than an
interaction: **the two arms of the BC-RNN row use different policy classes.** PH200 and MH200 ran with
`algo.gmm.enabled = True`, `num_modes 5`, `low_noise_eval True`; MG200s ran with `algo.gmm.enabled = False`
(gaussian False, vae False, l2 1.0 → a deterministic MSE head). Everything else is identical (LSTM 2×400,
`actor_layer_dims []`, lr 1e-4, seq_length 10, batch 100, 2000 epochs, same bank/protocol). Origin:
`baselines/robomimic/make_bcrnn_config.py:71` `config.algo.gmm.enabled = (dtype != "mg")`, copied deliberately from
robomimic's own `generate_paper_configs.py` (`if dataset_type == "mg": config.algo.gmm.enabled = False`) to reproduce
their published per-dataset recipe, and disclosed in that file's docstring. **The defect was not the choice but its
travel: a fact recorded in the code never reached the results, where it changed an interpretation** — the same shape as
several defects found on 09-07. Consequence: as published, the BC-RNN row (MH200 0.927 v MG200s 0.393) does not measure
demonstration source and cannot stand beside RLPD and DP as "three learners measuring one thing".*
- **Design (head held fixed across arms; nothing else changes):** **G1a `MG200s_gmm`** = MG200s with `gmm.enabled True`
  (5 modes) and **G1b `MH200_nogmm`** = MH200 with `gmm.enabled False`, 3 seeds each = 6 runs, robomimic's own trainer,
  2000 epochs, LAST checkpoint (`model_epoch_2000.pth`), scored by `eval_bcrnn_robosuite.py` on `bank_can50` (50 states,
  deterministic: `low_noise_eval True` for the GMM head, deterministic by construction for the MSE head).
  Reference cells already on record (same protocol): MH200-GMM 49/45/45 = **0.927**, MG200s-det 22/14/23 = **0.393**,
  PH200-GMM 46/46/46 = 0.92. Cost: measured 24–27 min/run incl. eval → **≈ 2.6 GPU-h**.
- **Statistic and its honest limit:** per-seed counts /50 and the 3-seed mean; an exact 3-v-3 permutation has 20 splits,
  so **p < 0.05 is unattainable by construction** — the decision is on EFFECT SIZE against a ±0.10 band, disclosed as
  such, and a borderline outcome (|Δ| in 0.08–0.12) is escalated to 8 seeds (+7 GPU-h) rather than argued.
- **Predictions (registered):** **P-A5-1: `MG200s_gmm` within 0.10 of 0.393** (the head is not the story on machine data
  — robomimic disabled it there because it did not help, not because it hurt). **P-A5-2: `MH200_nogmm` within 0.10 of
  0.927** (the head is not the story on human data either). Both are falsifiable in either direction, and the
  human-arm-switched-off cell is reported whatever it shows: a large move there would say the effect is architectural on
  both arms rather than only on one.
- **Decision rules:** (i) both predictions hold → the head is not the story; the BC-RNN row is re-reported head-matched
  (one head for both arms) and its remaining doubt is exactly the 200-tape-subsample doubt the A2 controls established.
  (ii) P-A5-1 fails (`MG200s_gmm` moves ≥ 0.10) → the published 0.393 was partly an architecture artefact; **the BC-RNN
  row as published is withdrawn** and may only be re-reported from head-matched cells. (iii) P-A5-2 fails → the head
  matters on human data too; same withdrawal, and head choice becomes a variable that must be held fixed in every future
  row of this leg. (iv) Any outcome where the two arms' best heads differ → the row is reported as two head-matched
  contrasts, never as one number.
- **Pre-committed consequences for the CROSS-LEARNER ORDERING** ("imitators lose most, online RL least": BC-RNN Δ 0.53,
  DP Δ 0.77, RLPD Δ 0.31 on MH200 v MG200s) — decided now, not after the numbers:
  1. **Under every outcome of A5, the ordering may NOT be quoted as a demonstration-SOURCE ordering.** The A2 controls
     already withdrew that reading for RLPD (MG718s 0.475 v MH200 0.455, Δ +0.020, p 0.886; MGall 0.610, above the human
     arm), and the DP and BC-RNN arms have never been run on MG718s/MGall — they inherit the same 200-tape-draw doubt.
  2. Under (i) the ordering survives only as **"three learners on one 200-tape MG draw versus one 200-tape MH draw"**,
     with the subsample caveat attached to every use, and it is not evidence about human-vs-machine demonstrations.
  3. Under (ii) or (iii) the ordering is **withdrawn outright** until BC-RNN is re-run head-matched; a row whose two arms
     differ in policy class cannot contribute a rank to an ordering of policy classes.
  4. The ordering is restored as a source claim only by running DP and BC-RNN on MG718s and MGall (≈ 16 DP runs ≈ 32 GPU-h
     + 6 BC-RNN runs ≈ 2.6 GPU-h) — registered here as the price, so it is not smuggled in by re-interpretation.

## A6. Amendment 2026-09-08 (registered BEFORE the runs): row-matched human arms — a two-source dose curve
*Trigger: the A2 controls withdrew the source reading (MG718s 0.475 v MH200 0.455, p 0.886; MGall 0.610) and left a
suggestive quantity relationship with only ONE uncontaminated human point. The user's original ask was "test like against
like"; the human side has never been row-matched to the machine side. This amendment converts an absence of evidence
("no detectable source effect") into a positive account, or refutes it.*
- **Arms:** **`MH80`** = a uniform seed-0 permutation of MH200's 200 tapes, taking the prefix whose cumulative row count
  is closest to MG200s's 16,501 → **80 tapes / 16,406 rows** (error 95 rows = 0.6 %), natively collected, NOT re-executed,
  built by the same `make_arms.py`/`convert_arms.py` path and cut rule as every other arm. **`MH300`** = the §3 secondary
  arm, already built (300 tapes / 61,548 rows), giving a human point beside MG718s's 59,222. RLPD, 100k decisions,
  8 seeds each = 16 runs ≈ 16 GPU-h; LAST on bank_can50, mode primary / sample secondary, exact two-sided permutation.
- **Resulting dose curve (rows → mode success):** human 16.4k / 41.1k / 61.5k vs machine 16.5k / 59.2k / 536.5k, with
  the machine side already measured at 0.147 / 0.475 / 0.610 and the human side at 0.455 for 41.1k.
- **Predictions:** **P-A6-1: `MH80` within 0.10 of MG200s's 0.147** (i.e. ∈ [0.05, 0.25]) — at matched rows the human
  arm is no better than the machine arm, and the MH200-v-MG200s gap is quantity, not source. **P-A6-2: `MH300` within
  0.10 of MG718s's 0.475 AND ≥ MH200's 0.455** (monotone in rows). **P-A6-3 (secondary, the re-execution isolation):
  `MH80` (16,406 rows, native) − `MH200_re15` (15,702 rows, re-executed) ≥ +0.10** → re-execution costs at least that
  much at matched scale; if |Δ| < 0.10 the `re15` collapse was mostly size and re-execution is comparatively cheap.
- **Decision rules:** (i) P-A6-1 and P-A6-2 both hold → the leg's central claim becomes affirmative: *RLPD performance on
  this task is governed by demonstration row count, and the human and machine dose curves coincide*; the MH200-v-MG200s
  ordering is fully explained by quantity. (ii) `MH80` exceeds MG200s by ≥ 0.15 → at matched rows human data IS better;
  a source effect survives at small scale, and A2's conclusion narrows to "large machine sets compensate", not "source
  is irrelevant". (iii) `MH80` ≤ 0.05 or `MH300` < MH200 (non-monotone) → the row-count account fails and the pattern is
  reported as unexplained. (iv) Any outcome: the tape mismatch (80 v 200 at matched rows) is disclosed — rows and tapes
  cannot both be matched, and rows are the axis the learner consumes.
- **Disclosures:** `MH80` ⊂ `MH200` ⊂ `MH300` (nested, not independent draws); the seed-0 prefix stopping rule slightly
  disfavours the longest tapes at the boundary (MH200 tape lengths 97/171/1046 min/med/max); all arms share the fixed
  seed-0 subsample nuisance already recorded in review S2-4.

### A4 readout rule — registered 2026-09-08, BEFORE the pair contrast is computed
*Written while `re15` is complete (0.080) and `rough15` has 3 of 8 seeds in (0/50, 0/50, 1/50), i.e. before the contrast
exists. Coordinator's instruction, adopted verbatim as the rule of record.*
- **Floor rule.** If both arms of an A4 pair have a mean at or near the floor (≤ ~0.10 on the bank), the contrast between
  them is **uninformative, not null**: a comparison of two arms that both fail almost completely has no power to detect
  anything, and "no difference between rough and re-executed" would imply a finding where there is only a floor.
  In that case: report the per-seed counts, state plainly that both arms floor, and **attach NO significance test** to
  the pair. This is the same treatment the project already applies to `slide_success` (0–6 %, reported as a task outcome
  with no p-value, PHASE_PLAN amendments (l)/(o)); P-A4-2′ is then **not evaluable**, not "met" or "failed".
- **What the pair still establishes, stated positively:** action edits at the registered roughness destroy the data —
  measured independently at BUILD time (yields: MG + smoothing 0/200, MH + noise at ε 0.406 5/200, at ε 0.15 98/200;
  `paper/REPLAY_YIELD_2026-09-08.md`) and at TRAINING time (both arms near the floor). Two independent measurements
  agreeing that the manipulation is destructive is a cleaner statement than any contrast between the two ruined arms.
- **A4's original question — does action roughness specifically explain the machine arm's weakness — cannot be answered
  by this design**, because the manipulation is too destructive to isolate the variable. That is a property of the
  design, stated explicitly, not an inconclusive-by-chance outcome.
- **One attribution that is NOT yet settled and must not be asserted:** the pair's floor is not necessarily caused by the
  manipulation. The CONTROL arm `re15` carries *unmodified* actions and also floors (0.080), and it is small
  (15,702 rows). **A6's `MH80` (natively collected, 16,406 rows) discriminates:** if `MH80` also floors, the A4 pair's
  floor is driven mainly by arm SIZE and the manipulation's learner-side effect is unmeasurable at this scale (a
  stronger statement about the design); if `MH80` is well above the floor, re-execution and/or the edit is the cause.
  Either way the pair contributes no p-value.
- **If the roughness question still matters after A6** it needs a gentler manipulation on a full-size arm; the build-time
  ladder suggests the usable range is narrow or empty (MG survives 3/40 even at the mildest β 0.30 tested; MH survives
  98/200 at ε 0.15, which is only 43 % of MG's roughness and already floors the learner).
