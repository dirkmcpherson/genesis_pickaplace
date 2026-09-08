# Robomimic leg — preparation log (started 2026-09-06 22:40, agent session; brief `~/wm_fix_2026-09-03/agent_brief_robomimic.md`)

Plan: `paper/ROBOMIMIC_PLAN_2026-09-05.md` (+ amendment §A1 for DP, registered here before any DP run). Everything below
is from tool output; job ids are Slurm ids on pax. Data root `$LAB/robomimic_data/`, runs `$LAB/robomimic_runs/`,
r2dreamer copy `$LAB/robomimic_r2d/`, venvs `$LAB/robo_venv` + `$LAB/r2d_venv_robo`. Code: `cluster/robomimic/`, `baselines/robomimic/`.

## 2026-09-06
- 22:40 read brief + plan; cluster reachable; 12 of the user's jobs running (wmfix_full ×8, dv3dbg ×4) — untouched.
- 22:50 launched on the login node (nohup): `download_data.sh`, `build_robo_venv.sh`, `build_r2d_venv_robo.sh`, `copy_r2d_tree.sh`.
- 22:51 `$LAB/r2d_venv_robo` BUILD-OK: `r2d_venv/bin/python -m venv --system-site-packages` overlay (python 3.11.15, torch
  2.8.0+cu126 and numpy 2.4.6 resolved from r2d_venv, mujoco 3.3.7) + robosuite 1.5.1, robomimic v0.5.0 (`--no-deps`), h5py.
- 23:01 downloads complete, sha256 (bytes): ph `3f2eb92e…2962` (46,889,752), mh `c4a34c83…f7d7` (112,792,088),
  mg sparse `1348033e86dea8bb…61d7` (1,098,749,512 — matches the plan's probe), paired `5ad06c35…5d66` (41,692,232).
- 22:59 / 23:01 `robo_venv` verify failed twice on lerobot's import chain (`accelerate`, then `serial`); recipe now installs
  accelerate/omegaconf/hydra-core and runs an import-fix loop (module → pip name); rebuild 3 launched 23:08.
- 23:10 r2dreamer copy `$LAB/robomimic_r2d` (471K, code only) patched: `demo_prefill.py` dims config-driven
  (`env.state_dim`, `env.size` × `env.image_channels`) + `robosuite` log-key branch (8 guarded edits), `envs/__init__.py`
  robosuite branch; `envs/robosuite.py`, `configs/env/robosuite_can_state.yaml`, `eval_robosuite.py` installed. Composed
  config check: task robosuite_can, time_limit 400, state_dim 23, actor_dist bounded_normal, act_entropy 3e-5, return_clamp 1.0.
- 23:12 `$LAB/robo_venv` BUILD-OK after four recipe rounds (lerobot's import chain needs accelerate, pyserial, ...;
  robomimic v0.5.0 imports transformers at module level; transformers 5.x breaks lerobot 0.4.5's groot dataclass →
  pinned `transformers<5` = 4.57.6). Final: python 3.10.14, torch 2.7.0+cu126, sb3 2.8.0, gymnasium 1.2.3, numpy 2.2.6,
  mujoco 3.3.7, robosuite 1.5.1, robomimic 0.5.0, lerobot 0.4.5 (fork genesis-fixes), torchcodec 0.3.0.
  `$LAB/r2d_venv_robo` re-verified after removing a shadow torch that a plain `pip install torchvision` had pulled in
  (torchvision 0.23.0+cu126 `--no-deps` instead): torch 2.8.0+cu126 / numpy 2.4.6 / mujoco 3.3.7 / robosuite 1.5.1 / robomimic 0.5.0.
- **23:20 G0 (plan §5) — the registered clause FAILS; the success flag agrees 10/10.** `baselines/robomimic/g0_replay.py`
  n=10 PH tapes, open-loop replay of the recorded actions from `reset_to({model xml, states[0]})` in robo_venv
  (`$LAB/robomimic_data/g0_report.json`; identical in r2d_venv_robo, `g0_report_r2dvenv_n10.json`):
  - success flag env == file on **10/10**, firing at the file's k or 1–2 decisions later (k_env − k_file ∈ {0,1,1,2,1,0,1,1,0,2});
  - final can position within 1 cm on **3/10** (errors 1.18 / 0.25 / 1.75 / 0.51 / 2.84 / 0.99 / 1.63 / 1.12 / 4.38 / 1.20 cm;
    9/10 within 3 cm); can error AT the first-success row k 1.9–8.8 cm (the can is in hand above the bin: a grasp-pose
    difference, not a task difference); eef tracking error along the whole replay ≤ 2.1 cm (max per tape 0.29–2.12 cm).
  - Mechanism probe (demo_0): restoring `states[t]` and stepping `actions[t]` TWICE gives different results (1.7–9e-3 norm)
    → robosuite carries controller/gripper state outside the MuJoCo state (stateful gripper `current_action`), so the
    single-step "restore + step vs next_obs" test is not a clean physics test; and robomimic's `next_obs` are extracted from
    the RECORDED states (dataset_states_to_obs.py resets to states[t+1]), so open-loop replay compares the installed
    dynamics against the original teleop physics. Live controller == file's controller_configs (OSC_POSE kp 150, damping 1,
    output_max 0.05/0.5, control_delta, uncouple_pos_ori; 20 Hz over 2 ms MuJoCo steps, Euler, elliptic cone, impratio 20).
  - Reading: the installed env reproduces every tape's outcome and timing, but not the can's post-release resting position
    to 1 cm; whether that is "version hell" (mujoco 3.3.7 vs the version the v1.5 files were generated with) is being
    tested with a throwaway mujoco-3.2.3 overlay (`robomimic_data/tmp_mj323_venv`). **Per the brief this is a STOP-AND-REPORT
    item: no GPU run of the primary matrix is submitted. Data preparation (bank, arms, conversions — CPU, deterministic)
    continues under a disclosed override (`prep_data.sh G0_OVERRIDE=...`) so the user can decide Monday with everything
    staged; the BC-RNN control (plan G1) is the decisive follow-up test of usability and is run as such (see below).**
- 23:35 **MuJoCo-version test (throwaway overlay `tmp_mj323_venv`: mujoco 3.2.3 = robosuite 1.5.1's minimum, robosuite
  1.5.1, robomimic 0.5.0):** same 10 tapes → success 10/10, final can errors 1.15 / 0.42 / 1.76 / 0.65 / 2.94 / 0.85 / 1.65 /
  0.97 / 4.32 / 1.12 cm (clause 4/10 vs 3/10 at 3.3.7; per-tape differences ≤ 0.2 cm; `g0_report_mj323_n10.json`).
  The replay residual does not move with the MuJoCo version → it is intrinsic to the re-hosted v1.5 files (recorded
  teleop states vs robosuite-1.5.1 physics), not an install defect. On the registered 5-tape form the clause reads 2/5
  in BOTH versions. The overlay is deleted after this readout; nothing of record runs on it.
- **23:55 CORRECTION — G0 PASSES as registered.** The 23:20/23:35 readouts scored the wrong slice: robosuite's
  `object-state` for PickPlace single-object is `[Can_to_robot0_eef_pos (3, can pose in the GRIPPER frame),
  Can_to_robot0_eef_quat (4), Can_pos (3, WORLD), Can_quat (4)]` (verified live against `sim.data.body_xpos["Can_main"]`
  and the file's obs; `robo_common.CAN_POS = state[16:19]`), so "object[0:3]" was the relative vector, not the can
  position. Re-scored on the WORLD can position (`g0_report.json`, robo_venv, mujoco 3.3.7, 10 PH tapes): final can
  error 0.64 / 0.71 / 1.33 / 0.86 / 0.37 / 0.43 / 1.16 / 0.42 / 1.15 / 0.98 cm → **registered 5-tape clause 4/5 PASS
  (need 4/5); 10-tape 7/10**; success flag agrees 10/10 (env fires at k, k+1 or k+2); t0 physical state (eef pose,
  gripper, can pose) restored exactly (error 0.0). Single-step restore diagnostic (positions only): can median
  0.1–1.0 mm, eef median 1.8–3.3 mm / max 14–18 mm — the eef one-step gap is the un-restorable controller state, not
  physics (restore+step twice differ), so open-loop replay is the fair test and it passes. The earlier "FAIL" entries
  above stand as written (wrong slice, disclosed). The MuJoCo-version conclusion (residual independent of 3.2.3 vs 3.3.7)
  is unaffected.
- Two robosuite observation facts recorded in `robo_common.py` (probe `robomimic_data/obs_probe.py`, `obs_layout.py`,
  `file_rel_probe.py`): (1) the dict RETURNED by `env.reset()/reset_to()` has a stale object block (zeros on the first
  reset, the previous episode's values later) — every reset path now re-reads `env.get_observation()`; (2) the FILES'
  t=0 rows carry robosuite's empty-cache artefact in `object[0:7]` (object pose treated as identity: |object[0:3]| =
  0.999–1.03, recompute error ~900 mm at t=0 vs 0.03–0.06 mm median for t ≥ 2, max 1.6–11 mm) — identical in PH and MH
  (one generator; MG checked below), so it cannot bias the source comparison; our online t=0 observation carries the
  TRUE relative pose (one row per tape differs between demo and online data — disclosed, not "fixed": the files are
  used as published, no re-encoding).
- 23:58 `prep_data.sh` chain launched (G0 record → bank_can50 → arm manifests + masked copies → conversions → random control).
- 00:05 (09-07) Bank + arms built (`prep.log`): `bank_can50.npz` sha256 `72b75550…`, K = 50, state dim 71, restore-twice
  error 0, restore-vs-reseed (physical dims) 1e-7; WORLD can t0 x ∈ [−0.006, 0.215], y ∈ [−0.419, −0.084], min pairwise
  1.02 cm. NB the plan §1 "t0 placements x ∈ [−0.28, 0.03], y ∈ [−0.18, −0.05] in object[0:3]" measured the RELATIVE
  can→eef slice (same layout slip); the registered t0-placement covariate must use `CAN_POS` (world). Arms (rows after
  the cut / tapes / len min-med-max): PH200 22,400 / 200 / 78-111-147; MH200 41,134 / 200 / 97-171-1046 (34/34/33/33/33/33
  per operator, better→worse); MG200s 16,501 / 200 / 45-77-146, 300-block histogram [0,0,0,0,0,0,2,6,10,29,42,55,56];
  MH300 61,548 / 300; MGall 536,522 / 3,900 (718 success); PH200pb 31,242 / 300 (200 success). Masked copies
  (`*_masked.hdf5`, sha 7d03e225… / ab011b3f… / 0074bc45…) carry `mask/<ARM>` for robomimic's trainer.
- 00:10 (09-07) `verify_env.sh`: robo_venv python 3.10.14 | torch 2.7.0+cu126 | numpy 2.2.6 | mujoco 3.3.7 | robosuite 1.5.1 |
  robomimic 0.5.0 | h5py 3.16.0 | gymnasium 1.2.3 | sb3 2.8.0 | lerobot 0.4.5 | hydra 1.3.6; r2d_venv_robo python 3.11.15 |
  torch 2.8.0+cu126 | numpy 2.4.6 | mujoco 3.3.7 | robosuite 1.5.1 | robomimic 0.5.0 | h5py 3.16.0 | gymnasium 1.2.0 |
  tensordict 0.9.1 | torchrl 0.9.2 | hydra 1.3.2. Launcher dry-runs OK (rlpd / r2d / bcrnn).
- 00:12 SUBMITTED (preempt QOS, `--exclude=pax077 --constraint=l40s|a100|l40|h200`, all started at once on pax151/152):
  smokes 2k steps on PH200 (10 bank episodes each): RLPD **3337812**, r2dreamer **3337813** (env.steps 24,600 = 22,600
  prefill + 2,000 online), DP **3337814**; BC-RNN control **3337815** = array 0-8 (%1 while the smokes run; ARM =
  (PH200 MH200 MG200s)[id/3], SEED = id%3; 2000 epochs × 100 steps, LAST checkpoint on the 50-state bank). ≤ 4 GPU jobs in flight.
- 00:20 (09-07) BC-RNN array 3337815 tasks 0-4 FAILED at import (25 s each): `transformers<5` had pulled huggingface_hub
  back to 0.36.2 and diffusers 0.40.0 (imported by `robomimic.algo`) needs `get_cached_repo_tree` (hf_hub ≥ 1.0). Fix:
  `diffusers==0.35.2` in robo_venv (the genesis env's pin; verified `robomimic.algo` + lerobot DiffusionPolicy +
  LeRobotDataset import together; recipe updated with the pin + the wider verify). Tasks 5-8 cancelled; array resubmitted.
  The DP smoke (lerobot's own diffusers use) was unaffected and kept training.
- 00:35 (09-07) SMOKE ROUND 1 (all three FAILED, each at a different, now-fixed point; nothing physics-related):
  - RLPD 3337812: training end-to-end OK (2,000 decisions in 0.02 h on an L40 incl. the 22,400-row demo half; 5 archived
    checkpoints + rlpd_final.zip + sidecars); the mode eval RAN (10 bank episodes in 44.8 s → 0/10 at 2k steps, as expected)
    but the launcher died on its own `grep "^\[eval\]"` (the evaluator prints `[eval rlpd mode]`) under `pipefail`. Fix:
    `^\[eval` + `|| true` in both online launchers. Resubmitted as 3337881.
  - r2dreamer 3337813: prefill OK (200 episodes, 22,600 rows → 22,914 transitions incl. stream padding; step accounting
    22,914 + 1,686 online), then `ImportError: triton_key` in torch.compile — the aborted `pip install torchvision` (23:07)
    had left torch's dependency set in the overlay (triton 3.8 vs the base's 3.4, functorch, nvidia-*/cuda-* cu13,
    setuptools 79 vs the pinned 77.0.3). Removed from the overlay (dirs only; never `pip uninstall` under
    --system-site-packages), recipe's cleanup widened + asserts triton 3.4/inductor import. Resubmitted as 3337882.
  - DP 3337814: lerobot-train OK (2,000 steps @ 0.058 s/step on an L40, 5 checkpoints + last, dataset 22,400 frames /
    200 episodes, 249M params) but the evaluator's `DiffusionPolicy.from_pretrained` failed in draccus 0.11.6
    (ParsingError: no top-level "type" in config.json). Pinned draccus==0.10.0 (the genesis env's version, the one every
    Genesis DP checkpoint was loaded with); load + one action verified on the smoke checkpoint. DP smoke resubmitted.
  - BC-RNN control array 3337852 (after the diffusers pin): task 0 trains at ~0.5 s/epoch (2000 epochs ≈ 17 min);
    throttle 4 → 2 while the smokes rerun (≤ 4 GPU jobs in flight).
- 00:45 (09-07) **Data prep chain COMPLETE** (`prep.log`, 23:37→23:45): conversions (rows after the cut | rlpd transitions |
  r2d rows incl. the final-obs row per tape | lerobot frames): PH200 22,400 | 22,400 | 22,600 | 22,400; MH200 41,134 |
  41,134 | 41,334 | 41,134; MG200s 16,501 | 16,501 | 16,701 | 16,501; MH300 61,548 | 61,548 | 61,848 | 61,548; MGall 536,522 |
  536,522 | 540,422 | — ; PH200pb 31,242 | 31,242 | 31,542 | —. Every arm's manifest carries the source sha256, tape list
  and cut rule; `rlpd/manifest.json` (+ sha of transitions.npz), `r2d/repeat.json` (action_repeat 1, terminal_reward 1,
  state_dim 23, image 16x16x3), `lerobot/robomimic_source.json` (fps 20, proprio 9). **G2a random-policy control on the
  bank: 0/50** (registered ≤ 2/50 — PASS; `robomimic_data/eval_random_bank50/metrics.json`).
- DP checkpoint "type" root cause confirmed: Genesis DP checkpoints on the cluster (draccus 0.10.0) carry
  `"type": "diffusion"` in config.json; the smoke's (saved under draccus 0.11.6) does not, and `PreTrainedConfig.from_pretrained`
  parses config.json against the abstract class → needs the key. Pin = save-time fix; the smoke retrains as 3337917.
- 00:55 (09-07) **SMOKE ROUND 2 — all three learners PASS end-to-end** (train → checkpoint + sidecar → fresh-process bank eval → metrics.json → RESULT line; 10 bank episodes each):
  - RLPD 3337881 (L40, pax152): COMPLETED in 3:00 wall; `RLPD-RESULT rlpd_PH200_smoke_s0 mode=0/10 sample=0/10` (2k decisions = 1k warm-up + 1k UTD-10 updates: 0 expected). Eval 10 episodes / 46 s.
  - r2dreamer 3337882: COMPLETED in 9:13 wall (torch.compile + prefill 22,914 rows + 1,686 online decisions), 12.6 GB RSS at buffer 47,200 rows; `R2D-RESULT r2d_PH200_smoke_s0 mode=0/10 sample=0/10` (0 expected); latest.pt + metrics.jsonl + both eval dirs written; eval 6.5 s/episode.
  - DP 3337917: COMPLETED in 6:10 wall (2,000 grad steps @ 0.058 s + eval); config.json now carries `"type": "diffusion"` (draccus 0.10.0 at save time); **`DP-RESULT dp_PH200_smoke_s0 sample=3/10`** at 2k steps — the data/env/evaluator chain is coherent (a policy trained on the file's obs succeeds in the live env from bank states).
  - Measured rates → primary-run estimates (submit_primary.sh HOURS): RLPD ≈ 59 ms/decision with UTD-10 → 100k ≈ 1.6 h + 8 min eval ≈ 2 h; DP 0.058 s/step → 100k ≈ 1.6 h + ~10 min eval ≈ 2 h; r2dreamer (train_ratio 512 → 250k updates for 500k online decisions) ≈ 6 h from the Genesis rate (UNVERIFIED until the pilot). 72 runs ≈ 240 GPU-h. Memory: 48 g holds a 5.2e5-row buffer; MGall (1.6e6 rows) would need ~96 g.
- **01:20 (09-07) G1 PASS — BC-RNN positive control on PH200** (robomimic's own low-dim BC-RNN recipe, 2000 epochs × 100
  steps, LAST checkpoint = model_epoch_2000, deterministic GMM mode, 50-state bank, fresh process; array 3337852 tasks
  0-2, ~25 min each on L40): **47/50, 46/50, 48/50 = 0.94 / 0.92 / 0.96, mean 0.94 ≥ 0.90** (paper: 100 on the old
  physics, max over checkpoints). The installed env + re-hosted v1.5 data + our bank/evaluator reproduce the published
  regime; together with G0 (4/5) and G2a (random 0/50) the leg's gates that can be run before the primary matrix all pass.
  MH200 / MG200s seeds follow in the same array (throttle 4).
- **01:30 (09-07) CORRECTION to the 01:20 G1 entry:** the launcher's checkpoint pick (`sort -t_ -k3 -n` on the FULL path)
  selected `model_epoch_950.pth`, not the last (`model_epoch_2000.pth`; training did reach 2000 epochs — "finished run
  successfully"). So 47/46/48 (PH200) and 49/50 (MH200 s0) are EPOCH-950 numbers, kept as `eval_bank50_ep950/`. The
  selector now sorts the basename's epoch number; `cluster/robomimic/bcrnn_reeval_last.sh` re-scores every finished run
  at its true LAST checkpoint (fresh process, CPU) — the array's remaining tasks (4-8) still run the old script (Slurm
  copies scripts at submission) and are re-scored by the same tool when they finish. G1 is re-read below.
- **01:50 (09-07) G1 at the TRUE LAST checkpoint (model_epoch_2000, `bcrnn_reeval_last.sh`, fresh CPU process, 50-state bank):
  BC-RNN PH200 s0/s1/s2 = 46/50, 46/50, 46/50 = 0.92 / 0.92 / 0.92 → mean 0.92 ≥ 0.90 — G1 PASS.** MH200 s0 = 49/50 (0.98).
  (Epoch-950 evals kept as `eval_bank50_ep950/`: PH200 47/46/48, MH200 s0 49.) Remaining array tasks (MH200 s1-2,
  MG200s s0-2) are re-scored at LAST by the same tool when they finish; the launcher itself is fixed for future runs.
- **02:20 (09-07) BC-RNN control, remaining arms at the LAST checkpoint (epoch 2000; array 3337852 tasks 4-7 COMPLETED
  23:50–26:51 wall each; re-scored by `bcrnn_reeval_last.sh`):** MH200 s1 45/50, s2 45/50 (with s0 49/50 → **0.98 / 0.90 /
  0.90, mean 0.927**); **MG200s s0 22/50, s1 14/50 (0.44 / 0.28)**; MG200s s2 (task 3337852_8, COMPLETED 23:26 wall) below.
  Read against the plan §5 BC-RNN predictions: MG200s below PH200 by ≥ 0.15 — met by a wide margin (0.92 vs ≈0.36, the
  same direction and size as robomimic Table 1 BC-RNN PH 100 → MG 68.7, larger here on v1.5 physics at LAST rather than
  max-over-checkpoints); MH200 within 0.10 of PH200 — met (0.927 vs 0.92). Every run's epoch-950 eval (the array's
  own eval step) is kept as `eval_bank50_ep950/` (verified present for MH200 s1/s2 and MG200s s0/s1/s2); `eval_bank50/`
  is the LAST-checkpoint score.
- **02:35 (09-07) BC-RNN control COMPLETE — all nine runs at the LAST checkpoint (epoch 2000), 50-state bank, deterministic:**
  PH200 46/46/46 (0.92 / 0.92 / 0.92, mean 0.92), MH200 49/45/45 (0.98 / 0.90 / 0.90, mean 0.927), **MG200s 22/14/23
  (0.44 / 0.28 / 0.46, mean 0.393)**. Registered BC-RNN predictions (plan §5): MG200s ≤ PH200 − 0.15 → Δ = −0.53 (met);
  MH200 within 0.10 of PH200 → Δ = +0.007 (met). Direction and size match robomimic Table 1 (BC-RNN PH 100 / MH 100 /
  MG 68.7 on the old physics, max over checkpoints); the MG drop is larger here (LAST checkpoint, v1.5.1 physics, our
  bank). G1 (PH200 ≥ 0.90) stands at 0.92. Array 3337852 tasks 0-8 all COMPLETED (23:26–26:51 wall each, L40/L40S);
  per-run `eval_bank50/metrics.json` (LAST) + `eval_bank50_ep950/` (the array's own eval, kept). No GPU job of ours left in the queue.

## 2026-09-07 13:30 — RLPD arm of the robomimic matrix READ OUT: the registered falsifier FIRED

`eval_bank50_mode` (LAST checkpoint, deterministic, 50 shared starts, 8 seeds): **MH200 [27, 44, 30, 1, 20, 9, 24, 27] = 182/400 (0.455) vs MG200s [10, 2, 2, 13, 6, 12, 9, 5] = 59/400 (0.147); Δ +0.307, exact two-sided permutation p = 0.008**; sampled actions 0.458 vs 0.168, Δ +0.290, p 0.011. Plan §5 falsifier (a) — an online learner with Δ ≥ 0.15 and p < 0.05 on the source axis — is met for RLPD: on an independent machine generator, RLPD is NOT source-indifferent; it learns far better from mixed-skill human demonstrations than from SAC rollouts. G3 learnability (≥ 0.5 in ≥ 3/8 seeds) passes for MH200 (4/8) and fails for MG200s (0/8). Caveats carried from the plan: rows are unmatched (MH200 ≈ 41k transitions vs MG200s ≈ 16.5k; MG tapes are short), MG starts come from SAC's own reset distribution, and RLPD's 100k-decision budget is short of BC-RNN's 0.93 on MH (per-seed spread 1–44 of 50 includes a dead seed). Follow-ups (registered as an amendment by the robomimic agent before running): RLPD on **MGall** (all 3,900 rollouts incl. failures, 585k rows — the registered secondary) and on **MG718s** (every successful rollout, ≈ 59k rows ≥ MH200's) to separate quantity from source; the registered budget extension (100k → 300k decisions) for the MG arm. r2dreamer and DP arms still running.

## 2026-09-07 — RLPD primary readout trigger + amendment A2 controls
- 11:00 Coordinator: RLPD arm read out with falsifier (a) firing on the quality axis. Verified from the run dirs
  (`robomimic_runs/rlpd/rlpd_<ARM>_s<k>/eval_bank50_{mode,sample}/metrics.json`, LAST): MH200 mode [27,44,30,1,20,9,24,27]
  = 0.455 / sample [29,41,31,2,22,7,28,23]; MG200s mode [10,2,2,13,6,12,9,5] = 0.147 / sample [7,1,4,16,12,11,9,7].
  **PH200 RLPD (3346687–94) has NO result: the 8 runs were CANCELLED (train.log only, no checkpoint)** — the registered
  PRIMARY contrast PH200 v MG200s is therefore not yet measured for RLPD; flagged to the coordinator.
- Amendment A2 registered in the plan (commit 10e5662) BEFORE building or submitting anything: C1 MGall ×8, C2 MG718s
  (new arm, every successful MG rollout) ×8, C3 MG200s @ 300k decisions ×8; predictions P1–P3 + decision rules there.
- Transition counts per arm (`robomimic_data/arms/arm_counts.json`; rlpd transitions = rows after the cut; rewarded =
  terminal +1 rows; density = rewarded/transitions): PH200 22,400 / 200 / 0.89 %; MH200 41,134 / 200 / 0.49 %; MG200s
  16,501 / 200 / 1.21 %; MH300 61,548 / 300 / 0.49 %; MGall 536,522 / 718 / 0.13 %; PH200pb 31,242 / 200 / 0.64 %.
  **MG200s demo rows by SAC checkpoint block (300-rollout blocks in index order, blocks 0–12):** tapes
  [0,0,0,0,0,0,2,6,10,29,42,55,56], row fractions [0,0,0,0,0,0,0.011,0.037,0.064,0.144,0.227,0.262,0.256] — 89 % of the
  MG200s demo rows come from the last four blocks (checkpoints 10–13 of 13), 0 % from the first six. MGall rows by block:
  0.084 ×6 (all-failure 150-row blocks), then 0.084, 0.082, 0.081, 0.068, 0.069, 0.057, 0.057 (successes are shorter).
- 11:25 MG718s built with the same tooling (`make_arms.py --only MG718s`, `convert_arms.py`): 718 tapes / 59,222 rows
  (rlpd 59,222 transitions, 718 rewarded, density 1.21 %; r2d 59,940 rows; lerobot 59,222 frames), tape len 45/76/150,
  MG200s ⊂ MG718s verified, block tapes [0,0,0,0,0,0,3,24,31,130,132,195,203]; mask `MG718s` added to the mg masked copy
  (its sha is now d6380b96…; MG200s/MGall masks unchanged). `arm_counts.json` updated.
- 11:27 **A2 controls SUBMITTED** (`sbatch_rlpd_robo.sh`, preempt QOS, `--requeue --exclude=pax077`, `--nice=5000` so the
  matrix's 16 pending dp runs go first; 20 GPUs in use by r2d ×16 + dv3dbg ×4): C1 MGall s0–7 = **3349120–3349127**;
  C2 MG718s s0–7 = **3349128–3349135**; C3 MG200s@300k s0–7 = **3349136–3349143** (TAG ext300k, K = 5 checkpoints).
  Outputs `robomimic_runs/rlpd/rlpd_{MGall,MG718s}_ctl_s<k>/` and `rlpd_MG200s_ext300k_s<k>/`, LAST on bank_can50 mode+sample.
- 12:10 (09-07) Adversarial review `paper/ADVERSARIAL_REVIEW_robomimic_ops_2026-09-07.md` S1/S2 read. **G2b (no-demo RLPD ≤ 0.10)
  SUBMITTED: 3350586–3350593** (`ARM=none`, 8 seeds, 100k decisions, recipe of record with the demo half EMPTY —
  `train_rlpd_robosuite.py` now forces demo_batch 0 for `--demo none` and asserts the demo sampler draws 0 rows; the
  earlier code would have fed a constant zero row as half of every batch), name robo_rlpd_nodemo, out `rlpd_none_s<k>/`.
- Action statistics recomputed from the arms' rlpd transitions (`robomimic_data/arms/action_stats.json`; arm dims 0-5,
  roughness = mean |a_t − a_{t−1}| within tapes): PH200 |a| 0.236, |Δa| 0.050, gripper binary 100 %, saturated 7.4 %;
  MH200 0.144 / 0.043 / 100 % / 2.4 %; **MG200s 0.664 / 0.274 / 1.0 % (1,903 distinct gripper values) / 12.2 %**;
  MG718s 0.666 / 0.274 / 1.1 % / 12.3 %; MGall 0.620 / 0.386 / 0.5 % / 7.8 %. Per-dim mean |a| MG200s
  [0.78, 0.71, 0.70, 0.58, 0.61, 0.60] vs MH200 [0.19, 0.35, 0.21, 0.02, 0.04, 0.06] — the SAC policy drives every OSC dim,
  incl. the three rotations humans barely touch, at ~4× the human magnitude. Confirms the review's S1-1 numbers.
- **13:05 (09-07) A4 build — MG tapes are NOT open-loop reproducible in the installed env.** `g0_replay.py --demos` on the
  first six MG200s tapes (successful SAC rollouts, `g0_report_MG200s_n6.json`): env success **0/6** (file: 6/6), eef
  error along the replay 6–94 cm, final can error 48–94 cm; one-step restore eef error median 5.5–9.6 mm (PH: ~2 mm).
  Failed MG rollouts (demo_1–4, can never moves) trivially "pass" (can error 0.0 cm) — uninformative. Consequently the
  A4 MG pair's CONTROL R0 (original actions re-executed) yields ≈ 0 (build log: 0/200 at the first checkpoints), so
  per the registered A4 rule the MG pair is **not buildable by open-loop replay**; only R3 v R2 can be read. The MH
  control (R2) re-executes at ~95 % (20/21 at the first checkpoint), the roughened treatment (ε for |Δa| 0.274) at
  ~5 % — a ε yield-ladder (40 tapes, ε ∈ {0.05, 0.1, 0.15, 0.2, 0.3}) is running to find the largest roughness that
  keeps ≥ 100 paired tapes; an addendum will register the chosen ε before any R2/R3 training.
  Interpretation to carry into every MG readout: every MG demo tuple's a → s′ is a response the installed robosuite-1.5.1
  controller does not reproduce (the SAC data were generated under an earlier robosuite/mujoco-py controller and only
  the states were re-hosted), whereas human tapes replay to ≈ 1 cm (G0). For an off-policy learner that bootstraps
  through demo tuples this is an "off-dynamics demonstration" confound shared by MG200s, MG718s and MGall.
- 13:15 **Correction of the 13:05 interpretation:** the one-step restore discrepancy is source-INDEPENDENT in relative
  terms (`robomimic_data/step_rel_err.py`, 5 tapes per source, ~20 restored steps each: |eef_env − eef_file| / |file eef
  step| median PH 0.57, MH 0.58, MG 0.56; p90 0.92 / 0.90 / 0.91). Absolute errors scale with the commanded step
  (2.5 / 2.8 / 6.9 mm for file steps of 8.2 / 8.9 / 14.8 mm) — i.e. the same un-restorable controller state for every
  source, not an MG-specific dynamics regime. So "off-dynamics demonstrations" is NOT supported; what is established:
  MG tapes' large, saturated motions are not tolerant to that discrepancy when replayed open-loop (0/6), human tapes are
  (PH 10/10, MH ~95 %). The consequence for A4 is unchanged (MG pair unbuildable by open-loop replay); the confound
  statement for the MG readouts should say "not open-loop reproducible", nothing stronger.
- **13:40 (09-07) A4 yields (measured BEFORE any training; `logs/build_A4_*.log`, `logs/ladder_A4_MH.log`):**
  - MG pair (registered target |Δa| 0.043 = MH200's; EMA β = 0.8597 by bisection, gripper binarised at 0; modified arm
    |a| 0.48, |Δa| 0.043): **R0 control 114/200 re-execute successfully (57 %) — so the 0/6 above was the first six tapes,
    not the arm; R1 treatment 0/200; BOTH 0/200.** R1's yield < 100 ⇒ the registered R1m (per-tape magnitude restored
    after the EMA) is being built now; a β ladder (40 tapes, β ∈ {0.3, 0.5, 0.7, 0.86}) runs alongside as a diagnostic.
  - MH pair (registered target |Δa| 0.274 = MG200s's; uniform noise, ε by bisection): control 130/141 at the last
    checkpoint (~92 %), treatment 4/141 (~3 %) — the registered roughness is not open-loop executable by human tapes.
    **ε yield-ladder (40 tapes, control 37/40):** ε 0.05 → |Δa| 0.061, both 33/40; ε 0.10 → 0.087, 29/40; ε 0.15 →
    0.115, 25/40; ε 0.20 → 0.144, 19/40; ε 0.30 → 0.205, 4/40. The largest roughness that keeps ≥ 100 paired tapes of 200
    is ε = 0.15 (|Δa| ≈ 0.115 = 2.7× MH's, 42 % of MG's; expected ~125 paired tapes); ε = 0.20 (|Δa| 0.144, 53 % of
    MG's) is borderline (~95 expected). Addendum to A4 to be registered with the chosen ε before any R2/R3 training.
- 14:05 (09-07) A4 registered-parameter builds finished (`tasks` output; 1,228 s for the MH pair):
  - MG200s pair: β = 0.8712 (bisected on the full arm; modified |a| 0.471, |Δa| 0.0431, gripper binary 100 %) →
    **control 114/200, treatment 0/200, BOTH 0/200 → FATAL, no arms written.**
  - MH200 pair at the REGISTERED roughness (ε = 0.4056 for |Δa| 0.2741, modified |a| 0.267) → **control 185/200 (93 %),
    treatment 5/200, BOTH 5/200**; the two 5-tape arms `MH200_re` / `MH200_rough` were written (732 / 740 rows) but are
    far below any usable N — not trainable, kept only as provenance.
  - MG β-ladder (40 tapes, control 20/40): β 0.30 → |Δa| 0.139, both 3/40; β 0.50 → |Δa| 0.103, both 0/40. Even a
    mild EMA destroys MG tapes' open-loop success, so R1/R1m are unbuildable at any smoothing that reaches the target.
- **14:30 (09-07) A4 arms built; addendum registered in the plan before submitting.** Final yields (of 200 source tapes,
  kept iff BOTH control and treatment re-execute to success): MG pair R0 114 / R1 0 / R1m 0 / β-ladder 3-0-0-0 → MG
  treatment ABANDONED per the registered rule (P-A4-1 withdrawn, unrun). MH pair: registered ε 0.4056 → control 185,
  treatment 5 (arms `MH200_re`/`MH200_rough` written, 5 tapes, never trained); **ε 0.15 → BOTH 95** (`MH200_re15`
  15,702 rows |Δa| 0.0455; `MH200_rough15` 15,681 rows |Δa| 0.1177); **ε 0.20 → BOTH 61** (`MH200_re20` 9,806 rows
  |Δa| 0.0439; `MH200_rough20` 9,781 rows |Δa| 0.1459). Dose = 43 % / 53 % of MG200s's roughness; gripper binary in all.
- **14:35 (09-07) A4 SUBMITTED: 32 RLPD runs, 8 seeds per arm, `--nice=6000` (behind the A2 controls and G2b), preempt QOS,
  100k decisions, LAST on bank_can50 (mode + sample):** MH200_re15 **3351474–3351481**, MH200_rough15 **3351482–3351489**,
  MH200_re20 **3351490–3351497**, MH200_rough20 **3351498–3351499, 3351501–3351506** (out `rlpd_<ARM>_a4_s<k>/`).
  Queue order of ours: 16 dp (matrix) → 8 G2b nodemo → 24 A2 controls (nice 5000) → 32 A4 (nice 6000).

## 2026-09-07 evening — filesystem-full incident and recovery
- 16:00–17:00 `/cluster/tufts/shortlab` reached 1.9T/1.9T (0 free); every job on the share died. Mine: 32 robo_rlpd_a4,
  24 robo_rlpd_ctl, 8 robo_rlpd_nodemo, 7 robo_dp, 1 robo_r2d FAILED (queued ones exit 0:53, 0 elapsed). Not a code
  fault. Coordinator freed 434 GB (77 % used at recovery time).
- **Inventory of my runs (what survived, verified file by file — including the timestamped-subdir gotcha: our r2dreamer
  logdir is set explicitly (`logdir=$LOGDIR`) so `latest.pt` sits directly in the run dir; `find -name latest.pt`
  confirms one per run, no timestamped subdir):**
  - RLPD: 16 matrix cells intact (MH200 s0-7, MG200s s0-7: `rlpd_final.zip` + both eval metrics + 5 archived ckpts).
    No A2/A4 dirs existed (those jobs died in the queue). G2b `rlpd_none_s0-7` and the old cancelled `rlpd_PH200_s0-7`
    held only partial/empty artefacts → deleted.
  - r2dreamer: 15 cells complete (MH200 s0-7, MG200s s0-6). **MG200s s7 has a `latest.pt` but the run died at step
    328,899 of 516,701 — a PARTIAL checkpoint, not a scoreable LAST → retrain, not re-eval** (the eval-only shortcut
    would have silently scored a two-thirds-trained model).
  - DP: MH200 s0-7 complete (training + eval). MG200s s0 complete; **s1-s4 finished training ("End of training",
    `last`→100000) but lost their evals → EVAL-ONLY recovery**; s5, s6 (`last`→060000) and s7 (`last`→040000) died
    mid-training → retrain.
- **Standing rule implemented (keep only the final checkpoint):** `sbatch_rlpd_robo.sh` now defaults `CKPT_FRACS=1.0`
  (was 0.2,0.4,0.6,0.8,1.0 — five ~25 MB zips per run scored by nothing, our statistic is LAST);
  `sbatch_dp_robo.sh` now defaults `SAVE_FREQ=$STEPS` (one ~1 GB checkpoint per run instead of five);
  r2dreamer already keeps a single overwritten `latest.pt`. All four launchers gained a **pre-flight disk guard**
  (`MIN_FREE_GB`, default 100 GB: refuse to start rather than half-write a run) and a new eval-only launcher
  `cluster/robomimic/sbatch_dp_eval_robo.sh` (refuses to score a checkpoint whose step ≠ the run's budget).
- **BATCH 1 submitted 17:2x (matrix completion; 434 GB free at submit):** r2d MG200s s7 retrain **3354420**;
  DP MG200s retrains s5 **3354421**, s6 **3354422**, s7 **3354423**; DP MG200s eval-only s1 **3354424**, s2 **3354425**,
  s3 **3354426**, s4 **3354427**. Batches 2-4 (G2b, A2 controls, A4) follow in order, each gated on ≥ 150 GB free.
- **BATCH 2 (G2b no-demo, gate 442 GB free): 3354437–3354444.** **BATCH 3 (A2 controls, gate 442 GB free, `--nice=5000`):
  MGall s0-7 3354448–3354455, MG718s s0-7 3354456–3354463, MG200s@300k s0-7 3354464–3354471.** Batch 4 (A4, 32 runs,
  `--nice=6000`) is staged behind them under the same ≥ 150 GB gate.
- **BATCH 4 (A4 dose pairs, 32 runs, gate 444 GB free, `--nice=6000`): 3354850–3354881** (MH200_re15 3354850–57,
  MH200_rough15 3354858–65, MH200_re20 3354866–73, MH200_rough20 3354874–81). All four recovery batches are now queued
  in the registered priority order; every launcher refuses to start below 100 GB free.
- 18:0x (09-07) Recovery batch 1, eval-only cells landed (DP, LAST checkpoint, sampled actions, 50-state bank):
  `DP-RESULT dp_MG200s_s1 1/50`, `s2 8/50`, `s3 2/50`, `s4 4/50` (with the surviving s0 6/50). DP MH200 (all 8, intact):
  41, 44, 39, 46, 45, 42, 43, 45 /50. DP MG200s s5-s7 are retraining (3354421-23); no DP contrast is computed until all
  8 MG200s cells exist. Disk 440 GB free; 8 G2b + 3 A2 running, 21 A2 + 32 A4 pending.
- **18:3x (09-07) G2b READ-OUT — registered gate PASSES, and it decides the reading of the MG result.** RLPD with NO
  demonstrations, 8 seeds, 100k decisions, LAST on bank_can50: **mode 0/50 on every seed (0.000), sample 0/50 on every
  seed** (jobs 3354437–3354444, `rlpd_none_s0-7`). Registered clause "RLPD without demos ≤ 0.10" → **PASS at 0.00**.
  Verified per run before reading: sidecars show `demo_batch 0`, `demo "none"`, `ckpt_step 100000` (budget reached),
  evals `episodes 50`, `bank_sha256 72b75550…` — i.e. a genuinely empty demo half, not the old zero-row placeholder.
  **Consequence for the MH200-v-MG200s result:** MG200s (0.147 mode) is *above* the demo-free floor (0.000), so MG
  demonstrations are worth something to RLPD — the effect is "MG helps less than MH", not "MG is worth nothing".
  Both readings were live before this run; only the first is now supported.
  Standing-rule check on the same runs: 1 archived checkpoint per run (`ckpt_100`) + `rlpd_final.zip`, 25 MB per run.
- **19:0x (09-07) A2 C1 (MGall) FIRST TWO SEEDS — directionally against prediction P2 and against the source reading.**
  `rlpd_MGall_ctl_s0 mode 37/50 (0.74), sample 33/50`; `s1 mode 30/50 (0.60), sample 35/50`. Compare MH200 0.455 and
  MG200s 0.147 (both n=8). Provenance verified on both runs before recording: `arm MGall`, demo sha `f0f3536e…`
  (536,522 transitions / 718 rewarded — the published set incl. 3,182 failed rollouts), gamma 0.99, demo_batch 128,
  ckpt_step 100000, eval episodes 50, bank `72b75550…`. **Registered P2 predicted MGall < MH200 by ≥ 0.15; the first two
  seeds are ABOVE MH200.** If this holds at n=8, the A2 decision rule fires: "if MG718s or MGall reaches MH200 − 0.10 →
  the effect was quantity/coverage, not source", i.e. the MH200-v-MG200s gap is a property of the 200-tape MG subsample
  (16.5k rows, late-checkpoint-only), not of machine provenance. NOT a readout: 2 of 8 seeds, no test computed yet.
- 19:3x (09-07) DP retrains landed: `dp_MG200s_s5 6/50`, `s6 6/50` (s7 3354423 still training). DP matrix so far
  (LAST, sampled, /50): MH200 41 44 39 46 45 42 43 45; MG200s 6 1 8 2 4 6 6 –. No DP contrast until s7 exists.
- **20:0x (09-07) DP ARM OF THE MATRIX COMPLETE (amendment A1 statistic: LAST checkpoint, SAMPLED actions, 50-state bank,
  n = 8 v 8):** MH200 [41,44,39,46,45,42,43,45] = **345/400 = 0.863** (per-seed sd 0.047) vs MG200s [6,1,8,2,4,6,6,5] =
  **38/400 = 0.095** (sd 0.046); Δ = **+0.767**, exact two-sided permutation **p = 0.00016** (12,870 splits — the minimum
  attainable at n = 8 v 8, i.e. complete separation). Cells s1-s4 are the eval-only recoveries, s5-s7 retrained after the
  filesystem incident; all eight carry `bank_sha256 72b75550…` and the LAST checkpoint at 100k grad steps.
  A1's registered DP prediction ("MG200s below PH200 by ≥ 0.15") cannot be tested as written — the matrix has no PH200
  DP arm (amendment A3 dropped PH) — but the same-direction MH200-v-MG200s contrast exceeds the 0.15 margin by 5×.
  Cross-learner picture on the identical arms (bank, LAST): BC-RNN MH 0.927 / MG 0.393 (Δ 0.53); DP MH 0.863 / MG 0.095
  (Δ 0.77); RLPD MH 0.455 / MG 0.147 (Δ 0.31, mode). All three learners lose on MG200s; the A2/A4 controls (running)
  decide whether that is quantity, budget, action process, or source — the first two MGall seeds (0.74, 0.60) already
  argue against the source reading.
- 20:2x (09-07) **VPN/DNS dropped** (`ssh: Could not resolve hostname login.pax.tufts.edu`) — the known intermittent
  outage, not a cluster fault. Jobs keep running unattended; at the last contact: r2d MG200s s7 retrain + 1 A2 control
  running, 21 A2 + 32 A4 pending, 430 GB free. Nothing to resubmit on reconnect — all four recovery batches are queued
  with the disk guard; the pickup is to read results with the monitor's job-id ranges (batch 1 3354420-27, G2b
  3354437-44, A2 3354448-71, A4 3354850-81).

## 2026-09-08 07:1x — A2 CONTROLS COMPLETE: the source reading is dead
All 24 A2 runs COMPLETED (100k / 300k decisions reached, `ckpt_step` = budget; MGall demo sha `f0f3536e…`, MG718s
`fd80caf7…`, MG200s@300k `3935c6bd…` = the primary arm's own set; bank `72b75550…`, 50 episodes, LAST checkpoint).
Mode counts /50, n = 8 (sample in brackets), against the primary cells MH200 0.455 and MG200s 0.147:
- **C1 MGall** [37,30,34,22,14,39,32,36] = **0.610** (sd 0.170; sample 0.570); ≥ 0.5 in **6/8** seeds.
- **C2 MG718s** [31,17,32,32,27,0,28,23] = **0.475** (sd 0.217; sample 0.477); ≥ 0.5 in **5/8**.
- **C3 MG200s @ 300k** [0,22,25,4,24,25,12,3] = **0.287** (sd 0.217; sample 0.280); ≥ 0.5 in **2/8**.
Exact two-sided permutations (mode | sample): MGall v MG200s **+0.463 p 0.0002** | +0.403 p 0.0011; MG718s v MG200s
**+0.328 p 0.0033** | +0.310 p 0.0042; MGall v MH200 +0.155 p 0.192 | +0.113 p 0.342; **MG718s v MH200 +0.020 p 0.886**
| +0.020 p 0.888; MG200s@300k v MG200s +0.140 p 0.115; MG200s@300k v MH200 −0.168 p 0.190; MGall v MG718s +0.135 p 0.202.
**Registered predictions:** P1 MET in its first half (MG718s − MG200s = +0.328 ≥ 0.10) and **FALSIFIED in its second**
(predicted MG718s < MH200 by ≥ 0.15; measured **+0.020**, p 0.886 — statistically indistinguishable from the human arm).
P2 MET (|MGall − MG718s| = 0.135 < 0.15 band on the sample cell, 0.135 mode; MGall did NOT sink) but its consequence
(MGall < MH200 by ≥ 0.15) is **FALSIFIED** — MGall is +0.155 ABOVE MH200. P3: the budget extension moved MG200s
+0.140 (p 0.115) and left it 0.168 below MH200 with 2/8 seeds ≥ 0.5 — G3 still not met at 300k; budget is a partial,
non-significant contributor, not the explanation.
**A2 decision rule fires:** "if MG718s or MGall reaches MH200 − 0.10 → the effect was quantity/coverage, not source."
Both do. **The MH200-v-MG200s ordering is not a demonstration-source effect for RLPD**; it is a property of the
200-tape MG subsample (16,501 rows, 200 rewarded terminals, 89 % of rows from the last four SAC checkpoint blocks).
Machine demonstrations at their published size match (MG718s) or exceed (MGall) the human arm.
Wall clock (sidecar `hours`): MGall 0.77–2.25 h/run (median ~1.04), MG718s 1.00–1.03, MG200s@300k 2.29–6.68 (median ~3.0);
MH200/MG200s primaries 0.76–1.60. So **8 RLPD seeds ≈ 8–12 GPU-h wall, ~1 h each in parallel** — the cost of the never-run
PH200 arm.
- **2026-09-08 07:4x — BC-RNN GMM-head confound CONFIRMED, and it is stronger than "an interaction".** Checked the
  configs that actually ran (`robomimic_runs/bcrnn/bcrnn_<ARM>_s<k>/config.json`) and, independently, the `config` blob
  inside `model_epoch_2000.pth`: **PH200 and MH200 ran with `algo.gmm.enabled = True`, num_modes 5, low_noise_eval True;
  MG200s ran with `algo.gmm.enabled = False`** (gaussian False, vae False, l2 1.0 → a deterministic MSE head). Everything
  else is identical across arms (RNN LSTM 2×400, actor_layer_dims [], lr 1e-4, seq_length 10, batch 100, 2000 epochs).
  Origin: `make_bcrnn_config.py:71` `config.algo.gmm.enabled = (dtype != "mg")`, copied deliberately from robomimic's own
  `generate_paper_configs.py` (`if dataset_type == "mg": config.algo.gmm.enabled = False`) and disclosed in that file's
  docstring — but never carried into the results as a confound. **So the BC-RNN row (MH200 0.927 v MG200s 0.393) compares
  two different policy classes on two different datasets; it is not a like-for-like demonstration-source contrast.**
  Cheap discriminating test (head held fixed across arms, everything else unchanged): (a) MG200s with GMM ON, (b) MH200
  with GMM OFF, 3 seeds each = 6 runs; measured BC-RNN wall clock 24–27 min/run incl. eval → **≈ 2.6 GPU-h**, no new data
  and no code beyond a one-line `--gmm on|off` flag in `make_bcrnn_config.py`. Held pending registration + go.
- **2026-09-08 08:0x A5 SUBMITTED (fixed-head BC-RNN control; amendment A5 registered first, commit f5340f0).**
  Code: `make_bcrnn_config.py --gmm recipe|on|off` (default `recipe` = robomimic's per-dataset setting, i.e. what every
  09-07 cell used) plumbed through `sbatch_bcrnn_robo.sh` as `GMM=`; verified by generating both configs before
  submitting — `MG200s --gmm on` → `gmm True, modes 5` (recipe would be False), `MH200 --gmm off` → `gmm False`
  (recipe would be True), filters/RNN/epochs unchanged. Jobs (3 seeds each, gate 406 GB free):
  **MG200s_gmm 3370075, 3370096, 3370097; MH200_nogmm 3370098, 3370099, 3370100** (run dirs
  `bcrnn_MG200s_gmm_s<k>`, `bcrnn_MH200_nogmm_s<k>`; LAST = epoch 2000, scored on bank_can50).
  Reference cells: MH200-GMM 0.927, MG200s-det 0.393, PH200-GMM 0.92. A4's 32 RLPD runs are running alongside (13 R, 10 PD).
- **2026-09-08 09:xx A4 control arm `MH200_re15` COMPLETE (8 seeds) and it FALSIFIES P-A4-3′:** mode [2,8,2,5,2,0,4,9]
  = **0.080** (sd 0.061; sample 0.060) versus MH200@100k **0.455** — a −0.375 gap, far outside the registered ±0.10
  neutrality band. These are UNMODIFIED human actions re-executed open-loop; the only differences from MH200 are the
  tape selection (95 of 200 tapes that survive both re-executions) and the row count (15,702 vs 41,134).
  Registered decision rule (iii) therefore fires: **the A4 pair may be read only within itself (rough15 v re15), never
  against MH200@100k.** Note the pattern this adds to the A2 controls — RLPD success tracks demo ROW COUNT across every
  arm measured so far, independent of source: re15 15.7k rows → 0.080, MG200s 16.5k → 0.147, MH200 41.1k → 0.455,
  MG718s 59.2k → 0.475, MGall 536.5k → 0.610.

## Methodological finding (2026-09-08): open-loop re-execution of recorded demonstrations is NOT free
Registered prediction P-A4-3′ ("the re-executed control arms are within 0.10 of the native arm") is **FALSIFIED**:
`MH200_re15` — *unmodified* human actions replayed open-loop from `reset_to({model, states[0]})`, tapes kept only where
the replay still succeeds — scores **0.080** (mode, 8 seeds) against MH200's **0.455**. Stated precisely, because the
same care the correction above demands applies here: the two arms differ in re-execution AND in size (15,702 vs 41,134
rows, 95 vs 200 tapes), so **the −0.375 is an upper bound on the cost of re-execution, not a clean estimate of it** —
the components are not separable with the data in hand (A6 below would separate them).
What IS clean, and is the transferable lesson for any work that plans to replay or retime demonstrations (including our
discrete-action line, where re-execution was assumed approximately free):
- **Human tapes:** 185/200 (93 %) reproduce their success when replayed open-loop with the ORIGINAL actions.
- **Machine (SAC) tapes:** 114/200 (57 %) do — the same env, the same replay code.
- **Any action edit collapses reproducibility:** MG + causal-EMA smoothing to human roughness → **0/200**; MG + EMA with
  magnitude restored → 0/200; MG at milder smoothing (β 0.30) → 3/40. Human tapes + uniform noise at the registered
  roughness (ε 0.406) → 5/200; at ε 0.20 → 63/200; at ε 0.15 → 98/200.
- So a "replay the recorded actions with a small modification" design loses most of its data at the build stage, and
  what survives is a selected, smaller set whose learner performance is far below the native arm's. Budget for both.
