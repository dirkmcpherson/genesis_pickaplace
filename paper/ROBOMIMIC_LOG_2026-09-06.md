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
