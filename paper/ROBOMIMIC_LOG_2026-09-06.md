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
