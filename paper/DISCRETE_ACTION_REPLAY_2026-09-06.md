# Discrete-action replay check: do snapped joystick commands still reproduce the demos? (2026-09-06)

*User question (09-06): "I'm curious about [the discrete action space] ... dv3 works better with discrete actions." Measured command statistics: `paper/OG_DEMO_ACTION_DISCRETENESS_2026-09-06.md`. Script: `baselines/discrete_replay_check.py`. Outputs: `~/wm_fix_2026-09-03/discrete_replay/` (per-uid json + trajectory npz per code, `vocab.json`, `table.{md,json}`, logs). Registered 2026-09-06 23:10 before any full run (§2); one smoke uid seen first, disclosed in §2.*

## 1. Setup (verified, not assumed)

- **Path of record for this check: the commanded-Cartesian replay** `baselines/cartesian_env.CartesianCanEnv(control='vel')` (built 07-24..26): each tape frame's twist integrates a tool setpoint at `DT = 0.025` s (clamped to the teleop workspace box; pitch setpoint clamped to `PITCH_RANGE`), the wrist is IK'd every frame, and `GenesisCanEnv.step` applies the joint-position targets + the gripper target (`gripper_targets(0..100)`) for 3 physics steps of 0.01 s (substeps 8). The tape is `inthewild_trials/<uid>_cartesian.npy`: one frame per `base_feedback` message (40 Hz, 0.025 s; the discreteness note's "30 Hz" was the `_episodes.npy` rate), `cartesian_velocity` (x, y, z, pitch = wy; wx = wz = 0 on every frame, asserted) and `gripper_pos` (0..100). Per-uid least-squares dt of Σ command vs `tool_pose` displacement: median 0.0257 s (consistent with the path's DT). Same path, same DT, same physics for every code — the comparison of record is raw-commanded vs snapped-commanded on this path.
- **World:** `gc_kp4_riser3_shelf6` via `sim_variant_hook.apply_pre` (before the world build) + `apply_post(env, name)` (after) — exactly as the recorder and `honest_rescore`. `CartesianCanEnv`/`GenesisCanEnv` do NOT apply the variant themselves (the 07-24 validation "uid 232 picks+contacts" was the base world); the script applies it and asserts the shelf top 0.17 / goal spawn 0.263 / pick_z 0.1505. Disclosed path detail: the wrist→tool offset is calibrated at reset against `REF_TOOL_AT_START` (a base-world constant); under the 3 cm riser this only mis-sizes the pitch lever arm (≤ 1.6 cm at the demo pitch extremes), identically for all codes.
- **Initial conditions:** `GenesisCanEnv.reset(uid=...)` = the recovered can placement of record (`can_pos_recovery/trial_placements.json`, `status ∈ {ok, ok_batch}`) + the static goal (0.672, −0.221) at the shelf-following spawn height. **Uids:** the 74 success-labelled solved trials (= the w3 census fit+holdout set, checked equal).
- **Stage predicates (the harness's own):** `picked` = GenesisCanEnv's sustained held-can guard; `contact` = GenesisCanEnv's (picked ∧ can–goal contact ∧ eef behind the can); `placed` = honest_rescore's `placed_shift` (picked ∧ shelf footprint ∧ shelf_top+0.01 < z < shelf_top+0.07 — the env's own `placed` band is unshifted and cannot fire in shelf6 worlds, recorded as `placed_env` only); `set-down` = placed ∧ grip cmd < 0.5 ∧ |z − rest| < 1.5 cm ∧ tilt < 20°; `nested` = `GenesisCanEnv._nested()` at the tape's end (100 settle steps, picked ∧ centre distance ≤ 0.081 ∧ both upright) — the eval rule of record; `tipped` = can tilt > 60° after the settle (also logged: first frame with tilt > 60°, and with tilt > 60° while the grip cmd < 0.3 = FullTaskEnv's tip rule). Stage ladder for flips: none < picked < placed < contact < nested (census order).
- **Determinism / load:** every process pins taichi + torch to 1 CPU thread (`--threads 1`; injected into `ti.init`); smoke uid 232: the 1-thread and default-thread runs are bit-identical (max per-frame can/tool difference 0.0). 8 processes on the idle 12-core box (load 0.6 at launch), 6 codes × 8 uid-shards = 48 jobs, every code runs the same shard uid order (same in-process history per shard).

### Codes

| code | arm (x, y, z at cap 0.11 m/s; pitch at cap 1.0 rad/s) | gripper |
|---|---|---|
| raw | recorded twist | recorded `gripper_pos` |
| tern | per axis: \|v\| ≥ 0.5·cap → ±cap, else 0 | ternary ramp |
| five | per axis: nearest of {0, ±0.5·cap, ±cap} | ternary ramp |
| held | tern, then a word-level mode filter over a 5-frame window (±2 frames; ties keep the centre word) — removes step-to-step jitter; non-causal by 2 frames (offline tape transform) | ternary ramp |
| tern_rawgrip | tern | recorded (isolates the arm code) |
| raw_terngrip | recorded | ternary ramp (isolates the gripper code) |

**Gripper ternary ramp.** This harness has no gripper bit, so the script builds one. Measured on the 74 tapes: the real gripper position ramps at 1.49 (closing) / 1.52 (opening) units per frame (medians of |Δgp| > 0.3) and stops wherever the button is released — closed plateaus median 63, p10 37, p90 88; open plateaus median 12, max 29; 1–3 close cycles per demo. So the operator's gripper command is itself three-valued, {open, hold, close}, driving a fixed-rate ramp. Code: `u_t = sign(gp_t − g_{t−1})` if `|gp_t − g_{t−1}| > 0.75` else 0; `g_t = clip(g_{t−1} + 1.5·u_t, 0, 100)`; `g_0 = gp_0` (the initial state). The ramp tracks the recorded position to within ~1.5 units (max deviation logged per tape).

### Vocabulary (pre-run, no simulation; `vocab.json`)

Over the 74 tapes (177,238 frames): 25.1 % of frames carry a nonzero ternary arm word. **45 distinct arm words** (of 81); the top 8 cover 77.9 %, the **top 16 cover 99.1 %**, 34 words reach 99.9 %. Per uid: median 13 distinct words (max 22). Ternary runs are short (median 4 frames, p90 11); the held filter lengthens them (median 7, p90 16). Top words: +z (13.2 %), +x (12.8 %), +x−y (11.0 %), −z (10.1 %), −y, +y, +x+y, −x−y, −x, −x+y (5.7 %), then the two pitch-only words (3.1 / 2.9 %) and z∓pitch combinations (≤ 1 %); every word past rank 16 is < 0.15 %.

## 2. Registered predictions (written 2026-09-06 23:10, before the runs)

Disclosure: the pipeline smoke ran raw and tern on ONE uid (232) before this was written: raw → placed (can 18 cm from the goal, tipped), tern → contact (6.7 cm, tipped). The predictions below are about the 74-uid net counts and were not adjusted to that uid.

"Cost" = (raw count − code count) over the 74 uids, per stage; a negative cost is a gain.

- **P1 (the brief's):** tern costs ≤ 3 picks and ≤ 3 contacts vs raw.
- **P2 (the brief's):** five costs ≤ 1 pick and ≤ 1 contact vs raw.
- **P3:** the gripper code alone (raw_terngrip vs raw) costs ≤ 1 pick and ≤ 1 contact (the ramp stays within 1.5 units of the recorded position).
- **P4:** held costs ≤ 3 picks and ≤ 3 contacts vs raw (±2 frames = ±50 ms of timing).
- **P5 (noise scale):** per-uid contact flips (either direction) under tern ≤ 10 of 74 — i.e. within ~1.5× the w3 census's 1 mm-IC-jitter noise floor (7 contact flips/uid, CONFOUNDS row 47); nested net cost ≤ 3; tipped net change within ±5.
- **Power caveat (registered):** the raw commanded path may itself sit well below the joint-target path's w3 funnel (picked 69 / placed 66 / contact 26 / nested 16 over the same 74 uids; that path feeds recorded joint targets, not the twist). If raw picks < 50 the deeper stages have little power; the comparison of record stays raw-vs-code on this path, and the joint-path funnel is context only.

**Decision rules.** P1 met → the ternary code is faithful enough to define a discrete-action arm; vocabulary = the 16 words that carry 99.1 % (+ the 3-valued gripper) — report whether tern_rawgrip and raw_terngrip attribute any loss to the arm or the gripper half. P1 failed but P2 met → the analog easing matters physically; the five-level code is the candidate (report its used vocabulary). Both failed → snapping is not faithful on this path; report whether the losses are timing (first-stage frame indices shift) or spatial (settled can position) before proposing anything. P4 vs P1: if held loses more than tern, the jitter is load-bearing (do not de-jitter the tapes); if held loses less, de-jittering is free.

## 3. Results

*(to be filled from `table.md` after the runs — numbers only from tool output)*

## 4. Per-uid flips

## 5. Recommendation
