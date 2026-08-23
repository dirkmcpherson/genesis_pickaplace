# Real→sim translation of the human demos: follower lab + simulator audit (2026-08-23)

Author: Fable session on the laptop (CPU Genesis, `~/workspace/genesis_local`, LOCAL_ENV.md).
All numbers are LOCAL (i7 AVX2, taichi CPU JIT): contact-marginal demos diverge from the
cluster, so this is a ranked list + diagnostics and the cluster confirms (commands in §7).
New, untracked-data-only code (no tracked file edited):
* `baselines/human_follower_lab.py` — follower candidates on top of `record_demos.HumanFollower/Recorder`,
  CLI `--config NAME [--sim VARIANT] [--uids …] [--parallel N [--fresh]] [--trace] [--report]`;
  writes contract-v1 tapes (validate_tape on every one) + `manifest.json` with per-uid
  diagnostics and fidelity to `baselines/demos_v1/_lab/<config>[@<sim>]/` (+`_fails/`).
* `baselines/sim_variants.py` — simulator variants (arm PD gains, gravity compensation, arm effort
  limits, robot mounting height) applied by monkeypatch before/after the world build; the
  exact permanent patch is in its docstring and §7.
* `baselines/sim_fidelity_lab.py` — replays the human's REAL joint stream through a sim variant
  and measures sim-vs-real tracking (per joint RMS/p95/max, static sag, dynamic lag, frames
  beyond cap/leash, error at grip close, hardened pick re-earned); `--offline` = the same from
  the tapes as recorded; `--report`.
Scratchpad helpers (penetration probe, FK check, per-uid table, fidelity-vs-command table)
are in the session scratchpad; their numbers are quoted inline.

## 0. TL;DR

1. **Follower only (MDP and simulator untouched).** Changing the waypoint-arrival test from
   "measured joints within tol of the human's MEASURED pose" to "measured OR the env's
   integrated target within tol of the human's COMMAND" (`--config arr_either`) takes the
   local kept count **49/66 → 55/66**, with identical fidelity to the human (dev vs the real
   arm path p50 0.033 → 0.034 rad, vs the tape's measured path 0.0024 → 0.0022), lower
   dilation (p50 1.042 → 1.026, max 2.41 → 1.65) and zero dwell stalls. Every gain (250, 257,
   267, 298, 330, 333) is a "dwell-stall" demo: the human's command leads the sim-measured
   pose, the follower's arm converges to the command and then sits AHEAD of the measured
   refs it was told to wait for. Cluster baseline 51 → expect ~57.
2. **Idle/slow-time compression ("speed-aware skipping") is NOT a win under the current
   simulator** (rescues the long demos 245/300 but flips marginal grasps 259/283/326: the
   human's idle frames are the time the sagging sim arm needs to creep to its target). Under a
   simulator that tracks (below) it is safe and rescues the long demos (§4.3).
3. **What remains after (1) is intrinsic to the MDP or a defect of the simulator.**
   Intrinsic: 234, 318 (can lying at t0 → tip rule at decision 1). Simulator (§4-5): the
   current Genesis arm cannot execute the real robot's trajectories — (a) no gravity
   compensation + soft PD (static sag p95 0.05 rad, 0.1 rad creep over seconds), (b) PD
   bandwidth: under the 0.125 leash the arm tops out at ~0.75 rad/s, the real arm did
   1.0-1.6 rad/s, (c) **the robot is mounted ≥3 cm too low relative to the table**: the real
   tool frame (bag `tool_pose`, matched by the sim FK to <1 mm) reaches z = −0.019…+0.013 m
   in base coordinates while the sim fingertips bottom out on the table at tool z ≈ +0.03.
   The human's recorded joints therefore push the sim fingers INTO the table; the arm is
   blocked/sags/drags — that is the "pressing" class (295, 299, 254, 256, 286, 298, 330) and
   the 246 "knock-over" (the can tips in the old world, not in the fixed one).
4. **Simulator fix, justified on the real measured joints (66 demos, 39 332 frames):** gravity
   compensation on the arm + kp×4/kv×2 + base raised 3 cm (`--sim gc_kp4_riser3`) reduces the
   sim-vs-real tracking error from **p50 0.047 / p95 0.100 rad to 0.004 / 0.030**, frames
   beyond the leash 7.9 % → 0.1 %, beyond the cap 87 % → 10 %, error at grip close 0.049 →
   0.002; replaying the real joints re-earns the hardened pick on 64/66 (the 2 misses are the
   lying-can ICs). With the follower: **58/66 kept** (`arr_either` or `either_skipg2nr_stat`),
   fidelity to the real arm p50 0.0026 / p95 0.017 rad (13× better than today's 0.034 /
   0.06), remaining drops = 234, 318 + the five 1257-2526-frame demos (time); slow-time compression
   of the human tape (5 mrad, grasp/arm guards) → **61-62/66** fresh-process
   (`either_skips5nr_stat` 61, `either_skips5L_stat` 62; dev vs real arm 0.003/0.02; left:
   234, 318, and 293/295 still time-bound at 1200 sim steps). The record_demos baseline follower is
   INCOMPATIBLE with any sim change (0/66 under `gc_kp4_riser3`): its progress test is tied to
   the old sim's measured states; the command-referenced test is required (or re-collect the
   stride-1 tapes under the new world).
   **This changes the simulator, not the MDP** (obs layout, delta/cap/LEASH semantics, repeat,
   horizon, reward/terminal, ICs, pick_z/goal untouched) — but it forces re-harvesting
   dDP/dR2D and re-running every positive/negative control (teachers were trained in the old
   world); consequences listed in §7.
5. **Gripper clipping the can** (§5.5): the four finger dofs are position-driven to the REAL
   gripper reading mapped linearly onto the URDF joint range; at a secure grasp the real
   reading is 0.83 (p50) while the sim fingers stall on the 66 mm can at 0.61 → the PD (kp 40,
   force range ±50) pushes 0.23 rad into the can ≈ 10 N·m per finger joint, 80-100 N contact
   force and **7-8 mm measured interpenetration** (uid 242 probe). Finger torque cap 2 N·m →
   4.3 mm, kp 10 → 3 mm. Left unchanged in every reported number (grasp-physics retune is a
   separate decision with its own validation).

## 1. Setup, metrics, caveats

* MDP (fixed): FullTaskEnv(scope pick, delta_joint, delta_ref target, cap 0.025 rad/sim-step,
  leash 0.125, action_repeat 4, 1200 sim steps, tip rule, hardened pick), recorder
  `record_demos.Recorder` (contract v1, state-only = `--no-images` equivalent; images via
  `--images`).
* Source tapes: `baselines/episodes_pick_phase_dppruned/*.npz` (66). NB they are SIM replays:
  `actions[:, :6]` = the REAL robot's measured joint positions (`inthewild_trials/<uid>_episodes.npy`
  'vel_cmd' is `/joint_states` position at ~30 Hz, trial_reader.py) used as absolute PD
  targets, 3 physics steps (dt 0.01, 8 substeps) per frame; `states[:, :6]` = the sim arm.
  So the tape's "measured path" is the OLD sim's arm, and the tape's "command path" is the
  real arm.
* Metrics: local kept /66 (primary); dilation = decisions·4/n_src; fraction of decisions at
  the cap (max|a_arm| ≥ 0.999); fidelity of kept tapes = nearest-neighbour L2 joint distance
  of every follower sim step to (i) the human's COMMAND path = the real arm (`dev_cmd`, the
  reference that stays valid when the simulator changes) and (ii) the tape's measured path
  (`dev_meas`); path-length ratio vs the command path; the last 13 sim steps after the
  geometric lift (not in the human tape) are excluded. Numbers: p50/p95 per tape → median
  over kept tapes, plus the max over all.
* Determinism: sequential episodes in one process carry solver residue (uid 242 terminates
  at sim step 238 fresh, 239 after uid 233). Screening = 3 shards of 22; the recommended
  configs were re-run one process per episode (`--parallel 3 --fresh`, §7). Contact-marginal
  demos (233, 283, 326, 330, 308/309 stalls) flip between machines — read aggregates.
* Timing: ~13 min per 66-demo config on 3 cores (build 25 s, 46-55 ms/sim step).

## 2. Baseline reproduction and failure taxonomy (current simulator)

`--config baseline` is bit-identical to `record_demos.py --teacher human` (uid 242: 60
decisions / 238 sim steps / dil 1.034, actions_delta equal). Local: **49/66 kept** (cluster
51): the cluster's 15 + 233 and 330 (contact-marginal: 233's gripper closes 1 cm off and
pushes the can 4 cm; 330 stalls 15× locally vs 4× on the cluster).

| class | uids (local baseline) | mechanism (per-decision traces, `--trace`) | intrinsic to the MDP? |
|---|---|---|---|
| IC-bound (can lying at t0, tilt 90°) | 234, 318 | tip rule fires at decision 1 (grip open ∧ tilt>60°) | YES |
| knock-over | 246 | follower on-path (‖q_f−q_h‖∞<0.003) reproduces the tape's own knock at frame ~427 (tape: tilt 46°→124°, lying pick); tip rule at 60° | only in the CURRENT world: under `gc_kp4_riser3` the can does not tip (§4.3) |
| dwell stalls, "follower ahead" | 250, 256, 257, 267, 298, 330, 333 (+308/309 kept at dil 2.4/1.6) | the human's command leads the sim-measured pose by 0.1-0.3 rad; the target reaches the command, the arm converges there = AHEAD of the measured refs the arrival test waits for → 8-decision dwell, stall, jump, repeat (20-30 stalls × 8 decisions) | NO — follower (`arr_either`) |
| blocked arm / pressing | 295, 299 (+ residual stalls in 254, 256, 286, 298) | target clamped at the leash (lag 0.125), arm does not move, command 0.2-0.43 rad "inside" the table on joints 1/2 | simulator (§4.1c) |
| horizon | 245, 286, 293, 300 (1257-1990 frames, idle 20-72 %) | real-time following cannot fit; T_path (command path at cap speed) is only 240-330 frames → idle-bound, not motion-bound | follower (idle compression) AND simulator (arm creep makes idle frames load-bearing) |
| contact-marginal | 233 | gripper 0.01 rad off at close, can pushed 4 cm | cross-machine FP |

Per-uid source diagnostics (frames, idle fraction, T_path, windows over cap, frames with the sim
arm > leash behind the real arm, lift frame) and the outcome per config: Appendix A.

## 3. Follower candidates (simulator = current)

All candidates translate the human's own command stream cmd_j / measured refs ref_j / grip_j
(no replanning, no reward, no can state; contract-v1 tapes). Knobs (docstring of
`human_follower_lab.py`): arrival ∈ {meas, target, either, passed}, aim ∈ {window,
lookahead, reach}, skip_tol/skip_ref/skip_grip/skip_guard/skip_dq/grip_settle/effort_settle
(idle compression + guards), stat_eps (stationary-arm fast stall), tol/max_dwell/dilation_cap.

| config | change vs baseline | kept | dil p50 / max | at-cap % | dev_cmd p50 / p95 / max (rad) | dev_meas p50 | path ratio (cmd) | dropped |
|---|---|---|---|---|---|---|---|---|
| baseline | — | **49** | 1.042 / 2.41 | 5.0 | 0.0328 / 0.052 / 0.209 | 0.0024 | 0.890 | 233 234 245 246 250 256 257 267 286 293 295 298 299 300 318 330 333 |
| **arr_either** | advance k if ‖q−ref_k‖∞<tol OR ‖target−cmd_k‖∞<tol | **55** | 1.026 / 1.65 | 6.5 | 0.0338 / 0.064 / 0.255 | 0.0022 | 0.897 | 233 234 245 246 256 286 293 295 299 300 318 |
| arr_passed | meas OR "passed the plane through ref_k" within 0.075 L2 | 55 | 1.027 / 1.42 | 7.1 | 0.0338 / 0.062 / 0.282 | 0.0023 | 0.896 | same set |
| skip_idle | + skip waypoints whose cmd is within 1e-3 of the target (idle compression) | 50 | 0.883 / 1.44 | 6.2 | 0.0337 / 0.056 / 0.211 | 0.0039 | 0.878 | loses 283 326; gains 233 245 330 |
| either_skip_stat | either + idle skip + stationary-arm fast stall | 55 | 0.853 / 1.53 | 7.9 | 0.0345 / 0.062 / 0.254 | 0.0039 | 0.879 | 234 246 256 259 283 286 293 295 299 318 326 |
| either_skipg_stat | … + grasp guard (no skip within 45 frames of a grip change; grip pos/effort settled) | 53 | 0.968 / 1.65 | 7.0 | 0.0341 / 0.062 / 0.255 | 0.0032 | 0.885 | 233 234 246 256 259 283 286 293 295 299 300 318 326 |
| either_skipg2_stat | … + no skip while the arm still moves (‖dq‖∞>0.002/decision) | 54 | 1.016 / 1.65 | 6.8 | 0.0338 / 0.063 / 0.255 | 0.0024 | 0.889 | 233 234 245 246 256 283 286 293 295 299 300 318 |

Reading:
* `arr_either` is the recommended follower for the CURRENT world: +6 kept, all dwell-stall
  demos; dilation down; fidelity unchanged (the 8-decision freezes let the arm drift to the
  command anyway, so nothing in the executed path changes — only the waiting goes away).
  `arr_passed` is equivalent here; its loose L2 radius costs max deviation.
* Idle compression is fragile in the current world: 259/283/326 flip even with all guards
  (28 skipped frames on 283 change a 0.026→0.029 m can nudge into a miss) because the
  sagging arm creeps 0.1 rad during the human's idle frames (trace: lag 0.10-0.12 while
  "idle"). The `skip_dq` guard turns compression off almost entirely (dil p50 1.016) and
  still loses 283. Not recommended in the current world.
* Dead ends: lookahead/reach aiming (the stalls are not a speed problem — the arm is either
  blocked or saturating), tol/dwell schedules (tol 0.035-0.05 lets the arm stray; dwell 4/16
  only reprices the stall), dilation cap 6 (257/298 then hit 1200 instead), `stat_eps` (never
  triggers once `arr_either` is on).
* What the cluster should see: baseline 51 → `arr_either` ≈ 57 (the six stall demos + its own
  233/330).

## 4. Simulator: does Genesis execute the real robot's trajectory?

### 4.1 'Before' numbers and the three causes
Because the tapes replay the REAL measured joints as PD targets, e_j = cmd_j − q_sim,j+1 IS the
real→sim discrepancy of the arm. Over 66 tapes / 39 332 frames (`sim_fidelity_lab.py --offline`):

| metric (per tape → median over tapes) | current sim |
|---|---|
| ‖e‖∞ p50 / p95 | 0.047 / 0.100 rad |
| frames with ‖e‖∞ > cap (0.025) / > leash (0.125) | 87 % / 7.9 % (295: 76 %, 299: 48 %, 298: 42 %, 254: 41 %) |
| RMS per joint | 0.029 0.044 0.048 0.028 0.036 0.013 |
| static sag p95 (command still ≥ 15 frames) | 0.018 0.038 0.054 0.014 0.029 0.004 |
| dynamic lag p95 | 0.064 0.075 0.087 0.077 0.073 0.039 |
| ‖e‖∞ at the grip-close frame | 0.049 rad |

**(a) No gravity compensation, soft PD.** `build_world` adds the Kinova URDF with no material
(Genesis default `gravity_compensation=0`) and kp [200,200,150,100,60,60], kv [20,20,15,10,6,6];
Genesis' position control is τ = kp·e − kv·q̇ clamped to the force range (rigid_solver_decomp.py
`_func_compute_control_force` / line 4585). Steady-state sag = τ_g/kp: the shoulder (dof 1)
carries 8-13 N·m extended → 0.04-0.066 rad (uid 267: constant −0.066 while stationary, and
the arm creeps for seconds toward the command). The real arm has zero steady-state error.

**(b) Bandwidth.** Under the leash the PD error cannot exceed 0.125 → steady speed
≈ (kp·0.125 − τ_g)/kv ≈ 0.75 rad/s on the big joints. Real joints: p99 0.6-1.35 rad/s, 5.1 % of
frames above the cap (0.75 rad/s), 1.8 % above 1.0 (Kinova's own limit: 1.0 rad/s joints 1-5,
1.57 joint 6, user guide Table 26). Trace (299, `arr_either`): lag pinned at 0.124 for 100+
decisions with |a| < 1.

**(c) Robot mounting height.** Sim: Kinova base at world z 0.05 = table top (`TABLE_TOP_Z`,
"the robot's mounting surface"). Real: bag `tool_pose` z (base frame) reaches −0.019 (286),
+0.004 (295), +0.007 (299), +0.008 (330), +0.009 (254) — exactly the pressing demos. The sim
FK reproduces those real tool poses to < 1 mm (`GenesisCanEnv.tool_pos_for(real joints)` vs
`tool_pose`, 8 demos), but the sim finger hulls hit the table at tool z ≈ +0.03 (eef_pos z
0.031-0.033 whenever the arm is "blocked"; the finger collision hull reaches ~1.5-2 cm below
the tool point, start-pose measurement). So the real work surface is ≥ 3 cm below the robot's base plane (mounting
plate/clamp), the humans grasped the can mid-height, and the current world turns their joint
trajectories into fingers pressed into the table: persistent 0.15-0.5 rad "lead" on joints
1/2 in 295/299/254/298/286/330, dragging friction, the 246 knock, and bottom-rim-looking
grasps in every replay video.

### 4.2 Sim variants (`sim_variants.py`) — real joint stream replayed through each (66 demos)

| variant | change | ‖e‖∞ p50 / p95 | >leash | >cap | RMS per joint | sag p95 per joint | pick re-earned | ‖e‖∞ @grip close |
|---|---|---|---|---|---|---|---|---|
| current (offline) | — | 0.047 / 0.100 | 7.9 % | 87 % | .029 .044 .048 .028 .036 .013 | .018 .038 .054 .014 .029 .004 | 66/66 (tape label) | 0.049 |
| gc | gravity comp 1.0 | 0.032 / 0.085 | 4.5 % | 65 % | .025 .036 .037 .027 .026 .013 | .009 .039 .035 .007 .013 .003 | 64/66 | 0.038 |
| kp4 | kp×4, kv×2 | 0.026 / 0.056 | 4.6 % | 47 % | .015 .033 .032 .013 .020 .006 | .005 .039 .019 .004 .007 .001 | 63/66 | 0.034 |
| gc_kp4 | both | 0.024 / 0.056 | 3.6 % | 44 % | .013 .032 .028 .012 .017 .006 | .003 .039 .016 .003 .005 .001 | 64/66 (299, 309) | 0.035 |
| riser3 | base +3 cm only | 0.035 / 0.078 | 0.4 % | 90 % | .019 .038 .018 .025 .020 .013 | .002 .036 .007 .003 .010 .003 | 63/66 (316 318 335) | 0.034 |
| **gc_kp4_riser3** | all three | **0.004 / 0.030** | **0.1 %** | **10 %** | .008 .007 .008 .011 .008 .006 | .002 .006 .009 .002 .003 .001 | **64/66** (234, 318 = lying ICs) | **0.002** |
| gc_kp4_riser4 | base +4 cm | 0.004 / 0.029 | 0.1 % | 9.7 % | .008 .007 .008 .011 .008 .006 | .001 .002 .001 .001 .003 .001 | 63/66 (234, 318, 316: can pushed 5.5 cm on approach) | 0.001 |

The riser alone removes the blocked-arm class (no uid with shoulder sag p95 > 0.06 vs 19 uids
without it; over-leash frames 7.9 % → 0.4 %); gravity comp + gains remove sag/creep and the
bandwidth limit; together the sim arm follows the real arm to 0.004 rad median / 0.030 p95
and every non-lying demo re-earns the hardened pick from its real joint stream. The halves
interact (gc_kp4 alone misses 299/309, riser3 alone misses 316/335 by pushing the can 5 cm on
approach). Residual over-leash frames under gc_kp4_riser3: 286 (2.9 %), 295 (2.5 %) — the two
demos whose real tool went lowest (−0.019 / +0.004), i.e. 3 cm is a lower bound; riser4 clears 295 and 299 and leaves 286 at 2.3 % — but 316's recovered x,y placement (fitted in the old world) then has the fingers push the can 5.5 cm on approach. riser3 is the conservative recommendation; the true mounting offset is 3-4 cm.

### 4.3 Follower × simulator (kept /66; fidelity = dev vs the REAL arm path)

| follower \ sim | current | gc_kp4 | riser3 | gc_kp4_riser3 | gc_kp4_riser4 |
|---|---|---|---|---|---|
| baseline (record_demos) | 49 (dev 0.033/0.052) | — | — | **0** (every window stalls: waits for old-sim measured poses) | — |
| arr_either | 55 (0.034/0.064) | — | — | 58 (0.0026/0.017); drops 234 318 245 246 286 293 295 300 | — |
| either_skipg2nr_stat (idle skip w/ guards, no ref test) | — | 56 (0.017/0.047); drops 234 318 245 246 247 286 293 295 299 300 | 57 (0.034/0.040); drops 234 318 245 246 286 293 295 300 316 | 58 (0.0028/0.017); drops 234 318 245 286 293 295 300 316 | — |
| either_skips5nr_stat (slow-time compression, 5 mrad, skip_dq 0.002) | — | — | — | 60 sharded / **61 fresh** (0.0028/0.018, max 0.127; dil p50 0.92, max 1.03; at-cap 5.4 %); drops 234 318 286 293 295 (+300 sharded) | 59 (0.0027/0.017; 316 lost: old-world placement) |
| either_skips5L_stat (as above, gated on arm convergence ‖target−q‖∞ ≤ 0.01 instead of arm velocity) | — | — | — | **62 fresh** (0.0034/0.020, max 0.136; dil p50 0.85, max 1.00; at-cap 6.0 %); drops 234 318 293 295 | — |

Notes: under `gc_kp4_riser3` 246 is no longer knocked over (kept by skipg2nr, truncated by
arr_either — its tape continues as a lying-can re-grasp that no longer matches the world); the
five long demos are now purely time-bound (245 reaches waypoint 1947/1990 at sim step 1200 with
dilation 0.60: the follower runs at real time and the human took 66 s). `either_skipg2nr_stat`
barely compresses them (the human's "idle" there is a 1-2 mrad/frame creep and long grip
fiddling that the guards protect); `either_skips5nr_stat` compresses command motion below
5 mrad/frame (path shape kept to 5 mrad; dil p50 0.913) and rescues 245 (reaches 1986/1990, dil 0.55), 246 (no knock in this world) and 316 → **60/66**; 300 (1264/1300), 286 (1130/1257), 293 (1387/1840), 295 (1159/2526) still end at sim step 1200 because the `skip_dq ≤ 0.002/decision` guard (needed in the old world against arm creep) also vetoes slow-motion stretches (the arm moves ≈0.02 rad/decision there); 38-78 % of their frames are compressible under the grip/effort guards. Gating on arm convergence instead (`either_skips5L_stat`: skip only while ‖target−q‖∞ ≤ 0.01) is the natural form for a tracking simulator — fresh-process result: **62/66** (`FRESH_either_skips5L_stat@gc_kp4_riser3`: drops 234, 318, 293 (reaches 1559/1840), 295 (979/2526, the residual table contact blocks compression); dev vs the real arm p50 0.0034 / p95 0.020 / max 0.136; dil p50 0.847, max 1.00; at-cap 6.0 %; path ratio 0.876). The non-compressing `arr_either@gc_kp4_riser3` (58) and the skip_dq variant (`FRESH_either_skips5nr_stat@gc_kp4_riser3`, **61/66**, 0.0028/0.018, dil 0.923) are the conservative alternatives.

## 5. World setup audit (code vs. specs/docs/real data)

| item | code (build_world / world_cfg) | spec / evidence | verdict |
|---|---|---|---|
| arm model | `gen3_lite_2f_robotiq_85.urdf` = kortex_description xacro output `gen3_lite_gen3_lite_2f` (meshes arms/gen3_lite + grippers/gen3_lite_2f); misnamed — it is NOT a Robotiq 2F-85 | link masses Σ 5.2 kg (spec 5.4 kg); joint limits ±2.53-2.68 rad (spec ±145-160°); URDF effort 10/14/10/7/7/7 N·m, velocity 1.6 (j6 3.2) rad/s | OK; Kinova soft torque limit 9 N·m/joint, speed 1.0 rad/s (j1-5) / 1.57 (j6) — Gen3 lite user guide Tables 26-28 (kinovarobotics.com, generation-robots mirror); Genesis ignores URDF velocity limits |
| arm effort limits | force range overridden to ±50/50/50/20/20/20 N·m | URDF 10/14/10/7/7/7; Kinova 9 | 3-5× too strong; not what limits tracking (PD saturates at kp·leash = 25 N·m) but lets the arm press the table at 50 N·m; `gc_kp4_urdf` variant available |
| arm control | pure PD per dof, no gravity comp, kp 200…60, kv 20…6; ZOH target at 30 Hz, 3×10 ms physics steps (8 substeps → 1.25 ms) | real: 1 kHz position servo, measured = command | defects (a)+(b): sag p95 0.05 rad, 0.75 rad/s ceiling → `gc_kp4` |
| robot mounting height | URDF at world z 0.05 = table top | real tool z down to −0.019 … +0.013 m (base frame); sim fingers bottom out at +0.03 | defect (c): raise the base ≥ 3 cm (`riser3/4`); the can/table/pick_z/ICs stay (the "cans sat on the robot's own table" note in build_world was right about the can, wrong about the mounting plane) |
| table | fixed box, top z 0.05, friction 0.5, 0.419×1.2 m | — | keep; move the robot |
| shelf box | dynamic rigid box 0.4×0.75×0.12, rho 1000 (36 kg) | — | irrelevant for pick scope |
| can | cylinder r 0.033, h 0.101, rho 1000 → 0.345 kg, friction 0.2 | 10.5 oz soup can 66 × 101 mm, ≈0.35 kg | OK. Contact friction = max(a, b) (`collider_decomp.py:1179`); URDF fingers default 1.0 → finger-can friction IS 1.0 (the 0.2 only acts vs the table, 0.5) |
| gripper | 4 finger dofs driven independently (Genesis 0.2.1: no mimic); target = REAL reading 0..100 mapped linearly onto driver −0.09..0.96 rad, tip −0.676θ+0.149; kp 40, kv 10, force ±50; collision = one convex hull per finger mesh (`decompose_robot_error_threshold = inf`) | real Gen3 lite 2F: linear actuator, reading 0-100, force not published; real reading at a secure grasp p50 0.83 (p10-p90 0.68-0.90) vs sim stall 0.61 (0.56-0.64) on the 66 mm can → linear map wrong by ~0.23 rad at the grasp | defect: PD pushes 0.23 rad into the can ≈ 10 N·m per finger (~200 N at the pad), 80-100 N contact force, 7.9 mm interpenetration (uid 242 probe); 2 N·m cap → 4.3 mm, kp 10 → 3.0 mm; 5 N·m → 7.3 mm (the kp term, not the cap, sets the squeeze) |
| contacts | MuJoCo-style soft constraints: timeconst = 2·substep_dt = 2.5 ms, dampratio 1, dmin/dmax 0.9/0.95 (`geom.py default_solver_params`) | — | penetration ∝ force; the squeeze is the lever |
| cameras | rig 64×64 top + wrist (attached to eef) | — | unaffected by the riser |

### 5.5 Gripper proposal (NOT applied, no number in this report depends on it)
Either cap the finger driver torque at ~2 N·m (≈40-50 N squeeze) or calibrate the
reading→angle map on the real stall (0.83 ↔ 66 mm can). Both cut the clipping to ≤ 4 mm but
change grasp physics (world_cfg finger_kp 40 / finger_force 50 were tuned for pickup in the old
world); validate with the raw replay (§4.2 "pick re-earned" column) before adopting.

## 6. What is intrinsic to the MDP (numbers)

* Tip rule at t0: 2/66 (234, 318; can tilt 90° at frame 0). Unrecoverable (decision 1).
* Tip rule mid-episode: 0/66 once the simulator is fixed (246's knock is a table-height
  artifact); 1/66 in the current world.
* Cap 0.025 rad/sim-step (= 0.75 rad/s): the real joints exceed it on 5.1 % of frames; the
  command path at cap speed (T_path) is 49-424 frames per tape, so the cap never needs more
  than the horizon; it costs dilation ≤ 1.3 on the fastest demos once the arm tracks;
  decisions at the cap in kept tapes 5-8 %.
* Horizon 1200: no demo is motion-bound (T_path ≤ 424); the 5 long demos (1257-2526 frames,
  idle 20-76 %) are time-bound: they fit only with idle/slow-time compression of the human's
  own tape (a translation choice, not an MDP limit).
* Leash 0.125: binds on 7.9 % of frames in the current sim, 0.1 % with `gc_kp4_riser3` — no
  proposal to change it.
* Ceiling for a perfect translation in the fixed world: 64/66 locally.

## 7. Recommendation, cluster commands, exact patch, consequences

1. Immediately (no sim change): record dH with `arr_either` (keeps the pipeline, +6 locally,
   expected ~57 on the cluster). Fresh-process verification: **55/66, same kept set, same
   fidelity** (`FRESH_arr_either`: dev_cmd 0.034/0.064, dil p50 1.027).
2. With the simulator fix (`gc_kp4_riser3`; riser4 if the cluster confirms the residual 286/295
   leash violations and re-fits 316's placement): `either_skips5L_stat` (**62/66 fresh**, dev
   vs real arm 0.0034/0.020, dil p50 0.85 — slow stretches of the human tape are compressed, path
   shape kept to 5 mrad) or, if the time profile must stay untouched, `arr_either` (58/66,
   0.0026/0.017, dil 0.99). THEN
   re-collect the stride-1 human tapes under the new world (the collector replays the same real
   joint streams; `sim_fidelity_lab.py` is exactly that replay), because every consumer of
   `states` (BC/DP on stride-1 tapes, dHunpruned control) otherwise trains on old-sim states.

```
# follower only (current world) — cluster confirmation, fresh process per episode
python baselines/human_follower_lab.py --config arr_either --parallel 3 --fresh
# follower + simulator
python baselines/human_follower_lab.py --config either_skips5L_stat --sim gc_kp4_riser3 --parallel 3 --fresh
python baselines/human_follower_lab.py --config arr_either        --sim gc_kp4_riser3 --parallel 3 --fresh
# re-checks with traces
python baselines/human_follower_lab.py --config arr_either --uids 250 257 267 298 330 333 --trace
# simulator fidelity (real joint stream through a variant) + table
python baselines/sim_fidelity_lab.py --variant gc_kp4_riser3 --parallel 3 ; python baselines/sim_fidelity_lab.py --report
python baselines/human_follower_lab.py --report
```
Outputs: `baselines/demos_v1/_lab/<config>[@<sim>]/` (+`_fails/`), `manifest.json` (per-uid
`src_diag`, `fid`, dilation, stalls, skipped, frac_cap, sim_applied).

Exact simulator patch (what `sim_variants.install/post_build` does at runtime; permanent form
for `can_pos_recovery/replay_harness.py::build_world`, defaults = today's world byte-identical):
```
 def build_world(..., rig_res=64,
+                arm_riser=0.0, arm_gravity_comp=0.0,
+                arm_kp=(200, 200, 150, 100, 60, 60), arm_kv=(20, 20, 15, 10, 6, 6)):
-    kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / urdf_file), fixed=True, pos=(0.0, 0.0, 0.05), **(urdf_extra or {})))
+    kinova = scene.add_entity(gs.morphs.URDF(file=str(REPO / urdf_file), fixed=True,
+                                             pos=(0.0, 0.0, 0.05 + arm_riser), **(urdf_extra or {})),
+                              material=gs.materials.Rigid(gravity_compensation=arm_gravity_comp))
-    kinova.set_dofs_kp(kp=np.array([200, 200, 150, 100, 60, 60] + fkp), dofs_idx_local=kdofs)
-    kinova.set_dofs_kv(kv=np.array([20, 20, 15, 10, 6, 6] + fkv), dofs_idx_local=kdofs)
+    kinova.set_dofs_kp(kp=np.array(list(arm_kp) + fkp), dofs_idx_local=kdofs)
+    kinova.set_dofs_kv(kv=np.array(list(arm_kv) + fkv), dofs_idx_local=kdofs)
```
`gc_kp4_riser3` = arm_riser 0.03, arm_gravity_comp 1.0, arm_kp (800,800,600,400,240,240),
arm_kv (40,40,30,20,12,12); force range unchanged (±50/50/50/20/20/20). `GenesisCanEnv.__init__`
would read the same four keys from `trial_placements.json['world']`.
`python -c "import sys; sys.path.insert(0,'baselines'); import sim_variants; print(sim_variants.describe('gc_kp4_riser3'))"`.

**Consequences (written down as required):** any sim change forces (1) re-harvesting dDP and
dR2D through the new world — the DP-r4 and r2dreamer teachers were trained in the old one,
their competence must be re-measured (PREREG §3.1) — and re-running the positive controls and
the random negative control; (2) re-collecting the stride-1 human tapes (old-sim `states`);
(3) re-validating the can-placement recovery: x,y were fitted under the old height; the raw
replay re-earns 64/66 under `gc_kp4_riser3`, so they hold for the pick scope, but the 12
tape-pose ICs and the place-scope entry bank are old-world states; (4) the dense shaping
potential mixes `tool_pos` (base-frame numbers) with the can's world z — a 5 cm offset today,
8 cm with the riser: harmless for the sparse pick arms, flag for the dense arms; (5) the
`record_demos.py` human adapter must switch to the command-referenced arrival test (it scores
0/66 in the new world).

## 8. Notes, dead ends, hooks

* `record_demos.py` hooks that would make this cleaner (not added): `HumanFollower` split into
  `aim()/arrived()`; `Recorder.run(on_decision=cb)` for traces; `build_env(sim_variant=…)`.
* Lookahead/reach aiming not run at scale (trace analysis: stalls are not a speed problem).
* 'target' alone vs 'either': target-only is the purest "replay the command stream rate-limited
  by cap+leash"; 'either' also advances on measured arrival; identical whenever the sim tracks.
* Penetration probe, finger force sweep, FK/tool-frame check, real speed statistics, per-uid
  table and dev-vs-command table: scratchpad scripts `pen_probe.py`, `fid_cmd.py`, `peruid.py`.
* Kinova sources: Gen3 lite user guide (Tables 16, 26-28: weight 5.4 kg, payload 0.5 kg,
  25 cm/s high-level Cartesian cap, joint speed 57.3°/s j1-5 / 90°/s j6, acceleration limits,
  9 N·m soft torque limit, 1 kHz loop); gripper force is not published (web claims of a
  Robotiq 2F-85 on the lite are wrong — the lite has the embedded 2F gripper).

## Appendix A — per-uid table
Columns: source frames n, idle fraction (|Δcmd| < 1e-3), T_path (command path length at cap
speed, frames), fraction of 4-frame windows over 4·cap, fraction of frames with the sim arm
> leash behind the real arm (current sim), human lift frame; then outcome · dilation · stalls
per config (KEPT / trunc = 1200 sim steps / nopick = path exhausted without the hardened pick /
tip).

| uid | n | idle | T_path | win>cap | lead>leash | lift@ | baseline | arr_either | either_skipg2_stat | arr_either@gc_kp4_riser3 | FRESH_either_skips5nr_stat@gc_kp4_riser3 | FRESH_either_skips5L_stat@gc_kp4_riser3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 232 | 233 | 0.24 | 79 | 0.05 | 0.00 | 231 | KEPT d1.05 s0 | KEPT d1.05 s0 | KEPT d1.05 s0 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.00 s0 |
| 233 | 256 | 0.29 | 89 | 0.03 | 0.23 | 254 | nopick d2.17 s7 | nopick d1.62 s2 | nopick d1.62 s2 | KEPT d0.97 s0 | KEPT d0.97 s0 | KEPT d0.95 s0 |
| 234 | 353 | 0.17 | 153 | 0.07 | 0.25 | 351 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 |
| 235 | 189 | 0.22 | 71 | 0.03 | 0.02 | 187 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d0.97 s0 | KEPT d0.97 s0 | KEPT d0.93 s0 |
| 242 | 232 | 0.31 | 98 | 0.04 | 0.00 | 230 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.00 s0 | KEPT d0.95 s0 | KEPT d0.88 s0 |
| 243 | 172 | 0.45 | 71 | 0.12 | 0.05 | 170 | KEPT d1.21 s1 | KEPT d1.07 s0 | KEPT d1.07 s0 | KEPT d1.02 s0 | KEPT d0.98 s0 | KEPT d0.93 s0 |
| 245 | 1990 | 0.72 | 241 | 0.02 | 0.00 | 1988 | trunc d0.60 s5 | trunc d0.60 s0 | trunc d0.60 s0 | trunc d0.60 s0 | KEPT d0.56 s0 | KEPT d0.49 s0 |
| 246 | 1556 | 0.68 | 207 | 0.02 | 0.00 | 1554 | tip d0.32 s0 | tip d0.32 s0 | tip d0.23 s0 | trunc d0.77 s0 | KEPT d0.73 s0 | KEPT d0.68 s0 |
| 247 | 440 | 0.47 | 93 | 0.01 | 0.01 | 438 | KEPT d1.05 s0 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.99 s0 | KEPT d0.80 s0 | KEPT d0.74 s0 |
| 248 | 481 | 0.56 | 79 | 0.02 | 0.00 | 479 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.90 s0 | KEPT d0.99 s0 | KEPT d0.78 s0 | KEPT d0.83 s0 |
| 250 | 1092 | 0.31 | 424 | 0.13 | 0.09 | 1090 | trunc d1.10 s19 | KEPT d1.04 s0 | KEPT d1.03 s0 | KEPT d0.90 s0 | KEPT d0.82 s0 | KEPT d0.76 s0 |
| 251 | 542 | 0.69 | 69 | 0.00 | 0.00 | 540 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.90 s0 | KEPT d0.98 s0 | KEPT d0.78 s0 | KEPT d0.63 s0 |
| 252 | 655 | 0.73 | 84 | 0.02 | 0.16 | 653 | KEPT d1.17 s2 | KEPT d1.17 s1 | KEPT d1.00 s1 | KEPT d0.99 s0 | KEPT d0.56 s0 | KEPT d0.48 s0 |
| 254 | 524 | 0.38 | 171 | 0.07 | 0.41 | 522 | KEPT d1.28 s1 | KEPT d1.24 s0 | KEPT d1.24 s0 | KEPT d1.00 s0 | KEPT d0.95 s0 | KEPT d0.97 s0 |
| 256 | 616 | 0.32 | 240 | 0.07 | 0.16 | 614 | trunc d1.95 s29 | nopick d1.41 s3 | nopick d1.39 s3 | KEPT d0.85 s0 | KEPT d0.83 s0 | KEPT d0.83 s0 |
| 257 | 346 | 0.40 | 99 | 0.06 | 0.31 | 344 | nopick d3.01 s26 | KEPT d1.35 s1 | KEPT d1.35 s1 | KEPT d0.99 s0 | KEPT d0.90 s0 | KEPT d0.83 s0 |
| 258 | 278 | 0.37 | 98 | 0.10 | 0.16 | 276 | KEPT d1.15 s1 | KEPT d1.06 s0 | KEPT d1.06 s0 | KEPT d1.01 s0 | KEPT d0.99 s0 | KEPT d0.94 s0 |
| 259 | 367 | 0.42 | 119 | 0.08 | 0.00 | 365 | KEPT d1.19 s2 | KEPT d1.05 s0 | KEPT d1.05 s0 | KEPT d1.01 s0 | KEPT d0.96 s0 | KEPT d0.91 s0 |
| 262 | 238 | 0.48 | 61 | 0.03 | 0.19 | 236 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d1.02 s0 | KEPT d0.97 s0 | KEPT d0.96 s0 | KEPT d0.91 s0 |
| 263 | 409 | 0.31 | 144 | 0.06 | 0.00 | 407 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.02 s0 | KEPT d1.03 s0 | KEPT d0.98 s0 | KEPT d0.89 s0 |
| 265 | 801 | 0.63 | 86 | 0.00 | 0.00 | 799 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.98 s0 | KEPT d0.99 s0 | KEPT d0.75 s0 | KEPT d0.76 s0 |
| 266 | 566 | 0.45 | 78 | 0.00 | 0.00 | 564 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.99 s0 | KEPT d0.99 s0 | KEPT d0.94 s0 | KEPT d0.83 s0 |
| 267 | 646 | 0.38 | 160 | 0.04 | 0.15 | 644 | trunc d1.86 s29 | KEPT d1.16 s0 | KEPT d1.16 s0 | KEPT d1.00 s0 | KEPT d0.92 s0 | KEPT d0.86 s0 |
| 269 | 459 | 0.54 | 103 | 0.01 | 0.00 | 457 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.95 s0 | KEPT d1.00 s0 | KEPT d0.82 s0 | KEPT d0.72 s0 |
| 273 | 402 | 0.59 | 81 | 0.03 | 0.10 | 400 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.94 s0 | KEPT d0.97 s0 | KEPT d0.64 s0 | KEPT d0.55 s0 |
| 274 | 476 | 0.36 | 179 | 0.11 | 0.06 | 474 | KEPT d1.27 s4 | KEPT d1.05 s0 | KEPT d1.05 s0 | KEPT d1.02 s0 | KEPT d0.92 s0 | KEPT d0.89 s0 |
| 275 | 421 | 0.33 | 125 | 0.02 | 0.00 | 419 | KEPT d1.03 s0 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.99 s0 | KEPT d0.97 s0 | KEPT d0.85 s0 |
| 276 | 601 | 0.55 | 95 | 0.01 | 0.00 | 599 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.83 s0 | KEPT d0.99 s0 | KEPT d0.80 s0 | KEPT d0.74 s0 |
| 277 | 282 | 0.42 | 65 | 0.03 | 0.00 | 280 | KEPT d1.14 s1 | KEPT d1.05 s0 | KEPT d1.05 s0 | KEPT d0.99 s0 | KEPT d0.98 s0 | KEPT d0.88 s0 |
| 278 | 263 | 0.39 | 88 | 0.08 | 0.05 | 261 | KEPT d1.26 s2 | KEPT d1.06 s0 | KEPT d1.06 s0 | KEPT d0.99 s0 | KEPT d0.99 s0 | KEPT d0.97 s0 |
| 279 | 856 | 0.73 | 70 | 0.00 | 0.00 | 854 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.78 s0 | KEPT d0.99 s0 | KEPT d0.69 s0 | KEPT d0.65 s0 |
| 280 | 228 | 0.48 | 77 | 0.07 | 0.00 | 226 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.02 s0 | KEPT d0.98 s0 | KEPT d0.96 s0 | KEPT d0.88 s0 |
| 281 | 464 | 0.59 | 121 | 0.06 | 0.00 | 462 | KEPT d1.14 s2 | KEPT d1.03 s0 | KEPT d0.87 s0 | KEPT d1.00 s0 | KEPT d0.85 s0 | KEPT d0.82 s0 |
| 283 | 419 | 0.42 | 89 | 0.02 | 0.02 | 417 | KEPT d1.02 s0 | KEPT d1.02 s0 | nopick d1.17 s0 | KEPT d0.95 s0 | KEPT d0.81 s0 | KEPT d0.70 s0 |
| 284 | 339 | 0.44 | 54 | 0.00 | 0.36 | 337 | KEPT d1.12 s0 | KEPT d1.12 s0 | KEPT d1.12 s0 | KEPT d0.97 s0 | KEPT d0.94 s0 | KEPT d0.84 s0 |
| 286 | 1257 | 0.20 | 299 | 0.01 | 0.06 | 1255 | trunc d0.95 s1 | trunc d0.95 s1 | trunc d0.95 s1 | trunc d0.95 s2 | trunc d0.95 s0 | KEPT d0.94 s2 |
| 287 | 797 | 0.51 | 155 | 0.01 | 0.00 | 795 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.87 s0 | KEPT d1.00 s0 | KEPT d0.88 s0 | KEPT d0.89 s0 |
| 293 | 1840 | 0.56 | 414 | 0.05 | 0.00 | 1838 | trunc d0.65 s7 | trunc d0.65 s0 | trunc d0.65 s0 | trunc d0.65 s0 | trunc d0.65 s0 | trunc d0.65 s0 |
| 295 | 2526 | 0.76 | 288 | 0.03 | 0.76 | 2524 | trunc d0.47 s33 | trunc d0.47 s32 | trunc d0.47 s30 | trunc d0.47 s2 | trunc d0.47 s2 | trunc d0.47 s2 |
| 297 | 344 | 0.47 | 76 | 0.00 | 0.03 | 342 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.98 s0 | KEPT d0.98 s0 | KEPT d0.94 s0 |
| 298 | 305 | 0.50 | 75 | 0.03 | 0.42 | 303 | nopick d3.00 s26 | KEPT d1.65 s5 | KEPT d1.65 s5 | KEPT d0.98 s0 | KEPT d0.96 s0 | KEPT d0.88 s0 |
| 299 | 763 | 0.32 | 342 | 0.17 | 0.48 | 761 | trunc d1.57 s26 | trunc d1.57 s22 | trunc d1.57 s19 | KEPT d1.04 s0 | KEPT d0.95 s0 | KEPT d0.91 s0 |
| 300 | 1300 | 0.48 | 335 | 0.07 | 0.02 | 1298 | trunc d0.92 s9 | trunc d0.92 s0 | trunc d0.92 s0 | trunc d0.92 s0 | KEPT d0.92 s0 | KEPT d0.85 s0 |
| 301 | 595 | 0.44 | 125 | 0.01 | 0.15 | 593 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.97 s0 | KEPT d0.95 s0 | KEPT d0.89 s0 |
| 302 | 595 | 0.44 | 131 | 0.02 | 0.09 | 593 | KEPT d1.30 s6 | KEPT d1.02 s0 | KEPT d1.01 s0 | KEPT d1.00 s0 | KEPT d0.94 s0 | KEPT d0.88 s0 |
| 304 | 502 | 0.63 | 93 | 0.03 | 0.01 | 500 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.97 s0 | KEPT d0.97 s0 | KEPT d0.77 s0 | KEPT d0.86 s0 |
| 305 | 384 | 0.60 | 79 | 0.06 | 0.10 | 382 | KEPT d1.18 s2 | KEPT d1.04 s0 | KEPT d1.00 s0 | KEPT d1.01 s0 | KEPT d0.93 s0 | KEPT d0.84 s0 |
| 306 | 629 | 0.36 | 202 | 0.06 | 0.00 | 627 | KEPT d1.07 s1 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d1.00 s0 | KEPT d0.92 s0 | KEPT d0.81 s0 |
| 308 | 347 | 0.41 | 106 | 0.01 | 0.01 | 345 | KEPT d2.41 s17 | KEPT d1.03 s0 | KEPT d1.00 s0 | KEPT d0.98 s0 | KEPT d0.89 s0 | KEPT d0.84 s0 |
| 309 | 390 | 0.42 | 118 | 0.06 | 0.00 | 388 | KEPT d1.60 s8 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.00 s0 | KEPT d0.97 s0 | KEPT d0.91 s0 |
| 311 | 378 | 0.28 | 151 | 0.10 | 0.00 | 376 | KEPT d1.18 s2 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d1.05 s0 | KEPT d0.99 s0 | KEPT d0.91 s0 |
| 315 | 839 | 0.34 | 309 | 0.16 | 0.00 | 837 | KEPT d1.36 s10 | KEPT d1.04 s0 | KEPT d0.93 s0 | KEPT d1.04 s0 | KEPT d0.87 s0 | KEPT d0.83 s0 |
| 316 | 779 | 0.36 | 221 | 0.08 | 0.00 | 777 | KEPT d1.09 s2 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.84 s0 | KEPT d0.83 s0 | KEPT d0.79 s0 |
| 317 | 391 | 0.40 | 117 | 0.09 | 0.00 | 389 | KEPT d1.10 s1 | KEPT d1.02 s0 | KEPT d1.01 s0 | KEPT d1.03 s0 | KEPT d1.00 s0 | KEPT d0.87 s0 |
| 318 | 582 | 0.19 | 200 | 0.09 | 0.00 | 580 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 | tip d0.01 s0 |
| 319 | 447 | 0.41 | 102 | 0.02 | 0.00 | 445 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.97 s0 | KEPT d1.00 s0 | KEPT d0.93 s0 | KEPT d0.83 s0 |
| 320 | 772 | 0.58 | 131 | 0.03 | 0.00 | 770 | KEPT d1.05 s1 | KEPT d1.02 s0 | KEPT d0.89 s0 | KEPT d1.01 s0 | KEPT d0.93 s0 | KEPT d0.83 s0 |
| 321 | 384 | 0.51 | 77 | 0.03 | 0.06 | 382 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.93 s0 | KEPT d1.01 s0 | KEPT d0.96 s0 | KEPT d0.91 s0 |
| 325 | 454 | 0.53 | 80 | 0.00 | 0.00 | 452 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.89 s0 | KEPT d1.00 s0 | KEPT d0.89 s0 | KEPT d0.71 s0 |
| 326 | 300 | 0.54 | 51 | 0.00 | 0.00 | 298 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d1.03 s0 | KEPT d0.95 s0 | KEPT d0.84 s0 | KEPT d0.79 s0 |
| 327 | 516 | 0.36 | 127 | 0.01 | 0.00 | 514 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.92 s0 | KEPT d0.99 s0 | KEPT d0.87 s0 | KEPT d0.81 s0 |
| 328 | 863 | 0.45 | 246 | 0.09 | 0.00 | 861 | KEPT d1.21 s6 | KEPT d1.03 s0 | KEPT d0.88 s0 | KEPT d1.02 s0 | KEPT d0.84 s0 | KEPT d0.77 s0 |
| 329 | 325 | 0.63 | 49 | 0.02 | 0.00 | 323 | KEPT d1.02 s0 | KEPT d1.02 s0 | KEPT d0.96 s0 | KEPT d1.00 s0 | KEPT d0.92 s0 | KEPT d0.85 s0 |
| 330 | 814 | 0.37 | 252 | 0.10 | 0.17 | 812 | trunc d1.47 s15 | KEPT d1.04 s0 | KEPT d1.00 s0 | KEPT d1.02 s0 | KEPT d0.94 s0 | KEPT d0.86 s0 |
| 333 | 500 | 0.62 | 92 | 0.02 | 0.07 | 498 | trunc d2.40 s32 | KEPT d1.12 s0 | KEPT d1.02 s0 | KEPT d0.99 s0 | KEPT d0.86 s0 | KEPT d0.79 s0 |
| 335 | 288 | 0.30 | 92 | 0.05 | 0.00 | 286 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d1.04 s0 | KEPT d1.01 s0 | KEPT d1.01 s0 | KEPT d0.97 s0 |

