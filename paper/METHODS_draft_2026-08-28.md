# METHODOLOGY — draft 2026-08-28

Status: complete-details draft for the author to rewrite. Every concrete value carries a pointer
(`file:line` in the repo at HEAD `8b29524`, branch `4dof-cartesian`; artifact paths under the
off-cluster mirror `~/workspace/final_rr_artifacts_2026-08-24/` = `M/`; cluster-only source under
`/cluster/tufts/shortlab/jstale02/` = `C/`). Where sources disagree both values are given and
flagged **[CHECK]**; the full **[CHECK]** list is collected in §8. Precedence when documents
conflict: `paper/AUDIT_results_2026-08-28.md` > `paper/PREREG_final_round_robin_2026-08-23.md`
(+A1–A10) > code > older narrative docs. Result numbers are deliberately *not* reproduced here
except where they characterise an instrument (fidelity, survivors, ceilings); results live in
`paper/RESULTS_TABLE_2026-08-25.md` (regenerate with `analysis/results_table.py`).

Notation: `d{source}_{learner}[-dense]_s{seed}`; sources dH (human), dDP (diffusion-policy
teacher), dR2D (r2dreamer teacher); learners DP, RLPD, r2dreamer (r2d), dv3.

---

> **LIVING UPDATES (2026-08-29, kept current by the agent during the cluster block).** Sections carry an
> `UPDATE 08-29` block wherever the recipe, seeds, or rules changed after this draft. Precedence: PREREG amendments
> A16–A19 > these blocks > the 08-28 text. Stable material the user is writing lives outside this file.

## 1. Real robot and demonstration data

### 1.1 Platform and teleoperation
| item | value | pointer |
|---|---|---|
| Robot | Kinova **gen3-lite**, 6 revolute arm joints (`joint_1..6`) + 2-finger gen3-lite gripper (`kortex_description/grippers/gen3_lite_2f`); ROS namespace `/my_gen3_lite` | `trial_reader.py:8-9,23-25`; `kinova.py:1-8`; URDF `gen3_lite_2f_robotiq_85.urdf` (misnomer: it is the kortex gen3-lite 2F gripper, not a Robotiq 2F-85 — `paper/real2sim_follower_lab_2026-08-23.md:239`) |
| Teleop modality | Cartesian-velocity joystick teleop; the operator commanded end-effector velocity in [x, y, z, pitch] (roll/yaw fixed), linear cap **0.11 m/s** (`VCAP`), pitch-rate cap **1.0 rad/s** (`PITCH_CAP`, = measured max \|wy\|), tool setpoint clamped to a base-frame workspace box `WS_LO=(0.30,-0.25,0.015)`, `WS_HI=(0.80,0.25,0.60)` | `baselines/cartesian_env.py:3-6,49,56,80-84`; `baselines/genesis_can_env.py:31-41` |
| 4-DOF verification | recorded `wx=wz=0` in 25/25 inspected demos, `wy≠0` in 25/25; over all 96 `*_cartesian.npy` tapes per-dim max \|v\| = [0.11, 0.11, 0.11, 0, 1.0, 0]; the plugin's Jacobian controller left roll/yaw free (drift up to 1.03/1.34 rad) | `cartesian_env.py:80-84`; measured from `inthewild_trials/*_cartesian.npy` |
| Gripper signal | `base_feedback…gripper_feedback[0].motor[0].position`, 0 open … 100 closed (observed range −0.98…100.69 in tapes); "closed" threshold `GP_CLOSE = 30` | `trial_reader.py:14`; `example.py:206-207`; `can_pos_recovery/replay_harness.py:60` |

### 1.2 What was recorded (ROS bags → `inthewild_trials/`)
| item | value | pointer |
|---|---|---|
| Topics (original reader) | `/my_gen3_lite/joint_states` (position[:6]), `/joy` (read, never stored), `/my_gen3_lite/base_feedback` (tool_pose x,y,z,θx,θy,θz; gripper motor position) | `trial_reader.py:1-2,11-26,30,50-51` |
| Topics (cartesian reader) | additionally `/my_gen3_lite/in/cartesian_velocity` (TwistCommand) and `base_feedback.tool_twist_*`; `/reward`,`/success` deliberately not extracted (untrusted auto-detector) | `trial_reader_cartesian.py:17,30-32,47-56` |
| Windowing | frame emitted whenever ≥1/60 s elapsed and both topics have ≥1 msg in the window; values are per-window means; last frame dropped | `trial_reader.py:32,43-54` |
| Effective rate | nominal "30 Hz" (`DATA_HZ = 30 # approx -- timestamps weren't saved; user_232 was ~31.6 Hz (919 frames / 29.06 s)`); joint_states itself is ~40 Hz (dt p95 25.4 ms); `*_cartesian.npy` for uid 232 has 1159 frames (~39.9 Hz) **[CHECK 1]** | `example.py:276`; `CAN_STARTING_POSITION.md:188`; `cartesian_env.py:51` (`DT = 0.025`) |
| Output (`<uid>_episodes.npy`) | pickled dict `{'vel_cmd': (T,6) float64, 'gripper_pos': (T,1)}`. **NB: despite the key name, `vel_cmd` is the measured joint position stream** (`/joint_states` position[:6]), replayed as PD position targets; several docs call it "commanded joint targets" **[CHECK 2]** | `trial_reader.py:69-74`; `example.py:284`; `real2sim_follower_lab_2026-08-23.md:85-86`; `trial_reader_cartesian.py:11` |
| Superset (`<uid>_cartesian.npy`) | `joint_pos, vel_cmd(=joint_pos), tool_pose (m; θ in degrees), tool_twist, gripper_pos, cartesian_velocity, cartvel_reference_frame` float32; tool_pose is in the ROBOT BASE frame; sim world z = base z + 0.05 | `trial_reader_cartesian.py:10-16,79-80`; `can_pos_recovery/read_bag.py:7` |
| Cameras | per-trial MP4s (`raw/user_<id>/cam_dev_video0` bottom-up through the translucent shelf, `cam_dev_video4` top view of the start), stored rotated ~90°, no factory calibration; **resolution/model not documented anywhere** **[CHECK 3]**; cameras are NOT in the bag reader and NOT used by any learner (sim cameras only, §3.2) | `CAN_STARTING_POSITION.md:123-124,189-190,218,283-284`; `HANDOFF_PLAN.md:80-82` |
| Object pose | never recorded (motivates §2.3) | `CAN_STARTING_POSITION.md:8-9` |
| Requirements | original reader needs ROS1 `rosbag`; cartesian reader uses pure-python `rosbags`; raw bags are not on the devbox | `trial_reader.py:1`; `trial_reader_cartesian.py:5-7,38` |

### 1.3 Trial census and labels
| count | meaning | pointer |
|---|---|---|
| 96 | `*_episodes.npy` files on disk, uids 232–335 (missing 241,264,271,272,291,292,323,332); = union of the label lists in `kinova.py` | `ls inthewild_trials`; `trial_reader.py:78-86`; `kinova.py:16-21` |
| 93 | trials in `can_pos_recovery/trial_placements.json` and `baselines/demo_manifest.json` (77 success / 16 fail). uids 253, 285, 289 (fail-list) are on disk but absent from both, reason undocumented **[CHECK 4]**. Docs say "93 in-the-wild trials recorded" | `PAPER_PLAN.md:75-80`; `paper/METHODOLOGY.md:65-72` |
| 2 stubs | 290, 322: gripper never closes; excluded | `CLAUDE.md`; `CAN_STARTING_POSITION.md:56,68,72,214` |
| 91 | graded = `baselines/demo_manifest_auth.json` (75 success / 16 fail) | measured |
| 16 fail-labeled | 238 240 249 260 268 270 282 288 296 307 310 312 313 314 324 334 (label bug: 322 and 331 appear in both a success and a fail list in `kinova.py`) | `demo_manifest.json`; `CAN_STARTING_POSITION.md:154,211` |
| 9 | success-labeled but never pick under replay: 236 237 239 244 255 261 294 303 331 | `demo_manifest_auth.json` |
| **66** | IL-usable human demos = 75 − 9; by furthest replay stage: picked-only 2 / placed 38 / contact 6 / nested 20 | `PAPER_PLAN.md:60-71,75-80`; crosstab of `demo_manifest_auth.json` |
| Frames | 66 IL tapes = 132,365 frames; all 96 tapes min 356 / median 1530 / max 6233 | measured from `n` fields |
| Rule | IL learners never train on failed demos (user rule 2026-07-30); fails are RL/WM negative data only | `PAPER_PLAN.md:357`; `METHODOLOGY.md:154-156` |
| `-p 0|1|2` | can-position bucket filter in `example.py` (nominal marked circles `POSITION_0=(0.4381,0.1)`, `_1=(0.4381,-0.05)`, `_2=(0.4381,-0.2)`; buckets wrong by up to ~5 cm, superseded by per-trial recovery §2.3); bucket counts 38/27/28 | `example.py:9,17,102-104`; `CAN_STARTING_POSITION.md:11-12,32-36` |
| Unlabeled 200–231 | never ingested ("~130 recordings" claim inconsistent with a 32-wide range **[CHECK 5]**) | `HANDOFF_PLAN.md:91-93` |

### 1.4 Task and geometry
| item | value | pointer |
|---|---|---|
| Task | pick a soup can from the table region and place/nest it against a static goal can on a shelf; stages picked → placed → contact → nested; **the study is scoped to the pick phase** | `paper/METHODOLOGY.md:45-48`; PREREG §0 |
| Can (real) | Campbell's soup can 66 × 101 mm, ≈0.35 kg | `CAN_STARTING_POSITION.md:170-171` |
| Can (sim) | cylinder r 0.033, h 0.101, ρ 1000 → 0.345 kg; `can_friction 0.2` (inert: finger–can pair takes max(1.0,0.2)) | `trial_placements.json['world']`; `replay_harness.py:125-128`; `gripper_lab_2026-08-25.md:62-64` |
| Goal can | same cylinder, ρ 1000, friction 2.0, static at `STATIC_BOTTLE_POSITION = (0.672, −0.221, 0.19)` | `replay_harness.py:51,129-132` |
| Goal recovery | user-validated demos 233 + 242 ("pick/place/slide perfect") must end touching the goal → two-circle intersection, south candidate chosen (matches slide directions). Rejected alternatives: fixed-radius ring fit on 61/75 real tool_pose slides → (0.662, −0.057) (rms 25 mm; biased by one-sided north approaches); placement-cluster median (0.656, −0.103); ancient (0.6, −0.2) | `replay_harness.py:43-51`; `can_pos_recovery/goal_from_slides.py:1-19`, `goal_from_slides.json`; `CAN_STARTING_POSITION.md:450-476`; `CLAUDE.md` 07-20 |
| Real shelf | open acrylic rack; **real shelf height above the table is unmeasured** (decides `shelf6` vs `shelf10`; inert for pick scope, blocks place/nested claims) | `CAN_STARTING_POSITION.md:190,208-210`; `paper/AUDIT_INDEX.md:82-83`; `UPDATE_2026-08-25.md:157-158` |
| Start pose | fixed arm start (`HARDCODED_START`), real tool pose at start ≈ (0.367, 0.011, 0.090) base frame, ~constant across demos | `example.py:180`; `replay_harness.py:62-64`; `cartesian_env.py:54` |
| Predicates | see §3.5 (pick) and §6.2 (placed/contact/nested; `NESTED_TOUCH_DIST = 0.066+0.015 = 0.081` m) | `replay_harness.py:52-56` |

---

## 2. Real-to-sim

### 2.1 Simulator pin
| item | value | pointer |
|---|---|---|
| Engine | Genesis, upstream `github.com/Genesis-Embodied-AI/Genesis @ 31951c3f` (reports `0.2.1`; ≈0.2.1 + ~270 upstream commits) + 2-file headless-render patch; reproduce: clone, checkout 31951c3f, `git apply cluster/patches/genesis_0.2.1_headless_render.patch`, `pip install -e .` | `cluster/patches/GENESIS_PIN.md:1-16`; commit `37b8d8f`; `gripper_lab_2026-08-25.md:30-31` |
| Patch content | `genesis/ext/pyrender/renderer.py`: per-env camera poses (`env_poses`) so one batched render serves N wrist cameras; `genesis/vis/camera.py`: `use_imshow=False` gate on all `cv2.imshow/waitKey` (headless) | `cluster/patches/genesis_0.2.1_headless_render.patch` |
| Engine gate | Genesis 1.2.1 rejected: 3.4× lower contact force, pinch grasp fails | `CAN_STARTING_POSITION.md:391-418` |
| Software | python 3.10; torch 2.7.0+cu126; taichi 1.7.4; numpy 2.2.6; cv2 4.11; one pip-only conda env serves genesis + lerobot + SB3 (conda-forge libstdc++ poisoned the stack 07-30 → "NEVER conda-install"); r2dreamer in a separate py3.11 venv (torch 2.8.0+cu126) | `CLAUDE.md:7`; `cluster/install_lerobot.sh:7-10,44-63`; `cluster_dv3_setup.sh:16`; `cluster/install_r2dreamer.sh:8,30-31`; `cluster/verify_env.sh:3-8,24-58` |
| Physics | `gs.init(precision="32", seed=0)`; `SimOptions(dt=0.01, substeps=8)` → substep_dt 1.25 ms; 3 physics steps per env step (30 Hz env clock); CPU backend for all training/eval worlds (one world per process) | `replay_harness.py:96-97,107-109`; `genesis_can_env.py:14,86,245-246`; `trial_placements.json['world']` |
| Contact solver defaults | every geom carries MuJoCo-default `sol_params` (timeconst 0.02 s, dampratio 1, dmin 0.90, dmax 0.95, width 1e-3, mid 0.5, power 2); this tree only `max()`-clamps timeconst at 2·substep_dt = 0.0025 s (the old comment in `replay_harness.py:98-99` claiming 0.2.1 hard-wires it is wrong for this tree) **[CHECK 6]** | `gripper_lab_2026-08-25.md:103-104,115-121` |
| Determinism | replay is machine- and load-dependent (cross-box placed +4/75; heavy CPU load flips borderline demos) → same-machine baselines, idle box, ×3 | `paper/METHODOLOGY.md:58-62`; `CLAUDE.md` 07-20 BB |

### 2.2 Scene (`can_pos_recovery/replay_harness.py::build_world`, lines 78-183)
| object | spec | pointer |
|---|---|---|
| Table (`table=True`) | fixed box (0.419, 1.2, 0.05) at (0.3395, −0.1875, 0.025); top `TABLE_TOP_Z = 0.05`; friction 0.5 | `:76,114-121` |
| Shelf ("box") | box `BOX_SIZE=(0.4,0.75,0.12)` at `BOX_POS=(0.75,−0.1875,0.05)` → `BOX_TOP_Z = 0.11` (only 6 cm above the table in the base world); ρ 1000 (~36 kg), friction 0.5; dynamic in base, `fixed=True` under variants | `:38-42,111-113`; `sim_variants.py:84-88,111-122` |
| Robot | `gen3_lite_2f_robotiq_85.urdf` at (0,0,0.05), fixed; arm kp `[200,200,150,100,60,60]`, kv `[20,20,15,10,6,6]`, force ±[50,50,50,20,20,20]; fingers kp 40 (`finger_kp`), kv 10, force ±50 (`finger_force`) | `:81,122-124,171-179`; `sim_variants.py:27-30` |
| Gripper | 4 finger DOFs with mimic (`tip = −0.676·bottom + 0.149`) enforced as equality constraints; reading 0..100 mapped linearly to driver θ ∈ [−0.09, 0.96] rad; collision = one convex hull per finger mesh | `gen3_lite_2f_robotiq_85.urdf:313`; `replay_harness.py:66-72`; `gripper_lab_2026-08-25.md:109-110,127-134` |
| Cans | §1.4 | |
| Cameras (`camera_rig=True`) | two 64×64 RGB cams: top at (0.40,−0.08,1.15) looking down (up=(1,0,0), fov 68); wrist fov 80 attached to eef with offset (0.10, 0, −0.03), pitched 30°, rolled 90°; obs image = top ++ wrist = (64,64,6) u8. Video cam (not an obs) 640×560 | `replay_harness.py:141-170`; `genesis_can_env.py:89,143-149` |

### 2.3 Can starting-position and goal recovery
- Method: FK at the grasp instant (closure interval that lifts and releases near the goal; grasp = reach-bottom) → fingertip midpoint = can xy; validated against bag `tool_pose` to ~1 cm xy / ~1 mm z; `fk_recovered.json` covers all 96 trials (`CAN_STARTING_POSITION.md:40-53,181-187`; `can_pos_recovery/recover_can_position.py`, `fk_all_trials.py`).
- Placement search around the FK seed: `search_placements.py`, `batch_harness.py` (GPU B=32), `cpu_research.py` (CPU-parallel, ok-class by construction, ~25 s/rollout, 16-way) → `trial_placements.json` (`CAN_STARTING_POSITION.md:192-203`; `cpu_research.py:1-17`). Current file: 93 trials; 72 "solved" (`ok_batch 52, ok 20`); 20 `no_shelf_batch`, 1 `shelf_but_no_slide`; unsolved 255, 303 (`CLAUDE.md` 07-20 BB).
- Independent panel (2026-07-08) overturned a rigged negative control and a coverage overstatement (winner drift up to 17 cm; 60 % depended on goal relocation); fixes baked in: fail-labeled demos never receive relocated goals; nested requires picked (`CAN_STARTING_POSITION.md:254-305`; `CLAUDE.md`).
- World corrections found 2026-07-04: table added; can geometry/mass corrected (was 70×75 mm ρ 2000); finger gains restored; substeps 1→4→8 (pick/run 0.59→0.74, 16==8 converged) (`CAN_STARTING_POSITION.md:160-178,341-349`). Note doc says finger kp 100 "sweet spot" but the live world uses `finger_kp 40` (commit `e8a07d6`, 07-14) **[CHECK 7]**.
- `--ic-from-tape` (PREREG A3): the 12 IL demos without a recovered placement (uids 233 259 262 266 267 275 278 301 319 321 329 333) reset to the can pose at frame 0 of their own stride-1 sim tape (`states[0][8:11]`, quat `[11:15]`), static goal; applied identically to all three sources so every source records from the same 66 ICs (`baselines/record_demos.py:662,731-757`; `PREREG:248-250`; `SESSION_LOG_2026-08-23_cluster.md:21`).
- Goal: §1.4. Whole-corpus replay at the final goal (75 success demos, base world): picked 61 / placed 47 / contact 15 / nested 16; fail-demo negative control nested 2/16 (`CLAUDE.md` 07-20; `METHODOLOGY.md:99-102`).

### 2.4 The human-command follower ("human" adapter)
The human demos are not re-encoded offline; they are **re-executed** inside the learners' MDP (§3) by a closed-loop waypoint follower, so every source is "recorded as executed" by one recorder.
| item | value | pointer |
|---|---|---|
| Source tape | stride-1 sim tape from `baselines/episodes_pick_phase_dppruned` (the 66 IL demos, DP-pruned, §4.5); `cmd = actions[:, :6]` (real measured joints used as targets), `ref = states[1:, :6]` (old-sim arm path), `grip = clip(actions[:,6])` | `record_demos.py:416-421,656`; `real2sim_follower_lab_2026-08-23.md:82-88` |
| Per decision | target waypoint `jt = min(j+3, n−1)` (≤4 waypoints per decision); `a_arm = clip((cmd[jt] − target_now)/(4·0.025), −1, 1)`; `a_grip = grip[jt]·2 − 1` | `record_demos.py:439-444` |
| Arrival rule | scan k from jt down to j; advance to k+1 if `‖q_meas − ref_k‖∞ < tol` (`meas`) OR, with `--arrival either`, `‖target_env − cmd_k‖∞ < tol`; **tol = 0.025 rad**; `max_dwell = 8` decisions without arrival → forced jump (counted as a stall); `dilation_cap = 3.0` (step cap = ⌈3·n/4⌉ decisions → `adapter_exhausted`); `settle = 25` decisions holding the last waypoint; `dilation = decisions·4/n_src` | `record_demos.py:427-437,448-470,668-671`; manifests `M/demos_v1/dH_either/manifest.json configs[0]` (`tol 0.025, max_dwell 8, arrival either, dilation_cap 3.0, settle 25`) |
| Rule of record | `--arrival either` (PREREG A6; lab: 49→55 local survivors, zero dwell stalls, identical path fidelity; commit 33d5bc9). The block's dH sets (matched_v2, matched_w3) are `either`; the original 51-tape set (matched_v1) and the dDP teacher's training data are `meas` (disclosed, A6) | `PREREG:272-276`; `real2sim_follower_lab_2026-08-23.md:23-31,132-133,274-276`; `SESSION_LOG:74-75` |
| Keep rule | kept iff the hardened pick (§3.5) is re-earned; drops reported as a source property (A1) | `PREREG:241-243` |

### 2.5 Sim variants (`baselines/sim_variants.py`)
Mechanics: `install(name)` monkeypatches `gs.Scene.add_entity` before build (robot gets `Rigid(gravity_compensation=gc)` and `pos.z += riser`; shelf gets `pos.z += shelf_dz`, `fixed=True`); `post_build(w, name)` sets arm kp/kv = BASE × mult, force range, optional `grasp_timeconst`, `goal_start_z += shelf_dz` (`:93-163`). Plumbed via `baselines/sim_variant_hook.py apply_pre/apply_post`; `'base'` = unpatched; stamped into every tape/manifest/sidecar/registry row; eval resolution "explicit CLI > checkpoint sidecar > 'base', mismatch = refuse" (`sim_variant_hook.py:1-10`; `record_demos.py:115-120,617-633,669`; `sbatch_dp.sh:195-196,300-303,475`; `sbatch_rlpd.sh:87,169-172,190,209`; `sbatch_r2dreamer.sh:209-212,252-258`; `wandb_eval.py:122-140`).

| variant | kp×/kv× | gravity comp | riser | shelf_dz | grasp_timeconst | line |
|---|---|---|---|---|---|---|
| `base` | 1/1 | 0 | 0 | 0 | — (engine default 0.02 s) | `:35` |
| `gc_kp4_riser3` | 4/2 | 1.0 | 0.03 | 0 | — | `:51` |
| **`gc_kp4_riser3_shelf6`** (world of record for the corrected block) | 4/2 | 1.0 | 0.03 | 0.06 (shelf top 0.17 world) | — | `:67` |
| `gc_kp4_riser3_shelf6_ts5` (pilot only) | 4/2 | 1.0 | 0.03 | 0.06 | 0.005 s on the 4 finger geoms AND the picked can (Genesis averages a pair's `sol_params`, so both sides must move); floor `tmin = 2·substep_dt` enforced by `max()` | `:69-78,138-156` |
| `gc_kp4_riser3_shelf10` (data-fit alternative) | 4/2 | 1.0 | 0.03 | 0.10 | — | `:80-82` |
| others (`gc`, `kp2`, `kp4`, `gc_kp8`, `*_urdf`, `riser2/4`, `fcap2`, `fmap`) | ablation ladder | | | | | `:36-60` |

kp×4 → arm kp `[800,800,600,400,240,240]`, kv×2 (`SESSION_LOG:136`). `post_build` prints but does **not assert** the 5-geom count (CRITIQUE Q8) **[CHECK 8]** (`sim_variants.py:156`; `CRITIQUE_decisions_2026-08-26.md:364-368`).

### 2.6 Physics defects found against real bag data, fixes, and fidelity
Source docs: `paper/real2sim_follower_lab_2026-08-23.md` (defects a–d) and `paper/gripper_lab_2026-08-25.md` (defect e). All numbers are local-devbox unless marked cluster.

| defect | evidence | fix |
|---|---|---|
| (a) no arm gravity compensation + soft PD (sag τ_g/kp; ~4 cm eef sag at the extended place pose, uid 242 sim eef 0.147 vs real 0.191) | `real2sim…:166,171-176,460-462` | `gravity_comp 1.0`, kp×4 / kv×2 |
| (b) bandwidth: leash-limited steady speed ≈0.75 rad/s vs real p99 0.6–1.35 rad/s | `:178-182` | kp×4 |
| (c) robot mounted ≥3 cm too low relative to the table (real tool z reaches −0.019…+0.013 base-frame; sim finger hulls bottom out at tool z ≈ +0.03) → "pressing" failures 295/299/254/256/286/298/330 and the 246 knock-over | `:184-195,208-215`; `sim_variants.py:46-49` | `riser 0.03` |
| (d) shelf modelled half-buried (top 6 cm above table; real release tool z 0.17–0.21 ⇒ ≥12 cm) — masked in the base world because the sag lowered the can onto the low shelf (two cancelling errors: riser-only world nested 20→3) | `:456-467`; `UPDATE_2026-08-25.md:91-96`; `sim_variants.py:61-66` | `shelf_dz 0.06` (shelf10 = data-fit upper bound; real height unmeasured) |
| (e) contact-solver `timeconst 0.02 s` = 8× the engine floor 2·substep_dt = 0.0025 s; penetration ∝ timeconst²; the old "finger reading map is miscalibrated" theory refuted (real gripper over-travels into rubber/current limit at readings 0.667–1.007 on the same can) | `gripper_lab:25-35,46-53,103-121,184-216` | `grasp_timeconst 0.005` (ts5) — **pilot only, not adopted for any result block** (§2.7) |

Fidelity (replaying the real robot's recorded joint stream through the follower, 66 tapes, 39 332 frames):

| metric | base | gc_kp4_riser3 / shelf6 | +ts5 | pointer |
|---|---|---|---|---|
| joint tracking ‖e‖∞ p50 / p95 (rad) | 0.047 / 0.100 (whole-corpus figure quoted as 0.045 / 0.090) | **0.004 / 0.030** (0.003 / 0.029 in the 75-demo table; 0.0033 / 0.0285 cluster) | 0.0035 / 0.0300 | `real2sim…:162-170,199-208,441-451`; `UPDATE:87-89,104-108` |
| frames beyond the leash (0.125 rad) | 7.9 % (4.0 % in the 75-demo table) | **0.1–0.2 %** | unchanged | same |
| frames beyond the cap | 87 % | 10 % | — | `real2sim…:199-208` |
| tracking error at grip close | 0.049 | 0.002 | — | same |
| pick re-earned under open-loop replay | — | 64/66 | — | `real2sim…:41-46` |
| full-task 75-demo replay picked/placed/placed_v2/contact/nested/tipped | 67/59/26/18/20/32 | 71/55/28/22/17/38 | 69/60/31/20/20/23 | `real2sim…:441-451`; `gripper_lab:521-527,623-640` |
| carry penetration med-of-max / p95 | 9.7–9.9 mm / 9.0 mm | 9.9 mm | **1.5 mm / 0.90 mm** | `gripper_lab:623-640`; `UPDATE:104-108` |
| free tip-overs / 75 | — | 20 | 12 (10 recoveries vs 2 losses, McNemar p≈0.039) | `UPDATE:106`; `CRITIQUE_decisions:37-42` |
| grip force med-of-max (obs[7]-related) | 143 N | **147 N** | **245 N** (fingers stall at the surface holding full PD error; every reduction lever loses the grasp; engine has no torsional friction/impratio) | `gripper_lab:54-61,737-758` |
| human pick recreation (recorder, /66) | 51 (meas) → 56 (either) old world | **57** base-control vs **58** ts5 (cluster-confirmed; ts5 rescues 246, loses 316/254) — but 2 vs 1 discordant, McNemar p=1.0 | | `UPDATE:100-109`; `gripper_lab:459-486`; `CRITIQUE_decisions:37-42` |
| random-teacher control | 0/30 cluster, 0/90 local | | | `UPDATE:104`; `gripper_lab:603-607` |
| E5 caveat | ts5 pair-averaging also stiffens can↔goal-can and can↔shelf contacts 0.020→0.0125 s (1.6×) — the metric-bearing contacts for contact/nested | | | `AUDIT_INDEX.md:75-77`; `CRITIQUE_decisions:182-201` |

**[CHECK 9]** the "57/66 vs 58/66 pick recreation" and "56/66 either" figures come from different runs (gripper-lab g_base control vs the 08-23 dH_either recording vs the 08-24 dH_w2 recording = 58/66 in `gc_kp4_riser3_shelf6`); state which one is meant.

### 2.7 Which world each block ran in
| world | sim_variant | demo root | seeds / cells | pointer |
|---|---|---|---|---|
| old | `base` | `baselines/matched_v2` (N=56) | DP 10–14 (dH/dDP/dR2D); RLPD 10–13 (sparse+dense, + fails arms sparse); **all r2dreamer cells** (80–93, 99–108, 200–203, 300–303); N5/N15/N13/N16 | `analysis/results_table.py:10-20`; `RESULTS_TABLE_2026-08-25.md:3`; `UPDATE:14`; `CRITIQUE_decisions:389-391` (C3) |
| corrected | `gc_kp4_riser3_shelf6` (adopted by user 08-24 ~08:15, shelf6 pending the real shelf measurement) | `baselines/matched_w3` (N=58; dH + dDP only, no corrected-world dR2D teacher) | DP 20–29; RLPD 20–29 (sparse+dense); N7 density 20–22; N14 split halves 40–42; r2d pilot 60/61 (0/2 at TL 1200) | `SESSION_LOG:129,141,158-160`; `M/matched_w3/MATCHED_SETS.json` |
| corrected + ts5 | `gc_kp4_riser3_shelf6_ts5` (commit 7c8195d "world of record from 08-26") | `matched_w4_pilot/dH` only (`dH_w4` recording 08-26 07:50) | DP pilot `dp_pilotw4` only; **no result block**; `matched_w4` proper never built; no kept count/readout for dH_w4 recorded in any note **[CHECK 10]** | `SESSION_LOG:153-156`; `results_table.py:19-20`; `POSTMAINT_COMMANDS.md:118-126` |

---

## 3. The MDP and the contract-v1 recorder (`baselines/record_demos.py`)

### 3.1 Environment
`FullTaskEnv(backend='cpu', max_steps=1200, scope='pick', action_mode='delta_joint', delta_cap=0.025, delta_leash_mult=5.0, action_repeat=4, delta_ref='target', camera_rig=True, pick_shaping=False)` (`record_demos.py:94-103,219-231`; `baselines/rl/full_env.py:265-270,310`). Asserted at construction: scope/mode/ref, repeat and cap, leash == 0.125, no shaping/hold reward, inner env never truncates (`genv.max_steps = 1e9`, the #26 bug guard) (`record_demos.py:226-230`; `full_env.py:351`). `sim_variant` applied pre/post build and stamped (`:221,231,781`).

### 3.2 Observation (17-dim state + images)
`state = [q[0:6], gripper motor 0..1, grip effort, can xyz, can quat wxyz, goal xy]` (`genesis_can_env.py:298-313`; `pick_env.py:36`). Gripper motor = `1 − (θ − (−0.09))/(0.96 − (−0.09))`; grip effort = |control force| on the two bottom finger drivers ("sim analog of motor current"; its scale moves with the world variant: carry median 18.4 base → 26–28 ts5, `gripper_lab:772-776`). lerobot split: `observation.state = states[:8]` (proprio), `observation.environment_state = states[8:]` (can pose + goal) (`convert_to_lerobot.py:81-89`). Images `(64,64,6)` u8 top++wrist (§2.2); WM learners use images only (`cnn_keys: image`, `mlp_keys: '$^'`), DP and RLPD use state only. **[CHECK 11]** `genesis_can_env.py:10` docstring still says `float32[16]`.

### 3.3 Action space and integrator
`Box(−1,1,(7,))` (`full_env.py:392`). Per sim step (`full_env.py:560-578`):
```
d  = clip(a[:6], -1, 1) * delta_cap              # 0.025 rad / sim step
sp = clip(target + d, ARM_LO, ARM_HI)            # delta_ref='target': integrate onto the running target
target = q_meas + clip(sp - q_meas, -leash, +leash)   # leash 0.125 rad around the measured joints
grip_phys = (clip(a[6], -1, 1) + 1) / 2          # absolute 0..1
```
`action_repeat=4`: the same `a` is applied on 4 consecutive sim steps (advance ≤ 4·a·cap), rewards summed, loop breaks on terminated/truncated (`full_env.py:531-542`). Target re-seeds from measured q on every reset (`:402-412`). Calibration: cap 0.025 = the demos' p99 per-frame commanded delta (44°/s at saturation); leash 5×cap = 0.125 ≈ the demos' PD-lead p99 (0.126) (`full_env.py:300-302`; `paper/METHODOLOGY.md:233-244`). Decision clock = 4 sim steps = 7.5 Hz. DP emits absolute window-end targets and is executed through the same integrator at eval ("hold-4", §6.2).

### 3.4 Horizon and termination
- `max_steps = 1200` sim steps = 300 decisions; `truncated = (not terminated) and t ≥ max_steps` (`full_env.py:689`); the tape ends exactly at the terminal or the cap (`record_demos.py:156,318-325`).
- **Tip rule**: after each sim step, if `grip_phys < GRIP_OPEN (0.3)` and can tilt > `TIP_DEG (60°)` → `terminated`, `info['tipped']`, reward += `TIP_PENALTY`. **`TIP_PENALTY = 0.0` for pick/full scope** (`PLACE_TIP_PENALTY = −0.25` place scope only). PREREG §2 says r=0. The "−0.5" in the task brief/CLAUDE.md history is the July Cartesian env; not current **[CHECK 12]** (`full_env.py:214-218,673-688,705`). Census basis: past 60° the can never recovers (31/32); grip-open guard needed because demos carry the can pitched >60° in hand (`CLAUDE.md`; `METHODOLOGY.md:120-124`).
- The **eval** harness uses `GenesisCanEnv` (no tip termination, runs the full horizon) — §6.2.

### 3.5 Pick predicate ("hardened lift")
All of `can_z > pick_z` AND `grip·100 > GP_CLOSE (30)` AND `‖eef − can‖ < PICK_EEF_DIST (0.20 m)` sustained `PICK_SUSTAIN = 10` consecutive sim steps; `eef` = wrist link position; `pick_z = table_top + can_h/2 + 0.05 = 0.05 + 0.0505 + 0.05 = 0.1505` (`genesis_can_env.py:48-65,255-265`; `replay_harness.py:184`; census header "pick_z 0.1505"). Pick scope terminates on the sim step the grant lands (`full_env.py:627-633`); no settle. Hardening history and the frozen-policy fairness certification: `paper/METHODOLOGY.md:109-118`.

### 3.6 Rewards
- Sparse: `STAGE_REWARD = {picked 1.0, placed 1.0, contact 2.0, nested 4.0}`, paid once per first grant; pick scope → +1 and terminate (`full_env.py:57,593-606,627-633`). Tapes carry sparse env rewards only ("shaping is NOT baked in", `record_demos.py:22`).
- **Fast-pick 2.0 quirk**: if the pick grant and the placed grant land in the same repeat-4 window (`GenesisCanEnv.step` sets `_picked` then `_placed` in the same sim step when the can is already in the shelf footprint/z-band), the terminal row carries reward 2.0. Seen on r2d-teacher tapes (~68-step picks); `to_dreamer_native.py` accepts `{0,1,2}` and refuses positive reward before the terminal row; matched_v2 `r2d/dR2D` total_reward 64 for 56 tapes (8 tapes pay +2) (`baselines/rl/to_dreamer_native.py:65-72`; `genesis_can_env.py:265-267`; `SESSION_LOG:114`). Mechanism reading is from code, not from a comment **[CHECK 13]**.
- Dense (potential-based): `r' = r + γ·φ(s') − φ(s)`, `φ = −PICK_SHAPING_SCALE·‖eef − can‖` with `SCALE = 2.0`, **φ(terminal) = 0** (`pick_shaping_terminal_zero`), γ = the learner's discount (RLPD 0.998 via `pick_shaping_gamma=args.gamma`, asserted equal; r2d 1 − 1/horizon = 0.997 after the patch; dv3 0.997); demo transitions relabelled offline from the recorded `eef_pos` with the same potential (`full_env.py:95-102,261-262,333,543-557`; `train_rlpd.py:99,245,255,308-312`; `cluster/patches/r2dreamer_final_rr.patch` hunks 1–2; PREREG §2).

### 3.7 `terminal_from_tape` and the terminal guard
`full_env.terminal_from_tape(tape, …) → {t_term, kind ∈ pick|tip|other, reward, layout}`: contract-v1 tapes → reads `terminated/picked/tipped` (`full_env.py:152-168`); legacy stride-1 tapes → recomputes the pick proxy + tip rule, warns that the hardened eef distance is unavailable (`:173-211`). Every legacy encoder/converter has `--demo-terminal-guard` default **on** (tip → done, r=0, later frames dropped; pick → done; cap → bootstrap) (`train_rlpd.py:145-153`; `train_sacfd_full.py:622`; `to_dreamer_demos.py:70`; PREREG §4.3). This guard is the fix for the 08-19 `dDP_RLPD` 0/6 collapse (fail tapes without terminal flags → critic bootstrapped through states the env would have terminated; N1).

### 3.8 Tape contract v1 — keys and manifest
One npz per episode, filename = rollout index ≥ 100000 (`rollout_base + 1000·shard_idx`, never a human uid; `record_demos.py:774-777,804`). Keys (`:16-33,329-341`): `states (n,17)`, `final_state (17,)`, `actions_delta (n,7)` (the executed normalized decision — the learners' action space), `actions (n,7)` (absolute window-end command: 6 joint targets rad + grip 0..1), `rewards, terminated, truncated, picked, placed, contact, nested, tipped (n,)`, `eef_pos (n+1,3)` (tool tip), `images (n+1,64,64,6)`, `sim_states (m,17)`, `sim_actions (m,7)` (per-sim-step sub-tape, n ≤ m ≤ 4n), `end_reason`, `n`. Scalars (`:115-120`): `uid, ic_uid, label ('success' iff picked on the last row), stage, teacher, teacher_ckpt, act_mode, sim_variant, action_repeat=4, delta_cap, delta_leash, delta_ref='target', pick_z, n, git_sha, env_class='FullTaskEnv', recorder='record_demos.py v1', contract='v1', end_reason, scope, max_sim_steps, verify`. Validator: exactly one of terminated/truncated on the last row and never earlier (`:125-200`).
Manifest (`:830-846`, merged `:599-635`): full argparse config incl. `tol, max_dwell, arrival, dilation_cap, settle, attempts, verify, seed, shard, git_sha, teacher_ckpt`; `n_rollouts, n_kept, n_fail, rejected_by_verify, teacher_success_rate_first_attempt, yield_frac`; per-rollout `records` (ic_uid, ic_can_xy, outcome, decisions, sim_steps, seconds, kept, end_reason, attempt, verify, dilation…); `content_sha256` (sha over sorted `name bytes + file bytes`), `built`.
Fails go to `<outdir>_fails/` (same schema, `label='fail'`); the 5xxxxx stems are assigned by the set builder (`make_matched_sets.py:151-161`: `500000 + stem % 100000`), 6xxxxx by `make_succ_control_set.py`.

### 3.9 Adapters and guards
| adapter | behaviour | pointer |
|---|---|---|
| `human` | §2.4; attempts forced to 1; `verify` off (the keep predicate is the pick itself) | `record_demos.py:395-470,694-695,742` |
| `dp` | lerobot DP queried once per decision on the current obs; absolute target q* → `a_arm = clip((q* − target_now)/(4·cap))`; grip = chunk value ·2−1; `--mode sample` (default) / `mode` (reseeds per episode); policy on GPU if visible, sim CPU (A4); `--attempts 3 --verify` | `:473-504,654,663`; manifests |
| `r2d` | champion loader, native delta actions, asserts cfg action_mode/repeat/cap/leash/size; `--mode sample --attempts 3 --verify`; `--torch-threads 2` | `:507-582` |
| `random` | uniform [−1,1]^7 per decision (rng seed+7919); warns if kept > n/30 | `:388-392,849-850` |
| `--verify` | open-loop replay of `actions_delta` from a fresh reset of the same IC must re-earn the pick; failures routed to fails with `outcome='verify_failed'`; pick scope only | `:362-371,685-687,798-803` |
| `--ic-from-tape` | §2.3 | `:662,731-757` |
| `--sim-variant` | default `base`; merge refuses mixed variants | `:669,619-622` |

---

## 4. Demonstration sources and sets

### 4.1 Recordings (all by `record_demos.py v1`, repeat 4, cap 0.025, leash 0.125, 1200 sim steps, `src = baselines/episodes_pick_phase_dppruned`, `--ic-from-tape`, 66 ICs cycled)
| recording | world | teacher / adapter | rollouts → kept | first-attempt rate | verify rejects | content sha (prefix) | pointer |
|---|---|---|---|---|---|---|---|
| `dH` (meas) | base | human, `--arrival meas`, git 91cd7b1/bde726d | 66 → **51** | — | — | `58fd89bc` | `paper/final_rr_sets_2026-08-23/recorder_manifest_dH.json` |
| `dH_either` | base | human, `--arrival either`, git 33d5bc9 | 66 → **56** | 0.848 | — | `4d4ea08f` | `…/v2/recorder_manifest_dH_either.json`; `M/demos_v1/dH_either/manifest.json` |
| `dDP` | base | dp, ckpt `baselines/outputs/dp_pilot/dH_DP_s0/checkpoints/020000/pretrained_model` (DP-r4 pilot s0, sel 13/15; PREREG §3.1 "median in-dist seed" → with 2 pilot seeds the lower), `--mode sample --attempts 3 --verify`, git 1dcd268 | 80 → **64** | 0.818 | 2 (uids 276, 317 → `dDP_fails_verifyrej`) | `c33a4e2e` | `M/demos_v1/dDP/manifest.json`; `SESSION_LOG:58,61`; PREREG A6 (teacher trained on the 51 meas tapes — a superset/variant relation to the block's dH, disclosed) |
| `dDP_fails` | base | same teacher | 16 → 14 files: 8 `env_truncated` at 300 decisions (uids 258 266 267 298 299 304 326 327) + 6 lying-can tapes of 1 decision (234×3, 318×3); 2 verify-rejected | 0.0 | 2 | `41a572f3` | `M/demos_v1/dDP_fails/manifest.json` |
| `dR2Dprov` | base | r2d, ckpt `C/r2dreamer/runs/pick_v5d4c_delta_shaped_dH_s51/BEST_selected.pt` (**PREREG A2 fallback**, not CHAMPION_1576820.pt), `--mode sample --attempts 3 --verify`, git 3167d12/f6d937f | 73 → **64** (8 tipped, 1 verify-failed uid 333) | not stamped | 1 | `16f62a43` | `…/recorder_manifest_dR2Dprov.json`; `AUDIT_prelaunch_2026-08-23.md:207` |
| `_smoke_negctl` | base | random, n=3 | 3 → **0** (2 truncated, 1 tipped) | 0.0 | — | — | `M/demos_v1/_smoke_negctl/manifest.json`; PREREG A5 (n=3, not the registered 30) |
| `dH_w2` | gc_kp4_riser3_shelf6 | human, `either`, git e306ec9 | 66 → **58** (drops: tipped 234, 318; env_truncated 245 246 286 293 295 300); dilation p50 0.99, max 1.07 | 0.879 | — | `07ce3f36` | `M/matched_w3/dH/manifest.json`; `SESSION_LOG:129` |
| `dDP_w2` | gc_kp4_riser3_shelf6 | dp, ckpt `baselines/outputs/dp_pilotw2/dH_DP_s0/checkpoints/100000/pretrained_model` (pilot hold 14/15, rnd 18/30), `attempts 3 --verify`, git 2061f08 | 76 → **64** (7 tipped, 5 truncated) | 0.879 | 0 | `1da70d3f` | `M/matched_w3/dDP/manifest.json`; `SESSION_LOG:141` |
| corrected-world dR2D | — | **does not exist** (no corrected-world r2d teacher; the `matched_w3/dR2D` row was a byte-identical copy of dDP and was purged, BL-7) | | | | | `M/matched_w3/MATCHED_SETS.json NOTE_2026-08-25`; `CRITIQUE_launch_plan_2026-08-25.md:210-234` |
| `dHunpruned{,_either}` | base | human on `episodes_pick_phase_all` (control for §4.5 pruning) | recorded; **no result appears in any results file** **[CHECK 14]** | | | | `SESSION_LOG:60,94` |

dH drop reasons vs PREREG A1 wording ("2 lying-can, 1 knock-over, 5 over-horizon, 7 over-cap" for the 15 meas drops): recorder outcomes are 3 tipped (234, 318 lying-can ICs; 246 knock-over), 10 `env_truncated` (1200-step horizon), 2 `adapter_exhausted` (dilation cap 3.0: 257, 298) — the 5/7 split does not match the 10/2 outcome codes **[CHECK 15]** (`recorder_manifest_dH.json records[]`; `PREREG:241-243`). Lying-can ICs 234 and 318 are the two human demos whose recorded can starts on its side; they are unachievable in the MDP and also make uid 234 the `sel` ceiling (§6.1).

Teacher circularity (disclosure): both model teachers descend from dH (dDP = DP trained on the 51-tape meas dH; dR2D = r2dreamer dense trained on the 08-19 human set) (PREREG §3.1, §10).

### 4.2 Matched sets (`baselines/make_matched_sets.py`)
Rule: `N = min(success counts, --cap-n 66)`; stratified round-robin over `ic_uid` with common capacity `c(u) = min over sources` so all sets cover the same uids with the same multiplicity (`ic_multiset_identical: true`); `rng = default_rng(seed=0)`; tapes hard-linked; refuses non-v1 tapes, label/picked mismatch, unterminated tapes, mixed `sim_variant` (`:64-78,103-142,167,183,207,211`). Frames are not matched (intrinsic; reported). `content_sha256` over sorted `name + bytes` (`:94-100`); every sbatch asserts the sha of the set it consumes. Fails arms: `F ≤ ⌊0.30·N/0.70⌋`; lying-can fail tapes (`n ≤ 1 and tipped`) excluded since commit ca05472 (W5) — hence matched_v1 fails arms carry 14 fails and matched_v2 carry 8 (`:222-231`). Per-learner derivations are built separately: lerobot via `convert_to_lerobot.py <set> <set>/lerobot 8 4 none image` (fps = 30/4 = 7.5, gated against `meta/info.json`), dreamer-native via `to_dreamer_native.py` (§4.7); `sbatch_dp.sh:14,221` wrongly says make_matched_sets builds them **[CHECK 16]**.

| root | built / git | N | dH sha | dDP sha | dR2D sha | dDPfails (N) | dR2DDPfails (N) | world | pointer |
|---|---|---|---|---|---|---|---|---|---|
| `matched_v1` | 08-23 13:53 / 0e1fda19 | 51 | 58fd89bc | 9be55219 | fe93dbdd | c84fd1c1 (65) | 4bf50466 (65) | base | `paper/final_rr_sets_2026-08-23/MATCHED_SETS.json` |
| **`matched_v2`** (block of record, old world) | 08-23 14:17 / ca054726 | **56** | 4d4ea08f | 299825ad | 10cbc9b2 | 89609cc1 (64 = 56+8) | b222df06 (64) | base | `…/v2/MATCHED_SETS.json`; `M/matched_v2/` |
| **`matched_w3`** (corrected world) | 08-24 11:46 / 2061f082 | **58** | 07ce3f36 | d3bf95f6 | — | none | none | gc_kp4_riser3_shelf6 | `M/matched_w3/MATCHED_SETS.json` |
| `matched_w4` | never built (only `matched_w4_pilot/dH`) | — | | | | | | ts5 | `SESSION_LOG:156` |

Provenance chain for the WM fails arm (state once): RLPD manifest sha `b222df06…` = `repeat.json.src_sha` → native sha `3a6df556…` → registry fingerprint `c5ae239b…` (`AUDIT_results_2026-08-28.md §5`).

### 4.3 Census (`analysis/characterize_demo_sets.py`; constants pick_z 0.1505, cap 0.025, leash 0.125, tip 60°/grip<0.3)
| metric | v1 dH / dDP / dR2D | **v2 dH / dDP / dR2D** | v2 dDPfails / dR2DDPfails | w3 dH / dDP | pointer |
|---|---|---|---|---|---|
| tapes | 51 | 56 | 64 / 64 (8 fails each) | 58 | `census.md` per root |
| transitions (decisions) | 6491 / 6294 / 861 | **7002 / 6909 / 954** | 9309 / 3354 | 6927 / 7285 | |
| tape length p50 (decisions) | 113 / 113 / 17 | **115 / 114 / 17** | 122 / 17 (fail len 300) | 110 / 120 | |
| tape length mean / max | 127,285 / 123,233 / 17,24 | 125,284 / 123,233 / 17,24 | 145,300 / 52,300 | 119,246 / 126,235 | |
| **fail share of buffer (rows)** | — | 0 | **0.258 / 0.716** (=25.8 % / 71.6 %; PAPER_NOTES' 74/28 was wrong) | 0 | `AUDIT_results:96`; `M/matched_v2/census.md` |
| unique ICs / grid cells (of 16) | 50 / 13 | 55 / 13 | 56 / 13 | 57 / 13 | |
| reward density | .0079 / .0081 / .0592 | .0080 / .0081 / .0587 | .0060 / .0167 | .0084 / .0080 | |
| decisions at the cap | .055 / .010 / .048 | .069 / .010 / .047 | | .054 / .024 | |
| human time dilation p50 / max | (blank) | 1.03 / 1.65 | | 0.99 / 1.07 | |
| tapes tipped at t0 | 0 (fails arms 6) | 0 | 0 | 0 | |
| all tapes: contract v1, repeat stamp 4, terminal on last row only, images present | yes | yes | yes | yes | |

The "decisions per tape" summary used in prose (dR2D ~17, dDP ~123, dH ~115) is the v2 p50/mean mix. Fail tapes are 300–301 rows each (cap-truncated, bootstrapped), 12× the median dR2D success tape and 3× the WM training `time_limit`; they are 70 % of the fails arm's demo rows (`AUDIT_results §1`).

### 4.4 Split halves (`baselines/make_split_halves.py`, N14)
Group tapes by `ic_uid`, shuffle uid list with `default_rng(seed=0)`, alternate whole ICs A/B; per-half manifest with `split_half, split_seed, sha` (`:28-56`). Run on matched_w3 dH and dDP: "58 tapes over 58 ICs → A 29 / B 29" (`SESSION_LOG:157`). Halves' shas not captured locally **[CHECK 17]**.

### 4.5 Pruning
- Leading-idle pruning applied ONCE to the human stride-1 tapes before re-recording (`make_dp_pruned.py`: collapse idle runs with action delta < 1e-3 before `j_pick − 150` frames; 29.6 % of frames) — this is `baselines/episodes_pick_phase_dppruned`, the `src` of every recording; the unpruned control is `dHunpruned` (`make_dp_pruned.py:7-15,75-81`; `PAPER_PLAN.md:400-407`; PREREG §3.1).
- Post-hoc density control sets (N7, `baselines/make_pruned_bc_set.py:46-49`): keep decision iff `max|a_arm| ≥ eps` OR `|Δa_grip| > 5e-3`; terminal always kept; BC-only (sim sub-tape dropped). Measured at eps 1e-3 on matched_w3/dH: 6927 → 6116 decisions (**11.7 % removed**; 414 of the removed had grip closed). The 1e-2 fraction ("22 %") exists in no artifact — source or delete **[CHECK 18]** (`SESSION_LOG:159`; `AUDIT_results:84,114`).

### 4.6 Fails arms and controls
- `dDPfails` = 56 dDP successes + 8 dDP fails (episode share 0.125; row share 25.8 %); `dR2DDPfails` = 56 dR2D successes + the same 8 fails (row share 71.6 %). The 8 fail tapes are byte-identical across the RLPD and WM arms (`AUDIT_results §0`).
- N16 controls (`make_succ_control_set.py`, PREREG A10): `dR2DDPsucc` = dR2D + the 8 longest dDP success tapes (6xxxxx stems; 1521 added rows vs the fails' 2400 → `length_match_fraction 0.634`; N 64) (`M/matched_v2/dR2DDPsucc/manifest.json`); `dR2Ddup13` = dR2D at `demo_duplicate 13` (ring share ≈27 % = fails arm) via `sbatch_r2dreamer.sh:199,236`. Predictions registered in `PAPER_NOTES.md` N16 (`:419-441`).
- Exposure arithmetic of record (supersedes every "1–3 %" / "1/17th" figure): with `demo_reinject_every 150000 × demo_duplicate 4` over a 3e6-step budget → 19 re-injections; steady-state ring share dR2D ≈ 9.8 %, fails arm ≈ 26.7 % (fail rows 18.8 %); RLPD 50 % demo per batch, fail rows 35.8 % → exposure ratio ≈ 2× (`AUDIT_results §1`).

**UPDATE 08-29.** Same-source fails arms only (user rule 08-28: never mix sources): `dHHfails` = 56 dH successes + 8 dH
fail tapes; `dDPfails` = 56 dDP successes + 8 dDP fail tapes (`baselines/make_samesource_fails_arm.py`; fail share 0.30 by
tape was requested but only 8 fail tapes per source survive the lying-can/success-label filters → share 0.125 by tape,
fail rows 2145 / ~2300 = 23–25 % of rows; 7 truncated + 1 tip-terminated (dH), 8 truncated (dDP)). Row-matched
same-source controls `dHsucc_dup` / `dDPsucc_dup` = the 56 successes + the 8 LONGEST own successes duplicated (6xxxxx
stems; `baselines/make_succ_control_set.py --donor <same arm>`; added rows 1755 / 1521). Reading rule (A17): a fails effect
is claimed only if +fails differs from +succ_dup in the same direction on both sources. Census per source in
`paper/tape_census_2026-08-29.txt` (`baselines/diagnostics/tape_census.py`): dH vs dDP identical on N, rows, terminal
rows, saturation (≤6 % on any joint), state ranges; dDP tapes carry `verify=pass` (open-loop replay at record time),
dH `verify=n/a`. The mixed-source `dR2DDPfails` / `dR2DDPsucc` arms are mechanism cells only (N15, A12).

### 4.7 Dreamer-native conversion (`baselines/rl/to_dreamer_native.py`)
`action[t] = actions_delta[t−1]` (zeros at 0); `reward[t] = rewards[t−1] × terminal_reward` (zeros at 0); `is_terminal[-1] = terminated[-1]` (cap-truncated → False → bootstraps); `discount = 1 − is_terminal`; `is_first/is_last`; images T = n+1; refuses shaping, rewards ∉ {0,1,2}, positive reward before the terminal, repeat-stamp mismatch, mixed cap; writes `repeat.json` (action_repeat, contract, delta_cap, scope, terminal_reward, src_sha, src_manifest_sha, census) (`:49-83,111-131,146-156`). r2dreamer dirs built with `--terminal-reward 1` (prefill multiplies by reward_scale 100, asserted); dv3 dirs with `--terminal-reward 100` — same effect, different file (`SESSION_LOG:100,111`; `r2dreamer_final_rr.patch` hunk 3).

---

## 5. Learners

### 5.1 DP — lerobot Diffusion Policy (state-only)
| knob | value | pointer |
|---|---|---|
| Code | lerobot fork `dirkmcpherson/lerobot` branch `genesis-fixes` (lerobot 0.4.5 + image_writer mkdir fix; cluster checkout `b63920e`), `torchcodec 0.3.*` | `cluster/install_lerobot.sh:17-26,49,60-68` |
| Inputs | `observation.state` (8) + `observation.environment_state` (9); no images (`cameras none`) | `sbatch_dp.sh:82-85,420`; `convert_to_lerobot.py:81-90` |
| Action | 7 = 6 absolute joint targets (window end) + grip 0..1; fps **7.5** | `convert_to_lerobot.py:42-51,90` |
| Train | `lerobot-train --policy.type=diffusion --seed=$SEED --batch_size=64 --steps=100000 --save_freq=20000` | `sbatch_dp.sh:187,198,203-204,448-454` |
| DiffusionConfig (lerobot defaults, nothing overridden) | n_obs_steps 2; horizon 16; n_action_steps 8; drop_n_last_frames 7; normalization MIN_MAX (state, action); UNet down_dims (512,1024,2048), kernel 5, n_groups 8, diffusion_step_embed 128, FiLM; DDPM 100 train timesteps, `squaredcos_cap_v2`, β 1e-4→0.02, ε-prediction, clip_sample 1.0; num_inference_steps = 100; Adam lr 1e-4, betas (0.95, 0.999), wd 1e-6; cosine schedule, warmup 500; vision backbone not instantiated | cluster fork `src/lerobot/policies/diffusion/configuration_diffusion.py:104-163`; `modeling_diffusion.py:204-207` |
| Checkpoints | K=5 at 20k…100k, each with `dp_sidecar.json` (action_repeat 4, demo_sha, sim_variant, git, node, config) | `sbatch_dp.sh:459-482` |
| Execution | hold-4 through the env integrator (§3.3); chunk of 8 decisions between replans; sampled actions, diffusion noise seeded per episode (`--seed k`); policy on GPU if visible | `wandb_eval.py:105-109,271-326`; `dp_runner.py:39-50`; `eval_sweep.sh:118-128` |
| Selection | `cluster/dp_select_confirm.sh`: score 5 ckpts on `sel`, best (ties → later, `-ge`) confirmed on `hold+rnd`, final also on `hold+rnd`; `DP-HEADLINE` line + `sweep/HEADLINE.txt` | `dp_select_confirm.sh:23-50`; PREREG A7 |
| Seeds | old world (matched_v2): dH/dDP/dR2D × **10–14** (n=5); corrected (matched_w3): dH/dDP × **20–29** (n=10); N7 density arms 20–22; N14 halves 40–42; pilots s0/s1 | `M/RUN_REGISTRY.jsonl`; `results_table.py:10-20` |
| Not run | the PREREG §1 300k "epoch-matched view" (no registry row, no sbatch path) **[CHECK 19]** | `PREREG:32` |
| Resources | `-p gpu,preempt --requeue --gres=gpu:1 --constraint="l40s\|a100" -n 8 --mem 48g --time 14h` | `sbatch_dp.sh:154-167` |

### 5.2 RLPD — SB3 SAC subclass (`baselines/rl/train_rlpd.py`, `rlpd_sac.py`, `cluster/sbatch_rlpd.sh`)
| knob | value | pointer |
|---|---|---|
| Base | stable-baselines3 2.8.0 (asserted) | `rlpd_sac.py:38-40`; `sbatch_rlpd.sh:240` |
| Critic ensemble | E = **10** LayerNorm critics (vectorised `EnsembleLinear`), LN after each hidden Linear, shared affine (`--per-member-ln off`) | `train_rlpd.py:58`; `rlpd_sac.py:91-126,372` |
| Target subset | Z = **2** random target critics, min → Bellman target, redrawn every grad step | `train_rlpd.py:60`; `rlpd_sac.py:258-260` |
| Networks | π (256,256), Q (256,256) | `rlpd_sac.py:345,371` |
| UTD | **10** critic grad steps per decision; actor + α once per decision | `train_rlpd.py:55`; `rlpd_sac.py:240-241,282-301,363-364` |
| Batch | 256 = **128 online + 128 demo** (symmetric sampling from an immutable demo buffer) | `train_rlpd.py:61`; `rlpd_sac.py:232,243-249,360` |
| γ | **0.998** | `train_rlpd.py:99` |
| Entropy | `ent_coef auto`, target −dim/2 = −3.5, entropy backup **off** | `train_rlpd.py:63,103,107`; `rlpd_sac.py:269-271,350-352` |
| lr / τ / buffer / learning_starts | 3e-4 / 0.005 / 300 000 online / 1 000 | `rlpd_sac.py:357-361` |
| Budget | **100 000 decisions = 400 000 sim steps** (`BUDGET_UNIT=decisions`); train horizon 1200 sim steps; eval horizon 1200 | `sbatch_rlpd.sh:33,80-83,89` |
| Env | `FullTaskEnv(scope='pick', action_mode='delta_joint', action_repeat=4, delta_ref='target', pick_shaping=…, pick_shaping_gamma=0.998, pick_shaping_terminal_zero=True)`, cap 0.025, leash 0.125 | `train_rlpd.py:238-246` |
| Demo format | `--demo-format native`: `(states[t], actions_delta[t], rewards[t] (+shaping from eef_pos when dense), next = states[t+1] or final_state, done = terminated[t])`; truncation bootstraps; sim_variant/repeat/cap/leash/delta_ref stamps checked; `--demo-terminal-guard on` for legacy tapes only | `train_rlpd.py:145-160,307-312`; `sbatch_rlpd.sh:191` |
| Dense | `--pick-shaping on --demo-shaping on --pick-shaping-terminal-zero on` (§3.6) | `sbatch_rlpd.sh:186-190` |
| Demo RNG | `torch.Generator().manual_seed(seed)` (post-2fbed2a fix) | `rlpd_sac.py:179-182` |
| Checkpoints | K=5 via `ArchiveCheckpointCallback` at 0.2…1.0 of steps → `ckpt_020…ckpt_100/rlpd_ckpt.zip` + `.action_mode.json` sidecar; `rlpd_final.zip` | `train_rlpd.py:171-175,440-471,479-480` |
| Eval action | deterministic (`predict(deterministic=True)`), policy on CPU | `wandb_eval.py:236,263`; `eval_sweep.sh:126` |
| Selection | same as DP (5 × sel → best, ties → later → hold+rnd; `SWEEP-HEADLINE`) | `sbatch_rlpd.sh:248-306` |
| Seeds | old world (matched_v2): dH/dDP/dR2D × sparse+dense × **10–13** (n=4); dDPfails, dR2DDPfails sparse 10–13; corrected (matched_w3): dH/dDP × sparse+dense × **20–29** (n=10). (Task brief's "10–14" applies to DP, not RLPD **[CHECK 20]**) | registry |
| Resources | `-p gpu,preempt --requeue --gres=gpu:1 --constraint="l40s\|a100\|l40\|h200" -n 8 --mem 48g --time 12h` | `sbatch_rlpd.sh:62-72` |
| Positive control | none passing (PREREG §10 disclosure); recipe tuned under old-world dynamics (N6/N10) | PREREG §10; `PAPER_NOTES.md` N6 |

**UPDATE 08-29 — recipe restart (PREREG A17).** All γ = 0.998 runs (seeds 10–19, 24–29; waves `final`, `w2final`) are
superseded, diagnostic-only (E9): 69/74 diverged (critic loss 10²–10⁵, Q-watchdog trips, hold ≤ 3/15). Fix factorial
(seeds 30–32, old world, sparse): γ 0.99/UTD 10 (**g99**) — dH 3/3 stable (max critic loss 0.016–0.027, 0 trips, hold
14/14/14, rnd 20/21/19); dDP hold 10/13/14, rnd 14/12/20 with 2/3 seeds diverging (max critic loss 22, 84). γ 0.99/UTD 5
same picture; γ 0.998/UTD 5 diverges → γ is the cause. **Matrix recipe = γ 0.99, UTD 10, E 10, Z 2, 50/50 demo batches,
LayerNorm critics, `ent_coef auto`, target entropy −3.5** (Ball et al. 2023 values except UTD 10 vs 20). Seeds (fresh ids,
A9): dH/dDP sparse 30–37; dHHfails/dDPfails sparse 30–37; dH/dDP dense 30–35; dHsucc_dup/dDPsucc_dup sparse 30–33.
Launcher knobs `GAMMA`, `UTD` (`cluster/sbatch_rlpd.sh`, registry keys). Dense arms relabel demo rows with the run's γ at
load (`train_rlpd.py:311`) — γ 0.998 and 0.99 dense runs never share a table. Q-watchdog is warn-only (`rlpd_sac.py:318-324`)
and its 2.0 threshold is invalid on dense arms (potential shaping lifts Q); health on dense arms = max critic loss +
final ≈ selected. Statistic (A16, pre-registered before any s33–37 readout): divergence rate (Fisher) + LAST-checkpoint
rnd-30 success (Welch + exact permutation, `analysis/stats.py`) — result 08-29 20:00: LAST rnd dH 0.700 vs dDP 0.494
(+0.206, CI [+0.105, +0.306], p 0.001), divergence 1/8 vs 6/8 (Fisher 0.041), see RESULTS_for_writing §2; hold is reported, not the statistic (dH at ceiling
14/15; hold ⊂ training ICs, A8). Time limit 18 h (one 12 h job lost its final-ckpt eval on a slow node). Env fix
timing: jobs started before 08-28 13:25 carried the +2 double pick grant (`full_env.py`, `n_double_grant` normalised in
the native loaders) — g99 runs are all post-fix.

### 5.3 r2dreamer (world model, pixels)
What it is: R2-Dreamer (Morihira et al., ICLR 2026) — a decoder-free, augmentation-free DreamerV3-family world model with a Barlow-Twins-style redundancy-reduction representation loss, shipped with its own PyTorch DreamerV3 reproduction (`model.rep_loss ∈ {r2dreamer, dreamer, infonce, dreamerpro}`); vs DreamerV3: no decoder, BlockGRU (8 blocks), barlow loss 0.05 (`lambd 5e-4`), and a **second, replay-based critic loss** (`repval`, weight 0.3) that backs up a λ-return along the recorded trajectory (demo/fail tapes included) bootstrapped with the imagined return (`C/r2dreamer/README.md`; `_base_.yaml:23,36,45,51,76`; `dreamer.py:539-563`, disc `:549`, clamp `:552-553`; `AUDIT_results §1`). Port: cluster tree at commit `c25eb3b` + working-tree edits captured in `cluster/patches/r2dreamer_final_rr.patch` and `cluster/r2dreamer_port.tar.gz` (`AUDIT_sources_2026-08-23.md:248-287`).

Patch (`cluster/patches/r2dreamer_final_rr.patch`): (1) `envs/genesis.py`: shaping γ from a ctor kwarg (`pick_shaping_gamma`, was hard-coded 0.999) with φ(terminal)=0, plus `R2D_SIM_VARIANT` hook; (2) `envs/__init__.py`: passes `1 − 1/horizon` = 0.997; (3) `demo_prefill.py`: native stride-4 ingestion (`repeat.json` stamp must equal `action_repeat`, `demo_downsample=1`, terminal reward 1.0 × reward_scale); (4) `eval_genesis.py`: `--ic-file/--ic-set/--ic-index`, labelled outputs.

| knob | value | pointer |
|---|---|---|
| Config | `configs/env/genesis_pick_v5d4c_delta_shaped.yaml` (dense; sparse = same without `_shaped`) | cluster yaml; `sbatch_r2dreamer.sh:190` |
| Budget | **3e6 sim steps** (incl. prefill) = 750k decisions; 6 CPU worlds | yaml `:33-34` |
| action_repeat / action space | 4; delta_joint cap 0.025, leash mult 5; actor `bounded_normal_clipped` (min_std 0.1, max_std 1.0) | yaml `:36,46-51`; `_base_.yaml:172-174`; `METHODOLOGY.md §7.3` |
| time_limit | **400 sim steps (100 decisions)** for every result cell (`TIME_LIMIT=400`); the PREREG §8 pilot at 1200 failed (0/2 seeds at 2e6) → PREREG's stated on-fail action "keep 400" — amendment not formally logged **[CHECK 21]** | yaml `:40`; `sbatch_r2dreamer.sh:240`; `CRITIQUE_decisions Q6` |
| Replay | `buffer.max_size 450000` rows, FIFO (`LazyTensorStorage`); `demo_duplicate 4`; `demo_reinject_every 150000` online steps (→ 19 re-injections/run; ring share ≈10 % / ≈27 % fails arm); buffer not persisted across requeue | `sbatch_r2dreamer.sh:307`; yaml `:59-60`; `buffer.py:15`; `AUDIT_results §1` |
| reward_scale | **100** (env and demo) | yaml `:61` |
| train_ratio | **512**; batch 16 × 64; pretrain 1000 | yaml `:39`; `configs.yaml:17-18,30` |
| horizon → discount | 333 → **0.997**; imag_horizon 15; λ 0.95; return_clamp 100 | yaml `:65,82`; `_base_.yaml:17,19`; `dreamer.py:494,502-503,549` |
| actor_bc_lambda | **0** (demos consumed as data only, by design) | yaml `:52` |
| act_entropy | 3e-5 | yaml `:66` |
| Losses | barlow 0.05, rew 1, con 1, dyn 1, rep 0.1, policy 1, value 1, **repval 0.3**; kl_free 1.0; unimix 0.01 | `_base_.yaml:16,35-48,79` |
| Model | size12M: deter 2048, hidden 256, discrete 16, stoch 32, cnn depth 16, units 256, SiLU, norm; symexp-twohot 255-bin reward & critic heads; CNN encoder only (`cnn_keys: image`) | `configs/model/size12M.yaml`; `_base_.yaml:68-81,134-146,185-197`; yaml `:68-73` |
| Optim | lr 4e-5, AGC 0.3, warmup 1000, Adam (0.9, 0.999); slow target update every step, fraction 0.02 | `_base_.yaml:25-33` |
| Demo sets | `baselines/matched_v2/r2d/<ARM>` (dreamer-native, terminal reward 1), stamp gate | `sbatch_r2dreamer.sh:204-259` |
| Selection | every `latest.pt` archived and scored in-job on `sel` (15 episodes **in one process**, `--mode sample --seed 0 --device cpu`) → `ckpt_scores.tsv`; `BEST = sort -rn | head -1` → `BEST_selected.pt` (byte copy, confirmed); pruning keeps best-2 + newest + K=5 fraction ckpts. Tie rule is `sort`-order, not "later" **[CHECK 22]** | `sbatch_r2dreamer.sh:444-548` |
| Protocol readout | `cluster/r2d_rescore.sh`: `BEST_selected.pt` on `hold` and `rnd`, one fresh process per episode, CPU, `PAR=4`, `--max-steps 1200`, asserted denominators, non-zero exit on missing → `RESCORE-RESULT` (replaces an inline loop that printed DONE over 540 crashed processes) | `r2d_rescore.sh:1-53`; commit `bf09060` |
| Seeds (all old world, matched_v2, dense, TL 400) | dR2D 80–93; dR2DDPfails 80–93 (s83 lost, A9); dH 99–108 (s102 → s108); dDP 100–107; controls dR2Ddup13 200–203, dR2DDPsucc 300–303; earlier 08-19 cells dH 50–53 (dense), dR2D 40–43 (sparse); corrected-world pilot 60/61 (TL 1200) | registry; `SESSION_LOG:167,173,179`; N16 |
| Not run | PREREG §1 sparse arm and 6 seeds per source at TL 1200 (dense only, TL 400 ran) **[CHECK 21]** | `PREREG:35` |
| Resources | `-p gpu,preempt --requeue --gres=gpu:1 --constraint="l40s\|a100\|l40\|h200" -n 8 --mem 48g --time 2d` (relaunches with `--time 24:00:00`); separate py3.11 venv | `sbatch_r2dreamer.sh:162-177` |

**UPDATE 08-29 — what is standard.** The replay-buffer critic loss (`loss_scales.repval 0.3`, `dreamer.py:539-563`)
matches the official DreamerV3 v2 recipe (`beta_repval 0.3`, danijar/dreamerv3; `paper/WM_CANDIDATES_2026-08-29.md`);
only the imagined-return clamp (`return_clamp 100`, `dreamer.py:502-503, 552-553`) is a deviation (E8 corrected).
**Standard arm = `env.return_clamp=0`, repval kept** (runs `pick_v5d4c_delta_shaped_NOCLAMP_{dH,dDP}_s12{2,3}`, packed).
The earlier "STD" attempt (clamp off + repval off, s120/121) died at launch on a non-existent hydra key and is void.
LAST-checkpoint hold re-scores (21 runs, `baselines/outputs/n12_rescore/*_LAST_hold_*`): 0/15 or ≥10/15 in 16/21 —
final policies are bistable; BEST-of-K selection on `sel` (training ICs) does the work and must be disclosed as such.
dup13 pilot (s202/203, sel-only): 0.93 / 0.00. Corrected-world gate pairs dH/dDP s80–83 (`DEMOSET=w3`, `gc_kp4_riser3_shelf6`, dense, 3M): **ignites** — sel max per seed
dH 0.40/0.93/0.93/0.93, dDP 0.00/0.87/0.13/0.93, bistable across adjacent checkpoints as in the old world; protocol
re-scores (BEST hold/rnd, LAST hold; tags `_W3`): dH BEST hold 10/15/11/10, rnd 20/25/22/16; dDP 1/2/2/15, 1/2/3/28
(n 4 v 4, direction human > DP, perm p 0.143); seeds 84–87 per source added 08-29 (4-seed packs, `-n 32`). Under A19 the WM arm reports in one named world per
table; the w3 run dirs carry no world in their name (seeds 80–83 of dH/dDP = corrected world only).
Packing: `cluster/sbatch_r2dreamer_pack.sh` (2 runs/GPU, 8 threads each, 16 CPUs, 96 GB; >2× per-GPU throughput, same
launcher/config per run; 4/GPU with `-n 32` on 64-core nodes from 08-29). Touchgoal probe (no demos, `DEMO_DIR=null`
now accepted by the launcher) s0/1 packed.

### 5.4 dv3 — DreamerV3 (dreamerv3-torch-genesis) — documented negative
Patch `cluster/patches/dreamerv3-torch-genesis_final_rr.patch`: converter refuses contract-v1 dirs / `--terminal-guard on`; `genesis_eval.py` gains `--ic-file/--ic-set/--ic-index/--device` and records `act_selection='deterministic'`; overlay `genesis_final_rr: time_limit 1200, steps 1e6`; new `sbatch_genesis_final_rr.sh` (K=5 archive, fresh-process eval sel → hold+rnd, `DV3-RESULT`). Config: action_repeat 4; time_limit 1200; steps 1e6 sim (250k decisions = 62.5k updates); train_ratio **256**; batch 16 × 64; prefill 2500 / pretrain 100; reward_scale 100; discount 0.997, λ 0.95, imag_horizon 15; RSSM deter 512, hidden 512, stoch 32×32, GRU blocks 8, units 512; model_lr 1e-4; dense variant `genesis_pick_shaping: True` γ 0.997 (`C/dreamerv3-torch/configs.yaml:40-58,94,101-104,384-492`). **Status: the final-RR dv3 block never ran** (registry: 0 rows of `sbatch_genesis_final_rr.sh`; one 2e4 smoke). The dv3 negative (N4: transient takeoffs at 140–320k, no sustained ignition) rests on the 08-19 `sbatch_genesis_multi.sh` 300k runs (dH s0–2, dDP s0–1, dR2D s0–1), whose harness is outside this repo **[CHECK 23]** (`PAPER_NOTES.md:87-92`; `CRITIQUE_launch_plan_2026-08-25.md:415`; `PREREG:37`). Earlier history: unnormalised demo actions poisoned runs v6–v13 (`METHODOLOGY.md §6.3`).

**UPDATE 08-29 — no longer "documented negative"; mechanism + standard arm.** Diagnosis (`paper/DV3_DIAGNOSIS_2026-08-28.md`):
on every unclamped run `value_mean` runs past the attainable return (dense ×100: 263–390 by 300k on 3/3 5M baselines;
touchgoal-from-scratch s0 learned to 0.82–0.96 success at 230–440k then collapsed as value crossed 100) — critic runaway
via bootstrapped reward-head leak; the torch port (NM512, pre-v2) lacks the official replay critic loss / EMA regulariser.
Arms (dH dense, seeds 20–22 at 1M; launcher `EXTRA_CONFIGS`/`TAGSUF`): **`genesis_dv3std`** = `genesis_reward_scale 1`
(symlog/two-hot as published), `return_clamp 0`, `train_ratio 512`, `time_limit 400`, demos = `matched_v2/r2d/dH`
(terminal 1.0, grant_slack 0) — the ONLY arm eligible for the matrix (A15); `genesis_dv3clamp` (clamp 100, diagnostic);
`genesis_dv3fix` (clamp + train_ratio 512 + TL 400, diagnostic). std s20: first picks ever (train success 0.22 @284k) with
value bounded (~29); clamp s20: 0.25 with value pinned at 98. 3M std pack (s23–25, `cluster/sbatch_dv3_pack.sh`, 3 runs/GPU,
375k updates ≈ r2dreamer's 373k) with an `afterany` resume chain (dreamer.py re-enters `latest.pt`). Disclosures: (i) every
dv3 job before 08-28 21:30 ran WITHOUT its overlay (launcher omitted `${EXTRA_CONFIGS}` on the real train/eval lines) — the
5M baselines were unaffected; (ii) two other launcher defects (env python, relative demo dir) killed the 08-28 morning
batch at start; (iii) `models.py` clamp is flag-gated (`return_clamp`, default 0). Eval uses `actor.sample()` labelled
deterministic (mislabel, measurement only). Speed: ~47k sim steps/h (train_ratio 512) unpacked.

### 5.5 SACfD (legacy)
`baselines/rl/train_sacfd_full.py` — demo-seeded SB3 SAC (no PER, no n-step, FIFO eviction, staged reward, 400k steps). Superseded by RLPD; retained only as history (absolute-action fling artefact; hardened predicate) (`train_rlpd.py:1-22`; `AUDIT_sources_2026-08-23.md:374`; `METHODOLOGY.md §6.2`).

### 5.6 Cross-learner invariants
All learners: delta_joint, cap 0.025, leash 0.125, repeat 4, sparse terminal +1 (×100 for WMs), tip rule, same tapes (per world), same IC file, K=5 checkpoints, selection on `sel`, headline on `hold+rnd`, 1200-step eval horizon. Differences that remain (disclosure, PREREG §10): state vs pixel modality; per-learner budget and unit (DP 100k grad steps; RLPD 100k decisions; r2d 3e6 sim steps; dv3 1e6); r2d training horizon 400 vs 1200 for the others; r2d/dv3 have replay critics that regress through recorded fail states; recipes tuned on dH; RLPD has no passing positive control.

---

## 6. Evaluation protocol

### 6.1 Initial conditions — `baselines/eval_ics.json` (generated once by `baselines/make_eval_ics.py`, seed 0, git dd58c15)
| set | content | pointer |
|---|---|---|
| `sel` (15) | demo uids **232 234 235 236 237 239 242 243 244 245 246 247 248 250 251** (= the historic first-15 demo ICs) — selection only | `eval_ics.json`; `make_eval_ics.py:46,91` |
| `hold` (15) | **252 254 256 265 273 276 284 294 295 302 311 325 327 331 335**, drawn `default_rng(0)` from the 46 remaining success uids (61-uid universe), disjoint from sel — **confirmation split of demo ICs, not held-out** (11/15 are training ICs of a 51-demo set; for RLPD every sel/hold uid is a training reset IC; A8) | `make_eval_ics.py:55-58,94-97`; `PREREG:277-280`; `AUDIT_prelaunch:114-118` |
| `rnd` (30) | support ICs: can xy ~ Uniform over the solved-demo bounding box ±1 cm, lo (0.3395, −0.2370), hi (0.592, 0.1887), upright, z 0.113; fixed goal; the only novel ICs | `ic_sampling.py:12-34`; `eval_ics.json support_box` |
| Ceiling | uid 234 (sel[1]) is a lying-can IC: picked **0/430** policy-evals across 22 DP runs → `sel` max = 14/15 = 0.933; `hold` has ~1 count of headroom; BC source claims therefore read on `rnd` (N11). N11's "tip rule fires at decision 1" wording describes the training env, not the eval env (no tip termination there) **[CHECK 24]** | `PAPER_NOTES.md:250-255`; `AUDIT_INDEX.md:51` |

### 6.2 Harness (`cluster/eval_sweep.sh` → `baselines/wandb_eval.py`; r2d via `eval_genesis.py`/`r2d_rescore.sh`)
- One episode per fresh process (`--ic-file --ic-set --ic-index k --seed k --no-wandb --max-steps 1200`), ≤1 world per 2 cores (`-n 8` → 4 concurrent), resume-safe; missing episodes reported as MISSING, never 0; denominators asserted from the IC file; `SWEEP-RESULT` line is authoritative (`eval_sweep.sh:37,60-66,114-166`).
- Sim always CPU (`GenesisCanEnv(backend='cpu', max_steps=1200)`); DP policy on GPU when visible, SAC on CPU (A4); sim_variant from CLI > sidecar > base, mismatch refuses; action mode/repeat/delta_ref from the checkpoint sidecar (no silent defaults) (`wandb_eval.py:122-152,171-210,281-326`).
- **Pick metric at eval** = the hardened predicate of §3.5 computed by `GenesisCanEnv` (no settle); episodes run the full 1200 steps (`done = t ≥ max_steps` only — **no tip termination at eval**); `placed`, `contact`, `nested` (100-step settle) also recorded but not used for pick-scope headlines (`genesis_can_env.py:255-294`; PREREG §10).
- Action selection: RLPD deterministic; DP sampled with per-episode seed; r2d sampled (`mode` reported alongside); dv3 deterministic — recorded in the result JSON (`act_selection`) with node class, git, sim_variant, ckpt step, n_steps, per-episode records (`wandb_eval.py:436-449`).
- r2d selection evals run 15 episodes in one process on `sel`; the protocol readout is the separate fresh-process re-score (§5.3).

### 6.3 Selection and headline rule (PREREG §5, A7)
K=5 checkpoints at 20/40/60/80/100 % of budget; selection = best `sel` (ties → later for DP/RLPD); **headline per seed = the selected checkpoint on `hold` (15) + `rnd` (30)**, reported by name ("sel / hold / rnd"), plus final-checkpoint hold/rnd and time-to-first-training-pick. Results are quoted only from `analysis/results_table.py` output (`SWEEP-HEADLINE`, `HEADLINE.txt`, `RESCORE-RESULT`; `R2D-RESULT` rows are flagged sel-only), deduplicated on (arm, seed), smoke seed 0 excluded, world inferred from wave/seed block (`results_table.py:10-40,64-74`).

### 6.4 Seeds per cell (registry mirror as of 08-28; `RESULTS_TABLE_2026-08-25.md` shows the older n=4/5 snapshot **[CHECK 25]**)
| learner | world | reward | arms | n |
|---|---|---|---|---|
| DP | old | – | dH, dDP, dR2D | 5 each (10–14) |
| DP | corrected | – | dH, dDP | 10 each (20–29) |
| DP | corrected | – | dHallpruned_1e3/1e2, dDPallpruned_1e3/1e2 (N7) | 3 planned (1–3 landed) |
| DP | corrected | – | dH_A/B, dDP_A/B (N14) | 3 planned (2/12 landed at audit time; resubmitted 08-28) |
| RLPD | old | sparse | dH, dDP, dR2D, dDPfails, dR2DDPfails | 4 each (10–13) |
| RLPD | old | dense | dH, dDP, dR2D | 4 each |
| RLPD | corrected | sparse, dense | dH, dDP | 10 each (20–29) |
| r2dreamer (RESCORE) | old | dense | dR2D / dR2DDPfails | 4 / 3 at audit (80–83 / 80–82; more re-scores in flight; s83 fails lost) |
| r2dreamer (sel-only) | old | dense | dH, dDP | 3 / 4 (not reportable) |
| dv3 | — | — | — | 0 at the final-RR budget |

### 6.5 Statistics (PREREG §7 as amended by the 08-28 audit)
- Unit = seed; primary statistic = per-seed success count on `hold` and on `rnd` of the selected checkpoint; paired by seed index within a learner; no pooling of episodes across seeds (episode-pooled Fisher tests are invalid and must not be quoted).
- Tests: Welch t on seed means with **Welch–Satterthwaite df** (N15: hold df 4.52 → CI [−0.259, +0.726]; rnd df 4.17 → [−0.228, +0.556]); exact two-sided permutation test (N15 p = 0.286; **minimum attainable p at 4 v 3 = 2/35 = 0.057**, at 3 v 3 = 2/20 = 0.100); 95 % bootstrap CI on the mean difference as effect size; hierarchical Beta–Binomial P(A>B) as the reported posterior; two primary contrasts per learner → Holm; everything else BH q = 0.1, labelled exploratory (`PREREG:172-175,194-201`; `AUDIT_results §1`).
- Run-identity rule A9: a seed id names one run; a rerun replaces; earlier readouts dropped and listed as lost.
- **[CHECK 26]** the only statistics code in `analysis/` is `bayes_source_effect.py` (08-08 wave, wandb `eval_indist/eval_random` keys) and `results_table.py` (means only); the Welch/permutation/Holm computations quoted in the notes were done ad hoc and are not scripted.

---

**UPDATE 08-29.** `analysis/stats.py` (Welch with Welch–Satterthwaite df and the correct critical value, exact permutation
p under both conventions, minimum attainable p; self-test reproduces the corrected N15 CIs) resolves **[CHECK 26]**. RLPD
statistic per A16; WM tables per A19 (one world per table); fails effect per A17 reading rule.

## 7. Compute, provenance, incidents

| item | value | pointer |
|---|---|---|
| Cluster | Tufts HPC (Slurm); GPUs L40S / A100 / L40 / H200; one GPU per training run; Genesis worlds CPU-side; partitions `gpu,preempt --requeue` for training, `gpu` for relaunches, `batch` for CPU re-scores (`-c 8 --mem 24g --time 4h`); per-user QOS cap **10 GPUs**; long jobs submitted with `--time 24:00:00` (no documented 24 h hard cap **[CHECK 27]**) | `sbatch_*.sh` headers; `sbatch_r2dreamer_pack.sh:5-6`; `SESSION_LOG:165,173,178` |
| Environments | one pip-only conda env `C/condaenv/genesis` for genesis + lerobot + SB3 (verified by `cluster/verify_env.sh`, incl. H200); r2dreamer venv `C/r2d_venv` (py3.11, torch 2.8) | `verify_env.sh:24-58`; `install_r2dreamer.sh` |
| Dev box | Ryzen 5950X / RTX 3080Ti / 62 GB; pilots and labs only; official numbers never mixed across machines | `METHODOLOGY.md:380-383` |
| Run registry | `cluster/run_registry.py` → `cluster/RUN_REGISTRY.jsonl` (cluster copy authoritative, 245 rows 08-18→08-28 in the mirror; local file is empty **[CHECK 28]**). Row fields: `script, arm, seed, knobs, demo_fingerprint (sha of sorted (filename,size)), git, semantic_key, full_key, timestamp, slurm_job_id, duplicate_ok_reason, stage, node`; registered at job START; full-key duplicate → `REGISTRY-REFUSE` unless `DUPLICATE_OK=<reason>`; semantic duplicate → `REGISTRY-WARN` (no `warn` field written yet, contrary to A9 **[CHECK 29]**) | `run_registry.py:12-18,57-67,92-102,156-216`; `sbatch_rlpd.sh:203-235`; `sbatch_dp.sh:352-381` |
| Registry knobs (RLPD) | steps, budget_unit, scope, action_mode, delta_ref, action_repeat, train/eval horizon, gamma, backup_entropy, per_member_ln, pick_hold_reward, utd, ensemble_size, subset_size, demo_batch, reward, pick_shaping, demo_format, demo_sha, wave, demo_shaping, terminal_zero, terminal_guard, sim_variant | `sbatch_rlpd.sh:203-208` |
| Session command log | `paper/SESSION_LOG_2026-08-23_cluster.md`: every create/run/submit/cancel/delete cluster command with EDT time, host, repo git sha at run time, exact command, exit code, first output line (146 rows + 21 NOTE bullets) | `SESSION_LOG:42-43` |
| Artifact mirror | `~/workspace/final_rr_artifacts_2026-08-24/` (9.7 GB): demo recordings + manifests, `matched_v2` (incl. `r2d/`, `dR2DDPsucc`), `matched_w3`, DP selected checkpoints and sidecars, RLPD checkpoints (+meta), all `.out` files (696), `HEADLINE.txt` (60), `sweep.json` (577), r2d re-score logs, registry, `pull.log` | `ls M/`; `AUDIT_INDEX.md §6` |
| Committed provenance | `paper/final_rr_sets_2026-08-23/` (v1/v2 manifests + censuses); `cluster/patches/` (genesis pin + patch, r2dreamer and dv3 patches); `cluster/RUN_REGISTRY.jsonl` (should mirror the cluster copy) | |
| wandb | entity `jambotime`; projects `genesis_paper` (DP/RLPD train + sweep summaries), `r2dreamer_genesis`, `dreamer_v3`, `genesis_pickaplace` (wandb_eval default) | `eval_sweep.sh:171`; `sbatch_dp.sh:189`; `sbatch_rlpd.sh:200`; memory `monitoring-box-setup.md` |
| Code identity | PREREG §2 "one commit hash per block; a fix restarts that learner's block" — the block spanned several shas (e.g. v2 sets ca05472, w3 sets 2061f08, later relaunches 0440a7f/bf09060); every registry row and manifest carries its sha, so per-run identity is exact even though the block-level rule was not kept **[CHECK 30]** | `SESSION_LOG` sha column |

Incidents affecting data (all logged; report in the paper's limitations):
1. **Disk incident (08-27)**: K=5 archiving shipped with no retention → `baselines/outputs` = 909 GB, shared volume 100 %; jobs died with bare exit codes (the same signature previously attributed to preemption/CUDA transients — that diagnosis is partly invalid); 176 checkpoint dirs pruned keeping selected + final of every completed run; 489 GB recovered; of the 20 DP resubmits of 08-27 02:13, 8 completed and 12 failed on the full volume; N14 had 2/12 and N7 9/12 landed before resubmission 08-28 (`SESSION_LOG` NOTE 08-27; `AUDIT_results §0,§6`; commit `30195ee`).
2. **Seed overwrite (A9)**: the 08-27 20:15 relaunch reused seed ids of completed runs — dR2DDPfails **s83** (sel 0.80, BEST written) overwritten in place and unrecoverable; dH s102 in-place relaunch cancelled 08-28 04:30 and resubmitted as **s108**; dDP s101 relaunch never ran. Pooled statistics drop the lost readouts (`PREREG A9`; `SESSION_LOG:178-179`).
3. **Over-broad `scancel --name=r2d-train --state=PENDING` (08-26)** killed the N13 and N12 top-up waves; 11 relaunched with `DUPLICATE_OK=relaunch-after-overbroad-scancel` (`SESSION_LOG:162-164`).
4. **Re-score false success**: an inline re-score loop printed DONE over 540 crashed processes (no exit-code check; missing `GENESIS_PICKAPLACE_ROOT`); replaced by `cluster/r2d_rescore.sh` with asserted denominators (`r2d_rescore.sh:11-15`; commit `bf09060`).
5. **E3 exposure arm cancelled** (BUFFER_MAX=40000 confounded staleness/capacity/reinject shock; 10.1 % vs 34.2 % demo share) — replaced by the N16 controls (`AUDIT_INDEX.md §4`).
6. **Audit-day duplicate submissions**: 24 DP jobs submitted with a wrong guard path, 11 duplicates cancelled before start (`AUDIT_results §6`).
7. Earlier instrument failures that shaped the protocol (fling exploit, env-vs-replay #26 horizon bug, unnormalised dv3 demos, eval action-mode default, six silent-default bugs): `paper/METHODOLOGY.md §8.4`; `CLAUDE.md`.

---

**UPDATE 08-29.** GPU packing (CPU-bound Genesis, 0 % GPU util): r2dreamer 2–4 runs/GPU, dv3 3 runs/GPU, RLPD unpacked
(GPU-bound at UTD 10). QOS cap 10 GPUs; `gpu` MaxTime 2 d; `scontrol top` denied → queue order via `scontrol hold` +
`--begin` release jobs. Unattended readouts: `cluster/harvest_readout.sh` on the batch partition at +3/+20/+44/+62 h →
`paper/harvest_<ts>.md`. Incidents 08-28/29: three silent pack deaths (40 s, found ~8 h later) → rule: check every pack
`.out` for `rc=1` within the first hour. Fallback WM (A18): DEMO3 codebase (TD-MPC2-based; MoDem / TD-MPC2 by flags) prepared
locally (`~/workspace/demo3_prep`, converter round-trip verified); `policy_pretraining=false` flag set is actor-BC-free.

## 8. [CHECK] list (every uncertain or conflicting value)

1. **Recording rate**: "30 Hz" (docs) vs ~31.6 Hz (uid 232 `_episodes.npy`, 919 frames/29.06 s) vs ~40 Hz `joint_states` / 1159 frames in `_cartesian.npy` for the same trial; timestamps not saved (`example.py:276`; `CAN_STARTING_POSITION.md:188`).
2. **`vel_cmd` semantics**: the stored arm stream is measured `/joint_states` position, not a velocity or a command; several docs say "commanded joint targets". Fix wording throughout (`trial_reader.py:69-74`; `METHODOLOGY.md:233-235,281-284`).
3. **Real camera resolution/model**: not documented anywhere.
4. **93 vs 96 trials**: uids 253, 285, 289 (fail list) on disk but absent from `trial_placements.json`/manifests, reason undocumented; also `kinova.py` lists 322 and 331 as both success and fail; "All 224 bags are local" (`HANDOFF_PLAN.md:87`) vs 96 processed.
5. "~130 recordings" for unlabeled uids 200–231 (`HANDOFF_PLAN.md:91`) — inconsistent with a 32-wide range.
6. **Contact-solver default**: `replay_harness.py:98-99` comment claims 0.2.1 hard-wires timeconst = 2·substep_dt; gripper lab shows this tree keeps 0.02 s and only clamps. Confirm which statement holds for the exact pinned commit 31951c3f.
7. **Finger kp**: `CAN_STARTING_POSITION.md:172-174` says kp 100 "sweet spot"; live world config is `finger_kp 40` (commit e8a07d6).
8. `sim_variants.post_build` does not assert the 5-geom count for `grasp_timeconst` (silent degrade possible while stamping `_ts5`).
9. **Human pick-recreation figures** come from different runs: 51/66 (meas, cluster), 56/66 (either, cluster, base), 57/66 (gripper-lab g_base control) vs 58/66 (ts5, cluster-confirmed), 58/66 (dH_w2 in gc_kp4_riser3_shelf6), 55/66 and 58–62/66 (local follower-lab variants). State which comparison each number belongs to.
10. **ts5 / `matched_w4` fate**: `dH_w4` recorded 08-26 07:50 (ts5) and `matched_w4_pilot/dH` built, but no kept count, census, or DP-pilot readout appears in any note or results table; `matched_w4` proper never built; commit 7c8195d calls ts5 "world of record from 08-26" while all subsequent result launches used `gc_kp4_riser3_shelf6`. The draft states "ts5 = pilot only, no result block".
11. `genesis_can_env.py:10` docstring says 16-dim state; code is 17.
12. **Tip penalty**: code `TIP_PENALTY = 0.0` (pick/full scope) and PREREG §2 r=0; the task brief's "−0.5 in training" is the July Cartesian env only.
13. **Fast-pick 2.0 mechanism**: the "picked + placed grants in the same window" explanation is read from `genesis_can_env.py:265-267`; the only written source is a comment in `to_dreamer_native.py:65-70` ("stage grant AND the hardened-pick terminal inside ONE repeat-4 window").
14. `dHunpruned` control was recorded twice but no result for it exists in any results file (PREREG §1 lists `dHunpruned_DP ×3`).
15. PREREG A1's drop taxonomy ("5 over-horizon, 7 over-cap") vs recorder outcome codes (10 env_truncated, 2 adapter_exhausted, 3 tipped) for the same 15 drops.
16. `sbatch_dp.sh:14,221` says lerobot datasets are "prebuilt by make_matched_sets.py"; they were built by `convert_to_lerobot.py` in separate steps.
17. Split-half set shas not captured locally.
18. The "22 %" pruned fraction at eps 1e-2 exists in no artifact.
19. DP 300k "epoch-matched view" (PREREG §1) never ran.
20. RLPD old-world seeds are 10–13 (n=4), not 10–14; DP old-world seeds are 10–14 (n=5).
21. r2dreamer ran dense only, TL 400, old world — PREREG §1 planned sparse+dense × 6 seeds at TL 1200 (pilot-gated); PREREG §8's on-fail action ("keep 400") was taken but no amendment was logged (CRITIQUE Q6).
22. r2dreamer checkpoint tie-break is `sort -rn | head -1` (not "later"); its in-job selection evals are 15 episodes in one process (not fresh-process).
23. dv3 final-RR block never ran; the N4 negative rests on 08-19 300k runs whose harness (`sbatch_genesis_multi.sh`) lives outside this repo; `results_table.py` has no DV3 parser.
24. N11's "tip rule fires at decision 1" mechanism for uid 234: the eval env has no tip termination; the 0/430 is real but the wording describes the training env.
25. `RESULTS_TABLE_2026-08-25.md` shows n=4/5 for corrected-world cells; the mirror/audit have n=10. Regenerate before quoting.
26. No scripted Welch/permutation/Holm/BH analysis exists in `analysis/`; `bayes_source_effect.py`/`results_significance.md` describe the 08-08 wave.
27. No document states a 24 h cluster wall-clock cap; only `--time 24:00:00` usage is observable (script defaults 12 h / 14 h / 2 d).
28. Local `cluster/RUN_REGISTRY.jsonl` is 0 lines; the mirror copy (245 rows) predates seeds 104–108, 200–203, 300–303.
29. Registry rows carry no `warn` field (A9 requires one); 0/245 rows have it.
30. PREREG §2 "one commit hash per block" was not kept (several shas within each block); per-run shas are exact.
31. E5: the ts5 change also stiffens can↔goal-can and can↔shelf contacts 1.6× — relevant only if any downstream (contact/nested) number from the gripper lab is quoted.
32. `GenesisCanEnv.placed` still tests the un-shifted `BOX_TOP_Z = 0.11` under shelf6 worlds (a correctly placed can scores False); inert for pick scope (`CRITIQUE_decisions:455-461`).
