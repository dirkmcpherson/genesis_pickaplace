# Item 06 — Simulation and real2sim description: audit against the notes and the code

**Status: done (audit + paste-ready text).** Every sentence of `sections/04-evaluation.tex` §Real-World Experiment
(task, Data Collection, Digital Twin, Machine Demonstrations + phase table, Results), the two figure captions in
`03-method.tex`/`01-introduction.tex`, and every number in `appendix/a-research-methods.tex` §real2sim was checked
against the project notes and the code (no simulation was run; one read-only `python3` pass over
`baselines/eval_ics.json` and `can_pos_recovery/trial_placements.json` to count starts). Verdicts and corrected LaTeX
below. `\llm{...}` one-liners were inserted at each corrected paragraph. Two claims cannot be sourced anywhere in the
repository and one is contradicted by it (participants); those are flagged, not silently rewritten. The pixel
world-model training horizon could not be verified from the local tree (config file absent); flagged.

Naming per the brief: task = restocking, "AI" → machine, DreamerV3 → DfD where the paper says so.

---

## A. Sentence-by-sentence table

Verdicts: **correct** / **wrong** / **unsupported** (no source in notes or code; not contradicted) / **imprecise**
(true but needs a qualifier the notes require). "State set" = `dHfull_all_rns10h` (74) / `dDPfull_first_rns10h` (72);
"pixel set" = the `_img` rebuilds the reported learners train on (`HRI_results/DEMO_SETS_2026-09-11.md` §3).

### A.1 `sections/04-evaluation.tex` §Real-World Experiment: Restocking (line 63)

| # | Paper sentence | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 1 | Task has three parts: pick, place, slide into contact with the goal can | correct | Stages `picked → placed_v2 → farside/slide_event → nested_v2`; outcome `home` = `slide_event ∧ nested_v2` | `baselines/rl/full_env.py:74-176`; `DEMO_SETS` §2 |
| 2 | Gripper fingers are wide, necessitating a slide or careful placement | unsupported | No measurement in the notes; consistent with the predicate design (`home` cannot be scored by a drop into the goal) | `DEMO_SETS` §2 (`home` row) |
| 3 | The can started in one of three marked spots | unsupported → consistent | No study document in the repo. The 74 recovered start positions (`trial_placements.json`, success-labelled uids 232–335) fall into three clusters of 26/21/27 with median radius 2.0–2.5 cm (max 5–10 cm); recovery precision is ~1.5–3.5 cm, so three spots + placement scatter is consistent | `can_pos_recovery/trial_placements.json` (k-means, this audit); `paper/CAMERA_GATE_2026-08-31.md` §2 |
| 4 | Goal can was static across all trials | correct (one caveat) | Static goal `(0.672, −0.221)` recovered from the human slide finals and validated by the user; the t0 census shows uid 255's goal can visibly disturbed (that trial never solved) | `baselines/ic_sampling.py:26-30`; `CAN_STARTING_POSITION.md` (2026-07-20); `CAMERA_GATE` §1 |
| 5 | RL received +10 when they slid the can into contact with the goal, 0 otherwise | imprecise | Ladder `nested_sparse10`: +10 once, on `home`, **terminal**; the tip rule (tilt > 60° and not in hand, 4 frames) also terminates with penalty 0. `nested_v2` requires centre distance ≤ 0.081 m (= diameter 0.066 + 1.5 cm), both cans upright, at rest, not in hand — i.e. within 1.5 cm of touching, not literal contact. DP ignores reward | `full_env.py:162-163,181`; `DEMO_SETS` §2, §4; `paper/TIP_RULE_2026-09-11.md` |

### A.2 §Data Collection (lines 67–68)

| # | Paper sentence | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 6 | Demonstrations from **forty non-expert users** | **wrong / unsupported** | "No participant identifier exists anywhere": `config.yaml` byte-identical across all 224 trials, `user_NNN` is a trial counter, no bag topic, METHODS states no count. The 74 tapes are **all from one session, 2024-12-18 12:41–17:51** (104 recordings). One person and several people are equally consistent with the data. The intro (`01-introduction.tex:38`) repeats "forty participants" | `paper/EARLY_TRIALS_ROTATION_2026-09-08.md` §1; `paper/E2E_AUDIT_BRIEF_2026-09-10.md` §11; `DEMO_SETS` §1 |
| 7 | IRB-approved public-space study | unsupported | Not mentioned in any note. Keep only if the authors hold the record; the repo cannot back it | — |
| 8 | Six-joint Kinova Gen3 Lite with a two-finger gripper | correct | `gen3_lite_2f_robotiq_85.urdf`, 6 arm dofs + 4 finger dofs | `paper/METHODS_draft_2026-08-28.md:92` |
| 9 | Xbox controller | unsupported | Notes say "joystick teleop" only | `METHODS_draft:28`; `baselines/cartesian_env.py:3` |
| 10 | Cartesian EE velocity in x, y, z and **pitch; roll and yaw fixed** | imprecise / partly wrong | Commanded twist: `wx ≡ wz ≡ 0` in 74/74 tapes, only `wy ≠ 0` (cap 1.0 rad/s, linear cap 0.11 m/s) — so 4 commanded dof is right. But the commanded `wy` drives the **realized world-z yaw** (r = −0.55) and not realized pitch (r = 0.000), and the plugin's Jacobian controller left roll/yaw **free** (drift up to 1.03/1.34 rad). "Fixed" describes the command, not the motion; "pitch" is the code's label, unverified | `paper/EEF_ACTION_DIST_2026-09-07.md:109-133`; `METHODS_draft:28-29` |
| 11 | Recorded joint states, tool pose, gripper position from ROS, plus overhead video | correct (detail) | Bag topics `/my_gen3_lite/joint_states`, `base_feedback` (tool pose, gripper), `cartesian_velocity`; cameras: cam4 top view of the start, cam0 bottom-up through the translucent shelf; 9 labelled trials have no video (303 among them); no factory calibration | `trial_reader.py:23-25`; `METHODS_draft:41`; `CAMERA_GATE` §1 |
| 12 | We recorded 74 successful trials | imprecise | Session 4 = 104 recordings: 75 operator-labelled successes, 16 labelled failures, 2 stubs (gripper never closes), rest unlabelled. 74 = the 75 successes minus unrecoverable uid 303. "Successful" is the operator's label: 10 of the 74 never pick in sim; at least one (254) shows a human hand handling the can at the end of the real recording | `DEMO_SETS` §1; `METHODS_draft:52`; `CAN_STARTING_POSITION.md`; `paper/CONFOUNDS.md` rows 89, 92 |

### A.3 §Digital Twin (line 71)

| # | Paper sentence | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 13 | Learners requiring online sampling → digital twin; RL on the real robot needs dense reward or human help | correct (framing) | — | — |
| 14 | Built the twin in Genesis, converted the demos, all training and evaluation in the twin | correct | Genesis 0.2.1, world `gc_kp4_riser3_shelf6` (arm PD gains ×4/kv ×2, gravity compensation 1.0, base riser +3 cm, shelf raised +6 cm to top z 0.17), 3 physics steps of 10 ms per env frame, substeps 8 | `baselines/sim_variants.py:67`; `baselines/genesis_can_env.py:14,263-264`; `eval_ics.json` `world_cfg` |
| 15 | "Due to differences between the simulator and true embedded arm controller our learners operate on joint-position control rather than end-effector control" | imprecise (rationale) | The measured reason: replaying the recorded **joystick twist** open-loop reproduces only **17/74** picks (the real Kortex controller realised it with a median 4.2 cm drift by grasp time), while the recorded **joint stream** reproduces 69/74. Learners emit 6-d **delta joint targets** (cap 0.025 rad per env frame, integrated on a persistent target, leash 0.125) + gripper, held for 4 env frames per decision | `paper/DISCRETE_ACTION_REPLAY_2026-09-06.md:132`; `full_env.py:803-830` |
| 16 | "...resulting in less than 0.1 radian difference between the real and simulated demonstrations" | imprecise | The tracking number of record is ‖e‖∞ median 0.004 rad / p95 0.030 rad, measured by replaying the real joint stream through the fixed controller on the 66-demo pick-scope set (39,332 frames), not on the re-executed tapes. "< 0.1 rad" is true but is the arm's tracking error, not demonstration fidelity (can outcomes differ, item 17) | `paper/real2sim_follower_lab_2026-08-23.md:161-170,197-214` |
| 17 | 74 trials recreated with attenuated success: 65/74 picked, 40/65 placed, 12/40 slid into contact | imprecise | These are the **state** set counts (picked 65, `placed_v2` 40, `home` 12; `slide_event` 25). The **pixel** set the reported learners train on reads picked 65, `placed_v2` 42, `slide_event` 26, `home` 13 (machine: 64 / 44 / 24 / 14). "Slid into contact" = `home`. Also: 10 tapes never pick (2 of them, uids 234/318, are lying-can placement artefacts, unwinnable by construction); 22 (state) / 26 (pixel) tapes end by the tip rule; re-execution is hardware-class sensitive after contact (1 human / 2 machine `home` tapes differ between the 64-core and 32-core rebuilds) | `DEMO_SETS` §1, §3; `CONFOUNDS` row 51 |

### A.4 §Machine Demonstrations + phase table (lines 75–90)

| # | Paper sentence | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 18 | Three machine sources: R2Dreamer (best RLfD), DP trained on human, motion planner | correct (detail) | R2 teacher = ONE r2dreamer seed (s15) trained on the human pixel set to 1M frames, MODE inference, **selected by the best rnd30 MODE `home` score** (n = 1 teacher); DP teacher = state-based lerobot DP `dp_phase/dHfull_pruned_DP_s0` trained on the **pruned** human set; planner = `experiments/full_state_planner_2026-09-12/` | `HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md:7`; `DEMO_SETS` §1; `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md` |
| 19 | All demonstration sets contain the same initial conditions | imprecise | DP = first attempt per human start (72 starts); planner and R2 = 72 attempts on the 67 distinct geometries of the DP tapes; across the four sets 56 positions agree within 5 mm; the human set has 74 tapes (one extra position + repeated attempts). Goal identical everywhere | `HANDOFF_PIXEL_4x4_2026-09-17.md` §1; `PLANNER_MATCHED_STARTS` |
| 20 | Recorded joint-control actions, numeric simulator states, 64×64 top and wrist RGB | correct | 17-d state (6 q, grip motor, grip effort, can xyz, can quat, goal xy); rig: top camera at (0.40, −0.08, 1.15) looking down, fov 68; wrist fov 80 at eef offset (0.10, 0, −0.03), pitched 30°; obs = (64,64,6) uint8 | `genesis_can_env.py:432-456,151`; `METHODS_draft:95`; `paper/PX_IMAGE_DEMOS_2026-09-12.md` |
| 21 | DP: "removing failed demonstrations and zero-actions" | imprecise | The pruned set = the 74 human tapes **minus the 10 no-pick tapes** (tapes that pick and then fail are kept) with idle runs collapsed **only before the grasp** (margin 38 decisions ≈ 5 s, `idle_eps 1e-3`); the last 50 actions of every tape are bit-identical to raw. The pixel DP in the 4×4 study trains on the **raw** human set (a pruned pixel set has not been built). Lane 03 owns the DP appendix | `DEMO_SETS` §1; `HANDOFF_PIXEL_4x4` §3 item 5, §5 |
| 22 | Phase table: Human 88.9 / 54.2 / 31.9 / 15.3 | **wrong / unsourced** | = 64/39/24/11 of 72. No set of record has these counts: state 65/40/25/12 of 74; pixel 65/42/26/13 of 74. Caption "each source contains 72 demonstrations" is wrong for the human set (74) | `DEMO_SETS` §3 |
| 23 | Phase table: DP 88.9 / 56.9 / 31.9 / 16.7 | correct for the state set | = 64/41/23/12 of 72 (state). Pixel set: 64/44/24/14 = 88.9 / 61.1 / 33.3 / 19.4 | `DEMO_SETS` §3 |
| 24 | Phase table: Motion planning 94.4 at every phase | wrong (see lane 01) | First-attempt collection: pick/place/slide credits 67/72, all 67 reach `home` → 93.1 %; lane 01 notes 68/72 matches a later training-corpus revision | `PLANNER_MATCHED_STARTS` ("Regenerated characterization"); `01_motion_planner_appendix.md` |
| 25 | Phase table: R2Dreamer 94.4 at every phase | correct | 68/72 pick, place, slide event, home; 4 tip endings | `HANDOFF_R2_TEACHER_DATASET:15-20` |
| 26 | Columns "Slide" and "Nested" | imprecise | Undefined. In the notes "Slide" = `slide_event` (released, tool on the far side, ≥ 1 cm goalward travel) and "Nested" = `home` (`slide_event ∧ nested_v2`), the paid event. Say so | `DEMO_SETS` §2 |

### A.5 §Results (line 93)

| # | Paper sentence | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 27 | 8 seeds of DP, RLPD, R2Dreamer, DV3 | correct (rule) | 8 per condition; world models = the 8 lowest seed numbers that reached 2M (result-blind) | `HANDOFF_PIXEL_4x4` §2 |
| 28 | RL policies train with sparse +10 for a successful slide | correct → add terminal | see item 5 | `full_env.py:162` |
| 29 | Episodes last at most 1200 simulation steps | imprecise | 1200 **env frames** = 300 decisions (`action_repeat` 4); one env frame = 3 physics steps of dt 0.01 s → 36 s of simulated time. Eval: `--max-steps 1200` for every learner; RLPD training `--train-max-steps 1200`. World-model training horizon **not verified locally** (the pixel config file is not in the local `~/workspace/r2dreamer` tree; the 09-13 audit states 1200 sim steps = 300 decisions). Human tapes run up to 601 decisions (2,404 frames), i.e. up to twice the learners' episode horizon | `PROVENANCE_nested_sparse10_pixel_2026-09-14.md:154-200`; `AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` §3; `genesis_can_env.py:62,263`; `DEMO_SETS` §1 |
| 30 | Evaluate on 30 new can positions chosen randomly from the demo support | imprecise | `eval_ics.json` `rnd`: 30 starts drawn once (seed 0, July, git `dd58c15`) uniformly over the axis-aligned bounding box (+1 cm) of the placements solved **at that time**: x 0.34–0.59, y −0.24–0.19; can upright, goal static; the same 30 starts for every learner, seed and checkpoint. The current 74 human starts span x 0.35–0.51, so 10/30 starts lie beyond the demonstrations' x-range and **5/30 (ICs 6, 8, 13, 19, 26; x 0.556–0.591) sit inside the shelf footprint (x ≥ 0.55)**, where `placed_v2` can fire at reset without a pick — `home` (the reported statistic) cannot, `placed` can. Notes name 4 such starts (see Discrepancies D5). Training-start chart = `hold` (15 demonstration starts, 14 of them in training; not held out). Actions sampled for every learner | `baselines/ic_sampling.py`; `baselines/eval_ics.json` (`rnd_note`, `support_box`); `CONFOUNDS` row 82; `paper/DP_PRUNED_GAP_2026-09-07.md`; `HANDOFF_PIXEL_4x4` §2 |

### A.6 Figure captions

| # | Caption | Verdict | Note | Source |
|---|---|---|---|---|
| 31 | `01-introduction.tex:30` Kinova Gen3 Lite, game controller, evaluated in simulation | correct / controller unsupported | as items 8–9 | — |
| 32 | `01-introduction.tex:38` "collected in-the-wild from forty participants" | **wrong / unsupported** | item 6 | `EARLY_TRIALS_ROTATION` §1 |
| 33 | `03-method.tex:58,64,67` real-world set-up / Digital Twin / slide one can into contact | correct | "in the wild": one afternoon in one public space is all the record supports | — |

### A.7 `appendix/a-research-methods.tex` §Bringing real demonstrations into the Genesis simulator

| # | Claim | Verdict | What the notes / code say | Source |
|---|---|---|---|---|
| 34 | Genesis 0.2.1 | correct | pinned; the v0.2.1 checkout carries a local headless-render patch and is +269 upstream commits (not pip-reproducible) | `CLAUDE.md` (2026-07-20); `AUDIT_R2D` §3 |
| 35 | Recordings contain joint readings and camera video but no object poses | correct (add tool pose, gripper, commanded twist) | item 11 | `trial_reader.py` |
| 36 | FK of the recorded grasp reproduces the recorded tool pose to within 1 cm | correct | FK fingertip midpoint vs bag `tool_pose` at gripper close: ~1 cm xy, ~1 mm z | `CAN_STARTING_POSITION.md:185-186,258-259` |
| 37 | Searched a small region around that point for the placement from which replay reproduces the grasp; validated by repeated replay | correct (detail) | CPU-parallel search (`cpu_research.py`), winners ×3-replayed on an idle box; 2 placements are artefacts (234/318 lying cans) | `CAN_STARTING_POSITION.md:192-203,223`; `CONFOUNDS` row 51 |
| 38 | Camera audit confirmed the validated placements to within 1.5–3.5 cm | imprecise | 1.5–3.5 cm is the **instrument precision** on the can-top plane in the central workspace (two spot checks, 242 ≈ 1.5 cm, 286 ≈ 3.5 cm). The comparison actually run: \|close-time can − placement\| median 2.9 cm, 14/15 ≤ 5 cm over the **15 videoed ok-class trials**; the full click pass over all trials was planned, not executed | `CAMERA_GATE` §2 (lines 52-81) |
| 39 | Tracking error 0.004 rad median / 0.030 p95 over 66 demos and 39,332 frames | correct (qualify) | ‖e‖∞ over joints per frame; measured **offline by replaying the real joint stream** through the controller of variant `gc_kp4_riser3` on the 66-demo pick-scope set; not the re-executed full-task tapes | `real2sim_follower_lab:161-170,206` |
| 40 | after adding gravity compensation and correcting the mounting height | imprecise | **Three** changes, all needed (they interact): gravity compensation 1.0, arm PD gains ×4 (kv ×2), base riser +3 cm. (The shelf was also raised 6 cm, world `..._shelf6`, on the camera/tool-pose adjudication that the real plate is 11–13 cm above the table) | `sim_variants.py:51,67`; `real2sim_follower_lab` §4.1–4.2; `CLAUDE.md` SHELF adjudication |
| 41 | At the set-down that begins the slide the simulated tool is within 0.2 cm (median) of the real tool | correct | 45 tapes; p75 3.4 cm; p90 23.8 cm (two late-regrasp tapes) | `SLIDE_ANATOMY_2026-09-07.md:22-23` |
| 42 | We do not train on the raw recordings; closed-loop waypoint follower; same recorder, clock, action contract, file format | correct | `HumanFollower`: `a_arm = clip((cmd_j − target)/(4·0.025), −1, 1)`, advance on arrival ‖q − ref‖∞ < 0.025 rad, ≤ 4 waypoints per decision, dwell cap 8, dilation cap 3×, 25-decision settle; no release filter in the sets of record | `baselines/record_demos.py:40-49,398-470`; `PHASE_RESULTS_2026-09-05.md:247` |
| 43 | 74 of 75 success-labelled trials; 16 operator-labelled failures excluded by label | correct | add: the 10 never-pick tapes are kept; the failures' recordings exist and were used only in the pick-phase all-data arm | `DEMO_SETS` §1 |
| 44 | 65 of 74 pick and 12 reach the goal | correct for the state set | pixel set: 65 pick, 13 `home` (machine 64 / 14) | `DEMO_SETS` §3 |
| 45 | "although every one did in reality" | **unsupported** | "Success" is the operator's label; the real endpoint can include human handling (uid 254 in this set: goal moves and a hand handles the can at 47–48 s) | `CONFOUNDS` rows 89, 92 |
| 46 | Shortfall concentrated in the slide; can travels 2–4 cm less; a tipped can ends the episode | correct | slide is control-limited at the stroke level (closed-loop micro-nudges vs open-loop replay), half the loss lateral; 22/26 tapes tip; grasp creep (row 49) and the 1.7 s real release (row 50) are the tip mechanisms | `SLIDE_ANATOMY` §3; `CONFOUNDS` rows 49, 50 |
| 47 | Four recorded frames per 0.12 s decision assuming 33.3 fps; actual 29.6–39.2 fps | correct | `REPEAT = 4`; `trial_reader.py` emits a frame per 1/60 s window only when `joint_states` and `base_feedback` both land in it (beat artefact, drops 2–26 % of samples); tapes carry no timestamps; measured 29.63–39.13 fps, median 32.79 | `record_demos.py:452`; `trial_reader.py:31-55`; `CONFOUNDS` row 46; `EEF_ACTION_DIST:21,188` |
| 48 | 2–7 % faster or slower | imprecise | p10/p50/p90 duration ratio 0.94/0.98/1.06; the extremes are ≈ +13 % (29.6 fps tape) and −15 % (39.2 fps); 25/74 tapes end > 2 s off, uids 286/287 +16 s | `CONFOUNDS` row 46 |
| 49 | median timing lag 0.25 s once the clock term is removed | correct | 0.72 s → 0.25 s (pick scope 0.36 → 0.15); residual = follower/PD lag + 4-frame grip quantisation | `CONFOUNDS` row 46 |
| 50 | Machine demonstrations, on the environment's own clock, are unaffected | correct | — | `CONFOUNDS` row 46 |

---

## B. Paste-ready LaTeX

Only what is wrong or missing is changed; the user's sentences are kept where they stand. Bracketed
`\todo{}` items are decisions the authors must make (participant count, IRB, controller).

### B.1 `sections/04-evaluation.tex` — §Real-World Experiment: Restocking (replaces line 63)

```latex
The restocking task has three parts (Fig.~\ref{fig:envs}). \textit{Pick} the can of soup off the table, \textit{Place} the can on the shelf, and \textit{Slide} the can into contact with a goal can already on the shelf. The gripper fingers are wide, making it difficult to place the two cans next to each other and necessitating a slide or careful placement. The can started in one of three marked spots, and the goal can was static across all trials. Reinforcement learning algorithms received a single terminal reward of $+10$ when the released can, pushed from its far side, came to rest upright within 1.5\,cm of touching the goal can (the \emph{home} event, Appendix~\ref{appx:real2sim}), and 0 at all other steps; an episode also ends, with no penalty, when a free can tips past $60^\circ$.
```

### B.2 §Data Collection (replaces lines 67–68)

```latex
We collected demonstrations in a public-space study\todo{IRB and participant count: the recordings carry no participant identifier, and the 74 trials used here were all recorded in one session on a single afternoon (2024-12-18); state the number of participants only from the study record, or write ``an unknown number of non-expert operators'' -- see 06\_sim\_real2sim\_audit.md D1}. Participants teleoperated a six-joint Kinova Gen3 Lite arm with a two-finger gripper using a game controller\todo{confirm Xbox} that commanded the gripper and the end-effector velocity in $x$, $y$, $z$ and one rotational rate about the tool axis; the remaining rotational rates were held at zero by the interface, which reduced the risk to the arm and scene for inexperienced users (the recorded twist has exactly one non-zero angular channel in every trial).
We recorded the joint states, the tool pose, the gripper position and the commanded velocity from ROS, together with a top-down video of the start area and a second camera looking up through the translucent shelf, to enable audit and recovery. The session produced 104 recordings; the operator labelled 75 as successful and 16 as failures. We use the 74 successful trials that could be reconstructed in simulation (Sec.~\ref{sec:dig_tw}); the failure-labelled trials were excluded by label. ``Successful'' is the operator's label: ten of the 74 trials never lift the can in simulation, and the real recordings occasionally end with the operator's hand steadying the can.
```

### B.3 §Digital Twin (replaces line 71)

```latex
In order to examine the effect of demonstration source we trained learners that require online sampling for a statistically meaningful number of seeds. Reinforcement learning on the real robot needs dense reward or human help to be tractable~\cite{wu2023daydreamer, luo2024precise}, but either would give the RL learners extra information that the imitation learners do not get. Therefore we built a digital twin of the task in the Genesis simulator~\cite{Genesis} (version 0.2.1), converted the demonstrations to run inside it, and all training and evaluation happen in the twin. The operators commanded end-effector velocities, but replaying those commands open loop reproduces only 17 of the 74 grasps: the real arm's Cartesian controller realised them with a median 4\,cm of drift by grasp time. Replaying the recorded \emph{joint} stream reproduces 69 of 74 grasps, so our learners act in joint space (six delta joint targets, capped at $0.025$\,rad per simulation frame, plus a gripper command, each decision held for four frames), and the simulated arm tracks the real joint trajectories to a median of $0.004$\,rad (95th percentile $0.030$\,rad). More details of the real2sim process are in Appendix~\ref{appx:real2sim}. Re-execution attenuates success: of the 74 trials, 65 pick the can, 42 place it on the shelf, 26 slide it, and 13 complete the task (\emph{home}) in the pixel-observation sets used here (65/40/25/12 in the state-observation rebuild); 26 end early because the can tips.
```

### B.4 §Machine Demonstrations — phase table (replaces lines 80–90; the DP paragraph is lane 03's)

Numbers are the pixel sets the reported learners train on (`DEMO_SETS` §3; planner and R2 from their handoffs).
Human n = 74, the others n = 72. Planner row uses the first-attempt collection (67/72); lane 01 explains the 68/72
alternative.

```latex
\begin{table}[htbp]\centering\small
\begin{tabular}{lrrrrr}\toprule
Demonstration source & $n$ & Picked & Placed & Slide & Home \\\midrule
Human            & 74 & 87.8 & 56.8 & 35.1 & 17.6 \\
DP               & 72 & 88.9 & 61.1 & 33.3 & 19.4 \\
Motion planning  & 72 & 93.1 & 93.1 & 93.1 & 93.1 \\
R2Dreamer        & 72 & 94.4 & 94.4 & 94.4 & 94.4 \\
\bottomrule\end{tabular}
\caption{Demonstration success by task phase (\% of tapes reaching each stage when the recorded actions are re-executed in the twin). \emph{Placed}: released upright inside the shelf footprint; \emph{Slide}: after a set-down the tool reaches the far side of the can and the can travels $\geq 1$\,cm toward the goal; \emph{Home}: the slide ends with the can at rest within 1.5\,cm of touching the goal can -- the only rewarded event. The human set keeps every attempt (74 tapes, ten of which never pick); the three machine sets hold one tape per start on the same 72 starts. The state-observation rebuilds of the human and DP sets differ by one to two tapes per phase.}
\label{tab:phases}
\end{table}
```

### B.5 §Results (replaces line 93)

```latex
We train 8 seeds \todo{replace with final number} of DP, RLPD, R2Dreamer, and DfD to investigate the effect of demonstration source on these algorithms. The hyperparameters and simulated configuration for training can be found in \ref{appx:training}. RL policies (RLPD, DfD, R2) train with a sparse terminal reward of $+10$ for reaching \emph{home}; nothing else pays. Episodes last at most 1200 simulation frames (300 decisions of four frames each; a frame is three 10\,ms physics steps, so 36\,s of simulated time), or end at \emph{home} or when the can tips. We evaluate every learner, seed and checkpoint on the same 30 fixed can positions, drawn once uniformly over the bounding box of the demonstration start positions with the can upright and the goal in place, and break the results out by task phase. The box is wider than the demonstrations' footprint: ten of the 30 starts lie beyond the demonstrations' range toward the shelf, and five of them are inside the shelf footprint, where the \emph{placed} predicate can be satisfied without a pick (\emph{home} cannot). We also report the 15 demonstration starts on which the training curves are read; 14 of these are training starts, so that set is in-distribution, not held out. Actions are sampled from the policy for every learner.
```

### B.6 `appendix/a-research-methods.tex` §Bringing real demonstrations into the Genesis simulator (replaces the whole section)

```latex
\section{Bringing real demonstrations into the Genesis simulator}\label{appx:real2sim}

We rebuilt the study scene in the Genesis simulator (version 0.2.1). The recordings contain the
robot's joint readings, its reported tool pose, the gripper position, the commanded velocity and
camera video, but no object poses. We recovered each can's starting position from the forward
kinematics of the recorded grasp, which reproduces the recorded tool pose to within 1\,cm, then
searched around that point for the placement from which replaying the recorded joint trajectory
reproduces the grasp; placements were validated by three repeated replays. A camera audit,
scoring the can at the closure that initiates the lift, agrees with the validated placements to a
median of 2.9\,cm on the 15 trials it covered (14 of 15 within 5\,cm), with an instrument precision
of about 1.5--3.5\,cm; two of the 74 placements are known artefacts (a can recorded lying on its
side that stands upright in the video) and those two trials cannot be completed in simulation.

The simulated arm follows the real joint stream with a median tracking error of 0.004\,rad
(95th percentile 0.030\,rad, worst joint per frame) over 66 demonstrations and 39{,}332 frames,
after three changes to the stock arm: gravity compensation, four times the default joint gains,
and raising the robot base by 3\,cm to match the real mounting height. The shelf was raised 6\,cm
to the height the video and tool poses agree on. The tracking figure was measured offline by
replaying the real joint stream through the corrected controller; it describes the arm, not the
can. At the set-down that begins the slide, the simulated tool is within 0.2\,cm (median) of the
real tool.

We do not train on the raw recordings. We re-execute each demonstration inside the learners' own
environment with a closed-loop waypoint follower, so every source shares the same recorder,
clock, action contract and file format: six delta joint targets capped at 0.025\,rad per
simulation frame plus a gripper command, one decision per four frames. The full-task human set
keeps every success-labelled trial of the session that could be recovered (74 of 75; the
session's 16 operator-labelled failures were excluded by label), including the ten trials in
which the can is never lifted in simulation. Re-execution is not lossless: 65 of the 74 tapes pick
in simulation, 42 place, 26 slide and 13 reach the goal, although the operator labelled every
one a success. The shortfall is concentrated in the slide: with the real tool tracked to
0.2\,cm, the can travels 2--4\,cm less in simulation than in reality, because the operator's
closed-loop nudges are replayed open loop; and a can that tips in simulation ends the episode
(26 of 74 tapes). The machine sets are recorded in the same environment and file format, one
tape per start on the same 72 start positions.

The recorder consumes four recorded frames per 0.12\,s decision, assuming 33.3\,fps; the recorder
of the original study dropped samples irregularly, so the actual frame rate varies from 29.6 to
39.2\,fps between recordings (median 32.8) and each human demonstration replays typically
2--7\,\% (at the extremes about 15\,\%) faster or slower than the human moved (median timing lag
0.25\,s once this clock term is removed). Human tapes run up to 601 decisions, twice the
learners' 300-decision episode horizon. Machine demonstrations, generated on the environment's
own clock, are unaffected.

The learners' observation is either the 17-dimensional simulator state (six joint positions,
gripper motor, grip effort, can position and quaternion, goal $xy$) or, in the pixel study, two
$64\times64$ RGB images (an overhead camera and a camera on the wrist) plus the eight
proprioceptive numbers. The task outcome \emph{home} requires that the can was released on the
shelf, that the tool then reached the far side of the can and pushed it at least 1\,cm toward the
goal, and that it came to rest upright with its centre within 8.1\,cm (one can diameter plus
1.5\,cm) of the goal can. It fires on 13 of 13 human and 14 of 14 machine simulated slides with no
false positive on the demonstration set.
```

---

## C. Discrepancies (paper vs notes/code)

| ID | Where | Paper | Notes / code | Evidence |
|---|---|---|---|---|
| D1 | 04:67, 01:38 | forty non-expert users / forty participants | No participant identifier exists; the 74 tapes are one session (2024-12-18); one vs many people undecidable. If the authors have a study record, cite it; otherwise the paper must not state a count | `paper/EARLY_TRIALS_ROTATION_2026-09-08.md` §1; `E2E_AUDIT_BRIEF` §11 |
| D2 | 04:67 | IRB approved; Xbox controller | Not in any note or code comment (only "joystick") | `METHODS_draft:28` |
| D3 | 04:67 | "pitch; roll and yaw were fixed" | Commanded: one angular channel (`wy`); realised: it drives world-z **yaw**, and roll/yaw were **free** under the Jacobian controller (drift 1.03/1.34 rad) | `EEF_ACTION_DIST_2026-09-07.md:109-133`; `METHODS_draft:29` |
| D4 | 04:84-89 | Human phase row 88.9/54.2/33.3/15.3; "each source contains 72" | Human = 74 tapes; no set has 64/39/24/11. State 65/40/25/12; pixel 65/42/26/13 | `DEMO_SETS` §3 |
| D5 | 04:93; `CONFOUNDS` row 82; `E2E_AUDIT_BRIEF` §3 | (paper silent) | The notes say **4** rnd30 starts lie in the shelf footprint (ICs 6/13/19/26; "every other IC x < 0.55"). `eval_ics.json` has **5**: IC 8 is at x = 0.5575, y = −0.006, inside x ∈ [0.55, 0.95], y ∈ [−0.5625, 0.1875]. Row 82's empirical check was on ic13 only. Someone should re-count | `baselines/eval_ics.json` `rnd[8]`; `can_pos_recovery/replay_harness.py:38-40,211-213` |
| D6 | 04:93 | "30 new can positions chosen randomly from the demo support" | Box = bounding box of the placements solved in July (x to 0.592); current human starts reach x 0.513 → 10/30 starts beyond the demonstrations' range (`DP_PRUNED_GAP`: 0/280 DP picks at x ≥ 0.52) | `eval_ics.json` `support_box`; `trial_placements.json` (this audit); `DP_PRUNED_GAP_2026-09-07.md` |
| D7 | appx:20-21 | "although every one did in reality" | Operator label only; uid 254 shows a hand handling the can at the end | `CONFOUNDS` rows 89, 92 |
| D8 | appx:8-9 | camera audit "confirmed the validated placements to within 1.5–3.5 cm" | 1.5–3.5 cm = instrument precision; the comparison covered 15 trials (median 2.9 cm, 14/15 ≤ 5 cm); the full click pass was never run | `CAMERA_GATE` §2 |
| D9 | appx:11-13 | tracking error "after adding gravity compensation and correcting the mounting height" | three changes incl. PD gains ×4; measured on the 66-demo pick-scope set by offline joint replay, not on the 74 re-executed tapes | `real2sim_follower_lab` §4.2; `sim_variants.py:51,67` |
| D10 | 04:63, 04:93 | "+10 when they slid the can into contact" | `home` = slide_event ∧ nested_v2; nested_v2 is centre distance ≤ 0.081 m (1.5 cm short of contact), upright, at rest, not in hand; terminal; tip also terminal | `full_env.py:149-176`; `DEMO_SETS` §2 |
| D11 | 04:78 | DP data "removing failed demonstrations and zero-actions" | pruned = minus the 10 no-pick tapes + pre-grasp idle collapse only; the pixel DP of the 4×4 trains on the RAW human set | `DEMO_SETS` §1; `HANDOFF_PIXEL_4x4` §3.5 |
| D12 | 04:93 | 1200 simulation steps | 1200 env frames = 300 decisions = 3600 physics steps; WM **training** horizon not verifiable from the local tree (`genesis_full_pixel.yaml` absent locally; cluster tree `$W/r2dreamer_px` not checked) | `genesis_can_env.py:62,263`; `PROVENANCE…:154-200` |
| D13 | 04:71, appx:20 | 65/40/12 | state-set numbers; the primary (pixel) sets read 65/42/26/13 human, 64/44/24/14 machine; class-sensitive re-execution differs on 1 + 2 `home` tapes | `DEMO_SETS` §3 |
| D14 | appx:27 | "2–7 % faster or slower" | p10–p90 band; extremes ≈ +13 % / −15 % | `CONFOUNDS` row 46 |
| D15 | 04:86 | Motion planning 94.4 | 67/72 = 93.1 (first-attempt collection); see lane 01 for the 68/72 corpus | `PLANNER_MATCHED_STARTS` |
| D16 | appx (missing) | — | Human tapes up to 601 decisions (2,404 frames) vs the 300-decision learner horizon; the notes flagged that a share of full-scope reward grants lies beyond the training horizon | `DEMO_SETS` §1; `ADVERSARIAL_REVIEW_data_2026-09-07.md:164` |
| D17 | 04:75 | "same initial conditions" | 72 starts shared (56 within 5 mm across the four sets); human has 74 tapes; planner/R2 are 72 attempts over 67 geometries | `HANDOFF_PIXEL_4x4` §1 |

---

## \llm notes inserted

All one-liners, Edit tool, unique anchors; full text lives in this file.

- `sections/01-introduction.tex:38` — after "forty participants (Figure~\ref{fig:restock_full})": participant count unsupported; one session; see 06 D1.
- `sections/04-evaluation.tex:63` (Restocking paragraph end) — reward/terminal/`home` definition; B.1.
- `sections/04-evaluation.tex:68` (Data Collection end) — participants, controller, pitch/yaw, 74-of-104 accounting; B.2, D1–D3.
- `sections/04-evaluation.tex:71` (Digital Twin end) — joint-control rationale (17/74 vs 69/74), tracking numbers, pixel-set phase counts; B.3.
- `sections/04-evaluation.tex:89` (phase-table caption) — human row unsourced, n = 74, column definitions, planner 93.1; B.4, D4/D15.
- `sections/04-evaluation.tex:93` (Results end) — horizon definition, start set provenance, 5/30 in-footprint, hold15 in-distribution; B.5, D5/D6/D12.
- `appendix/a-research-methods.tex` §real2sim, paragraphs 1–4 — one note each: camera-audit scope (D8), three controller changes + offline measurement (D9), "every one did in reality" (D7) + pixel counts (D13), timing extremes + horizon (D14/D16); B.6.

## Sources

Paper: `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sections/{01-introduction,03-method,04-evaluation}.tex`,
`appendix/a-research-methods.tex`, `preamble.tex` (`\llm` macro).

Notes: `HRI_results/late_night_9_18/00_BRIEF.md`; `HRI_results/DEMO_SETS_2026-09-11.md`; `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`;
`HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`; `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`;
`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`; `HRI_results/late_night_9_18/01_motion_planner_appendix.md` (planner count);
`paper/CONFOUNDS.md` rows 46, 47, 49, 50, 51, 82, 89, 92; `paper/E2E_AUDIT_BRIEF_2026-09-10.md` §3, §11;
`paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` §3, §5; `paper/SLIDE_ANATOMY_2026-09-07.md`; `paper/EARLY_TRIALS_ROTATION_2026-09-08.md` §1;
`paper/METHODS_draft_2026-08-28.md` rows 28–29, 38, 41, 52, 92, 95, 421; `paper/EEF_ACTION_DIST_2026-09-07.md`; `paper/CAMERA_GATE_2026-08-31.md`;
`CAN_STARTING_POSITION.md`; `paper/real2sim_follower_lab_2026-08-23.md` §4; `paper/DISCRETE_ACTION_REPLAY_2026-09-06.md`;
`paper/DP_PRUNED_GAP_2026-09-07.md`; `paper/PX_IMAGE_DEMOS_2026-09-12.md`; `paper/PHASE_RESULTS_2026-09-05.md:247`;
`paper/ADVERSARIAL_REVIEW_data_2026-09-07.md:117,164`; `analysis/dataset_dynamics_four_sources_2026-09-15/REPORT.md:5`.

Code: `baselines/rl/full_env.py` (ladders 74–176, tip guard 181, action_repeat/delta cap 803–830, max_steps 884–886);
`baselines/genesis_can_env.py` (8–20, 55–70, 140–152, 225–265, 432–456); `baselines/sim_variants.py` (33–116, 185–190);
`baselines/record_demos.py` (30–75, 398–470); `trial_reader.py` (23–55); `baselines/ic_sampling.py`; `baselines/eval_e2e.py` (10–20, 46–59);
`baselines/make_eval_ics.py` (115–133); `can_pos_recovery/replay_harness.py` (37–51, 94, 211–213); `cluster/wmfix_full.sbatch:241-247`.

Data (read-only): `baselines/eval_ics.json`, `baselines/eval_ics_v2.json` (rnd start coordinates, support box, hold/sel uids);
`can_pos_recovery/trial_placements.json` (74 success-labelled starts, k-means k=3 by this audit).

Not found locally: `~/workspace/r2dreamer/configs/genesis_full_pixel.yaml` (world-model training horizon unverified, D12).
