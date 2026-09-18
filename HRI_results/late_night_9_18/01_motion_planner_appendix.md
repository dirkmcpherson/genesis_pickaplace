# Item 01 — Appendix entry describing the motion-planning algorithm precisely

**Status: done.** The planner is a hand-engineered, privileged-state phase controller (`experiments/full_state_planner_2026-09-12/run.py`, class `Controller`, sha256 `dcc65672…` = the hash pinned in every collection plan) with one joint-space RRT-Connect query (`path_planning.py`, sha `5bc96e5d…`) for the pre-grasp transit and Cartesian IK servoing for everything else; collision queries run in a separate kinematic "shadow" Genesis scene. The dataset the paper trains on is **revision 3** (`matched72_completed_2026-09-15/training_v3_reference_terminals`, cluster `planner72_px_2026-09-15`): 72 non-empty tapes, 17,872 decisions, **68 reach `home`** (not 67 — see Discrepancies), 2 tip terminals, 67 distinct start geometries. Everything below is read from the code and the collection manifests, not from prose summaries; every number carries its source. The `\llm{}` note is inserted in `sections/04-evaluation.tex`.

---

## 1. Paste-ready main-text sentence (§04 "Machine Demonstrations", replaces `\todo{reference appendix}`)

```latex
The planner is a hand-engineered controller with privileged access to the simulator state (arm joints, can and goal poses, contacts): it reaches a pre-grasp pose by joint-space RRT-Connect, then servos the tool through grasp, lift, transport, set-down, release and a fingertip push with damped-least-squares IK, emitting the same capped delta-joint and gripper actions the learners use; the algorithm, its parameters and its failure modes are given in Appendix~\ref{appx:planner}.
```

## 2. Paste-ready appendix subsection

Suggested location: `appendix/a-research-methods.tex`, after `\section{Bringing real demonstrations into the Genesis simulator}\label{appx:real2sim}` (as a new `\section` if the appendix keeps top-level sections; the label is `appx:planner` either way).

```latex
\subsection{Motion-planning demonstrations}\label{appx:planner}

\paragraph{Overview.}
The ``Planner'' demonstration set was produced by a hand-engineered controller
with privileged access to the simulator, not by a learned policy. It has no
trained parameters and no learning seed; the only randomness is a per-attempt
integer seed that drives the sampling-based planner and the NumPy/Torch RNGs.
It runs inside the same environment class, world, action contract and horizon
budget as the learners, so its tapes are recorded by the same recorder as the
human and DP sets.

\paragraph{Privileged state.}
At every decision the controller reads, directly from the physics engine: the six
arm joint angles and the full robot configuration; the world-frame positions and
orientations of the manipulated can and the goal can; the tool pose from a
URDF forward-kinematics chain evaluated at the measured joints; the number of
solver contacts between can and robot, can and shelf, and can and goal; and the
environment's own stage flags (\texttt{picked}, \texttt{placed\_v2},
\texttt{settled\_after\_release}, \texttt{slide\_event}, \texttt{nested\_v2},
\texttt{home}). The learners trained on this data never see any of this: the
pixel learners receive two $64\times64$ RGB views and an 8-d proprioceptive
vector (six joints, gripper motor, grip effort).

\paragraph{Action interface.}
Every command passes through \texttt{FullTaskEnv.step()} in
\texttt{delta\_joint} mode: a 7-d action in $[-1,1]$ whose first six entries are
scaled by the cap $0.025$\,rad and added to a persistent joint target (clipped to
a leash of $5\times0.025=0.125$\,rad around the measured joints), held for
\texttt{action\_repeat}$=4$ physics frames of $0.03$\,s (one decision $=0.12$\,s;
a decision can therefore move each joint by at most $0.1$\,rad), and whose
seventh entry is an absolute gripper command ($-1$ open, $+1$ closed, mapped to
motor fraction $(a_7+1)/2$). The controller converts a desired joint vector
$q^\star$ into the action
$a_{1:6}=\mathrm{clip}\big((q^\star-q_{\text{target}})/(4\cdot0.025),-1,1\big)$,
so joint-space waypoints more than $0.1$\,rad away are approached over several
decisions. The planner never teleports the robot, attaches the can, or bypasses
the PD controllers.

\paragraph{Inverse kinematics.}
Cartesian targets are converted to joint targets by the simulator's
damped-least-squares IK (Genesis 0.2.1
\texttt{RigidEntity.inverse\_kinematics}: $\Delta q=J^{\!\top}(JJ^{\!\top}+\lambda^2 I)^{-1}e$
with $\lambda=0.01$, per-iteration step clamped to $0.5$\,rad, joint limits
respected, warm-started from the previous solution, at most $4$ random
restarts $\times\,80$ iterations). Transit and servo targets use tolerances of
$1$\,mm / $0.01$\,rad; the millimetre-scale push uses $10\,\mu$m / $0.001$\,rad
to avoid a solver dead-band. Each solution is re-checked by an independent
NumPy URDF forward-kinematics chain and rejected if the tool is more than
$4$\,mm or $0.04$ (Frobenius norm of the rotation difference) from the request.
The tool orientation is fixed to a side grasp with the approach axis along
world $+x$ (towards the shelf), optionally yawed about $z$.

\paragraph{Collision model.}
Every candidate joint edge is validated in a second, purely kinematic Genesis
scene (``shadow'') built from the same morphs, synchronised to the execution
scene's measured configuration and object poses before each query (the
synchronisation asserts the two wrists agree to $0.1$\,mm). Validity means no
solver-detected collision pair except (i) the fixed base-mount overlap, (ii)
finger--can contact while a grasp or push is intended, (iii) can--shelf contact
while lowering a can that is already within $2$\,mm of its resting height. When
the can is held, the shadow places it by the last measured tool-to-can
transform and additionally rejects configurations that would tilt it by more
than $15^\circ$. Edges are checked at a discrete joint resolution of
$0.03$\,rad (planner) or $0.025$\,rad (servo), so clearance is sampled, not
proven, and a slipping can is not guaranteed to follow the checked path.

\paragraph{Planner.}
Only the first motion, from the reset pose to the pre-grasp pose, is planned:
bidirectional RRT-Connect in the 6-d joint space (bounds = URDF limits clipped
to $\pm\pi$), extension step $0.25$\,rad ($\ell_\infty$ metric), goal bias
$0.1$, at most $500$ iterations, seeded by the attempt seed; a direct valid edge
is returned without search and no path shortcutting is applied. The path is
interpolated at $0.05$\,rad and each waypoint is tracked for up to $12$
decisions until every joint is within $0.015$\,rad (otherwise the attempt
aborts with ``tracking timeout''). Every later motion is a receding
Cartesian servo: at each decision the tool advances at most $s$ metres toward
the target (and at most $0.06$\,rad in rotation), the step is solved by IK,
edge-checked in the shadow, and executed; the servo converges at $3$\,mm /
$0.035$\,rad and aborts after $110$ decisions.

\paragraph{Phase structure and targets} (can centre $c$, goal centre $g$,
shelf top $z_s$, can height $h=0.101$\,m, all offsets in metres, world frame).
\emph{Pick:} (1) \texttt{pregrasp} -- RRT-Connect transit to $c+(-0.075,0,0)$
with the gripper open; (2) \texttt{approach} -- servo to $c$ at $s=0.005$ with
finger--can contact allowed; (3) \texttt{close} -- the gripper command ramps
linearly from $-1$ to $+0.56$ over 9 decisions and is held 4 more (motor
fraction $0.78$, a deliberate partial close); (4) \texttt{lift} -- servo
$+0.16$ in $z$ at $s=0.008$, hold 4, then verify \texttt{picked} and a live
robot--can contact, else abort. \emph{Place:} the set-down target is
$p=g+(0,+0.12,0)$ with $p_z=z_s+h/2$, reserving a $12$\,cm push corridor on the
$+y$ side of the goal; (5) \texttt{transport} -- servo the \emph{can} (not the
tool) to $p+(0,0,0.04)$ at $0.010$ per decision, re-estimating the tool-to-can
transform every step and applying a $\le0.06$\,rad tilt correction, converging
at $6$\,mm and tilt $<8^\circ$; (6) \texttt{lower} -- the same servo to $p$ at
$0.005$ per decision until the can touches the shelf within $12$\,mm and
$<15^\circ$; (7) \texttt{release} -- gripper command $-1$ for 2 decisions (a
step, not a ramp); (8) \texttt{withdraw} -- servo $-0.06$ in $x$ at $0.007$,
hold 4, then verify \texttt{settled\_after\_release}, else abort.
\emph{Slide:} (9) \texttt{reposition} -- with the gripper fully closed ($+1$,
commanded for 6 decisions while raised) the tool is moved to
$z=c_z+0.14$, then above and down onto the pre-push pose
$c+R_z(\psi)(-0.015,0.065,0)$, where $\psi$ yaws the closed distal finger's side
face (mesh normal $14.1^\circ$ off the tool $-y$ axis) toward the goal;
(10) \texttt{push} -- up to 100 decisions of $2.5$\,mm steps toward the contact
point $c+R_z(\psi)(-0.015,0.0443,-0.008)+0.003\,\hat d$ ($\hat d$ the unit
can-to-goal direction, recomputed every decision), stopping when the planar
can--goal distance falls below $0.075$\,m and aborting if the can tilts more
than $25^\circ$; (11) \texttt{final\_withdraw} -- servo $0.08$ back along
$-\hat d$; (12) \texttt{settle} -- hold 15 decisions.

\paragraph{Termination.}
The episode ends on the environment's own terminals: the sticky \texttt{home}
event of the \texttt{nested\_sparse} ladder (set-down, tool on the far side,
$\ge1$\,cm credited goalward slide, then an instantaneous \texttt{nested\_v2}
arrival: both cans upright ($<20^\circ$), planar distance $\le0.081$\,m, can in
the shelf band, tool outside the in-hand radius, 12-frame rest window) or the
tip rule (can tilt $>60^\circ$ while not in hand for 4 frames). In every
completed attempt \texttt{home} fired during \texttt{final\_withdraw}, before
the settle block. Collection used a budget of $2{,}400$ physics frames
($600$ decisions, $72$\,s), matching the DP collection budget; the controller's
development default was $1{,}200$ frames. A controller abort (planning, IK,
tracking or verification failure) ends the tape without a terminal flag. After
a completed episode a 100-step physics hold verifies that both cans remain
settled on the shelf (``retention''); it earns no credit and is not recorded in
the training tapes.

\paragraph{Reward export.}
The planner ran under the $+1$ \texttt{nested\_sparse} ladder; tapes are exported
with the reward multiplied by 10 to match the learners' \texttt{nested\_sparse10}
ladder (no re-scoring). Following the convention of the reference human and DP
sets, the rewarded \texttt{home} rows are exported as \emph{non}-terminal
(discount~1, \texttt{is\_terminal}=0; 68 rows) so the RL learners bootstrap
through them, while the two tip terminals and every recording boundary
(\texttt{is\_last}) are retained.

\paragraph{Starts, seeds and outcomes.}
One attempt was scheduled for each of the 72 DP tapes, resetting to the same
recovered trial placement (67 distinct can geometries, including the two
lying-can starts, uids 234 and 318) and auditing the initial 17-d state against
the DP tape's first row ($<10^{-5}$ geometry, $<10^{-4}$ proprioception). Attempt
$i$ used seed $40000+i$ ($40000$--$40071$); the simulator itself is initialised
with seed~0. Of the 72 first attempts, 67 reached \texttt{home} (and passed
retention) and 5 failed before pick-up: two lying-can starts tipped at the
first decision, one pre-grasp joint-tracking timeout (136 decisions), and two
pre-action aborts with zero recorded actions (an initial hand--can overlap that
made the start configuration invalid, and an IK failure of $8.3$\,mm at the
pre-grasp pose). At the user's request the two empty attempts were re-run once
with a bounded recovery (a checked upward escape from the initial overlap, and
pre-declared alternative pre-grasp offsets of $+1$\,cm, $+2$\,cm in $x$ or
$+1$\,cm $x$/$+3$\,cm $z$), same seeds and starts, no further retries: one
completed \texttt{home} (305 decisions) and one recorded a 17-decision escape
followed by a planning abort. The training corpus is therefore a curated
72-non-empty-tape revision (70 first attempts $+$ 2 recovery attempts) with
68 completed episodes, 17,872 decisions in total, median 255 decisions per
tape, and no post-pick failures; it is not an unbiased first-attempt success
estimate. Mean wall-clock per attempt was $110$\,s on one CPU thread,
including world construction.

\paragraph{What was recorded.}
At every decision boundary: the 17-d simulator state (6 joints, gripper motor,
grip effort, can position and quaternion, goal $xy$), full \texttt{qpos}/\texttt{qvel},
the normalised 7-d action, the environment's stage flags, reward, terminal and
truncation flags, the physics frame index, the URDF and metric tool poses, and
the three contact counts. The $64\times64$ top and wrist RGB frames used for
training were \emph{not} taken from the collector process: the presence of the
second (planning) scene perturbed its renderer, so every sealed action tape was
replayed in a fresh single-scene environment, checked at every boundary
(poses, contacts, flags, rewards, clocks), and the replay frames were released.
Exports: native RL tapes (RLPD, DfD, R2Dreamer), a pixel LeRobot set for DP
(two cameras + 8-d proprioception, no object state), and an audit HDF5 with the
original $+1$ rewards and native termination flags.

\begin{table}[htbp]\centering\small
\caption{Motion-planner hyperparameters (all values from the frozen controller and collection plans).}
\label{tab:planner-hparams}
\begin{tabular}{ll}\toprule
Parameter & Value \\\midrule
World / variant & Genesis 0.2.1, \texttt{gc\_kp4\_riser3\_shelf6}, dt $0.01$\,s, 8 substeps \\
Action mode & delta joint, cap $0.025$\,rad/frame, leash $0.125$\,rad, repeat 4 \\
Decision period & $0.12$\,s (4 frames $\times$ $0.03$\,s) \\
Horizon (collection) & $2{,}400$ frames $=600$ decisions ($72$\,s) \\
Ladder / terminals & \texttt{nested\_sparse} (+1 at \texttt{home}), tip $>60^\circ$ not-in-hand 4 frames \\
RRT-Connect & 6-d joint space, step $0.25$\,rad, goal bias $0.1$, $\le500$ iter., edge res.\ $0.03$\,rad \\
Path execution & interpolate $0.05$\,rad; $\le12$ decisions/waypoint; converge $0.015$\,rad \\
IK (transit / push) & DLS $\lambda=0.01$, 4 restarts $\times$ 80 iter., tol $1$\,mm,$0.01$\,rad / $10\,\mu$m,$0.001$\,rad \\
IK acceptance (FK check) & $\le4$\,mm position, $\le0.04$ rotation \\
Cartesian servo step & 0.004--0.010\,m/decision (phase-specific); rot.\ $\le0.06$\,rad; $\le110$ decisions \\
Servo convergence & $3$\,mm, $0.035$\,rad \\
Pre-grasp offset & $c+(-0.075,0,0)$ \\
Grasp close & gripper $-1\to+0.56$ linear over 9 decisions, hold 4 \\
Lift & $+0.16$\,m \\
Set-down target & $g+(0,+0.12,0)$, $z=z_s+h/2$; approach from $+0.04$\,m \\
Carry servo & $0.010$ / $0.005$\,m per decision (transport / lower); tilt corr.\ $\le0.06$\,rad \\
Release & gripper $-1$ for 2 decisions; withdraw $-0.06$\,m in $x$ \\
Pre-push pose & $c+R_z(\psi)(-0.015,0.065,0)$, raised $0.14$\,m; gripper $+1$ \\
Push & $2.5$\,mm steps to $c+R_z(\psi)(-0.015,0.0443,-0.008)+0.003\hat d$; stop $<0.075$\,m; tilt abort $25^\circ$ \\
Final withdraw / settle & $0.08$\,m along $-\hat d$; hold 15 decisions \\
Shadow collision res. & $0.025$\,rad; carried-can tilt limit $15^\circ$ \\
Seeds & attempt $i$: $40000+i$ ($i=0..71$); simulator seed 0 \\
Threads & 1 (CPU physics, no GPU) \\
\bottomrule\end{tabular}
\end{table}
```

Notes on the LaTeX: the reward-ladder row uses the `nested_sparse` name because that is the environment the planner ran in; the ×10 export is stated in the "Reward export" paragraph. If the appendix uses `\section` at top level (it does today), change `\subsection` → `\section` and keep the label.

---

## 3. Per-claim sources (so the appendix can be audited line by line)

| Claim | Source |
|---|---|
| Privileged reads (joints, can/goal pose, contacts, stage flags) | `experiments/full_state_planner_2026-09-12/run.py` `Controller.record/xyz/contacts/pose`, `self.info` from `env.step` |
| Action conversion `clip((q−target)/(repeat·cap))`, gripper `2·grip−1` | `run.py` `Controller.step` (line ~263) |
| Delta-joint integration, cap 0.025, leash 5×, `delta_ref='target'`, repeat 4 | `run.py` `build()`; snapshot `baselines/rl/full_env.py` `_step_once` (lines 1385–1406) |
| 0.03 s per env frame, 0.12 s per decision | `collect_episode.py` manifest `seconds_per_environment_frame=.03`; `README.md` "each environment frame advances three scene steps" |
| IK: Genesis `inverse_kinematics`, `max_samples=4, max_solver_iters=80, pos_tol 1e-3/1e-5, rot_tol .01/.001`, `respect_joint_limit=True`, warm start `init_qpos=self.qseed` | `run.py` `Controller.ik`; algorithm (DLS `J^T(JJ^T+λ²I)^{-1}e`, `damping=0.01`, `max_step_size=0.5`) from `~/workspace/Genesis/genesis/engine/entities/rigid_entity/rigid_entity.py` lines 812–830, 1178–1262 (revision `f41427d2…` per `genesis_source_manifest.json`) |
| FK acceptance 4 mm / 0.04 | `run.py` `Controller.ik` (`pe > .004 or re > .04`) |
| Side-grasp rotation (tool z → world +x) | `run.py` `self.side_rotation = [[0,0,1],[-1,0,0],[0,-1,0]]` |
| Shadow scene, exemptions (base pair, finger–can when contact, can–shelf during lower within 2 mm), carried-can tilt 15° | `run.py` class `Shadow` (`sync`, `_base_pair`, `valid`) |
| RRT-Connect params (step 0.25, goal bias 0.1, edge res 0.03, ±π bounds, `max_iterations=500`, seed = attempt seed, no shortcut) | `path_planning.py` `plan_joint_path` defaults; `run.py` `Controller.transit` (`max_iterations=500`, `seed=self.seed`; `shortcut_path` never called) |
| Path execution: interpolate 0.05 rad, ≤12 decisions per waypoint, 0.015 rad convergence | `run.py` `Controller.transit` |
| Servo: ≤110 decisions, 3 mm / 0.035 rad convergence, rot step 0.06, edge check 0.025 | `run.py` `Controller.move` |
| Carry servo: ≤85 iters, 0.010/0.005 m steps, 6 mm & tilt<8° / shelf contact & 12 mm & tilt<15°, grip 0.78 | `run.py` `Controller.carry_to` |
| Phase targets and offsets (−0.075 x pre-grasp; close ramp `linspace(0,.78,9)` + hold 4; lift +0.16; place = goal+(0,.12,0), z = shelf_top+h/2, +0.04 approach; release hold 2 at 0; withdraw −0.06 x; reposition offsets (−.015,.065,0), +0.14 z, grip 1.0 for 6; push (−.015,.044347,−.008)+.003·d, 2.5 mm steps, stop <0.075, tilt 25°; final withdraw 0.08·d; settle 15) | `run.py` `Controller.execute` |
| Finger side-face normal 14.1° (`arctan2(−.970,.243)`) | `run.py` `execute` comment + yaw formula |
| Pick verification (`picked` ∧ robot–can contact) and set-down verification (`settled_after_release`) | `run.py` `execute` (`RuntimeError('Physical pickup verification failed')`, `'Set-down did not settle after release'`) |
| Gripper motor fraction 0.78 ↔ action +0.56 | `2*0.78−1 = 0.56` (`Controller.step`) |
| `home` predicate clauses, constants (0.081 m, 20°, 12 frames, 0.10 m reach, 60° cone, 1 cm gain) | snapshot `baselines/stage_predicates.py` lines 118–160, 437–484 |
| Tip rule 60°, `not_in_hand` sustain 4 frames | snapshot `baselines/rl/full_env.py` `TIP_DEG = 60.0` (line 654), `TIP_GUARD_SUSTAIN = {'not_in_hand': 4}` (line 207) |
| World settings (kp 4, kv 2, riser 0.03, shelf +0.06, can 0.101/0.033/1000, finger 40/50, substeps 8) | `run.py` `build()` asserts; `README.md` table |
| Horizon 2400 frames / 600 decisions; dev default 1200/300 | `matched72_completed_2026-09-15/collection_plan.json` (`horizon_environment_frames: 2400, nominal_max_decisions: 600, horizon_rationale`); `run.py` `build()` `max_steps=1200` |
| Retention hold 100 scene steps, no credit | `run.py` `Controller.retention` |
| Reward ×10 export, home rows non-terminal (68), tip terminals 2, rewarded_terminals 0 | `matched72_completed_2026-09-15/export_native.py` lines 55–101; `training_v3_reference_terminals/TERMINAL_REVISION.json` (`home_rewards 68, training_terminals 2, rewarded_terminals 0`); `cluster/planner72_px_2026-09-15/CONTRACT.json` `replay_counts` |
| Starts matched to the 72 DP tapes, 67 geometries, tolerances 1e-5 / 1e-4, `reset_to({'uid': …})` | `collection_plan.json` (`matching`, `expected_geometry_tolerance`, `unique_geometries`); `matched72_completed_2026-09-15/collect.py` initial-state audit |
| Seeds 40000–40071, simulator seed 0 | `collection_summary.json` (`seed` per attempt; min 40000, max 40071, 72 unique); `collect.py` manifest `simulator_initialization_seed=0` |
| Lying-can tips = uids 234 and 318 | `collection_summary.json` attempts 18 and 34 (`recovered_trial_uid`, `can_quat` ≈ 90°) |
| First-attempt outcomes 67 home / 2 tip / 1 timeout (136 dec.) / 2 zero-action aborts (invalid start config; IK failed 8.34 mm, 0.050 rot) | `matched_starts_2026-09-14/collection_summary.json`; `analysis/dataset_dynamics_matched_2026-09-14/PLANNER_PHASES.md` §Failures |
| Recovery procedure (escape offsets (0,0,.08)/(−.08,0,.04)/away·.08+.04 z; alt pre-grasp +1 cm x / +2 cm x / +1 cm x+3 cm z; same seeds; single retry) | `matched72_completed_2026-09-15/recovery.py`; `REGISTRATION.md` (P-MP-72NONEMPTY-20260915) |
| Rev-3 outcomes: 68 home, index 56 = 17-decision escape + "Invalid goal configuration" abort, index 59 = 305-decision home with +1 cm alternative pre-grasp and a 10-waypoint RRT path | `matched72_completed_2026-09-15/collection_summary.json` (`events` of attempts 56/59); `HANDOFF.md` |
| 17,872 decisions, median 255 per tape, max 413; mean wall 110 s | computed from `matched72_completed_2026-09-15/collection_summary.json` (`transitions`, `wall_seconds`) |
| Recorded fields | `collect_episode.py` `export()`; `collect.py` (`camera_replay.npz`, `dp_raw` absolute targets `target + 0.1·a[:6]`) |
| Images from fresh single-scene replays, not the collector | `matched_starts_2026-09-14/CAMERA_CORRECTION.md`; `matched72_completed_2026-09-15/replay_audit.py`; `HANDOFF.md` ("All released camera frames are verified fresh single-scene replays") |
| 17-d state layout | snapshot `baselines/genesis_can_env.py` `_obs` (lines 432–450) |
| `rig_obs` = top RGB ++ wrist RGB, (64,64,6) uint8 | snapshot `baselines/genesis_can_env.py` lines 150–156 |
| Training corpus of record = rev 3 on the cluster | `cluster/planner72_px_2026-09-15/CAMPAIGN.json` (`local_data: …/training_v3_reference_terminals`, `n_home 68`, `n_transitions 17872`); `HRI_results/PLANNER72_AND_R2_TEACHER_QUEUED_2026-09-15.md` |

---

## Discrepancies

1. **Phase table "Motion planning 94.4 at every phase" vs the dataset docs.** `sections/04-evaluation.tex` lines 86 (table `tab:phases`, caption "Each source contains 72 demonstrations"). 94.4 % = **68/72**, which matches the **revision-3 curated training corpus** (`matched72_completed_2026-09-15/HANDOFF.md` table: pick/place/slide/home = 27+23+18 = 68 at each stage; `cluster/planner72_px_2026-09-15/CONTRACT.json` `home: 68`). The table generator confirms this: `HRI_results/dataset_dynamics_four_sources/dataset_phase_success.py` reads `matched/planner/attempts/<eid>/{episode.npz,result.json}`, and that bundle's `README.md` (line 34) states the 2026-09-17 snapshot replaced the planner files with `matched72_completed_2026-09-15`. It does **not** match the first-attempt figures quoted in `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md` and `analysis/dataset_dynamics_matched_2026-09-14/PLANNER_PHASES.md` (**67/72 = 93.1 %**), nor 67/70 non-empty = 95.7 %. So the paper's number is internally consistent with the corpus the learners trained on, but it is the rate of a **curated corpus that includes two second attempts** (`REGISTRATION.md`: "not an unbiased first-attempt planner success estimate"). The paper text/caption should say so (or footnote "first attempts: 67/72"). Also note the R2Dreamer row is likewise 94.4 at every phase — `dataset_dynamics_four_sources/README.md` line 40 gives the R2 source "68 native Nested completions and four tip failures", so that is a genuine coincidence, not a copy-paste, at least for the Nested column.
2. **"67 rewarded terminals cleared" (task brief / `PLANNER_MATCHED_STARTS` header) applies to revision 2 (70-tape corpus).** The revision actually trained on cleared **68** (`training_v3_reference_terminals/TERMINAL_REVISION.json`: `home_rewards 68, rewarded_terminals 0, training_terminals 2`). The appendix text above uses 68.
3. **Horizon.** §04 "Results" says "Episodes last at most 1200 simulation steps"; the planner (and DP) demonstrations were collected under a **2,400-frame / 600-decision** budget (`collection_plan.json` `horizon_rationale: "Match DP collection maximum of 600 decisions … old planner cap was 300 decisions"`). 5 of the 72 planner tapes exceed 1,200 frames (max 1,652 frames = 413 decisions; computed from `collection_summary.json` `frame`). The learners' evaluation horizon and the demonstration horizon therefore differ; worth a sentence in the paper.
4. **Reward ladder.** The planner ran under `nested_sparse` (+1) and its tapes were exported ×10 (`export_native.py` line 66: `reward=np.r_[0, 10*z['rewards']]`), whereas the paper says learners "received a reward of +10". Consistent after export, but the training metadata still stamps `terminal_reward=1` (legacy; `export_native.py` lines 97–101) — do not quote that field.
5. **Images.** §04 says "We recorded … 64×64 top and wrist RGB views" for all machine sources. For the planner the released images are **replays** of the recorded action tapes in a fresh single-scene environment, because the collector's own renders (taken with the planning scene alive) differed from the trainer renderer (mean abs diff 8.06/255, max 213/255 on episode 000's first frame; `CAMERA_CORRECTION.md`). States/actions/flags are the collector's; the pixels are replay-verified. The appendix paragraph "What was recorded" states this.
6. **Termination and "settle".** The paper's task description implies the slide ends in contact with the goal; native `home` is a settled-proximity predicate (planar distance ≤ 0.081 m, no contact required — `PLANNER_PHASES.md`: "not literal can–goal contact"), and the planner's push loop stops at 0.075 m. All 68 completions fired `home` during `final_withdraw`, never reaching the `settle` block (`PLANNER_PHASES.md` phase table, `settle: 0 entered`).
7. **Frame-rate metadata.** Planner LeRobot exports stamp fps = 8.333 (1/0.12 s) while the reference human/DP sets stamp 7.5 (`HRI_results/PLANNER_CLUSTER_PREPARATION_2026-09-15.md`). Same physical clock; metadata differs — not a physics difference.
8. **Five old planner jobs (3729323–27) trained on the 70-tape v2 corpus** and are excluded from the planner72 cohort (`PLANNER72_AND_R2_TEACHER_QUEUED_2026-09-15.md` "Preserved and cancelled work"). Any planner-source learner number in the paper must come from the `planner72` campaign, not those.

---

## \llm notes inserted

- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sections/04-evaluation.tex`, line 75, anchor `details about the motion planning algorithm can be found at \todo{reference appendix}.` — appended:
  `\llm{Item 01: paste-ready appendix subsection appx:planner (\subsection{Motion-planning demonstrations}) and a 1--2 sentence main-text version are in genesis_pickaplace/HRI_results/late_night_9_18/01_motion_planner_appendix.md; replace this \todo with Appendix~\ref{appx:planner}. Note: the phase table below (Motion planning 94.4 at every phase = 68/72) matches the revision-3 training corpus, not the first-attempt 67/72 = 93.1% in PLANNER_MATCHED_STARTS; see the MD Discrepancies section.}`
  (Inserted by string replacement of the unique anchor; no other text in the file was touched.)

---

## Sources

Repository (`/home/james/workspace/genesis_pickaplace`):
- `HRI_results/late_night_9_18/00_BRIEF.md`
- `experiments/full_state_planner_2026-09-12/README.md`, `run.py` (sha256 dcc65672d510f619…), `path_planning.py` (sha256 5bc96e5d1b0816a7…), `collect_episode.py`, `genesis_source_manifest.json`
- `experiments/full_state_planner_2026-09-12/snapshot_w3_committed/baselines/{rl/full_env.py, genesis_can_env.py, stage_predicates.py, eef_delta_control.py}` (frozen at commit 91fba145…)
- `experiments/full_state_planner_2026-09-12/matched_starts_2026-09-14/{HANDOFF.md, ANALYSIS.md, CAMERA_CORRECTION.md, collection_plan.json, collection_summary.json}`
- `experiments/full_state_planner_2026-09-12/matched72_completed_2026-09-15/{collect.py, recovery.py, export_native.py, replay_audit.py, HANDOFF.md, REGISTRATION.md, collection_plan.json, collection_summary.json, training_v3_reference_terminals/README.md, training_v3_reference_terminals/TERMINAL_REVISION.json}`
- `analysis/dataset_dynamics_matched_2026-09-14/PLANNER_PHASES.md`
- `HRI_results/{PLANNER_MATCHED_STARTS_2026-09-14.md, PLANNER_TERMINAL_MATCH_AND_DISPATCH_2026-09-15.md, PLANNER_QUEUED_HANDOFF_2026-09-15.md, PLANNER_CLUSTER_PREPARATION_2026-09-15.md, PLANNER72_AND_R2_TEACHER_QUEUED_2026-09-15.md, HANDOFF_PLANNER72_NONEMPTY_2026-09-15.md}`
- `HRI_results/dataset_dynamics_four_sources/{README.md, dataset_phase_success.py}`
- `cluster/planner72_px_2026-09-15/{CONTRACT.json, CAMPAIGN.json, HANDOFF.md}`
- `~/workspace/Genesis/genesis/engine/entities/rigid_entity/rigid_entity.py` (IK signature lines 812–830; solver kernel lines 1178–1262)

Overleaf (`~/workspace/overleaf/6a96f0e5337340dfd4edea88`): `sections/04-evaluation.tex`, `appendix/a-research-methods.tex`, `preamble.tex` (`\llm` macro), grep of `*.tex` for `planner|94.4|appx:`.

Commands: `python3` one-liners over the two `collection_summary.json` files (outcome counts, seeds, transitions, frames, wall time), `sha256sum` of `run.py`/`path_planning.py`, `grep`/`sed` as listed above. No cluster access was needed; nothing was committed.
