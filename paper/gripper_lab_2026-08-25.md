# Fixing the simulated gripper: contact-model diagnosis, joint retune, and what it costs (2026-08-25)

Author: Fable session on the laptop (CPU Genesis, `~/workspace/genesis_local`, LOCAL_ENV.md).
Follow-up to `paper/real2sim_follower_lab_2026-08-23.md` §5 / §9.4, which left the gripper as the
one un-fixed defect of the world of record: the sim fingers clip 7-10 mm into the 66 mm can at
80-150 N, and the two single-knob mitigations tried there (`finger_force=2.0`, `finger_map=0.735`)
each bought half the penetration at the cost of the grasp or the pick.

All numbers are LOCAL, same machine, and compared against the **world of record
`gc_kp4_riser3_shelf6`** (arm gravity comp + kp x4 + 3 cm riser + shelf standing on the table).
Cross-machine caveat unchanged: contact-marginal demos flip between this laptop (AVX2) and the
cluster (AVX-512), so read aggregates and let the cluster confirm (§8).

New, untracked-data-only code, **no tracked file modified**:
* `baselines/gripper_lab.py` -- the gripper config space + CLI (`bench|pick|full|fid|negctl|report`).
  It injects its configs into `sim_variants.VARIANTS` and wraps `sim_variants.install/post_build/
  grip_frac` at runtime, so `human_follower_lab.py`, `sim_fidelity_lab.py`,
  `fulltask_fidelity_lab.py` and `record_demos.py --sim-variant` all drive a gripper config with
  no edits of their own.

---

## 0. TL;DR

1. **The clipping is a contact-solver bug, not a gripper-calibration bug.** Every geom in this
   world carries MuJoCo's default `sol_params`, whose time constant is **0.02 s** -- 16x the
   substep and **8x the engine's own stability floor** (`2 * substep_dt = 0.0025 s`). Steady-state
   penetration scales as `timeconst^2`, so that one number is worth ~2 orders of magnitude of
   clipping. The 2026-08-23 report's §5 statement that "0.2.1 hard-wires timeconst to 2*substep_dt
   = 2.5 ms" is true of *released* genesis-world 0.2.1 and **false of the 0.2.1+270-commit tree
   this project actually runs** (§1.2).
2. **The fix is one parameter.** Set the contact time constant of the grasp pair -- the four
   finger geoms **and** the picked can, because Genesis averages a pair's params and has no
   `priority` -- to **0.005 s**. Nothing else changes: not the reading->angle map, not the finger
   PD, not the force range, not the friction, not the collision geometry.
3. **It fixes the clipping and *improves* the grasp** (local, same machine, vs the world of record
   `gc_kp4_riser3_shelf6`): finger-can penetration during carry **9.0 -> 0.9 mm** at the 95th
   percentile of frames (median-of-max 9.9 -> 1.5 mm, worst frame anywhere 15.9 -> 3.1 mm), and at
   release 0.20 -> 0.00 mm; **pick-scope recreation of the 66
   human demos 57 -> 58 kept**; full-task **tip-overs 38 -> 23, free-tips 20 -> 12**, bench in-hand
   slip 3.79 -> 1.29 mm; placed 55 -> 60, placed_v2 28 -> 31, nested 17 -> 20. Costs: strict
   `contact` 22 -> 20 and open-loop full-task picked 71 -> 69. Arm fidelity to the real robot is
   unchanged (e95 0.0285 -> 0.0300 rad).
   **So it is adoptable NOW** -- pick-scope neutral-or-better -- and it does not need to wait for
   the place/nested phase, which it also improves.
4. **The 2026-08-23 report's proposed remap is ill-posed and should be dropped.** The real
   `gripper_pos` in the bags is the Kortex *measured* motor position, and over the 75 success
   demos of the SAME 66 mm can it plateaus anywhere from **0.667 to 1.007** -- and every one of
   those values is far more closed than any geometry allows (66 mm = f 0.380 on the measured sim
   jaw, 0.405 on the vendor CAD's ~111 mm stroke). The real gripper **over-travels past contact**,
   exactly as the sim's PD does; it absorbs the over-travel in a documented soft rubber grip and a
   current-limited drive, while the rigid sim absorbs it in the CAN. The reading carries no
   aperture information and cannot calibrate a map (§1.6).
5. **What is NOT fixed, and cannot be in this engine version: the grip force.** With the
   penetration gone the finger PD keeps its full over-travel error, so the contact force *rises*
   (147 -> 245 N) against a real device whose grip is bounded by a 1.2-1.8 A motor current. Every
   way of lowering it that Genesis 0.2.1+ offers -- driver-torque caps at 10/5/2/1/0.5 N*m, finger
   `kp` 40 -> 10, the remap, pad friction 2.0 -- loses the grasp, because the base world's grasp is
   a *penetration* grasp (the pads cage the can in a converging V) rather than a friction grasp,
   and this engine has no torsional friction, no `impratio`, no elliptic cone, no NoSlip and no
   per-geom `priority` to hold a low-force pinch (§4.6, §6.3).
6. **Two other confirmed defects, both no-ops rather than errors**: finger-can friction is
   `max(1.0, 0.2) = 1.0`, so the world's `can_friction = 0.2` has never acted on the grasp
   (§1.3); and the finger collision hulls are CAD-true (pad hull volume 1.149x the STL but the
   **same AABB to 0.06 mm**) -- the hull fills concavities, it does not fatten the gripping face,
   so "shrink the pad hull" is refuted by measurement (§1.4).
7. **The URDF mimic joints ARE enforced on this tree** (3 joint-equality constraints) and are
   effectively rigid under load (violation <= 0.001 rad while the can is crushed at 250 N), so the
   distal linkage is not where the over-travel goes. `example.py`'s comment saying otherwise --
   and its "[-0.09 open .. 0.96 closed]" polarity comment, which is inverted -- are stale (§9).
8. **Cost of adoption** is the same class as the arm/shelf fix (re-harvest dDP/dR2D, re-run the
   positive and negative controls, re-collect the stride-1 human tapes) plus one new item: the
   `obs[7]` grip-effort dimension shifts (carry median 18.4 -> 26), so normalisation stats and any
   policy reading it must be refit. **Land it together with the arm/shelf fix so the bill is paid
   once** (§6.4). All numbers are LOCAL; the cluster confirms with §8.

---

## 1. Contact-model diagnosis (measured at runtime, before any tuning)

### 1.1 How penetration is measured here
Two independent measures, reported side by side in every table:

* **solver penetration** -- `collider.contact_data['penetration']`, max over finger-can contact
  pairs (what `fulltask_fidelity_lab.py` already reported);
* **geometric penetration** (new, `gripper_lab.PenProbe`) -- the can collision geom is a convex
  32-gon prism, so its exact signed distance field is `sd(p) = max_f n_f . (p - v_f)`; penetration
  is `-min sd` over **every collision vertex of the four finger geoms**, i.e. the depth of the
  deepest finger vertex below the can surface, in metres. Finger vertices are carried in
  link-local coordinates and transformed by the link pose each frame (validated against
  `geom.get_verts()` to 1.2e-4 mm). This is a pure contact-geometry quantity: it does not read the
  solver at all.

They agree to within 0.4 mm everywhere in the sweep (e.g. base world, 6 demos: 8.09 vs 8.43 mm
median-of-max), so the report's earlier solver-based number was not an artefact. The geometric
measure is the one to quote because it survives any change of solver parameters.

### 1.2 What Genesis actually does at these contacts (this tree, verified live)

| item | measured value | where |
|---|---|---|
| substeps / dt | 8 / 0.01 -> `substep_dt` 1.25 ms | `trial_placements.json['world']`, `build_world` |
| contact solver params, EVERY geom | `timeconst 0.02, dampratio 1.0, dmin 0.90, dmax 0.95, width 1e-3, mid 0.5, power 2` | `gu.default_solver_params`; read back from `solver.geoms_info` |
| solver floor on timeconst | `2 * substep_dt = 0.0025` (`TIME_CONSTANT_SAFETY_FACTOR = 2.0`) | `rigid_solver_decomp.py` `_sanitize_sol_params` + geom-init kernel |
| contact law | MuJoCo-identical: `b = 2/(dmax*timeconst)`, `k = 1/(dmax^2 * timeconst^2 * dampratio^2)`, impedance spline `imp(x)`, `x = penetration/width` | `genesis/utils/geom.py::imp_aref` |
| pair combination | friction `max(mu_a, mu_b)` floored at 1e-2; **sol_params = plain average `0.5*(a+b)`** (no `solmix`, no `priority`, no min-rule) | `collider_decomp.py:1170-1181` |
| friction, finger geoms | **1.0** (URDF default; `gs.materials.Rigid(friction=...)` is ignored for URDF entities on this tree) | read back from `solver.geoms_info` |
| friction, can / table / shelf / goal | 0.2 / 0.5 / 0.5 / 2.0 | `build_world` args |
| finger collision geometry | ONE convex hull per collision mesh, never convex-decomposed (`is_robot` -> `decompose_robot_error_threshold = inf`); 249-252 verts each | `rigid_entity.py`, read back |
| mimic joints | **3 joint-equality constraints ARE active** (`eq_type 2`, data `[0.149, -0.676]` x2 and `[0.0, -1.0]`), same sol_params as contacts | `utils/urdf.py::parse_equality`; `solver.n_equalities == 3` |
| finger dof PD | kp 40, kv 10, force range +-50 N*m on all four (world_cfg `finger_kp`/`finger_force`) | `build_world` |

**Three corrections to the 2026-08-23 report's §5 gripper row.**

1. **The contact time constant is 8x the engine's own floor, not at it.** The report says
   "timeconst = 2*substep_dt = 2.5 ms". That is true of *released* genesis-world 0.2.1 (which
   overwrote `sol_params[0]` unconditionally); this box runs 0.2.1+270 upstream commits, where the
   value is only `max()`-clamped, so the effective contact timeconst is the MuJoCo default
   **0.02 s = 16x substep_dt = 8x the floor**. Since steady-state penetration scales as
   `timeconst^2`, that single stale fact is worth a factor ~64 of clipping and is the reason the
   penetration is millimetres rather than tens of microns.
2. **`width = 1e-3` means the contact stops getting stiffer past 1 mm.** The impedance ramps from
   `dmin 0.90` to `dmax 0.95` over 1 mm of penetration and then saturates. Every grasp in this
   world lives on the saturated branch, where the contact is a plain linear spring: penetration is
   simply proportional to squeeze force. That is exactly the regime the report observed
   ("penetration ∝ force").
3. **The mimic joints are enforced, and they are effectively rigid.** `example.py`'s comment
   ("Genesis does NOT merge the URDF `<mimic>` joints: all four are independent DOFs") is only
   half true here: the dofs are independent *and* three soft joint-equality constraints pull them
   onto the URDF relation. Measured violation while a 66 mm can is being crushed at 150-300 N:
   **<= 0.001 rad** on every bench demo. So the distal linkage is NOT where the over-travel goes;
   it all goes into the can. (Consequence for other work: a policy that commands the four finger
   dofs independently -- cartesian arms, 4-dof grip action spaces -- is fighting three
   constraints of stiffness k ~ 2770. Flagged, not investigated here.)

### 1.3 Friction at the finger-can interface: the report's suspicion is right
`collider_decomp.py:1179` combines friction as `ti.max(ti.max(friction_a, friction_b), 1e-2)`.
Finger geoms carry the URDF default **1.0**, the can carries `can_friction = 0.2`, so the
finger-can coefficient **is 1.0** and the world's `can_friction` knob only ever acted against the
table (`max(0.2, 0.5) = 0.5`). Verified by reading `solver.geoms_info['friction']` back after the
build. Two consequences: (i) any past sensitivity sweep over `can_friction` was a no-op for the
grasp; (ii) to *lower* the pad-can coefficient you must lower BOTH geoms; `gripper_lab` therefore
exposes `pad_friction` (set on the four finger geoms via `geom.set_friction`, which does work)
separately from `can_friction`.

Also relevant and not fixable here: this Genesis version has a **pyramidal friction cone and a
single scalar friction coefficient** -- no torsional and no rolling friction (they arrive in
1.2.3 / 1.3.0). A can held between two pads is therefore free to spin about the contact normal,
which is a plausible contributor to the tip-over failure mode the full-task lab counts.

### 1.4 The pad hull does NOT inflate the pads
Convex hull vs the CAD STL, per finger link (trimesh, 200k surface samples):

| link | STL verts / faces | STL volume | hull volume | ratio | AABB STL vs hull (mm) | max hull-vertex distance to the STL surface |
|---|---|---|---|---|---|---|
| right_finger_dist (the PAD) | 6546 / 13088 | 9.69 cm3 | 11.13 cm3 | **1.149** | 43.58x20.05x23.59 vs 43.64x20.05x23.61 | **1.26 mm** |
| left_finger_dist (the PAD) | 6546 / 13088 | 9.69 cm3 | 11.13 cm3 | 1.148 | same | 0.30 mm |
| right_finger_prox | 8665 / 17330 | 11.59 cm3 | 25.86 cm3 | 2.231 | 52.76x39.47x24.10 vs 52.74x39.50x24.15 | 0.35 mm |
| left_finger_prox | 8655 / 17310 | 11.59 cm3 | 25.87 cm3 | 2.232 | same | 1.31 mm |

The hull adds 15 % of volume to the pad link and **zero** to its outer envelope (AABB identical to
0.06 mm): it fills the mesh's internal concavities (screw pockets, the hollow back), it does not
fatten the gripping face. **So "the hull is inflating the pads" is refuted, and shrinking the pad
hull is not an indicated fix.** This also re-confirms the 2026-07-10 finding recorded in
CAN_STARTING_POSITION.md ("0.2.1's hull collision AABB == true STL AABB for the distal pads"),
and that session's box-finger experiment (`gen3_lite_2f_boxfingers*.urdf`, `..._meshbox_d*.urdf`)
is a documented dead end -- flat boxes grip *worse* than the tapered hulls -- so it was not retread.

### 1.5 Jaw kinematics: where a 66 mm can can actually sit
Measured by sweeping the commanded closing fraction `f` kinematically and taking the world
vertices of the collision geoms (`scratchpad gl/probe_aperture.py`):

* **distal-pad clear span**: 104.79 mm at `f = 0` (fully open) down to 1.02 mm at `f = 1`, and it
  is linear to <1 %: `gap_mm ~= 104.8 - 103.8 * f`. A 66 mm can first touches the pads at
  **f = 0.380**.
* **proximal knuckles form a fixed 49.0 mm throat** that does not open further. A 66 mm can can
  therefore never be seated between the proximal links: it is always a distal-pad grasp, ahead of
  the knuckles.
* the pads are **not** a parallel jaw. The URDF mimic (`-0.676 q + 0.149`) makes the distal link's
  absolute orientation `0.324 q + 0.149` rad, i.e. an included angle of 52.7 deg wide open
  narrowing to 13.7 deg at full close: a converging V. The can is wedged in that V, which is why
  the taper grips better than a flat box (2026-07-10) -- and why the grasp keeps needing squeeze.

### 1.6 The real gripper reading over-travels: the remap premise is ill-posed
`trial_reader.py` records `base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0]
.motor[0].position` -- the Kortex **measured** gripper motor position in percent. Over the 75
success demos, the plateau reading while the can is held is

```
per-demo max reading   p10 0.693  p25 0.809  p50 0.877  p75 0.909  p90 1.004  (min 0.667, max 1.007)
```

Two things follow, and they undo the report's §5.5 proposal:

1. **The same 66 mm can produces readings from 0.667 to 1.007.** A kinematic finger position
   cannot vary by a third of full stroke on a rigid steel can. (Fail-labelled demos span the same
   range, 0.004-1.004, so it is not a success/failure discriminator either.)
2. **Every one of those readings is far more closed than any geometry allows.** 66 mm is `f = 0.380`
   on the sim jaw and `f = 0.405` on the vendor CAD's ~111 mm clear stroke. Even the *smallest*
   observed plateau, 0.667, corresponds to a 37 mm span.

So the real reading keeps advancing past contact, exactly the way the sim's finger PD does. The
real device absorbs that over-travel in its own compliance -- Kinova's user guide describes the
finger as "a hard, but flexible plastic ... and a soft, rubber-like plastic for the grip" -- and
in the drive train, until the motor hits its current limit. **The sim has rigid fingers and rigid
pads, so the entire over-travel is absorbed by the CAN.** That is the physical statement of the
defect, and it is a *contact-compliance* statement, not a *calibration* one:

> the reading -> angle map is not miscalibrated; the over-travel is real. What is wrong is where
> the over-travel goes.

Consequently `finger_map = 0.735` (fit so the 0.83 stall reading lands on the sim's 66 mm stall
angle) is fitting a parameter to a number that carries no aperture information -- it happens to
halve the penetration, but it also makes the sim jaw unable to close on anything narrower than
~28 mm, and it costs picks (report §9.4: 71 -> 65). It is reproduced below as a control
(`g_fmap`) and it is not recommended.

---

## 2. What the real device is, and what the literature says to do (sourced)

### 2.1 The Kinova Gen3 lite 2F gripper
* **It is Kinova's own integrated gripper, not a Robotiq 2F-85.** The Gen3 lite user guide never
  says "Robotiq"; it says "The robot incorporates a two finger gripper ... actuated by a linear
  actuator inside the robot wrist" (Table 18: "2 finger gripper actuated by one linear actuator").
  `ros2_kortex`: *"For the Gen3 Lite, the only option is `gen3_lite_2f`"* vs *"Possible values for
  the Gen3 are either `robotiq_2f_85` or `robotiq_2f_140`"*. And a 2F-85 masses 0.9 kg against the
  lite's 0.5 kg total payload. The myth survives because the shared Kortex API still names some
  gripper status enums `RobotiqGripperStatusFlags`.
  Sources: <https://www.kinovarobotics.com/resource/gen3-lite-user-guide> (mirror:
  <https://static.generation-robots.com/media/Kinova-lite-fiche-technique.pdf>);
  <https://github.com/Kinovarobotics/ros2_kortex/blob/main/README.md>;
  <https://robotiq.com/hubfs/Product-sheets/Adaptive%20Grippers/Product-sheet-Adaptive-Grippers-EN.pdf>;
  <https://docs.clearpathrobotics.com/docs_robots/accessories/manipulators/kinova_gen3_lite/>.
* **Kinova publishes NO grip force and NO stroke for this gripper.** Not in the user guide, not in
  the 2024 one-pager, not in the reseller sheets. This is a genuine absence. Beware the search
  result "16 mm stroke, 5 N gripping force" -- that is the **UFactory Lite 6** gripper, a different
  manufacturer (<https://www.generationrobots.com/en/404301-gripper-for-lite-6-robotic-arm.html>).
  What *is* published is the thing that actually bounds the force: **motor current limits, 1.2 A
  threshold / 1.4 A warning / 1.8 A hard limit** (user guide Table 43 "Gripper Safety items").
  Clear stroke derived from the vendor CAD is ~111 mm (this session measures 104.8 mm on the
  distal pads of the repo's URDF, §1.5).
* **There is no force command at all.** `Base.GripperMode`: `GRIPPER_FORCE = 1` is documented as
  *"Force control (in Newton) (not implemented yet)"*; `GripperCyclic/MotorCommand.force` is
  *"deprecated and unused"*; `gripper_max_force` in ros2_kortex is a percentage, not newtons.
  Feedback (`GripperCyclic/MotorFeedback`) carries position %, velocity %, `current_motor` (mA),
  voltage, temperature -- **no force field**. Practitioners infer grip force from the current peak
  (ros_kortex #296).
  Sources: <https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/enums/Base/GripperMode.md>,
  <https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/GripperCyclic/MotorCommand.md>,
  <https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/GripperCyclic/MotorFeedback.md>,
  <https://github.com/Kinovarobotics/ros_kortex/issues/75>, <https://github.com/Kinovarobotics/ros_kortex/issues/296>.
* **Pad material, official and qualitative only**: *"The finger body is composed of two different
  materials. The finger structure is made of a hard, but flexible plastic ... and a soft,
  rubber-like plastic for the grip."* No durometer, no pad thickness, no compliance figure.
* **The `-0.676 / 0.149` mimic is a linearised four-bar coupler, not a parallelogram.** A
  parallel-jaw parallelogram would be multiplier `-1.0`, offset 0. The tip joint's URDF limits
  (`-0.50, 0.21`) are exactly the mimic relation evaluated at the driver's limits, i.e. they were
  back-computed from it -- one actuator, one intended DOF, kinematically determined. Whether the
  physical coupling is a rigid bar or a spring-returned under-actuated linkage is **not published**
  and stays unresolved; the compliance Kinova documents is in the rubber grip, not in the linkage.
  Source: <https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/noetic-devel/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_macro.xacro>.

**Bottom line for modelling**: the real device is a **position servo with a fixed current limit and
no force command**. Grip force is emergent from over-travel against a current cap. That is
qualitatively what `build_world` already does (position PD past contact, force range as the cap) --
so the actuation model is right and the *contact* model is what is wrong.

### 2.2 Soft contacts: penetration vs stability
* **MuJoCo `solref`/`solimp`** (Genesis' `sol_params` is byte-identical to MuJoCo's defaults and
  its docstring cites this page): `solref = (timeconst, dampratio)` -> `b = 2/(d_width*timeconst)`,
  `k = d(r)/(d_width^2*timeconst^2*dampratio^2)`; **steady-state penetration
  `r = a0 (1-d) timeconst^2 dampratio^2`** -- quadratic in timeconst and mass-independent when
  critically damped. `solimp = (d0, d_width, width, midpoint, power)` with default
  `0.9 0.95 0.001 0.5 2`: impedance ramps over `width` and saturates. *"Larger [timeconst] values
  correspond to softer constraints."* The hard rule: *"The timeconst parameter should be at least
  two times larger than the simulation time step, otherwise the system can become too stiff
  relative to the numerical integrator ... This is enforced internally"* (`refsafe`, clamp
  `max(solref[0], 2*timestep)`).
  <https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters>,
  <https://mujoco.readthedocs.io/en/stable/modeling.html#reference>,
  <https://mujoco.readthedocs.io/en/stable/computation/index.html#parameters>,
  <https://mujoco.readthedocs.io/en/stable/XMLreference.html#option-flag-refsafe>.
* **MuJoCo on slip specifically**: *"if you see gradual slip ... the intuitive explanation may be
  that the friction is insufficient, but that is rarely the case in MuJoCo. Instead the solref and
  solimp parameter vectors need to be adjusted."*
  <https://mujoco.readthedocs.io/en/stable/overview.html#softness-and-slip>. Anti-slip playbook
  (<https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip>): force*friction >>
  weight; `condim` 4/6 for torsional/rolling friction; `multiccd` for more contact points;
  `impratio > 1` with an elliptic cone; NoSlip iterations.
* **What practitioners actually do to gripper pads -- they make them HARDER, not softer.**
  `mujoco_menagerie/robotiq_2f85` uses `solref="0.004 1" solimp="0.95 0.99 0.001"` on the pad
  geoms (5x stiffer than default, at the `refsafe` floor), `priority="1"` so the pad's contact law
  wins outright, two pad boxes instead of one for more contact points, `impratio=10` and an
  elliptic cone -- and pad friction 0.6-0.7, *below* MuJoCo's default 1.0. README, verbatim:
  *"Broke up collision pads into two pads for more contacts"*, *"Increased pad friction and
  priority"*, *"Added impratio=10 for better noslip"*.
  <https://github.com/google-deepmind/mujoco_menagerie/blob/main/robotiq_2f85/2f85.xml>.
  The opposite pattern (softer, higher-friction pad overlay) exists in robosuite
  (`panda_gripper.xml`: pad `solref="0.01 0.5"`, `friction="2 0.05 0.0001"`).
* **The coupled-jaw recipe**, verbatim from the Menagerie Franka README: *"Added an equality
  constraint so that the left finger mimics the position of the right finger"* + *"Added a tendon
  to split the force equally between both fingers and a position actuator acting on this tendon"*,
  with `forcerange` on the actuator. MuJoCo's docs endorse the clamp explicitly: *"Force clamping
  at actuator output with forcerange ... useful for e.g. position actuators, to keep the forces
  within bounds."* <https://mujoco.readthedocs.io/en/stable/modeling.html#force-limits>. The
  "command past contact and let the clamp set grip force" idiom is standard **practice** but has
  no maintainer-blessed writeup (mujoco discussion #1988 was never resolved) -- report it as
  practice, not doctrine.
* **Drake is the only vendor that commits to a penetration number**, and it is the yardstick used
  in §5 below: *"in the robotic manipulation of ordinary daily objects the user might set this
  number to 1 millimeter"*, and *"Drake's defaults are chosen so that penetrations are a few
  submillimeters for household objects"*. Also, directly on our case: *"rubber pads in a gripper
  or robotic feet are better modeled using lower contact stiffnesses. A good way to estimate
  stiffness is from known values of deformation (penetration) for a given force."*
  <https://drake.mit.edu/doxygen_cxx/group__contact__defaults.html>,
  <https://drake.mit.edu/doxygen_cxx/group__compliant__contact.html>.
* **How much penetration is acceptable for LEARNING is unquantified in the literature.** I found
  no paper measuring the effect of sim interpenetration depth on learned-policy performance or
  sim-to-real transfer. The closest evidence that rigid-pad modelling costs fidelity is
  IPC-GraspSim (arXiv:2111.01391): modelling the compliant jaw tips with incremental potential
  contact gives F1 0.85 over 2000 physical grasps, +0.09 F1 over Isaac Gym -- an F1 delta, not a
  millimetre budget. Grasp datasets treat penetration as a binary filter (AO-Grasp: *"end-effector
  intersections with objects by any amount are labeled as failures"*).

### 2.3 Which of those levers this Genesis version actually has
| MuJoCo/Drake practice | available here? |
|---|---|
| per-geom `solref`/`solimp` | **yes**, `solver.geoms_info.sol_params[idx] = [...]` at runtime (no public setter; URDF import hard-codes the default) |
| `priority` (pad's law wins the pair) | **no** -- Genesis averages, `0.5*(a+b)`. A finger-only change therefore moves the finger-can contact only HALF way |
| `solmix`, min-rule | no |
| `condim` 4/6 (torsional / rolling friction) | **no** -- scalar friction only, pyramidal cone (arrives in 1.2.3 / 1.3.0) |
| `impratio`, elliptic cone, NoSlip iterations | no |
| splitting the pad into several geoms for more contacts | possible via a custom URDF (not tried: §7 dead ends) |
| actuator `forcerange` | **yes** -- `set_dofs_force_range`, and it is already the world's `finger_force` |
| tendon force splitting | no; the two drivers are commanded symmetrically instead |
| equality mimic | **yes**, already active (§1.2) |

So the two levers Genesis 0.2.1+ leaves us are exactly **contact stiffness (`sol_params`)** and
**actuator force/gain**, plus per-geom friction. That is the config space `gripper_lab.py` sweeps.

---

## 3. `baselines/gripper_lab.py`: config space, hooks, protocol

**Config space** (every default reproduces `gc_kp4_riser3_shelf6` byte-identically):

| knob | meaning | world default |
|---|---|---|
| `base` | sim_variants variant the gripper config sits on | `gc_kp4_riser3_shelf6` |
| `map_scale`, `map_offset`, `map_max` | commanded closing fraction `f -> clip(off + scale*f, 0, map_max)` before the URDF linear angle map. `map_max` is a squeeze CLAMP ("close until the jaw is this far shut, no further"); the sim jaw is linear, `gap_mm = 104.8 - 103.8 f` | 1.0 / 0.0 / 1.0 |
| `map_obs` | invert the map on the observed reading so `obs[6]`/tape `state[6]` stays in REAL-gripper-reading units across configs | True |
| `finger_kp`, `finger_kv`, `tip_kp`, `tip_kv` | finger PD; tip overrides let the distal dofs stay stiff (rigid-linkage stand-in) | 40 / 10 |
| `finger_force`, `driver_force`, `tip_force` | +-N*m force range, all four / the two bottom drivers only / the two tip dofs only | +-50 |
| `pad_friction`, `can_friction`, `goal_friction` | per-geom friction (`geom.set_friction`) -- note the pair rule is `max`, so lowering one alone is a no-op | 1.0 / 0.2 / 2.0 |
| `pad_sol`, `can_sol`, `global_sol` | contact solver params `{timeconst, dampratio, dmin, dmax, width, mid, power}` on the finger geoms / the can / every geom. Clamped to the engine floor `2*substep_dt = 0.0025` | `0.02, 1, 0.9, 0.95, 1e-3, 0.5, 2` |
| `urdf` | alternative Kinova URDF (pad-geometry variants) | the world's own |

**Hooks.** `register_all()` (i) inserts every config into `sim_variants.VARIANTS` (inheriting its
`base` entry) and (ii) wraps `sim_variants.install / post_build / grip_frac`. `post_build` is
called by `sim_variant_hook.apply_post`, which `record_demos.build_env` already calls, so the same
config name flows through `human_follower_lab.py --sim`, `sim_fidelity_lab.py --variant`,
`fulltask_fidelity_lab.py --variant` and `record_demos.py --sim-variant` untouched. The one thing
`sim_variants` could not express is the reading->angle map inside `env.step`: `genesis_can_env` and
`full_env` hold their own references to `replay_harness.gripper_targets`, so `gripper_lab` patches
the function object in all three modules. **Missing hook, worth adding to the tracked code later:**
`build_world` should take the finger PD / force / friction / sol_params as kwargs the way the
2026-08-23 report's §7 arm patch does, and `genesis_can_env` should route the grip command through one overridable
function instead of a module-level import.

**Protocol.** Determinism on this box is exact within a process, but sequential episodes carry
solver residue, so:
* **bench** (screening only) runs many configs in ONE process, restoring a snapshot of every
  touched quantity between configs (PD gains, force ranges, per-geom friction, per-geom
  `sol_params`, and the reading->angle map -- an identity map is re-installed for configs that do
  not use one, so a map cannot leak from one config to the next). Its job is to rank, not to
  produce report numbers.
* **pick** runs **one process per episode** (`--fresh` semantics, 3 at a time), which is the
  protocol the 2026-08-23 report used for its headline follower numbers.
* **full** / **fid** shard 3 ways, one process per shard, as the host tools do.
* Every config that is quoted in §6 was run twice (the bench and the 66-demo pick run are
  independent processes and independent code paths through the env), and the `g_base` control was
  re-run through the same machinery and checked against the existing
  `_fulltask/gc_kp4_riser3_shelf6` manifest from 2026-08-23 (§5.3).

Pre-registered predictions were written before the first sweep result
(`scratchpad gl/PREREG.md`, reproduced in §7 with what actually happened).

---

## 4. Screening bench (6 short human pick tapes, one process, medians)

Replay of the REAL joint + gripper streams of uids 243/235/242/262/278/233 (short, spread over the
three can buckets and over the grip-depth range 0.68-0.89), plus 40 held frames. `pen` = geometric
penetration (§1.1) median-of-max over the closed phase; `F` = max finger-can contact force;
`slip` = max drift of the can in the EEF frame after the pick (the grasp-stability discriminator);
`tilt` = max can tilt; `f_stall` = measured closing fraction at the deepest squeeze.

| config | pick | pen_geom mm | pen_solver mm | F_max N | F_p50 N | slip mm | f_stall | tilt deg |
|---|---|---|---|---|---|---|---|---|
| **g_base** (world of record) | 6/6 | **8.09** | 8.43 | 132 | 107 | 3.79 | 0.565 | 4.8 |
| g_padstiff (fingers 0.0025 -> pair 0.011) | 6/6 | 4.16 | 4.08 | 177 | 143 | 2.13 | 0.532 | 5.0 |
| **g_stiff5** (fingers+can 0.005) | 6/6 | **1.58** | 1.60 | 265 | 166 | **1.29** | 0.490 | 5.1 |
| **g_padcanstiff** (fingers+can 0.0025) | 5/6 | **0.59** | 0.55 | 314 | 167 | **0.87** | 0.482 | 5.1 |
| g_padcanstiff_dmax99 | 6/6 | 0.54 | 0.47 | 310 | 187 | 0.92 | 0.479 | 4.9 |
| g_globalstiff (every geom 0.0025) | 6/6 | 0.64 | 0.60 | 304 | 171 | 0.97 | 0.483 | 5.5 |
| g_dmax99 (impedance 0.95 -> 0.99 only) | 6/6 | 4.21 | 4.19 | 224 | 157 | **5.71** | 0.522 | **10.8** |
| g_stiff_f10 (stiff + 10 N*m driver cap) | 6/6 | 0.56 | 0.53 | 214 | 154 | 1.09 | 0.481 | 5.1 |
| g_stiff_f5 | 6/6 | 0.46 | 0.45 | 179 | 112 | **16.98** | 0.423 | 5.1 |
| g_stiff_f2 | 6/6 | 0.41 | 0.38 | 140 | 90 | **37.47** | 0.408 | **68.5** |
| g_stiff_f1 | 6/6 | 0.39 | 0.36 | 125 | 82 | **19.73** | 0.389 | **56.0** |
| g_stiff_f05 | 6/6 | 0.40 | 0.34 | 120 | 76 | **10.90** | 0.388 | **36.5** |
| g_stiff_f5_mu2 (+ pad friction 2.0) | 6/6 | 2.19 | 2.20 | 164 | 95 | 3.03 | 0.495 | 14.7 |
| g_stiff_f2_mu2 | 6/6 | 1.69 | 1.75 | 118 | 76 | 14.05 | 0.491 | 37.4 |
| g_stiff_f1_mu2 | 6/5 | 1.52 | 1.50 | 101 | 64 | 28.02 | 0.499 | 62.9 |
| g_stiff_kp10 (stiff + finger kp 40->10) | 6/6 | 0.27 | 0.28 | 103 | 59 | 5.24 | 0.387 | 6.8 |
| g_kp10 (kp 40->10 alone) | 5/6 | 3.45 | 3.61 | 47 | 29 | **19.53** | 0.449 | **61.7** |
| g_mu2 (pad friction 2.0 alone) | 6/6 | **18.09** | 18.14 | 29 | 25 | 2.91 | 0.717 | 4.5 |
| g_fcap2 (report's finger_force=2, all 4) | 5/6 | 4.27 | 4.21 | 46 | 23 | **27.32** | 0.512 | **54.5** |
| g_fcap2_drv (2 N*m on the DRIVERS only) | 6/6 | 5.96 | 5.88 | 72 | 60 | 4.60 | 0.524 | 5.0 |
| g_fmap (report's finger_map=0.735) | 6/5 | 4.45 | 4.47 | 57 | 38 | 5.25 | 0.470 | 9.7 |
| g_fmap_stiff | 5/6 | 0.40 | 0.34 | 157 | 96 | **17.71** | 0.405 | 7.7 |

**What the bench says.**
1. **Contact stiffness is the whole story for penetration, and it costs nothing in stability.**
   `timeconst 0.02 -> 0.005 -> 0.0025` takes penetration 8.09 -> 1.58 -> 0.59 mm (the predicted
   `timeconst^2` law: 0.005/0.02 squared = 1/16, observed 8.09/1.58 = 5.1 because the pair average
   and the impedance spline blunt it), and it *improves* slip (3.79 -> 1.29 -> 0.87 mm) and leaves
   tilt flat. This is the opposite trade to the one the report assumed.
2. **Every squeeze reduction is paid for in grasp stability.** Force caps (`f5/f2/f1/f05`), the
   report's `fcap2`, the low-kp configs and the remap all cut the force but blow up slip and tilt.
   Raising pad friction to 2.0 does not rescue them.
3. **`fcap2_drv` > `fcap2`**: capping only the two DRIVER dofs (leaving the tip dofs stiff, so the
   distal linkage still behaves like the real rigid coupler) keeps a stable grasp (slip 4.6 mm,
   tilt 5.0) where capping all four destroys it (slip 27.3 mm, tilt 54.5). The report's `fcap2`
   failure was partly an artefact of clamping the mimic-emulating dofs.
4. **Pad friction alone makes the clipping WORSE** (`g_mu2`: 18.1 mm). Higher tangential friction
   lets the wedge drag the can deeper into the converging V before it locks.
5. **Impedance alone (`dmax 0.99`) is not a substitute for timeconst**: it halves penetration but
   destabilises (slip 5.7 mm, tilt 10.8, one 90-degree tip).
6. **The force never becomes physical.** With stiff contacts the fingers stall near the geometric
   contact point, so the PD error stays at the full commanded over-travel (~0.36 rad) and
   `kp * error / lever ~= 40*0.36/0.05 = 290 N`. Reducing that to a plausible real value requires
   reducing kp or capping torque, and both lose the grasp. **This is a genuine limitation of the
   model, discussed in §6.3 -- the recommended fix removes the clipping, not the over-force.**

---

## 5. Results

### 5.1 (a) PICK-SCOPE RECREATION -- the headline metric

`human_follower_lab.py --config arr_either`, 66 pruned human pick demos, **one process per
episode** (3 at a time), world of record + the gripper config. `dev_cmd` = nearest-neighbour joint
distance of every follower sim step to the REAL arm's path (the fidelity measure that stays valid
when the simulator changes). `grip_eff` = the MDP's own `obs[7]` (sum of |applied control force| on
the two finger drivers -- the sim analogue of the real gripper's motor current), median over the
tape's post-pick decisions, median over kept tapes.

| config | kept /66 | dil p50 | at-cap % | dev_cmd p50 | dev_cmd p95 | grip_eff carry | vs base |
|---|---|---|---|---|---|---|---|
| **g_base** (world of record) | **57** | 0.994 | 5.1 | 0.0027 | 0.0170 | 18.4 | -- |
| g_padstiff (fingers only) | **58** | 0.994 | 5.0 | 0.0030 | 0.0180 | 23.2 | +301 |
| **g_stiff5** (fingers+can 0.005) | **58** | 0.994 | 4.9 | 0.0038 | 0.0197 | 26.1 | +246 +301 / -316 |
| **g_padcanstiff** (fingers+can 0.0025) | **58** | 0.996 | 5.0 | 0.0037 | 0.0216 | 28.4 | +246 +301 / -254 |
| g_stiff_f10 (that + 10 N*m driver cap) | **58** | 0.997 | 4.9 | 0.0037 | 0.0217 | 20.0 | +246 +301 / -254 |
| g_fcap2_drv (2 N*m drivers, contacts unchanged) | **58** | 0.997 | 5.0 | 0.0027 | 0.0176 | 4.0 | +301 |

**Every gripper config in the shortlist is pick-scope neutral-or-better: 58/66 against the
control's 57/66.** The structure is the same in all of them: the seven irreducible drops are
234/318 (can lying at t0 -> tip rule at decision 1, intrinsic to the MDP) and 245/286/293/295/300
(1257-2526-frame demos, time-bound at the 1200-sim-step horizon -- a follower/horizon issue, not a
gripper one). Base additionally drops 246 and 301; every stiffened world recovers 301 (and the
stiffer ones also 246, the "knock-over" demo -- a stiffer contact deflects the can less on the
brush-past), at the price of one different contact-marginal demo (316 or 254). Net +1, and the
+1 is the same demo (301) in all six.

Fidelity to the REAL arm is untouched: dilation 0.994 -> 0.994-0.997, decisions at the cap
5.1 % -> 4.9-5.0 %, dev_cmd p50 0.0027 -> 0.0030-0.0038 rad (p95 0.017 -> 0.018-0.022). Those
deltas are the same size as the demo-to-demo flips, i.e. the gripper change moves the arm's path
by well under a millirad-scale margin.

*Context worth noting:* the same follower at `gc_kp4_riser3` (riser only, shelf NOT corrected)
also scores 58/66 with drop set {234, 245, 246, 286, 293, 295, 300, 318}. Raising the shelf to
stand on the table cost exactly one demo (301, which then runs out of waypoints before earning the
hardened pick), and `grasp_timeconst 0.005` gives it back. So on the pick-scope metric the
corrected world plus the gripper fix is back at the riser-only world's count while being right
about the shelf.

*Caveat, stated as required:* single flips like 316/254/246/301 are exactly the contact-marginal
class that is documented to differ between this laptop (AVX2) and the cluster (AVX-512). The
aggregate (57 vs 58) is the readable quantity; the identity of the marginal demo is not.

*Obs-distribution note:* `obs[7]` (grip effort) is part of the learners' state vector, and
stiffening raises its carry-phase value 18.4 -> 23-28 because the fingers stall nearer the can
surface and keep the full PD error. `g_stiff_f10` is the only stiffened config that leaves it at
the base world's value (20.0 vs 18.4), because a 10 N*m driver cap is almost exactly the torque
the base world's PD actually develops. This matters for re-harvesting (§6.4).

### 5.2 (c) Grasp robustness -- where it can and cannot be measured
The pick-scope tapes **terminate on the hardened pick**, so they contain no carry phase: in-hand
drift after the grant is 0.00 mm and `heldfrac` is 1.000 for every config, and no kept tape ever
raises the tip flag. Those columns are therefore uninformative by construction, and grasp
robustness has to come from (i) the bench's post-pick `slip` (§4) and (ii) the full-task replay,
which carries the can to the shelf and releases it (§5.3).

### 5.3 (b)(c) FULL-TASK replay: penetration, force, carry stability, downstream stages
`fulltask_fidelity_lab.py` on all 75 success demos (the human's REAL joint + gripper command
streams, per-trial placements, south goal), 3 shards, one process each. `pen_c` = finger-can
penetration during the closed (carry) phase, median-of-max and max-of-max over demos; `F_c` =
median-of-max contact force; `e50/e95` = whole-episode joint tracking error vs the real arm.

**Control check first.** `g_base` (the world of record driven through `gripper_lab`'s
registration + monkeypatch machinery, with every gripper knob at its default) reproduces the
2026-08-23 `gc_kp4_riser3_shelf6` manifest **exactly**: 75/75 uids, **0 differing fields** across
picked / picked_hard_at / placed / placed_v2_at / contact / nested / tipped / tipped_free /
pen_max_closed / force_max_closed / err_inf_p95. So (i) the harness is a genuine no-op on the
control, and (ii) this box is bit-reproducible across two days -- both preconditions for reading
the deltas below.

| variant (grasp contact timeconst) | picked | hard | placed | placed_v2 | contact | nested | tipped | tipped_free | pen_c p95 mm | pen_c med-of-max mm | pen_c worst mm | F_c p95 N | F_c med-of-max N | e50 | e95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **g_base** (0.02 s, world of record) | **71** | **70** | 55 | 28 | **22** | 17 | 38 | 20 | 9.00 | **9.90** | 15.90 | **130** | **147** | 0.0033 | 0.0285 |
| g_padstiff (fingers only -> pair 0.011 s) | 69 | 69 | **64** | 27 | 20 | 18 | 33 | 19 | 4.00 | 4.80 | 9.00 | 173 | 202 | 0.0035 | 0.0289 |
| **g_stiff5** (fingers+can 0.005 s) | 69 | 68 | 60 | **31** | 20 | **20** | **23** | **12** | **0.90** | **1.50** | **3.10** | 185 | 245 | 0.0035 | 0.0300 |
| g_padcanstiff (fingers+can 0.0025 s) | 69 | 68 | 51 | 27 | 17 | 18 | 27 | 15 | **0.40** | **0.60** | 2.90 | 182 | 267 | 0.0036 | 0.0314 |
| g_stiff_f10 (0.0025 s + 10 N*m driver cap) | 69 | 68 | 52 | 26 | 19 | 18 | 28 | 16 | **0.40** | **0.50** | 3.70 | 153 | 214 | 0.0035 | 0.0313 |

(`pen_c p95` = the 95th percentile over carry FRAMES, median over demos -- i.e. what the contact
looks like almost all the time; `med-of-max` = the median demo's single worst frame; `worst` = the
worst frame of the worst demo.)

Post-release penetration and force are 0.00 mm / 0.0 N in every stiffened world (0.20 mm / 1.3 N
in the base world): once the gripper opens it stops touching the can at all.

**Reading.**
* **Penetration falls monotonically and exactly as the `timeconst^2` law predicts, blunted by the
  impedance spline and by the rising force**: 9.90 -> 4.80 -> 1.50 -> 0.60 mm median-of-max, and
  the worst demo in the whole set goes 15.9 -> 3.1 mm at `g_stiff5`. **`g_stiff5` is the first
  configuration that lands on Drake's stated yardstick for household manipulation**: 95 % of carry
  frames at **0.90 mm** (vs 9.00 mm today) against Drake's ~1 mm "penetration allowance" and "a
  few submillimetres for household objects", with the worst frame anywhere in the 75 demos at
  3.1 mm.
* **Carry stability improves, it does not degrade**: tip-overs 38 -> 23 and free-tips (the MDP's
  own tip rule: tilt > 60 deg while the grip is open) 20 -> 12 at `g_stiff5`, with 10 demos that
  free-tip in the base world no longer doing so and only 2 new ones. That is the single strongest
  signal here, and it is the opposite of what the `finger_force=2.0` mitigation did in the
  2026-08-23 report (tips 38 -> 62, free-tips 20 -> 39).
* **Downstream stages are neutral-to-better at `g_stiff5`**: placed 55 -> 60, placed_v2 28 -> 31,
  nested 17 -> 20, contact 22 -> 20. (`contact` and `nested` disagree in sign because `contact` is
  a hard `get_contacts` test sampled every 30 frames while `nested` is the settled proximity test;
  with less penetration the can rests a couple of mm further from the goal and the strict test
  samples through fewer touching frames. `nested` is the metric of record.)
* **The cost is 2 picks in the open-loop full-task replay** (71 -> 69 picked, 70 -> 68 hardened;
  uids 255, 303, 316 lost, 331 gained). Note this is the *open-loop replay* pick, not the
  pick-scope recreation metric, which goes the other way (57 -> 58, §5.1). Both are contact-
  marginal-demo counts on a laptop.
* **More stiffening is not better, and adding a torque cap does not help either.**
  `g_padcanstiff` (0.0025 s) buys 0.9 mm more of penetration than `g_stiff5` and gives back placed
  (60 -> 51), contact (20 -> 17), nested (20 -> 18) and tips (23 -> 27). `g_stiff_f10` --
  `g_padcanstiff` plus the 10 N*m driver cap that keeps `obs[7]` at the base world's value and
  holds the carry force to 153 N p95 (vs 185 for `g_stiff5`, 130 for base) -- lands in the same
  place as `g_padcanstiff` downstream (placed 52, placed_v2 26, contact 19, nested 18, tips 28) and
  is beaten by `g_stiff5` on every one of them. `g_padstiff` (fingers only, the most surgical
  option) only gets to 4.8 mm and keeps base-level tips. **0.005 s on both geoms, with no torque
  cap, is the interior optimum of the four.**
* Arm tracking is untouched (e50 0.0033 -> 0.0035, e95 0.0285 -> 0.0300, carry-phase e95 0.0267 ->
  0.0272) -- see also §5.4.

### 5.4 (d) Arm fidelity to the REAL robot -- a gripper change must not move it
`sim_fidelity_lab.py`: the human's REAL measured joint stream replayed as the sim's PD targets
over all 66 tapes (39 332 frames), tracking error `e = cmd - q_sim`.

| variant | e_inf p50 | e_inf p95 | frac > leash | frac > cap | static sag p95 (max joint) | err at grip close | hardened pick re-earned |
|---|---|---|---|---|---|---|---|
| g_base | 0.0043 | 0.0324 | 0.14 % | 13.2 % | 0.0031 | 0.0017 | 64/66 |
| g_stiff5 | 0.0054 | 0.0353 | 0.49 % | 15.1 % | 0.0050 | 0.0026 | 63/66 |

Unchanged for practical purposes -- the deltas are 1 mrad on p50 and 3 mrad on p95, against the
0.047 -> 0.004 rad that the arm fix itself bought (report §4). The small increase is the expected
direction: a stiffer contact pushes back harder on the arm while the fingers are closing on the
can, so a little more of the tracking error appears in the grasp window (`err at grip close`
0.0017 -> 0.0026 rad, i.e. still 10x inside the 0.025 rad cap). The raw-replay hardened pick is
re-earned on 63/66 vs 64/66 (the misses are the lying-can ICs 234/318 plus one marginal).


### 5.5 (e) Negative controls
**Fail-labelled demos (16), full-task replay** -- the same open-loop streams from demos the human
failed. A world change must not manufacture downstream success on them.

| variant | picked | placed | placed_v2 | contact | nested | tipped | tipped_free | pen_c med mm | F_c med N |
|---|---|---|---|---|---|---|---|---|---|
| old world (`base`) | 10 | 8 | 6 | 2 | 2 | 0 | 0 | 7.30 | 114 |
| **g_base** (`gc_kp4_riser3_shelf6`) | 10 | 4 | 1 | 2 | 1 | 11 | 9 | 6.50 | 77 |
| **g_stiff5** | 8 | 5 | 3 | 2 | 2 | 10 | 6 | **1.00** | 89 |

`contact` stays at 2/16 and `nested` at 2/16 -- the documented 1-2 false-positive floor of the
proximity predicate (the 12.5 % nested-FP the 2026-07-20 note records), unchanged from both the
old world and the world of record. Nothing is manufactured. The 8-10/16 `picked` is the long-known
FK-seeded-IC replay artefact and is essentially variant-independent. Free tip-overs on the fail
demos go 9 -> 6, consistent with the success-demo result.

**Random teacher (`record_demos.py --teacher random --ic-mode demo --sim-variant g_stiff5`),
pick scope: 3 shards x 30 rollouts = 90 episodes, `kept = 0/90`, every one `env_truncated` at the
1200-sim-step horizon; `teacher_rate = 0.0`.** The recorder's own built-in ceiling
(`kept <= n_roll/30`) is not approached. The stiffened contact does not make the pick predicate
reachable by accident.

## 6. Recommendation

### 6.1 What to adopt
**One parameter: the contact solver time constant of the grasp pair (the four finger geoms AND
the picked can), `0.02 s -> 0.005 s`.** Nothing else. Not the reading->angle map, not the finger
PD gains, not the force range, not the friction, not the collision geometry.

```
grasp_timeconst = 0.005     # = 4 * substep_dt; engine floor is 2 * substep_dt = 0.0025
```

Scorecard against the world of record, same machine, ranked as instructed -- (a) pick-scope
recreation and (c) grasp robustness first:

| metric | `gc_kp4_riser3_shelf6` | + `grasp_timeconst 0.005` | verdict |
|---|---|---|---|
| **(a) pick-scope recreation, kept /66** (arr_either, fresh process per episode) | 57 | **58** | **better** |
| **(c) free tip-overs, full task /75** (the MDP's own tip rule) | 20 | **12** | **much better** |
| **(c) tip-overs, full task /75** | 38 | **23** | **much better** |
| **(c) in-hand slip after the pick, bench median** | 3.79 mm | **1.29 mm** | **better** |
| **(b) finger-can penetration, carry** (p95 of frames / median-of-max / worst frame) | 9.00 / 9.90 / 15.90 mm | **0.90 / 1.50 / 3.10 mm** | **10x better; lands on Drake's ~1 mm yardstick** |
| (b) finger-can penetration, release | 0.20 mm | **0.00 mm** | better |
| (b) contact force, carry (p95 of frames / median-of-max) | 130 / 147 N | 185 / 245 N | **worse** (see §6.3) |
| downstream: placed / placed_v2 / nested | 55 / 28 / 17 | **60 / 31 / 20** | better |
| downstream: contact (strict, sampled) | 22 | 20 | slightly worse |
| full-task open-loop picked / hardened | 71 / 70 | 69 / 68 | slightly worse |
| **(d) arm fidelity to the REAL joint stream** (e50 / e95 / carry e95, rad) | 0.0033 / 0.0285 / 0.0267 | 0.0035 / 0.0300 / 0.0272 | unchanged |
| (a) fidelity of the recreated tapes to the real arm (dev_cmd p50/p95, rad) | 0.0027 / 0.0170 | 0.0038 / 0.0197 | unchanged |
| (a) dilation p50 / decisions at the cap | 0.994 / 5.1 % | 0.994 / 4.9 % | unchanged |
| **(e) random-teacher negative control, kept** | (n/a) | **0/90** | passes |
| (e) fail-demo replay: contact / nested false positives /16 | 2 / 1 | 2 / 2 | at the documented FP floor |

**It is adoptable NOW.** It is pick-scope neutral-or-better (58 vs 57), it improves the grasp
stability metrics rather than trading them away, and it does not need to wait for the place/nested
phase -- it improves that too (nested 17 -> 20, placed 55 -> 60, placed_v2 28 -> 31, tips 38 -> 23).
The only regressions are the strict `contact` count (22 -> 20) and the open-loop full-task pick
(71 -> 69), both single-digit contact-marginal-demo counts on one machine.

**But adopt it *with* the arm/shelf fix, not separately.** Both are world changes with the same
re-harvest bill (§6.4); landing them together pays it once. The recommended world of record
becomes `gc_kp4_riser3_shelf6 + grasp_timeconst 0.005`.

**If a single number must be defended in the paper**: the finger-can interpenetration during
carry drops from 9.0 mm to 0.9 mm at the 95th percentile of frames -- from a jaw sitting 18 mm
(27 % of the can's diameter) inside the can's surface, visible in every wrist-camera frame the
image arms consume, to the sub-millimetre range Drake calls acceptable for household manipulation
-- while the pick-phase recreation of the human demos goes 57/66 -> 58/66 and the tip-over count
is nearly halved.

### 6.2 Exact patch (flagged; defaults leave today's world byte-identical)

**1. `can_pos_recovery/replay_harness.py::build_world`** -- one new kwarg, applied after
`scene.build()` (the geoms exist only then), next to the existing finger gain/force block:

```diff
 def build_world(show_viewer=False, backend='gpu', finger_force=None, finger_kp=None,
                 can_height=BOTTLE_HEIGHT, can_rho=2000, substeps=1,
                 table=False, can_radius=BOTTLE_RADIUS, camera=False, can_friction=0.2,
                 urdf_file='gen3_lite_2f_robotiq_85.urdf', urdf_extra=None,
                 constraint_timeconst=None, rigid_extra=None,
-                table_friction=0.5, goal_friction=2.0, table_top=TABLE_TOP_Z, rig_res=64):
+                table_friction=0.5, goal_friction=2.0, table_top=TABLE_TOP_Z, rig_res=64,
+                grasp_timeconst=None):
@@
     ff = [10, 10, 5, 5] if finger_force is None else [finger_force] * 4
     kinova.set_dofs_force_range(
         lower=np.array([-50, -50, -50, -20, -20, -20, -ff[0], -ff[1], -ff[2], -ff[3]]),
         upper=np.array([50, 50, 50, 20, 20, 20, ff[0], ff[1], ff[2], ff[3]]),
         dofs_idx_local=kdofs)
+    if grasp_timeconst is not None:
+        # Contact softness of the GRASP pair (finger pads <-> picked can), in seconds.
+        # Genesis' per-geom sol_params default to MuJoCo's (timeconst 0.02, dampratio 1,
+        # dmin .9, dmax .95, width 1e-3, mid .5, power 2) -- 8x the engine floor
+        # 2*substep_dt -- which lets the finger PD drive 8-10 mm INTO the can at 150 N.
+        # A contact pair's params are the plain average of the two geoms', and there is no
+        # `priority` in this engine, so BOTH the fingers and the can must be set.
+        # See paper/gripper_lab_2026-08-25.md.
+        _sol = scene.sim.rigid_solver
+        _tc = max(float(grasp_timeconst), 2.0 * float(_sol._substep_dt))
+        assert _tc == float(grasp_timeconst), f'grasp_timeconst below the engine floor {_tc}'
+        _sp = [_tc, 1.0, 0.9, 0.95, 1e-3, 0.5, 2.0]
+        for _l in kinova.links:
+            if 'finger' in _l.name:
+                for _g in _l.geoms:
+                    _sol.geoms_info.sol_params[_g.idx] = _sp
+        for _i in range(bottle.geom_start, bottle.geom_end):
+            _sol.geoms_info.sol_params[_i] = _sp
```

**2. `can_pos_recovery/trial_placements.json` `['world']`** gains the key, and every consumer
reads it the way it already reads `finger_kp` / `finger_force` -- **by subscript, not `.get()`**,
so a stale world file fails loudly (silent-default rule, AUDIT_REQUEST_Fable.md):

```diff
   "world": { "can_height": 0.101, "can_radius": 0.033, "can_rho": 1000,
              "finger_kp": 40.0, "finger_force": 50.0, "substeps": 8, "table": true,
+             "grasp_timeconst": 0.005 }
```
Call sites to thread it through (each already passes `finger_force`/`finger_kp` from `world_cfg`):
`baselines/genesis_can_env.py::GenesisCanEnv.__init__`, `baselines/genesis_vec_env.py`,
`baselines/fulltask_fidelity_lab.py`, `can_pos_recovery/remeasure_contact.py`,
`can_pos_recovery/render_all_episodes.py`, `render_videos.py`, `batch_harness.py`,
`goal_nested_fit.py`, `tip_onset.py`, `probe_table_height.py`.

**3. `baselines/sim_variants.py`** -- one key in `VARIANTS` and six lines in `post_build`, so the
change is A/B-able the way the arm fix was:

```diff
+    # gripper contact stiffness: sol_params timeconst (s) on the four finger geoms AND the
+    # picked can; None = untouched (engine default 0.02).  paper/gripper_lab_2026-08-25.md
+    'gc_kp4_riser3_shelf6_grasp5': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0,
+        effort='base', riser=0.03, shelf_dz=0.06, grasp_timeconst=0.005),
@@ def post_build(w, name):
+    if v.get('grasp_timeconst') is not None:
+        sol = w['scene'].sim.rigid_solver
+        tc = max(float(v['grasp_timeconst']), 2.0 * float(sol._substep_dt))
+        sp = [tc, 1.0, 0.9, 0.95, 1e-3, 0.5, 2.0]
+        for l in w['kinova'].links:
+            if 'finger' in l.name:
+                for g in l.geoms:
+                    sol.geoms_info.sol_params[g.idx] = sp
+        for i in range(w['bottle'].geom_start, w['bottle'].geom_end):
+            sol.geoms_info.sol_params[i] = sp
     return dict(name=name, ..., grasp_timeconst=v.get('grasp_timeconst'))
```
(`gripper_lab.apply_gripper` is exactly this code plus the knobs that were swept and rejected.)

### 6.3 What this does NOT fix (state it plainly)
**The simulated grip force stays unphysically large.** With a stiff contact the fingers stall
within ~0.5 mm of the can surface, so the finger PD keeps the whole commanded over-travel as
position error: `tau = kp * (f_cmd - f_contact) * 1.05 ~= 40 * 0.36 = 14 N*m` per driver, i.e.
~250-300 N at the pad, against the base world's ~150 N and against a real device whose grip
force is bounded by a 1.2-1.8 A motor current (unpublished in newtons, but certainly tens of
newtons, not hundreds).

Every way of reducing it that this engine offers was swept and every one costs the grasp
(§4.2): driver-torque caps at 10/5/2/1/0.5 N*m, finger `kp` 40 -> 10, the reading remap, and pad
friction up to 2.0 to compensate. The mechanism is visible in the numbers: in the base world the
grasp is not a friction grasp, it is a **penetration grasp** -- the pads sink 8-10 mm into the can
and cage it in a converging V (§1.5). Take the penetration away and only friction is left, and
this Genesis version has a pyramidal cone with a single scalar coefficient and **no torsional
friction**, so a can held between two angled pads can still spin and walk out. The levers that
MuJoCo's own anti-slip playbook would reach for next -- `condim 4/6`, `impratio` with an elliptic
cone, NoSlip iterations, a `priority` pad contact law, splitting the pad into several geoms --
**do not exist in this Genesis version** (§2.3).

So the honest statement of the result is: *the clipping is a contact-stiffness bug and it is
fixable today; the over-force is a contact-model limitation of the engine version and it is not.*
The recommended change removes the geometric artefact -- each pad buried 9 mm into a 66 mm can, so
the jaw sits 18 mm (27 % of the diameter) inside the can's surface in every wrist-camera frame the
image arms consume -- without touching the grasp the rest of the pipeline was built on.

### 6.4 What it costs
Same class of cost as the arm/shelf fix, and for the same reason -- it is a **world** change, not
an MDP change (obs layout, delta/cap/leash semantics, action repeat, horizon, reward/terminal,
ICs, `pick_z`, goal, tip rule are all untouched):

1. **Re-harvest the model-demo sets.** `dDP` and `dR2D` were harvested by teachers trained in the
   old world; their competence has to be re-measured before their tapes mean anything
   (PREREG §3.1). Same for any ouroboros lineage.
2. **Re-run every positive control and the random negative control** in the new world.
3. **Re-collect the stride-1 human tapes** -- every consumer of `states` (BC/DP, dHunpruned)
   otherwise trains on old-world states. Note this is the *same* re-collect the arm/shelf fix
   already requires, so if the two land together it is paid once.
4. **`obs[7]` (grip effort) shifts**: carry-phase median 18.4 -> 26-28 raw units under the
   recommended config (20.0 if the 10 N*m driver cap is taken with it). It is a state dimension,
   so normalisation statistics and any policy that reads it must be refit; a mixed-world buffer
   would be silently inconsistent. Every tape/manifest already carries `sim_variant`, and the
   consumers already assert it -- that assertion is what stops the mix, and it must be extended to
   the new world name.
5. **Nothing else moves.** No predicate constant is world-geometry-dependent here (unlike the
   shelf fix): `pick_z`, `BOX_TOP_Z`, `SHELF_REST_Z`, `goal_start_z`, the placed z-band and the
   can placements are all untouched, and the can x,y recovery keeps its validity (the full-task
   replay re-earns the hardened pick on the same demos, §5.3).
6. **Cross-machine**: these are laptop numbers. Contact-marginal demos are exactly the class that
   flips between AVX2 and AVX-512, and a *contact* change is the one most likely to move them, so
   the cluster must re-run §8 before anything is adopted for the paper.

## 7. Notes, dead ends, and the pre-registered predictions

**Pre-registered predictions vs outcome** (written before the first sweep, `scratchpad gl/PREREG.md`):

| # | prediction | outcome |
|---|---|---|
| P1 | contact stiffness is the dominant lever on penetration and nearly free of grasp quality; `padcanstiff` cuts 8-10 mm to 0.2-1 mm at the same force and the same-or-better pick | **CONFIRMED** (0.59 mm, slip improves 3.79 -> 0.87); force is NOT the same, it rises 1.5-2.4x (§4.6) |
| P2 | `dmax 0.95 -> 0.99` cuts penetration a further ~5x | **PARTLY WRONG**: alone it halves penetration but destabilises (slip 5.7 mm, a 90-degree tip); on top of a stiff contact it adds nothing (0.59 -> 0.54 mm) |
| P3 | the report's two failures reproduce | **CONFIRMED** (`g_fcap2` slip 27 mm / tilt 55; `g_fmap` loses a hardened pick and doubles slip) |
| P4 | with stiff contacts a physical squeeze becomes viable (`stiff_f5`/`stiff_f2` keep the pick that `fcap2` lost) | **REFUTED** -- this was the main hypothesis and it is wrong. Once the penetration is gone the grasp loses the form closure the penetration was providing: every cap <= 5 N*m slips badly on the bench (17-37 mm) and tips the can. A 10 N*m cap (`g_stiff_f10`) does hold the grasp (slip 1.1 mm, pick-scope 58/66) but buys nothing downstream -- full task placed 52, nested 18, tips 28 against `g_stiff5`'s 60 / 20 / 23 -- so the cap is not part of the recommendation (§4.2, §5.3, §6.3) |
| P5 | capping only the two driver dofs beats capping all four | **CONFIRMED** (slip 4.6 vs 27.3 mm, tilt 5.0 vs 54.5) |
| P6 | pad friction alone does nothing to penetration | **WRONG in sign** -- it makes it much worse (18.1 mm), because the wedge drags the can deeper |
| P7 | global stiffening risks table/shelf instability and will not be recommended | **held** (it works on the bench but is strictly broader than the finger+can change for no measured gain) |

**Dead ends and things deliberately not retread.**
* *Shrinking the pad collision hull.* Ruled out by measurement, not by preference: the hull's outer
  envelope is the STL's to 0.06 mm (§1.4). The 2026-07-10 session already built and tested
  CAD-true box fingers (`gen3_lite_2f_boxfingers{,_d3,_d5,_d7}.urdf`,
  `gen3_lite_2f_meshbox_d{0,3,5}.urdf`, still in the tree) and found flat boxes grip *worse* than
  the tapered hulls (form closure) -- see CAN_STARTING_POSITION.md. Not repeated.
* *Splitting each pad into several collision geoms* (the Menagerie "more contacts" trick) would
  need a new URDF and a re-verified grasp; `gripper_lab` has the `urdf` knob for it but it was not
  run. Genesis 0.2.1+ has no `priority`, so a multi-geom pad cannot be given its own contact law
  either -- it would only add contact points.
* *`solver.set_global_sol_params()` is broken in this Genesis build* -- it passes a 1-D array to
  `_sanitize_sol_params`, which `np.stack(..., axis=1)`s it and raises `AxisError`. `gripper_lab`
  writes `solver.geoms_info.sol_params[i]` per geom instead. (Latent, unrelated: 
  `replay_harness.build_world` accepts `constraint_timeconst=` and forwards it into
  `gs.options.RigidOptions`, which has no such field on this tree -- the field is called
  `constraint_resolve_time` here and `constraint_timeconst` only from Genesis 0.3.3. Nothing in
  the repo passes it today, so it has never fired.)
* *Calibrating the reading->angle map on the real stall reading* -- ill-posed, §1.6.
* *Lowering `can_friction` to weaken the grasp* -- a no-op: the pair rule is `max` and the fingers
  are pinned at 1.0 (§1.3).
* *Squeeze clamps* (`map_max`, "close until the jaw is 58 mm and stop") are implemented in the
  config space and are the cleanest way to make the force physical, but they are **object-specific
  by construction** (the clamp encodes the can's diameter) and they contradict the real device,
  which over-travels (§1.6). Not carried forward.

**Missing hooks in the tracked code** (would remove every monkeypatch in `gripper_lab.py`):
`build_world(..., grasp_timeconst=None, finger_kv=None, pad_friction=None)`; a single overridable
`grip_targets` entry point in `genesis_can_env` instead of three module-level imports of
`replay_harness.gripper_targets`; and `world_cfg` keys for all of the above so that every
consumer (`GenesisCanEnv`, `genesis_vec_env`, `remeasure_contact`, `render_*`) inherits them.

---

## 8. Reproduce on the cluster

```bash
# 0. the module registers itself into sim_variants; nothing tracked is modified
python baselines/gripper_lab.py list

# 1. screening bench (one process, ~20 min for the whole grid on 6 short tapes)
python baselines/gripper_lab.py bench --tag grid1 \
    --cfg g_base g_padstiff g_padcanstiff g_stiff5 g_dmax99 g_padcanstiff_dmax99 g_globalstiff \
          g_fcap2 g_fcap2_drv g_fmap g_fmap_stiff g_mu2 g_kp10 g_stiff_kp10 \
          g_stiff_f10 g_stiff_f5 g_stiff_f2 g_stiff_f1 g_stiff_f05 \
          g_stiff_f5_mu2 g_stiff_f2_mu2 g_stiff_f1_mu2

# 2. (a) PICK-SCOPE RECREATION -- the headline metric. ONE PROCESS PER EPISODE, 3 at a time.
for c in g_base g_stiff5 g_padcanstiff g_stiff_f10 g_padstiff g_fcap2_drv; do
  python baselines/gripper_lab.py pick --cfg $c --parallel 3
done

# 3. (b)(c) FULL TASK -- penetration/force per phase, tips, placed/contact/nested
for c in g_base g_padstiff g_stiff5 g_padcanstiff g_stiff_f10; do
  python baselines/gripper_lab.py full --cfg $c --parallel 3
done
python baselines/gripper_lab.py full --cfg g_stiff5 --label fail --parallel 3   # neg control

# 4. (d) arm fidelity vs the REAL joint stream (must not move)
python baselines/gripper_lab.py fid --cfg g_base   --parallel 3
python baselines/gripper_lab.py fid --cfg g_stiff5 --parallel 3

# 5. (e) random-teacher negative control (must keep ~0)
python baselines/gripper_lab.py negctl --cfg g_stiff5 --n 30 --parallel 3   # 3 x 30 rollouts

# 6. tables
python baselines/gripper_lab.py report
```
Outputs: `baselines/demos_v1/_grip/_bench/bench_*.json`, `_lab/arr_either@<cfg>/manifest.json`,
`_fulltask/<cfg>/manifest_{success,fail}.json`, `_simlab/<cfg>/manifest.json`.
Measured cost on this laptop (i7-8665U, 3 concurrent processes): bench 22 configs x 6 tapes
~25 min total; **pick 17 min per config** (66 fresh processes); **full 28-37 min per config**
(75 demos x 3 physics steps per frame); full `--label fail` 11 min; fid 10-12 min; negctl 37 min.
Total for everything in this report: ~7 h wall clock.

---

## 9. Stale comments found while doing this (comments only; behaviour is correct)
* `example.py:171` "Genesis does NOT merge the URDF `<mimic>` finger joints: all four are
  independent DOFs (dof_idx_local left_bottom=8, right_bottom=6, left_tip=9, right_tip=7)".
  Measured on this tree: the dofs ARE independent *and* three joint-equality constraints enforce
  the mimic; and the local dof indices are left_bottom=**7**, right_bottom=6, left_tip=9,
  right_tip=**8** (`kdofs = [0,1,2,3,4,5,7,6,9,8]`). Behaviour is unaffected because everything is
  indexed through `kinova.JOINT_NAMES`.
* `example.py:202` "Driver = right_finger_bottom, [-0.09 open .. 0.96 closed]" is **inverted**:
  measured span is 104.8 mm at theta = 0.96 (open) and 1.0 mm at theta = -0.09 (closed).
  `harcoded_start`'s comment on line 176 ("fingers = open config") is the correct one, and
  `gripper_targets(..., invert=True)` is consistent with it.
* `replay_harness.build_world:97` "0.2.1 hard-wired [constraint_timeconst] to 2*substep_dt
  (=0.0025 at ss=8)" -- true of *released* genesis-world 0.2.1, false of the 0.2.1+270-commit tree
  actually installed, where the value is 0.02 (§1.2). This is the fact the whole clipping problem
  hinges on.
* `paper/real2sim_follower_lab_2026-08-23.md` §5 gripper row repeats that 2.5 ms figure and says
  "Genesis 0.2.1: no mimic"; both are wrong for this tree.

---


---

## Appendix A -- per-uid changes, world of record vs `grasp_timeconst 0.005`

**Pick-scope recreation (66 demos, `arr_either`, fresh process per episode).**
Common drops in both worlds: 234, 318 (can lying at t0 -> tip rule at decision 1, intrinsic to the
MDP) and 245, 286, 293, 295, 300 (1257-2526 source frames, time-bound at the 1200-sim-step
horizon). `g_base` also drops 246 and 301; `g_stiff5` recovers both and drops 316 instead.

| uid | g_base | g_stiff5 | what changed |
|---|---|---|---|
| 301 | `adapter_exhausted` (179 dec, 716 sim, waypoints 595/595) | **picked** (166 dec, 661 sim) | the grasp earns the hardened pick before the waypoints run out |
| 246 | `env_truncated` (300 dec, 1200 sim, 1191/1556 wp) | **picked** (240 dec, 958 sim) | the "knock-over" demo: a stiffer contact deflects the can less on the brush-past, so the grasp lands and the episode ends early instead of running the horizon out |
| 316 | picked (163 dec, 636/779 wp) | `adapter_exhausted` (225 dec, 779/779 wp) | contact-marginal flip |

**Full task (75 success demos, open-loop replay of the real streams).**

| predicate | lost by `g_stiff5` | gained by `g_stiff5` |
|---|---|---|
| picked | 255, 303, 316 | 331 |
| contact | 246, 255, 259, 267, 297, 298, 321, 325, 328 | 237, 242, 261, 262, 293, 295, 315 |
| nested | 247, 267, 297, 320, 330 | 237, 248, 259, 261, 262, 293, 309, 317 |
| tipped_free (**fewer is better**) | -- | stops tipping: 255, 258, 261, 286, 287, 301, 303, 326, 333, 335 (10); starts tipping: 247, 256 (2) |

The free-tip column is the cleanest signal in the whole study: ten demos that tip the can over in
the world of record stop doing so, and only two new ones appear.

## Appendix B -- artefacts produced

| path | what |
|---|---|
| `baselines/gripper_lab.py` | the module (new, untracked) |
| `baselines/demos_v1/_grip/_bench/bench_grid1.json` | the 22-config screening bench, per-uid rows |
| `baselines/demos_v1/_lab/arr_either@g_{base,padstiff,stiff5,padcanstiff,stiff_f10,fcap2_drv}/` | pick-scope tapes + `manifest.json` (66 each, contract v1, `validate_tape` on every one) |
| `baselines/demos_v1/_fulltask/g_{base,padstiff,stiff5,padcanstiff,stiff_f10}/manifest_success.json` | full-task per-uid records; `g_stiff5/manifest_fail.json` = the fail-demo negative control |
| `baselines/demos_v1/_simlab/g_{base,stiff5}/manifest.json` | arm-fidelity records |
| `baselines/demos_v1/_grip/negctl_g_stiff5/` | random-teacher negative control |
| scratchpad `gl/` | `PREREG.md`, `probe_geom.py`, `probe_hull.py`, `probe_setters.py`, `probe_aperture.py` (+`aperture.json`), `picktab.py`, `fulltab.py`, `tapecarry.py` |
