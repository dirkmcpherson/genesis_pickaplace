# Localized pads: active end-to-end recovery experiment

Objective: find a soft-pad configuration that improves real2sim for the complete
pick, carry, shelf release and final slide. This remains unproven. The preceding
assessment was evidence-gathering progress; this stage implements and executes
the comparison. No candidate is admitted to a training bank.

Latest result: 393 completed full-source replays/controls across48 distinct
demos. Expanded verification is complete:144/144 replays and36/36 groups.
Original / soft1 / soft2 supplied metric6/36,6/36,7/36; strict3/36,1/36,1/36.
Soft2's additional191 proximity pass moves the goal178.10mm with no supported
release. No e2e pad improvement or candidate adoption.

The user requests reconsideration of the approach. See
[the reassessment](../APPROACH_REASSESSMENT_2026-09-10.md): make faithful full-task
recovery the objective and retain pads as a secondary hypothesis. No further
pad sweep is queued. The soft-pad goal remains unachieved; the proposed broader
goal is a recommendation, not a completed or registered replacement.

CG1000 removes the observed cap exits and restores233 GPU completion; the native
CPU control matches the independent replay exactly. CPU/GPU can trajectories
still differ by up to14.02mm, and8/16 substeps retain different outcomes despite
zero cap exits. The float64 CG/Newton references are now terminal0 after an exact14-array
control gate. Both saved sequences complete233; physical/numerical readout remains
to be reviewed. No simulation batch remains live.

Preceding calibration follow-up: 181 full replays are complete across the same12 unique demos.
Normal-only doubled damping passes every assembled-row/nonmutation check and
two identity controls reproduce every archived array. On the original calibration
trio, rigid / soft have supplied-metric passes0/3 /1/3 and strict completions0/3
for both. Soft233 makes a supported40.9mm slide and ends0.468mm short of geometric
contact, with0.636mm goal movement. A complete verified real/sim review is in
`normal_damping/233_normal_damping_real_sim.mp4`. Both early originals and both
conditional113 treatments fail supported release. No candidate is admitted.

`normal_damping/README.md` reports all10 runs and image comparisons. The goal
estimate itself used233/242 contact constraints, so submillimeter strict gaps
are not independently calibrated real-world errors. Both scores remain intact.
A frozen36-run four-condition comparison across the existing day-balanced9-demo
panel has terminal results in `elliptic_panel/`; it measures the broader elliptic
contact effect separately from softness and normal damping. No new damping level
or initial pose is selected for it.

Previous coupled-damping investigation ended at171 full replays.
The fixed-elastic-coefficient, doubled-contact-damping candidate is rejected:
matching rigid / soft both score 0/3 under each full-task criterion on the original
calibration trio. Both conditional113 comparisons also fail release. It loses
undamped soft233 completion. All four identity replays reproduce every archived
array; all treated contacts preserve the engine's static reference term and
multiply both normal and tangent velocity terms as declared. See
`contact_damping/corrected/README.md`. Twelve completed full replays include
four identity verifications; eight separate first-attempt compilation failures
occurred before stepping and are excluded from the full-replay count. No bank
or parameter adoption. Normal-only damping remains untested.

Preceding release investigation:
The exact release-force audit identifies shelf-edge ejection after finger
separation in archived 113 (`release_wrenches/README.md`). A closer real-video
check corrects the provisional impression of support at 20.04 s: the real can
also moves downward relative to the hand before settling.

A nominal image-derived initial XY, declared before its outcomes, improves
loaded seating under both rigid and soft pads (`vision_initial_probe/README.md`).
All three conditional comparisons fail end-to-end. Rigid gains upright supported
release but loses the final push; soft still fails release. The pose estimate
remains conditional on camera and table alignment and is not adopted. Two exact
force-observation replays of this pair are complete: soft opening adds downward
impulse before the shelf supplies backward ejection. See
`vision_initial_probe/release_wrenches/README.md`. A clock-matched rigid release
review is available at `vision_initial_probe/113_conditional_rigid_release_real_sim.mp4`;
its successful settling is followed by failed task continuation.

Proximal-only extra compliance also fails all three calibration demos
(`proximal_compliance/README.md`). Its observation replay finds 8,660 of 8,663
detected can–finger contacts on distal tips in 113 during 7–21 s. No material
configuration has established an end-to-end improvement.

Preceding validation result: the fresh-feedback Genesis 1.4 soft candidate is rejected
on the frozen nine-demo panel. Original / matching rigid / soft strict completions
are 2/9, 1/9, 1/9; supplied-metric passes are 3/9 for all three. All 18 executions
finished, bringing the experiment ledger to 144 completed full replays, including
historical candidates and independent verification runs, across 12 unique demos.
See `g14_validation/README.md`. These repeated selected recordings do not measure
population recovery. `g14_release_diagnostic/README.md` records a conditional
early seating mismatch in 113; camera uncertainty prevents a placement correction.
`TEXTURE_ASSESSMENT.md` retains effective roughness as an untested hypothesis.
The sections below preserve the progression of earlier experiments.

## Candidate and controls

The actual processed collision meshes were exported from the current full-world
Genesis build (`reference/manifest.json`). Four original finger geometries are
partitioned into twelve: each link has an inner cap, backing and joint-adjacent
region. The cap has a maximum normal depth of 3 mm, with a local x boundary at
-8 mm. Inner-face directions follow the CAD outline; material boundaries and
depth remain explicit assumptions. This is not a measured Kinova pad model.

`geometry_review_v2/regions.png` shows the proposed regions. The construction
preserves the reference convex envelope: volume relative error is below 2e-16
and support difference in 2,000 fixed directions is zero before mesh export.
The engine's left distal reference contains a tiny missing face; a closed hull
is reconstructed from the same vertices. Loading/export can change ordering and
rounding, so the reconstructed-hull control is required and is not assumed to
reproduce the old trace bit-for-bit.

Generated pieces bypass repeated mesh decimation only; the original arm and
other objects retain their usual loading path. Original visual meshes, link
inertias, joints, adaptive transmission, friction, commands and world settings
remain fixed. Actual collision region counts, parameter assignment and callbacks
are checked during replay. Pad, backing and joint-region contact observations
are saved separately after each original scene step.

The first declared batch is trial 233 with reconstructed unsplit hulls, split
geometry at original contact parameters, and split geometry with pad time
constants 0.025/0.03/0.04 s. The backing and can retain 0.02 s. Effective pair
time constants are the means. These are exploratory solver settings, not elastic
moduli. `initial_plan.json` and `initial_code_sha256.json` pin this batch.

## Evidence needed for the objective

An improvement must survive the whole task, not just a loaded-grasp image.
Compare against both the original fixed-coupling replay and the adaptive hand,
as well as the split-rigid control. Trial 233 originally completes the strict
sequence; the adaptive hand currently loses it. Restoring only its supplied
proximity-metric pass is insufficient.

After an initial candidate improves the joint grasp/release result, extend to
the calibration set 113/184/233 (corrected early-day yaws), freeze the choice,
and evaluate 176/185/237 without fitting to those replays. Those outcomes were
seen historically; they are excluded from new fitting, not pristine unseen data.
Record failures and regressions as well as gains. Validation needs both unchanged
metric-of-record scores and strict physical sequence evidence, with real-video
checks of carry, opening and supported final slide. If localized compliance has
no advantage over split-rigid geometry, attribute the effect to geometry rather
than claiming a soft-pad recovery improvement.

No changes to the supplied predicate, source EEF/gripper commands, timestamps,
initial placements or yaw are permitted as a route to passing. No og4, added
push or terminal hold. A final candidate also requires saved-action replay
verification and annotated real/sim review artifacts. Six selected trials alone
cannot establish a broad population recovery rate; report the scope explicitly.

`summary.json` is generated from successful terminal execution records only;
failed executions are retained as errors. Incomplete jobs are not scored.

## Readouts and current direction

`HAND_MECHANICS_READOUT.md` consolidates all 48 completed calibration replays.
The current surface-pad candidate improves loaded seating and restores a real
supported slide in 233 relative to the same hand with original contact. It still
stops 4.19 mm short of goal contact and fails pickup in both early calibration
trials. It is not accepted as a general end-to-end improvement.

The divided-geometry approach described above is historical and rejected; current
pads use original geometry with a spatially localized contact law. See
`INITIAL_READOUT.md`, `SURFACE_READOUT.md`, and `SPRING_READOUT.md` for the first
stages. Every subsequent batch has a plan and terminal execution records.

The frozen candidate and rigid controls have completed evaluation on 176/185/237.
`VALIDATION_READOUT.md` reports the paired six-demo results: supplied-metric
passes are 3/6 original fixed, 2/6 matching rigid and 3/6 soft; strict completions
are 3/6, 0/6 and 1/6. Independent saved-action checks cover 233 and 176. `LIVE_STATUS.json` records the
last verified process handle; poll it rather than restarting a quiet batch.
`233_soft_pad_real_sim.mp4` is the inspected annotated real/sim review video.

## Subsequent inertia audit

The default 0.1 kg m² added inertia on every finger causes large unloaded
curl and oscillation in the adaptive hand. `unloaded_armature/README.md`
records a causal bench and corrects the earlier inertia-free settling argument.
`reference_hand_pads/README.md` reports six original-hand attribution controls;
its extra proximity pass is rejected as early-release mismatch.
`low_armature/README.md` reports seven completed mass comparisons.
`critical_return/` tests a spring/force setting derived from the verified bench.
The goal remains unachieved; no candidate has been adopted.

## Completed expanded validation and current assessment

The critical-return candidate is rejected on all nine declared validation pairs: supplied-metric passes original / matching rigid / soft are 3/9, 2/9, 1/9; strict completions 2/9, 2/9, 1/9. See `critical_validation/README.md` for per-demo distances and goal displacement. All 19 jobs completed, including exact saved-action verification.

Seven further limited-compression calibration runs are complete (`compression_limit/README.md`); neither early pickup was recovered. `TEXTURE_ASSESSMENT.md` considers physical roughness through localized effective friction without additional geometry. `finger_inertia_alignment/README.md` records a separate coordinate-sign audit and an isolated, not yet runtime-tested URDF candidate. The e2e goal remains unachieved.

The inertia-sign audit now has a verified runtime bench and seven completed full comparisons. Aligned rigid / soft strict completions are 1/3 / 0/3; no e2e gain. `contact_diagnostics/README.md` reports an exact read-only replay of the preceding soft 113 failure and its slowed real/sim video. That older-hand approach diagnostic should not be substituted for the corrected-hand release failure. All batches from this stage are terminal.

`aligned_contact_diagnostics/README.md` verifies carry creep below friction capacity in an exact corrected-hand replay. `engine_contact_review/README.md` reports four completed engine-contact bench conditions: newer elliptic friction with tangential impedance10 sharply reduces drift while retaining assigned normal softness. This is a candidate direction only; full-world1.4 adaptation and task validation remain.

The first full Genesis1.4 controls are now complete (`g14_full_replay/README.md`). All four corrected-frame233 runs fail pickup; soft settings cause metres of goal displacement. The port exhibits pre-grasp finger oscillation despite matching declared mechanics. Diagnose integrated hand dynamics before interpreting further contact-material changes. No engine or pad adoption.

The new-engine oscillation is now traced to stale controller feedback and corrected without changing mechanics (`g14_feedback/README.md`). Ten fresh-feedback calibration runs plus exact saved-action verification are complete (`g14_feedback_full/README.md`). Soft233 improves retention within the new-engine pair; elliptic113 improves carry through20s but fails descent/release. Both early pyramidal pairs fail. No general e2e gain or adoption; annotated233 and113 review videos are provided.
