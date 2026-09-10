# Localized pads: active end-to-end recovery experiment

Objective: find a soft-pad configuration that improves real2sim for the complete
pick, carry, shelf release and final slide. This remains unproven. The preceding
assessment was evidence-gathering progress; this stage implements and executes
the comparison. No candidate is admitted to a training bank.

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

- `INITIAL_READOUT.md`: five runs completed; divided-pad geometry rejected.
- `SURFACE_READOUT.md`: four runs completed; surface-only compliance improves
  loaded image position but does not recover the whole sequence.
- `SPRING_READOUT.md`: six runs completed; stronger return springs alone do not
  recover the task. Includes the reference-drive stiffness/damping derivation.
- `matched_drive_plan.json`: next nine runs, with paired original-contact
  controls; `matched_drive_execution.json` exists only after the batch terminates.

The first 15 full replays are all on calibration trial 233, not 15 independent
demonstrations. No pad candidate has yet passed the end-to-end gate or been
evaluated on the reserved validation trials. See `LIVE_STATUS.json` for the
last verified process handle; re-poll it before inferring that work has stopped.
