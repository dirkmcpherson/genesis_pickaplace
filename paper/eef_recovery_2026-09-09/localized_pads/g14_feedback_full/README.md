# Fresh-feedback full replays: local gains, no general e2e improvement yet

Ten declared calibration runs and one independent saved-action verification are
complete. The controller feedback correction is independently established by
`../g14_feedback/`: no physical parameter changed to eliminate the oscillation.
All runs retain original time, path, grip, initial placement and early-day yaw.
No appended motions, holds or modified scoring thresholds were used.

## Completed task outcomes

| Demo | Contact formulation | Rigid .02 s | Soft .03 s |
|---|---|---|---|
| 233, late day | Pyramidal / ratio1 | Retention fails: final gap2.967 mm | **Complete:** gap.956 mm, supported push23.07 mm |
| 233, late day | Elliptic / ratio10 | No supported push to contact; final distance104.82 mm | **Complete:** final distance65.99 mm |
| 113, yaw-corrected early day | Pyramidal / ratio1 | No supported release; final distance210.87 mm | No supported release;221.40 mm |
| 113, yaw-corrected early day | Elliptic / ratio10 | No supported release;23.50 m | No supported release;4.15 m |
| 184, yaw-corrected early day | Pyramidal / ratio1 | No supported release;20.47 m | No supported release;24.81 m |

Long-distance failures follow loss of the can and continued motion/rolling on
the unbounded floor during the original long recording. They are not near-misses
at the final slide. Trial184 holds at20–22 s but loses the can during23–25 s;
real video previously showed it retained through33 s. Neither pad setting fixes
that mismatch. See `summary.json` for every condition and actual goal displacement.

The cleanest local softness benefit is233 pyramidal: its soft run finishes with
only.12 mm final goal displacement (.30 mm maximum), versus.06 mm for rigid.
The elliptic soft233 run shifts the goal9.86 mm, versus.09 mm rigid, which must
remain visible when interpreting its completion. The new pyramidal rigid/soft
strict count is0/3 versus1/3 on the three calibration demos. The preceding pinned
aligned rigid/soft count was1/3 versus0/3; this is not a gain over the best existing
control on that same three-demo panel. No population claim or bank adoption.

## Carry improvement and remaining failure

Elliptic contact directly transfers the bench's creep improvement to113 carry:
both rigid and soft versions remain held through20 s (can center about.282 m),
while both pyramidal versions are already down and hand-free by12 s. Real video
retains the can through the previously verified18 s checkpoint. Both elliptic
versions then lose the can during descent around21 s and are tipped/hand-free by
22 s. Thus the carry gain is a contact-formulation effect, not a softness-specific
result, and the full task still fails. Release/descent is the next divergence to
inspect rather than assuming the old early carry loss remains unchanged.

## Review artifacts and validation

- `233_fresh_soft_real_sim.mp4`: entire trial233, two clock-matched real cameras,
  sim overview and hand close-up.161 frames decoded and SHA verified; maximum
  camera timing errors16.3/20.9 ms. `233_video_verification.json` records checks.
- `113_elliptic_carry_release_real_sim.mp4`: selected6–24 s carry/release segment,
  with failure called out in the annotation. This is a diagnostic clip, not a
  complete-task success video.
- `233_soft_seating.png` uses the existing fixed real rim annotations and camera
  hypotheses. Nominal rim errors are12.21/7.63/25.46/34.15 px at14/16/22/25 s;
  matching rigid11.85/7.45/25.22/33.53 px. Differences under.7 px do not establish
  better visual seating, and placement still sits lower relative to the wrist
  than the real reference. Camera uncertainty and unmeasured3D pose remain.
- `action_verification.json`: replaying233's saved seven-dimensional action tape
  reproduces every saved array exactly, including the final successful sequence.
- `readout.py`: verifies frozen wrapper source hashes, realized URDF hash,
  unchanged targets/grip/mount, source-frame coverage, no extra motion, per-substep
  qpos/feedback agreement, callback counts and actual pad treatment activity.
  Supplied `slide_predicate` and strict `eef_task_sequence` remain unchanged.

The videos render saved poses using the existing pinned renderer; they do not
rerun candidate physics. Viewpoints differ and are not registered overlays.
The new engine's analytic primitive mass and processed mesh differences remain
cross-engine confounds.

## Neutral self-collision filter

New Genesis defaults `enable_neutral_collision=False`; its world-coordinate voxel
check disables the opposite distal-finger pair in the113 yaw16 build, but not233.
This is identical within each rigid/soft pair. `neutral_collision_audit.json`
records the discrepancy. A separate collision-only check explicitly enables the
pair and finds **zero pair contacts across all4,187 saved decision poses** of
113 pyramidal soft (`113_neutral_pair_pose_audit.json`). This weakens that pair
as an explanation of the observed113 carry loss, but does not cover intermediate
physics substeps or prove dynamically identical behavior. Explicit policy control
is required before adoption.

No tested configuration yet establishes improved real2sim for the complete task
across the intended recovery set. Next useful tests are the elliptic descent
failure against real footage, and a frozen broader panel for the local233 soft
benefit; any proposal must also preserve original-control fidelity and final
release/slide behavior. Do not substitute carry or bench success for the goal.

The inspected113 clip shows the real can still upright around20.7 and22.5 s while
the simulated can falls and tips. This does not by itself establish whether the
real can is still held or has been set down for a regrasp. `113_descent_onset.json`
records the simulated opening/contact-loss neighborhood; inspect real support
and vertical seating before treating it as another friction failure.
