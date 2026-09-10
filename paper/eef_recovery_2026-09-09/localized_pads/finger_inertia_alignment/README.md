# Finger inertial-coordinate audit

`audit.py` compares the four watertight finger meshes with their URDF inertial
blocks. Three published centers of mass are outside their own mesh bounds.
The published inertia magnitudes closely match uniform-density mesh integration,
but several center-of-mass and left-finger product-of-inertia signs disagree.
The pinned official raw CAD URDF contains the same values; this is not evidence
of a new transcription error introduced by the current pad experiment.

`candidate_check.json` records an isolated proposed correction: right proximal
and distal COM x become negative; left proximal COM x/y and left distal COM y
become negative; both left Ixy entries become negative. Right Ixy is already
consistent with its mesh and stays positive. Masses and all magnitudes remain
unchanged. Every non-inertial XML element is exactly unchanged.

Uniform-density centroids corroborate the proposed signs within 16 micrometers;
they are not measurements of the real two-material fingers. A mass distribution
outside the represented geometry would be an alternative explanation for an
out-of-bounds COM. The canonical URDF has not been modified.

The isolated candidate has now passed runtime mass/COM/tensor/quaternion
readback and all 951 steps of the unloaded bench. All seven full-task calibration runs completed; no general recovery improvement
has been established. `can_pos_recovery/align_finger_inertia.py` refuses the canonical URDF
and records provenance when applied to an isolated candidate.

## Runtime bench

`bench_comparison.json` applies identical calculations to both traces. Original / aligned peak joint-target error is 2.116 / 2.156 degrees; peak unloaded-relation error 1.289 / 1.370 degrees. Hold-end joint-target error falls from .00189 to .0000134 degrees. Aligned peak substep speeds are symmetric at 4.047 rad/s proximally and 3.620 rad/s distally. Do not present this as reduced peak target error or as calibrated hardware bandwidth.

`full_task_plan.json` declares six calibration comparisons and one original-inertia wrapper control. The latter already reproduces every saved array of critical-return rigid 233 exactly. The batch is terminal; it must not be restarted.

## Full-task result: soft setting rejected

| Demo | Aligned rigid final distance, mm | Aligned soft final distance, mm | Strict rigid / soft |
|---|---:|---:|---|
| 113 | 210.79 | 431.09 | No supported release / no supported release |
| 184 | 109.11 | 101.15 | No supported push / no supported push |
| 233 | 66.09 | 68.52 | Complete / final not retained |

Both 113 runs now pass the scorer's pickup gate, but neither reaches supported
release. Both 184 runs reach supported release but not the required final push.
Soft 233 passes the immutable proximity metric but has a 2.52 mm final surface
gap and fails the unchanged strict ending tolerance. Thus soft strict completions
are 0/3 versus aligned rigid 1/3; no new e2e recovery is established.

Goal displacement remains problematic: rigid/soft 113 end with 76.25/71.88 mm
goal displacement, and 184 with 20.11/22.12 mm. Scoring against a stationary
initial goal would hide these interactions and is not used.

The existing real-rim comparison, with no new pose fit, gives aligned rigid/soft
cap-relative errors 9.45/7.43 px at 14 s, 8.09/5.55 at 16 s, 28.64/27.56 at 22 s
and 36.39/34.54 at 25 s. Softness improves all four over the aligned rigid hand,
but original fixed-coupling placement errors were lower (15.59/26.99 px at
22/25 s). The inspected soft overlay still places the can too low relative to
the real wrist during placement, conditional on the existing camera hypothesis.
This is not a calibrated 3D seating measurement.

The completed wrapper's outer factory URDF hash precedes alignment. Its nested
`inertia_alignment.after_sha256` is correct; `readout.py` verifies that against
the actual file for every corrected run. Archived `run_aligned_finger_inertia_v1.py`
identifies the executed code. The current wrapper refreshes the outer hash for
future runs; no completed trajectory or metadata was rewritten.
