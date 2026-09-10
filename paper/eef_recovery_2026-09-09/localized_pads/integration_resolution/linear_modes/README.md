# Local hand linearization

Three unloaded poses (grip 0, 45, 90) use the engine's finger mass matrix with
its implicit damping contribution removed. The resulting physical mass matrix
is positive definite. The declared spring stiffness, damping and added armature
give fastest natural angular frequencies of approximately 1111–1120 rad/s.
At eight substeps, each fastest-mode period has only 4.49–4.53 integration steps.

The linearized explicit-spring, implicit-damping update has spectral radius below
one at all four resolutions. It does not establish a local linear instability.
Exact proximal opposition is an idealization here: finite equality-solver error,
contacts, force saturation, moving targets and full-arm coupling are excluded.
This cannot prove stability or identify the cause of a full-task failure.

The separate unloaded trajectory refinement also becomes closer from 16 to 32
substeps (`../../unloaded_resolution/summary.json`). These are three static mass
audits, not full demonstrations, and no physical parameter is calibrated by them.
