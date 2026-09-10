# Experiment checkpoint and local artifacts

This checkpoint preserves the recovery implementation, frozen experiment plans,
executed source snapshots, candidate descriptions, numerical audits, per-trial
JSON results, figures and real-image review evidence. It records failed and
rejected candidates as well as successes. No pad or engine configuration is
adopted, and no training bank is changed.

New binary replay arrays, rendered videos and runtime logs remain local, following
the repository's policy of transferring datasets outside Git. `LOCAL_ARTIFACTS.json`
lists their repository-relative paths, sizes and SHA256 digests. It is an inventory,
not a backup of those files. Preserve or copy them separately to reproduce the
exact archived readouts. Existing already-tracked arrays/videos are unchanged.

Readouts require the corresponding local arrays and, where indicated, the
original real recordings under `inthewild_trials`. Launchers retain the frozen
source, preset and reference hashes. Most are single-use launchers that refuse
to overwrite an existing plan or output; do not rerun them inside an archived
experiment directory. Use a new output directory for further work.

The tested engine is the isolated Genesis1.4 environment described in the
experiment reports, with a separate supported Torch2.8 CUDA overlay for native
GPU tests. This is distinct from the project's pinned original engine. No
environment installation or upgrade is committed here.

Shared simulation files had pre-existing changes during these experiments.
Their complete executed versions are preserved in the native benchmark's
`executed_sources` and `supported_sources` directories. The functional early-day
yaw additions needed by the recovery path are committed separately from unrelated
shared-file edits; exact historical reproduction should use the frozen sources
named in each plan, rather than assume every live shared file equals the archive.
