# Broader elliptic-contact panel: frozen comparison, no adoption

The prior nine-demo panel tested pyramidal friction. The broader effect of the
elliptic contact model has not been measured across those recordings. This
experiment separates that gap from the latest normal damping hypothesis.

The fixed UID list is176/185/237/156/181/224/198/243/246: three recordings per
day. Each gets rigid/soft pads at normal damping gains1 and2, all in Genesis1.4
elliptic/impedance-ratio10. The original fixed-hand replay is retained as a
cross-engine reference. There are36 declared new full replays. No source
movements, early yaw, initial poses, shelf, goal or scoring thresholds change.

These recordings were used in prior model comparisons and are not pristine
unseen data or independent participants. The settings and UID list are frozen
before this experiment. Results must show gains and regressions, per-day
outcomes, goal movement and real-video fidelity before any adoption. The
conditional113 placement is excluded from this panel.

This broadens evidence gathering despite the incomplete calibration result;
it does not waive the end-to-end requirement. Repeated tuning on three
calibration recordings, one with independently identified placement uncertainty,
cannot establish the overall effect of the elliptic contact model. No new
damping level is selected here.

`run.py` requires the preceding normal-only implementation comparison to finish
with its invariants passing before launch. `readout.py` reports pending and
failed executions separately and compares aggregate outcomes only on complete
four-condition UID groups. Every selected row mapping, untouched reference,
contact parameter, original source command and paired collision/geometry policy
is checked. This is a forward-simulation experiment; no backward/autodiff claim.

See `plan.json` and `execution.json` after launch for the actual frozen manifest
and completed processes. This document alone is not evidence that a run started.

The historical goal estimate used end-contact constraints from233/242. Strict
contact differences of less than a millimeter are therefore not independent
measurements of real-world geometric error. Both the unchanged supplied metric
and strict physical sequence are reported, with that calibration limitation and
goal movement, rather than treating one binary label as the entire fidelity test.

## Latest results: no overall recovery gain established

All36 attempts are terminal:33 full replays completed and three UID181 conditions
stopped on reference-verification checks. Their cause is unresolved; they are
not scored as physical task failures. Results use the same eight demos with
all four conditions complete.

| Configuration | Supplied slide metric | Strict ordered task sequence |
| --- | --- | --- |
| Original fixed hand | 3/8 | 2/8 |
| Elliptic rigid, normal damping1 | 2/8 | 0/8 |
| Elliptic soft, normal damping1 | 2/8 | 1/8 |
| Elliptic rigid, normal damping2 | 2/8 | 1/8 |
| Elliptic soft, normal damping2 | 2/8 | 2/8 |

Soft damping2 completes the same176/237 strict sequences as original, adds no
new strict recovery, and loses224 under the supplied metric.176 goal movement
improves7.629→0.028mm, a possible local fidelity gain requiring video review.
237 goal movement worsens4.257→47.181mm; its strict pass alone cannot establish
faithful reconstruction. Original224 metric-only success also moves the goal
86.770mm and fails the strict sequence, so metric counts alone do not rank fidelity.

Both yaw-corrected early days and the late day are included; no new strict
recoveries occur on either. These are selected, previously used recordings,
not a population estimate. See `summary.json` for all attempts. No adoption.
Resolve UID181 verification stops before reporting a complete nine-demo comparison.

## Ninth comparison completed

The verification issue is resolved in `verification_retry_v3/README.md`, with
all original attempts preserved. Two exact repeat controls and independent
Jacobian checks pass. Full nine-demo results are original3/9 metric,2/9 strict;
rigid1 2/9,0/9; soft1 2/9,1/9; rigid2 2/9,1/9; soft2 2/9,2/9.
No new strict recovery or candidate adoption. The eight-demo table above records
the earlier incomplete readout; use `verification_retry_v3/merged_summary.json`
for the completed comparison.
