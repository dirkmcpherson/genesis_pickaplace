# Proximal-only extra compliance: completed candidate rejected

All seven declared replays completed. The candidate keeps original geometry,
source commands, initial poses and corrected day yaws. It applies the existing
.03 s soft response only to inner proximal pad regions; distal pads retain
.02 s. Elliptic friction and impedance ratio 10 are fixed within every pair.
Visible proximal blue covering motivates this spatial hypothesis, but its
material and thickness are unknown. This is not hardware calibration.

Both the matching rigid hand and proximal-only candidate complete 0/3 strict
sequences and pass 0/3 supplied predicates. The all-soft 233 control reproduces
its prior completion, including approximately 9.86 mm final goal displacement.

| UID | Rigid final distance, m | Proximal-only final distance, m | Candidate failure |
|---|---:|---:|---|
| 113 | 23.503 | 53.062 | No supported release |
| 184 | 23.182 | 8.957 | No supported release |
| 233 | 0.1048 | 0.2403 | No supported push to contact |

Large distances follow failed handling and subsequent motion on the unbounded
simulated floor; they are retained as failures. Rigid/proximal final goal motion
is 60.32/59.85 mm in 113, 19.293/8.801 m in 184, and .086/54.21 mm in 233.
These outcomes do not justify adoption or a broader validation batch.

`readout.py` verifies source commands, complete frame coverage, realized URDF and
preset hashes, fresh feedback and callback counts, paired collision policy,
and material activity by geometry. Rigid 113, rigid 233 and all-soft 233 reproduce
every array of their archived fresh-feedback references exactly. All seven
execution records and source snapshots are retained under `execution.json`,
`plan.json`, `logs/` and `executed_sources/`.

There is an important treatment-exposure qualification. At the saved scene-step
samples, proximal-only material activations total just three in 113, versus
7,544 in 184 and 12,678 in 233. Distal activations are exactly zero in every
proximal-only run. The all-soft 233 control records activity on both proximal
and distal pads. Thus 113's failure does not test strongly engaged proximal
compliance. `diagnostic_plan.json` declares an unchanged-source contact-location
audit to determine where its detected contacts lie; that diagnostic is separate
from the completed seven-run comparison.

The contact audit is now complete and reproduces every saved trace array exactly.
Across 1,401 scene-step samples at 7–21 s, it records 8,663 detected can–finger
contacts: 8,660 distal and only three proximal. All contacts from 7–18 s are
distal and satisfy the material-region mask. The three proximal contacts occur
near release and are classified correctly. Thus poor exposure is caused by
which fingers touch the can, not accidental exclusion of sustained proximal
contacts by the mask. These are detected contacts before force solution, not
force-weighted support measurements. `contact_location_readout.json`,
`diagnostic_execution.json` and `113_contact_audit/contact_locations.json` retain
the evidence. The extra full replay brings this stage to eight completed runs.

The inspected `113_seating.png` retains the early axial seating mismatch.
Its cap-relative error changes by at most .51 px versus all-soft 113, below the
approximately 5 px landmark uncertainty. At 20.16 s the can loses hand contact
above the shelf, and no supported release follows. `113_descent_samples.json`
retains the sampled contact/pose evidence. The inspected `233_seating.png`
shows failed later placement after the proximal-only material change.

`../g14_release_diagnostic/README.md` explains the independent multi-view seating
evidence that preceded this test. No fitted real-can pose was injected into a
simulation or used to change an initial placement. No engine, URDF, or candidate
is adopted into the canonical experiment or training banks.
