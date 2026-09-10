# Integration resolution: full-source numerical sensitivity diagnostic

This three-case study keeps the soft material and hand parameters fixed while
refining numerical substeps8/16/32 (1.25/0.625/0.3125ms). Scene dt10ms, recorded
control interval30ms, source commands and endpoint stay fixed. It is motivated
by full-task differences across solver implementations and a zero-spin extra row.
It is not a material parameter search or independent recording sample.

The8-substep control reproduces all14 archived233 arrays exactly. At16 substeps,
233 fails supported release and ends225.73mm from the goal, versus65.99mm and
strict completion at8. Maximum can-position difference207.47mm;501 decision
frames have different contact counts. Differences exceed1mm at6.06s. Geometry,
masses, armature, normal contact parameters, source commands and duration pass
paired invariants.32 substeps remains running at this snapshot.

A completion at one resolution is not numerical convergence. The next decision
must use successive-refinement behavior, physical image comparison and both
unchanged task predicates, rather than choosing the setting with a passing score.
The original CPU material panel and training banks remain unchanged.

## Three-case result; refinement continues

All8/16/32 cases are terminal.32 substeps completes both task criteria and ends
65.99mm from the goal. However8 versus32 final can positions differ22.53mm, and
16 versus32 differ202.08mm. Goal movement at8/16/32 is9.863/52.964/0.302mm.
The pass/fail/pass pattern does not establish convergence. A declared64-substep
follow-up is running in`../integration_resolution_finer`, using identical physical
settings. The v2 runner only extends the accepted CLI substep choices.

The32-substep seating overlay was visually inspected with the same existing
camera hypotheses and real rim annotations. Cap-relative14/16/22/25s errors are
18.20/19.08/29.70/27.60px. Carry alignment remains similar, and the22s difference
is smaller than the8-substep soft reference40.42px, but substantial mismatch and
camera uncertainty remain. Better selected images and low goal motion alone do
not prove global e2e recovery or justify selecting a nonconverged resolution.
