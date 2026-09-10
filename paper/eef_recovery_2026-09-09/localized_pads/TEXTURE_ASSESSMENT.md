# Texture assessment

Physical surface roughness remains a plausible contact-model ingredient. It does
not require explicitly meshing microscopic bumps. A visual texture or rendering
roughness map, however, does not change rigid contact in our pinned engine.

The installed Genesis source separates `options/surfaces.py` texture properties
from `engine/materials/rigid.py` physical friction. In
`engine/solvers/rigid/collider_decomp.py:1044–1053`, contact friction is the maximum
of the two geometry friction values after their runtime ratios are applied,
floored at 0.01. Changing one material below the other material's value therefore
does not change the pair's friction. Our existing `SurfacePads` modifies normal
contact response only; it deliberately leaves friction unchanged.

The public [surfaces documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/rendering/surfaces_textures.html)
also distinguishes visual appearance from physical material properties. Current
[rigid-material documentation](https://genesis-world.readthedocs.io/en/latest/api_reference/engine/material/rigid.html)
includes torsional and rolling friction, but those options are absent from our
pinned material constructor. Current documentation is not evidence that the
installed 0.2.1 engine supports them.

## Evidence and decision

- Explicit finger subdivision already changed the contact manifold and damaged
  recovery even in its rigid control (`INITIAL_READOUT.md`, confound 62).
  Detailed collision texture would need its own geometry control.
- The historical `can_pos_recovery/friction_sweep.log` reports increasing can
  friction from 1 to 2 reduced picks from 13/16 to 10/16 and contact from 5/16
  to 2/16. This was a different model and a whole-can treatment; it neither
  validates nor rules out localized pad friction in the current adaptive hand.
- Confound 49 documents contact creep below the friction limit and the failed
  full-demo stiffness interventions. Slip alone does not establish inadequate
  friction, and a fixed-pad bench alone does not establish a task improvement.
- The completed critical-return validation rejects the current soft candidate:
  original/matching-rigid/soft metric passes are 3/9, 2/9, 1/9. Reducing soft
  compression travel to 0.5 or 1 mm also fails to recover early calibration
  pickups (`compression_limit/README.md`). These are softness results, not a
  physical-texture ablation.

Prefer a localized effective-friction hypothesis on the existing pad regions
over adding collision bumps. Before choosing a coefficient, inspect tangential
motion and contact-force utilization at the earliest real/sim divergence; the
current saved maximum force norms cannot determine friction-cone saturation.
Separate approach ejection, loaded sliding, release sticking and final pushing.
Any future treatment must apply by pad material region to every contacting
object, preserve commands and both scoring definitions, and succeed through
release and final slide. A higher grip yield alone is insufficient.

No texture/friction treatment was implemented or adopted in this assessment.
No hardware roughness, friction coefficient or rubber compression has been
measured. The useful hypothesis is inexpensive effective contact behavior;
there is presently no evidence that detailed surface geometry would improve
end-to-end fidelity.

## Follow-up: corrected-hand carry diagnostic

The every-substep observation in `aligned_contact_diagnostics/README.md` now
addresses the force-utilization question above. In corrected-hand soft trial113,
the can creeps during carry while most loaded contacts remain well below the
pyramidal friction limit. This supports investigating contact regularization;
it does not establish that the real pad needs a larger friction coefficient.

The four completed conditions in `engine_contact_review/README.md` provide a
specific lead: elliptic friction with impedance ratio10 in isolated Genesis1.4
reduces drift in the soft fixed-pad bench. Full-demo validation remains necessary,
especially release and final sliding. This is a solver formulation experiment,
not a texture treatment or a measurement of real surface roughness.

Decision: retain localized effective roughness as a possible material hypothesis,
without adding collision bumps. Prioritize checking the existing creep lead
before selecting another friction coefficient. Visual textures may help video
appearance, but do not change collision physics (also stated explicitly in the
linked Genesis surfaces documentation, checked again during this assessment).

## Completed full-task follow-up

The corrected-feedback engine comparison now extends beyond the fixed-pad bench.
Elliptic friction improves 113 carry through 20 s under both rigid and soft pad
settings, but both fail descent/release (`g14_feedback_full/README.md`). This is
not a softness-specific gain. The frozen nine-demo pyramidal panel is also
complete: original / same-hand rigid / soft strict completions are 2/9, 1/9, 1/9,
with supplied-metric passes 3/9 for all three (`g14_validation/README.md`).

The current evidence therefore does not justify selecting a larger physical
friction coefficient or adding collision texture. Effective pad roughness remains
an inexpensive, untested material hypothesis. A useful future friction comparison
must report contact-force utilization and release as well as carry, since stronger
grip can change where the can seats and when it separates. A clock-matched 113
camera check now finds conditional evidence of a seating mismatch near pickup;
its three-dimensional placement estimate is too uncertain to adopt
(`g14_release_diagnostic/README.md`). Resolve that ambiguity before interpreting
another parameter sweep as improved physical fidelity.

## Release-force follow-up

Two exact archived-reference force replays identify a shelf-edge impulse after
finger separation in113 (`release_wrenches/README.md`). Three full comparisons
at a nominal image-only initial XY all fail e2e, despite improved grasp images
under both rigid and soft pads. This pose is conditional and remains unadopted.

A further exact rigid/soft observation pair at that conditional pose separates
the release sequence (`vision_initial_probe/release_wrenches/README.md`). During
20.10–20.16s, the soft fingers impart net downward contact impulse, predominantly
normal, while rigid retains net upward support. Soft reaches shelf contact with
a faster drop and receives a much larger backward shelf impulse. Rigid settles
but fails later task continuation. The force data therefore support investigating
normal compression and damping during opening before choosing more friction.
They do not prove which part of the uncalibrated hand/contact model is wrong.

No texture treatment has been tested. Effective local roughness remains a cheap
physical hypothesis; detailed collision bumps remain unsupported. A future
damping test must verify actual solver coefficients and preserve static normal
stiffness to separate damping from a new softness change. Current evidence still
does not establish an end-to-end soft-pad improvement.

## Completed damping comparison

The next declared damping experiment preserves the elastic coefficient while
doubling both normal and tangent velocity reference damping at pad contacts.
All eight treatment replays fail supported release: original113/184/233 each
with matching rigid and soft pads, plus the conditional113 pair. Both pad types
score0/3 on the original calibration trio under both task criteria; undamped
soft233 completion is lost. Four identity executions exactly reproduce their
references. See `contact_damping/corrected/README.md`.

This rejects the coupled damping setting. It does not test normal-only damping
or a physical friction/texture coefficient. No stronger-friction conclusion
follows from the result. Source inspection identifies a prospective isolated
normal-row intervention, with implementation and validation still required.

## Normal-only follow-up

The isolated normal-row intervention now leaves tangential reference damping
unchanged, verified at every physics substep. All10 calibration/conditional
replays are complete and both identity controls are exact. Soft233 preserves a
proximity pass with a supported40.9mm slide and0.468mm ending gap, but the strict
contact sequence fails; neither early original is recovered. The goal estimate
used233/242 contact constraints, so the submillimeter gap is not independent
real-world calibration. See `normal_damping/README.md` and its complete real/sim
review. No texture or friction-coefficient treatment has been tested. The broader
elliptic-contact effect will be measured with fixed settings on the existing
day-balanced panel before choosing another coefficient.
