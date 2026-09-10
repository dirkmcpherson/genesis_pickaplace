# Localized finger pads: feasibility assessment, 2026-09-09

Decision: a localized compliant contact surface is a physically motivated next
candidate. Detailed texture/ridge geometry is not yet justified. This assessment
does not implement a new pad model or report additional recovery experiments.

## Hardware evidence

Kinova's Gen3 lite User Guide, printed page 21, describes a structural hard but
flexible plastic and a soft rubber-like gripping plastic, with a wrist-mounted
linear actuator. This supports separating structural and gripping contact
properties. It does not provide pad thickness, friction coefficient, stiffness,
or damping; those are not identified by the existing image comparisons.

Source: [Kinova Gen3 lite User Guide, manufacturer document hosted by Clearpath](https://docs.clearpathrobotics.com/assets/files/clearpath_robotics_022586-TDS2-e371286ff4ca19bcebd345a43380ef0f.pdf#page=21).

## Local implementation evidence

- `gen3_lite_2f_robotiq_85.urdf` uses one collision mesh per proximal/distal
  finger link. The paths refer to Kinova `gen3_lite_2f` meshes despite the root
  filename. A trimesh connected-component inspection finds one component per
  finger STL (13,088 faces per distal mesh; 17,310/17,330 per proximal mesh).
  There is no ready-made disconnected pad to assign a different material.
- The installed engine's `genesis/utils/urdf.py` iterates all collision elements
  on a link and creates individual geometries. Separate pad/backing collision
  regions can therefore share an existing rigid link without adding articulated
  degrees of freedom. Collision geometry count and contact work would increase;
  runtime cost has not been measured.
- `genesis/engine/solvers/rigid/collider_decomp.py::_func_add_contact` averages
  the two geometry solver parameters and takes the maximum of their effective
  friction coefficients (with a 0.01 floor). A pad friction reduction can be
  masked by the other object's coefficient. Record actual effective pair values,
  not only requested material values. The URDF loader also defaults collision
  friction; any treatment requires explicit post-build assignment/readback.
- The existing softness runner assumes exactly four finger collision geometries.
  A pad implementation must update indexing, treatment assignment and contact
  diagnostics to distinguish pad contacts from structural contacts.

## What this could and could not fix

The completed two-trial probe supports an image-space seating improvement in
233 and a release regression, with no carry rescue in 113. It does not identify
excessive friction, pad absence, or the return mechanism as the cause of release
failure. Local compliance could preserve a firmer backing and avoid softening
outer finger contacts, but it still acts during release. Improvement is a
hypothesis, not an established consequence of localization.

The earlier 9–10 mm contact overlaps cannot be adopted as measured pad thickness
or compression. A soft surface with rigid backing is a contact approximation,
not a continuum material model. Its force/displacement response needs checking
before treating it as physical compression.

## Concrete next comparison

1. Identify pad boundaries from the real gripper views and CAD. Preserve the
   current unloaded outer contact envelope: replace the relevant region rather
   than adding thickness over an intact collision mesh. Keep inertial properties
   and joints unchanged. Disclose uncertain boundaries and backing depth.
2. Compare the original geometry against the divided geometry with identical
   contact properties first. Mesh division can change contact manifolds even
   when the outer surface is the same; quantify that difference separately.
3. Compare that divided-geometry control against pad-only compliance, initially
   keeping friction and transmission unchanged. Do not combine a friction
   increase, softer contact and stronger motor in the same treatment.
4. Check indentation/force response and unloaded opening, then real-video grasp
   seating, can slip during carry, and release timing. Only then assess final
   supported slide and the unchanged metric. Keep recorded actions, clocks,
   initial conditions and corrected early-day yaw fixed; no phase switches or
   goal-dependent forces.
5. Use the existing calibration set 113/184/233 and, after freezing the candidate,
   the reserved physical-reference validation set 176/185/237. All six outcomes
   have previously been seen; these are held out from new fitting, not pristine
   unseen trials. Report every attempted trial and both gains and regressions.

This route adds a small number of material regions rather than modeled ridges
or a deformable whole can. Its justification is the hardware's material layout;
the current results do not yet justify a claim that it improves recovery.
