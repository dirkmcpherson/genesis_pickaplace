# Candidate finger transmission mechanism

Kinova patent [US11584022B2](https://patents.google.com/patent/US11584022B2/en),
Figures 1–9 and the detailed description of the grasp sequence, documents a slider
acting through a skeleton member and flexible hinges. When an object blocks proximal
rotation, continued slider movement can rotate the distal segment around the object.
This is consistent with the user's observation. The inspected drawings show both
shallow pinch and enveloping configurations. The patent does not establish that the
study's exact Gen3 Lite revision implements this embodiment, nor supply calibrated
parameters for our robot. Original PDF and rendered Figure 9 are retained here.

## Consequence for the current tests

The existing compliance surrogate removes distal PD and softens its coupling, but
continues driving the proximal joint. It does not represent actuator transmission
through the outer member. Freeing a distal hinge therefore does not test the full
candidate mechanism: available motion and the route by which actuator force reaches
that motion are different requirements.

The declared 16-trial surrogate diagnostic remains useful for measuring sensitivity,
but cannot establish or disprove this transmission model. Keep its parameters and
selection unchanged; do not interpret a negative result as evidence against the
user's physical observation. No new mechanism is adopted based on patent resemblance.

A grounded implementation requires checking vendor geometry for the outer member
and hinge locations, modeling the slider/transmission and flexure return, and checking
unloaded and object-blocked motion before replaying task recordings. Force, stiffness,
slider stroke and the exact hardware correspondence remain unresolved.

## Vendor asset inspection

The two right-finger STL files each contain one connected mesh. Their projections
and the two URDF joint locations are shown in vendor_mesh_projection.png; this is
an actual geometry projection, not a reconstructed physical mechanism. The URDF
inventory in urdf_topology.json has proximal/distal rotational joints and fixed
mimic relations, with no independently represented slider or outer-member joint.
The meshes alone therefore do not supply an articulated transmission assembly.
Mesh appearance is insufficient to establish the patent-to-product correspondence.

## Direction check against the actual URDF

Exact joint transforms and a distal-end point from the vendor mesh show that a
positive right-tip residual moves the end outward at fixed proximal angles 0.3,
0.5 and 0.7 rad. At 0.5 rad, +5 degrees moves it about 3.73 mm farther from the
center plane; -5 degrees moves it about 3.77 mm inward. See distal_direction_check.json.

Applying this point calculation to the measured 0.3 s surrogate traces separates
outward splay from inward curl (contact_direction_readout.json). This is a simulated
geometry diagnostic, not physical angle validation. The positive residual peaks
must not be described as evidence of the user's inward curling mechanism.
