# Genesis 1.4 initial full-world check: stale-feedback port fails pickup

**Follow-up:** `../g14_feedback/README.md` isolates a stale joint-position getter
as the source of the hand oscillation. Refreshing kinematics before the external
spring callback restores the pinned hand bench with unchanged physical
parameters. The failures below characterize the delayed-feedback port; they do
not reject the correctly controlled candidate. New results are in
`../g14_feedback_full/`.

The four declared trial233 controls completed all962 original decisions, with
no added motions or holds. All fail pickup and both unchanged slide definitions.
The pinned aligned rigid hand completes this demo. These full-world results
reject adopting the tested port; the fixed-pad contact-bench improvement has
not transferred to recovery.

| Contact setting | Pad time constant | Final can–goal distance | Final goal displacement | Strict result |
|---|---:|---:|---:|---|
| Pyramidal, ratio1 | .02 s | 388.17 mm | .003 mm | not_picked |
| Pyramidal, ratio1 | .03 s | 3103.69 mm | 7014.85 mm | not_picked |
| Elliptic, ratio10 | .02 s | 406.81 mm | .010 mm | not_picked |
| Elliptic, ratio10 | .03 s | 8817.86 mm | 4835.99 mm | not_picked |

The soft runs move the goal by metres. Their final distances are therefore
measurements of pathological failed scenes, not near-misses at the final slide.
The supplied release predicate fires before pickup; its pushed and arrived
clauses fail. This is why strict task sequencing remains necessary alongside it.

## What was verified

`readout.py` checks the completed execution records, wrapper source hashes,
loaded candidate URDF hash, all962 commands, callback counts and pad activity.
Tool targets differ from the pinned control by at most5.6e-16 in matrix entries,
joint commands by7.8e-11 rad; recorded grip and mount are exact. EEF action arrays
differ by at most8.95e-8 because the initial measured reset pose differs slightly.
All subsequent target poses are preserved. Each run has23,096 torque callbacks
and2,887 scene-step observations. Both soft runs actually modify classified pad
contacts; both rigid runs leave contact time constants unchanged.

Runtime mechanics checks cover arm/finger armature, passive limits, remaining
proximal equality, finger masses/COMs, link-frame inertia tensors, zero finger
PD gains and declared damping. Normal geometry parameters are explicitly set
and read back. The material classification is in the finger link frame, applied
to every contacting object, with the existing layered law. No friction
coefficient, geometry input, source placement or yaw was fitted to these results.

Two port issues were caught and retained:

1. `233_pyramid_rigid_port1` stopped before stepping: the new engine stores
   principal inertia and its quaternion separately. Comparing the tensor after
   rotation to the authored link frame passes; no inertia was changed to pass.
2. `233_pyramid_rigid_port2` completed, but its classifier incorrectly treated
   recentered mesh coordinates as finger coordinates. The soft run was deliberately
   interrupted and the remaining conditions were not started. `frame_audit_port3`
   verified the corrected transform. Processed proximal support-plane offsets
   still differ from the pinned engine by +3.8/-10.4 micrometres; distal offsets
   agree to sub-nanometre precision. Input equality is not processed-mesh equality.
   The corrected rigid replay exactly reproduces every port2 saved array while
   detecting1,119 material-region contacts, verifying the classification fix is
   a physics no-op in the rigid control.

`plan_port3.json` declares all four corrected conditions before execution;
`execution_port3.json` records terminal exit0 for all. The interrupted port2
comparison is not part of the four scored conditions. Archived scripts preserve
both preceding implementations.

## Remaining compatibility issue

`233_hand_and_carry.png` and `.svg` show pronounced oscillation in the new hand
before grasp. In the .3–3 s approach window, sampled can–hand contact counts are
zero, yet the maximum finger deviation from its unloaded relation is15.14° in
the new pyramidal setting and14.56° elliptic, versus.049° pinned. Rigid and soft
settings have identical approach statistics within each cone setting. Full-run
peak finger velocities reach67–84 rad/s. This is not a collision-disabled bench:
robot self-contact and other world forces have not been excluded. Matching
parameter readback does not establish matching integrated hand dynamics.

Next gate: diagnose that oscillation in an isolated hand schedule, checking
actual torque application, integration/damping and self-contact before another
recovery sweep. It would be premature to attribute these failures solely to the
friction formulation, softness or measured hardware properties. The new engine
also uses analytic can mass0.345541 kg versus pinned mesh-derived0.343325 kg and
reports existing arm COM/mesh inconsistencies; these cross-engine differences
remain disclosed. No canonical URDF, pinned environment or training bank changed.
