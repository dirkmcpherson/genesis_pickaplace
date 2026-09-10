# Engine contact review: promising bench result, no full-task result

The corrected soft hand captures 113 but loses it during carry. An exact
full-source replay, observed at every physics substep during 7–16 s, finds
several mm/s of contact motion far below the friction limit. Real video shows
the can remains held. See `../aligned_contact_diagnostics/README.md`.

## Why revisit the pin

`cluster/patches/GENESIS_PIN.md` preserves upstream31951c3f plus rendering patches.
The substantive earlier gate is `CAN_STARTING_POSITION.md:391–418`: Genesis1.2.1
was rejected after twelve configurations because of weaker pinch forces and
extrusion. That negative result still matters. The old gate's .0025 s effective
contact-time assertion conflicts with METHODS CHECK6, which identifies .02 s
per-geometry parameters in the pinned tree. New comparisons read back actual
normal parameters rather than trusting either comment.

Upstream later added [elliptic friction and impedance-ratio controls](https://github.com/Genesis-Embodied-AI/genesis-world/pull/3028).
The [armature override fix](https://github.com/Genesis-Embodied-AI/genesis-world/pull/3072)
resolved issue3051; that issue must not be cited as proof of a remaining manifold
bug. These developments justify a new isolated test, not an automatic upgrade.

## Completed benchmark

Four scenes each contain eight spatially separated fixed-pad/can sets: original
and softer normal response, two initial overlaps and centered/off-center grasp.
All 32 case records completed. The input mesh hashes and assigned normal contact
parameters match exactly across conditions. Free-can armatures are zero. The
new engine uses analytic cylinder inertia/mass (0.345541 kg); the old engine uses
mesh-derived mass (0.343325 kg), a 0.645% difference. The same-engine comparisons
therefore provide the cleaner friction-formulation contrast.

For soft pads (geom timeconst .03 s, can .02 s), after two seconds:

| Initial overlap / offset | Pinned displacement, mm | 1.4 pyramidal / ratio1 | 1.4 elliptic / ratio1 | 1.4 elliptic / ratio10 |
|---|---:|---:|---:|---:|
| .5 mm / centered | 6.957 | 3.897 | 2.036 | .236 |
| 2 mm / centered | 5.072 | 2.635 | 1.340 | .169 |
| .5 mm / 30 mm offset | 43.997 | 29.679 | 20.501 | 3.728 |
| 2 mm / 30 mm offset | 34.704 | 24.201 | 15.067 | 2.653 |

At .5 mm overlap and 30 mm offset, droop is 72.00 degrees pinned, 50.97 newer
pyramidal, 35.52 elliptic/ratio1 and 6.51 elliptic/ratio10. The complete rigid and
soft controls, force histories and geometry hashes are in `summary.json` and
per-condition reports. The inspected `soft_contact_comparison.png` and `.svg`
show all four soft cases. Assigned normal parameters remain fixed, but actual
forces can change with contact formulation and evolving contact geometry;
do not claim identical realized normal forces throughout.

Four earlier new-engine attempts stopped before physics stepping: two used an
old parameter API; two used a legacy-mesh-specific mass check that excluded the
new analytic mass. Both fixes are documented, with archived executed scripts and
terminal failures. The mass check now validates against declared dimensions and
density within1%; no mass or physics parameter was changed to pass it.

## Scope and next gate

This is a fixed-pad diagnostic. It has no moving robot, no source demonstration
and no localized layered material law. It establishes neither improved task
recovery nor calibrated rubber properties. The elliptic ratio10 condition is a
candidate for a full-world comparison with the existing soft hand, preserving
source commands, yaw, placements, release/push scoring and goal-motion checks.
The old global-normal-impedance intervention remains rejected (confound49).

Genesis1.4.0 and CPU torch2.8 are installed only in `/tmp/genesis-contact-1.4`.
`environment.json` and package metadata record the setup. The pinned recovery
runtime, canonical URDF and training banks remain unchanged. Full-world API
adaptation, realized mechanics checks and paired full replays are outstanding.
No engine or pad configuration has been adopted; the e2e goal remains unachieved.

Follow-up: `../g14_full_replay/README.md` records four completed full-world233 controls. All fail pickup. Pre-grasp finger oscillation and severe soft-run goal displacement prevent transfer of this bench result to recovery; the new engine remains unadopted.
