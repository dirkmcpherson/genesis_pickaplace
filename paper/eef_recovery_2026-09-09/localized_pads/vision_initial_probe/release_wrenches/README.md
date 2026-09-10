# Conditional 113: soft opening accelerates the drop, then the shelf ejects it

Both observation replays are complete and reproduce every reference NPZ array
exactly. They use the already declared conditional image-derived initial XY;
no commands, parameters or world settings change. `../observe_release.py`
archives the observer and simulation sources. `../read_release.py` verifies
provenance, full-trace equality and force/momentum balance.

| Release observation | Rigid pads | Soft pads |
| --- | ---: | ---: |
| Last loaded finger contact | 20.1675 s | 20.1575 s |
| Downward can speed at 20.16 s | 0.315 m/s | 0.758 m/s |
| First loaded shelf contact | 20.225 s | 20.20375 s |
| Shelf x impulse, entire observed 19.8–20.6 s window | -0.0313 N s | -0.2458 N s |
| Supported upright release in full trace | Yes | No |
| Full-task completion | No | No |

The shelf impulses cover different contact durations: soft loses the shelf by
20.26125 s, whereas rigid remains supported through the observation window.
At 20.16–20.24 s, soft can x momentum changes by -0.16005 N s while shelf x
impulse is -0.16020 N s. No finger contact remains in that interval. Thus the
large backward velocity develops at shelf impact, independently corroborating
the archived-placement failure mechanism. `release_comparison.png` was inspected.

There is also a preceding difference during opening. At 20.10–20.16 s, soft
finger normal contact contributes -0.05824 N s vertically and tangential contact
-0.00057 N s. Rigid contributes -0.00667 N s normal and +0.10147 N s tangential.
Soft therefore imparts a net downward contact impulse while rigid still supplies
net upward support. This is not merely an earlier unsupported gravitational
drop. `../decompose_opening.py` reconstructs normal/tangent components using the
recorded geometry-side force convention and retains all intervals.

The soft pair's net contact work over that opening interval is +0.1411 J versus
-0.02494 J rigid. This is work from force on body b dotted with relative velocity
b-a; it is not actuator work, a measured pad energy, or proof of numerical energy
creation. Compliant contact can return previously stored energy. The current
observations do not distinguish geometric wedging, stored compression, hand
dynamics and solver regularization as the sole underlying cause.

Force/momentum agreement supports attribution: maximum absolute component
residual among the declared windows is below 0.00084 N s. Instantaneous
acceleration-error maxima are 0.363/1.168 m/s² for rigid/soft and remain disclosed.
The 0.05 N loaded-contact cutoff is diagnostic only and changes neither score.
Floor/shelf/pick-table geometry labels use the checked builder order0/1/2.

This makes the normal response during opening a better-supported next hypothesis
than arbitrarily increasing friction or adding collision texture. A damping
comparison would need to preserve the intended static normal stiffness and
verify actual solver coefficients; changing a time constant also changes other
response terms. It would still need full-task comparisons at the original
placements and broader validation. Neither a new pad candidate nor this initial
pose is adopted from the diagnostic.

The inherited all-soft preset contains an obsolete prose `spatial_hypothesis`
describing proximal-only treatment. Its authoritative `compliant_part="all"`
and runtime material audit activate all four fingers, as in the reference.
The hashed preset was retained unchanged for reproducibility; its prose does not
describe the executed spatial treatment.
