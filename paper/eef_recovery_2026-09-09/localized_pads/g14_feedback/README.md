# Verified controller-feedback delay in the Genesis 1.4 port

Two completed951-step hand-only schedules differ only in whether forward
kinematics is refreshed before the external spring controller reads joint angles.
Both use zero gravity, disabled collision, the same aligned hand URDF, armature,
damping, spring law, limits and time step. Actual commanded torque is checked
against the engine's control field, and indexed getters against the full array.

| Feedback | Peak velocity | Peak target error | Largest feedback–integrated-angle mismatch |
|---|---:|---:|---:|
| Cached getter | 59.703 rad/s | 15.408 degrees | .074629 rad |
| Refreshed getter | 4.047 rad/s | 2.156 degrees | exactly0 |

The refreshed physical joint trace agrees with the archived pinned aligned-hand
bench within2.96e-5 rad over the complete schedule. This is a controller port
correction, not a fitted spring or friction improvement. Both input URDF and
preset hashes match across conditions. See `comparison.json`, terminal
`execution.json`, and each run's every-substep trace (cached angle, integrated
angle, angle used, velocity and commanded torque).

Installed1.4 source explains the failure: MuJoCo-compatible `RigidSolver.substep`
leaves `_is_forward_pos_updated=False` after integration; `get_dofs_position`
reads `dyn_state.dofs.pos` without refreshing it, while integration advances
`rigid_info.qpos`. `update_forward_pos` populates the angle cache from qpos.
A spring callback before the next engine substep therefore uses stale feedback
unless it explicitly refreshes. The isolated intervention eliminates the lag
and oscillation. This excludes robot/can collision and gravity as necessary
causes of this particular oscillation.

The full wrapper now refreshes before every spring read and checks it against
integrated qpos; it also refreshes after each scene step so saved poses and task
predicates use the current configuration. Earlier full1.4 failures remain
archived but cannot establish how the correctly controlled candidate performs.
`../g14_feedback_full/` holds the new full-task comparison; this bench alone does
not establish end-to-end recovery or calibrated hardware mechanics.
