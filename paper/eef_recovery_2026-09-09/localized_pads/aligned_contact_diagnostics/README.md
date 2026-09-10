# Aligned-hand carry diagnostic

The full 113 soft replay completed and reproduced every saved array of the
inertia-aligned reference exactly. All 100,496 constraint callbacks match the
transmission callbacks. Unlike the earlier approach diagnostic, this observer
samples every physics substep during 7–16 s.

The real camera shows a held can at 8, 10, 12, 14 and 18 s. The simulated can
creeps through the fingers and falls before 12 s. At 8–9, 9–10 and 10–11 s,
median tangent speeds are 3.72, 6.54 and 7.73 mm/s while median friction-pyramid
utilization is .295, .304 and .237. Fewer than .5% of loaded contact samples in
each interval reach .95 utilization. Normal forces remain several newtons.
This supports regularized contact drift as a contributor, rather than a simple
insufficient-friction-capacity explanation. It is not a measurement of rubber
friction or proof that an engine change will recover the task.

In 184, the real can remains in the fingers through 33 s and is visibly released
by 36 s. The aligned soft simulation has already lost hand contact by 28 s.
The real frames and contemporaneous sim height/contact labels are saved in
`184_real_carry_release.jpg`; these observations do not independently establish
exact release timing between frames.

`summary.json` retains all diagnostic intervals, including the final loss. Force
cutoff .05 N excludes unstable ratios near zero and does not change task scores.
Contact rows are dependent samples. Velocities precede current force integration.
The next comparison concerns separating friction regularization from normal
softness, not increasing the physical friction coefficient without evidence.
