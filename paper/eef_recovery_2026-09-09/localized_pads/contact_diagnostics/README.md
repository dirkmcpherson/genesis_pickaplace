# Contact diagnostic: 113 approach failure

The read-only observer completed all 4,187 source actions. Every saved array is
exactly identical to the critical-return soft reference; its 100,496 constraint
callbacks match the transmission callbacks. No physics parameters changed.

Only 14 sampled finger–can contacts occur, all on distal links: twelve at
5.71–5.89 s, then two at 6.73 and 6.82 s. The first right-tip contacts have normal
forces 11.58 and 3.48 N, utilization .53 and .13, and sampled forces pushing the
can in positive x and negative y. Left-tip contacts follow. The can is displaced
before any sustained grasp; no sampled finger contact remains after 6.82 s.
This corrects an intermediate commentary estimate ending the interval at 6.2 s.

Of the twelve main-approach samples, two are near the friction-pyramid limit
(utilization >= .95), with median utilization .55. All have pre-integration
relative tangent speed above 1 mm/s. This weakens a simple insufficient-friction
explanation but does not falsify a localized friction treatment.

The observer samples one of eight substeps, after constraint resolution and
before force integration. It may miss rapid transients and does not measure
steady-state friction or hardware coefficients. The 0.05 N force cutoff and
utilization/speed summaries are diagnostic conventions, not changed task metrics.
Rows are dependent contact samples, not independent trials. No extrapolated
impulse estimate is used.

`113_approach_contact_dynamics.png` and `.svg` show the observed approach.
`readout.py` verifies exact trajectory identity before summarizing the data.
The first-contact geometry, approach pose and actual unloaded finger posture
need attention before assuming the main failure is a pad-friction limit.
