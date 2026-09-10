# Finite return-spring bracket: no complete recovery

All six declared runs completed with exact source-action identity. None
completed the full physical sequence or passed the supplied metric.

| Return stiffness | Pad time constant | Strict failure | Final center distance |
|---|---:|---|---:|
| 4 | 0.020 s | Not picked | 258.57 mm |
| 4 | 0.030 s | No upright supported release | 106.50 mm |
| 4 | 0.040 s | No supported push to contact | 154.27 mm |
| 8 | 0.020 s | No upright supported release | 243.82 mm |
| 8 | 0.030 s | No upright supported release | 107.05 mm |
| 8 | 0.040 s | No upright supported release | 92.96 mm |

Increasing the return spring alone is not a demonstrated remedy. These
experiments retain the original adaptive actuator scale and damping, which were
uncalibrated starting values. They do not isolate the effects of freeing the
tips relative to a dynamically equivalent fixed-coupling hand.

## Next mechanical control, derived from existing gains

The reference replay sets all four finger position gains to 40 and velocity
gains to 10. Along its coupled coordinate theta, the joint derivative is
`j = [-1, 1, -0.676, -0.676]`. Therefore the reference modal stiffness is
`sum(40*j**2) = 116.55808`, and modal damping is `sum(10*j**2) = 29.13952`.

The adaptive actuator coordinate has gradient `a = [-0.5, 0.5, 0.1, 0.1]`.
Its original actuator stiffness 80 gives modal stiffness
`80*(a.dot(j))**2 = 59.8303232`, about half the reference. Original passive
damping gives modal damping `2*1 + 2*0.02*0.676**2 = 2.01827904`.
This discrepancy is a control-model confound, not a measurement of the real
actuator's force or damping.

The next declared nine-run bracket (`matched_drive_plan.json`) matches these
two modal coefficients using actuator stiffness 155.85151310030028 and proximal
damping 14.56062048, keeping distal damping and force limits unchanged. It tests
return stiffness 2/4/8, each with contact time constant 0.020/0.030/0.040 s.
The runner reads and checks the original gains before installing the candidate.

This is matching in the unloaded coupled direction below saturation, not full
dynamic equivalence: finite passive articulation and implicit damping still
differ. Both stiffness and damping change as one declared reference-drive
control; do not attribute any result to only one of them. A soft-pad benefit
must be assessed against the original-contact case at the same drive and spring
settings, and subsequently against real-video and held-out-from-fitting trials.
