# Twofold pad contact damping at fixed elastic coefficient

The frozen experiment tests gain2 at the existing inner-pad contacts. The
engine computes b=2/(dmax*tc) and k=1/(dmax²*tc²*dampratio²). After the existing
layered softness law, tc is divided by2 and dampratio multiplied by2. Thus k
and impedance at fixed penetration stay unchanged, while b doubles. All objects
contacting those material regions receive the treatment; it does not key on the
can, task phase, release time or success label.

Genesis uses the same reference damping for the normal and tangent contact rows.
This is explicitly a coupled contact-damping experiment. It does not isolate
normal damping alone, change the Coulomb coefficient, or represent measured
rubber properties. Matching rigid-pad gain2 controls distinguish a softness
benefit from a general damping change. Hand mechanics, source movements, timestamps,
yaw, geometry, shelf, goal and scoring stay fixed.

The selected original-placement calibration trio is113/184/233. An additional
113 pair uses the previously declared conditional image-derived initial XY solely
as a diagnostic. Neither that pose nor the candidate is adopted. Two gain1
identity controls must reproduce every reference NPZ array exactly.

The wrapper checks the installed gu.imp_aref function at every treated contact:
the zero-velocity normal response is preserved; normal and tangent velocity terms
multiply by the declared gain. It also checks the time constant against the
2*substep_dt safety floor. These checks do not prove full dynamics are equivalent
or that the pad response is physically calibrated. Full source replays and both
success definitions remain the outcome tests.

The first attempt uses an unsupported local alias for the engine ContactData
field inside a compiled kernel. Its eight treatment jobs stop before stepping;
these are implementation failures, not failed full demonstrations. Both identity
controls bypass that kernel. Their logs and sources remain in this directory.
`corrected/` preserves the same frozen design with only the field-reference fix.
No treatment parameter was changed in response to an outcome. The corrected
batch includes its own two identity controls for exact reproduction.

`readout.py` retains pending, execution-failed and completed results separately,
checks source/metadata/code/URDF hashes, original source commands and fresh
feedback, and records goal movement alongside full-task outcomes. No incomplete
run is counted as a failed demonstration or a completed full replay.

## Terminal result

All jobs are terminal. The corrected ten-run batch and the first attempt's two
identity controls add12 completed full replays. All four identity controls match
every reference array. Doubled damping yields0/3 original-placement calibration
passes for both rigid and soft under both task criteria, and loses the undamped
soft233 completion. Both conditional113 treatments also fail supported release.
See `corrected/README.md` and `corrected/summary.json` for the retained outcomes,
including large goal displacement. Candidate rejected; normal-only damping is
a distinct untested hypothesis (`normal_only_capability.json`).
