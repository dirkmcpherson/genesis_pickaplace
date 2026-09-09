# Early-day contact-timeconstant diagnostic

Ten trials were declared before treatment results: eight valid-initialization
pre-support tipping failures with stable recorded grip, plus the two early-day
physical slide controls (176 and 225). Trial 113 is the mechanistic index case;
the remaining failures are hash-ranked within day. See plan.json for full IDs.

Only the existing grasp_timeconst=.005 intervention is applied, on four finger
geoms and the manipulated can. This also changes can–world effective contact
parameters through Genesis pair averaging. Each fresh worker asserts all five
readbacks and the solver stability floor, alongside the full-world yaw/gain/shelf
checks. Motion, grip, clock, initial positions, impedance, shelf and goal are held
to the matching baseline reconstruction. No og4 or added motion is involved.

The static 113 diagnostic motivates testing reduced slip, but does not validate
this setting. Earlier December 18 work found mixed outcomes and higher grip
forces. A passing trial here is not sufficient for adoption as realistic physics,
and these files must not be silently mixed into existing demo banks.

run.py executes all declared cases and retains failures. summarize.py reports
paired metric components, distances, tilt and strict contact diagnostic, including
lost controls. A new candidate would still need independent saved-EEF replay with
the same local variant registration, visual review, and physical qualifications.

## Final verdict: not adopted

All ten declared trials finished with successful process exits and contact/world
readbacks. No new supplied-metric pass or physical-sequence completion appeared.
Both successful controls, 176 and 225, lost both criteria. Thus completion falls
from 2/10 to 0/10 under either criterion on this deliberately selected sample.
Some failed grasps retain cans longer or finish upright, but those local gains
do not produce complete recoveries. No files from this experiment enter a demo
bank, and no independent success verification is needed because there are none.
The reusable verifier remains available for reproducibility checks.

This repeats the broader warning from earlier December 18 experiments: correcting
a contact artifact alone can damage the recorded manipulation sequence. It does
not establish a calibrated compliant hand. The baseline world is unchanged.
See summary.json for every paired result and SHELF_EVIDENCE.md for the separate
conditional camera-height estimate; it does not justify a geometry sweep.
