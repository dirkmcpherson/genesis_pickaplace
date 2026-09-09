# Broader distal-compliance diagnostic

Completed: 16/16 processes exited successfully. Three complete controls remain
complete; none of the thirteen incomplete timestamp controls becomes complete.
Final outcomes: 3 complete, 4 not picked, 5 without supported release, and 4 without
a supported push to contact. Trials 248 and 256 regress from slide failure to
release failure; 259 and 286 improve from release failure to slide failure.
`summary.json` retains every paired outcome. The three numerical completions are
controls, not new bank admissions or independently verified treatment successes.
This parameter should not be adopted as a recovery improvement from these results.

A declared 16-trial sample covers all three fixed-clock pickup failures, five
placement failures, five slide failures and three complete controls. Selection is
by deterministic hash rank within strata, not by inspecting treatment outcomes.
The reference strata use upright-corrected controls for 234/318. Exact UIDs and
selection rules are in plan.json.

The batch waits for all 74 timestamp controls, then uses those same source tapes
and initial poses. The only treatment is the existing 0.3 s distal equality
surrogate with distal PD gains zero. Original proximal control, contact geometry,
physical limits and measured grip remain. No og4 or extra push is permitted.

This expands diagnostic coverage beyond the initial two trials. The surrogate is
still uncalibrated and is not automatically admissible as a real hand model even
if task recovery improves. Failures and successful-control regressions are retained.
No treatment outcome has been used to choose this sample or its parameter.

Run state is in runner.log and per-trial execution files. The runner has exited
with code zero; all declared treatments are accounted for.

A subsequently found Kinova patent suggests a different actuator transmission
architecture; see ../finger_transmission/README.md. This diagnostic does not model
that architecture and cannot reject it. The registered treatment is unchanged.
