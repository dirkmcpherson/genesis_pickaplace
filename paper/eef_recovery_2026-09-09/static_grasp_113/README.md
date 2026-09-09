# Trial 113 static-grasp diagnostic

The timestamp-matched real/sim review shows the real can retained while the
simulation drops it near 13.4 s under steady recorded grip. The simulated can's
tool-local y coordinate moves from about 30.6 mm at 8 s to 52.6 mm at 13 s before
rapid separation. This motivates a static-hold diagnostic, not an uncalibrated
friction increase or a success-oriented motion change.

The experiment reproduces 267 original EEF reconstruction steps (8.01 s), then
repeats the last recorded joint target and grip for 167 steps (5.01 s). The object
is never frozen, attached or teleported. Full Day 1 world and yaw are read back.
This intentionally modified command sequence must never enter a real-demo bank.

`analyze.py` requires a bit-identical original prefix, including the post-prefix
observation, then compares tool motion, tool-relative can drift, tilt and contacts
with the same-duration original continuation. A static failure would show that
continued arm motion is unnecessary for failure at this reproduced grasp; static
retention would implicate movement or changing gravity orientation, without
isolating the responsible physical parameter. Neither outcome calibrates passive
curl, contact compliance, friction or real initial placement.

The existing gripper-lab and CONFOUNDS contact-compliance results remain relevant;
this probe does not replace or independently rediscover those broader audits.

## Completed result

The original prefix is bit-identical in trajectory, EEF actions, observations,
finger joints and contact counts. Over the following 5.01 s, the can moves
22.16 mm relative to the tool with the recorded continuation and 21.72 mm with
the static target. Tool endpoint displacement is 185.60 mm versus 3.87 mm
(residual physical settling under the fixed target). Both retain sampled hand
contact and have no shelf contact throughout this window. Neither has tipped
by its endpoint; the window ends shortly before the original 13.38 s tip.

Thus continuing the large recorded arm motion is unnecessary for approximately
22 mm of in-grasp slip in this one reproduced grasp. This is evidence for static
grasp instability; it does not by itself establish the eventual static-hold drop,
nor identify friction, compliance, geometry or force calibration as the cause.
