# Raw measurement timestamp diagnostic

Completed: four collections, no new metric successes. Trial 233 remains successful
and passes exact independent saved-action replay. All four pass released/pushed;
235, 124 and 185 fail arrival. Final distances are 65.863, 90.552, 128.540 and
110.675 mm respectively. Trial 235 is also physically tipped at about 90 degrees.
The window-mean 10 ms controls likewise have only one success. This pilot does
not support adopting raw receipt-time interpolation as a recovery improvement.
Exact results are in `../slide_metric_of_record/raw_measurement_clock/results.json`.

The existing timed extraction assigns a mean of each joint/grip window to the
window's closing message time. The 10 ms experiment still interpolates those
window means. This paired follow-up removes that averaging and time assignment:
it separately interpolates every joint-state position and motor-feedback position
at its own bag receipt timestamp.

Trials 233, 235, 124 and 185 are the same four selected before the finer-interval
experiment. Initial positions, full world, day-specific yaw, source start/end
times, 10 ms EEF execution and original motor scale stay fixed. There is no og4,
added push, terminal hold, force adjustment or pose fitting. Joint and grip
timestamp reconstruction change together, so a difference cannot be attributed
to one signal alone.

Raw measurements are about 40 Hz; interpolating at 100 Hz creates no additional
sensor information. Compared with the 10 ms window-mean targets, maximum joint
differences are 0.024–0.027 rad and maximum motor-position differences 1.27–2.12
units on the original 0–100 scale. Counts, differences and clamped endpoint
samples are retained in `plan.json`; exact raw arrays and source tapes are local.
Bag receipt time is not necessarily hardware acquisition time, and measured
positions remain distinct from original hardware commands.

`run.py` reuses the isolated finer-interval execution modules. Results use the
unchanged supplied `slide_predicate.py` through the disclosed 120 ms schema
adapter; the stricter contact diagnostic is secondary. Metric passes receive a
fresh replay from saved EEF actions, with exact trajectory/action/observation
comparison. No result is automatically admitted to a training bank.
