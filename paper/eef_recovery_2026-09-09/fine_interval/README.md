# 10 ms target-cadence diagnostic

Completed: all four collections finished. Only the already-successful control 233
passes the supplied metric of record; its independent saved-action replay is
bit-identical. Trials 235, 124 and 185 still fail arrival. Trial 124 regresses from
upright release to a tipped final can. `paired_metric_readout.json` retains final
distances and all metric conditions against the 30 ms controls. This sample gives
no evidence to adopt the finer interval as a recovery improvement.

The source/trace endpoint-overrun field inherited from 30 ms controls was corrected
where present; `endpoint_metadata_correction.json` records the changes. This was a
metadata-only correction and did not change the commands or replay duration.

This experiment tests whether 30 ms resampling discards motion that matters for
contact. It uses a 10 ms target interval with the existing 10 ms physics timestep,
instead of three physics steps per target. Physics parameters, day-specific yaw,
initial positions and recorded gripper measurements are inherited from each
matched 30 ms control. No og4, added push or terminal hold is used.

Four outcome-selected diagnostics are declared before running: December 18 trial
233 (complete control), 235 (placement failure), December 16 trial 124 (no slide
recontact), and December 17 trial 185 (near-goal unsuccessful push). These are not
a representative success-rate sample.

The experiment holds pickup duration at 300 ms and the development scorer's
release and goal-contact windows at 90 ms. The legacy slide counter also retains
its physical duration. The eight adversarial scorer tests pass at the finer
cadence; two additional tests reject 30 ms release/contact transients. A successful
trace is independently replayed from saved EEF actions and compared exactly for
trajectory, actions and observations. Visual review remains required.

`fine_env.py`, `fine_sequence.py`, and `fine_replay.py` are isolated source copies;
their exact differences and hashes are retained beside them. Existing production
and active census modules are unchanged. These copies support this recorded-path
experiment only, not training, added-motion repair or terminal-settle evaluation.
The collector's effectively unlimited episode horizon avoids the latter path.
The standard 30 ms renderer and bank loader must not be used without adapting
their cadence explicitly.

Recording measurements as position targets still does not recover the original
hardware command stream. Finer interpolation is not hardware calibration and may
improve or worsen recovery. Initial-position provenance and timing reconstruction
remain disclosed in each source and collected trace.
