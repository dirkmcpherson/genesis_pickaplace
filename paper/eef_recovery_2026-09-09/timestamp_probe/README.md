# Timestamp-based EEF playback probe

This isolated development test reconstructs the measured arm and grip paths on their
bag timeline. It does not apply og4, add slide motion, alter physical parameters, or
change frozen tapes. Trial 234 uses the previously inspected upright-start correction;
233 and 235 retain archived initial poses.

The bag extraction matches every original joint and grip sample exactly for all three
trials. Linear interpolation samples both streams at 30 ms action-start intervals.
The final target is clamped to the original endpoint, giving 35–38 ms endpoint overrun.
These are timestamped window means, not hardware command messages; this reconstruction
reduces a known timing error but does not establish exact real-world playback.

| Trial | Previous duration | Bag sample span | Reconstructed duration | Outcome |
|---|---:|---:|---:|---|
| 233 | 27.78 s | 28.82 s | 28.86 s | Complete |
| 234 | 27.00 s | 29.63 s | 29.67 s | Picks, tips on shelf; no upright release |
| 235 | 26.52 s | 29.79 s | 29.82 s | Picks, tips on shelf; no upright release |

All three runs finished successfully as processes. Both failed placements end near
90 degrees tilt. Correcting timing alone therefore does not rescue these two cases;
the successful control remains complete. This is a selected three-trial experiment,
not a corpus-wide estimate. No new episode enters a demonstration bank.

`plan.json`, `run.py`, `timing_validation.json` and `summary.json` retain setup,
provenance and outcomes. No simulated video inspection has been performed in this pass.

Control 233 also passed fresh saved-action replay: trajectory, EEF actions and
observations are bit-identical. Timing provenance survives the replay sidecar.
