# Completed fresh-feedback validation: soft candidate rejected

All 18 full-source replays completed, covering nine frozen rigid/soft pairs.
The latest soft-pad setting adds no supplied-metric or strict-sequence successes
over the same-hand rigid control and loses the original fixed gripper's strict
completion on 237. No engine or pad configuration is adopted.

| Condition | Supplied slide predicate | Strict full sequence |
|---|---:|---:|
| Original fixed gripper | 3/9 | 2/9 |
| Genesis 1.4, matching rigid hand | 3/9 | 1/9 |
| Genesis 1.4, soft pads | 3/9 | 1/9 |

Each day contributes three selected recordings. December 16 strict completions
are 1/3 for every condition; December 17 is 0/3 for every condition; December 18
is 1/3 original and 0/3 for either new hand. These small selected strata are not
population recovery estimates. Prior model outcomes were known; new-engine
outcomes were not used to choose this panel.

Values below are final can-to-goal center distance / final goal displacement,
both in millimetres. Goal motion must accompany proximity claims.

| UID | Original fixed | Matching rigid | Soft |
|---|---:|---:|---:|
| 176 | 65.84 / 7.63 | 66.01 / 5.39 | 66.01 / 4.35 |
| 185 | 108.27 / 7.79 | 95.30 / 74.33 | 2576.71 / 2532.69 |
| 237 | 66.55 / 4.26 | 67.60 / 49.24 | 66.57 / 48.92 |
| 156 | 127.86 / 0.04 | 186.01 / 0.01 | 183.86 / 0.01 |
| 181 | 144.40 / 0.01 | 147.26 / 0.00 | 149.51 / 0.00 |
| 224 | 65.87 / 86.77 | 69.57 / 43.21 | 71.73 / 39.73 |
| 198 | 181.86 / 0.01 | 582.54 / 0.01 | 254.52 / 0.01 |
| 243 | 189.06 / 0.00 | 1140.13 / 0.02 | 328.32 / 0.00 |
| 246 | 845.35 / 0.00 | 93.30 / 0.10 | 1880.16 / 0.00 |

The candidate uses refreshed controller feedback, pyramidal friction, impedance
ratio 1, the aligned critical-return hand and pad time constants .02 s (rigid)
or .03 s (soft). The can retains .02 s. These are solver settings, not measured
rubber properties. The previous calibration-only 233 benefit did not transfer.

`plan.json` and `executed_sources/` preserve the declared inputs and code.
`execution.json` retains all 18 terminal zero-exit records. `readout.py` checks
archived code, realized URDF and preset hashes; full frame coverage; original
tool targets, arm commands, grip and yaw; absence of added motion; callback
counts and exactly refreshed feedback; and material activity. Neutral-pose
collisions remain disabled (`enable_neutral_collision=False`), and valid geometry pairs
are recorded and identical within each rigid/soft pair. Cross-yaw filtering and
other engine differences remain disclosed in the preceding port audit.

`summary.json` retains every failure reason, source-command difference and goal
displacement. No thresholds changed: the supplied predicate is reported beside
the independent supported-release, push, contact and retention sequence. A
supplied-metric pass alone does not establish physical recovery (confound 60).
