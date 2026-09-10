# Expanded episode verification:36 additional recordings

**Complete:** all144 replays and36 four-condition groups passed execution and
invariant checks. Original / soft1 / soft2 supplied-metric passes are6/36,6/36,
7/36; strict completions are3/36,1/36,1/36. Soft2's additional191 proximity pass
moves the goal178.10mm and lacks supported release. No candidate is adopted.
See [FINAL_READOUT.md](FINAL_READOUT.md) for paired outcomes and day breakdowns.

The user requested a broader episode sample and more parallel execution.
This frozen comparison adds36 distinct recordings,12 per study day, to the12
already used in the soft-pad experiment. There are144 new replay jobs: rigid
and soft pads, each with normal damping1 and2. Original fixed reconstructions
are retained as references. Settings match the preceding panel; the new spin
hypothesis is excluded. No parameter or pose is fitted to this sample's outcomes.

Selection sorts eligible UIDs by SHA256 of the fixed string
`soft-pad-expanded-2026-09-10:<uid>` and takes12 per day. It does not inspect
outcomes, durations or initial geometry. All151 eligible unused source records
and their selection keys are in `plan.json`. These existing source pools were
used in historical development; the sample is new to this material experiment,
not pristine unseen data or independent participants.

- 12-16: 173, 161, 179, 137, 139, 162, 165, 151, 180, 154, 138, 158
- 12-17: 220, 209, 218, 204, 196, 225, 197, 201, 210, 213, 231, 191
- 12-18: 262, 308, 254, 298, 333, 232, 239, 248, 244, 242, 263, 320

`initial_geometry_audit.json` flags can/shelf overlap in220,161,196,151,201.
All five remain in the declared sample, with source poses unchanged. Report both
the entire sample and a clearly labeled geometry diagnostic subset if helpful;
never discard failures or tune metrics to improve yield.

Eight one-thread CPU workers launched after the five-run numerical verification
gate and two exact controls passed. Host inspection confirms eight workers at
about96% CPU each, roughly1.3GB RSS each, and8.6GiB available RAM. `gate_passed.json`
records the validated merged report/identity hashes. No cluster resources used.

`execution.json` and per-job logs are authoritative for terminal runs.
`readout.py` verifies complete source coverage, commands, geometry, feedback and
contact invariants, and reports aggregates only on complete four-condition groups
with the denominator stated. Pending runs and implementation failures are separate
from task outcomes. No recovery claim or adoption from launch alone.

## Initial partial results and video coverage

At the current readout40/144 replays and10/36 complete four-condition groups are
available. Original supplied/strict passes0/10,0/10; rigid1 1/10,1/10;
soft1 2/10,1/10; rigid2 2/10,1/10; soft2 2/10,1/10.262 is the sole new strict
completion so far and every new condition recovers it. These partial totals are
not the completed sample result; consult `summary.json` for later updates.

The real-camera inventory covers all36 selected UIDs.23 have both camera streams
with matching monotonic timestamp counts and sampled decode checks.13 have
missing timestamp files or count/decode checks needing diagnosis; all remain in
the physics sample. `physical_reference/manifest.json` preserves those errors.
The262 review video is verified; independent EEF-action verification is pending.
