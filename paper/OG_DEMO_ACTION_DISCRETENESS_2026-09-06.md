# Are the original joystick commands suitable for a discrete action space? (2026-09-06, user question)

*Source: `inthewild_trials/<uid>_cartesian.npy`, key `cartesian_velocity` (the real joystick twist command, 30 Hz, reference frame 0), 96 trials, 226,130 steps. Note: the `vel_cmd` key in these files and in `<uid>_episodes.npy` is a copy of `joint_pos` (joint angles, ±2.6 rad), not a command — it was checked first and set aside. Script: inline, numbers from tool output 2026-09-06 23:05.*

## 1. What the commands look like

- **Four live axes:** linear x, y, z (cap 0.11 m/s) and wrist pitch (cap 1.0 rad/s); the other two twist components are always zero. Caps are exact (99.9th percentile = max = 0.11 / 1.0).
- **Idle dominates:** 68.2 % of steps have all four axes at zero. Of the active steps, 12.6 % move one axis, 18.7 % two, 0.5 % three or four.
- **Bang-bang on the linear axes, analog on pitch.** Of the nonzero values: x at the cap 58 %, y 55 %, z 80 %; pitch only 22 % at the cap and 64 % below half the cap. Between 12 % and 35 % of nonzero x/y values are below half the cap (analog easing at the start and end of a push).
- **Small vocabulary:** 76 distinct sign patterns over the active steps; the 16 most common cover 97.8 %, the top 8 cover 70 % (±x with ±y, single-axis pushes, z-down with pitch).
- **Not held as clean segments:** the constant-command run length has median 1 step (95th percentile 12), because analog values jitter step to step even during a "held" push.
- **Gripper:** a button; `gripper_pos` ramps at ~1–2 units per step on a 0–100 scale (4.4 % of steps change by > 1 unit). A binary open/close command with the recorder's ramp reproduces it.

| statistic | value |
|---|---|
| steps fully idle | 0.682 |
| active rows where every axis is zero-or-at-cap | 0.36 |
| active rows altered by > 0.25 cap on some axis under ternary snapping (|v| ≥ 0.5 cap → ±cap, else 0) | 0.277 |
| mean snapping error per active row (fraction of cap) | 0.026 |

## 2. Assessment

The recollection is right about the *intent*: the operator drives the linear axes bang-bang and combines at most two axes at a time, so the demonstrated behaviour lives on a small discrete vocabulary — a ternary code per axis, {−1, 0, +1}^4 = 81 words (16 of which carry 98 % of the data), plus a gripper bit. It is wrong about the *signal*: only 36 % of active steps are exactly zero-or-cap on every axis, the pitch axis is genuinely analog, and 28 % of active steps would move by more than a quarter of the cap if snapped. The sub-cap values are mostly the easing at the edges of a push, so snapping shortens or lengthens pushes by a few 30 Hz steps rather than changing where the arm goes.

**Suitable, with one empirical check before use.** A discrete action space is a faithful representation of the demonstrations' intent, and it would remove the idle/hesitation structure that currently makes the raw human set hard for pure imitation (the pruning issue) by turning it into "hold the zero word". The check that decides it: replay the snapped commands (ternary per axis, gripper bit, at 30 Hz) through the existing commanded-replay harness on the 74 success uids and compare the success funnel with the raw-command replay (pick 69 / placed 66 / contact 26 / nested 16 in w3). If snapping costs fewer than ~3 picks and ~3 contacts, the discrete space is usable for every learner (Dreamer supports categorical actors natively; DP and RLPD need discrete heads or a lookup over the 16–81 words). If it costs more, the analog easing matters physically and a 5-level code (0, ±0.5, ±1) is the next candidate.

## 3. What this would and would not change for the paper

It does not touch any result so far: every learner here uses continuous delta-joint actions re-derived from the replayed joint trajectories, not the joystick twist. A discrete-action arm would be a new condition (registered before running), most naturally for the world model (categorical actor) as a test of whether the human demos become *more* useful when their action space matches the operator's interface. Not started; nothing submitted.
