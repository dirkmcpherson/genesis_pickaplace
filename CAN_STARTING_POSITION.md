# Trial-specific can starting-position — status & handoff

_Working notes from a debugging session. Goal: give each demo episode the **true** can
starting position instead of the current 3-bucket guess._

---

## TL;DR
- The can's true start pose was **never recorded** — each `inthewild_trials/<uid>_episodes.npy`
  holds only `vel_cmd` (arm joint angles) + `gripper_pos`. No object pose.
- `example.py` currently snaps every episode to one of **3 hand-coded positions**
  (`POSITION_0/1/2`) via UID lists in `kinova.py`. Those buckets are **wrong by up to ~5 cm**
  and quantize away the real spread.
- **Working method:** recover each can position by **forward-kinematics on the recorded
  trajectory at the grasp instant** (pure FK, no physics). Deterministic, robust to human-demo
  noise. Tool: `can_pos_recovery/recover_can_position.py`.
- **Open question / blocked:** how to *validate* the recovered positions. Physics **playback is
  the wrong yardstick** (see "Why close-loop doesn't work"). Validate against vision / the
  robot's reported `tool_pose` instead.

---

## The problem in detail
- `trial_reader.py` builds the episodes from ROS bags but keeps only joint angles + gripper. It
  **reads and then discards** the robot's Cartesian `tool_pose` (`trial_reader.py:15`).
- `example.py` → `setup()` looks up the UID in `kinova.py`'s `TRIALS_POSITION_0/1/2` lists and
  places the can at one of three fixed `(x, y)` points (all at `x=0.4381`, only `y` differs).
- Consequence: the replayed arm reaches for where the can *really* was, but the sim put it
  somewhere else → grasp misses/clips. Real positions are continuous; buckets are 3 points.

### Measured bucket error (12-episode sample, 4 per position)
| pos | bucket (x, y)   | recovered mean (HIGH/MED conf) | x error |
|-----|-----------------|--------------------------------|---------|
| P0  | (0.438, +0.100) | **(0.475, +0.096)**            | ~+0.05 m |
| P1  | (0.438, −0.050) | (0.456, −0.032)                | ~+0.02 m |
| P2  | (0.438, −0.200) | (0.441, −0.165)                | ~+0.01 m |

---

## The method that works: FK-at-grasp (pure trajectory, NO playback)
`can_pos_recovery/recover_can_position.py`

1. **Find the grasp instant** robustly against human-demo noise (people exercise the gripper in
   mid-air and miss the can, so "first gripper close" is unreliable):
   - Binarize `gripper_pos` with an **absolute** closed threshold (rejects "never closed").
   - The real pickup = the closed interval that **lifts** and **releases near the known goal**
     (anchor on the fact that the can must end at the goal).
   - Grasp instant = **reach-bottom** (min EEF height) inside that interval.
   - Attach a **confidence** (HIGH/MED/LOW); LOW = a miss/never-closed → fall back, don't trust.
2. **Forward-kinematics** the recorded joint angles at that instant → world position of the
   **fingertip-midpoint** = can `(x, y)`. `z` = known table/ground height (~0.05).
   - This uses Genesis only as an FK calculator: `set_dofs_position(...)` then read link poses.
     **It never calls `scene.step()`** — no gravity, contacts, or dynamics. Deterministic.

### Result on the 12-episode sample
- **10/12 recovered with confidence**; the 2 LOW (`322`, `290`) are genuine misses where the
  gripper never closed — correctly flagged, not silently wrong.
- Recovered positions cluster tightly within each bucket (P0 x spans just 1.4 cm across 4
  independent episodes) — that consistency is the signal it's real.

### Per-episode recovered positions (sample)
```
uid pos  conf   recovered (x,y)      (LOW -> keep bucket)
235  0   HIGH   (0.478, +0.088)
232  0   HIGH   (0.488, +0.086)
257  0   MED    (0.442, +0.098)
242  0   HIGH   (0.493, +0.114)
322  1   LOW    -- miss (gripper never closed); FK leans toward P2 y, consistent w/ its p2_fail label
243  1   MED    (0.452, -0.037)
236  1   HIGH   (0.469, -0.055)
280  1   MED    (0.447, -0.004)
290  2   LOW    -- miss
278  2   HIGH   (0.445, -0.174)
237  2   HIGH   (0.453, -0.182)
234  2   HIGH   (0.424, -0.137)
```

### Why the blocking concern does NOT break recovery
The can can physically block/resist the arm during the demo — but `vel_cmd` is the robot's
**actual measured joint-encoder angles** (`/my_gen3_lite/joint_states`), not commands. They
already embed that interaction. The can being there is *why* the gripper stopped there. FK on
real achieved angles → true gripper pose → can position. Blocking helps, not hurts.
(Caveat: if a *failed* approach nudged the can before the successful grasp, the recovered point
is where the can was *at grasp*, which may differ from its original rest pose. Minor for clean
grasps; the confidence flag catches the messy ones.)

---

## Why "close-loop" (playback) does NOT validate this
We tried placing the can at the recovered position and re-running the replay to see if success
went up. It **did not** (`close_loop.py`: bucket 2/12 vs recovered 1/12). That is **not**
evidence the recovery is wrong — playback is the wrong test, for fundamental reasons:

1. **The recorded trajectory is a closed-loop human artifact.** The person felt the can
   resist/block and adjusted in real time. Replaying those joint-position commands **open-loop**
   against the can means the position controller jams into the can (force spikes, can shoved),
   with no feedback to adapt. Placing the can *correctly* can make playback *worse* by surfacing
   the blocking the human negotiated away.
2. **The success metric conflates three things** — grasp + transport + the place landing on the
   *fixed* `goal_bottle` at (0.6, −0.2, 0.19). Correct pickup doesn't fix the place.
3. **GPU contact non-determinism** flips borderline episodes between identical runs (e.g. ep 237
   passed in one baseline run, failed in another). With ~1–3 successes/12 the metric is noise.
4. Even baseline replay only reproduces ~**3/12 (25%)** of *real-world-successful* trials → the
   open-loop replay is low-fidelity regardless of can position.

**Takeaway:** validate the recovery with a method that doesn't depend on physics rollout (below),
and treat closed-loop control as the real fix for the task, not open-loop replay.

---

## Status
- [x] Confirmed the data lacks object pose; buckets are wrong (~5 cm, measured).
- [x] FK-at-grasp recovery implemented, robust to test-pulses/misses, confidence-flagged.
- [x] Recovered 10/12 sample episodes; flagged the 2 misses.
- [x] Established that playback/close-loop is the wrong validation (and wrong control approach).
- [ ] **IN PROGRESS / NEXT** (see below).

## Next steps (priority order)
1. **Validate recovery without playback.** Two gripper-independent ground truths:
   - **`tool_pose`**: re-process the raw bags keeping the Cartesian EEF pose that
     `trial_reader.py:15` currently throws away. Robot-reported, no FK/URDF mismatch. Compare to
     the FK estimate; should agree.
   - **Vision**: detect the can in frame 0 of the raw camera MP4s
     (`inthewild_trials/raw/user_<id>/cam_dev_video*`) + `config.yaml` (calibration), back-project
     to the table plane. Independent check.
2. **Run recovery over ALL ~77 successful episodes** (not just the 12 sample) → produce a
   per-episode `{uid: (x, y)}` table to replace the bucket lookup in `kinova.py` / `example.py`.
3. **Fix the place/goal too**, not just pickup: recover each episode's *release* point (already
   computed as a by-product) and/or make `goal_bottle` per-episode, so the success reward is
   actually reachable.
4. **Use the confidence flag as a demo-quality filter** — drop/triage the flagged misses.
5. **For task success, move to closed-loop control** (servo on can/EEF pose) instead of open-loop
   joint playback — the open-loop replay can't reproduce a closed-loop human demo.

---

## Tooling (in `can_pos_recovery/`, runs in the project venv)
```bash
.venv/bin/python can_pos_recovery/recover_can_position.py        # recovered positions + confidence
.venv/bin/python can_pos_recovery/validate_grasp.py <force|baseline>   # success across episodes/positions
.venv/bin/python can_pos_recovery/close_loop.py <bucket|recovered>     # the playback test (see caveats above)
```
These are standalone; none of them import or modify `example.py`.

## Environment notes
- venv at `.venv` (Python 3.12), `genesis-world` 1.2.0, torch cu130, runs on GPU.
- Robot meshes **vendored** into `kortex_description/` — no ROS install needed.
- `inthewild_trials/` is a **gitignored symlink** to the external dataset (too big to vendor).

## Related issues found along the way (not the position work, but relevant)
- Don't "fix" gripper clipping by lowering grip force — it **breaks the open-loop grasp** (drops
  the can). The current `example.py` finger-force cut (±10/±5) regressed pickup.
- Gyration/"wild" motion is separately reducible via solver `substeps` (orthogonal to position).
- `kinova.py` label bugs: UID **322** is in both `p1_succ` and `p2_fail`; **331** is duplicated.
