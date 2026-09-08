# Human reference: end-effector action distribution per 0.12 s decision

**Date:** 2026-09-07 · **Branch:** `4dof-cartesian` · **Scope:** HUMAN arm only (offline, local data).
**Script:** `can_pos_recovery/eef_action_dist.py` · **Figure:** `paper/figures/fig_eef_action_dist_human.{png,pdf}`
**Per-decision arrays:** `paper/figures/eef_action_dist_human.npz` (data, not committed) · **shared bins:** `paper/figures/eef_action_bins.npz`

The learner half (DP / RLPD / world-model rollout actions through the same binning) is **not** in this
document — it needs policy rollouts and the cluster was unreachable (Tufts VPN down). The script and the
bin file are the drop-in point: `--source <arm> --rollouts <npz> --bins paper/figures/eef_action_bins.npz`,
then `--plot --source human --source <arm>` overlays them on the identical grid. That path is smoke-tested
(§7).

## 1. What is measured, and why not `vel_cmd`

The learners do not act in raw joystick space: they emit `delta_joint` on a **0.12 s decision cadence**
with a per-step cap of 0.025. The quantity that is comparable across arms is therefore the
**end-effector displacement per decision** — `dx, dy, dz` [m] and `dyaw` [rad about world z].

Each tape is resampled onto a uniform 0.12 s grid using the **real timestamps** in
`inthewild_trials/<uid>_timed.npz` (`fb_t` / `fb_tool`), and the pose is differenced across consecutive
decisions. Per CONFOUNDS row 46 the tapes run at **29.63–39.13 fps (median 32.79)** — measured here over
all 74 tapes — so the recorder's fixed 4-frames-per-decision stride is wrong by up to 20 %; §6 quantifies
what that stride costs on this statistic.

## 2. Tapes used and skipped (no membership invented)

`inthewild_trials/` holds **96** `*_cartesian.npy` tapes and **74** `*_timed.npz`. The 74 uids with a
timestamped re-extraction are **exactly** the 74-uid full-scope set of record — verified by set equality
against the w3 honest census (`baselines/demos_v2/census_ts5w4_0903/full_gc_kp4_riser3_shelf6_honest.json`):
0 in one and not the other. All 74 carry `label: success` in `baselines/demo_manifest_auth.json`.
**All 74 were used; 0 were skipped for data reasons** (every tape yielded ≥ 2 decisions).

The 22 tapes not used are not missing bags (every one of the 22 bags is present under
`inthewild_trials/raw/`); they are outside the set of record: **16 `fail`-labeled**, **5 absent from the
manifest** (stubs: 253, 285, 289, 290, 322) and **1 `success`** — uid **303**, the single documented
recovery casualty. Excluded uids:
`238 240 249 253 260 268 270 282 285 288 289 290 296 303 307 310 312 313 314 322 324 334`.

**n = 37 216 decisions** over 74 tapes (per tape: median 378, min 146, max 1591).

## 3. Verification verdicts

### V1 — What the six `tool_pose` columns are, and radians vs degrees

**VERDICT: `[x, y, z]` in metres in the robot base frame, then `(theta_x, theta_y, theta_z)` in DEGREES,
extrinsic-xyz Euler (ROS roll–pitch–yaw): `R = Rz(θz)·Ry(θy)·Rx(θx)`. `theta_z` is the yaw about world z.**

* **Degrees, not radians.** Pooled ranges over all 74 tapes: `θx ∈ [82.51, 107.63]`, `θy ∈ [−180.00, 180.00]`,
  `θz ∈ [4.04, 164.66]` — impossible as radians, and `θy` sits on the ±180 branch cut.
* **Convention determined, not assumed.** The relative rotation `ΔR = R(t+1)R(t)ᵀ` was compared, as an
  axis direction, against the independently measured `tool_twist` angular velocity on high-rotation frames
  (median per-tape cosine, 74 tapes):

  | candidate | ω in base frame | ω in body frame |
  |---|---|---|
  | **`xyz` extrinsic (RPY)** | **+0.841** (p10 +0.503, p90 +0.969) | −0.056 |
  | `XYZ` intrinsic | +0.057 | +0.590 |
  | `zyx` extrinsic | −0.064 | −0.050 |
  | `ZYX` intrinsic | −0.037 | +0.052 |

  `xyz`-extrinsic with a **base-frame** ω wins decisively. (This also fixes `tool_twist[:,3:6]` as base-frame.)
* **Independently confirmed by forward kinematics**, which is a different signal entirely (`joint_pos`).
  FK through `gen3_lite_2f_boxfingers.urdf` (joint_1…joint_6, then dummy_link + 0.130 m along z = `tool_frame`)
  reproduces `tool_pose[:, :3]` to **0.0000 m median, 0.0023 m p95** over all 74 tapes. (FK stopped at
  `end_effector_link` is off by exactly 0.1300 m — the tool offset — which is the cross-check that the
  matching one is not a coincidence.) This simultaneously confirms `joint_pos` = joint_1…joint_6 in order,
  in radians.
* **Which column is yaw.** World-z yaw extracted from the FK rotation matrix vs `θz`: per-tape Pearson
  **r = 1.0000 median (min 0.9988)**, with a **constant 180.000° offset** (a tool-frame convention offset
  that cancels in every difference). So `θz` *is* the yaw about world z.
* **Base-joint sanity check — partly confirmed, stated honestly.** Numerical `|∂yaw/∂q_i|` (median over
  74 tapes) = **[1.000, 0.069, 0.069, 0.820, 0.130, 0.013]** for joint_1…joint_6. joint_1, the base
  rotation about world z, has sensitivity **exactly 1.000** as expected — but joint_4 contributes 0.820,
  so yaw is *not* dominated by the base joint alone. A naive `yaw ~ a·j1 + b·j6` regression gives only
  R² 0.50 (0.20 in delta form) and would have been misleading; **FK, not the regression, is what settles
  the column.**
* **Ancillary, worth recording:** the tool frame's *y* axis points along world −z in **100 %** of frames
  (z-component p1 −0.998, p50 −0.997, p99 −0.992), and roll/pitch are near-constant (per-tape ptp median
  `θx` 2.8°, `θy` 0.8°). The arm is effectively **4-DOF in task space** — which is what makes
  (x, y, z, yaw) the right coordinates.

### V2 — Does yaw wrap?

**VERDICT: `θz` never wraps in this corpus; the code handles the wrap anyway, so the `dyaw` tails are not
fabricated.**

`θz` spans 4.04–164.66° pooled, per-tape ptp median 26.1°, and **0 of 74 tapes** contain a `|Δθz| > 180°`
step. (`θy` does cross the branch cut in 2/74 tapes, but `θy` is not used.) The script nonetheless unwraps
`θz` *before* interpolation and wraps every `dyaw` into (−π, π]; both are no-ops on this data, verified.

### V3 — `tool_pose` vs `fb_tool`: which is the genuine feedback?

**VERDICT: `fb_tool` (from `_timed.npz`) is the genuine feedback signal and is what I used.**

Both come from the same `base_feedback` field. `_cartesian.npy` `tool_pose` is a **per-window mean** on a
~32.8 Hz grid driven by only three topics, with **no timestamps**; `_timed.npz` `fb_tool` is **every**
`base_feedback` message, **unaveraged**, on a clean **40.00 Hz** clock (measured: exactly 40.00 min and max
across all 74 tapes) **with** timestamps. They agree: on a normalized-time comparison the position
difference is **0.1 mm median / 6.4 mm p95 / 54.9 mm max** and the `θz` difference **0.0034° p50 / 0.233°
p95** (the tails are resampling mismatch between two different grids, not disagreement). `fb_tool` also
genuinely refreshes every message — the fraction of consecutive identical rows is **0.000** — so there is
no staircase to alias.

Consistency of pose against the *other* independent channel: net yaw change per tape from the pose vs the
integrated base-frame ω_z gives a **ratio of 0.951 median**.

### V4 — Unexpected, and ambiguous: the commanded yaw channel

Across all 74 tapes the commanded joystick twist `cartesian_velocity` has **`wx ≡ 0` and `wz ≡ 0` exactly**
(1 unique value each over 177 238 rows). The only commanded rotation channel is `wy`, and `wy` correlates
with the realized **dyaw** (median per-tape r **−0.550**, p10 −0.691 at the 0.12 s scale) and with the
realized `dθ_y` at r = **0.000**. So the operator's single rotation axis produced **world-z yaw**.
`baselines/cartesian_env.py` calls that channel `PITCH_CAP` / `dpitch`.

**This is AMBIGUOUS and I did not resolve it or change any code.** `cartvel_reference_frame` is **0**
(`UNSPECIFIED`) in all 74 tapes, so the frame is not self-documenting. The −0.55 (not −0.9) correlation is
consistent with `wy` being about the *tool* y axis — which we measured to point along world −z — but is
too weak to be proof. **What would settle it:** a Kortex-side reference-frame confirmation, or one
deliberate single-axis teleop trial. Flagged for the head agent because it bears on the naming and the cap
in `cartesian_env.py`, not on any number below (everything below is derived from the pose).

## 4. Marginal quantiles (n = 37 216 decisions)

| channel | 1 % | 5 % | 25 % | 50 % | 75 % | 95 % | 99 % | max abs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| dx [mm] | −13.149 | −4.749 | −0.041 | 0.000 | 0.148 | 10.162 | 13.270 | 21.588 |
| dy [mm] | −13.486 | −7.803 | −0.182 | −0.001 | 0.093 | 7.207 | 13.470 | 26.371 |
| dz [mm] | −12.521 | −2.185 | −0.108 | −0.000 | 0.113 | 3.617 | 13.216 | 34.420 |
| dyaw [deg] | −2.932 | −0.299 | −0.020 | −0.000 | 0.020 | 0.293 | 2.178 | 5.693 |

The p1/p99 of all three translation channels sit at **±13.2–13.5 mm**, which is
`VCAP × dt = 0.11 m/s × 0.12 s = 13.2 mm` — the teleop plugin's velocity cap
(`baselines/cartesian_env.py:49`). The action range is set by the joystick, not by the task. Excursions past
it (max 21.6 / 26.4 / 34.4 mm) are the 0.1 % tail: overshoot and settling.

## 5. Zero-motion fraction, and the shape

Fraction of decisions with `|Δ| <` threshold:

| threshold | dx | dy | dz | | threshold | dyaw |
|---|---:|---:|---:|---|---|---:|
| 0.01 mm | 0.300 | 0.237 | 0.267 | | 0.02° | 0.501 |
| 0.10 mm | 0.524 | 0.470 | 0.489 | | 0.10° | 0.751 |
| 0.50 mm | **0.707** | **0.674** | **0.721** | | 0.50° | **0.926** |
| 1.00 mm | 0.762 | 0.755 | 0.819 | | 1.00° | 0.949 |
| 2.00 mm | 0.803 | 0.811 | 0.882 | | 2.00° | 0.971 |

The **0.5 mm / 0.5°** row is the operative one: it is comfortably above the measured still-band and well
below the 13.2 mm cap. Independent confirmation that 0.5 mm is above the noise floor: **31.6 %** of
decisions are *quiet* (no `/cartesian_velocity` message landed in the 0.12 s window, i.e. the joystick was
centred), and on those the displacement is `|dx|` p50 **0.0087 mm** / p95 0.99 mm, `|dy|` 0.018 / 1.08,
`|dz|` 0.013 / 1.17, `|dyaw|` 0.0031° / 0.161°.

At that threshold: **51.4 % of all decisions move nothing at all** in any of the four channels. Per
channel, motion occurs in dx 29.3 %, dy 32.6 %, dz 27.9 %, **dyaw only 7.4 %**.

**Shape: a zero atom plus a broad, near-uniform plateau out to the joystick cap.** Among decisions that do
move, the median displacement is dx 4.56 mm, dy 3.00 mm, dz 1.50 mm, dyaw 1.62°, and only 14–18 % are
within 10 % of the cap. So the realized end-effector motion is **not** bang-bang — even though the operator
was driving a capped joystick.

**Axis alignment (the visible cross in the figure).** Of decisions with any translation motion, the
fraction moving exactly one axis rises with the size of the motion — 0.360 at a 0.5 mm threshold, 0.578 at
2 mm, **0.682 at 5 mm**, 0.746 at 8 mm (three axes at once: 0.235 → 0.039 → 0.006 → 0.003). Large human
motions are close to axis-aligned; small ones are not.

## 6. Correlation structure

Pearson r over all 37 216 decisions:

|  | dx | dy | dz | dyaw |
|---|---:|---:|---:|---:|
| **dx** | 1.000 | −0.116 | −0.023 | −0.000 |
| **dy** | −0.116 | 1.000 | −0.000 | 0.044 |
| **dz** | −0.023 | −0.000 | 1.000 | **−0.215** |
| **dyaw** | −0.000 | 0.044 | −0.215 | 1.000 |

The four channels are **very nearly independent**: the largest coupling is dz–dyaw at −0.215 (the wrist
yaws while lowering), then dx–dy at −0.116; everything else is within ±0.05 of zero. Combined with the
axis-aligned cross above, the joint density is well approximated by the product of its marginals plus a
mild dz–dyaw shear — which is what makes the per-channel discrete analysis in §7 the right one.

**Clock sensitivity (CONFOUNDS row 46).** Rebuilding the same statistic on the recorder's naive
4-frames-per-decision stride (`--grid frames`, arm `human_framesgrid`) inflates the distribution: p99 rises
by **+10.8 % (dx), +8.1 % (dy), +6.6 % (dz), +3.2 % (dyaw)** and the per-channel sd by +2.4 % to +3.5 %,
with the zero-fraction essentially unchanged (0.707 → 0.717 for dx). The naive stride therefore overstates
how fast the human moved per decision by up to ~11 % in the tails — the sign and size expected from a
median 32.79 fps against an assumed 33.33.

## 7. The discrete-action question

Setup: a uniform grid of **B levels per channel** spanning ±R, snapping each decision to the nearest level.
Odd B always places a level at exactly 0, so the 51 % zero atom is represented perfectly. Two ranges are
reported: **R = p99.9** (`dx` 13.79, `dy` 15.81, `dz` 15.55 mm, `dyaw` 5.40°, clipping 0.1 % of decisions)
and **R = max** (21.59 / 26.37 / 34.42 mm, 5.69°, clipping nothing).

### How many bins to cover the data without clipping meaningful motion

* **Range.** ±13.2 mm covers 99 % of every translation channel — it is the joystick cap and the natural
  clip point. Nothing is lost that matters until you go below it; going above it to ±34 mm buys the 0.1 %
  overshoot tail.
* **Resolution.** Bins needed so that the half-bin width falls under a given tolerance:

  | tolerance | dx | dy | dz | dyaw |
  |---|---:|---:|---:|---:|
  | 2 mm / 2° | 7 | 8 | 8 | 3 |
  | 1 mm / 1° | 14 | 16 | 16 | 6 |
  | 0.5 mm / 0.5° | 28 | 32 | 32 | 11 |

  The measured still-band is ~1 mm (quiet-decision p95), so **≈15 levels per translation channel and ≈6 for
  yaw** is the point at which quantization stops being distinguishable from sensor noise.

### What a small grid actually represents

| B | bin width dx | err/(half-bin), **all** decisions | err/(half-bin), **moving** decisions | frac of decisions within 1 mm of a level | open-loop tape drift, xyz [mm] p50 / p95 / max |
|---:|---:|---:|---:|---:|---:|
| 3 | 9.20 mm | 0.15 | **0.48** | 0.786 | **68.1** / 119.6 / 135.1 |
| 5 | 5.52 mm | 0.17 | **0.50** | 0.819 | 32.6 / 64.1 / 111.9 |
| 7 | 3.94 mm | 0.18 | **0.52** | 0.852 | 24.8 / 52.0 / 66.3 |
| 9 | 3.07 mm | 0.19 | 0.53 | 0.897 | 19.5 / 40.5 / 50.3 |
| 15 | 1.84 mm | 0.22 | 0.54 | 0.999 | 10.3 / 27.3 / 42.2 |
| 21 | 1.31 mm | 0.24 | 0.53 | 1.000 | 9.3 / 20.2 / 30.2 |
| 31 | 0.89 mm | 0.26 | 0.49 | 1.000 | 8.0 / 17.0 / 31.6 |

(R = p99.9. With R = max and no clipping at all, the drift ladder is 110.7 / 59.9 / 44.2 / 32.4 / 23.2 /
17.0 / 10.9 mm p50 for B = 3 / 5 / 7 / 9 / 15 / 21 / 31; yaw drift p50 8.32° / 4.44 / 3.44 / 2.07 / 1.45 /
1.27 / 0.93.) "Open-loop tape drift" = the end-of-tape position error from replaying the snapped deltas
instead of the true ones — the direct test of whether the action space can *express* the human data.

**The decisive column is `err/(half-bin) on moving decisions`, which is ≈0.5 at every B.** That is exactly
the value expected when a signal is spread *uniformly inside its bin* — i.e. **the human's realized
end-effector motion is genuinely analog, not a coarse code that a small grid happens to land on.** The
flattering-looking `all decisions` column (0.15–0.26) and the "79 % within 1 mm at B = 3" figure are both
artifacts of the 51 % zero atom: a 3-level grid nails the stillness and butchers every actual movement.

### Answer

**A small discrete end-effector grid is a poor fit for this dataset; a moderate one is fine.**

* **B = 3** (the ternary code that worked on the *commanded* joystick, `DISCRETE_ACTION_REPLAY_2026-09-06.md`)
  gives a 9.2 mm bin and a median **68 mm** open-loop drift per tape — **larger than the can's 66 mm
  diameter**, and roughly 20× the millimetre-scale tolerance of the pinch grasp. Unusable for the realized
  EEF action.
* **B = 5 → 33 mm, B = 7 → 25 mm** median drift. Still far above anything the grasp or the nesting test
  (8.1 cm centre-distance) can absorb without changing outcomes.
* **B ≈ 15 per translation channel and B ≈ 6 for yaw** is the first honest setting: 1.8 mm bins put the
  quantization error inside the measured still-band (99.9 % of decisions land within 1 mm of a level) and
  bring median tape drift to **10 mm**, comparable to the sim's own real2sim fidelity. As a *factored*
  action space that is 15 + 16 + 16 + 6 = **53 logits**, which is cheap; as a *joint* grid it is
  15·16·16·6 ≈ 23 000 actions, which is not.
* The reason the ternary result does **not** transfer: the *commanded* joystick is nearly a 3-level code
  (81–93 % of samples are exactly zero, and the non-zero ones ride the ±0.11 cap), but the arm's *realized*
  displacement per 0.12 s is a smooth, plateau-shaped analog distribution — the same gap that made the raw
  commanded-Cartesian path reproduce only 17/74 picks. Discretize the realized velocity, not the command,
  and use ≥15 levels.

**Caveat on scope:** this is a *representation* result — how well a grid can express the human tapes
open-loop. A closed-loop policy re-plans every decision and can absorb some quantization, so B = 7 is not
necessarily fatal for *learning*; it is fatal for *replaying* human data. Deciding the former needs a
trained arm, which needs the cluster.

## 8. Reproduce

```bash
PY=~/workspace/genesis_sim2real/venv/bin/python
$PY can_pos_recovery/eef_action_dist.py --build --report --plot          # human reference
$PY can_pos_recovery/eef_action_dist.py --build --source human_framesgrid \
      --from-tapes --grid frames --bins paper/figures/eef_action_bins.npz   # row-46 sensitivity
# later, a learner arm on the SAME bins:
$PY can_pos_recovery/eef_action_dist.py --build --source rlpd_dH --rollouts <rollouts.npz> \
      --bins paper/figures/eef_action_bins.npz
$PY can_pos_recovery/eef_action_dist.py --plot --source human --source rlpd_dH
```

Rollout npz format is in the script docstring (`deltas` (N,4) in m/m/m/rad, or `poses` (N,4) +
`episode`); anything else raises rather than being inferred. The overlay path was smoke-tested by plotting
`human` against `human_framesgrid`.

## 9. What is not settled

1. **The learner half.** No policy rollouts exist locally; every learner panel is blocked on the cluster.
2. **V4** — whether the joystick's `wy` is a tool-frame yaw command (measured to drive world yaw, r −0.55)
   or something the `cartesian_env.py` `PITCH_CAP` naming describes correctly. `cartvel_reference_frame`
   is `UNSPECIFIED` in every tape, so the data cannot decide it.
3. **The 74-tape denominator carries uids 234 and 318**, which CONFOUNDS row 51 records as unwinnable in
   sim by construction (90° lying-can ICs; the tip rule fires at decision 1 there). Their *real* tapes are
   full-length human demonstrations and contribute **627 of the 37 216 decisions (1.7 %)** here. Dropping
   them moves nothing that matters: dx p1/p99 −13.149/+13.270 → −13.146/+13.270, dyaw p1/p99
   −2.932/+2.178 → −2.920/+2.169, and the 0.5 mm zero-fraction for dx 0.7070 → 0.7107. Kept, for
   consistency with the other n = 74 tables.
