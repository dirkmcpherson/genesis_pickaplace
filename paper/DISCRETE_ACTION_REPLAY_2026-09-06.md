# Discrete-action replay check: do snapped joystick commands still reproduce the demos? (2026-09-06)

> **Second follow-up 2026-09-08 — replay is NOT free, and there is now a mechanism for it.** See `REPLAY_YIELD_2026-09-08.md`. Measured on an independent task: human demonstrations replay to success 185 of 200 times, machine demonstrations 114 of 200, in the same environment with the same code — and **any** action edit collapses the yield, to 0 of 200 under two different smoothing conditions and 5 of 200 for human actions at a realistic roughness. **The mechanism:** relative one-step error is source-independent at about 0.57, so absolute divergence scales with the size of the commanded step, which is 4.6× larger for the machine data. That is a property of step size, not of demonstration quality.
>
> **Consequence for this document's programme.** The recommended next step here — build a discrete arm on realised velocity and re-check — must budget for losing most of its data at build time, and for what survives being both smaller and selected. Test the replay yield first: it costs minutes of CPU and can be measured before committing any GPU time. This contradicts the assumption under which the discrete-action line was planned, where re-execution was treated as approximately free.

> **Follow-up 2026-09-07 — read this before quoting "ternary costs nothing".** This document measures the *commanded* joystick signal, and its own power caveat fired: that path reproduces only 17 of 74 picks, because the integrated twist drifts a median 4.2 cm from the real tool pose by grasp time. The recommended follow-up — build the discrete arm on the **realised** end-effector velocity instead — has now been done offline (`EEF_ACTION_DIST_2026-09-07.md`), and it reverses the practical conclusion. On realised per-decision motion, quantisation error stays at roughly half a bin at *every* grid size, and open-loop replay through a quantised space drifts a median of **68 / 33 / 25 mm at 3 / 5 / 7 levels** per channel against a **66 mm** can. The apparent "79 % of decisions represented at 3 levels" is the **51.4 % zero atom**, not fidelity on the decisions that move. First honest setting: **~15 levels per translation channel, ~6 for yaw**. So: snapping the *commanded* signal is lossless, snapping the *realised* motion to a small grid is not, and only the latter is the action space a learner would use.

*User question (09-06): "I'm curious about [the discrete action space] ... dv3 works better with discrete actions." Measured command statistics: `paper/OG_DEMO_ACTION_DISCRETENESS_2026-09-06.md`. Script: `baselines/discrete_replay_check.py`. Outputs: `~/wm_fix_2026-09-03/discrete_replay/` (per-uid json + trajectory npz per code, `vocab.json`, `table.{md,json}`, logs). Registered 2026-09-06 23:10 before any full run (§2); one smoke uid seen first, disclosed in §2.*

## 1. Setup (verified, not assumed)

- **Path of record for this check: the commanded-Cartesian replay** `baselines/cartesian_env.CartesianCanEnv(control='vel')` (built 07-24..26): each tape frame's twist integrates a tool setpoint at `DT = 0.025` s (clamped to the teleop workspace box; pitch setpoint clamped to `PITCH_RANGE`), the wrist is IK'd every frame, and `GenesisCanEnv.step` applies the joint-position targets + the gripper target (`gripper_targets(0..100)`) for 3 physics steps of 0.01 s (substeps 8). The tape is `inthewild_trials/<uid>_cartesian.npy`: one frame per `base_feedback` message (40 Hz, 0.025 s; the discreteness note's "30 Hz" was the `_episodes.npy` rate), `cartesian_velocity` (x, y, z, pitch = wy; wx = wz = 0 on every frame, asserted) and `gripper_pos` (0..100). Per-uid least-squares dt of Σ command vs `tool_pose` displacement: median 0.0257 s (consistent with the path's DT). Same path, same DT, same physics for every code — the comparison of record is raw-commanded vs snapped-commanded on this path.
- **World:** `gc_kp4_riser3_shelf6` via `sim_variant_hook.apply_pre` (before the world build) + `apply_post(env, name)` (after) — exactly as the recorder and `honest_rescore`. `CartesianCanEnv`/`GenesisCanEnv` do NOT apply the variant themselves (the 07-24 validation "uid 232 picks+contacts" was the base world); the script applies it and asserts the shelf top 0.17 / goal spawn 0.263 / pick_z 0.1505. Disclosed path detail: the wrist→tool offset is calibrated at reset against `REF_TOOL_AT_START` (a base-world constant); under the 3 cm riser this only mis-sizes the pitch lever arm (≤ 1.6 cm at the demo pitch extremes), identically for all codes.
- **Initial conditions:** `GenesisCanEnv.reset(uid=...)` = the recovered can placement of record (`can_pos_recovery/trial_placements.json`, `status ∈ {ok, ok_batch}`) + the static goal (0.672, −0.221) at the shelf-following spawn height. **Uids:** the 74 success-labelled solved trials (= the w3 census fit+holdout set, checked equal).
- **Stage predicates (the harness's own):** `picked` = GenesisCanEnv's sustained held-can guard; `contact` = GenesisCanEnv's (picked ∧ can–goal contact ∧ eef behind the can); `placed` = honest_rescore's `placed_shift` (picked ∧ shelf footprint ∧ shelf_top+0.01 < z < shelf_top+0.07 — the env's own `placed` band is unshifted and cannot fire in shelf6 worlds, recorded as `placed_env` only); `set-down` = placed ∧ grip cmd < 0.5 ∧ |z − rest| < 1.5 cm ∧ tilt < 20°; `nested` = `GenesisCanEnv._nested()` at the tape's end (100 settle steps, picked ∧ centre distance ≤ 0.081 ∧ both upright) — the eval rule of record; `tipped` = can tilt > 60° after the settle (also logged: first frame with tilt > 60°, and with tilt > 60° while the grip cmd < 0.3 = FullTaskEnv's tip rule). Stage ladder for flips: none < picked < placed < contact < nested (census order).
- **Determinism / load:** every process pins taichi + torch to 1 CPU thread (`--threads 1`; injected into `ti.init`); smoke uid 232: the 1-thread and default-thread runs are bit-identical (max per-frame can/tool difference 0.0). 8 processes on the idle 12-core box (load 0.6 at launch), 6 codes × 8 uid-shards = 48 jobs, every code runs the same shard uid order (same in-process history per shard).

### Codes

| code | arm (x, y, z at cap 0.11 m/s; pitch at cap 1.0 rad/s) | gripper |
|---|---|---|
| raw | recorded twist | recorded `gripper_pos` |
| tern | per axis: \|v\| ≥ 0.5·cap → ±cap, else 0 | ternary ramp |
| five | per axis: nearest of {0, ±0.5·cap, ±cap} | ternary ramp |
| held | tern, then a word-level mode filter over a 5-frame window (±2 frames; ties keep the centre word) — removes step-to-step jitter; non-causal by 2 frames (offline tape transform) | ternary ramp |
| tern_rawgrip | tern | recorded (isolates the arm code) |
| raw_terngrip | recorded | ternary ramp (isolates the gripper code) |

**Gripper ternary ramp.** This harness has no gripper bit, so the script builds one. Measured on the 74 tapes: the real gripper position ramps at 1.49 (closing) / 1.52 (opening) units per frame (medians of |Δgp| > 0.3) and stops wherever the button is released — closed plateaus median 63, p10 37, p90 88; open plateaus median 12, max 29; 1–3 close cycles per demo. So the operator's gripper command is itself three-valued, {open, hold, close}, driving a fixed-rate ramp. Code: `u_t = sign(gp_t − g_{t−1})` if `|gp_t − g_{t−1}| > 0.75` else 0; `g_t = clip(g_{t−1} + 1.5·u_t, 0, 100)`; `g_0 = gp_0` (the initial state). The ramp tracks the recorded position to within ~1.5 units (max deviation logged per tape).

### Vocabulary (pre-run, no simulation; `vocab.json`)

Over the 74 tapes (177,238 frames): 25.1 % of frames carry a nonzero ternary arm word. **45 distinct arm words** (of 81); the top 8 cover 77.9 %, the **top 16 cover 99.1 %**, 34 words reach 99.9 %. Per uid: median 13 distinct words (max 22). Ternary runs are short (median 4 frames, p90 11); the held filter lengthens them (median 7, p90 16). Top words: +z (13.2 %), +x (12.8 %), +x−y (11.0 %), −z (10.1 %), −y, +y, +x+y, −x−y, −x, −x+y (5.7 %), then the two pitch-only words (3.1 / 2.9 %) and z∓pitch combinations (≤ 1 %); every word past rank 16 is < 0.15 %.

## 2. Registered predictions (written 2026-09-06 23:10, before the runs)

Disclosure: the pipeline smoke ran raw and tern on ONE uid (232) before this was written: raw → placed (can 18 cm from the goal, tipped), tern → contact (6.7 cm, tipped). The predictions below are about the 74-uid net counts and were not adjusted to that uid.

"Cost" = (raw count − code count) over the 74 uids, per stage; a negative cost is a gain.

- **P1 (the brief's):** tern costs ≤ 3 picks and ≤ 3 contacts vs raw.
- **P2 (the brief's):** five costs ≤ 1 pick and ≤ 1 contact vs raw.
- **P3:** the gripper code alone (raw_terngrip vs raw) costs ≤ 1 pick and ≤ 1 contact (the ramp stays within 1.5 units of the recorded position).
- **P4:** held costs ≤ 3 picks and ≤ 3 contacts vs raw (±2 frames = ±50 ms of timing).
- **P5 (noise scale):** per-uid contact flips (either direction) under tern ≤ 10 of 74 — i.e. within ~1.5× the w3 census's 1 mm-IC-jitter noise floor (7 contact flips/uid, CONFOUNDS row 47); nested net cost ≤ 3; tipped net change within ±5.
- **Power caveat (registered):** the raw commanded path may itself sit well below the joint-target path's w3 funnel (picked 69 / placed 66 / contact 26 / nested 16 over the same 74 uids; that path feeds recorded joint targets, not the twist). If raw picks < 50 the deeper stages have little power; the comparison of record stays raw-vs-code on this path, and the joint-path funnel is context only.

**Decision rules.** P1 met → the ternary code is faithful enough to define a discrete-action arm; vocabulary = the 16 words that carry 99.1 % (+ the 3-valued gripper) — report whether tern_rawgrip and raw_terngrip attribute any loss to the arm or the gripper half. P1 failed but P2 met → the analog easing matters physically; the five-level code is the candidate (report its used vocabulary). Both failed → snapping is not faithful on this path; report whether the losses are timing (first-stage frame indices shift) or spatial (settled can position) before proposing anything. P4 vs P1: if held loses more than tern, the jitter is load-bearing (do not de-jitter the tapes); if held loses less, de-jittering is free.

## 3. Results (2026-09-07 00:29; 48 jobs, 8 × 1-thread processes, all rc=0; numbers from `table.md`)

Funnel over the 74 success-labelled uids (counts; `placed` = honest shifted band; `nested` = eval rule of record after 100 settle steps; `tipped` = settled tilt > 60°; `tilt>60 any` = any frame; arm L1 = mean per-axis snapping error as a fraction of the cap; words/uid = median distinct arm words per tape):

| code | n | picked | placed | set-down | contact | nested | tipped | tilt>60 any | arm L1 | words/uid |
|---|---|---|---|---|---|---|---|---|---|---|
| raw | 74 | 17 | 12 | 4 | 2 | 1 | 21 | 23 | 0.000 | nan |
| tern | 74 | 23 | 17 | 6 | 6 | 2 | 24 | 29 | 0.014 | 14 |
| five | 74 | 19 | 14 | 5 | 3 | 1 | 23 | 24 | 0.008 | 33 |
| held | 74 | 24 | 17 | 6 | 6 | 1 | 26 | 28 | 0.020 | 13 |
| tern_rawgrip | 74 | 23 | 17 | 6 | 4 | 1 | 22 | 26 | 0.014 | 14 |
| raw_terngrip | 74 | 19 | 13 | 4 | 3 | 1 | 20 | 22 | 0.000 | nan |

**Registered predictions — all MET, with the power caveat FIRED.** "Cost" = raw − code:

| prediction | picked | contact | verdict |
|---|---|---|---|
| P1 tern ≤ 3 / ≤ 3 | −6 (17→23) | −4 (2→6) | MET (a gain) |
| P2 five ≤ 1 / ≤ 1 | −2 (17→19) | −1 (2→3) | MET (a gain) |
| P3 raw_terngrip ≤ 1 / ≤ 1 | −2 (17→19) | −1 (2→3) | MET (a gain) |
| P4 held ≤ 3 / ≤ 3 | −7 (17→24) | −4 (2→6) | MET (a gain) |
| P5 tern per-uid contact flips ≤ 10; nested cost ≤ 3; tipped within ±5 | flips 6 (1 lost, 5 gained); nested +1 (1→2); tipped +3 (21→24) | | MET |
| power caveat: raw picks < 50 | raw picked **17/74** | | FIRED |

**Reading.** (1) No code loses net stages on this path: every snapped code matches or exceeds raw at every rung (tern: picked +6, placed +5, set-down +2, contact +4, nested +1; tipped +3, inside the ±5 noise band). Per-uid the flips are two-way (tern: 3 picks lost / 9 gained; 1 contact lost / 5 gained), the first-pick frame among same-stage uids moves by a median −2 frames (IQR −3..0; bang-bang pushes travel slightly faster than eased ones), and the settled can xy among same-stage uids moves by a median 3.1 cm, p90 32 cm — the post-release phase is chaotic, as in the w3 census noise floor (CONFOUNDS row 47). (2) Five-level is outcome-closest to raw (10/74 uids change stage, first-pick shift median 0, settled-position shift median 1.0 cm, p90 15 cm): it is nearly a bit-faithful copy of raw, and buys nothing over ternary. (3) The gripper ternary ramp is lossless in practice: raw_terngrip vs raw flips 2/74 uids (both up), first-pick shift 0, settled shift median 0.3 cm; the ramp tracks the recorded position within 2.2 units median / 4.1 max. tern_rawgrip vs tern isolates the arm code: same picked/placed (23/17), contact 4 vs 6 — the gripper half adds nothing systematic. (4) Held-word (de-jittered) is not worse than ternary (24/17/6/1 vs 23/17/6/2): the step-to-step jitter is not load-bearing; de-jittering is free (but non-causal as implemented).

**The registered power caveat fired, and its cause is diagnosed (offline, no sim).** The raw commanded path reproduces picked 17 / placed 12 / contact 2 / nested 1 in w3 against the joint-target path's 69 / 66 / 26 / 16 on the same 74 uids (recorder-path census, context only). Integrating the recorded twist at the tape's own dt and comparing with the bag's `tool_pose`: by the frame of the first gripper close the integrated command has drifted from the real tool position by a **median 4.2 cm (p25 2.0, p75 11.2, p90 23.8 cm)**; at the tape end median 12.9 cm. Raw-picked uids have median 2.6 cm drift at close (p90 5.3), the 57 non-picked 6.2 cm (p90 27). Only 19/74 tapes are within 2 cm at close (32 within 3 cm). I.e. the real arm did **not** realize the commanded twist 1:1 (the plugin's tracking, workspace clamp and command timing are not in the tape), so open-loop integration of the joystick command misses the can in most demos regardless of snapping. The 07-24 "uid 232 picks+contacts" validation was one favourable uid in the base world. This is a property of the commanded path, not of the codes — and it means the raw joystick twist is a poor *action label* for the demonstrations in absolute terms even before discretisation. Disclosed post-hoc context (not registered, 02:09–02:20): the raw code re-run in the `base` world on shards 0–1 (20 uids, `ctx_base/`): base picked 4 / placed 3 / contact 0 / nested 1 vs w3 on the same 20 uids 5 / 3 / 0 / 0 — the weak commanded path is world-independent (uid 232, the 07-24 validation uid, does reach nested in base and only placed in w3; 254 picks in both; the other 17 never pick in either).

Deviations / disclosures: the harness has no gripper bit, the script builds the ternary ramp (§1); held-word uses a ±2-frame window (non-causal); the env's own `placed` band is unshifted and never fires in shelf6 (`placed_env` 0 everywhere; the honest shifted band is reported); a transient pytest from another project (`fightspoon/wargame`) ran on the box for ~1 min during the last wave — irrelevant to single-thread deterministic physics (bit-exactness vs the default-thread run verified on the smoke uid); one deterministic pass per cell, no repeats.

## 4. Per-uid flips (stage ladder none < picked < placed < contact < nested; lists from `table.md`)

Raw stages: nested [259]; contact [308]; placed [232, 235, 251, 257, 261, 265, 266, 309, 321, 331]; picked [243, 248, 254, 258, 302]; the other 57 never pick.

**tern vs raw** (n=74): stage UP 12 ['232:placed->contact', '252:none->placed', '255:none->picked', '258:picked->contact', '265:placed->contact', '275:none->placed', '295:none->contact', '297:none->nested', '298:none->contact', '301:none->picked', '305:none->picked', '317:none->placed']; DOWN 3 ['254:picked->none', '266:placed->none', '331:placed->none']
- picked: lost 3 [254, 266, 331]  gained 9 [252, 255, 275, 295, 297, 298, 301, 305, 317]
- placed: lost 2 [266, 331]  gained 7 [252, 258, 275, 295, 297, 298, 317]
- setdown: lost 1 [266]  gained 3 [252, 297, 317]
- contact: lost 1 [259]  gained 5 [232, 258, 265, 295, 298]
- nested: lost 0 []  gained 1 [297]
- tipped: lost 7 [235, 236, 242, 252, 254, 273, 286]  gained 10 [233, 262, 265, 266, 295, 298, 299, 301, 304, 316]

**five vs raw** (n=74): stage UP 6 ['254:picked->placed', '273:none->picked', '298:none->contact', '306:none->contact', '316:none->picked', '317:none->placed']; DOWN 4 ['243:picked->none', '257:placed->none', '308:contact->placed', '331:placed->none']
- picked: lost 3 [243, 257, 331]  gained 5 [273, 298, 306, 316, 317]
- placed: lost 2 [257, 331]  gained 4 [254, 298, 306, 317]
- setdown: lost 0 []  gained 1 [321]
- contact: lost 1 [308]  gained 2 [298, 306]
- nested: lost 0 []  gained 0 []
- tipped: lost 6 [236, 242, 252, 257, 273, 286]  gained 8 [243, 256, 262, 274, 277, 298, 317, 335]

**held vs raw** (n=74): stage UP 13 ['243:picked->placed', '252:none->placed', '255:none->picked', '258:picked->placed', '261:placed->contact', '273:none->picked', '275:none->placed', '297:none->contact', '298:none->contact', '301:none->picked', '305:none->picked', '317:none->contact', '325:none->picked']; DOWN 3 ['254:picked->none', '266:placed->none', '331:placed->none']
- picked: lost 3 [254, 266, 331]  gained 10 [252, 255, 273, 275, 297, 298, 301, 305, 317, 325]
- placed: lost 2 [266, 331]  gained 7 [243, 252, 258, 275, 297, 298, 317]
- setdown: lost 1 [266]  gained 3 [252, 275, 321]
- contact: lost 0 []  gained 4 [261, 297, 298, 317]
- nested: lost 0 []  gained 0 []
- tipped: lost 6 [236, 242, 252, 254, 286, 321]  gained 11 [262, 266, 274, 297, 298, 300, 301, 304, 305, 317, 319]

**tern_rawgrip vs raw** (n=74): stage UP 10 ['252:none->placed', '255:none->picked', '258:picked->placed', '275:none->placed', '295:none->contact', '297:none->placed', '298:none->contact', '301:none->picked', '305:none->picked', '317:none->placed']; DOWN 3 ['254:picked->none', '266:placed->none', '331:placed->none']
- picked: lost 3 [254, 266, 331]  gained 9 [252, 255, 275, 295, 297, 298, 301, 305, 317]
- placed: lost 2 [266, 331]  gained 7 [252, 258, 275, 295, 297, 298, 317]
- setdown: lost 1 [266]  gained 3 [252, 258, 317]
- contact: lost 0 []  gained 2 [295, 298]
- nested: lost 0 []  gained 0 []
- tipped: lost 8 [236, 242, 252, 254, 258, 286, 306, 309]  gained 9 [262, 265, 266, 275, 295, 297, 298, 301, 316]

**raw_terngrip vs raw** (n=74): stage UP 2 ['298:none->contact', '316:none->picked']; DOWN 0 []
- picked: lost 0 []  gained 2 [298, 316]
- placed: lost 0 []  gained 1 [298]
- setdown: lost 0 []  gained 0 []
- contact: lost 0 []  gained 1 [298]
- nested: lost 0 []  gained 0 []
- tipped: lost 2 [242, 329]  gained 1 [298]

Timing vs spatial (`diag`): among the flipped uids the first-pick frame, where both codes pick, is within ±10 frames (e.g. 258: 421 → 411); the flips are spatial — a few cm of integrated-path difference at the grasp decides the pick (254/266/331 lost under every arm code; 252/255/275/297/298/301/305/317 gained under both tern and held), and once the can is released the settled position is chaotic (same uid: 18 → 88 cm for 331 under tern).

## 5. Recommendation (10 lines)

1. **Snapping is not the bottleneck.** On the commanded path a discrete code costs nothing: ternary arm + ternary gripper matches or beats the raw twist at every stage (P1–P5 all met, every "cost" negative); five-level is outcome-closest to raw but buys nothing; held-word shows the jitter is not load-bearing.
2. **A ternary code is faithful enough to define a discrete-action arm**, and the gripper is naturally three-valued too ({open, hold, close} driving a 1.5 unit/frame ramp — lossless here).
3. **Vocabulary:** 45 of 81 arm words occur; **16 words carry 99.1 %** (34 for 99.9 %; 8 for 77.9 %); per tape median 13. With the gripper: a 16 × 3 = **48-word codebook** (or the full 81 × 3 = 243 product for a categorical actor; the unused words simply never appear in the data).
4. **But the raw joystick twist is a poor action label in absolute terms:** open-loop, it reproduces only 17/74 picks in w3 (joint path 69/74) because the real arm realised it with a median 4.2 cm drift by grasp time. A discrete arm built by snapping the *recorded joystick* would inherit this drift.
5. **Faithful construction for the arm:** snap the *realised* Cartesian velocity of the tapes of record (the joint-path replay's per-frame tool velocity — `baselines/derive_cartesian_realized.py` already derives it — quantised to the same 16(+gripper) words), then repeat this replay check on those words before building a learner (same script, ~1 h on 8 processes). If the realised-velocity ternary tape reproduces the joint-path funnel to within the census noise (picked ≥ 66, contact ≥ 21), the discrete arm is defined; otherwise the word rate (40 Hz) or the cap needs a registered change.
6. **Do not** build the discrete arm on `_cartesian.npy` twist directly; **do not** read the +6 pick gain as "discrete is better" — it is within the two-way flip structure of a path that misses most grasps.
7. Nothing blocked a code; the held-word row was cheap and ran; no learner was built or trained; frozen sets, `matched_w3`, `demos_v2` and the cluster untouched.
