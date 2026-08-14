# Fable handoff — 2026-08-13 (Opus session → Fable tonight)

Author: Opus 4.8 session (local box). Written in Simplified Technical English
(short sentences, active voice, one instruction per line) because this doc
coordinates agents.

Purpose of this doc:
1. Let Fable CHECK this session's work and results.
2. Coordinate the implementation changes to the three learner arms: dv3, RLPD,
   r2dreamer.
3. Initiate testing of everything.

Read alongside: PAPER_PLAN.md (living matrix), SESSION_STATE_2026-08-13.md
(prior handoff), dreamerv3-torch/MANISKILL_VS_GENESIS.md,
paper/rlpd_literature_comparison_2026-08-13.md.

---

## 1. What this session established (the load-bearing findings)

1. **dv3 (dreamerv3-torch) has NEVER produced a working policy.** Every apparent
   success (v1 20% train, v2 0.20 eval, v4 30% train) was a measurement artifact
   caught by paired controls. dv3 is a rigorously instrumented null.

2. **dv3 was never run with the recipe that made r2dreamer work.** dv3 has only
   generic `action_repeat`. It lacks delta-joint actions, short horizon, and the
   single terminal reward. The fix list is dreamerv3-torch/MANISKILL_VS_GENESIS.md
   "Top-3 concrete changes." Those changes were "never tried in dv3-torch."

3. **CORRECTION to earlier notes: `bounded_normal` is NOT a dv3 gap.** dv3 is
   already clean: `ContDist(absmax=1.0)` at models.py:278 bounds its actor samples.
   `bounded_normal` was an r2dreamer-only bug fix. Do NOT list it as a dv3 change.

4. **The ManiSkill audit and the RLPD literature doc INDEPENDENTLY blame the same
   two things: reward density and time-to-reward/horizon.** Same stock DreamerV3
   recipe solves ManiSkill PickCube and fails on genesis. The difference is task
   shaping, not architecture. This is a strong convergent methods story for the
   paper.

5. **RLPD ignites — the machinery is the claim; 0.40 is a SELECTED MAX, not a
   confirmed row number.** (Corrected by newbox_supp from primary evidence; I
   re-verified by diffing the confirm-log episodes.)
   - Stride-1: **6/7 seeds ignited vs SACfD 0/16.** THIS is the surviving claim —
     ignition machinery, not a performance number.
   - Seed 0 / 150k / demo-IC: **6/15 = 0.40 ± 0.13** (1 SD binomial). This is ONE
     deterministic measurement. The es1/es2/es3 "x3" is a NULL operation: demo-IC
     fixes the ICs and the SAC eval policy is deterministic, so all three return the
     byte-identical pattern `P.P...P..P..P.P`. Do NOT report "confirmed across 3
     eval seeds."
   - Random-IC: **0.156** (0.20/0.20/0.07 = 3/15,3/15,1/15). This arm IS stochastic,
     so its 3 eval seeds are genuine. Supersedes the ~0.30 snapshot in
     SESSION_STATE_2026-08-13. Gen gap 0.40 -> 0.156.
   - SELECTION CAVEAT: 0.40 = best checkpoint (150k of 4) of the best seed (s0 of 7),
     chosen after seeing evals. To make it reportable: fix the checkpoint by a
     held-out criterion BEFORE eval, or use n=45 stochastic samples (r2dreamer
     protocol).
   - PROTOCOL now in PAPER_PLAN: on demo-IC evals, "x3" must mean 3 TRAINING seeds or
     3 stochastic-policy samples. Repeating a deterministic rollout adds no
     information (same class as the poller-filter and syncer false alarms).
   - The RLPD row is SINGLE-SOURCE (dH only) — not yet a human-vs-model comparison.
     The dDP_RLPD twin is unbuilt.

6. **repeat-n demo subsampling is safe for the pick phase from N=1 to N=8.** The
   gate passed 5/5 at every N, with FLAT lift margin (no corner-cutting even at
   4 Hz). The all-phase check across all 91 demos is RUNNING (see §6); results
   append to this doc when the aggregator finishes.

7. **Design law adopted this session (do NOT reverse without the user):** do NOT
   pre-bake repeat-n into demo datasets. Baking creates a dataset-configuration
   dependency and a silent-mismatch surface (baked-at-6 demos + env-at-4 = dead
   policy, no error). Keep the subsampling LIVE via
   `train_sacfd_full.delta_encode_transitions_repeat` (the one encoder) + the env's
   `action_repeat`. If a file loader ever needs baked demos (dv3), stamp `repeat`
   in metadata and ASSERT `env.action_repeat == dataset.repeat` at load.

---

## 2. Verify my work (checklist for Fable)

- [ ] `git diff` on baselines/rl/sacfd_delta_gate.py. I added `all_demos_sweep()` +
      `--all-demos/--shard-idx/--shard-n`. Confirm the sweep reads the env's OWN
      stage (`env._granted`), not a reimplemented ladder. Confirm `_encode()` is the
      shared encoder (one source of truth).
- [ ] Re-run the pick gate to reconfirm the pipeline: `.venv-eval/bin/python
      baselines/rl/sacfd_delta_gate.py --action-repeat 4` → expect GATE PASS 5/5.
- [ ] RLPD numbers: demo-IC 0.40 +- 0.13 (one deterministic measurement, NOT "3
      seeds"); random-IC 0.156. See finding #5 for the selection + protocol caveats.
      Do not restate the deterministic "x3" as replication.
- [ ] Confirm the phase-sweep result (this doc, §7 once appended). The honest claim
      is repeat-1 vs repeat-N under the SAME env measurement. Do not compare to the
      recorded d['stage'] label (different predicate).
- [ ] COMMIT/PUSH STATE — VERIFY FIRST, there is a discrepancy:
      - `git log origin/4dof-cartesian..HEAD` is EMPTY (2026-08-13) — HEAD is NOT
        ahead of origin. So 587990f (RLPD impl) + 32c1e8e (ar4) may ALREADY be on
        origin, OR the local origin ref is stale. Run `git fetch` and re-check before
        assuming they are unpushed. Do not blindly push.
      - UNCOMMITTED working-tree edits from this session (need commits):
        * genesis_pickaplace: `baselines/rl/sacfd_delta_gate.py` (M, all_demos sweep),
          `FABLE_HANDOFF_2026-08-13.md` (new).
        * dreamerv3-torch: `sbatch_genesis_multi.sh` (M, ACTREP knob).
      - newbox_supp has local commit 825c1e9 (PAPER_PLAN.md, P1 caveat) — unpushed.
      - Coordinate the push with newbox_supp; it also edits PAPER_PLAN.md.

---

## 3. The three-arm change plan

Common root cause (both diagnostic docs): the task reward is too sparse and too far
in discounted time. The fix family: shorter horizon (fewer decisions to reward),
terminate on success, denser or larger terminal reward, self-correcting
position-delta actions. r2dreamer already has these and ignites. RLPD and dv3 do not
yet have the full set.

### 3a. RLPD  (model-free, WORKS — extend it)
- DONE: action-repeat-4 (dH_RLPD-ar4_s0..s5, all finished). Horizon half of the fix.
- RUNNING (2026-08-13, USER CHOSE repeat-8 for the initial pick test): dH_RLPD-ar8
  _s0..s5, 6 seeds, pick scope, delta_joint, 100k decisions, local box. This is the
  strongest horizon-compression lever (~75 decisions to pick vs ManiSkill ~100).
  Rationale: pick scope makes the P1 downstream-fidelity problem irrelevant, so the
  most aggressive stride is the cleanest test of "is horizon compression worth it".
  READ THIS FIRST when online. Build the ar8-vs-ar4-vs-stride-1 ignition/pick table.
- dDP_RLPD-ar8 twin (newbox_supp): **BLOCKED ON DATA — near-miss caught 2026-08-13.**
  My instruction named `episodes_pick_phase_dppruned` as the model-demo set. WRONG:
  `dppruned` = DP-PRUNED HUMAN demos (idle-frame collapse, the DP-arm preprocessing),
  uids 232+, label=success. NO model-harvest demos exist on this box — every
  episodes_* dir is human-uid-keyed; a real harvest carries rollout indices. The twin
  needs `genesis_m1all` rsynced from the cluster (user-owned; PAPER_PLAN 2026-08-09)
  or a fresh local harvest from dp_pick_pruned weights (different harvest than the
  dDP_DP row used — worse). newbox_supp stopped before launching; had it run, the
  "human vs model" row would have been human-vs-human with plausible curves and no
  downstream check to catch it. Same silent-default family as AUDIT_REQUEST_Fable.
  Runs on THIS box once data exists → pair can be official (same machine).
- NOT YET TRIED: reward-density half. Two options, literature-supported:
  1. Per-step hold reward over the whole grasped-hold region (not one terminal +1).
  2. Keep demos un-truncated through the sustained hold (they truncate ~2 frames
     past the pick now, so the buffer barely contains the success state).
- Owner call: after the ar8 readout, re-run RLPD at the chosen N with the
  reward-density change. Keep ≥3 seeds, negative control.

### 3b. dv3  (world model, NULL — give it the recipe it never got)
- Port the MANISKILL_VS_GENESIS.md "Top-3": delta position-target actions +
  short horizon + terminate-on-success + a single sparse terminal reward
  (relabel demo picks to a large terminal, like dreamer.py already does for
  maniskill/pusht). The adapters exist in r2dreamer to copy.
- Do NOT add `bounded_normal` (dv3 is already clean — finding #3).
- Do NOT add actor-BC (user rule: no actor-BC in the world-model arm; demos are
  dynamics/reward data only).
- Outcome is scientifically clean either way:
  - dv3 ignites → the action geometry / horizon was the blocker, not reconstruction.
  - dv3 stays null → the decoder-free representation (r2dreamer) was the deciding
    factor. That is a strong methods claim.
- This is real implementation (~half day). Route to Opus, not Fable, per the
  project's own delegation pattern (implementation → Opus; Fable orchestrates).
- SETUP DONE (2026-08-13): sbatch_genesis_multi.sh now has an ACTREP knob
  (--action_repeat) + SCOPE=pick. Submit shape:
    RUNS="TAG=dH_ar8 SCOPE=pick ACTREP=8 DEMODIR=<stride8_pick_demos> SEED=0 STEPS=1e6"
- BLOCKER (why dv3 repeat-8 is NOT a config flip): dv3 does NOT downsample demos for
  action_repeat and loads demos from FILES. Its pick demos are STRIDE-1 (and carry
  IMAGES — dv3 is the pixel arm). ACTREP=8 with stride-1 demos = demo/online
  time-base MISMATCH (the silent bug). A valid dv3 repeat-8 run needs a stride-8
  IMAGE demo set first (genesis_pick_pruned_delta25 -> stride-8, stamped). Generating
  it is part of THIS recipe port. Until it exists, do not submit ACTREP>1 for dv3.

### 3c. r2dreamer  (world model, WORKS — but unstable)
- Champion exists: 0.91 sampled / 1.00 mode
  (r2dreamer/runs/pick_delta25d4_s0/CHAMPION_1576820.pt).
- The remaining problem is STABILITY, not ignition. The arm is bistable
  (~1-in-7 lottery per checkpoint).
- We do NOT have a fix. The clamp hypothesis (return_clamp=100) was tested and
  REFUTED — it removed target overshoot but entropy-collapse cycles persisted.
  Cause still open (AMP inf-grads / reinject shocks are the live suspects).
- This is a DIAGNOSIS task, not an implementation task. Do not "implement" a fix
  blindly. Current handle is measurement-side: best-checkpoint + independent
  confirmation. That is honest but not a training fix.
- Cheap firming experiment (no new code): one more matched cluster wave, dH and dDP
  seeds 20-29, to firm the dH 3/10 vs dDP 0/10 ignition contrast (p≈0.10 now).

---

## 4. Testing plan (initiate everything)

Order matters — cheapest and most-informative first.

1. **Phase sweep (DONE — see §7 + P1 in §8).** Pick/place robust to N=8; contact/
   nested inconclusive and flagged as P1. Pick-scope experiments unaffected.
2. **RLPD ar8 + ar4 readout (ar8 RUNNING, ar4 DONE).** When ar8 s0..s5 finish, build
   the ar8-vs-ar4-vs-stride-1 table: ignition rate, peak in-dist picked, decisions-
   to-pick. Does repeat-8 raise/stabilize picks vs repeat-4? Send eval videos to user.
   HAZARD (newbox_supp, will bite this table): **count picks from the LOGS, never
   from eval video files.** Video dirs ACCUMULATE across runs — filenames encode the
   outcome (ep4_picked vs ep4_fail), so a re-eval writes a NEW file instead of
   overwriting. Counting *_picked.mp4 read random-IC 0.07 as 0.47 (6x overstatement).
   Demo-IC is immune (identical outcomes -> identical filenames -> real overwrite).
   Proper fix later: stamp eval-seed/run-id into the video path.
3. **dv3 recipe port (NEW, Opus).** Implement §3b. Gate on the ManiSkill fingerprint:
   `train/action_entropy` MUST fall (it stays flat-high 8-9.5 nats in the failing
   runs). If entropy falls and picks appear, the recipe transferred.
4. **RLPD reward-density (NEW, Opus).** Implement §3a option 1 or 2 at the swept N.
5. **r2dreamer firming wave (cluster).** Matched dH/dDP seeds 20-29.

Every trained policy → eval videos to the user. Every reported number → ≥3 seeds,
negative control, pre-registered criterion, same-machine baselines.

---

## 4b. Sidecar state (2026-08-13 late)

- train_rlpd now writes the action-mode sidecar next to EVERY snapshot (authored at
  save time from live config — commit 51b4e27). Before that, only rlpd_final got one;
  snapshot evals fell back to absolute@1 (third silent-default instance; newbox_supp
  caught it before it zeroed the 100k post-hoc sweep).
- The 50 pre-existing snapshots got BACKFILLED sidecars, inferred from the directory
  name. Each is stamped {"backfilled": true, "backfill_source": "dir-name-inference
  2026-08-13"} so reconstructions are distinguishable from authored records. Treat a
  backfilled sidecar as a claim, not a fact — if a dir was ever misnamed, the
  reconstruction silently wins (same inference-from-name shape as the dppruned
  near-miss). Authored (no flag) = trustworthy.
- The ar8 runs hold pre-fix code in memory: their final snapshots land sidecar-less.
  Re-run the backfill once they finish (same stamp).

## 5. Running jobs + how to monitor

- **RLPD ar8 (RUNNING — read first):** dH_RLPD-ar8_s0..s5 (wandb genesis_paper),
  repeat-8, pick scope, 100k decisions, local box. Logs scratchpad/rlpd_ar8_s{0..5}
  .log. Checkpoints baselines/rl/checkpoints/rlpd_ar8_dH_s{0..5}/. Eval every 12.5k
  decisions. NB count picks from LOGS not video files (§4 hazard).
- **RLPD ar4 (DONE):** dH_RLPD-ar4_s0..s5 all finished at 100k. Logs
  scratchpad/rlpd_ar4_s{0..5}.log; checkpoints rlpd_ar4_dH_s{0..5}/. s1 hit 0.10 at
  50k; final snapshots low — needs the best-checkpoint readout, not final-only.
- **Phase sweep (DONE):** logs scratchpad/phase_sweep_r{1,4,6,8}_s{0..5}.log;
  aggregator scratchpad/aggregate_phase_sweep.py (re-run any time). Results in §7/§8.
- **dDP_RLPD-ar8 twin:** owned by newbox_supp (may be on another machine).
- **Cluster:** short-wave stragglers + dv3 matrix-row jobs (see PAPER_PLAN). Poll
  env_step growth, NOT wandb run state (syncer marks finished every cycle).

Monitoring rule (hard-won): the monitoring layer fails more often than the runs.
Verify poller filters against LIVE run names. Read primary evidence (.out,
metrics.jsonl, .log) before any urgent claim.

---

## 6. Coordination + standing rules

- **Second session `newbox_supp`** is live on this box in parallel. Proposed split
  (sent, but the cross-session channel was not reachable via SendMessage — user to
  relay): newbox_supp takes the RLPD s0 confirm writeup + videos, and the PAPER_PLAN
  matrix/decision-log update. I keep the phase sweep + this handoff. It agreed not to
  push or touch my checkpoints/logs until we split.
- **Do not conda-install into the genesis env.** Datasets travel by rsync ONLY,
  never git.
- **No actor-BC in the world-model arm** (user rule).
- **Cluster paths:** /cluster/tufts/shortlab/jstale02/ (never `~`).
- **Full autonomy** except destructive actions. Every policy → videos to user.

---

## 8. P1 — RESOLVED 2026-08-13 (diagnosis complete; fixes staged)

**VERDICT (full report: paper/p1_delta_divergence_2026-08-13.md, commit 3a7a713):**
Of the 8 nested-labeled demos that lost the pick under delta replay:
- **6 = FROZEN TARGET DRIFT (root cause).** The delta encoder encodes command
  DIFFERENCES, so one cap-clipped frame leaves a PERMANENT offset in the integrated
  target — open-loop delta replay never re-converges (uid 328: offset froze at
  0.2260 rad, std 0.00000 over the last quarter). Velocity replay re-sends ABSOLUTE
  commands and self-heals; that is the structural delta-vs-velocity difference.
  Drift begins in FREE SPACE (can error exactly 0.0mm for hundreds of steps) — not
  contact chaos. An offline encoder+leash predictor separates losers from keepers
  (0.060 vs 0.045 rad at grasp, p=0.0010).
- **2 = a #26 REGRESSION in full_env.py** (inner GenesisCanEnv left at default
  max_steps=1200 → 100 phantom sim steps per call past 1200; the collector fixed
  this 07-20, FullTaskEnv never got it; 54/72 sweep replays crossed 1200).
  **FIXED + pushed (3a7a713).** Training was unaffected (outer horizon 900 < 1200);
  replays and any eval past 1200 inner steps were not.
- **0 = truncation.** Max pick frame 3693 < 4000.
- Cap saturation stays refuted (the trace agent found a discriminator, then
  withdrew it as length-confounded — documented in the report).
- Manufactured successes have three sources: nested-proxy transients (305),
  phantom-settle perturbation (274, 316 — gone post-fix), lucky drift (261).

**FIXES, ordered:** (1) inner-horizon fix — DONE; (2) closed-loop re-record of
demos in the delta env (the only cure for frozen offset; derive_cartesian_realized
is the template); (3) delta-for-pick-only + velocity downstream = valid interim;
larger cap/leash = partial at best. Never report proxy-derived nested.

**STILL TODO:** §7 phase-sweep re-run post-fix (54/72 replays were contaminated);
re-run when the eval sweep frees the cores. Original problem statement follows.

### (original P1 statement, kept for the record)

**User flagged this as a problem (2026-08-13). It is bigger than the repeat-n question.**

FACT: the demo LABELS (from the CANONICAL velocity replay, collect_all_classified.py
through GenesisCanEnv) say contact 9 / nested 22. The delta_joint open-loop replay
through FullTaskEnv reproduces only contact 5 / nested ~1-4 at repeat-1. The
delta representation loses the demonstrated downstream outcomes.

LOCALIZATION (from the repeat-1 sweep log; nested-labeled demos, 22 total):
- 6 UNRESETTABLE (not in the placements map — separate issue, see below).
- **8 LOSE THE PICK entirely** (delta replay grants no-pick).
- 2 stop at picked, 5 reach placed, 1 reaches nested.
So the dominant failure is NOT a downstream-predicate mismatch — it is that the
delta replay fails to reproduce even the PICK on HALF the resettable nested demos.

HYPOTHESIS TESTS DONE:
- Cap saturation REFUTED: pick-losers and pick-keepers both peak at ~2x delta_cap
  and exceed cap on only ~0.4-2% of frames — statistically identical. Not the cause.
- Length correlates: pick-losers median ~2600 frames vs pick-keepers ~1480. The long,
  complex (multi-attempt / drag / regrasp) demos are the ones that diverge.

LEADING DIAGNOSIS: delta_joint open-loop integration accumulates tracking divergence
over long trajectories, so the arm drifts off the demonstrated path before the grasp.
The canonical VELOCITY replay does not have this problem (it produced the 22 nested
labels). This is a delta-vs-velocity FIDELITY cost, echoing the historical #26
env-vs-replay divergence theme.

CROSS-CHECK (newbox_supp, 2026-08-13) — the disagreement runs in BOTH directions:
- Only 1 of the 22 nested-LABELLED demos reaches nested under delta replay, yet the
  repeat-1 baseline reports 4 env-measured nested (over 72). So ~3 of those 4 come
  from demos the canonical velocity replay did NOT label nested. The measurement
  MANUFACTURES successes as well as losing them.
- Combined with the 11-15 demos that GAIN a stage under subsampling, the downstream
  measurement is high-variance in BOTH directions on a 4-5 demo baseline, not merely
  lossy. Pure integration drift predicts loss ONLY, so drift is necessary but NOT
  sufficient — a second effect (predicate flakiness at the placed-band / touch /
  nested-proxy boundaries) is also in play.
- During diagnostic #1, if the divergence trace on 300/328 shows clean drift, the
  GAINED-stage demos are the ones that discriminate the second effect.
RULE (adopted): report NO contact/nested number from delta open-loop replay until
fidelity is resolved — including numbers that would flatter us. Pick baseline (34) is
trustworthy; downstream (4-5) is not.

WHY IT MATTERS: all three arms train on delta_joint actions. If the demo buffer cannot
reproduce the downstream phases in the delta representation, the full-task downstream
reward/dynamics signal from demos is corrupted. The pick phase is fine (that is the
current scope). The FULL-TASK extension is blocked until this is understood.

CAVEAT ON EARLIER RESULTS: the pick-gate "5/5" and "repeat 1..8 safe" used the 5
GENTLEST hand-picked pick demos. They do NOT generalize to the full population's
downstream phases. Do not cite them as full-task evidence.

NEXT DIAGNOSTIC (Fable, GPU-free, ~1-2h):
1. Per-demo divergence trace: for a lost-pick nested demo (e.g. 300, 328), log the
   delta-replay arm qpos vs the demo command each step; find WHERE it first diverges.
2. Compare delta-replay vs velocity-replay of the SAME demo side by side (velocity =
   collect_all_classified path). Confirm velocity keeps the pick where delta loses it.
3. If divergence is the cause, options: (a) higher delta_cap / longer leash for the
   full task; (b) closed-loop demo re-recording in the delta env (record what the
   delta env actually executes, like derive_cartesian_realized did for cartesian);
   (c) accept delta for pick-scope only and use velocity/abs for downstream.
4. SEPARATELY: 19/91 demos are UNRESETTABLE (not in the env placements map:
   233,240,255,259,262,266,267,268,275,278,282,288,301,303,319,321,324,329,333).
   6 of them are nested-labeled. The placements map needs these added or a reason why.

---

## 7. Phase-sweep results (DONE 2026-08-13)

All 91 demos, delta-joint open-loop replay, env's OWN stage grants. 72 resettable
(19 not in the placements map). Baseline = repeat-1 under the SAME measurement.

BASELINE (repeat-1) env-measured reach: picked 34, placed 31, contact 5, nested 4.
NOTE: the demo LABELS say placed 43 / contact 9 / nested 22. So the delta-joint
open-loop replay ALREADY drops most downstream phases at repeat-1 — the replay is
lossy on the delicate phases regardless of subsampling. Contact/nested baselines are
tiny (5 and 4 demos).

Preservation vs repeat-1:
| N | picked | placed | contact | nested |
|---|--------|--------|---------|--------|
| 4 | 32/34 (94%) | 30/31 (97%) | 2/5 (40%) | 2/4 (50%) |
| 6 | 30/34 (88%) | 28/31 (90%) | 4/5 (80%) | 3/4 (75%) |
| 8 | 29/34 (85%) | 27/31 (87%) | 2/5 (40%) | 2/4 (50%) |

Also 11-15 demos GAINED a stage vs repeat-1 at each N (subsampling should not
improve outcomes) -> the downstream measurement is high-variance.

READING:
- **Pick and place survive subsampling well through N=8** (85-97%; large baseline,
  trustworthy). Degradation is graceful and some of the "loss" is measurement noise
  (many demos also gain).
- **Contact and nested are INCONCLUSIVE from this test.** Baseline is 4-5 demos AND
  open-loop replay drops these phases even at N=1. The numbers bounce
  non-monotonically (N=6 > N=4) = noise, not signal. Do NOT report a contact/nested
  safe-N from this.

DECISION: **repeat-6 is the recommended default for the PICK phase** (ManiSkill
parity ~100 decisions-to-pick; 88% pick / 90% place preservation). This matches the
current experimental scope (all three arms are scope=pick). To pin N for the full
task's delicate phases later, we need a higher-fidelity test (closed-loop, or a
better replay) — the open-loop delta_joint replay cannot answer it.
