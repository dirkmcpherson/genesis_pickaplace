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
   - Stride-1: ~~"6/7 seeds ignited"~~ **DEFINITION-INFLATED (newbox_supp post-hoc
     analysis, 2026-08-13 night; Fisher + binomial verified by Fable).** "Ignited" =
     nonzero at ANY snapshot = multiple testing: stride-1 seeds were scanned over
     ~8 snapshots x n=10 = ~80 eps, where a TRUE pick rate of 0.03 (indistinguishable
     from broken) registers "ignited" 91% of the time (37% at one fixed n=15
     endpoint). Post-hoc at fixed 100k, one protocol: stride-1 3/7 seeds >=1 pick
     (4/105 eps), ar4 2/6 (2/90), Fisher p=0.69 — the arms are indistinguishable and
     both near the floor at 100k. At n=15, >=3 picks is the first
     not-explainable-by-chance level; NO seed in either arm reaches it at 100k.
   - **RESTATED CLAIM (use this):** "6/7 seeds produced at least one pick at some
     checkpoint; at a fixed 150k checkpoint the best seed reaches 0.40 (6/15, well
     clear of the noise floor); at 100k no seed exceeds 2/15." The RLPD-vs-SACfD
     contrast SURVIVES — SACfD = literally zero picks across 16 seeds and every
     snapshot, immune to the scanning artifact; RLPD produces real picks and one
     checkpoint well above floor. The "6/7" figure must NOT go in the paper as-is.
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
- **PORT COMPLETE (overnight 08-13→14, Opus agent; gates independently re-run by
  Fable: encoder PASS, reward PASS, replay 5/5 PASS).** dreamerv3-torch commits
  eed7afb/d4b9503/69b11e4, PUSHED. Demo set demonstrations/genesis_pick_msr_delta25_r4
  (67 eps, stride-4, stamped repeat.json, +100 terminal, 6700 total reward audited,
  91-demo negative control clean; 31MB, RSYNC ONLY — not in git). dreamer.py CRASHES
  on stride/terminal mismatch or unstamped dir + repeat>1; sbatch pre-checks the same.
  MSR=1 knob in sbatch_genesis_multi.sh. Full writeup: dreamerv3-torch/
  MSRECIPE_PORT_STATUS.md.
- KNOWN RISKS (agent-flagged): (1) time_limit 150 decisions = MS parity but covers
  only ~70% of the demos' own pick times (stride-4 pick decision median 127, max
  630) — if the run stalls, time_limit 900 is the FIRST knob; (2) converter refuses
  non-pick scopes by design (P1 frozen-target drift loses downstream stages).
- GATE ON THE RUN: train/actor_entropy MUST FALL (MS fingerprint: ≈−2.1 nats by
  ~17k updates ≈ 2.7e5 env steps at this train_ratio — readable inside 3e5).

**MORNING COMMANDS (user):**
```
# 1. cluster picks up the port
ssh <cluster>  &&  cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch  &&  git pull
# 2. demo set travels by rsync (NEVER git)
rsync -av ~/workspace/dreamerv3-torch/demonstrations/genesis_pick_msr_delta25_r4/ \
  <cluster>:/cluster/tufts/shortlab/jstale02/dreamerv3-torch/demonstrations/genesis_pick_msr_delta25_r4/
# 3. submit (3 seeds, one job)
REPO_DIR=$PWD WANDB=1 \
RUNS="TAG=dH_msr_ar4_s0 MSR=1 SCOPE=pick ACTREP=4 DEMODIR=genesis_pick_msr_delta25_r4 SEED=0 STEPS=3e5 | \
      TAG=dH_msr_ar4_s1 MSR=1 SCOPE=pick ACTREP=4 DEMODIR=genesis_pick_msr_delta25_r4 SEED=1 STEPS=3e5 | \
      TAG=dH_msr_ar4_s2 MSR=1 SCOPE=pick ACTREP=4 DEMODIR=genesis_pick_msr_delta25_r4 SEED=2 STEPS=3e5" \
sbatch sbatch_genesis_multi.sh
# 4. r2dreamer firming wave, dH seeds 20-29 (from genesis_pickaplace checkout root):
for S in 20 21 22 23 24 25 26 27 28 29; do
  CONFIG=genesis_pick_v5d4_delta STEPS=1e6 SEED=$S \
  LOGDIR=/cluster/tufts/shortlab/jstale02/r2dreamer/runs/dH_R2Dshort_s$S \
  sbatch --time=1-00:00:00 cluster/sbatch_r2dreamer.sh
done
# 5. dDP twin seeds 20-29: reuse your 08-12 wave-B submission line from shell
#    history (it carries DUPLICATE=7 + the m1 DEMO_DIR) with SEED=20..29 —
#    do not retype it from memory.
# 6. DECISION still open: rsync genesis_m1all to this box (unblocks dDP_RLPD +
#    dDP world-model twins — recommended) or approve a fresh local harvest.
```

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

## 4a-0. **P2 — CROSS-EPISODE STATE CONTAMINATION**

**STATUS 08-14 ~01:30 — FABLE'S PREDICTION REFUTED; PROTOCOL, NOT ENV FIX, IS THE
RESOLUTION.**

1. Controller-target fix (e9f6e24) closed ONE channel (divergence 0.0506 ->
   7.4e-06 rad at 20-step probe horizon; used-vs-used resets 1.3e-05). Fable's
   registered prediction ("no prefix flips 243 post-fix") was **REFUTED**: post-fix
   dose-response T,F,T,F,F,F,F,T over prefix lengths 1-8. History still decides
   borderline episodes. Residual channel = solver/contact state.
2. **Fable's "bounded small" claim was a probe-horizon error**: 20-step bound
   extrapolated to 400-step contact-rich episodes (20-fold gap, compounding).
   Named as such; do not repeat the pattern (a bound is only as long as its
   measurement window).
3. **The fix CHANGED THE BASELINE** (243 alone: pre-fix False -> post-fix True).
   Post-fix numbers are a NEW measurement, not corrected versions of old ones.
   NOTHING pre-fix is comparable to anything post-fix.
4. **Determinism-given-sequence means reproducibility is evidence of nothing**:
   identical sequences reproduce byte-identically; the value is one fixed
   arbitrary draw. Borderline episodes exhibit sensitive dependence — microscopic
   state resolves them arbitrarily. Repeating an eval cannot sample the
   distribution; only changing the sequence/process can.
5. **STANDARD (final): one episode per FRESH PROCESS** for every paper-facing
   number. Only protocol that is reproducible AND meaningful here. Episode order
   becomes moot by construction.
6. **NO further env-level fixes now** — chasing solver-state reset is a separate,
   longer project, not a prerequisite for trustworthy numbers.
7. In flight: newbox_supp measuring ar4_s2 demo-IC as 15 fresh processes (sequence
   protocol said 0.07) — the delta = the protocol-artifact size on a real cell.
8. ar8 in-train curves: doubly deprecated (sequence-based AND straddling the fix
   boundary). Only fresh-process post-hoc evals count, under a re-registered
   protocol.
9. What still survives P2 in ANY protocol: SACfD = 0 across every seed, snapshot,
   and sequence (no draw ever produced a pick) vs RLPD producing picks under many
   draws. Magnitudes: all carry unknown protocol error until fresh-process
   measurement.

(Discovery record follows.)

**Discovery (newbox_supp):** same checkpoint, same flags, deterministic policy —
uid 243 picks as episode 7 of a 15- or 10-episode sequence, does NOT pick alone.
Episode outcomes depend on what ran before them in the same process.

**Mechanism (Fable, code + probe, CONFIRMED):** step() issues control_dofs_position
targets (genesis_can_env.py:231-232); reset() NEVER re-issues them — it teleports
positions (set_dofs_position), zeroes velocities, then runs scene.step() under the
PREVIOUS episode's final controller target. Probe (scratchpad/reset_leak_probe.py):
post-reset obs differs by history (qpos ~0.01-0.02 rad; obs dim 7 gripper-effort
differs 33.9 fresh-vs-used); an IDENTICAL 20-step command sequence after different
histories diverges up to **0.0506 rad = 2x the delta cap** — enough to flip a
marginal grasp. Genesis solver caches may add residue beyond this; the controller
target is the dominant confirmed channel.

**Blast radius:** every multi-episode single-process eval: RLPD/SACfD wandb_eval,
BC central-table evals, r2dreamer/dv3 periodic evals, the phase sweeps. Sequences
were fixed → results REPRODUCIBLE but not independent draws: binomial +-, Fisher,
multiple-testing math, and the >=3/15 threshold all assumed independence they do
not have. Between-ARM comparisons are partially protected where both arms ran the
SAME fixed sequence (same residue context), but the significance machinery needs
re-derivation. Training rollouts always had this property (less of a validity
issue; RL policies trained AND evaled under the same residue distribution).

**Second bug (newbox_supp):** wandb_eval `--uids` silently IGNORED unless
`--random 0` (line ~205), and `--random` caps the demo-IC episode count (why n=10
vs n=15 differed across runs). Same silent-default family. Fix post-wave.

**DOSE-RESPONSE (newbox_supp, CONFIRMED 08-14):** uid 243 as terminal episode,
same checkpoint/flags/horizon: 0 predecessors = False (x2), 1 predecessor = False,
**7 predecessors = True**. Residue ACCUMULATES across resets. The 400-vs-1600
"horizon effect" was never physics — longer predecessors leave different residue.

**TRIAGE (newbox_supp, adopted):**
- DEAD: the ar4 sensitivity column (horizon changes the residue schedule — it
  cannot isolate truncation). The "inconclusive-elevate" branch of the ar8
  pre-registration is WITHDRAWN with it (it compared primary vs sensitivity).
- DEAD: all absolute success rates; all cross-n comparisons (n=10 in-train vs
  n=15 post-hoc = different residue schedules; the earlier "pipeline validated by
  s2 overlap" finding is withdrawn by its author).
- WEAKENED BUT USABLE: matched-protocol comparisons (same episode count + order).
  stride-1 0.039 vs ar4 0.023 demo-IC, Fisher p=0.69 = "no detectable difference"
  stands — a null is the one result contamination cannot manufacture.
- The >=3/15 threshold survives with a NEW rationale: "3 picks exceeds anything
  observed from contamination alone", NOT binomial p<0.05.

**RULES + FIX PLAN (in force):**
1. IMMEDIATE RULE: compared conditions must share episode COUNT and ORDER; never
   compare across different n.
2. PAPER-FACING NUMBERS: one episode per process (clean by construction, ~15x
   process overhead, affordable).
3. RESET FIX (Fable, after in-flight probes finish, announced per shared-tree
   rule): reset() re-issues controller targets (arm + gripper) to HARDCODED_START
   BEFORE its scene.step() — the probe-confirmed dominant channel. Gate: re-probe
   must show history-independent post-reset obs AND identical-command trajectories.
   Solver-cache residue beyond the controller channel cannot be excluded — which
   is why rule 2 stays even after the fix.
4. ar8 trainers: LET FINISH (training validity is not the issue — policies train
   and act in the residue env either way); re-evaluate later under the fixed
   protocol + re-registered criterion.
5. OPEN QUESTION FOR THE USER (neither session acts alone): does P2 touch the
   central BC table (dH_DP vs dDP_DP, n=8, P=0.994)? Those evals ran the same
   multi-episode path. Matched protocols across arms may protect the COMPARISON;
   absolute rates and the significance machinery need re-derivation or re-running.
   User's call on how much re-running the paper warrants.
Also note: the §7 phase sweeps (pre- AND post-fix) used sequential per-shard
replays → contaminated start states; the pre/post #26 comparison retains meaning
(matched schedules), absolute stage counts get the same caveat.

## 4a-1. P2 RESOLUTION (08-14 ~03:30) — robustness structure + closed HOLD

- **s0@150k re-measured: 0.40, episode-for-episode identical** across (i) the
  original sequence protocol, (ii) post-fix sequence, (iii) 15 FRESH PROCESSES.
  Same six episodes pick in all three. Combined with the floor checkpoint
  (ar4_s2: 0.07/0.07/0.07, same uid picking), the STRUCTURE is: **real capability
  = robust-basin picks that survive protocol AND env-fix changes; floor-level
  rates = borderline picks that flip under microscopic state.** Mid/high rates
  trustworthy; single-pick cells not. `placed` is NOT covered by this robustness
  (moved 0.00->0.07 across the fix) — picked only.
- **JOINT RECOMMENDATION TO USER (Fable + newbox_supp): do NOT re-run the BC
  central table.** Its rates (0.6-0.8) sit squarely in the robust regime with
  direct evidence mid-range rates survive both changes. Ship a methods paragraph:
  the eval-layer defect, the fresh-process standard going forward, and this
  robustness check. More informative than a silent re-run.
- **0.40 vindicated, with a provenance caveat recorded:** the original confirm
  log carries no action_mode/repeat/horizon lines and the invoking script was not
  kept — the headline figure was not reproducible from its own record until the
  fresh-process replication. NEW RULE: every eval log must record mode / repeat /
  horizon / checkpoint; keep the invoking script with the result.
- newbox_supp's HOLD withdrawn; its table enters the unified readout with the
  agreed caveats (floor cells indistinguishable, sensitivity column dead,
  threshold-sensitive ignition counts, no cross-n comparisons).

## 4a-2. ar8 verdict protocol — RE-REGISTERED 08-14 (~03:30, before any ar8
## post-hoc data)
- Fixed checkpoint: 100k decisions (rlpd_100000_steps.zip per seed).
- Protocol: **15 fresh processes** demo-IC + 15 fresh random-IC per seed,
  explicit --action-mode delta_joint --action-repeat 8, 400 env steps, picks
  from stdout, post-fix code only.
- Decision rule: ar8 "worth pursuing" iff >=1 seed reaches >=3/15 demo-IC.
  Rationale: 3 picks exceeds anything observed from contamination/borderline
  effects alone (NOT a binomial claim). No sensitivity column (withdrawn — it
  confounds residue schedule with horizon).
- EXECUTION (newbox_supp, staged behind the last trainers): 180 fresh processes.
  APPROVED DEVIATION: random-IC arm = 15 INDEPENDENT ICs (--random 1 --seed k,
  k=0..14), not one stream's 15 — independent draws are if anything cleaner; the
  criterion arm (demo-IC) is exact. newbox_supp's registered prediction
  (pre-data): 0/6 seeds reach >=3/15. Fable registers NO counter-prediction —
  the in-train signal (one stray 0.10 at 62.5k across 6 seeds) points the same
  way, and a second identical prediction adds nothing.

## 4a-3. ar8 VERDICT (newbox_supp, 08-14 ~05:30, per §4a-2): **NOT WORTH PURSUING.**
0/90 demo-IC picks across all 6 seeds (1/90 random-IC = borderline noise). Both
registered predictions honored (newbox_supp predicted 0/6; correct — noting a
confirmed null prediction is weak evidence next to a failed one).
SCOPE LIMIT (caption material): trained at repeat-8, EVALUATED at 400 env steps
= 50 decisions (equal sim time). The defensible claim is operational — "at equal
physical time, repeat-8 produces no picks" — NOT a training-regime mechanism claim
(the column that would separate those was killed as confounded).

**REPEAT-N CLOSED, one protocol, 100k decisions:** stride-1 0.039 (4/105) |
ar4 0.023 (2/90) | ar8 0.000 (0/90). Monotone decreasing, nothing clears the noise
floor, stride-1 vs ar4 p=0.69. **Action-repeat bought nothing measurable at this
budget; the literature's horizon hypothesis is unsupported by our data.** Clean
negative result; report it as such. ar8 in-train curves doubly superseded — cite
only this table.

## 4a. ORIGINAL pre-registration — superseded by 4a-2 (kept for the record)

- Checkpoint: **100k decisions** = rlpd_100000_steps.zip per seed (same step as
  rlpd_final at this budget; use the _steps file for symmetry with other arms).
- PRIMARY: 400 env steps (equal SIM TIME across arms; at ar8 that is 50 decisions —
  that IS the action-repeat tradeoff, not a confound). demo-IC n=15 + random n=15,
  explicit --action-mode delta_joint --action-repeat 8, picks from stdout.
- SENSITIVITY: 3200 env steps (decision-matched to stride-1's 400). Runs entirely
  POST-fix (unlike ar4's mixed pair). Informs interpretation; does not decide.
- DECISION RULE: ar8 = "worth pursuing" iff >=1 seed reaches >=3/15 picks demo-IC
  under PRIMARY. If a seed crosses >=3/15 ONLY in the sensitivity column, the
  verdict is "inconclusive-elevate": bring to the user (policy may have learned but
  needs more physical time than equal-sim-time allows), do not auto-kill and do not
  auto-pursue.
- Unified-table rule: any "best-any-snapshot" exploratory column is labeled with the
  episode count it was scanned over (verify per arm from logs; ~80 eps expected for
  all three at their cadences — 8 snapshots x n=10 — but COUNT, do not assume).

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

**SHARED WORKING TREE (learned the hard way 2026-08-13):** both sessions operate in
the SAME checkout on this box. An edit to a runtime file (env, encoder, eval) is
LIVE for every process the other session launches from that instant — no pull, no
notification. Two of newbox_supp's sensitivity evals silently launched pre-fix and
four post-fix across one of my commits. RULE: announce any edit to a shared runtime
file to the other session BEFORE saving it while that session has jobs in flight;
the announcer states the file, the change, and the commit; the other session
records which of its in-flight processes predate it. (Processes hold the OLD code
in memory; only new launches pick up the edit — both facts matter for attribution.)

**PGREP SELF-MATCH, CROSS-SESSION VARIANT (08-14, bit BOTH sessions):** Fable's
ar8 watcher gated on `until ! pgrep -f "train_rlpd.*action-repeat 8"` — the
pattern matched the watcher's OWN cmdline, so it deadlocked (backfill never ran),
AND newbox_supp's sweep gate matched Fable's stuck watcher, stalling its sweep an
hour behind trainers that had already finished. RULE: gate on ARTIFACTS (marker
files, checkpoint counts, output staleness), never process patterns; kill by PID
or TaskStop, never by pattern when siblings share substrings.

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

## 10. CLOSED-LOOP RE-RECORD (P1 SOLUTION) — census 08-14 ~05:00

baselines/rl/rerecord_delta_demos.py (commit a164cdf): waypoint follower in
FullTaskEnv(delta_ref='measured'), advance-on-arrival vs recorded MEASURED poses,
dwell-capped, settled-nested scoring (collector semantics, proxy kept separate).

**CENSUS (72 resettable, stride-1): picked 61 / placed 55 / contact 28 / nested 20**
vs labels 65/62/25/16 and the target-ref tape 42/40/5/1. DOWNSTREAM FULLY
RECOVERED. Dilation median 1.013x (p90 2.27x, 1 truncated at the 3x cap; 59/72
used dwell catch-up somewhere). Encoder round-trip max err 9.5e-7 — the tapes are
bit-consistent with delta_encode_transitions_measured. Overshoot vs labels
(contact +3, nested +4) = borderline flips in our favor (P2 class), flag not
celebrate. 9 demos below label incl. 6 no-picks — census aggregate is the signal.
Demo set: baselines/episodes_delta_rerecord/ (72 npz + shard manifests, RSYNC ONLY).
CAVEAT: sequential shards -> P2 residue; fresh-process re-census before paper use.

**skip-N (user directive "how short can trajectories get"): RUNNING** — follower
extended to decision-level N (window-end target + window-end grip, furthest-arrived
advance; smoke uid 308 @N=8: 1354 frames -> 508 decisions, stage placed).
Censuses at N=4 and N=8 in flight (episodes_delta_rerecord_r{4,8}). NB the ar8
RLPD training null does NOT cancel this — different question (demo compression
for credit assignment, not policy training at coarse control).

## 9. Post-P2-fix gate re-verification (08-14 ~02:00)

dv3 msrecipe gate re-run on the FIXED env: **GATE PASS 4/5** — but uid 326 FLIPPED
(pre-fix lift 0.155 -> post-fix 0.102, fail). Independent confirmation, from a
different pipeline, of both P2 lessons: the reset fix changed the baseline, and
marginal episodes resolve arbitrarily under microscopic state changes. The >=4/5
tolerance absorbed it by design — thresholds with slack survive P2; exact
per-episode outcomes do not. The RLPD stride-gate and msrecipe gate pass on both
sides of the fix; cluster submit spec remains valid.

## 11. skip-N re-record censuses (08-14 morning) — COMPLETE

| N | >=picked | >=placed | >=contact | >=nested | median decisions | saturation |
|---|---|---|---|---|---|---|
| 1 | 61 | 55 | 28 | 20 | 1957 | 1.4% |
| 4 | 58 | 32 | 10 | 8 | 1138 | 1.0% |
| 8 | 43 | 29 | 6 | 4 | 592 | 2.3% |

READING: pick survives N=4 nearly intact (58 vs 61); everything downstream
degrades sharply already at N=4 (55->32 placed, 28->10 contact, 20->8 nested).
Effective compression is SUB-LINEAR (1957->1138 = 1.7x at nominal 4x; 3.3x at
nominal 8x) because dwell catch-up eats the stride. Saturation stays low at all
N — strain shows as TIME DILATION and stage loss, not clipping (the follower
dwells instead of saturating). DECISION GUIDANCE: N=4 usable for PICK-scope
demos; stride-1 re-record is the full-task set. Same caveats as §10 (sequential
shards / P2 residue; single census per N; fresh-process re-census before paper).
r4/r8 npz stamped via backfill after shard exit.

## 12. RLPD AUDIT (08-14, paper/rlpd_audit_2026-08-14.md) — ROOT CAUSE FOUND

**Bug 1 (CONFIRMED, quantitatively sufficient): entropy backup was ON; RLPD turns
it OFF for every sparse domain.** Inherited from SB3 SAC. At gamma=0.998 the
critic's zero-reward fixed point is 500*alpha*H; the formula reproduces EVERY
logged Q window (269..2400 vs max return 1.0). The +1 was 0.04-2% of the
regression target, and terminals (the picks) got target 1.0 vs ~400 for
non-terminals — **400:1 against completing the task.** Explains marginal
ignition, post-ignition decay, SACfD's uniform zeros (same defect), and the
fixed-alpha arm's Q->1.6e5 (that prescription is RETIRED). The Q-watchdog fired
correctly at step 1001 (Q=2.82) and was waved off; one-shot design never
re-flagged the ride — now re-arms every 10k.
**Bug 2 (queued lever): ensemble members share LayerNorm affine params** (inter-
member corr measured -0.015 at init is fine but diversity degrades in training).
**Bug 3 (minor): 2-layer critic vs paper's 3.**
CLEAN: truncation bootstrapping (both paths, measured 0.000 mistreated), demo
tensors/grip/reward parity, subset-min over targets, tau, target_entropy.
ALSO SURFACED: eval-horizon asymmetry — RLPD row evals at 400 sim steps, the
DP/SACfD matrix at wandb_eval's 1200 default (median pick frame 662). Size
unknown; flag on any cross-arm table.

**CONSEQUENCE: the repeat-N verdict (§4a-3) is RE-DECLARED PROVISIONAL.** A flat
result across N is exactly what a signal-invisible critic produces at any N.
RLPD>SACfD survives (shared defect, strict zero vs occasional picks).

**FIX WAVE (running): dH_RLPD-nb_s{0,1,2}** — backup_entropy off (the ONLY
lever changed; bug 2/3 deliberately deferred for single-lever attribution),
stride-1, 100k, same demos/protocol. Commit f906ee6; flag explicit + in sidecar
+ cfg line; loaded-model attr verified False. SUCCESS CRITERION (pre-registered):
>=1 seed >=3/15 demo-IC at fixed 100k under fresh-process protocol. If flat, bug
2 is the next single lever.
