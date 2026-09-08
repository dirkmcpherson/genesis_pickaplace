# PHASE PLAN — human vs machine demonstrations, per task phase (registered 2026-09-04 09:15, before any learner run)

**Ask (James, 2026-09-04 08:20):** repeat the human-vs-machine comparison for each phase of the task; initial conditions
may come from the demonstrations or from policies that can do the earlier phases; expect less data per later phase;
Place and Contact at least.

## 1. Phases and boundaries (predicates of record)
| phase | entry state | success predicate | terminal | horizon |
|---|---|---|---|---|
| 1 Pick | task start (done: RESULTS_WM_HUMAN_VS_MACHINE) | `picked` (hardened lift) | on pick | 1200 sim |
| 2 Place | can lifted, at the **pick grant** | `placed_v2` = grip command < 0.45 (released) ∧ can in the shelf footprint z-band ∧ tilt < 20°, sustained 10 frames (`FullTaskEnv.scope='place'`, `full_env.py:679-688`) | on placed_v2; tip rule | 600 sim (existing scope cap) |
| 3 Contact | can released on the shelf, at the **placed_v2 grant** | `contact` = solver contact can↔goal ∧ picked-history ∧ eef behind the can (`genesis_can_env.py:271-274`) | on contact; tip rule | 600 sim |
| 4 Nested | (not in this plan) | settled nested test | — | — |

The legacy `placed` flag (z-band while held) is unearnable in this world (CONFOUNDS row 47) and is NOT used. The per-step
`nested` proxy under-counts ~3× and is not used as a boundary; phase 4 waits on that fix.
Because the pruning rule only collapses idle runs BEFORE the pick (`make_dp_pruned.py`), the raw and pruned human tapes
are identical from the pick grant on ⇒ **one human arm per later phase** (no raw/pruned split for phases 2–3).

## 2. Data chain (all in the corrected world `gc_kp4_riser3_shelf6`, cluster, rsync-only)
1. **Human full-task tapes** `baselines/demos_v2/dHfull_w3`: `record_demos.py --teacher human --scope full` from the 75
   success-labelled full-length tapes `episodes_all_v2r` (the follower consumes their absolute command streams), 8 CPU
   shards, array 3256732 (launched 09:05). Yields per phase are a RESULT, recorded here when in (the sim-box replay
   funnel predicts ≈ 66 placed / 26 contact / 16 nested of 74).
2. **Machine full-task teacher**: a DP trained on the PRUNED human full tapes (pre-pick idle collapsed by
   `make_dp_pruned.py`, from-pick-on untouched; lerobot conversion; 100k grad steps, seed 0) — the full-scope analogue
   of the dDP teacher (`dp_pilotw2/dH_DP_s0`). **Machine full-task tapes** `dDPfull_w3`: `record_demos.py --teacher dp
   --scope full --ic-mode demo --attempts 3 --mode sample --verify` from the same demo ICs. Yield per phase is a result.
3. **Entry banks** (JSON, the existing bank schema: frame, qpos[6], grip_cmd, grip_obs, can_pos, can_quat, goal_xy):
   `bank_place_{h,m}.json` at the pick grant and `bank_contact_{h,m}.json` at the placed_v2 grant (offline predicate =
   the env's, as in `to_dreamer_demos_place.py`), one entry per tape that reaches the boundary, per source (h = human,
   m = machine). Restore survival (`_restore_place_entry` verify) must be ≥ 0.9 per bank, else the bank is rebuilt with
   a longer settle — a gate, not a tuning knob.
4. **Phase-segmented demos**, native stride-4 with state, terminal +1 at the phase's own grant, nothing else:
   `demos_state/{dH,dDP}_{place,contact}` = tape[entry frame → grant frame + 1]. Converter: extend
   `to_dreamer_native.py` with `--phase place|contact` (it currently cannot cut and rejects full-scope tapes).
5. **Evaluation ICs per phase** (shared across arms and learners): (a) `holdE` = entries of the 15 hold uids' human tapes
   (the demo-IC analogue) plus (b) `polE` = entries produced by POLICIES that do the earlier phases — the eight stage-1
   r2dreamer pick policies of the *human* arm run from the 30 fixed random placements to the pick grant (phase 2), and
   the phase-2 winners run to placed_v2 (phase 3) — the random-IC analogue, outside the demo support. Both banks are
   frozen files with the generating checkpoints' names inside them.

## 3. Learners, in order
- **r2dreamer** (fixed recipe `bnormclamp1ent5`; adapter already takes `scope='place'` + `place_entry_bank`): phase 2 first,
  then phase 3 after `scope='contact'` is added to `FullTaskEnv` (private copy) and the adapter. 8 seeds per arm, 1M
  sim steps (episodes are ≤ 600 sim steps so this is ≥ 2× the decisions of phase 1), evals hold-E + pol-E in sample and
  mode, LAST checkpoint. Training entry distribution = each arm's OWN bank (human arm trains from human entries,
  machine arm from machine entries); evaluation banks shared.
- **RLPD**: needs `--scope place|contact`, an entry-bank reset and phase ranks in its relabeler (`train_rlpd.py`,
  `train_sacfd_full.py:32`); built after the WM phase-2 read-out, same seeds and evals.
- **DP**: phase datasets via `convert_to_lerobot.py` (phase-agnostic) + an entry-state-capable evaluator (today's
  `wandb_eval.py` builds `GenesisCanEnv` and cannot restore a mid-task state) — built after RLPD.
- **dv3**: excluded (no working configuration, stage 1e).

## 4. Statistic, gates, predictions (registered)
- Per phase and learner: success rate on **pol-E** (the shared out-of-support entry bank) at the LAST checkpoint, per-seed
  counts, exact two-sided permutation test, n = 8 v 8; hold-E secondary; sample and mode both reported.
- Negative controls before any read-out: random policy from each bank (expected ≈ 0 for placed_v2 and contact); the
  stage-1 pick policies replayed past the pick without the phase learner (documents what "do nothing new" earns).
- Gate for a phase to be reported as a comparison: both arms have ≥ 20 phase demos AND both arms' recipe produces
  ≥ 0.5 on hold-E in ≥ 3/8 seeds (a learnability floor; otherwise the phase is reported as "not learnable at this
  budget", with the yields). Prediction (from phase 1): |Δ| < 0.10 on pol-E for phase 2; no registered prediction for
  phase 3 beyond "machine demos ≤ human demos in count".
- Disclosed by construction: demo counts per arm per phase (the point of the ask), entry-distribution asymmetry between
  arms during training, the machine teacher's own phase yields, that phase-3 entries for training come from tapes but
  for pol-E from phase-2 policies.

## 5. Order of work and time (cluster)
Human recording ≈ 20 min (running) → banks + converter + contact scope ≈ 3 h of code with smoke tests on the recorded
tapes → pruned full set + lerobot + DP teacher ≈ 2 h → machine harvest ≈ 1–2 h → WM phase-2 runs (16 × ~3 h, parallel)
→ phase-3 the following night. RLPD/DP tooling in the gaps.

### Amendment 2026-09-04 11:10 — (a) the release band was stale; (b) broader-IC retest of the phase-1 models (James)
(a) `placed_v2` (and the legacy `placed`) reference `BOX_TOP_Z = 0.11`, but `gc_kp4_riser3_shelf6` raises the shelf
by `shelf_dz = 0.06` (top 0.17 world) and no predicate constant moved with it (sim_variants.py:66 says they MUST).
Measured on the 74 human full-task tapes: 52/64 pick-reaching tapes release the can after the pick, at can-centre z
p10 0.220 / p50 0.231 / p90 0.256, 51/52 inside the shelf footprint, 47/52 upright — and **0/52 inside the coded
band 0.12–0.18**; can centre at the contact frame p50 0.228 = shelf top 0.17 + half a 0.101 m can. Fix (private env
copy + bank builder, `shelf_band_patch.py`): the band is `shelf_top_z + [0.01, 0.07]` with `shelf_top_z = BOX_TOP_Z +
shelf_dz(variant)` read from `sim_variants.VARIANTS` — 0.18–0.24 in this world. Banks and yields are rebuilt with
it; the phase-2 boundary definition is otherwise unchanged. CONFOUNDS row 47's blast radius now has a concrete
instance: every prior place-phase dataset/bank and the frozen block's `placed` numbers used the 0.11 band.
(b) **Registered (James, 11:05): re-evaluate the phase-1 (pick) checkpoints of record on a MUCH broader random-IC
set.** Design: `rnd300` = 300 placements from `ic_sampling.sample_support_ics(env, 300, seed=1)` (the same support
box as rnd30 — `baselines/eval_ics.json` `support_box` — a different seed so rnd30 is not a subset), frozen as
`baselines/eval_ics_rnd300.json`; all 16 stage-3 r2dreamer checkpoints (dHv2raw s0–7, dDP s0–7), LAST, MODE and
SAMPLE; statistic = per-seed picks /300, exact permutation as before; also the two dH-pruned pilot seeds. Predicts
the rnd30 estimate within ±0.05 per arm (rnd30 is a 30-draw sample of the same support). If RLPD/DP checkpoints of
record are on disk, the same file is run through their evaluators so the broader set is shared. NOT yet submitted
(VPN down at registration time).

## 6. Work order 2026-09-04 12:15 (James) — three items, 60 h of queue inside a 12 h VPN window
1. **ALL-DATA human arm, new label `dHv2all`** = every recorded human episode in the corrected world at pick scope:
   the 66 `dHv2raw` successes + the recorder's kept fails/partials of the same run + the 16 real-fail demonstrations
   (never recorded before; recorded now with `record_demos.py --teacher human --scope pick --uids <16 fail uids>`
   from `episodes_pick_phase_all`, which carries all 91). Fail tapes carry reward 0 throughout, terminal by cap or tip.
   The old sets and results are untouched. Learners: r2dreamer (recipe of record) and RLPD (recipe of record, native
   arm added to `cluster/sbatch_rlpd.sh`), 8 seeds each; DP is not run (it degrades on raw and cannot use zero-reward
   tapes). Evals: hold15, rnd30, holdv2, alldemo74 and rnd300, sample and mode. **Registered prediction (James's
   hypothesis): the world model GAINS from human failures — r2dreamer dHv2all > dHv2raw on rnd30 MODE by ≥ 0.05;
   RLPD no registered direction.** Statistic as before (LAST ckpt, per-seed counts, exact permutation, n = 8 v 8
   against the existing dHv2raw runs).
2. **All-phase experiments** = §1–5 of this plan (place, contact), executed as a Slurm dependency chain so it runs
   unattended: banks → teacher data → DP full-task teacher → machine harvest → machine banks + phase segments +
   evaluation banks → 16 WM place runs; contact runs gated at job start on the registered yield floor (≥ 20 demos
   per arm), exiting cleanly with a logged reason otherwise.
3. **Visual + quantitative characterization of dDP vs dHv2raw** (delegated): per-tape descriptors on both sets with
   the same IC pairing (length, idle fraction, path length, speed and jerk, time-to-grasp, approach height and
   direction, grip-close timing, can displacement before the grasp), a side-by-side video gallery of matched uids,
   and a short note; read-only on the data, no learner runs.
Queue plan (GPU-hours): r2dreamer dHv2all 8 × 3 h = 24; RLPD dHv2all 8 × ~4 h = 32; rnd300 evals 36 × 0.3 h ≈ 11;
WM place 16 × 3 h = 48; WM contact 16 × 3 h = 48 (gated); DP teacher 3 h; ≈ 165 GPU-h total, ≈ 60 h of wall at the
cluster's usual 3-job-per-hour drain, all submitted with dependencies inside the window.

### Note 2026-09-04 15:30 — relevance of the sim-box adoption (`gc_kp4_riser3_shelf6_og4`, CONFOUNDS row 50) to this plan
The adopted change is a RECORDER-path filter (record_demos.HumanFollower `_filter_grip`, `grip_open_gain=4`): opening
moves of the human's grip stream are amplified 4× below the running hold plateau; closing/holds untouched; the world is
bit-identical to w3; the code asserts `teacher == 'human'` (machine harvests never get it). Census effect: tipped at
release 32→20, honest nested 16→23, set-down 46→50, contact 26→28, picked unchanged.
- Phase 1 (pick): NOT affected — the filter acts at the release, pick-scope tapes end at the pick, the world is the same.
  Nothing in RESULTS_WM_HUMAN_VS_MACHINE, the dHv2all arm or the rnd300 retest changes.
- Phase 2 (place) and later: AFFECTED in principle. The human full-task tapes of this plan (`dHfull_w3`) were recorded
  with the plain w3 recorder (no filter): 15/74 tips at release, placed_v2 39/64. Under og4 the human releases would tip
  less and more tapes would register a place — but the machine teacher/harvest cannot receive the filter as coded, so
  og4 human data vs plain-recorder machine data would carry the row-50 asymmetry into the very phase that is about
  releasing. The running chain is therefore left on the plain recorder for BOTH arms (symmetric, registered).
- Proposed follow-up, not launched: a disclosed `dHfull_og4` human recording (15 CPU-min) for a within-human
  comparison of place-phase yield and learner performance (plain vs og4 releases), and — if the sim-box agent makes the
  filter applicable to DP teacher grip outputs — an og4 machine harvest, so the place phase could later be run
  symmetrically under og4. The og4 recorder code is uncommitted in the sim-box working tree (record_demos.py,
  sim_variants.py); the cluster recorder used here is the committed version.
- Amendment (b) note, 2026-09-04 19:50: one rnd300 cell (dDP s1, deterministic actions) stalls the simulator
  deterministically on placement index 269 (twice, same episode). Treatment, disclosed: the evaluator's new
  `--ic-skip` records that episode as outcome `hang`, counted as a FAILURE (0 picks), so the cell is 299 scored + 1
  failure rather than missing; no other cell of the 36 stalled. The rnd300 statistic for that seed is therefore a
  lower bound by at most 1/300.

### Amendment 2026-09-04 22:00 — machine phase sets capped to ONE tape per IC (registered before the machine runs started)
The full-task harvest keeps up to 3 attempts per demo IC (195 tapes over 72 ICs; picked 179, contact 46, nested-proxy
16, tipped 32). Uncapped, the machine place set would be 104 segments vs the human arm's 39 from 39 ICs — demo COUNT
would be confounded with source. The arm of record therefore takes the FIRST tape (sorted filename) per IC that reaches
the phase, the same one-tape-per-IC rule as the phase-1 machine set: **dDP place 63 segments (63 ICs) vs dH 39; dDP
contact-after-release 25 vs dH 11.** The uncapped sets are kept (`demos_state/dDPuncapped_*`, disclosed, not run).
Consequences: the place phase runs two-armed (both ≥ 20); the contact phase runs ONE-ARMED — the human arm is below the
registered floor (11) and GATE-SKIPs, the machine arm (25) trains — reported as a learnability result, not a comparison.
Training entry banks stay uncapped (machine 179 entries, human 64): the reset distribution over starts, not the demo
count, and disclosed as such. The DP full-task teacher's own headline: hold 14/15, rnd 14/30 (pick predicate).
- Addition 2026-09-05 01:30 (registered before use): a SECOND policy-generated place bank, `polE_place_dDP`, from the eight
  MACHINE-arm phase-1 pick checkpoints on the same 30 random placements (same dump procedure as `polE_place`, which
  came from the human-arm checkpoints). Every place checkpoint of both arms is scored post hoc on it, both modes, so
  the polE statistic exists under both entry-generating sources; the registered statistic stays the human-generated
  bank (registered first), the machine-generated bank is reported beside it. Prediction: the two banks rank the arms
  the same way (|Δ of Δ| < 0.10); a reversal would mean the entry distribution, not the demos, drives the difference.

### Amendment 2026-09-05 14:10 — contact phase continued (James); the action-repeat question (James)
**(a) Contact-from-pick, two-armed — new phase `carrycontact`.** Entry = the pick grant (same banks as place: human 64 /
machine 179 entries; eval banks holdE_place, polE_place, polE_place_dDP); success = the env's `contact` predicate
reached by EITHER route (set down then slide, or carry into the goal); segment = [k_pick, k_contact]; tips terminate;
horizon 600 sim steps. Yields: human 21 tapes reach contact (11 after a release + 10 while holding) ≥ floor 20;
machine 46 tapes / one per IC. Registered as the phase-3 comparison of record; prediction as for place (|Δ| < 0.10 on
polE MODE). Same recipe, 8 seeds per arm.
**(b) Contact-from-release, human arm below the floor — disclosed, not of record.** 8 human seeds on the 11 release-
route segments, so the one-armed machine result (25 demos, holdE 0.98) has a comparator; reported with the floor
violation stated, no test of record.
**(c) Action repeat 4 as a possible mask (James).** Every learner sees decisions at 7.5 Hz; the human tapes' within-
window structure (jitter, micro-corrections at 30 Hz) is averaged into one window-end target, the machine teacher's
output is smooth by construction. Registered two-step check: (1) a data-only diagnostic on the source tapes
(`sim_actions`/`sim_states` per sim step): within-window action variance, sign reversals, spectral energy above
3.75 Hz, and the fraction of window-end targets that differ from the window mean, human vs machine, paired by IC;
(2) IF the sets differ at 30 Hz by more than they differ at 7.5 Hz (d ≥ 0.5 on any of those), a repeat-1 pilot
(r2dreamer, dHv2raw vs dDP re-encoded at stride 1, 4 seeds per arm, 1M sim steps = 1M decisions, same evals) is
submitted; otherwise the clock is reported as "no structure to hide". Note: the old-world RLPD +0.21 was ALSO at
repeat 4 (final round robin 08-23), so the clock is not what removed it.

### Amendment (c′) — repeat-1 clock pilot: registered settings (2026-09-05 09:55, before any pilot run)

The diagnostic rule in amendment (c) fired (machine target sign reversals within the 4-step window d_z −1.93; `clock_diag.py`, log 2026-09-05). Before building the pilot I measured the per-sim-step target structure in the matched tapes and it corrects the registered cap:

- **The recipe of record already integrates `a × cap` at EVERY sim step under action_repeat 4** (`envs/genesis.py`: "Under action_repeat N the target integrates a*cap EACH sim step"), so the physical per-step cap at repeat 4 is 0.025 rad per sim step and a decision moves the target up to 0.1 rad. The tapes' `sim_actions` targets change every sim step (per-step max-joint |Δtarget| p50/p90/p99: human 0.0002/0.0145/0.025, machine 0.0025/0.0175/0.025). A per-step cap of 0.025/4 = 0.00625 (my (c) wording) would clip 22.7 % of human and 36.7 % of machine sim steps — the demos would no longer be reproducible and the pilot would be confounded. **Registered cap: 0.025 per sim step (unchanged)**; at that cap the stride-1 re-encoding clips 1.3 % (human) / 0.9 % (machine) of steps, the same order as the repeat-4 decision-level clipping (p99 of the per-decision delta = 0.1 = the cap).
- Settings held physically equal to the recipe of record (`configs/env/genesis_pick_state_r1.yaml`, diff vs `genesis_pick_state.yaml`): action_repeat 1; delta_cap 0.025 and leash 5 (unchanged); time_limit 1200 sim steps (`TimeLimit(time_limit // action_repeat)`); **horizon 1332** (discount per sim step 1−1/1332 = (1−1/333) per 4-step decision); **train_ratio 128** (trainer update period = batch_steps/train_ratio × action_repeat sim steps = 1024/128 × 1 = 8 = 1024/512 × 4: equal gradient updates per sim step); steps 1e6 sim steps (= 1e6 decisions; the repeat-4 runs had 2.5e5 decisions in the same physical budget); buffer 5e5 rows (demos never evicted; stride-1 demo rows: human ≈57k, machine ≈29k). Changed by construction and NOT compensated (disclosed): decisions per episode 1200 vs 300; batch_length 64 decisions = 64 sim steps of model context vs 256; the actor/critic see 4× more decisions per unit experience.
- Demos: `demos_state_r1/{dHv2raw,dDP}` = the SAME 66/58 `matched_w3` tapes re-encoded at one row per sim step from `sim_states`/`sim_actions` (`to_dreamer_native.py --repeat 1 --stride1-cap 0.025 --with-state --state-only`; delta = (target_t − target_{t−1})/0.025 clipped to [−1,1]; +1 on the last row; the recorder's last window can be shorter than 4 steps and is kept as recorded). Build gate (`r1_build.sbatch`): same tape count and total reward as the repeat-4 dirs; local check on three tapes: `cumsum(action×0.025)` reconstructs `sim_actions` exactly (max error 0), state rows equal `sim_states`, grip column equal.
- Runs: r2dreamer, 4 seeds × {dHv2raw, dDP}, 1e6 sim steps, smoke (5k steps, hold15) gates the 8 runs (`afterok`). Eval: LAST checkpoint, fresh process, hold15 + rnd30, sample and mode (the 74-IC set is skipped in-job at 1200 decisions/episode; `EVAL_SETS="hold rnd"`).
- **Predictions (registered).** P1 (the clock hides nothing): |Δ(dHv2raw − dDP)| on rnd30 MODE < 0.10 (n=4v4; minimum two-sided exact p = 0.029). P2 (the clock hid a source effect): |Δ| ≥ 0.15 with the same sign on hold15 and rnd30; my prior is P1 at ≈ 2:1, and if P2, human > machine (the machine's sub-window sign reversals are chunk-boundary jitter that a 30 Hz learner must model as action noise; the human's joystick targets are smooth at 30 Hz). Disconfirm branch: if any arm has all 4 seeds < 0.30 on rnd30, the repeat-1 recipe (never tuned; the control ladder was built at repeat 4) is uninformative about the source question — reported as "recipe not transferable to repeat 1", no re-tuning without a new registration. Cross-clock comparisons (repeat-1 vs repeat-4 rates) are confounded by the uncompensated items above and are descriptive only.

#### (c′) VERDICT — read out 2026-09-05 14:55 (all 8 runs, LAST checkpoint, fresh process, `wmfix_s2` in-job evals)

| cell | human dHv2raw (4 seeds) | machine dDP (4 seeds) | Δ (human − machine) | exact perm p |
|---|---|---|---|---|
| rnd30 MODE (registered) | 22, 18, 18, 19 = 77/120 (0.642) | 18, 20, 19, 21 = 78/120 (0.650) | −0.008 | 1.000 |
| rnd30 SAMPLE | 20, 20, 17, 19 = 76/120 (0.633) | 17, 20, 19, 18 = 74/120 (0.617) | +0.017 | 0.829 |
| hold15 MODE | 60/60 | 60/60 | 0 | 1.000 |
| hold15 SAMPLE | 59/60 | 60/60 | −0.017 | 1.000 |

**P1 met, P2 not met:** at action_repeat 1 (one decision per simulator step; cap, leash, discount and updates per sim step held equal) the two demo sources are indistinguishable, |Δ| = 0.008 on the registered cell against the 0.10 margin; the disconfirm branch ("recipe not transferable") did not fire — every seed of both arms learned (rnd30 0.60–0.73). The action-repeat hold was not hiding a source effect. Descriptive cross-clock line (uncompensated batch_length, 4× more decisions per unit experience): the same four seeds at repeat 4 scored human 15, 19, 19, 19 / machine 20, 18, 18, 18 on rnd30 MODE, i.e. the repeat-1 runs are within a few episodes of their repeat-4 counterparts; picks are faster at repeat 1 (mean episode length 229–349 decisions ≈ 8–12 s of sim time vs the 300-decision cap at repeat 4). Runs 3290344–47 (human), 3290348–51 (machine); demos `demos_state_r1/{dHv2raw,dDP}` (build gate in the log); config `genesis_pick_state_r1.yaml`.

### Amendment (d) — END-TO-END full-task arm (registered 2026-09-05 10:05, before any run)

User (2026-09-05): "now that everything is working maybe we should try to learn the entire task end to end in addition to phases." Registered design:

- **Task.** `scope='full'` of the private `full_env.py` copy: from the pick-scope ICs (rnd / hold15 / rnd300, can on the table, arm at home), the recorded staged sparse reward `STAGE_REWARD` = picked 1 / placed 1 / contact 2 / nested 4 (each paid once at its grant; nested terminates; tips terminate with no penalty; time_limit 1200 sim steps = 300 decisions). No shaping. Same reward function as the RLPD full-task arms (A31/A33), so the learner comparison is like-for-like on reward.
- **Learner.** r2dreamer recipe of record (`bnormclamp1ent5`: state obs, stock `bounded_normal`, act_entropy 3e-5, reward_scale 1, train_ratio 512, horizon 333, action_repeat 4, buffer 5e5) with **return_clamp 8.0** = the maximum staged return (the pick recipe's 1.0 is *its* maximum return; the clamp's job is λ-targets past the attainable return, so it scales with the reward function). `configs/env/genesis_full_state.yaml`; launcher `wmfix_full.sbatch` bakes the actor/entropy overrides in. Budget **2e6 sim steps** (2× the single-phase budget; the pick alone needed ≈ 3–5e5), 4 seeds per arm first wave (n=4v4, min two-sided p 0.029); 8v8 only if both arms learn the pick (registered: wave 2 is triggered by "both arms ≥ 0.5 picked on rnd30 MODE", not by the sign of any difference).
- **Demos (symmetric plain recorder, world w3, one row per decision).** Human `dHfull_all` = EVERY full-task tape of the 74 success uids: 3 nested + 61 partial (61 picked, 18 contact) + 10 no-pick (recorded reward sums 7 / 3 / 1 / 0; total 118) — the "all data incl. failures" principle of the human arm. Machine `dDPfull` = the 195-tape dDP full-task harvest over 72 ICs (16 nested, 163 partial, 16 no-pick) reduced to ONE tape per IC by the highest recorded reward sum (ties: shortest) → 72 tapes (`--one-per-ic-best`). Rewards are the recorded staged grants (`--reward-from-tape`). Asymmetries disclosed: 74 vs 72 tapes; the machine set is the teacher's best attempt per IC (selection), the human set is one attempt per uid (no selection); the machine has 16 nested tapes vs 3 human; both include their no-pick failures.
- **Evaluation.** LAST checkpoint, fresh process, hold15 + rnd30 in-job (sample and mode), rnd300 and the 74-IC set post hoc; **success-by-stage** (picked / placed_v2 / contact / nested granted at any time in the episode; `eval_genesis.py` `stages`), the scope's success = nested. Test: exact two-sided permutation on per-seed counts, per stage.
- **Predictions (registered).** P1: both arms learn the pick from scratch under the staged reward (picked ≥ 0.5 on rnd30 MODE in ≥ 3/4 seeds per arm). P2 (the main hypothesis, consistent with pick/place): |Δ(human − machine)| < 0.10 at every stage that either arm reaches ≥ 0.2. P3: nested < 0.2 for both arms at 2e6 (the chained task at 300 decisions with sparse grants is hard; the phase experiments remain the sensitive comparison). Disconfirm branches: if only ONE arm learns the pick, that is a demo-source effect specific to the long-horizon setting (the pick-only sets were matched successes; here the human set carries 10 no-picks and 43 pick-only tapes) — reported as such, then a matched control (human successes only) before any claim. If neither arm picks, the staged-reward recipe is not transferable to the full task (the clamp/entropy ladder was built for +1 tasks) and the result is uninformative; no re-tuning without a new registration.

### Amendment (e) — MATCHED demo counts per phase (user, 2026-09-05 11:15; registered 11:25 before any matched-N run reported)

User: "Number of demos is a confound to keep in mind. We really should be testing same vs same unless it's impossible to get ignition with the small number of human demos." Audit of the arms as run:

| comparison | human tapes | machine tapes | matched? |
|---|---|---|---|
| pick (stage 3, rnd300, repeat-1 pilot) | 66 | 58 (teacher success at the same 66 ICs) | count ≈ matched, machine fewer; rows 14.4k vs 7.3k (human tapes are longer) |
| place phase (§2 result of record) | 39 | 63 (one per IC, of 104) | NO — machine 1.6× |
| contact after release (one-armed machine + sub-floor human) | 11 | 25 | NO |
| carrycontact | 21 | 36 | NO |
| end-to-end (d) | 74 | 72 (best per IC) | count matched; stage yields differ by source (nested 3 vs 16) — that is the treatment, disclosed |
| all-data dHv2all vs dDP | 106 | 58 | NO by design (registered as the "gain from failures" arm) |

**Rule of record from now on:** the primary comparison at every phase uses the machine set subsampled UNIFORMLY (seed 0, without replacement; `matched_n.py`, kept files listed in `repeat.json`) to the human tape count; the uncapped machine set becomes a disclosed secondary. The human count is never increased. If the human count is below the ignition floor (contact: 11), the matched machine arm runs at the same count anyway and the pair is reported as sub-floor, both arms; the one-armed 25-demo machine result stays as the "what the machine can do with its own yield" row.

Submitted 11:25 (8 seeds each, recipe of record, `TAG=bnormclamp1ent5_n<N>`, `DEMO_OVERRIDE`): **carrycontact dDP n21** (the unmatched 36-demo machine runs 3290199–206 were cancelled before starting; the human n21 runs 3290191–98 are the other arm), **place dDP n39** (matched confirmation of §2; prediction: the matched machine arm stays within |Δ| < 0.10 of human 0.703 on polE MODE — if it drops below human by ≥ 0.10 the §2 null was carried by the extra machine demos and the paper reports the matched number), **contact dDP n11** (vs the sub-floor human n11 runs 3290207–14; both arms sub-floor, reported as such).

### Amendment (f) — policy-generated CONTACT entry bank `polE_contact` (registered 2026-09-05 13:35, before any readout on it)

The contact-after-release phase has only the 11-entry human holdE bank, and every arm saturates it (machine 8 seeds 86/88; first two sub-floor human seeds 22/22). To separate the arms, a harder bank is built the same way as `polE_place`: the 16 place checkpoints of record (dH and dDP `bnormclamp1ent5` s0–7) are run in the PLACE scope on the human `polE_place` bank (148 entries, deterministic), and the state before each placed_v2-granting decision is dumped (`eval_genesis.py --dump-entries`), merged with pseudo-uids 920000+i into `phase_banks/polE_contact.json` — **10 entries per checkpoint, uniform, seed 0 (160 entries; the first dump yielded 112 placed states from one checkpoint, so the full union of ≈1,700 would make every post-hoc cell hours long; the full dumps stay on disk)**. Scoring: every contact checkpoint (machine n25 one-armed, human n11, machine n11) is evaluated on it post hoc (sample and mode), one episode per entry; `restore_failed` counts as failure. **Statistic of record for the contact phase becomes polE_contact MODE** (holdE stays as the ceiling check); prediction: |Δ(human n11 − machine n11)| < 0.10; the one-armed machine n25 row is descriptive. The bank mixes entries produced by human-arm and machine-arm place policies (8 + 8 checkpoints), so it is symmetric by construction; the per-source split is reported as a covariate.

### Verdicts read out 2026-09-06 (all numbers in PHASE_RESULTS_2026-09-05.md)
- **(e) matched counts:** place human 0.703 vs machine-39 0.647 (Δ +0.056, p 0.227) — prediction met; contact-after-release human-11 0.593 vs machine-11 0.602 (Δ −0.009, p 0.841) — met; carrycontact human-21 0.807 vs machine-21 0.796 (Δ +0.011, p 0.348) — met. The uncapped machine place arm (63 demos, 0.709) becomes the disclosed secondary; the +0.062 it gained from 24 extra demos is itself a count effect.
- **(f) polE_contact bank:** discriminates (0.59–0.60 vs holdE at ceiling); zero restore failures; adopted as the contact statistic of record.
- **(d) end-to-end, wave 1 (4 v 4):** P1 met for the human arm only (machine 1/4 seeds ≥ 0.5, mean 0.500); P2 not met at contact (+0.117, p 0.49); P3 marginally not met (human nested 0.208). Wave 2 (seeds 4–7) submitted 22:40 per the registered trigger; the 8 v 8 result decides. No claim from wave 1 beyond "every stage points human ≥ machine, none significant".
- **(c′) repeat-1:** P1 met (read out 2026-09-05 14:55).
- **(d) end-to-end, wave 2 (8 v 8, read out 2026-09-07 08:40):** P1, P2 and P3 all met at 8 seeds per arm; the wave-1 contact gap (+0.117) fell to +0.042 (p 0.64) and the signs alternate by stage — no source effect. PHASE_RESULTS §5.1.

### Amendment (g) — stricter contact predicate `contact_push`, logged alongside `contact`; post-hoc re-score (registered 2026-09-07 14:47, before any re-score job was submitted)

User (2026-09-07): "is contact getting credit for touching the gripper to the goal can? contact is ideally through a slide where the gripper and the goal can are on opposite sides of the pick-can." The predicate of record (`genesis_can_env.step`) is `contact` = picked ∧ solver contact between the pick-can and the goal can ∧ ee_x < can_x. The gripper alone touching the goal never counts, but the far-side test is along robot-x only and gripper–goal contact is not excluded.

- **New predicate (LOGGED ONLY; `contact` and every reward/termination rule are unchanged).** `contact_push` = picked earlier in the episode ∧ pick-can↔goal solver contact this sim step ∧ **dot(ee_xy − can_xy, goal_xy − can_xy) < 0** (eef link on the far side of the pick-can along the can→goal line, table plane) ∧ **no gripper↔goal solver contact this sim step** (`goal.get_contacts(kinova)` empty). Sticky once true. Logged with it: `contact_frame` / `contact_push_frame` (sim step of first grant), `contact_gripper_goal` (any gripper–goal contact on a pick-can/goal contact frame after the pick), `contact_farside` (any such frame with dot < 0). Implemented in the repo (`baselines/genesis_can_env.py`, `baselines/rl/full_env.py` `_granted` bookkeeping, no reward), the private cluster copy (`$W/gp_root`), the r2dreamer adapter (`log_contact_push` next to `log_contact`; `log_*` keys are excluded from the encoder and unmatched by `mlp_keys: 'state'`, so no model dim changes) and the evaluator (`stages` gains `contact_push`; per-episode `contact_diag` with the failure reason; summary `contact_push_diag`). Patch scripts `~/wm_fix_2026-09-03/contact_push_patch.py`, `contact_push_r2d_patch.py`. Unit check `baselines/diagnostics/contact_push_check.py` (scripted states, CPU).
- **Re-score (CPU, ≤ 8 jobs `cpsc_rescore_L0-7`, `cpsc_rescore.sbatch`).** The checkpoints of record, same seeds (0), same banks, same evaluator settings, written to NEW dirs `fresh_eval_<tag>_<mode>_cp` (the cells of record are never overwritten): contact-after-release human n11 `s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s0-7` and machine n11 `..._dDP_bnormclamp1ent5_n11_s0-7` on `polE_contact` (160) and holdE (11); carrycontact human `s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s0-7` and machine n21 `..._dDP_bnormclamp1ent5_n21_s0-7` on `polE_place` (148) and holdE_place (13); end-to-end `full_r2d_state_{dHfull_all,dDPfull}_bnormclampS8ent5_s0-7` on rnd30 and hold15; mode AND sample. Reproduction check: the per-episode `contact` column must equal the cell of record episode by episode (asserted by the comparison script; any mismatch is reported as a finding, not silently absorbed). Statistic: exact two-sided permutation on per-seed `contact_push` counts, human vs machine, per comparison and mode.
- **Predictions (registered).** P1: |Δ(human − machine)| < 0.10 on `contact_push` in every comparison (contact polE MODE, carrycontact polE MODE, end-to-end rnd30 MODE; sample cells reported alongside). P2: the failing fraction (episodes with `contact` but not `contact_push`) is ≤ 0.15 of the `contact` credit, and does not differ between arms by more than 0.05. Disconfirm branches: if P2 fails because of gripper–goal contact (the finger tips protrude past a HELD can, so a straight push can register finger–goal contact — seen in the unit check's gap-0 row), `contact_push` is reported as a stricter *secondary* column and the release-vs-hold distinction is stated per phase; if P2 fails because of the side test, the contact credit of record includes non-slide contacts and the paper says so with the fraction per arm; if P1 fails while `contact` stays null, the source effect lives in HOW contact is made and becomes a stated result, not a re-analysis. `contact` stays the predicate of record for every registered comparison in this plan; `contact_push` is an added column. Doc: `paper/CONTACT_PUSH_2026-09-07.md`.

### Amendment (i) — symmetry controls for the matched place arm and for carrycontact (registered 2026-09-07 16:40, BEFORE submission; from the statistics adversarial review S2)

The machine-policy entry bank `polE_place_dDP.json` (149 entries, from the 8 machine-arm pick policies) was only ever scored on the superseded 63-demo machine place arm; carrycontact has never had a machine-policy-bank cell. Submitted now (CPU, `wmfix_phase_eval_cpu.sbatch`, tag `polEdDP`, sample and mode, new dirs only): place machine-39 s0–7 and carrycontact human s0–7 + machine-21 s0–7 on `polE_place_dDP.json`. Prediction: |Δ(human − machine)| < 0.10 on the machine-policy bank for both comparisons, and |Δ_polE − Δ_polEdDP| < 0.05 (the bank of origin does not carry the null). Registered before any of these cells exist.

### Amendment (g′) — `contact_push` projection corrected to the TOOL point before any re-score readout (registered 2026-09-07 15:08)

ADVERSARIAL_REVIEW_eval_env_2026-09-07.md S2-5 (coordinator, 15:00): `ee` in `genesis_can_env.step` is the WRIST link, ~0.145 m behind the tool; `contact`'s `ee_x < can_x` holds on 332/332 banked states (vacuous) and the amendment-(g) wrist-based far-side test fires on 132/148 HELD pick-grant states and 123/160 just-released states — it would grant on the pure carry it exists to exclude. The first re-score submission (3350229–36, 14:47) was cancelled at 15:05 before any cell finished (0 metrics.json written; nothing read out; wrist-based logs archived under `cpsc_logs_cancelled_wrist/`).

- **Corrected predicate (of record from now on):** `contact_push` = picked ∧ pick-can↔goal solver contact this step ∧ **dot(tool_xy − can_xy, goal_xy − can_xy) < 0 with tool = `GenesisCanEnv.tool_pos()`** (the gripper tip reconstructed from the wrist with the offset calibrated at HARDCODED_START on every reset) ∧ no gripper↔goal solver contact this step. The wrist-based sign is still logged (`contact_farside_wrist`) so the two definitions can be compared per episode. Unit check re-run with the tool point: PASS (the HELD can pushed straight in now fails on BOTH clauses — the tool point sits ahead of a held can's centre — and passes only once the can is ≥ 6 cm ahead of the fingers with no finger–goal contact). A tool-vs-wrist count on every banked state is reported as its own table in the doc (`baselines/diagnostics/contact_push_bank_geometry.py`).
- **S2-4 (bank grip scale) — handled by analysis, not by rebuilding banks (another agent owns the bank fix):** `--dump-entries` stores the raw [−1,1] action as `grip_cmd`; the restore reads physical 0..1 (clipped). The re-score runs the banks AS THEY ARE (the cells of record did); the doc lists the affected pseudo-uids per bank and reports every contact/contact_push table twice: all entries, and the subset whose restore preserves the intended grip (polE_place: `grip_cmd ≥ grip_obs`, the review's criterion; polE_contact: intended and restored both "released" (< PLACE_RELEASE 0.45) — every entry, with the strict |restored − intended| ≤ 0.05 subset as a sensitivity row; holdE banks are human/physical and unaffected).
- Predictions P1/P2 of amendment (g) stand unchanged for the corrected predicate; they are evaluated on the corrected re-score only. Everything else in (g) (cells, seeds, banks, new `_cp` dirs, reproduction assertion) unchanged.

### Amendment (h) — PLACE phase for Diffusion Policy and RLPD (registered 2026-09-07 15:10, BEFORE any run; the user wants Pick and Place for all three learners)

Repeat the r2dreamer place comparison (§1–§4, amendments (a)–(f); PHASE_RESULTS §2/§2.y) with the two other learners so the paper has a learner × source Place table.
- **Learners, recipes of record (not tuned).** DP = lerobot Diffusion Policy, `cluster/sbatch_dp.sh` recipe (state-only 17-dim obs split 8 + 9, ABSOLUTE window-end joint targets + grip 0..1 at fps 7.5, 100k grad steps, batch 64, K = 5 checkpoints, LAST = 100k), executed hold-4 through the env's own delta_joint integrator (q* → clip((q* − target)/(4·0.025)), cap 0.025 / leash 0.125 — wandb_eval.py's DP rule). RLPD = `cluster/sbatch_rlpd.sh` recipe (SAC subclass, UTD 10, E10/Z2, LN critics, γ 0.99, 50/50 demo batches, delta_joint cap 0.025 / leash 5×, action_repeat 4, `delta_ref target`), training horizon 600 sim steps (the place cap), **budget 1e6 sim steps = 250k decisions** (the WM place arm's physical budget; the RLPD PICK recipe of record was 100k decisions = 4e5 sim steps, so this is 2.5× that in decisions — disclosed; the K = 5 archive (`ckpt_040` = 100k decisions) makes the pick-budget read available post hoc without a new run). Tips terminate without penalty (`phase_sparse`), +1 once at the placed_v2 grant, exactly the WM cells' env (the private `$W/gp_root/baselines/rl/full_env.py` ported verbatim into `baselines/rl/full_env.py`; shelf band `shelf_top_z + [0.01, 0.07]` = 0.18–0.24 m in `gc_kp4_riser3_shelf6`, read from `GENESIS_SIM_VARIANT`).
- **Sources (matched 39 v 39).** Human = `demos_state/dH_place` (the 39 human full-task tapes that reach placed_v2, cut [k_pick, k_placed_v2]; pruned ≡ raw from the pick grant on, so ONE human arm — §1). Machine = `demos_state/dDP_place_n39` (amendment (e): uniform subsample, seed 0, of the 63 one-per-IC machine segments; the matched set of record). RLPD trains on those r2dreamer-native segment rows VERBATIM (`baselines/rl/place_demos.segment_transitions`: (state_t, delta action, r, state_{t+1}, done), one +1 per segment — the same rows the WM trained on, no re-encoding). DP trains on the SAME cuts of the FULL contract-v1 tapes (`place_demos.py cut` at the manifest's k_pick..k_placed_v2, absolute targets) converted with `convert_to_lerobot.py` at fps 7.5: 39 human tapes / 4265 rows, 39 machine tapes / 4727 rows, cross-checked row-for-row against the segments (`place_demos.py check`: first/final state, actions_delta, one +1 — CHECK-OK both).
- **Reset distribution (RLPD only).** The HUMAN pick-grant bank `phase_banks/human_place.json` (64 entries) for BOTH arms (the head agent's instruction). This differs from the WM place runs, whose machine arm reset from the 179-entry machine bank (PHASE_RESULTS §6 asymmetry 2): that asymmetry is removed here at the price of the machine demos' start states sitting slightly off the online reset distribution — disclosed. DP has no reset distribution.
- **Evaluation.** `baselines/eval_place.py`, one process per (bank, mode), LAST checkpoint, 600 sim steps, banks `holdE_place` (13) and `polE_place` (148), **every entry exactly once, in order; a failed restore is a failure** (the r2dreamer evaluator drew entries with replacement and substituted failing ones — adversarial review 2026-09-07 S1-3; the difference is disclosed and the WM cells are being re-scored by another agent). RLPD: sample AND mode (sampled = the statistic of record per the user's 09-07 decision, mode reported beside it); DP: sample (seed 0; no mode exists for a diffusion policy). Per review S2-4 the policy-generated banks were dumped with normalised grips; the evaluator refuses a bank whose grips leave [0,1] or a `polE*` bank without a `bank_version` field, so the polE cells run only on the REBUILT physical-grip `polE_place.json` (bank of record for these cells); holdE is a human tape bank (physical grips) and is unaffected. Cap/leash parity with training is asserted from the checkpoint sidecar (new fields `delta_cap`/`delta_leash`).
- **Statistic and gates.** Per learner: polE success count per seed, n = 8 v 8 (seeds 0–7), exact two-sided permutation; holdE secondary; learnability floor as §4 (≥ 0.5 on holdE in ≥ 3/8 seeds per arm, else "not learnable at this budget" with the yields and the archived checkpoints).
- **Predictions (registered).** P1: |Δ(human − machine)| < 0.10 on polE for BOTH learners (RLPD sampled, and mode; DP sampled) — the WM null carries over. P2 (secondary): DP place ≥ 0.5 on polE (its in-distribution pick strength; place starts from a held can). P3: RLPD clears the learnability floor at this budget on both arms. Disconfirm branches: a learner that fails P3 on ONE arm only is a demo-source effect for that learner (reported, then a matched re-run before any claim); on both arms the recipe is not transferable to the phase (uninformative, no re-tuning without a new registration).
- **Disclosed by construction.** Budget units differ across learners (DP grad steps, RLPD decisions, WM sim steps); DP's absolute action parametrisation is converted to the delta MDP only at eval; entry-restore survival is reported per cell; the polE bank is the rebuilt one (its raw-grip predecessor is kept as `*_rawgrip.json`).
- **Jobs.** `cluster/sbatch_rlpd_place.sh` / `cluster/sbatch_dp_place.sh` (+ `cluster/place_eval_cells.sh`, the re-runnable 4-cell eval stage): `-p preempt --qos=preempt --requeue --nice=2000 --exclude=pax077 --constraint=l40s|a100|l40|h200`, names `pl_rlpd_<arm>_s<seed>` / `pl_dp_<arm>_s<seed>`, 2 learners × 2 arms × 8 seeds = 32 jobs; a requeued run restarts clean (RLPD) / resumes from its last lerobot checkpoint (DP, the launcher of record's mechanism). Code runs from a private clone `$LAB/gp_place` at the commit named in the submission report (the shared checkout is untouched: the pending robomimic jobs run from it). Datasets: `$LAB/genesis_pickaplace/baselines/matched_w3/{dH_place,dDP_place_n39}` (+ `lerobot/`), segments `$W/demos_state/{dH_place,dDP_place_n39}` unchanged. Smokes (2k steps, holdE ≤ 10 entries, both learners) precede the submission and are logged in the report, not here.

### Amendment (k) — in-training-distribution evaluation set `spots60` (registered 2026-09-07 18:05, BEFORE any evaluation on it; user 17:55: "don't drop the three-spot proposal, that's the in-training-distribution set")

`baselines/eval_ics_spots60.json`: 60 starts = 20 random draws per marked can spot from the Gaussian (mean, sd) of the success-labelled human can starts at that spot (`trial_placements.json`, 74 uids: spot 0 centre (0.465, 0.098) sd (0.033, 0.033); spot 1 (0.461, −0.031) sd (0.019, 0.016); spot 2 (0.439, −0.176) sd (0.020, 0.024)); can z 0.113 upright, static goal (0.672, −0.221); seed 20260907; rnd schema (uid null). Role: the **in-training-distribution test set** (random draws from the distribution the demonstrations came from, never a hand-picked subset); the random box set (rnd30/rnd300, reported stratified by support) stays the **out-of-distribution** set; hold15 is a training-start set and stays internal. Statistic: sampled actions (and deterministic alongside), LAST checkpoint, one episode per start, 8 seeds per arm. Predictions: every learner scores higher on spots60 than on rnd30; the human-vs-machine null (|Δ| < 0.10) holds on spots60 for the world model and RLPD at pick; DP's pruned-human arm exceeds 0.85 on spots60 (its dead starts lie outside the spots).

### Amendment (j) — evaluation fixes from the eval/env adversarial review, physical-grip bank rebuild, and a CPU re-score of every affected world-model cell (registered 2026-09-07 15:25 local / 15:25 cluster, committed BEFORE any re-score job; brief `~/wm_fix_2026-09-03/agent_brief_eval_fixes.md`)

Source: `ADVERSARIAL_REVIEW_eval_env_2026-09-07.md` (87a6dba) S1-1, S1-2, S1-3, S2-4, S3-6, S3-7. Patch of record `cluster/eval_fixes/eval_fixes_patch.py` (exact-anchor, idempotent, asserts every anchor is unique), applied identically to the repo, `$W/gp_root`, `$W/r2dreamer_fix` and the local mirror `~/wm_fix_2026-09-03/cluster_r2d` (md5-verified). Nothing in it changes a reward, a termination or a stored training row; every running job is one Python process with the old modules already loaded, so none is affected (a requeued job would run the new code and is identifiable by the `eval_fixes` stamp below).

**Fixes**
1. **Pinned entries in every bank scope (S1-3).** `eval_genesis.reset_to_uid` now calls `env._env.reset(options={'uid': uid})` in the place scope too (it called `env._env.reset()` unpinned: uniform draws WITH replacement over the whole bank, ~94 of 148 distinct entries per cell, and silent substitution of any entry that failed to restore, while contact/carrycontact pinned — the whole "restores in one scope but not the other" mechanism). A pinned entry that does not survive the restore raises → `restore_failed`, counted as a failure, never substituted. Per episode: `restored_uid`, `entry_frame`; summary `pin_stats`; an assertion that every enumerated uid was restored (or recorded as restore_failed/hang). The video name `ep<i>_uid<uid>_<outcome>.mp4` therefore names the restored entry.
2. **Physical grip in dumped banks (S2-4).** `GenesisPick.grip_phys(a) = (clip(a[6], −1, 1) + 1) / 2` is the ONE policy→physical grip map (the adapter's `step` now calls it — numerically identical to before; `pick_env.denormalize_action` is the same affine map). `--dump-entries` stores `grip_cmd = grip_phys(a)` plus `grip_cmd_raw`, `grip_units='physical01'`, `bank_version='physgrip_2026-09-07'`. The banks `polE_place.json` (148), `polE_place_dDP.json` (149) and `polE_contact.json` (160) are rebuilt by `cluster/eval_fixes/rebuild_banks_physgrip.py` from the per-checkpoint dump JSONs (each merged entry is re-derived from its dump entry — same source checkpoint, ic_index, frame, qpos and stored grip — then converted with exactly that map; schema unchanged, `bank_version` per entry so `eval_place.py` (h) accepts it); the byte-identical originals stay as `*_rawgrip.json`. Per-bank counts (entries changed, negative raw values, commanded-below-measured before/after, released-below-0.45 before/after) and the restore-survival of every entry before vs after (the env's own restore, one world per scope, CPU: `bank_restore_check.py`) are reported in `EVAL_FIXES_2026-09-07.md`. Coordination: the (g)/(g′) contact_push lanes 3350666–74 (started 15:13–15:14 cluster time, protocol "banks as they are") read the canonical file names; their cell list and generator were pointed at the `_rawgrip` copies (byte-identical), and the canonical names are swapped to the rebuilt banks only after every polE cell of those lanes has ended (their later cells are hold banks / no bank).
3. **Honest nested for the full scope (S1-1).** `nested_honest` = `GenesisCanEnv._nested()` (100 settle steps at the episode end, centre distance ≤ NESTED_TOUCH_DIST, picked, both upright — the predicate the DP/RLPD path reports at its horizon), computed by the adapter at termination AFTER the terminal obs/reward are taken (log key `log_nested_honest`, scope=full only; the training reward and terminal are untouched) and by the evaluator at its own horizon — exactly one settle per episode, after the last decision. The old per-step `nested` (sticky contact ∧ grip commanded open ∧ both upright, terminating) is reported as `nested_proxy`; the `stages.nested` key and the per-episode `outcome` taxonomy are KEPT equal to the proxy so the (g) reproduction check (per-episode contact / outcome / steps) still holds against the cells of record; `outcome_honest` ∈ {nested_honest, proxy_only, tipped, timeout}.
4. **`placed_v2` in the full scope (S1-2).** The release predicate (grip command < 0.45 ∧ shelf footprint ∧ the WORLD's shelf band ∧ tilt < 20°, sustained 10 frames) is computed in scope=full too, LOGGED ONLY (no reward, no termination); `placed` is kept and flagged stale (`stage_notes`).
5. **Small (S3-6, S3-7).** `--ic-skip` stale-stages bug: already fixed in the (g) patch, verified. `FullTaskEnv.__init__` asserts that the env-var-derived `shelf_top_z` equals the BUILT world's shelf top (the Box entity of size BOX_SIZE, base-link z + half height) on every construction path.
6. **Stamps.** Every summary carries `eval_fixes='j'`, `entries_pinned`, `bank_path`, `bank_sha256`, `bank_version`, `world_shelf_top_z`, `stage_notes`.

**Re-score** (CPU only, `cluster/eval_fixes/rs2_rescore.sbatch`, 4 lanes × 4 evals = 16 concurrent, job prefix `rs2_`; NEW dirs `fresh_eval_<tag>_<mode>_v2`, the cells of record are never overwritten; same `latest.pt`, seed 0, evaluator flags of record; 320 cells from `rs2_make_cells.sh`): place human `s2_r2d_place_state_dH_bnormclamp1ent5_s0-7`, machine-39 `..._dDP_bnormclamp1ent5_n39_s0-7`, machine-63 `..._dDP_bnormclamp1ent5_s0-7` × {polE (rebuilt), polEdDP (rebuilt), holdE (pinned; the record drew its 13 entries with replacement)}; carrycontact human s0-7 / machine-21 s0-7 × {polE, polEdDP} (rebuilt); contact-after-release human sub-floor s0-7 / machine n11 s0-7 / machine n25 s0-7 × polE_contact (rebuilt); end-to-end `full_r2d_state_{dHfull_all,dDPfull}_bnormclampS8ent5_s0-7` × {hold15, rnd30}; mode AND sample everywhere. Before the re-score, `rs2_prep.sbatch`: the restore-survival tables and an evaluator smoke (pinned tiny bank with the five carrycontact restore-failures + two controls; a physical-unit dump; two full-scope episodes reporting nested_honest / nested_proxy / placed_v2).

**Statistic.** Unchanged (§4, (d), (e), (f)): per-seed counts, exact two-sided permutation, 8 v 8, both modes; primary cells = polE MODE (place), polE MODE (carrycontact), polE_contact MODE (contact), rnd30 MODE (end-to-end).

**Predictions and decision rules (registered).**
- P1 place (matched machine-39 vs human, pinned entries + physical grip): the (e) null survives iff |Δ(human − machine)| < 0.10 on polE MODE; likewise on polEdDP — the (i) prediction is evaluated on these `_v2` cells (the (i) cells, produced under the old protocol, are reported as the old-protocol row). Machine-63 is descriptive. If |Δ| ≥ 0.10 the §2/§2.y numbers are superseded by the re-scored ones and the paper states that the null was carried by the evaluation bugs.
- P2 carrycontact (was pinned; physical grip is new): the (e) null survives iff |Δ| < 0.10 on polE MODE; `restore_failed` is expected to change from 5/148 and is reported — if the five failures vanish under physical grip, the units bug was their driver (the review's hypothesis), otherwise not.
- P3 contact-after-release (fingers restored at the intended released command instead of fully open): the (f) prediction |Δ(human n11 − machine n11)| < 0.10 on polE_contact MODE; absolute rates may move either way (no directional prediction); machine n25 descriptive.
- P4 end-to-end: (d) P2 (|Δ| < 0.10 at every stage either arm reaches ≥ 0.2) is re-derived on picked / placed_v2 / contact / nested_honest, and (d) P3 (nested < 0.2 for both arms) on nested_honest, with the proxy column beside. No directional prediction for nested_honest vs nested_proxy (a held can touching the goal counts honest but not proxy; a released can knocked away counts proxy but not honest). Reproduction guard: per-episode `outcome`, `steps`, `stages.contact` and `stages.nested` of every `_v2` end-to-end cell MUST equal the cell of record (no bank; identical physics up to the last decision) — any mismatch is a finding. Bank cells restore different states by construction (pin / grip), so no per-episode identity is asserted for them.
- Verdict per comparison in `paper/EVAL_FIXES_2026-09-07.md` (cell of record vs re-scored: per-seed lists, arm totals, Δ, exact p, both modes): "the null survives" iff the primary MODE cell meets the registered |Δ| < 0.10; `PHASE_RESULTS_2026-09-05.md` gets a dated §9 pointing to it (earlier sections untouched).

### Amendment (l) — SLIDE/END-TO-END success is contact-after-release; `nested` is folded into it (user, 2026-09-07 17:05; registered before any job)

User: "train for contact (with the can placed on the shelf and not held by the gripper), rather than nested. Treat it as one category."

**Predicate `slide_success`** (new; logged beside the existing keys, which are not changed): sticky-true from the first step at which ALL of
1. `picked` was granted earlier in the episode (unchanged precondition),
2. the solver reports contact between the pick-can and the goal can,
3. the gripper is commanded open (`grip_cmd < 0.3`, the threshold the existing nested proxy uses),
4. the pick-can centre lies in the shelf footprint and its tilt is < 20° (the `placed_v2` geometry, i.e. it is standing on the shelf, not held or falling),

hold simultaneously, sustained for 3 decisions. Rationale: "on the shelf and released, touching the goal" is the task as demonstrated; it makes release load-bearing, which the old `contact` did not require and which the carrycontact re-score showed ~69 % of contact credit was exploiting (the can still in the grasp). It also removes the `nested_proxy`/`nested_honest` split (amendment (j) S1-1) from the headline: `nested` becomes a sub-case reported for continuity, not the target.

**Scope of the change.** (a) End-to-end: `slide_success` is the success statistic of record; `nested_honest`, `nested_proxy`, `contact` and `contact_push` stay as reported columns. (b) Slide phase (`scope='contact'`, entry = a released placed-on-shelf state): success becomes `slide_success` rather than bare `contact`; the entry definition is unchanged. (c) Carrycontact keeps bare `contact` and is now explicitly the "contact by any route incl. still held" control against which `slide_success` is read. (d) Rewards are NOT changed by this amendment: the world-model end-to-end runs of record keep the staged reward (user: do not re-run the world model yet), and any new RLPD end-to-end arm uses the same staged reward so the two are comparable.

**Re-scoring, not re-running:** every existing world-model contact / carrycontact / end-to-end checkpoint is re-scored on CPU with `slide_success` added; no training is repeated. Prediction: on the end-to-end rnd30 cells `slide_success` is at or below `nested_honest` for both arms, and the human-vs-machine difference stays within 0.10; on the slide phase the human-vs-machine difference likewise stays within 0.10 but both arms fall well below their bare-`contact` rates.

#### Addendum to amendment (h) — checkpoint retention (registered 2026-09-07 17:35, before the resubmission; storage only, no statistic changes)

Between 16:00 and 17:00 the shared filesystem hit 0 bytes free and every queued job on the cluster was cancelled at launch (exit 0:53, 0 s elapsed), including all 32 place runs and all 44 pick re-evals; the coordinator freed 434 GB and set a standing rule: a launcher keeps only the final checkpoint, plus a selected one where the protocol needs it. Measured cause and fix, disclosed here because the (h) registration named a K = 5 archive:
- **DP.** One lerobot checkpoint is 2.85 GB (949 MB `pretrained_model` + 1.9 GB `training_state`); `save_freq = STEPS/5` would have held 5 per run × 16 runs ≈ 228 GB. Now `save_freq = STEPS/2` (at most two numbered checkpoints exist *during* a run, giving the preempt queue one mid-run resume point), and after training every checkpoint except the final is deleted together with the final's `training_state` — 949 MB per run at rest. A requeue after the budget was reached skips training instead of resuming (`TRAIN-DONE-ALREADY`). The evaluated artefact is unchanged: the LAST (100k) checkpoint's weights.
- **RLPD.** Periodic snapshot zips are off (`train_rlpd.py --ckpt-every 0`, new flag; default 50 000 keeps every earlier run byte-identical) and the archive is `--ckpt-fracs 0.4,1.0` instead of K = 5: **ckpt_100 = LAST (the statistic of record) and ckpt_040 = 100k decisions (the RLPD pick-budget read that (h) registered)**. 3 × 12 MB per run. The three dropped fractions (0.2/0.6/0.8) were never part of any registered statistic.
Nothing else about (h) changes: same demos, banks, budgets, seeds, statistic and predictions. Before each resubmission batch the launcher operator checks `df -h /cluster/tufts/shortlab` and holds below 150 GB free (434 GB at resubmission).

### Amendment (g'') — (g') re-score restarted after the cluster disk-full outage; frozen code copy; `slide_success` added as a logged column (registered 2026-09-07 20:16)

Between 16:00 and 17:00 `/cluster/tufts/shortlab` reached 0 bytes free and every cluster job died, including the (g') lanes 3350666-74 (85 of 192 cells had finished; they are preserved under `$W/cpsc_v1_backup/` for provenance and are NOT used for any reported number). Not a code fault; the filesystem is back to 444 GB free (checked before resubmitting, registered floor 150 GB).

Three protocol notes for the restarted sweep, all registered before it was submitted:
1. **Frozen code copy.** The shared trees (`$W/gp_root`, `$W/r2dreamer_fix`) are being edited concurrently by the eval-fixes agent (amendment (j), landed 15:26: pinned bank entries, `nested_honest`/`nested_proxy`, `placed_v2` in the full scope, shelf-band assertion). The killed sweep had cells produced on both sides of that edit. The restart runs a snapshot, `$W/cpsc_frozen/` (taken 20:14, includes (j)), so every one of the 192 cells comes from ONE code version and later edits by other agents cannot split it. All 85 pre-outage cells were deleted and are being re-run.
2. **Banks unchanged.** The (g)/(g') protocol is "banks as they are": the cell list points at `phase_banks/polE_{place,contact}_rawgrip.json` (byte-identical copies of the banks the cells of record used), so the (j) physical-grip bank rebuild does not enter this sweep. The S2-4 subset tables (doc 3c) remain the way the grip-scale defect is handled here; the rebuilt-bank numbers are the eval-fixes agent's `_v2` re-score.
3. **`slide_success` (amendment (l)) as a logged column.** Requested by the coordinator for these tables. It is implemented in the FROZEN copy only (`~/wm_fix_2026-09-03/slide_success_patch.py`), never in the shared trees: the authoritative implementation belongs to the (j)/(l) patch. Definition as registered in (l): sticky from the first decision at which picked (earlier) AND pick-can/goal solver contact AND gripper commanded open (< 0.3) AND pick-can in the shelf footprint with tilt < 20 deg all hold, sustained 3 consecutive decisions (the per-sim-step conjunction is computed in the env, the decision counter in the adapter, zeroed on every evaluator reset path). Never rewarded, never terminates. If the landed (j)/(l) implementation differs in any detail, THAT one is of record and this column is labelled as the (g'') frozen-copy computation.

### Amendment (l′) — correction to (l): the "sustained 3 decisions" clause is unimplementable in-episode (registered 2026-09-07 20:40, before any `slide_success` cell exists)

My amendment (l) required `slide_success` to hold "sustained for 3 decisions". That clause cannot be evaluated inside an episode and is withdrawn as written: `scope='contact'` terminates on the first contact frame (`full_env.py:766`) and `scope='full'` terminates on the nested proxy (`:817`), which fires at or before slide's own clauses, so a literal implementation returns `slide_success = 0` in every cell for a mechanical reason, not a behavioural one. This was an error in (l), found by the eval-fixes agent before any cell was produced.

**Replacement, registered here:** the four clauses of (l) (picked earlier ∧ pick-can/goal solver contact ∧ gripper commanded open, `grip_cmd < 0.3` ∧ pick-can in the shelf footprint with tilt < 20°) are evaluated over the **first 12 frames of the existing 100-step end-of-episode settle, with the last command held**, and a grant records its route (`sustained` if the clauses already held on consecutive in-episode decisions, `settle` if they first co-occur during the settle). The settle already existed and its budget and measurement point are unchanged, so `nested` and `nested_honest` stay bit-identical to their recorded values; only the new column is added. Rationale for using the settle rather than lengthening episodes: it tests exactly what the definition means physically — the can is standing on the shelf, released, and touching the goal, and stays that way when the policy stops acting.

**Disclosed side effect (cross-agent):** because the patched evaluator is shared, phase cells re-scored after 2026-09-07 20:30 carry this 100-step post-episode settle where their cells of record did not. `contact`, `contact_push` and the scope success keys are computed at the same points as before, so per-episode reproduction should be exact; any mismatch from episode ≥ 1 in a re-score should suspect the settle first. Reported in `EVAL_FIXES_2026-09-07.md` §6.

### Amendment (g''') — the (g'') sweep re-frozen on the landed (l') implementation of `slide_success` (registered 2026-09-07 20:21, before resubmission)

The eval-fixes agent landed `slide_success` in the shared trees at ~20:18 with the (l') correction: both scopes where it is the statistic of record terminate at the FIRST contact frame, so a "sustained 3 decisions" window can never close inside the episode; (l') evaluates the four clauses over the first 12 frames (3 decisions x action_repeat 4) of the post-episode settle with the last command held, and records the route (`sustained` = the window closed inside the episode, `settle` = only during the held continuation). My (g'') frozen copy carried my own reading of the original (l) wording, which for the contact and carrycontact scopes would have logged all-zeros. Action, taken before any cell of the restarted sweep finished (all cells deleted, 0 reported numbers affected):

- `$W/cpsc_frozen` re-taken at 20:20 from the shared trees, so the frozen copy now carries the AUTHORITATIVE (j) + (l') implementations plus the (g')/(g'') `contact_push` code; my private `slide_success_patch.py` is withdrawn and not used. Lanes resubmitted from that snapshot.
- Consequence for the (g) reproduction check, disclosed: the (l')/(j) implementation adds ONE post-episode settle (100 scene steps, the `_nested()` settle, now shared with the slide window) to the phase scopes, which the cells of record did not run. Episode 0 of every re-scored cell must still match the record on `contact`, `outcome` and `steps`; from episode 1 the settle changes the world state carried into the next restore, so a mismatch is expected and is NOT evidence about `contact_push`. The pre-outage sweep already showed the same scope is not bit-reproducible across compute nodes either (doc S7). The reproduction verdict is therefore reported per episode-0 and as an aggregate-rate comparison, not as a bit-identity claim.
- Columns reported side by side in `paper/CONTACT_PUSH_2026-09-07.md`: `contact` (predicate of record), `contact_push` ((g') geometric diagnostic), `slide_success` ((l'), with its sustained/settle route split).

### Amendment (m) — CONTACT / SLIDE phase for Diffusion Policy and RLPD (registered 2026-09-07 21:20, BEFORE any run; user released the hold on Slide, wants Pick+Place+Slide for all three learners)

Mirrors amendment (h) exactly, one phase later. `scope='contact'`: the episode starts from a banked `placed_v2` state (the can standing released on the shelf) and the policy must slide it into the goal can.
- **Sources (matched 11 v 11, both SUB-FLOOR).** Human `demos_state/dH_contact` = the 11 human tapes whose contact follows a release (989 rows); machine `demos_state/dDP_contact_n11` = the registered matched subsample (seed 0) of the 25 one-per-IC machine contact segments (580 rows). Both are below the §4 floor of 20, so — exactly as the world-model contact pair (PHASE_RESULTS §3) — **the pair is reported as sub-floor: no learnability claim, and the comparison is descriptive.** RLPD trains on those r2dreamer-native segment rows verbatim (`place_demos.segment_transitions`); DP trains on the same cuts of the full contract-v1 tapes (`place_demos.py cut --phase contact`, rows [k_placed_v2, k_contact], absolute targets) converted at fps 7.5, cross-checked row-for-row against the segments.
- **Reset distribution (RLPD).** The HUMAN contact bank `phase_banks/human_contact.json` (39 released-on-shelf entries) for BOTH arms, as in (h). Disclosed: the machine arm's demonstrations therefore start slightly off its own reset distribution. DP has none.
- **Reward is unchanged** (amendment (l)(d)): the env pays +1 and terminates on its bare `contact` predicate. `slide_success` is never a training signal.
- **Statistic of record: `slide_success`** as registered in (l) and corrected in (l′) — picked earlier ∧ pick-can↔goal solver contact ∧ grip commanded open (< 0.3) ∧ can in the shelf footprint with tilt < 20°, evaluated over the first 12 frames of the existing 100-step end-of-episode settle with the last command held, route (`sustained` / `settle`) recorded. **Not reimplemented here:** `baselines/eval_place.py --scope contact` calls the eval-fixes agent's landed `GenesisCanEnv.end_of_episode()` once per episode and records its dict verbatim (the evaluator asserts the method exists). Bare `contact` and the settled `nested` are reported alongside for continuity.
- **Evaluation banks.** `holdE_contact` (11 human placed states, in-distribution) and `polE_contact` (160 policy-generated placed states, the discriminating bank). **The bank file used is `polE_contact_physgrip.json`** — the eval-fixes agent's rebuilt physical-grip bank (`bank_version=physgrip_2026-09-07`, sha 219cb48e70d3bf9c, EVAL_FIXES §2). Checked 2026-09-07 21:00 and disclosed: the canonical `polE_contact.json` is currently byte-identical to `polE_contact_rawgrip.json` (grips −0.991…−0.127, no `bank_version`), i.e. the raw normalised-grip original, and the same is true of `polE_place.json` / `polE_place_dDP.json`; the evaluator refuses such a bank by construction, so every polE cell of mine names the `*_physgrip` file explicitly. `spots60` does NOT apply to this phase (it is a pick-scope IC set — can on the table, no entry bank).
- **Runs.** 8 seeds × 2 arms × 2 learners = 32. RLPD: recipe of record, 250k decisions (= 1e6 sim steps), horizon 600, sampled AND deterministic cells, LAST checkpoint. DP: recipe of record, 100k grad steps, sampled (no deterministic mode exists), LAST checkpoint. Launchers `cluster/sbatch_{rlpd,dp}_contact.sh` + `place_eval_cells.sh PHASE=contact`, `--nice=8000` (behind the robomimic recovery and dv3 G3), the (h)-addendum checkpoint retention, and the 150 GB free-space hold.
- **Predictions (registered).** P1: |Δ(human − machine)| < 0.10 on polE_contact `slide_success` for BOTH learners (the null of every phase so far, now at n = 11 demos per arm). P2 (from (l′)): both arms' `slide_success` sits well below their bare-`contact` rates, and the `settle` route dominates `sustained`. P3: no floor claim — with 11 demonstrations a failure of either arm is reported as sub-floor, not as a demo-source effect. Disconfirm branch: if one arm alone collapses (< 0.1 where the other is > 0.3) that is reported as a source effect **specific to the sub-floor regime**, and any claim waits for a matched re-run at a higher count, which the human yield cannot supply.

### Amendment (n) — END-TO-END (full task) for Diffusion Policy and RLPD (registered 2026-09-07 21:20, BEFORE any job; the user wants the three-learner full-task table)

The world model already has an 8-v-8 end-to-end result (PHASE_RESULTS_2026-09-05.md §5/§5.1, amendment (d)). This
amendment runs the SAME comparison for the other two learners so the paper has a learner x source table for the whole
task, not only for the phases. Design copied from (d) except where stated; recipes are the ones of record and are not
tuned.

- **Task.** `scope='full'` of `baselines/rl/full_env.py` (the repo copy is byte-identical to `$W/gp_root`'s and to
  `$LAB/gp_place`'s: md5 896f270be6135c4c2fde62a47125b6b1; `genesis_can_env.py` 6d4f1c32b61dae52b6fe61b71ff84ccd),
  world `gc_kp4_riser3_shelf6`, reset from the pick-scope initial conditions (`FullTaskEnv.reset` draws a success uid:
  can on the table, arm at home) for BOTH arms of both learners, horizon 1200 sim steps = 300 decisions at
  action_repeat 4. Reward = the STAGED sparse ladder `STAGE_REWARD` (picked 1 / placed 1 / contact 2 / nested 4, each
  paid once; the nested proxy terminates; tips terminate with TIP_PENALTY 0.0) — identical to (d), so the three
  learners are like-for-like on reward. The user deferred the sparse +100 variant and instructed that the world model
  not be re-run; DP ignores reward entirely, so this matters only for RLPD.
- **Sources (74 human vs 72 machine, the (d) sets verbatim).** Human `dHfull_all` = every full-task tape of the 74
  success uids (`$W/demos_state_full/dHfull_all`, 74 tapes, Sigma recorded reward 118, 64 picks / 10 no-picks); machine
  `dDPfull` = the 195-tape dDP full-task harvest reduced to ONE tape per IC by highest recorded reward sum
  (`--one-per-ic-best`, 72 tapes, Sigma 206, 70 picks / 2 no-picks). RLPD trains on those r2dreamer-native segment rows
  VERBATIM (`baselines/rl/full_demos.py segment_transitions_full`: (state_t, delta action_{t+1}, r_{t+1}, state_{t+1},
  is_terminal_{t+1}) — the same rows the world model trained on, no re-encoding, staged rewards kept as recorded). DP
  trains on the SAME tapes as FULL contract-v1 tapes with ABSOLUTE window-end joint targets (`full_demos.py select`
  matches each segment to its source tape by the recorded rollout `uid` and asserts T == n + 1 on every tape; 74/74 and
  72/72 matched) converted with `baselines/convert_to_lerobot.py` at fps 7.5. No pruning on either arm, both learners.
- **Seeds / budgets.** 8 seeds per arm per learner = 32 runs. RLPD 1e6 sim steps = 250 000 decisions (the (h) place
  budget); DP 100 000 gradient steps (the recipe of record). Only the final checkpoint is kept (RLPD also keeps
  `ckpt_040` = 100k decisions, the pick-budget read of (h); DP's `training_state` is deleted), and each launcher holds
  the registered 150 GB free-space floor before it trains.
- **Evaluation.** LAST checkpoint, fresh process, `baselines/eval_e2e.py` on `FullTaskEnv(scope='full')` in the same
  world at 1200 sim steps, ONE episode per start, in order. IC sets: `hold15` (training starts, in-distribution,
  internal — REVIEW_GUIDE §8 item 7: it is not held out), `rnd30` (out-of-distribution random box; reported stratified
  by training support per `DP_PRUNED_GAP_2026-09-07.md` §0.4), `spots60` (`baselines/eval_ics_spots60.json`, the
  registered in-training-distribution set of amendment (k)) in job; `rnd300` post hoc on CPU. Action selection:
  sampled AND deterministic for RLPD; DP is sampled by construction (no deterministic mode for a diffusion policy) —
  disclosed. **Success-by-stage** columns, every one taken from the env, none re-implemented: `picked`, `placed`
  (legacy, stale base-world band — reported as stale), `placed_v2` (amendment (j) S1-2, logged in the full scope),
  `contact`, `contact_push` ((g')), `slide_success` ((l)/(l'), computed by `GenesisCanEnv.end_of_episode()`),
  `nested_honest` (the settled predicate from the same single end-of-episode settle) and `nested_proxy` (the sticky
  training proxy the episode terminates on). The evaluator calls `end_of_episode()` exactly once per episode, after the
  last decision, the way the world-model adapter does.
- **Statistic of record.** `slide_success` (amendment (l): on the shelf, released, touching the goal) with SAMPLED
  actions on `spots60` — sampled because the user made sampled actions the statistic of record on 2026-09-07 and DP has
  no other mode; `spots60` because (k) registered it as the in-training-distribution test set. Per-seed counts, exact
  two-sided permutation, 8 v 8, per stage and per cell; the minimum detectable effect implied by the observed
  per-seed spread is reported with every null (the world-model end-to-end MDE is ~0.21). `rnd30` MODE with the §5.1
  stage columns is reported as the cell comparable with the world-model row of record.
- **Predictions (registered).** P1 (learnability): for each learner, EACH arm reaches `picked` >= 0.5 on spots60
  sampled in >= 3 of 8 seeds. P2 (main hypothesis, consistent with pick / place / contact / carrycontact / (d)):
  |Delta(human - machine)| < 0.10 on the statistic of record for BOTH learners, and at every stage that either arm
  reaches >= 0.2. P3 (re-derivation of (l) for these learners): `slide_success` <= `contact` in every cell for both
  arms, and `slide_success` <= `nested_honest` on the rnd30 cells. P4 (re-derivation of (k)): both learners score
  higher on spots60 than on rnd30.
- **Disconfirm branches.** (i) If only ONE arm learns the pick for a learner, that is a demo-source effect specific to
  the long-horizon setting for that learner: reported as such, then a matched control (human successes only, 64 tapes,
  vs a 64-tape machine subsample) before any claim. (ii) If NEITHER arm picks for a learner, the recipe is not
  transferable to the full task at this budget — uninformative, and no re-tuning without a new registration. (iii)
  **The DP idle confound, registered in advance:** DP is trained here on RAW (unpruned) human tapes, and
  `DP_PRUNED_GAP_2026-09-07.md` shows raw-human DP scores far below pruned-human DP at the pick (rnd 0.204 vs 0.520 in
  w3), while the machine tapes carry essentially no idle decisions. So if DP shows |Delta| >= 0.10 WITH THE MACHINE
  AHEAD, that is not read as a source effect: the registered follow-up is a DP human-PRUNED control arm
  (`prune_full_v1` on the same 74 tapes, 8 seeds) before the word "source" is used. The idle fraction of both sets is
  measured and reported with the datasets.
- **Asymmetries disclosed by construction.** (1) The machine set is best-of-3 per start on 89 % of its ICs: Sigma
  reward 118 vs 206 and 3 vs 16 demonstrated nested completions (REVIEW_GUIDE §8 item 8) — a selection asymmetry in the
  machine arm's favour on top of the tape-count match. (2) Rows differ (human 29 295 vs machine 32 923 decisions);
  "matched N" matches tapes, never rows. (3) The machine tapes come from a DP teacher trained on PRUNED human
  full-task demos; the human tapes are the recorder's follower on the raw joystick streams; both recorded with the
  plain recorder (no og4 release filter, CONFOUNDS row 50). (4) Budget units differ across learners (DP gradient
  steps, RLPD decisions, WM sim steps). (5) DP's absolute action parametrisation is converted to the delta MDP only at
  evaluation (the hold-4 rule of `wandb_eval.py`). (6) The world-model `nested` column of §5/§5.1 is the TRAINING
  PROXY (REVIEW_GUIDE §8 item 3); these cells report both `nested_proxy` and `nested_honest`, and only the proxy column
  is comparable with §5.1 until the world-model cells are re-scored. (7) RLPD's online reset distribution is the env's
  success-uid set for both arms (symmetric).
- **Jobs.** `cluster/sbatch_rlpd_e2e.sh` / `cluster/sbatch_dp_e2e.sh` (+ `cluster/e2e_eval_cells.sh`, the re-runnable
  eval stage, and `cluster/e2e_build_sets.sh` for the DP datasets): `-p preempt --qos=preempt --requeue --nice=9000
  --exclude=pax077 --constraint=l40s|a100|l40|h200`, names `e2e_rlpd_<arm>_s<seed>` / `e2e_dp_<arm>_s<seed>`, 32 jobs,
  deliberately behind the robomimic recovery, the dv3 G3 gate and the contact-phase runs. Code runs from the private
  clone `$LAB/gp_e2e` at the commit named in the submission report; the shared checkout and every other agent's tree
  are untouched. Datasets: RLPD reads `$W/demos_state_full/{dHfull_all,dDPfull}` unchanged; DP reads
  `$LAB/genesis_pickaplace/baselines/matched_w3/{dHfull_all,dDPfull}` (+ `lerobot/`), built by `full_demos.py select`.
  Smokes (2k steps / 2k grad steps, a truncated IC set, both learners) precede submission and are logged in the
  submission report, not here.

### Amendment (o) — the SLIDE phase pays what it scores: `scope='contact'` grants on the release-based clauses, not on bare `contact` (registered 2026-09-07 22:10, BEFORE any job; the 32 (m) jobs 3355056-87 are HELD, not cancelled)

**The defect.** As registered in (m) the phase pays +1 and TERMINATES on the env's bare `contact` predicate (picked ∧ pick-can↔goal solver contact ∧ ee_x < can_x) while the statistic of record is `slide_success` (release-based, (l)/(l′)). A reward-maximising policy therefore has no incentive to open the gripper: the optimal behaviour under the reward is to drive a HELD can into the goal, which scores 0 under the statistic. This is not a hypothesis — it is what the evidence already shows: my (m) smokes gave bare contact 5/11, 7/11 and 2/11 with `slide_success` 0/11 and the per-clause diagnostic `grip_closed` on every checked episode, and the carrycontact re-score gave contact 124/148 with `slide_success` 0/148. Training the 32 runs unchanged would spend ≈130 GPU-h learning the wrong behaviour and return 0 for both arms — a null with no content. The user's instruction ("train for contact **with the can placed on the shelf and not held by the gripper**") is what this amendment implements; aligning the reward completes that instruction rather than changing it.

**The change (reward only; the statistic is untouched).** In `scope='contact'` the env pays +1 and terminates when the `slide_success` clauses hold: picked earlier in the episode ∧ pick-can↔goal solver contact ∧ grip **commanded open** (< `GRIP_OPEN_CMD` = 0.3) ∧ pick-can centre in the shelf footprint with tilt < 20°, sustained `SLIDE_SUSTAIN` (= 3 decisions × action_repeat 4 = 12 env frames). **Bare `contact` no longer terminates the episode**: it is still logged and still added to `_granted`, so a policy that touches the goal while gripping keeps running and must go on to release to be paid. Unchanged: `phase_sparse` (tips terminate, no penalty), the entry definition (a banked `placed_v2` state), the demo sources, the banks, the seeds and the statistic. **`carrycontact` is NOT changed** — (l)(c) makes it the explicit "contact by any route, including still held" control, and it keeps its bare-`contact` grant.

**One definition, not two.** The grant reads the SAME per-frame clause code the score reads — `GenesisCanEnv._slide_clauses()` / the `_slide_run` counter that sets `info['slide_success']`, i.e. the eval-fixes agent's (l′) implementation — so reward and score cannot drift apart again. Nothing in `end_of_episode()`, `slide_fail_reason` or the `nested` path is modified; the settle-based route stays the fallback for episodes that reach the horizon, and in-episode `sustained` grants now become possible precisely because the episode no longer ends at first contact (the mechanical reason (l′) had to move the window into the settle).

**Horizon.** 600 decisions (2400 sim steps) per episode, per the coordinator's instruction, replacing (m)'s 600 SIM steps (= 150 decisions). Justification, measured: the longest human demonstration of this phase runs 232 decisions from `placed_v2` to its grant, so the (m) cap could not contain the longest demonstrated behaviour, and (o) makes segments longer still (touch → release). Cost is disclosed: at the unchanged 250k-decision budget an episode cap of 600 decisions means ≈4× fewer episodes per run than under (m), and evaluation cells cost proportionally more on timeouts. **Registered fallback, usable without a new amendment if cost becomes binding:** a 300-decision cap, still above the 232-decision longest demonstration; any run using it says so in its sidecar.

**Demo segments are re-cut.** Under (m) a segment ended at the bare-contact grant; under (o) the demonstrated segment must end at the frame the release-with-contact clauses close (`k_slide`), computed offline with the same four clauses (`place_demos.py`, sustained 3 decisions on the tape's decision rows). Tapes whose contact never occurs with the gripper commanded open have **no** (o) segment, so the per-arm yields may differ from (m)'s 11 v 11; the new yields are a RESULT and are reported before any run. The (m) sets are kept untouched for provenance.

**Predictions (registered).** P1: |Δ(human − machine)| < 0.10 on polE_contact `slide_success`, as at every other phase. P2: both arms score far above the (m) design's structural 0, and the `sustained` route dominates the `settle` route (the release now happens inside the episode because it is what is paid). **Disconfirm branch (registered, and the reason this amendment exists):** if NEITHER arm exceeds ≈0.1 on polE_contact at this horizon, the phase is reported as **unlearnable from 11 demonstrations at this budget** — explicitly NOT as a demonstration-source null; with both arms at the floor the comparison carries no information about the source, and any source claim waits for a human yield the data cannot currently supply. If exactly one arm collapses, that is reported as a source effect *specific to the sub-floor regime*, per (m).

**Sequencing.** No job is submitted under (o) until the slide session's re-scoring of all 74 human tapes lands: the user's eyeball count (~18 good slides) disagrees with our label (11), and if the human set widens BOTH arms are rebuilt (re-cut, re-matched) before training. The held jobs 3355056-87 stay held; they were queued under the (m) reward and must not run.

#### Addendum to (o) — measured yields: at the REGISTERED clause thresholds the slide grant is demonstrated by NEITHER arm (measurement, 2026-09-07 22:35; no design change is made here)

(o) requires the demonstrated segment to end where the release-with-contact clauses close (`k_slide`). Measured offline over the full recordings with the registered clause set — picked ∧ pick-can↔goal contact ∧ grip commanded open (< `GRIP_OPEN_CMD` 0.30) ∧ can in the shelf footprint with tilt < 20°, sustained 3 decisions (`place_demos.py phases`; the tape stores the env's STICKY contact flag, so the per-frame contact clause is the sticky flag, an UPPER bound on live contact — the numbers below are therefore optimistic):

| grip "commanded open" threshold | HUMAN tapes with ≥ 3 consecutive (a grant) | ICs | MACHINE tapes with a grant | ICs |
|---|---|---|---|---|
| **0.30 (registered)** | **0** of 74 (39 reach `placed_v2`) | 0 | **0** of 195 (104 reach `placed_v2`) | 0 |
| 0.35 | 3 | 3 | 6 | 6 |
| 0.40 | 6 | 6 | 12 | 11 |
| 0.45 (= what `placed_v2` already calls "released") | 7 | 7 | 13 | 12 |

**So (o) as written is not buildable: both arms yield zero segments, and a reward nothing demonstrates is as empty as the (m) score nothing could earn.** The mechanism is a threshold split between two registered predicates, not a data problem: `placed_v2` calls the gripper released below **0.45**, `slide_success` calls it commanded open below **0.30**, and the human slides live in the gap — contact-frame grip commands cluster at 0.33–0.41 in 5 of the 11 (m) tapes (minima per tape: 0.00, 0.11, 0.33, 0.39, 0.39, 0.41, 0.65, 0.75, 0.87, 0.94, 1.00). Physically the demonstrator sets the can down and then **pushes it into the goal with the fingers nearly closed**, which satisfies "not held" in the user's sense but not `grip_cmd < 0.3`. Dropping my proximity proxy entirely or loosening it to 0.12 m changes nothing (2 frames either way), so the proxy is not the cause.

**Not decided here.** The 0.30 threshold belongs to (l)/(l′) — the eval-fixes agent's predicate, currently being applied by the slide session to all 74 human tapes — so changing it would move the STATISTIC as well as the reward and is a cross-agent decision. The options, with the evidence attached, are: (i) align the slide clause with `placed_v2`'s 0.45 (yields 7 / 13; one "released" definition across the plan); (ii) keep 0.30 for the score and define the reward's "not held" by finger↔can contact rather than a command threshold (needs a new env signal; no tape support offline); (iii) keep both as registered and report the slide phase as not demonstrated by either source.

**Relevant to the 11-vs-18 reconciliation:** both (m) and (o) gate the phase ENTRY on `placed_v2`, so a demonstration that slides the can home without ever satisfying `placed_v2` is invisible to the phase by construction. Measured: **7 human tapes (7 ICs) reach `contact` with no `placed_v2` at all** (machine: 10 tapes / 9 ICs), and 1 human tape reaches the nested proxy that way. 11 + 7 = 18 — which matches the user's eyeball count exactly, so the discrepancy is very likely this entry gate rather than a scoring error. Recommended for the reconciliation: score entry on "can standing on the shelf, not held" rather than on `placed_v2`'s sustained-release definition.

No job is submitted and no dataset is rebuilt on this addendum; the held jobs stay held.
