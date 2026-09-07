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
