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

### Amendment (p) — the grip clause of (l) is WITHDRAWN: it contradicts the demonstrations (registered 2026-09-07 23:40, before any cell or reward uses it)

**What went wrong.** Amendment (l) required the gripper to be commanded open (`grip_cmd < 0.3`) *at the moment of contact*. Scored against the demonstrations themselves (all 74 `dHfull_w3` tapes, re-executed through the landed `end_of_episode()` implementation, slide session, `can_pos_recovery/slide_score.py`), that predicate passes **2 of 74** — and **44 of the failures are the grip clause alone**. The mechanism, confirmed by the user in his own words ("sometimes its easier to push with the gripper closed"): the human sets the can down, opens **fully**, then **re-closes to ≈ 0.4 and pushes the can home with a half-closed gripper**. Worked example uid 232 (nested, in our set of 11): grip 0.8 at grasp → 0.05 at t = 14 (a genuine release) → 0.4 from t = 20, contact at t = 24.5, last commanded grip 0.405. Median last-commanded grip over the 74 tapes is 0.39; only 27 of 74 ever end below 0.3. So the clause encoded an assumption about *how* the slide is performed that the demonstrations contradict, and as implemented `slide_success` was **stricter** than the 11 contact-after-release tapes rather than a widening of them. Consequence avoided: amendment (o), which would have made the training reward match this predicate, was **stopped before landing** — it would have penalised the demonstrated solution and trained the arm away from it. The 32 Slide jobs remain held.

**Replacement predicate (registered).** `slide_success` = all of:
1. `picked` granted earlier in the episode;
2. **a prior release onto the shelf**: `placed_v2` granted earlier — grip commanded < 0.45 with the can resting upright inside the shelf footprint, sustained as that predicate already defines;
3. solver contact between the pick-can and the goal can;
4. the pick-can inside the shelf footprint with tilt < 20°;
5. **the can is supported by the shelf, not clamped by the fingers, at the moment of contact** — so that "release, re-grasp, carry into the goal" cannot pass while "release, then push with a half-closed gripper" does.

Clauses 1–4 are settled. **Clause 5's threshold is NOT registered by guess**: it must be calibrated from the demonstration traces themselves (finger separation versus can diameter, and/or the can's centre height versus its resting height on the shelf) and the calibrated value reported with the distribution it came from. Acceptance test for the calibration, stated in advance: the predicate must pass uid 232 and must pass a count in the neighbourhood of the ~18 tapes the user identified by eye as good slides, while failing tapes where the can is carried in still grasped. Evaluation point is unchanged from (l′) — the first 12 frames of the existing 100-step end-of-episode settle, with the route (`sustained` / `settle`) recorded.

**Sequencing, before anything trains.** The slide session has shown that `dHfull_w3` and the census recording of the same 74 ICs disagree on 24 of 74 action streams and on 11 contact flags, so the phase bank and the scoring must be built from **one** lineage and it must be the lineage that is scored. Do not re-cut contact segments or widen the human set until that is settled; the four counts of record (26 census contact / 21 tape-flag contact / 16 nested / 11 contact-after-release) are explained by that lineage gap, not by predicate differences.

**Not affected by this amendment:** `PHASE_RESULTS` §5/§5.1 end-to-end cells (they score picked/contact/nested, not `slide_success`), all world-model numbers, and `contact_push` (a different clause set, unaudited against this finding).

### Amendment (q) — clause 5 calibrated, lineage fixed, and the human-set filter left OPEN (registered 2026-09-08 00:55)

Calibration and lineage work by the slide session; write-up on disk at `paper/SLIDE_CLAUSE5_LINEAGE_2026-09-07.md` (uncommitted at its author's standing rule — not committed by this session).

**Clause 5 (amendment (p)) is calibrated as a SUPPORTED test, with no gripper term:** the pick-can's centre height ≤ resting + **0.010 m** (resting = shelf_top + can_h/2 = 0.2205). Basis, over all 74 tapes post-pick with the can upright in the footprint: released-and-resting frames (grip cmd < 0.15, n = 897) have |z − rest| median 0.00 cm, p95 0.09, p99 0.65, max 0.77 cm; the threshold is 1.5× that p99 and 1.3× its max, and well inside the 2.13 cm median lift of a carry. Height cannot serve as a *clamp* test — 41 % of commanded-closed frames are also within 1 cm of resting, because the human holds the can down on the shelf while still gripping — but clause 5 asks for *supported*, which is what this measures. If the env can expose a pick-can↔shelf solver contact cheaply, use that instead: same meaning, no constant. **Registered prohibition:** do not reintroduce a gripper term via finger position. Measured trap — while the can is provably in the hand, actual finger position sits in a stalled band (p1 0.423 / median 0.577 / p99 0.709) because the 66 mm can blocks closure, so a reading *above* that band means the fingers closed on empty air, i.e. a legitimate **fist push**; five of the eleven contact-after-release tapes are fist pushes (317, 259, 325, 321, 273), and separating uid 232 (0.405) from the clamp floor (0.423) would rest on 1.8 % of range.

**Acceptance test, reported as registered, not fitted:** passes uid 232 on both lineages — YES. Fails a can carried in still grasped — **untestable in this data: no human tape does it**; the prior-release clause already excludes it, so clause 5 only bites on machine policies. Lands near ~18 — **NO, it lands at 14 (census) / 12 (dHfull)**; see below, and no tuning was done.

**Lineage of record: the CENSUS recording.** It re-executes bit-exact on the machine doing the scoring (15/15 in this check; `honest_rescore` 74/74 separately), whereas `dHfull_w3` re-executes bit-exact 2/74 and, replayed locally, loses the pick entirely on 286, 293, 294, 295, 300 (all five picked in the census, two nested). The two recordings differ on 24/74 action streams and 11 contact flags — that lineage gap, not any predicate difference, is what produced the four irreconcilable counts (26 / 21 / 16 / 11). **Binding rule: bank, scorer and runs must share ONE lineage AND ONE machine.** This does not assert census-correct/cluster-wrong; the cross-machine question is still open and must be settled before anything is re-cut.

**OPEN, for the user: which human set the Slide phase trains on.**
- **Sim-success filter → 14 tapes** (census: 232 233 236 237 255 273 294 297 298 299 305 317 325 330). Train/score aligned.
- **Demonstrated-intent filter → ~30 tapes** (the 14 plus 16 "short/upright": the human released and pushed, and the *simulated* can stopped 2–4 cm short — a systematic, control-limited sim shortfall per `SLIDE_ANATOMY_2026-09-07.md`, not a bad demonstration). The user's eyeball count of ~18, made off the real footage, sits between the two, and both numbers are correct measures of different things: 14 = "the slide succeeded in simulation", ~30 = "the human performed a good slide".
- **Asymmetry that the choice must account for (raised here, not by the calibrating session):** the machine demonstrations are harvested from policies *acting in this simulator*, so they are 100 % sim-achievable by construction. Filtering the human set by sim success (14) matches that and keeps the source comparison fair; not filtering (30) gives the human arm demonstrations that do not achieve the goal in the environment both arms are scored in, which is a systematic disadvantage for the human arm and would be especially damaging to the imitation learner, which has no reward to correct it. Against that, filtering by sim success selects for the geometry this world happens to reproduce and biases the phase toward easy starts.
- **Recommendation, for decision:** run the phase on the 14 as the comparison of record, and report the 16 short/upright tapes as a disclosed exclusion with their cause, rather than silently choosing either. If the phase is later reframed as "can the learner reproduce demonstrated intent", the 30-tape set is the right one and the machine arm must then be harvested without a sim-success filter too, or the asymmetry simply moves.

### Amendment (r) — the phase ENTRY gate has the same grip-threshold defect as (l); corrected (registered 2026-09-08 02:45)

Adjudication by the slide session between two competing accounts of the user's "~18 good slides"; evidence in `paper/SLIDE_CLAUSE5_LINEAGE_2026-09-07.md` (uncommitted at its author's standing rule).

**Finding.** The Slide phase gates its ENTRY on `placed_v2`, which contains a gripper term (`grip_cmd < 0.45`). Seven human tapes reach contact without ever satisfying it, and for three of them the binding clause is the gripper alone — uid 308 passes footprint/z/tilt on 27 frames but grip on 0 of 270; 328 passes on 20 frames, grip 0 of 194; 333 on 81 frames, grip 0 of 422. That is the **same disease as the withdrawn (l) clause**, in a different place: a gripper threshold encoding an assumption the demonstrations contradict. One more (242) is a sustain near-miss whose binding input is again grip. The other three are correctly excluded and are not good slides: 297 sits 2.4–2.8 cm above resting height, 309 and 320 fail tilt, and all three end at 90°.

**Corrected entry, registered:** `picked` earlier ∧ can inside the shelf footprint ∧ |can_z − resting_z| ≤ 0.010 m (resting = 0.2205) ∧ tilt < 20°, **sustained 10 frames, with NO gripper term**. Same calibration as clause 5 (released-and-resting population, n = 897: p99 0.65 cm, max 0.77 cm; a carried can is 2.13 cm up at the median, so it cannot qualify). This removes the last gripper threshold from the pipeline. It deliberately admits "hold the can down on the shelf, then push", which is 41 % of commanded-closed frames and is correct for an *entry* condition: if the can is at shelf height, upright and inside the footprint for 10 frames, it is on the shelf regardless of where the fingers are.

**Measured effect (offline; re-score in-env before cutting):** entry count 39 → 43 (dHfull) and 43 → 48 (census); full-predicate passes 12 → 13 (dHfull) and **14 → 15** (census, set 232 233 236 237 255 273 294 297 299 300 305 308 317 325 330, 11 of 15 honest-nested). It recovers 308 of the three fist-push tapes; 328 and 333 still fail on no-contact-after-entry and tilt.

**The 11 + 7 = 18 arithmetic is a COINCIDENCE and must not be cited as evidence.** It holds only on the dHfull lineage; the identical statistics on the census lineage give 13 + 9 = 22, and only 4 of the 7 appear there. A quantity that moves by 4 when the recording changes cannot explain a fixed eyeball count. The honest statement is that ~18 lies inside [14, 30] and **no automated count reproduces it**, because the user was judging the human's performance while every automated count scores the simulation's outcome.

**Full ledger of the counts, each with its pipeline** (this closes the reconciliation): 21 = tape-recorded contact flag, dHfull. 26 = same statistic, census (recorded and re-executed agree 74/74) — the 21-vs-26 gap is **lineage**, not predicate. 16 = honest nested after settle, census. 11 = contact-after-`placed_v2`, dHfull (census analogue 13). 7 = contact with no `placed_v2`, dHfull (census analogue 9). 14 = amendment-(p) predicate, census (12 dHfull), → 15 under this amendment. 30 = census tapes showing demonstrated slide intent (the 14 plus 16 short/upright sim shortfalls).

**Still open and unchanged (amendment (q)):** whether the phase's human set is filtered by simulation success (15) or by demonstrated intent (~30). This amendment does not decide it; it buys 1 uid, not 7.

### Amendment (s) — end-to-end evaluation protocol: both shared-process and isolated cells, registered before either exists (2026-09-08 03:50)

Arising from the two effects established overnight (see `OVERNIGHT_STATE_2026-09-07.md`, section "STATE AT 03:30"): full-scope episode results depend on (1) the machine's physical core count and (2) the position of the episode in the process's RNG stream, because the world model samples its latent inside `act` and the torch RNG is not re-seeded per episode. Neither is an environment defect; the environment's post-reset state is bit-identical across conditions in all 21 measured fields.

**Registered protocol.** Every end-to-end run produces **two** cells per evaluation set:
- **shared** (`fresh_eval_<set>_<mode>`): all episodes in one process, in the recorded order — the protocol the world-model cells of record used. **Verified, not assumed:** the world-model cells enumerate exactly the same starts in the same positional order as the DP/RLPD evaluator (30/30 can positions on `rnd30`, 15/15 uids on `hold15`), so the arms are genuinely order-matched and the ordering confound is excluded for this comparison.
- **isolated** (`..._iso`): one process per episode, which is the only way to get true independence — Genesis permits one world per process, so in-process re-initialisation is unavailable. Measured cost on the same hardware: 59 s vs 28.5 s per episode (2.05×), i.e. RLPD evaluation 18 → 50 min and DP 35 → 75 min against 30 h and 16 h walltimes. Affordable, so both are produced rather than one being chosen.

**Statistic of record: the shared cells**, because they are order-matched to the published world-model row and therefore comparable within the three-learner table. **The isolated cells are published alongside as the registered correctness check, with the shared-minus-isolated delta stated per learner.** If that delta is material, it is itself the evidence for re-scoring `PHASE_RESULTS` §5.1 under isolation — and because both cells exist for every seed, that decision costs a re-analysis and never a re-run.

**Provenance stamps (required, now implemented):** every episode records `node`, `pid` and within-process `order`; every cell records the node set, pid set and the isolation protocol; the table builder prints protocol and node set per cell and warns explicitly when a comparison spans nodes. Isolation removes the ordering effect only — node sensitivity remains, but seeds are spread across nodes so it enters as variance rather than bias, and the per-episode node stamps allow it to be quantified post hoc by grouping seeds by node class.

**Prediction, registered before the cells exist:** |shared − isolated| < 0.05 per arm per stage; if it exceeds that for either learner, §5.1 is re-scored under isolation before the three-learner table is published.

### Amendment (t) — end-to-end cells to be re-scored on one pinned CPU model; and the published `nested` over-counts 2.5× (registered 2026-09-08 05:45)

**§5.1 reproduces exactly.** The sequence check (dHfull_all s3, rnd30 MODE, 30 episodes re-run in order on the same CPU model as the record) gives **0/30 per-episode differences** and identical aggregates: picked 19/30, contact 13/30, nested 10/30, Δ +0.000 on all three. Whole-sequence reproduction on a matched CPU model holds, so the published cell is not wrong *as the quantity it measured*.

**But that quantity is not the one we thought.** On the same cell: published `nested` (= `nested_proxy`) 10/30 = 0.333, while **`nested_honest` is 4/30 = 0.133** — the published end-to-end nested figure **over-counts by 2.5×**, with 6 of the 10 proxy-only (episodes 0, 3, 15, 17, 18, 27). DP and RLPD report the honest predicate, so the three-learner table cannot mix them. Also on that cell: `placed_v2` 8/30 = 0.267 where the stale `placed` reads 0/30 — **8 episodes did release the can onto the shelf**, which refutes the "policies carry without releasing" reading in `PHASE_RESULTS` §5.1 and §2.7 (already once corrected; this is the measurement that settles it). `contact_push` 10/30 against `contact` 13/30. `slide_success` 2/30, ≤ `nested_honest`, so (l)'s prediction holds for this cell. Design validation: both `slide_success` grants arrived via the settle window (`sustained: 0, settle: 2`) — a literal in-episode counter would have reported 0/30, the silent zero flagged in (l′).

**New confound, and it is the consequential one: hardware class is partially confounded with ARM in the published 8 v 8.** Mapping cells to nodes shows the machine arm is 8/8 sapphirerapids (AVX-512) while the human arm has two cells on AVX2 broadwell and one on graniterapids. Since CPU class demonstrably flips long-horizon outcomes, arm and instruction set are not independent in that comparison. Its magnitude is being measured directly (same cell on sapphirerapids versus its broadwell record).

**Registered remedy:** re-score **all 64 end-to-end cells on a single pinned CPU model**, pinned by model string read from `/proc/cpuinfo` or explicit node list — never by Slurm feature label, which misreports (`pax001` advertises broadwell and is Cascade Lake). That removes the confound entirely and yields one internally consistent set, reported with `nested_honest`, `placed_v2`, `contact`, `contact_push` and `slide_success` columns. Phase cells need no pinning (3488 episodes bit-exact across classes). The published §5.1 numbers stand as a record of what was measured and are superseded by the pinned re-score for the three-learner table; both are reported, with the difference attributed.

---

## Amendment (u) — ignition speed as a demonstration-source effect (registered 2026-09-08, BEFORE the replication data exists)

**Origin, stated honestly: this is a post hoc observation, not yet a finding.** Learning curves built from training-rollout logs show that on the end-to-end task the human-demonstration arm reaches the pick threshold about **262k environment steps earlier** than the machine arm — per-seed medians 549,998 versus 799,996, exact permutation p = 0.018 on ignition step — while the published final-checkpoint comparison for that same pair is a null. The threshold (0.2), the bin grid (40) and the stage (`picked`) were all chosen after seeing the data, and three phase families were examined, so a Bonferroni correction over three puts it at 0.054. **It is not citable as it stands and must not be written up from the discovery sample.**

**Why it is worth registering rather than discarding.** Every comparison in this project is final-checkpoint, so the entire design is blind to differences in *sample efficiency*. "Demonstration source does not change what a learner ultimately reaches, but human demonstrations get it there sooner" is a coherent, plausible and practically important claim that our tables cannot currently see. It is also the kind of effect that would explain why practitioners believe demonstration source matters while our endpoint comparisons keep returning nulls.

**Pre-registered replication, fixed before the data exists.**
- **Replication set:** the 16 RLPD end-to-end runs (8 human, 8 machine) currently queued. Diffusion Policy is excluded by construction — it has no online interaction and therefore no ignition.
- **Statistic:** the environment step at which a run's binned `picked` rate first reaches **0.2 and holds it**, on a **40-bin** grid over the run's own budget. These values are fixed here and may not be tuned.
- **Stage:** `picked` only. Other stages are exploratory and reported descriptively.
- **Test:** exact two-sided permutation on per-seed ignition steps, 8 v 8, α 0.05.
- **Prediction (directional, from the discovery sample):** the human arm ignites earlier. A result in the opposite direction, or a null, disconfirms.
- **Handling of non-igniting runs:** a run that never reaches threshold is assigned its full budget and flagged; if more than 2 of 8 in either arm fail to ignite the test is reported as inconclusive rather than significant, because the statistic degenerates.
- **Disconfirm branch:** if the replication returns p > 0.05, the observation is reported as a non-replicating post hoc artefact of the discovery sample and is not carried into the paper's claims.

**Independent of the outcome**, the learning curves themselves are reported descriptively with their training-rollout caveat attached, since their endpoints are not the evaluation statistic.

### Amendment (u) — two constraints recorded the same day, before any replication data exists

**1. The discovery is a WORLD-MODEL result, not a general one.** Every learning curve behind it is r2dreamer, because it is the only learner in this project that logs online training rollouts. RLPD writes nothing training-time — checkpoints only — and Diffusion Policy is offline by construction and can have no success curve at all, its intermediate checkpoints having been pruned for disk. So the ignition effect is currently a statement about one learner. Replicating it on RLPD would test whether it **generalises across learners**, which is a stronger and different question from whether it reproduces.

**2. The registered replication is not executable as written, and is being repaired before the data exists rather than after.** The statistic — the step at which binned `picked` first reaches and holds 0.2 — requires per-episode training-rollout records that RLPD does not currently produce. The 16 queued runs have not started, so the intended fix is to enable that logging before they do. If it cannot be enabled, the fallback is a two-point comparison from the checkpoints that do exist (`ckpt_040` at 100k decisions and the final checkpoint), which is far weaker and must be registered as such. **A registered test that cannot be run will not be left standing**; whichever way this resolves is recorded here before any replication data exists.

**What does not change:** the discovery-sample result, its post hoc status, and the bound that makes it interesting — no learning-speed difference at pick (p 0.505), place (p 0.414) or slide (p 1.000), and a difference only on the long-horizon end-to-end task (p 0.018–0.032 by definition). An effect present everywhere would suggest an artefact; one confined to the task with a long credit-assignment chain is falsifiable.

### Amendment (u) — resolution of the executability problem (same day, still before any replication data)

**Fixed without cancelling anything.** The queued RLPD runs read the trainer at execution time, so per-episode rollout logging was added to jobs that had not yet started — all 16 verified still pending at the time of the edit. Each run now appends one record per finished online rollout with its step and sticky stage flags, **deliberately in the same shape as the world model's records**, so both learners' curves are computed by one code path rather than two conventions reconciled after the fact. Cost is about 0.16 MB per run and no measurable runtime. The change is logging-only: it wraps no environment, consumes no randomness, alters no training dynamics, and swallows its own exceptions so it cannot kill a run.

**A distinction that must be preserved when this is read out.** Because the discovery is a world-model result, the RLPD replication tests whether the effect **generalises across learners** — not merely whether it reproduces. These are different outcomes and must not be reported as one:

- **Reproduces on RLPD** → the effect is a property of learning from demonstrations on this long-horizon task, not of one implementation.
- **Null on RLPD** → the effect may still be real for the world model. This is a *boundary condition*, not a failure to replicate, and the honest report is "present in the world model, absent in RLPD", which is itself informative about why.
- **Opposite direction on RLPD** → treat as disconfirming the general claim.

**Diffusion Policy cannot participate at all** — it is offline, produces no rollouts, and its intermediate checkpoints were pruned for disk. Any cross-learner learning-speed figure is therefore a two-learner figure and must say so on its face.

### Amendment (u) — three qualifications that weaken the finding, recorded before any replication

**1. The effect is graded, not a clean dichotomy.** Place trends the same way as end-to-end (human earlier, p 0.075 on the steady-state definition) rather than being null. The honest shape is "clearest end-to-end, weakly in the same direction at place, absent at pick and slide". A graded effect is more plausible than one that switches on, so this is not damaging — but "only on the long-horizon task" overstates it and should not be written.

**2. "Stage in isolation versus stage in a chain" is not a controlled contrast.** The end-to-end `picked` curve is a pick occurring inside the full task, under the staged ladder, from task-start initial conditions. The pick-scope curve is a dedicated task with its own terminal reward, its own initial conditions and its own demonstration sets. Reward, horizon, termination and data all differ, so the comparison currently conflates **chain** with **setting**. The attractive interpretation — that credit assignment over a long chain is where demonstration source matters — is not yet supported by a design that isolates it.

**3. The most serious qualification: the demonstration sets differ in construction exactly where the effect appears.** The end-to-end arms are the human set of all 74 attempts including failures against a machine set of 72 **best-of-3 selected** attempts, Σ reward 206 against 118. The pick arms are matched successes. **So an end-to-end-only speed difference is equally consistent with "best-of-3 selection changes what the machine set teaches early" as with anything about who produced the demonstrations.** The selection confound and the effect coincide precisely — the confound lands on the one comparison that separates and is absent from the ones that do not. This is the leading alternative explanation and must be stated wherever the finding is.

**What would settle it.** First, the registered RLPD replication, which now has the data it needs: it says whether the effect generalises across learners or is a world-model artefact. Beyond that, **an end-to-end arm whose machine set is built without best-of-3 selection** would separate "how the set was built" from "who produced it". That control should be registered before this becomes a headline rather than an observation.

Standing caveat unchanged: exploratory, one learner, thresholds and grid chosen after seeing the data.

### Amendment (u) — magnitudes corrected; the discovery numbers were contaminated

**The originally reported ~262k-step difference was computed on contaminated flags and is withdrawn.** In the full scope, stage flags are only written when an episode terminates *inside* the adapter; a horizon truncation happens outside it, so a truncated episode logged all-zero flags even when it had picked. On one run, 1,198 of 2,911 episodes were all-zero and every one was exactly the horizon length — 608 of them had in fact scored. Flag-based `picked` read 0.480 against 0.688 by score.

**Corrected by deriving the stages from the accumulated reward stream**, which survives truncation. The effect stands, with smaller and better-behaved magnitudes:

| stage | human | machine | Δ | p |
|---|---|---|---|---|
| picked | 424,998 | 574,998 | −156,249 | **0.019** |
| contact | 824,997 | 949,996 | −187,499 | **0.044** |
| nested (proxy) | 1,024,996 | 1,224,995 | −249,999 | 0.059 |

**This is a better result than the contaminated one, not merely a smaller one.** The stages now order sensibly — later stages ignite later, and the gap grows along the ladder — which is what a genuine credit-assignment effect should look like. The pick, place and slide curves are unaffected, because each of those tasks terminates *on* its own stage, so their flags were always exact.

**The diagnostic tell, recorded as a standing check rather than an anecdote.** The contamination announced itself: flag-based `picked`, `contact` and `nested` all reported *identical* ignition steps. Three strictly nested stages cannot ignite at the same step. **After computing any per-stage statistic, assert that strictly harder stages differ from easier ones** — when a metric that must vary across nested conditions reports the same value for all of them, it is measuring something other than its name, which is the same failure family as the contact predicate and the nested proxy.

Online performance levels were also understated by the same defect and move up; this affects the curves' captions, not the evaluation cells.

### Amendment (u) — the selection control is a PREREQUISITE for any seed increase, not a follow-up

Ordering decision, recorded before either is run. The learning-speed effect currently says: human demonstrations reach each stage about 156k steps sooner, measured on a machine set assembled by **keeping the best of up to three attempts per initial condition** (Σ demonstrated reward 206 against 118; 16 demonstrated completions against 3) versus a human set that keeps **every** attempt including failures.

**If more seeds sharpened that to p < 0.01, we still would not know whether we had measured who produced the data or how we filtered it.** More power on a confounded comparison buys precision about the wrong quantity. And a significant result that later turns out to be about our own set construction is *worse* than the null, because by then it would be published.

**Therefore: the end-to-end arm built from a machine set WITHOUT best-of-3 selection must run before, not after, any increase in seeds.** If the effect survives the selection control, additional seeds are worth spending on it. If it does not, the seeds would have been spent establishing an artefact with confidence.

This inverts the intuitive order — power first, controls later — and it is the right way round whenever the candidate finding sits exactly where a known confound sits, which is the case here.

### Operational note — lane-level instructions and job-level instructions compose badly

A targeted instruction ("deprioritise A4, keep A6") and a blanket one ("robomimic on the backburner") were issued to two different lanes, and the blanket one silently overrode the targeted one, burying the single experiment that had been explicitly protected. It was caught only because the agent holding it queried the arithmetic rather than executing quietly.

**Two standing habits adopted.** When an instruction names a *lane* rather than a job family, state the blast radius back before acting — "that is 17 jobs including the 16 you protected, confirm?" costs one line. And **prefer reversible actions on blanket instructions**: deprioritising was undoable, which is the only reason this was recoverable. The identical error under a "cancel" instruction would have destroyed completed work with nothing to restore.

### Amendment (v) — DE-CONFOUNDED end-to-end machine arm: first-attempt-per-IC, all three learners (registered 2026-09-08, user's top priority, BEFORE any build or run)

**Why.** The end-to-end machine set of record (`dDPfull`) keeps the **highest-reward attempt per IC** from a 195-tape
harvest (`--one-per-ic-best`), while the human set keeps **every** attempt including failures. Measured from the surviving
harvest (72 ICs, 64 with >1 attempt; attempt order from the sequential rollout uids):

| machine set | Σ reward | nested demos | picked |
|---|---|---|---|
| BEST per IC (of record) | 206 | 16 | 70 |
| **FIRST attempt per IC** | **131** | **8** | 63 |
| HUMAN `dHfull_all` | 118 | 3 | 64 |

Selection raises reward **+57 %** and **doubles** demonstrated completions; `best ≠ first` on **24 of 72** ICs. Against the
human set the gap largely collapses without it (Σ 131 v 118; picked 63 v 64). So any human-vs-machine end-to-end result —
including the learning-speed effect, the only positive we have — is currently confounded with **how we chose tapes**.
Selection cannot be matched upward (each human uid has one recorded attempt, deterministic replay reproduces it), so the
only symmetric fix is to **de-select the machine arm**.

**Set.** `dDPfull_first` = one tape per `ic_uid` from `$W/demos_state_full/src_dDPfull` (195 tapes, 72 ICs), chosen as the
**lowest rollout uid** = the first attempt (`to_dreamer_native.py --one-per-ic-first`, new flag; deterministic, no reward
term). Built in all three formats by the existing `e2e_build_sets.sh` path so it is row-for-row comparable with the sets of
record. Human arm unchanged (`dHfull_all`).

**Arms and budget.** Three learners — r2dreamer, RLPD, Diffusion Policy — **4 seeds** on `dDPfull_first` each, at the same
budgets as their sets of record (WM 2e6 sim steps, RLPD 250k decisions, DP 100k grad steps). This is a **pilot**: 4 seeds
cannot support an equivalence claim (exact permutation floor p = 0.029), and expansion to 8 is gated on feasibility, not on
the sign of any difference.

**Statistic.** Unchanged from (d)/(l): rnd30 (random-uniform starts) as the cell of record, all stages reported, exact
two-sided permutation on per-seed counts, pinned to one CPU class. `slide_success` remains a diagnostic while (p) clause 5
is uncalibrated.

**Predictions (registered).**
- **P1 — the source null survives de-selection.** |Δ(human − machine-first)| < 0.10 at every stage either arm reaches
  ≥ 0.2 on rnd30. If it does, the end-to-end null is a statement about demonstration source and not about tape choice.
- **P2 — the selection contrast is positive.** machine-best ≥ machine-first on final-stage rates (world model, where both
  arms exist at 8 and 4 seeds). If machine-first ≥ machine-best, then best-of-3 selection did not help the learner even
  though it improved the data, which is itself reportable.
- **P3 — learning speed, the decisive one.** The human arm currently ignites ~156k steps before machine-best on `picked`
  (p 0.019). Registered readings, fixed in advance: **(a)** the human advantage persists or grows against machine-first →
  the effect is not explained by selection and survives as the project's one positive result; **(b)** it shrinks toward
  zero → the effect was substantially our filtering, and no source claim can rest on it; **(c)** it reverses → report as
  such. No outcome is treated as confirmatory of a source effect on its own, because the sets still differ in idle
  fraction (36.5 % v 0.7 %) and in whether failures are included.

**Disclosed.** The machine-first set is *not* matched to the human set on rows or on outcome content; it is matched on
**protocol** (one attempt per start, no selection), which is the axis the confound lives on. Four seeds per learner; the
existing 8-seed machine-best arms remain the published comparison until this reads out.

**Code state (added at submission, before any (v) run produced a step).** Amendments (j)/(l) added an end-of-episode
settle to the world-model adapter *after* the end-to-end arms of record had trained. It is logging-only — it runs after
the terminal observation, reward and termination are taken — but leaving it on would put a **code difference between the
two arms being compared**, which is the thing this amendment exists to remove. The call is therefore made skippable
(`R2D_EOE`, default unchanged) and the four (v) world-model runs train with `R2D_EOE=0`, reproducing the training path
of the human arm of record. Consequence, stated rather than discovered later: those runs' *in-job* evaluations carry no
`nested_honest` or `slide_success` key, so the (v) world-model cells of record must come from the pinned post-hoc
re-score, exactly as the arms of record's cells do. RLPD and DP are unaffected — both arms of each are training in the
same tree, at the same time, on the same code.

---

## Amendment (w) — episode-record requirement for all future runs (user, 2026-09-08)

**The principle: observability must not be bought by changing the reward.** The current full-scope runs violate this by accident. Stage *flags* are written only when an episode terminates inside the adapter, so a horizon truncation logs all zeros even for an episode that picked — measured at 1,198 of 2,911 episodes on one run, every one exactly the horizon length, 608 of which had scored. The only truncation-proof channel left was the accumulated reward, which means **what we can observe is limited to what we pay for**. That is why end-to-end curves exist for pick, contact and the nested proxy but **not for placement or slide**: the reward's `placed` rung uses the stale release predicate that is essentially never granted, `placed_v2` exists only as a flag, and slide is not a rung at all.

**Requirement 1 — one complete episode record, written on every exit path.** Each episode emits exactly one record, from a single code path reached by **both** termination and truncation, carrying **cumulative (sticky)** outcomes for every stage of interest: `picked`, `placed_v2`, `contact`, `contact_push`, `nested`, and the accepted-slide predicate. Sticky means once achieved in the episode it remains true, so truncation cannot erase it. One path, not two, so the branches cannot diverge — the present defect is exactly two paths where one was assumed.

**Requirement 2 — settle-dependent metrics must not perturb training.** Any predicate needing extra simulation (the settled nested predicate; accepted-slide, which requires a release and a rest) must be evaluated in a way that cannot advance or mutate the training environment's state. Either compute it in evaluation only, or from a snapshot/deferred pass. **Measuring must not change what is being measured** — and a settle executed inline during training would do exactly that.

**Requirement 3 — the reward is unchanged.** Logging is added; no rung is introduced, removed or reweighted to make a stage visible. Runs remain comparable with existing arms on the reward they optimise, and the observability change is orthogonal to it.

**What it buys:** acquisition curves for pick, placement and accepted slide within the full task, on one axis, without the reward-versus-score mismatch that currently makes the end-to-end runs optimise a proxy while the statistic of record sits elsewhere.

**Implementation note.** The per-episode logging callback added to the queued RLPD runs is the model: logging-only, wrapping no environment, consuming no randomness, swallowing its own exceptions so it cannot kill a run, at about 0.16 MB per run. Cost is negligible; the discipline is that it must sit on the shared exit path and must never call anything that steps the simulator.

**Standing check.** After computing any per-stage statistic, assert that strictly harder stages are subsets of easier ones. Three identical ignition steps across nested stages is what exposed the present defect, and the check is already in `HRI_results/curves/learning_curves.py`.

---

## Amendment (x) — approved parallel R2 long-run pilot (user, 2026-09-08)

Approved human-all versus machine-first, n=4 per arm, 4M ONLINE simulator steps,
existing 500k FIFO/no reinjection, and 2M/4M milestone checkpoints. Allocate one
human/machine pair per node on four comparable L40S nodes (eight concurrent GPU
jobs). The single-node A100 proposal is superseded without having been submitted.
Exact seeds, release digest, node blocks, analyses and interruption policy are in
[LONG_RUN_REGISTRATION_2026-09-08.md](LONG_RUN_REGISTRATION_2026-09-08.md).
Only new jobs use the isolated release; existing jobs and their paths stay untouched.

---

## Amendment (x) — `slide_success`, settled definition (user decision, 2026-09-09)

**Both earlier definitions failed the same way: they inferred "released" from the GRIPPER.** (l) required `grip_cmd < 0.3`, which passes **2 of 74** human demonstrations, because people release fully, re-close to about 0.4 and push the can home with the fingers partly shut. (p) replaced it with a displacement clause whose threshold was never calibrated. Release is a fact about **where the can's weight is**, not about the hand.

**Definition — three state conditions, no free threshold:**

1. **Released** — the can stays put while the tool moves away from it. A held can tracks the tool; a released one does not. No gripper term, and no dependence on the tool point, which is what made the old `contact` clause vacuous (ee = wrist).
2. **Pushed** — after release, the can gets closer to the goal. Carrying is already excluded by requiring release *first*, so no far-side geometric test is needed to rule it out.
3. **Arrived** — at the end the can is within the nested proximity of the goal and is not tipped.

Constants are the metrics of record or noise floors, not tuned: proximity 0.081 m (can diameter + 15 mm), stillness 2 mm, tool motion 10 mm, sustained 10 frames, goalward gain 10 mm.

**Measured on the human set: 64 tapes → released 64, pushed 64, arrived 15, `slide_success` = 15.** That **independently reproduces the 15 sim-completing tapes** the per-uid census ladder found by a different route — two methods, same set.

**Conditions 1 and 2 are non-binding on human demonstrations and that is the design.** Every human tape releases and every one nudges the can goalward at some point, so the discriminating condition here is arrival. They exist to bite on **policies**, which earn ordinary `contact` by carrying the can in and never releasing — 84–86 % of policy grants against 46 % of human ones.

**The reward moves with the definition** (user, 2026-09-09): breaking comparability touches only the slide and end-to-end sliding comparisons, neither of which is functional for sliding, so nothing is lost. This also closes the reward-versus-score mismatch, where the end-to-end ladder pays its top rung on the `nested` training proxy while the statistic of record sits elsewhere.

**Implementation:** `can_pos_recovery/slide_predicate.py` (classifier, writes per-tape json) and `can_pos_recovery/slide_reels.sh` (builds `slide_successes.m3u` / `slide_failures.m3u` from the existing real-versus-sim census renders; nothing re-rendered). Applies unchanged to a larger set as real2sim recovery adds tapes.

### Amendment (w) — correction: ALL FOUR stages are registerable inline, settle included

An earlier statement in this amendment, and in `MUST_HAVE_RESULTS.md`, implied that settle-dependent stages could not be logged during training without perturbing it. **That is too strong and is corrected here (user, 2026-09-09).**

**Nothing prevents registering pick / place / can-contact / can-settle on every episode.** The environment already tracks all of them in a sticky set that a stage enters the first time its predicate fires and never leaves. The information exists at every step; it was simply never emitted. The curves cannot be rebuilt *retrospectively* for runs already finished, because those runs did not log it — but that is a property of the old logs, not a limitation of the measurement.

**The settle caveat dissolves under amendment (x).** It applied to the environment's `nested` check, which simulates 100 settle steps and would change training if run inline. Predicate (x) simulates nothing: it reads the **final state** — can within 0.081 m of the goal, upright. The one thing a final-frame read misses is *at rest*, since the state vector carries no velocities; that is recoverable from the trajectory itself as can displacement below ~2 mm across the last few frames. Both are free and inline.

**So the episode record carries all four stages, with no extra simulation and no reward change**, and the settle field is the (x) predicate rather than the settle-simulating one. The three requirements of this amendment are otherwise unchanged: one record from the single exit path both termination and truncation reach; logging only; reward untouched.

---

## Amendment (y) — the tape-reward validation is pick-scope and must be scope-aware (2026-09-09)

**The defect.** `to_dreamer_native.convert_one()` takes no scope argument, so its reward validation is applied unconditionally to every tape. Two clauses encode pick-scope assumptions: rewards must be drawn from **{0, 1, 2}**, and **no positive reward may appear before the terminal row**. Its own comment says why — a fast pick can land the stage grant and the hardened-pick terminal inside one repeat-4 window, giving 2.0 on the terminal row — and the intent is to stop shaping being baked into demonstration tapes.

**Both clauses are wrong for full scope by construction.** The staged ladder pays at four separate moments during an episode (picked, place, can-contact, can-settle) and its top rung is 4.0. So a correct full-scope tape necessarily violates both.

**The existing full-scope sets already violate it and predate it.** Inspecting `demos_state_full/dHfull_all`: converted tapes carry rewards at non-terminal rows — indices 71 and 205 in a 256-step tape, 44 in a 171-step tape — which the "no positive reward before the terminal row" clause forbids. They exist and every full-scope run to date has trained on them. So the guard postdates the data it would now reject, and **making it scope-aware restores the prior behaviour rather than weakening a check that was ever enforced on this scope.**

**Checked and excluded: this is not a value-function constraint.** `train_rlpd.py` has no reward scaling, reward clipping, Q-clipping or value normalisation — the only normalisation is on actions. So the {0,1,2} bound protects nothing downstream in RLPD, and the r2dreamer return clamp is unaffected because the per-episode ceiling is 8.0 under both ladders.

**Change.** Thread `scope` into `convert_one` and apply both clauses only when `scope == 'pick'`. For full scope, retain the checks that remain meaningful: rewards non-negative, finite, and the tape terminating or truncating on its last row. Pick-scope behaviour is unchanged.

**Why it matters now.** Without it, r2dreamer cannot train on the relabelled `_rx` demonstrations, which would leave it taking one objective from its demonstration buffer and a different one from the environment — the mismatch the relabel exists to remove.

### Amendment (y) — implemented, and there were THREE pick-scope behaviours, not two

Registered as two validation clauses; a third was found only by checking arithmetic after conversion, and it was the dangerous one.

1. **`{0,1,2}` value check** — now `scope == 'pick'` only. Failed loudly.
2. **No positive reward before the terminal row** — now `scope == 'pick'` only. Failed loudly.
3. **`rew = np.minimum(rew, 1.0)`** — a hard clip of every reward to 1.0, applied unconditionally. Its rationale is pick-specific: a single-stage pick pays exactly one terminal, and 2.0 rows were the env's double-grant bug fixed 2026-08-28, so older tapes are normalised here. **In full scope this silently flattens the staged ladder.** Now `scope == 'pick'` only.

**The third would have shipped.** It produces a well-formed dataset with no error, in which every stage pays the same. Measured: source `dHfull_all_rx` totals 238 with values {1:136, 2:19, 4:13, 6:2}; converted it read **170, every value 1.0**. The only symptom downstream would have been learners that never prioritised finishing the task — indistinguishable from a genuine null.

**Two further conversion errors caught the same way, by reconciling totals rather than reading exit status:** `--terminal-reward` defaults to **100** and multiplies the tape rewards, giving a first conversion of 17000 against a source of 238 — which, with the return clamp at 8, would have saturated the critic immediately. Both were caught because the reported total did not match the source, not because anything failed.

**Full scope retains a real check** rather than losing one: rewards must be finite and non-negative.

**Verified after the fix:** converted totals 238 and 237, exactly matching source, with magnitudes intact ({1:136, 2:19, 4:13, 6:2} human; {1:133, 2:17, 4:16, 6:1} machine).

**Note on provenance:** the pre-existing full-scope sets carry {1,2,4,6}, so they predate all three behaviours. The converter accumulated pick-scope assumptions *after* the full-scope data it is meant to produce, and nothing re-derived that data in between.

---

## Amendment (z) — the unified ladder, and a staged-versus-sparse verification pilot (registered 2026-09-11, BEFORE any pilot job)

Registers the design of `paper/LADDER_UNIFY_BRIEF_2026-09-10.md` (D1–D8), the implementation of
`paper/LADDER_IMPL_NOTES_2026-09-10.md`, the calibration of `paper/NESTED_V2_PREDICATE_2026-09-10.md`,
and the pilot the user asked for on 2026-09-11 00:40 ("start a set of shorter runs to verify").
Written and committed before any pilot job was submitted; the four demonstration sets and the four
smokes below already exist and are reported with the commands that produced them.

**Why there is an amendment at all.** `E2E_AUDIT_BRIEF_2026-09-10.md` §2 established that the two
learners of the 64-run long batch trained on DIFFERENT reward ladders — `FULLENV_REWARD_X=1` was
passed to all 64 jobs and was inert in the tree {RLPD} loaded. §8 defect 5 established that the (x)
ladder's top rung could never be paid in training, because the episode terminated on the UNPAID
`nested` proxy whose clauses are a subset of the slide's. §4a established that that proxy REVERSES
the human-versus-machine ordering. This amendment removes the gate, the proxy terminal and the
proxy itself from every paid and terminal path.

### (z).1 The two ladders

Both are compiled into one tree and selected by a **constructor argument**, never an environment
variable, and both are stamped into every log, checkpoint sidecar, `ladder_provenance.json` and
`metrics.json` (D6).

| ladder | rungs (sticky, paid once, in-episode) | terminal | max return = r2dreamer `return_clamp` | demo sets |
|---|---|---|---|---|
| `staged` | `picked` 1 / `placed_v2` 1 / `contact_push` 2 / `slide_success` 4 | `slide_success`, `tipped` | 8 | `_rz` |
| `sparse` | `nested_v2` 1 | `nested_v2`, `tipped` | 1 | `_rs` |

`contact_push` requires `placed_v2` to have been GRANTED first (the release-gated form), so the
rung a policy could previously farm by pressing the still-held can against the goal no longer
exists. `nested_v2` replaces `nested_proxy` in every log, table and figure; the legacy `nested`,
`placed` and bare `contact` are still computed and logged and pay nothing. Everything other than
reward and terminal — tip rule, every logged stage, the tracker diagnostics, the observation, the
amendment-(w) episode record (now always on, D8) — is identical between the two ladders, so an
arm-versus-arm comparison across them is a comparison of objectives and nothing else.

### (z).2 The stamp, and the code of record

`$LAB/gp_unified` @ `21c58b49` (branch `ladder-unify-2026-09-11`; the smoke logs quoted below were
produced at `b89478d3`, which differs only in the evaluator fix of §(z).10 item 6 — a file that is
not in the stamp), world-model port `$W/r2dreamer_unified` @ `77b2c61`. `git describe` reads `-dirty` in every job because the run
registry (`cluster/RUN_REGISTRY.jsonl`, seeded from `$LAB/gp_e2e` so duplicate submissions stay
detectable) is a tracked file that every {RLPD} job appends to; the state is the same for both
learners, so the four fields P1 compares are unaffected.

    [ladder] unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 |
             max_return=8 | terminal=slide_success+tipped | shaping=off |
             full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 |
             git=known-good-2026-08-27-815-gb89478d3-dirty

    [ladder] unified-2026-09-10 | ladder=sparse | nested_v2=1 |
             max_return=1 | terminal=nested_v2+tipped | shaping=off |
             full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 |
             git=known-good-2026-08-27-815-gb89478d3-dirty

### (z).3 The demonstration sets (D5: relabel by re-execution)

Built by `cluster/relabel_e2e_sets.sbatch` → `baselines/rl/relabel_reward.py`, job **3537411**, one
node, four sets in sequence, **pax080, 64 physical cores, AVX-512, Intel Xeon Gold 6438M** (the
core count is read from `/proc/cpuinfo` inside the job and asserted, never taken from a Slurm
feature label). Each tape is re-executed through `FullTaskEnv(scope='full', ladder=…)` — the same
code path training uses — so **tape reward == env reward by construction**. There is no predicate
in the builder.

| set | ladder | tapes | decisions | Σ recorded | Σ re-exec | picked | placed_v2 | contact_push | slide_success | nested_v2 | pushed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `dHfull_all_rz` | staged | 74 | 29 221 | 118 | **171** | 65 | 40 | 9 | **12** | 14 | 27 |
| `dDPfull_first_rz` | staged | 72 | 36 834 | 131 | **183** | 64 | 41 | 13 | **13** | 15 | 25 |
| `dHfull_all_rs` | sparse | 74 | 29 221 | 118 | **14** | 65 | 40 | 9 | — | **14** | 27 |
| `dDPfull_first_rs` | sparse | 72 | 36 834 | 131 | **15** | 64 | 41 | 13 | — | **15** | 25 |

(`contact_push` / `slide_success` in the sparse rows are LOGGED, not paid; they are listed to show
that the two ladders see the same episode.) Episode end reasons: human staged `slide_success` 12 /
`tipped` 14 / `stream_exhausted` 31 / `truncated` 17; machine staged `slide_success` 13 /
`tipped` 13 / `stream_exhausted` 7 / `truncated` 39; human sparse `nested_v2` 14 / `tipped` 14 /
`stream_exhausted` 30 / `truncated` 16; machine sparse `nested_v2` 15 / `tipped` 13 /
`stream_exhausted` 5 / `truncated` 39.

**Action streams are byte-identical to the sources, checked twice.** The builder asserts the
sha256 of `action` per tape and over the set; an independent pass
(`verify_sets.py`, written for this registration) re-loaded all 292 tape pairs and compared every
array: `action`, `state`, `image`, `discount`, `is_first`, `is_last`, `is_terminal`, `logprob` are
identical in every tape, **`reward` is the only array that differs**, and the only new keys are
three `reward_*` provenance fields and nine `rz_*` diagnostics (twelve in all). Reward value census —
human staged `{1: 104, 2: 8, 3: 1, 4: 12}`, machine staged `{1: 104, 2: 12, 3: 1, 4: 13}`, both
sparse sets `{1: n_paying}` — every value inside the ladder's reachable subset sums, which is what
`full_demos.py` validates.

**Selection provenance is INHERITED, never re-derived.** `relabel_reward.py` now writes the
`repeat.json` both launchers gate on (it previously wrote only its own `manifest.json`, which
neither launcher reads — that alone would have stopped every pilot job). It copies the source
manifest and overwrites only the counts the relabel changed. `dDPfull_first_rz` / `_rs` therefore
carry `one_per_ic_first: true`, the PHASE_PLAN (v) de-selection attestation, taken from the source
rather than from a command-line flag — the `b756259` defect, where a set selected upstream recorded
`one_per_ic_first: False`, a false claim rather than a missing key.

**Disclosed: `terminal_reward: 1.0` and the other inherited stamps describe the SOURCE build**
(`to_dreamer_native.py`), not the re-executed reward column. They are kept because the world-model
launcher asserts `terminal_reward`; they are not evidence about this ladder.

**Disclosed: `is_terminal` is left exactly as recorded** (LADDER_IMPL_NOTES §4, resolved here in
favour of keeping the streams whole). A relabelled tape can therefore carry its recorded terminal
at a different decision from the re-execution's, and decisions after the re-execution's terminal
are paid 0. The alternative — marking the new terminal — requires truncating the tape, which
changes the action stream and breaks the sha256 identity that makes this a relabel rather than a
rebuild. Every tape records `rz_end_reason` and `rz_end_decision`, and the manifest counts them.

**Disclosed: re-execution fidelity is bimodal. WHICH tapes diverge is a property of the tape; HOW
FAR they diverge, and therefore what they are paid, is machine-dependent** (§(z).4 quantifies the
second half against an independent build).  `can_dev_max_m` is the largest can-position difference
between the recording and the re-execution:

| set | p50 | p75 | p90 | max | ≤ 10 mm | 10–50 mm | > 50 mm | worst joint error |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| human | 8.1 mm | 62.3 mm | 168.7 mm | 685.7 mm | 39/74 | 14 | 21 | 0.059 rad |
| machine | 10.8 mm | 51.3 mm | 157.7 mm | 754.0 mm | 34/72 | 20 | 18 | 0.281 rad |

Lane 2's off-cluster dry run saw the same two of its six tapes diverge (78 mm, 155 mm) that this
64-core cluster run also diverges on (180 mm, 142 mm) — different magnitudes, same tapes — so the
mechanism is chaotic amplification after contact, and the machine class sets its size rather than
its location. **What matters for the
reward column is where the divergence sits relative to the grants, and it sits after them:** every
one of the 12 human and 13 machine tapes that pay the top rung re-executes to between 1.3 mm and
**33.9 mm**, except machine uid 302 at 145.7 mm; the six worst tapes in each set (246–754 mm) pay
`picked` only, granted early while the trajectory still matches. The residual honest caveat is that
on a badly diverging tape the STATE column (recorded) and the REWARD column (re-executed) describe
different trajectories. That is a consequence of D5 as designed and is not fixed here; the
alternative (writing the re-executed states as well) would make the tape self-consistent at the
cost of no longer being a relabel of the set of record, and is a coordinator decision.

### (z).4 P3, stated against Lane 1's expectation, with the uids

Expected (LADDER_UNIFY_BRIEF, "Lane 1 outcomes that bind the merge"): the human `_rs` set pays on
**11** tapes and the human `_rz` top rung fires on **≤ 11**. Measured: `_rs` pays on **14** and
`_rz`'s top rung on **12**. The uids:

* `_rz` `slide_success` (12): 232 233 236 237 247 251 273 275 302 304 316 317
* `_rz` / `_rs` `nested_v2` (14): the 12 above plus **244** and **299**
* `_rz` `contact_push` (9): 233 236 255 259 273 299 305 321 325
* machine `_rz` `slide_success` (13): 233 242 243 251 259 262 263 265 274 281 302 305 321
* machine `nested_v2` / `_rs` paying (15): the 13 above plus **295** and **300**
* machine `_rz` `contact_push` (13): 233 242 251 257 262 275 277 279 295 298 302 305 308

**The second half of P3 is met exactly.** Every paying `_rs` tape pays exactly one +1 and no other
tape pays anything; the `nested_v2` grant DECISION is identical between the two ladders on all 74
human and all 72 machine tapes (`cmp.py`, zero disagreements), which is the strongest available
evidence that the two sets differ in reward and terminal alone.

**The first half is not met, and the reason is that the expectation was keyed to a different
lineage.** Lane 1's 11 was measured on the `src_dHfull_all` census recording; the pilot trains on
`dHfull_all`, a DIFFERENT recording of the same 74 initial conditions (CLAUDE.md: the two differ on
24 of 74 action streams). **Lane 5's independent full-set census, run on the local box through this
same relabel path, pays the sparse rung on 14 human tapes of `dHfull_all` as well** — so 14, not
11, is the number for this set, on two machines independently, and P3's "11" is a statement about
the census lineage. Both counts belong in the record. The 64-core cluster build is the count of
record because it is the hardware the pilot runs on.

Coarse stages agree across every reading — Lane 1 reports `picked` 65, `placed_v2` 40,
`released` 40, `tipped` 14, identical to this build — so the environment and the predicate are not
in dispute; the disagreement lives entirely in the late, contact-rich stages, and there it is a
MACHINE effect (next paragraph). One reading difference also has to be kept straight: Lane 1's
headline 11 is an end-of-episode read ("end or lastK") and their sticky variant reads 12, while the
environment grants stickily.

**Per-tape agreement between the two machines, which is what the `can_dev` column exists to
measure.** Comparing every tape's six stage grants (cluster manifests versus Lane 5's
`census_{human,machine}.json`):

| arm | tapes agreeing on all six stages | Σ staged, cluster v box | `nested_v2` | `slide_success` | `placed_v2` | `contact_push` | `pushed` |
|---|---|---:|---:|---:|---:|---:|---:|
| human | **68 / 74** | 171 v 179 | 14 v 14 | 12 v 13 | 40 v 42 | 9 v 10 | 27 v 28 |
| machine | **64 / 72** | 183 v 192 | 15 v 16 | 13 v 14 | 41 v 44 | 13 v 14 | 25 v 28 |

Every disagreeing tape is one whose can diverges: human 236 (22.5 mm, cluster-only slide),
244 (15.9 mm), 256 (64.9 mm), 259 (14.9 mm), 274 (14.1 mm), 326 (84.0 mm); machine 246 (18.0),
254 (46.4), 255 (6.0), 266 (64.2), 274 (3.7), 295 (606.3), 311 (308.0), 328 (20.6). Note that the
human `nested_v2` TOTAL is 14 on both machines with **different membership** — 236 and 244 on the
cluster, 256 and 259 on the box — so an equal count is not evidence of an equal set. This is
hardware-class sensitivity of a contact-rich re-execution, the same phenomenon as the eval-cell
node sensitivity in `cluster/e2e_eval_cells.sh`, and it is why the set of record must be built
once, on one machine class, and stamped (it is: pax080, 64 cores, in every `repeat.json`).

The three uids the brief names as known misses — **255, 305, 308** — are absent from this build's
`nested_v2` set too, consistent with Lane 1's account (255 and 305 end while the can is still
moving, so `at_rest` cannot hold; 308 never gets `placed_v2`; 255 and 305 do appear under
`contact_push`, which does not require `at_rest`). **Uid 308 additionally differs by lineage**:
its settled `nested_honest` is 0 on `dHfull_all` (Lane 5's census, both arms checked) and 1 on the
census recording — a second instance of the same lineage effect, on the reference predicate rather
than the new one.

The settled `nested_honest` reference has NOT been re-derived on a 64-core node. It is obtained by
running `baselines/diagnostics/replay_tape_stagerec.py` plus `nested_v2_validate.py` there. No
pilot number depends on it: the demonstration sets are built and stamped, and the pilot's statistic
is measured on policies, not on tapes.

### (z).5 P2 in the demonstration sets

Structural, checked on the built sets rather than asserted: every tape granting `contact_push` is
also a tape granting `placed_v2` (human 9 of 9, machine 13 of 13); every tape granting
`slide_success` also grants `nested_v2` and `pushed`. No tape grants `contact_push` without a prior
`placed_v2`, count **0**, which is what P2 predicts for the policy rollouts.

### (z).6 Five properties of this ladder that are REGISTERED, not fixed

Found by Lane 5's census of the demonstrations. Each is a consequence of definitions this
amendment adopts deliberately. **No predicate and no reward is changed for any of them**; they are
written down so that a reader of a pilot number knows what it means.

1. **The rungs are not nested.** `slide_success` (+4) pays WITHOUT `contact_push` (+2) on **10 of
   13** human slides and 6 of 14 machine slides on Lane 5's box census, and on **9 of 12** human
   and **7 of 13** machine slides in the cluster build of record, because `nested_v2` uses the
   0.081 m proximity of the metric of record while `contact_push` requires SOLVER contact, and two
   cans touch at
   0.066 m. A tape can therefore arrive without the two rungs below it ever firing, and a return of
   6 (1+1+4) is a complete task, not a partial one. By design; disclosed.
2. **The sparse rung can be paid by a pure set-down.** `nested_v2` can be granted on the same
   decision as `placed_v2` with `pushed` False — releasing the can at the goal pays it at the
   instant of release, because `placed_v2`'s 10-frame sustain already implies at rest. **P8 is
   therefore already answerable on the demonstrations, and it comes out the opposite way round from
   P8's prediction**: on Lane 5's box census humans slide 13 v drop 1 and the machine set slides
   14 v drops 2; on this cluster build, human 12 v 2 (drop-route uids 244 and 299) and machine
   13 v 2 (295, 300). Both ways round, the demonstrations reach settled contact by the SLIDE route
   in the large majority. P8 as registered is about what the POLICIES do, and that stays open.
3. **The reference predicate has no release clause.** `nested_honest` — the settled predicate
   `nested_v2` was validated against — nests a can that is STILL IN THE GRIPPER on 1 of 146 tapes
   (machine 283: lever 0.014 m, grip 0.63, `placed_v2` never granted, settled `nested` = 1).
   `nested_v2` rejects it. The reference is the weaker predicate in this one respect.
4. **Four tapes' recordings stop mid-push**, so `contact_push` fires on their LAST decision
   (human 255 and 305, machine 277 and 257). Their rung is real but their episode is truncated by
   the recording, not by the task.
5. **`placed_v2` at grip < 0.45 admits a fist push.** Human 232 completes its slide at a commanded
   grip of 0.41. That is amendment (p)'s intent — (p) withdrew (l)'s `grip < 0.3` clause precisely
   because humans release and then push with the fingers re-closed — and it is disclosed here
   rather than treated as a leak.

### (z).7 The pilot

Both learners run from `$LAB/gp_unified`; {r2dreamer} additionally from `$W/r2dreamer_unified`
(named by `R2_TREE`, which this amendment makes REQUIRED — it used to be hardcoded to the
in-flight `$W/r2dreamer_fix`). QOS **normal**, partition **gpu**, so the pilot does not compete
with the old batch's preempt allocation. Job names are `lz_<rl|r2>_<staged|sparse>_<dH|dM>_s<seed>`.

| learner | ladder | arms | seeds | budget | checkpoints / milestones |
|---|---|---|---|---|---|
| {RLPD} | staged | human / machine-first | 2 v 2 (940–941 / 960–961) | 100k decisions | `ckpt_040` 40k, `ckpt_100` 100k |
| {r2dreamer} | staged | same | 2 v 2 (940–941 / 960–961) | 1M ONLINE steps | 0.5M, 1M |
| {RLPD} | sparse | same | 2 v 2 (945–946 / 965–966) | 250k decisions | `ckpt_016` 40k, `ckpt_040` 100k, `ckpt_100` 250k |
| {r2dreamer} | sparse | same | 2 v 2 (945–946 / 965–966) | 4M ONLINE steps | 0.5M, 1M, 2M, 4M |

The sparse arm runs at the FULL budget because a sparse null at a short budget is uninformative;
its 100k / 1M milestones give the like-for-like read against the staged pilot at the same step.
`R2_LONG_RUN=1` makes the world-model budget explicitly ONLINE (origin + budget), which the (x)
batch did not: those runs did 3 970 594 and 3 962 512 online steps for a "4M" budget.

n = 2 per arm. **This is a verification pilot and a sparse-feasibility probe; it is NOT a
demonstration-source comparison**, and no cell from it may be reported as one.

### (z).8 Predictions and decision rules (verbatim from the brief, plus their checks)

* **P1** every run's `ladder_provenance.json` and Slurm `[ladder]` stamp are identical within a
  ladder and differ between ladders ONLY in `ladder`, `stage_reward`, `terminal_stages`,
  `return_clamp` (assert; a mismatch aborts the pilot).
  *Status before submission: MET on four smokes — {RLPD} staged **3537917**, {RLPD} sparse
  **3538260**, {r2dreamer} staged **3538337**, {r2dreamer} sparse **3539030** (the last was in
  flight when this amendment was first committed and is recorded here on its readout:
  `[ladder] ladder=sparse return_clamp=1.0 (env.return_clamp AND model.return_clamp)` plus the
  trainer's independent `[ladder] return_clamp=1.0 (env and model agree)`). Both learners
  print the stamps in §(z).2 with the same three file hashes, character for character (the `git`
  field differs between the smokes run before and after the evaluator fix of §(z).10 item 6, a file
  that is not in the stamp and not one of the four fields P1 compares); the
  world-model launcher and trainer additionally agree on `return_clamp` (8.0 staged / 1.0 sparse)
  and the trainer refuses to start if `env.return_clamp` and `model.return_clamp` do not both equal
  the ladder ceiling.*
* **P2** zero `contact_push` grants without a prior `placed_v2` in every episode record
  (structural; count = 0).
* **P3** the `_rz` and `_rs` human sets pay their top rung on the same tapes the (x) classifier
  selects; `_rs` pays exactly one +1 per paying tape at the `nested_v2` frame.
  *Status: second clause MET exactly (one +1 per paying tape, at a `nested_v2` decision identical
  to the staged set's, on all 146 tapes). First clause NOT MET as written — 14 paying human tapes
  and a top rung of 12, against an expected 11 — because the expectation was keyed to the census
  lineage and not to `dHfull_all`; §(z).4 gives the uids, the independent confirmation on a second
  machine, and the per-tape cross-check.*
* **P4** by the end of the staged budget ≥ 1 seed per arm shows `contact_push` in training
  rollouts; disconfirm → rerun that arm with D7 `goalward` shaping ON, disclosed.
* **P5** max episode return ≤ 8 (staged) / ≤ 1 (sparse) and no negative drift in `picked` versus
  the (x)-batch curves at the same step.
* **P6** no job ends `FAILED 2:0 00:00:00` (requeue guard).
* **P7 (the user's question, sparse arm)** settled contact is reached by ≥ 1 seed per arm within
  the full budget.
* **P8** the route census — prediction: under BOTH ladders the majority of `nested_v2` events have
  `pushed` = False (drop route); disconfirm = slide route ≥ 50 % in any arm, which would mean
  paying the outcome alone induces the slide.

**Decision rule, registered now:** if sparse reaches `nested_v2` at ≥ the staged rate at the
matched milestone, the intermediate rungs are unnecessary and sparse becomes the primary for the
16v16; if sparse is 0 in all seeds at the full budget while staged > 0, the shaping is necessary.
**Readout:** `rnd30` mode cells at every milestone with `nested_v2`, `slide_success`, `pushed`,
`nested_honest`. NOT a source comparison (n = 2).

### (z).9 The submit commands of record

The 16 commands live in `cluster/submit_lz_pilot.sh`, committed with this amendment, so that what
was registered and what ran are the same text rather than two retypings of it:

    cd $LAB/gp_unified
    DRYRUN=1 bash cluster/submit_lz_pilot.sh    # prints the 16 commands, submits nothing
    bash cluster/submit_lz_pilot.sh             # submits

Each is of the form

    GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified LADDER=<staged|sparse> ARM=<dH|dDPfirst> SEED=<s> \
      STEPS=<100000|250000> DEMO=$W/demos_state_full/<set> CKPT_FRACS=<0.4,1.0|0.16,0.4,1.0> \
      sbatch -J lz_rl_<ladder>_<dH|dM>_s<s> -p gpu --qos=normal --nice=0 cluster/sbatch_rlpd_e2e.sh

    R2_TREE=$W/r2dreamer_unified GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified LADDER=<staged|sparse> \
      R2_LONG_RUN=1 R2_MILESTONES='<list>' \
      sbatch -J lz_r2_<ladder>_<dH|dM>_s<s> -p gpu --qos=normal \
        cluster/wmfix_full.sbatch <set> <seed> <1000000|4000000>

The script re-checks, before submitting anything, that both trees are what they claim to be, that
all four `repeat.json` manifests exist, and that the filesystem is above the registered 150 GB
floor. Every job id, its full command and the `[ladder]` line from its own log go in
`paper/LADDER_PILOT_LOG_2026-09-11.md`.

### (z).10 Nine defects found and fixed around the merge and the submission

Seven of the eight were found by RUNNING the thing, not by reading it. Two would have stopped
every job at the gate; two would have made a run lie about its own objective; one killed a
training run three minutes in; two would have let an arm train to its full budget and then produce
no cell at all; and the last would have produced eight plausible, wrong cells. The first seven are
committed before any pilot job; **the eighth (below) was found AFTER submission, from the smoke's
own written cell, and fixed before any pilot job started** — the evaluator is a separate process
launched at the end of a job, so the fix reaches every pilot run.

1. **Relabelled sets had no `repeat.json`.** `relabel_reward.py` wrote only `manifest.json`, which
   NEITHER launcher reads. Both gate on `<set>/repeat.json`. Every pilot job would have exited at
   the demo gate. (`9647d58`)
2. **`cluster/wmfix_full.sbatch` hardcoded its world-model tree** to `$W/r2dreamer_fix` — an
   in-flight tree, and the same "the launcher, not the submission, decides which code runs" defect
   as the `GENESIS_PICKAPLACE_ROOT` default that put the two learners on different ladders.
   `R2_TREE` is now required. (`9647d58`)
3. **The world-model trainer stamped the module default ladder.** `train.py` called
   `ladder_provenance()` / `ladder_stamp()` with no argument, so every sparse run would have
   written `staged` into its own `ladder_provenance.json` and printed a staged `[ladder]` line —
   D6's failure mode reintroduced one level down. Fixed to read `config.env.ladder`, and to REFUSE
   to start unless `env.return_clamp` and `model.return_clamp` both equal the ladder's ceiling
   (a staged clamp of 8.0 on a sparse run silently disables the clamp that made this port stable).
   (r2dreamer `b4ca104`)
4. **The online-budget contract was only half ported.** Under `R2_LONG_RUN=1` the trainer
   re-targets the counter to origin + budget, but `train.py`'s accounting print and its assertion
   still read `env.steps` as a raw counter target: a long-run job logged "4000000 → 3970594 online
   env steps" while actually running 4 000 000, and a budget smaller than the prefill died on an
   assertion that does not apply to it. Found by the 15k-step smoke on a 29 406-row prefill.
   (r2dreamer `49c02f8`)
5. **The D8 record certificate was missing from ordinary step rows.** `observation_space` and
   `reset()` declared `log_ep_record_valid`; `step()` did not, and `envs/parallel.py` stacks the
   vector observation key by key — so the first time one sub-env sat on a reset row while the
   others stepped, training died with `KeyError: 'log_ep_record_valid'`. LADDER_IMPL_NOTES §4 had
   flagged this merge as "sound on paper and untested in a buffer". (r2dreamer `77b2c61`)
6. **A sparse checkpoint could not be evaluated at all.** `eval_e2e.py`'s `--ladder` DEFAULTED to
   `staged` and its sidecar-disagreement check fired against that default;
   `cluster/e2e_eval_cells.sh` passes no `--ladder`, so every cell of a sparse run died with
   "checkpoint sidecar says ladder='sparse' but --ladder is 'staged'" — the sparse arm would have
   trained for 250k decisions and produced nothing to read. The sidecar is now the SOURCE and
   `--ladder` an optional ASSERTION. Found by running the smoke THROUGH its eval stage instead of
   stopping at TRAIN-OK. (`21c58b4`)
7. **The world-model evaluator's summary collided with itself.** `scope='full'` now names
   `slide_success` as its success key (D2/D4 moved it off the withdrawn proxy), and
   `eval_genesis.py` splatted `**{success_key: ...}` into a `dict()` call that already passes
   `slide_success=` explicitly: `TypeError: dict() got multiple values for keyword argument
   'slide_success'`, raised at the LAST step, after a complete training run and 15 evaluated
   episodes. All eight {r2dreamer} pilot cells would have been lost. Both quantities (an outcome
   rate over the terminal taxonomy, and a stage-grant rate) are now kept under names that cannot
   collide. Found by the staged smoke, job 3538337. (r2dreamer `197a1a3`)

8. **The world-model evaluator built a STAGED environment for a SPARSE run.**
   `eval_genesis.py` constructed `GenesisPick(...)` without `ladder=`, so it took the constructor
   default. A sparse policy would have been rolled out under a different terminal
   (`slide_success` rather than `nested_v2`) and a different reward from the one it optimised —
   all eight sparse {r2dreamer} cells wrong, and wrong in the quiet way, with a plausible number
   in every column. **Caught by the D6 stamp doing exactly its job**: the sparse smoke's own
   `fresh_eval_hold15_sample/metrics.json` carries `ladder: staged, max_return: 8.0` on a run
   whose logdir, config and training stamp all say sparse. The ladder now comes from
   `cfg.env.ladder`, the source is printed, and the env's own provenance stamp is asserted against
   the config before any episode runs. (r2dreamer `f1c134c`)
9. **The outcome taxonomy's success was a hardcoded `slide_success` in BOTH evaluators.** That is
   the paid terminal of the `staged` ladder only; under `sparse` it is `nested_v2`, so every sparse
   success that did not also satisfy `pushed` was recorded as a **`timeout`** — an arm's outcome
   column contradicting its own stage column, in a project where `timeout` is already a residual
   label. **The stage columns were never affected** (they are read from the env's sticky grants, so
   `nested_v2` — the sparse arm's statistic of record, and what P7 is read from — was correct
   throughout); this is the outcome / `success_rate` half only. The terminal now comes from the
   env's own D6 provenance stamp in both evaluators, and `eval_e2e.py` writes it into
   `metrics.json` as `terminal_stage` so a reader can see which taxonomy a row used.
   (`7913caa`, r2dreamer `903c6a2`)

Defects 8 and 9 both landed AFTER the 16 jobs were submitted and BEFORE any of them started; all
16 were still PENDING, so `$LAB/gp_unified` was fast-forwarded and every pilot job runs the fixed
code and stamps one `git describe`.

`pytest` was also run on the cluster for the first time (Lane 2 could not): **37 passed**
(`test_ladder_unified.py`, `test_stage_predicates.py`, `test_terminal_guard.py`).
