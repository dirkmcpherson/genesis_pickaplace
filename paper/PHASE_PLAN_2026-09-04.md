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
