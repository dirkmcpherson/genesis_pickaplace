# Evaluation fixes after the eval/env adversarial review, and the re-score of every affected world-model cell (2026-09-07)

Registered as `PHASE_PLAN_2026-09-04.md` amendment (j) (commit 2fd0660, 15:25, BEFORE any job: prep 3350963, re-score `rs2_*` below).
Spec: `ADVERSARIAL_REVIEW_eval_env_2026-09-07.md` (87a6dba). Brief: `~/wm_fix_2026-09-03/agent_brief_eval_fixes.md`. Every number below
is copied from tool output (`rebuild_banks_physgrip.py`, `bank_restore_check.py`, the prep job's console, `eval_fixes_table.py`).

## 1. Fixes (file:line = the patched files; the same patch, `cluster/eval_fixes/eval_fixes_patch.py`, is applied to the repo, `$W/gp_root`, `$W/r2dreamer_fix` and the local mirror `~/wm_fix_2026-09-03/cluster_r2d`; md5 after patch: eval_genesis `1c2a0349…`, envs/genesis `73a41dfd…`, full_env `75707e19…`, identical on all trees)

| # | review | fix | where |
|---|---|---|---|
| 1 | S1-3 | **Pinned bank entries in every scope.** `reset_to_uid` calls `env._env.reset(options={'uid': uid})` for the place scope too (it called `env._env.reset()` unpinned: uniform draws WITH replacement over the whole bank and silent substitution of any entry that failed to restore, while contact/carrycontact pinned — the whole "restores in one scope but not the other" mechanism). A pinned entry that fails to restore raises inside `FullTaskEnv._reset_place` (30 identical retries) → `restore_failed`, counted as a failure. Per episode `restored_uid` / `entry_frame`; summary `pin_stats`; assertion per episode (`restored == enumerated`) and at the end (`n_restored_match + n_restore_failed + n_hang == n_enumerated`). The video name `ep<i>_uid<uid>_<outcome>.mp4` therefore names the restored entry. | `r2dreamer_fix/eval_genesis.py:181-205` (reset_to_uid), `:319` (per-episode assert), `:446` (final assert) |
| 2 | S2-4 | **Physical grip in dumped banks.** `GenesisPick.grip_phys(a) = (clip(a[6], −1, 1) + 1) / 2` is the one policy→physical map; the adapter's `step` now calls it (numerically identical to the inline expression it replaces; `pick_env.denormalize_action` is the same affine map). `--dump-entries` stores `grip_cmd = grip_phys(a)` with `grip_cmd_raw`, `grip_units='physical01'`, `bank_version='physgrip_2026-09-07'`. Banks rebuilt from the dump JSONs (§2). | `r2dreamer_fix/envs/genesis.py:219` (grip_phys), `eval_genesis.py:213` (BANK_VERSION), `:367` (dump) |
| 3 | S1-1 | **Honest nested for the full scope.** `nested_honest` = `GenesisCanEnv._nested()` (100 settle steps at the episode end, centre distance ≤ NESTED_TOUCH_DIST 0.081, picked, both upright — the predicate the DP/RLPD path reports at its horizon). Computed by the adapter at termination AFTER the terminal obs/reward/`is_terminal` are taken (log key `log_nested_honest`, scope=full only; the training reward and terminal are untouched) and by the evaluator at its own horizon — exactly one settle per episode, after the last decision. The old per-step `nested` (sticky contact ∧ grip commanded open ∧ both upright, terminating) is reported as `nested_proxy`; `stages.nested` and the per-episode `outcome` taxonomy are KEPT equal to the proxy so the (g) reproduction check (per-episode contact/outcome/steps vs the cells of record) still holds; new per-episode `outcome_honest` ∈ {nested_honest, proxy_only, tipped, timeout}, summary `nested_honest`, `nested_proxy`, `outcomes_honest`. | `envs/genesis.py:45` (FULL_EXTRA_KEYS), `:324` (settle at termination), `eval_genesis.py:341` (settle at the evaluator's horizon), `:348` (outcome kept), `:286` (STAGES) |
| 4 | S1-2 | **`placed_v2` in the full scope.** The release predicate (grip command < 0.45 ∧ shelf footprint ∧ the WORLD's shelf band `shelf_top_z + [0.01, 0.07]` ∧ tilt < 20°, sustained 10 frames) is computed in scope=full too, LOGGED ONLY (no reward, no termination); `_pv2_run` reset on every reset path; `placed` kept and flagged stale in `stage_notes`. | `baselines/rl/full_env.py:803-816`, `:516`, `:620`; `eval_genesis.py:434-441` (stage_notes) |
| 5a | S3-6 | `--ic-skip` wrote the previous episode's `stages` dict (NameError on index 0): already fixed in the (g) patch (14:43) — verified present in the mirror and on the cluster (`eval_genesis.py:295`). | — |
| 5b | S3-7 | **Shelf band asserted against the built world.** `FullTaskEnv.__init__` derives `shelf_top_z` from `R2D_SIM_VARIANT`/`GENESIS_SIM_VARIANT`; it now asserts that this equals the BUILT world's shelf top (`_world_shelf_top`: the Box entity whose morph size is `BOX_SIZE`, base-link z + half height; `sim_variants.install()` moves that box by `shelf_dz` at build time) on every path that constructs the env. | `full_env.py:388-395`, `:428-437` |
| 6 | — | **Stamps.** Every summary carries `eval_fixes='j'`, `entries_pinned`, `pin_stats`, `bank_path`, `bank_sha256`, `bank_version`, `world_shelf_top_z`, `stage_notes`, so a cell produced by the new evaluator (e.g. a requeued (i) job) is identifiable. | `eval_genesis.py:430-441` |

Not changed: any reward, termination or stored training row; the (i) `wmfix_pheval_cpu` jobs (3350491–535) and the (g′) `cpsc_rescore` lanes (3350666–74) were already running with the old modules loaded. The cpsc lanes' cells reference the canonical bank names, so the canonical files were swapped to the rebuilt banks only after every polE cell of those lanes had ended (§2.3); their cell list / generator now name the byte-identical `*_rawgrip.json` copies for any resubmission.

## 2. Banks

### 2.1 Conversion (`rebuild_banks_physgrip.py`; every merged entry re-derived from its per-checkpoint dump entry — same source checkpoint, ic_index, frame, qpos, stored grip — before conversion)

| bank | n | matched to a dump entry | entries changed | raw grip_cmd < 0 | grip_cmd range raw → physical | mean physical | commanded-below-measured (restore clip) before → after | released (< 0.45) before → after |
|---|---|---|---|---|---|---|---|---|
| `polE_place` (148, dH pick policies) | 148 | 148 | 148 | 2 | −0.032…1.000 → 0.484…1.000 | 0.911 | 35 → 5 | 13 → 0 |
| `polE_place_dDP` (149, dDP pick policies) | 149 | 149 | 149 | 0 | 0.123…0.999 → 0.562…1.000 | 0.908 | 21 → 9 | 11 → 0 |
| `polE_contact` (160, 16 place policies) | 160 | 160 | 160 | 160 | −0.991…−0.127 → 0.005…0.436 | 0.127 | 160 → 151 | 160 → 160 |

"commanded-below-measured" = what the restore actually commanded (`clip(grip_cmd, 0, 1)`) is below the measured closure `grip_obs` of the entry (the review's criterion; 35 → 5 for `polE_place` reproduces its numbers exactly). For `polE_contact` every entry was restored with the fingers commanded FULLY open (raw values all negative → clipped to 0); the physical values are all below the release threshold, i.e. the fingers are commanded open by the amount the policy actually commanded (mean 0.127), and 151/160 are still opening relative to the measured closure — consistent with a releasing gripper. The human banks (`holdE_*`, `human_*`, `machine_*`) are tape-derived physical values and are unchanged.

### 2.2 Restore survival, every entry, raw-grip vs physical-grip (`bank_restore_check.py`, one world per scope, the env's own restore; prep job 3351417)

| scope | bank | n | survived (raw grip) | survived (physical grip) | entries whose survival changed | failing entries |
|---|---|---|---|---|---|---|
| place | polE_place | 148 | 143 (0.966) | 143 (0.966) | 0 | 900002, 900007, 900055, 900058, 900126 (both banks) |
| carrycontact | polE_place | 148 | 143 (0.966) | 143 (0.966) | 0 | the same five |
| place | polE_place_dDP | 149 | 146 (0.980) | 146 (0.980) | 0 | 910043, 910051, 910103 (both banks) |
| contact | polE_contact | 160 | 160 (1.000) | 160 (1.000) | 0 | none |

**Two findings.** (a) *The grip-units bug is not what made entries fail to restore*: over all 467 entries, not one survival
outcome changes between the raw-grip and the physical-grip bank (`fail_before_only` and `fail_after_only` are empty in every
scope). The fix changes what the fingers are commanded to do at the entry — and therefore the episode's opening dynamics —
not whether the state can be restored. The alternative registered in (j) P2 ("if the five failures vanish under physical
grip, the units bug was their driver") is answered: **no**.
(b) *The place/carrycontact asymmetry was purely the missing pin*: the same bank gives byte-identical survival (143/148) and
the identical five failures in both scopes, confirming review S1-3 directly. The carrycontact cells of record were right to
report 5/148 `restore_failed`; the place cells' 0/148 was the substitution artefact. Under the fix the pinned place and
polEdDP cells are expected to report 5/148 and 3/149 restore failures respectively (counted as failures, symmetric across
arms since both arms share the bank).

### 2.3 Names, and how the contact_push lanes were kept clean
Three files per bank: `*_rawgrip.json` (the byte-identical raw-grip original), `*_physgrip.json` (the rebuild: per-entry
`bank_version='physgrip_2026-09-07'`, `grip_cmd_raw`, `converted_from`), and the canonical `polE_place.json` /
`polE_place_dDP.json` / `polE_contact.json`. sha256 (stamped into every re-scored cell as `bank_sha256`):
`polE_place_physgrip` fb48fca7395608fe…, `polE_place_dDP_physgrip` 56d20534262656ad…, `polE_contact_physgrip` 219cb48e70d3bf9c….

The (g)/(g′) contact_push lanes (3350666–74, started 15:13–15:14) had already read their cell list, which names the
canonical files, and their registered protocol is "banks AS THEY ARE"; each of their evals opens the bank at its own start
time. So the **(j) re-score runs against the `*_physgrip.json` files by explicit path** — same content, sha-stamped per cell —
and the canonical names are repointed to the rebuild only after all 64 of those polE cells have written their
`metrics.json` (`cluster/eval_fixes/rs2_swap_banks.sh`, gate checked, raw originals kept). Their cell list and generator
were also pointed at the byte-identical `*_rawgrip.json` copies so any resubmission of those lanes is unaffected by the
repointing. Net effect: no file another agent's running job reads is mutated, and no (j) cell uses a raw-grip bank.

## 3. Evaluator smoke (prep job 3351417, CPU; `rs2_prep/`)

Bank of 7 entries = the five that fail to restore + two controls, place scope, checkpoint `s2_r2d_place_state_dH_bnormclamp1ent5_s0`:

| check | result |
|---|---|
| pinning | `pin_stats {n_enumerated 7, n_restored_match 2, n_restore_failed 5, n_hang 0}`; every survivor's `restored_uid` equals the enumerated uid (900000, 900001) with `entry_frame` 12 / 11; the five failures log the SAME entry retried 30× (`tried [(900002, 12), (900002, 12), …]`) — no cross-entry substitution is possible any more |
| restore_failed accounting | 5/7 = 0.714 reported, all stages False, episode counted in `n` |
| `--dump-entries` units | 2 entries: raw −0.978 → physical 0.011 and raw −0.970 → 0.015, with `grip_units='physical01'`, `bank_version='physgrip_2026-09-07'` (a place-scope dump is taken at the placed_v2 grant, i.e. an OPEN gripper, so small physical values are correct) |
| full scope keys | `nested_proxy`, `nested_honest`, `outcomes_honest {nested_honest 0, proxy_only 0, tipped 0, timeout 1}`, `stages.placed_v2` present; hold IC 0 = uid 252, `restored_uid` 252, timeout at 300 decisions |
| phase scopes | `nested_proxy` / `nested_honest` / `outcomes_honest` are `null` (full scope only), as designed |
| stamps | `eval_fixes 'j'`, `entries_pinned true`, `bank_path`, `bank_sha256`, `bank_version`, `world_shelf_top_z 0.170` |
| shelf assertion (S3-7) | passed on every env construction in the prep job (0.170 = BOX_TOP_Z 0.11 + shelf_dz 0.06 of `gc_kp4_riser3_shelf6`, equal to the built shelf box's top) |

Both smoke evals returned rc=0; `PREP-DONE` 16:12:50.

## 4. Re-score: cells of record vs re-scored (`eval_fixes_table.py`)

PENDING — jobs `rs2_rescore_L0-3` (4 lanes × 4 evals = 16 concurrent, CPU), 320 cells, new dirs `fresh_eval_<tag>_<mode>_v2`.

*Note written before any `_v2` cell exists (2026-09-07 15:5x):* the end-to-end reproduction guard registered in (j) asks that
every per-episode `outcome` / `steps` / `stages.contact` / `stages.nested` equal the cell of record. Within an episode the
fix changes nothing (the settle runs after the last decision), so **episode 0 must match exactly**. Later episodes could
still differ if the 100 settle steps leave solver state that `GenesisCanEnv.reset` does not clear — the same exposure the
DP/RLPD path has had all along (`eval_core.run_eval` calls `_nested()` at the end of every episode, then resets), which is
precisely the protocol this fix restores parity with. So the pre-stated reading is: mismatches starting at episode ≥ 1,
with episode 0 exact, are an inter-episode physics-leak signature of the settle (reported, quantified, and the honest
column still stands because each episode is fully reset); a mismatch at episode 0, or in a bank cell's own first episode,
would be a real bug in the patch and blocks the readout.

## 5. Verdicts

PENDING.

## 6. `slide_success` (PHASE_PLAN amendment (l), fe09d91) — added to the same re-score

Registered predicate: picked earlier ∧ pick-can↔goal solver contact ∧ gripper commanded open (< 0.3) ∧ pick-can centre in
the shelf footprint with tilt < 20°, **sustained 3 decisions**; statistic of record for the slide phase (`scope='contact'`)
and end-to-end. Implemented in `cluster/eval_fixes/slide_success_patch.py` across the same four files as (j); logged only —
no reward, no termination, no training row changes; `contact` and `carrycontact` keep their own statistics.

**The sustain window cannot be measured inside the episode, and this is load-bearing.** Both scopes where `slide_success`
is the statistic of record terminate at the first frame the predicate could hold: `scope='contact'` returns on the first
`contact` frame (`full_env.py:766-768`) and `scope='full'` returns on the nested proxy (`:817`), which fires on
contact ∧ grip commanded open ∧ both upright — at or before slide's clauses. An in-episode counter therefore reaches 1 and
never 3 decisions, so a literal implementation would report `slide_success = 0` in every cell for a purely mechanical
reason (the silent-zero failure mode this repo has been bitten by repeatedly).

**Operationalisation (decided and recorded before any job ran).** The window is evaluated over the continuation that
already exists: the 100-step post-episode settle `_nested()` has always run, during which the last commanded action is
held (controller targets unchanged). The four clauses must hold continuously over its first 12 frames
(3 decisions × action_repeat 4; one frame = the 3 scene steps `env.step` takes). `GenesisCanEnv.end_of_episode()` now
performs that one settle and returns BOTH readings, with the total step budget and the point at which `nested` is measured
unchanged — so `nested` / `nested_honest` stay bit-identical to (j), and only extra state *reads* occur during the window
(reads do not perturb the solver; #26 trace ablation). Each grant records its route: `sustained` (earned inside the
episode, only possible when nothing terminated) or `settle` (earned over the held continuation); route counts are reported
per cell in `slide_routes`, so the two are never conflated. The DP/RLPD path gets the same predicate for free, because the
settle lives in `GenesisCanEnv.step`'s own `done` branch (`eval_core` reports it without changing its `nested`).

`success_key` / `outcome` are deliberately NOT switched to `slide_success`: that would break both the (g) reproduction
guard and the record-vs-re-scored comparison. `slide_success` is a first-class summary field and `stages` column, and the
comparison tables compute the statistic of record from it.

**Prediction (l), evaluated on these cells:** end-to-end rnd30 `slide_success` at or below `nested_honest` in both arms
and |Δ(human − machine)| < 0.10; slide phase |Δ| < 0.10 with both arms well below their bare-`contact` rates.

## 7. Why an end-to-end cell did not reproduce: TWO independent causes, neither of them the patches (investigated 2026-09-07 21:00–22:15)

**Symptom.** `full_r2d_state_dHfull_all_bnormclampS8ent5_s3` rnd30 MODE **episode 0**: cell of record (job 3303000,
2026-09-05) = `nested`, 32 steps; a rerun = `timeout`, 300 steps, no pick. Episode 0 has no history, so this was not the
settle-leak branch pre-stated in §4.

### 7.1 Excluded by measurement (in this order)
- *World, data, config, checkpoint.* Only the four patched files changed in `$W/gp_root` / `$W/r2dreamer_fix` since the
  record; `trial_placements.json` (world block, `substeps 8`), `replay_harness.py`, `sim_variants.py`, `pick_env.py`, the
  URDFs, the run's `.hydra/config.yaml` (09-05 14:29) and `latest.pt` (09-05 21:14, distinct md5 per run) are untouched;
  the ICs compare equal element-wise.
- *Pipeline non-determinism.* Record vs re-run over 23 finished `_cp` cells = **3488 episodes, 0 differences** in steps,
  outcome or contact flag (0.7314 vs 0.7314; `$W/repro_check.py`).
- *Amendments (j) and (l).* With (j)+(l) reverse-applied on a copy of both trees (`rev_apply.py`), the reset state and
  every action/state through 12 decisions are bit-identical to the current trees, **and the full episode ends identically**
  (`nested`/32/r=7.0 under both). **The patches are exonerated.**
- *Job geometry.* `-n 4` vs `-n 8` changes nothing on any node tested.
- *Incomplete reset.* All 21 observable post-reset fields are bit-identical (`reset_probe.py`): qpos, qvel, both cans'
  pos/quat/vel/ang, the 17-dim obs, contact counts, `_granted`, `_pv2_run`, the adapter's delta targets. The env resets
  correctly; an earlier "hidden solver state" reading of this probe was wrong and is withdrawn (see 7.3).

### 7.2 Cause 1 — hardware class (explains the ep0 case)
| node | CPU (from `/proc/cpuinfo`) | ISA | ep0 |
|---|---|---|---|
| **pax109** (the record's node) | Xeon **E5-2695 v4** (Broadwell), 36 | AVX2 | `nested`/32, r=7.0 — reproduces (4 runs, `-n 4` and `-n 8`) |
| **pax154** | Xeon **E5-2695 v4** (Broadwell), 36 | AVX2 | `nested`/32, r=7.0 — reproduces (2 runs) |
| pax001 | Xeon **Gold 6248** (Cascade Lake), 80 logical | AVX-512 | `timeout`/300 (both allocations, all thread pinnings) |
| pax030 | Xeon **Gold 6438M** (Sapphire Rapids), 64 | AVX-512 | `timeout`/300 |
| **pax004** (node of the first failing rerun) | sapphirerapids, 64 | AVX-512 | `timeout`/300 |

The split is exact along the CPU model: the two nodes carrying the *same* pre-AVX-512 Broadwell part reproduce the record
bit-for-bit; every AVX-512 part (Cascade Lake, Sapphire Rapids) diverges. That is the signature of different vectorised
code paths in the Genesis/taichi CPU kernels, not of core count or thread count. **Operational trap:** Slurm's feature
label is unreliable here — `pax001` is advertised `AvailableFeatures=broadwell` but is a Cascade Lake Gold 6248, so a
`--constraint=broadwell` reservation does NOT guarantee a reproducing node; pin by CPU model or by explicit nodelist.

Each node is self-consistent; nodes of different class disagree. **Not fixable by pinning threads:** `TI_NUM_THREADS` ∈
{4, 8, 36} on pax001 and 36 on pax030 all still give `timeout`/300, while pax109 gives `nested`/32 at both 4 and 36. So the
difference is arithmetic-level — AVX-512 versus AVX2 kernels giving different rounding — amplified chaotically over a
300-decision contact-rich horizon. Short phase episodes (1–19 frames) cannot amplify — which is exactly why the 3488
phase episodes reproduce bit-exactly across nodes. *An earlier claim in this section that the split was "not a CPU-family
split" was wrong: it rested on an unverified assumption that the first failing rerun ran on pax154; it ran on pax004.*

This also explains the neighbouring puzzles: the s0 hold15 record was made on **pax033 (64c)**, so re-running it on pax154
(36c) legitimately differs; and the contact_push agent's records came from pax070 (36c) while its reruns ran on pax097
(48c), pax053 (64c) and pax069 (40c).

### 7.3 Cause 2 — the policy consumes global RNG, so episodes in one process are not independent
`Dreamer.act` samples the RSSM posterior latent even at `eval=True` (`--mode mode`), and the evaluator never re-seeds
between episodes. Measured (`rng_test.py`, one process, identical observation): action under a re-seeded RNG is
**bit-identical** to the reference, action after advancing the RNG differs by **3.75e-2**. Hence uid 254 gives r=1.0 run
standalone and r=3.0 as episode 2 of a sequence — same node, same code, same IC.

Consequences: (a) world-model cells are **mode** cells, not deterministic ones — `--mode mode` fixes the actor's output given the latent, but the latent is sampled, so a cell is reproducible only given the RNG stream; a cell reproduces only when
re-run **as a whole sequence from process start**, which is why every whole-cell and ep0 comparison reproduces and only
subset reruns diverge; (b) pulling a single episode out of a recorded sequence and comparing it to that record is invalid —
an artefact I hit myself with uid 254 and initially mis-read as episode-order dependence of the *env*; (c) the fix is a
per-episode re-seed (`torch.manual_seed(seed + ep)`), which would make episodes independent and subset reruns
reproducible — it changes numbers relative to every existing cell, so it needs its own registration, not a silent patch;
(d) `baselines/eval_e2e.py` (DP/RLPD end-to-end, amendment (n)) has the same single-process sequential structure
(one env, `for k, ic in enumerate(ics)`, `genv.reset` redirected), so its cells inherit the same property.

### 7.4 What this means for the numbers
Neither cause is a defect in the environment or in amendments (j)/(l). Both are reproducibility constraints:
**an end-to-end cell is reproducible as a whole sequence on the same CPU class, and not otherwise.** Every end-to-end cell
should therefore record its node and be re-run whole. Phase cells are immune on both counts (short episodes, bank-restored
starts, 3488 episodes bit-exact). Whether the cell-level aggregates of `PHASE_RESULTS §5.1` move is measured by the paired
30-episode in-order reruns — RESULT BELOW (§7.5).

### 7.5 The published cell reproduces exactly: §5.1 stands on its own terms
`dHfull_all` s3 rnd30 MODE, all 30 episodes re-run in order on pax154 (same CPU model as the record's pax109), current
(j)+(l) code:

| | record | re-run | Δ |
|---|---|---|---|
| per-episode outcome / steps | — | **0/30 differ** | — |
| picked | 19/30 (0.633) | 19/30 (0.633) | +0.000 |
| contact | 13/30 (0.433) | 13/30 (0.433) | +0.000 |
| nested | 10/30 (0.333) | 10/30 (0.333) | +0.000 |

So the published aggregates are exactly reproducible under whole-sequence, same-CPU-model conditions, and the patches do
not move them. What the re-score adds on that same cell:

| column | rate | reading |
|---|---|---|
| `nested` = `nested_proxy` | 10/30 = 0.333 | the training proxy — what §5.1 published |
| **`nested_honest`** | **4/30 = 0.133** | the settled predicate DP/RLPD report: the published number over-counts **2.5×**; proxy-only episodes are 0, 3, 15, 17, 18, 27 |
| **`placed_v2`** (fix 4) | **8/30 = 0.267** | earnable in the full scope now; the stale `placed` is 0/30 — 8 episodes DID release onto the shelf band, so the "carry the can in without releasing" reading (§5.1/REVIEW_GUIDE §2.7) is refuted by measurement, not just by argument |
| `contact_push` | 10/30 vs `contact` 13/30 | 3 of 13 contact credits fail the stricter test |
| **`slide_success`** (l) | **2/30 = 0.067** | ≤ `nested_honest`, meeting amendment (l)'s registered prediction on this cell |
| `outcomes_honest` | nested_honest 4 / proxy_only 6 / tipped 9 / timeout 11 | |

`slide_routes = {sustained: 0, settle: 2}`: **both** grants were earned in the held continuation, confirming that a literal
in-episode "sustained 3 decisions" counter would have reported 0/30 (§6).

### 7.6 NEW CONFOUND: the end-to-end arms were evaluated on different hardware classes
Mapping each of the 64 end-to-end record cells to its producing job and node (rnd30 MODE, the §5.1 statistic of record):

| arm | node classes over the 8 seeds |
|---|---|
| human `dHfull_all` | **2 × broadwell/36 (AVX2: s2, s3)**, 1 × graniterapids/96 (s4), 5 × sapphirerapids/64 |
| machine `dDPfull` | **8 × sapphirerapids/64** |

Three human cells were produced on hardware classes that no machine cell used, two of them on the AVX2 class shown in §7.2
to flip long-horizon outcomes. Since CPU class demonstrably changes end-to-end episode outcomes, **hardware class is
partially confounded with arm in the published 8 v 8**. Measured on a real cell: `dHfull_all` s3 rnd30 MODE (record: broadwell) re-run
whole on sapphirerapids gives **24/30 episodes differing**, with aggregates picked 0.633 → 0.600 (−0.033), contact
0.433 → 0.533 (**+0.100**), nested 0.333 → 0.333 (+0.000), and `slide_success` 0.067 → 0.133. So cross-class movement
reaches **0.100 on a stage of record** for this cell, while the two same-class re-runs (pax109, pax154) both give 0/30
and Δ +0.000 on every stage. Recommendation regardless of that number: re-score
all 64 end-to-end cells on ONE pinned CPU model, which removes the confound and yields one internally consistent set; phase
cells need no pinning (3488 episodes bit-exact across classes).

### 7.7 The "re-scoring moves only the human arm" alarm: it is the 36-core class again, and it is a scheduling accident
A tables-side check reported that re-scoring moved 26 human-arm statistics and 0 machine-arm statistics, with equal
cross-class exposure, and concluded the hardware account was dead and a directional defect remained. Re-derived here from
the `_cp` cells directly (`movers2.py`, `contingency.py`, attributing every cell to the job that logged writing that exact
directory, not to the first matching log):

| record-node class | cells whose `picked`/`contact`/`nested` or per-episode outcome moved | unmoved |
|---|---|---|
| **36-core** (Xeon E5-2695 v4, AVX2) | **16** | **0** |
| 32 / 48 / 64 / 96-core | 0 | 8 / 56 / 108 / 4 |

Perfect separation: 16/16 versus 0/176. The movers' originals are **pax109** (the 8 end-to-end cells, `dHfull_all` s2 and
s3) and **pax070** (the 8 contact cells, `dH` sub-floor s0/s1/s4/s5/s7) — both the AVX2 class of §7.2. The claim that no
mover had a 36-core original is inverted; every one does.

*Why it looked arm-directional:* there is no `36-core | MACHINE` row at all — **no machine-arm cell was ever evaluated on
that class**. Which runs landed on the minority hardware is a scheduler accident, and it happened to be human-arm only.

*The other two candidate explanations are excluded by measurement:* all 16 record jobs are `COMPLETED` (none preempted or
requeued), and all records date 2026-09-05/06 while all `_cp` re-scores date 2026-09-07 21:0x — uniform across movers and
non-movers, so no patch boundary separates them.

*A second, benign population was being conflated with the first:* 49 further cells differ **only** in `placed_v2`, moving
from a structural 0 with **zero** differing episodes. That is amendment (j) fix 4 making an unearnable column earnable, and
it is not arm-directional (human 19 / machine 30).

**Conclusion: re-scoring is not directionally biased.** The re-scores are right and the affected originals are the ones
that cannot be reproduced off their own CPU class — which is why the end-to-end re-score pins all 64 cells to one class
(REQUIRE_CORES=64, enforced in the lane), and why the phase pass is unaffected (no place or carrycontact record was
produced on a 36-core node).
