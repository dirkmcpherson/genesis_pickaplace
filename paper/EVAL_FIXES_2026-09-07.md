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

### 2.2 Restore survival, every entry, raw-grip vs physical-grip (`bank_restore_check.py`, one world per scope, the env's own restore; prep job 3350963)

PENDING — filled in from `rs2_prep/restore_<scope>_<bank>.json`.

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

## 3. Evaluator smoke (prep job 3350963)

PENDING.

## 4. Re-score: cells of record vs re-scored (`eval_fixes_table.py`)

PENDING — jobs `rs2_rescore_L0-3` (4 lanes × 4 evals = 16 concurrent, CPU), 320 cells, new dirs `fresh_eval_<tag>_<mode>_v2`.

## 5. Verdicts

PENDING.
