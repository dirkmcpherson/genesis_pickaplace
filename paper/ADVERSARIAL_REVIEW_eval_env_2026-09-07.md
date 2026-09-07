# Adversarial review — evaluation and environment code (2026-09-07)

*Mandate: break the code that produces every success number — stage predicates (`genesis_can_env.py`,
`rl/full_env.py` + the private cluster copies), `eval_genesis.py` (success keys, `stages`, `restore_failed`,
`--ic-set all`, `--entry-bank`, `--dump-entries`), the DP/RLPD eval path (`wandb_eval.py`, `eval_core.py`,
`dp_runner.py`), the world-model adapter (`r2dreamer_fix/envs/genesis.py`), the restore-failure mechanism, and
the `contact_push` patch. Read-only. Repo = `~/workspace/genesis_pickaplace` @ `4dof-cartesian` (working tree,
uncommitted `contact_push` included); cluster = `pax`, `$LAB=/cluster/tufts/shortlab/jstale02`,
`$W=$LAB/wm_fix_2026-09-03`. Reproductions written only under `~/wm_fix_2026-09-03/adv_review/`.*

---

## Findings

### S1-1 — the end-to-end arm's `nested` is the TRAINING PROXY, not the settled nested; the docs describe it backwards. CONFIRMED

`eval_genesis.py:263` maps `scope='full' → success_key "nested"`. That flag comes from
`full_env.py:663-670`:

```
if info.get('contact') and float(a_phys[6]) < 0.3 and 'nested' not in self._granted:
    if tilt_deg(bottle) < 20 and tilt_deg(goal) < 20:  info['nested'] = True
```

i.e. **sticky `contact` earlier in the episode ∧ grip COMMANDED OPEN ∧ both cans upright**, granted the
instant those hold and terminating the episode (`full_env.py:783`). The honest predicate
(`genesis_can_env.py:317-327`: 100 settle steps, then centre distance ≤ `NESTED_TOUCH_DIST` 0.081, picked,
both upright) can never run on this path, because the adapter sets `self.genv.max_steps = 10**9`
(`full_env.py:359`, by design) so `GenesisCanEnv.step` never reaches its `done` branch.

Consequences:
1. `REVIEW_GUIDE §2.7` "nested 0.163 vs 0.192" means *"the can touched the goal at some earlier step, and at
   some later step the grip was commanded open with both cans upright"* — **no proximity at the end and no
   settle**. A can knocked away after contact and then released still scores.
2. `PHASE_RESULTS_2026-09-05.md:145` states "`nested` requires picked + proximity + both upright + settled,
   not a release", and `REVIEW_GUIDE §4.4` says "`nested` does not require release". Both describe the *other*
   predicate. The one that scored these runs **requires a release and does not require proximity**.
3. **Cross-learner parity is broken for this stage**: DP/RLPD go through `eval_core.run_eval` on a raw
   `GenesisCanEnv` (`eval_core.py:47`, `wandb_eval.py:154-159`), which reports the *honest* settled `nested`.
   Any table putting a world-model `nested` next to a DP/RLPD `nested` compares two different quantities.

*Smallest fix:* at the eval's terminal call `env._env.genv._nested()` (or set `genv.max_steps` to the eval
horizon) and report that; otherwise rename the column `nested_proxy` and restate §2.7.

### S1-2 — the "policies carry without releasing" route claim rests on a predicate registered as unearnable, and is contradicted by the metric that granted the successes. CONFIRMED

`PHASE_RESULTS §5.1:145` and `REVIEW_GUIDE §2.7`: *"the release-based `placed` stage is granted 0–4 times in
240 episodes → the policies carry the held can straight into the goal without ever releasing it."*

- `placed` (`genesis_can_env.py:271-272`) has **no release term at all**: it is `picked ∧ in_shelf_footprint ∧
  BOX_TOP_Z+0.01 < can_z < BOX_TOP_Z+0.07`, i.e. a z-band *while held*.
- That band is the **stale base-world band 0.12–0.18** m. In `gc_kp4_riser3_shelf6` the shelf top is 0.17 and a
  can resting on it sits at 0.2205 (measured, `adv_review/goal_spawn_probe.py`). `PHASE_PLAN:15` and
  amendment (a) (`PHASE_PLAN:74`) already declare this flag "unearnable in this world … NOT used"; CONFOUNDS
  row 47 likewise. It nevertheless carries the route argument.
- CONFIRMED from the evaluation files: every place-scope polE cell reports `stages.placed = 0.0` (0/148);
  full-scope cells report 0.033/30 and 0.067/15 — a transient shelf-penetration artefact (the only way a can
  centre reaches 0.12–0.18 over the shelf footprint is while intersecting the shelf box, e.g. falling past its
  front face). So the "0–4 of 240" is a bug rate, not evidence about releasing.
- Worse: by S1-1 **every episode counted `nested` did command grip < 0.3** — it released. The route narrative
  is contradicted by its own success predicate.

*Smallest fix:* delete the inference from §5.1 and §2.7, or replace it with `placed_v2` computed in `full`
scope (today `full_env.py:764-772` computes `placed_v2` only under `scope=='place'`, so the `placed_v2` column
in the end-to-end `stages` block is structurally 0 — correctly noted at `PHASE_RESULTS:134`, but then the
weight is shifted onto the broken `placed`).

### S1-3 — place-scope evaluation does not evaluate the bank entries it reports: it draws them uniformly at random with replacement and silently substitutes any entry that fails to restore. This is also the actual mechanism of the "restores in one scope but not the other" mystery. CONFIRMED

`eval_genesis.py:185-190`:

```
if getattr(env._env, 'scope', 'full') == 'place':
    env._env.reset()                            # <- uid NOT pinned
else:
    env._env.reset(options={"uid": int(uid)})
```

`FullTaskEnv._reset_place` (`$W/gp_root/baselines/rl/full_env.py:502-510`) then takes `pool = self._entries`
(all 148) and draws `self.np_random.integers(len(pool))`, retrying up to `PLACE_MAX_TRIES = 30`.

- **The restore code and the survival predicate are byte-identical for `place` and `carrycontact`** —
  `_reset_place`/`_restore_place_entry` serve both (`full_env.py:488`), and both test `bp[2] > PLACE_HELD_Z`
  (`full_env.py:483`). The *only* difference between the two evaluations is the uid pinning above. With a
  pinned uid the pool is one entry, so a failing entry is retried 30 times identically and raises →
  `restore_failed`; unpinned, a failing entry is replaced by a random other entry and the episode still runs.
- This resolves the open mechanism in `WM_FIX_LOG` 2026-09-05 15:25 / 15:40 / 15:55 (three successive
  withdrawals, ending "mechanism stays open"). No history-dependent physics is needed. Evidence in the
  evaluation files: `carrycontact` polE cells report `restore_failed = 5/148` in **every** cell of both arms;
  all 32 `place` polE cells report `restore_failed = 0/148`.
- Consequences: (a) "148 policy-generated starts" (`REVIEW_GUIDE §2.4`) is really **148 draws with replacement
  over ~94 distinct entries**; (b) the five unrestorable entries are excluded from `place` but counted as
  failures in `carrycontact`, so §2.4 and §2.6 are **not on the same bank**; (c) `metrics.json` per-episode
  `uid` and the video filenames record the enumerated pseudo-uid (`eval_genesis.py:330`, `_ic_label`), **not
  the entry actually restored** — any per-start analysis or matched-start reel taken from place-scope records
  is mislabelled. (Pairing the two arms *by episode index* is still valid: both draw from the same seed-0
  stream, seeded once at `_build`.)

*Smallest fix:* pin the uid in the place scope as well (or enumerate `self._entries` directly) and report `rf`
for place; re-describe §2.4's denominator.

### S2-4 — `--dump-entries` stores the RAW normalized grip action as `grip_cmd`; the restore reads it as a physical 0..1 grip. CONFIRMED

`eval_genesis.py:314` writes `grip_cmd=float(a[6])` where `a` is the agent's action in `[-1,1]^7`; the
physical grip is `(a[6]+1)/2` (`full_env.py:653`). The restore then does
`gripper_targets(float(e['grip_cmd']) * 100.0)` (`full_env.py:478` and `:561`), and `gripper_targets`
(`replay_harness.py:84-91`) clips `g_pos/100` to `[0,1]`. The human banks store the physical command
(`make_phase_banks.py:39`, `grip_cmd=float(a[f,6])` on a 0..1 tape).

Measured on the banks (`$W/phase_banks`):

| bank | grip_cmd range | interpretation |
|---|---|---|
| `polE_contact` (machine-generated) | −0.991 … −0.127 | every entry restores with the fingers commanded **fully open** (clipped at 0) |
| `polE_place` (machine-generated) | −0.032 … 1.000, mean 0.823 | true physical mean 0.911 → systematic under-squeeze |
| `holdE_place` / `holdE_contact` (human) | 0.674…0.915 / 0.000…0.450 | already physical 0..1 |

**35 of 148** `polE_place` entries restore with the fingers commanded to OPEN relative to the measured closure
(`grip_cmd < grip_obs`); with the correct scale only **5 of 148** would. This is exactly the observation the
log recorded at 15:40 ("entry can z 0.190, grip_cmd 0.657 vs grip_obs 0.799 — the restore commands the fingers
to … OPENING from the measured closure") without identifying the cause: 0.657 is the raw action; the physical
command was 0.829, i.e. a *closing* command. Symmetric across arms (one bank per cell), so the Δ is not
biased, but the bank does not reproduce the states it claims to, and it is the plausible driver of the
marginal restore failures.

### S2-5 — `contact`'s directional clause is vacuous, and the new `contact_push` far-side test measures the WRIST, not the gripper. CONFIRMED by measurement

Both use `ee = w['eef'].get_pos()` (`genesis_can_env.py:262`), the **wrist link**, ~0.145 m behind the tool
tip (the file's own `PICK_EEF_DIST` note: a held can "tracks the eef at ~0.146 m"; `tool_pos()` exists at
`genesis_can_env.py:105`). I teleported the arm to every banked qpos in the w3 world and evaluated both tests
against the recorded can/goal (`~/wm_fix_2026-09-03/adv_review/contact_geometry_probe{,2}.py`):

| bank (n) | `ee_x < can_x` (`contact`, l.279) | `dot<0` with WRIST (`contact_push`, l.302) | `dot<0` with TOOL tip | `tool_x < can_x` |
|---|---|---|---|---|
| polE_place (148, can held at pick grant) | **148/148** | 132/148 | 100/148 | 69/148 |
| polE_contact (160, can just released on shelf) | **160/160** | 123/160 | 123/160 | 126/160 |
| holdE_place (13) | 13/13 | 13/13 | 11/13 | 11/13 |
| holdE_contact (11) | 11/11 | 7/11 | 9/11 | 9/11 |

- `contact`'s `ee_x < can_x` clause holds on **332/332** real states of both arms: it rejects nothing.
  Describing `contact` as "eef behind the can" (`PHASE_PLAN:13`) or as containing a directional requirement
  overstates it; operationally `contact ≡ picked ∧ solver contact can↔goal`. (It also admits the gripper
  touching the goal while holding the can — no exclusion.)
- `contact_push` (amendment (g)) is meant to require "gripper and goal on opposite sides of the pick-can". With
  the wrist, `dot < 0` already fires on **132/148 held pick-grant states** and **123/160 just-released
  states** — configurations where nothing has been pushed. It will therefore grant on a pure carry, which is
  the case it exists to exclude. Switching to the tool tip changes the answer materially (148/148 → 69/148 on
  the same states), so the choice is load-bearing, not cosmetic.

*Smallest fix (before any re-score is read out):* use `self.genv.tool_pos()` (calibrated at reset, as
`GenesisCanEnv.reset` does) instead of `w['eef'].get_pos()` in the `contact_push` block. The
gripper↔goal-contact clause is sound as written.

### S3-6 — `--ic-skip` records a stale `stages` dict, and crashes if the skipped index is 0. CONFIRMED by inspection

`eval_genesis.py:274` references `ep_stages` inside the skip branch, but `ep_stages` is only bound at
`:308`. Skipping episode 0 raises `NameError`; skipping any later index writes the **previous** episode's
stages dict (by reference) into that episode's record. Used in the rnd300 sweep (`--ic-skip 269`). Summary
counts are unaffected (`stage_counts` is not incremented on a hang), so no headline number moves; the
per-episode records for skipped episodes are wrong.

### S3-7 — the shelf band and the world are resolved from *different* sources; nothing asserts they agree

`full_env.py:382-388` derives `self.shelf_top_z` from `R2D_SIM_VARIANT`/`GENESIS_SIM_VARIANT`; the assertion
there (`_vn != 'base' or _dz == 0.0`) is a tautology and the only other signal is a print. `wandb_eval.py:129-150`
resolves the variant from `--sim-variant`/the checkpoint sidecar (with a correct FATAL on mismatch) and never
exports it to the environment. So the DP/RLPD place/contact evaluations the user asked for on 09-07 would
build the w3 world and compute the band from `base` → `placed_v2` unearnable and `_restore_contact_entry`'s
band check always false. *Fix:* pass the resolved variant into `FullTaskEnv` and assert
`shelf_top_z == sim_variants.shelf_top(name)` against the built world.

### S3-8 — the RLPD eval's delta constants are hard-coded, unlike every neighbouring constant

`wandb_eval.py:236`: `DJ_CAP, DJ_LEASH = 0.025, 5.0*0.025`, with no sidecar read — while `action_repeat`
(`:297-307`) and `delta_ref` (`:207-221`) both read the sidecar and FATAL on mismatch. The world-model adapter
uses `DELTA_CAP = 0.04`, leash `3×` (`envs/genesis.py:52-54`), so the two learners act in different MDPs (a
known asymmetry, but I could not find it disclosed in `CONFOUNDS.md`), and any RLPD run trained at another cap
would be silently evaluated at 0.025. The adapter's leash `3×0.04 = 0.12` is also just below the demos' p99 PD
lead of 0.126 quoted in its own comment.

### S4-9 — `_restore_contact_entry` (`full_env.py:542-567`) is a verbatim copy of `_restore_place_entry`
(`:447-483`) differing only in the return predicate — the duplicated-predicate family this repo has been
bitten by three times (grip column ×3, control mode ×3).

---

## Claims that survived

- **Cluster vs repo divergence.** `genesis_can_env.py`, `wandb_eval.py`, `eval_core.py`, `dp_runner.py` and
  `eval_ics.json` are **byte-identical** between the repo working tree and `$W/gp_root/baselines/` (`diff`, all
  IDENTICAL). `full_env.py` differs only by the registered `contact`/`carrycontact` scope patch and
  `phase_sparse`; `sim_variants.py` differs only by variant rows the cluster copy predates. No unregistered
  divergence found. (`contact_push` is present on both sides, uncommitted, and is logged-only —
  `full_env.py:686-688` never rewards or terminates on it.)
- **The stale goal z in the rnd IC files is harmless.** `eval_ics.json`/`eval_ics_rnd300.json` carry
  `goal_pos z = 0.203` (the base-world spawn), which in w3 puts the goal can 1.7 cm inside the raised shelf.
  I built the w3 world and measured (`adv_review/goal_spawn_probe.py`): after reset z = 0.2047, after 100 steps
  z = 0.2199 with **0.00 cm xy drift and 0.0° tilt** — identical to the uid path's settled pose (0.2199), and
  goal z is not in the observation. No number is affected. Tried to break it; could not.
- **rnd30 vs rnd300 are the same distribution.** Both are i.i.d. uniform over the same frozen support box
  (`ic_sampling.sample_support_ics` and `make_eval_ics_rnd300.py:14-18`, box read from `eval_ics.json` rather
  than the drifted table), seeds 0 and 1, `can_z` and `goal_pos` copied. The "rnd300 agreed with rnd30" retest
  is like-for-like.
- **Horizon parity between learners.** World model: 300 decisions × `action_repeat` 4 (`eval_genesis.py:294`,
  `:124`) = 1200 `FullTaskEnv` steps; DP/RLPD: `GenesisCanEnv(max_steps=1200)` (`wandb_eval.py:30`, `:154`).
  Each is 3 scene steps, so both arms get 3600 physics steps.
- **`picked` parity.** Both paths take `picked` from the same hardened `PICK_SUSTAIN = 10` predicate
  (`genesis_can_env.py:60, 265-270`); the §2.1 headline is like-for-like across learners. The whack-fling
  guard (`|ee−can| < 0.20` sustained) is intact on both.
- **`restore_failed` is symmetric within a scope.** Verified: 5/148 in every carrycontact polE cell of both
  arms, with all-False `stages` and the episode counted in `n` (`eval_genesis.py:283-287`) — the
  "counted as failures, symmetric" disclosure in §2.6 is accurate *within* carrycontact (see S1-3 for the
  cross-scope problem).
- **Adapter/`FullTaskEnv` action-repeat is not double-applied**: the adapter does the repeat itself
  (`envs/genesis.py:255-275`) and constructs `FullTaskEnv` without `action_repeat` (default 1), and
  `sync_delta_target` is re-seeded on every reset path including `reset_to_uid`/`reset_to_ic`.
- **State layout matches the banks.** `state = [q(6), motor, grip_effort, can(3), quat(4), goal_xy(2)]`
  (`genesis_can_env.py:337-341`); `--dump-entries` slices `[:6]`, `[6]`, `[8:11]`, `[11:15]`, `[15:17]`
  correctly (`eval_genesis.py:313-316`). Only `grip_cmd` is on the wrong scale (S2-4).

---

## Summary (≤ 15 lines)

1. The end-to-end arm's success metric is the **training proxy** `nested` (sticky contact + grip commanded
   open + both upright), not the settled proximity test; the docs of record describe it backwards, and it is
   not the same `nested` the DP/RLPD path reports. **S1.**
2. The "policies carry the can in without releasing" route claim is built on `placed`, a flag that has no
   release term and whose z-band is the stale base-world band (registered as unearnable in w3, measured 0/148
   in every place cell); the `nested` successes it is contrasted with all commanded a release. **S1.**
3. Place-scope evaluation never pins the bank entry: it draws uniformly with replacement (~94 of 148 distinct
   entries per cell) and silently substitutes any entry that fails to restore. This — not history-dependent
   physics — is why the same five entries fail in carrycontact and never in place (the restore code is shared;
   only the uid pinning differs). Three log entries chasing this mechanism can be closed. **S1.**
4. `--dump-entries` stores the raw `[-1,1]` grip action where the restore expects physical `0..1`, so 35/148
   machine-generated place entries restore with the fingers commanded *open* (should be 5) and every
   `polE_contact` entry restores fully open. Symmetric across arms, but the banks do not reproduce the states
   they claim to. **S2.**
5. `contact`'s "eef behind the can" clause is vacuous (332/332 banked states pass), and the new `contact_push`
   far-side test uses the wrist rather than the gripper tip — it fires on 132/148 *held* pick-grant states, so
   it will grant on the carry it exists to exclude. Fix before the re-score reads out. **S2.**
6. Smaller: `--ic-skip` writes a stale/unbound `stages` dict; the shelf band is read from an env var that no
   eval path asserts against the built world (blocks the requested DP/RLPD phase evals); the RLPD eval
   hard-codes `delta_cap 0.025` while the world model uses 0.04.
7. Survived scrutiny: repo↔cluster code identity, horizon parity, `picked` parity, rnd30/rnd300 distribution
   identity, and the stale goal spawn height (measured benign: settles to 0.2199, 0.00 cm drift).

*Out of scope, S1-adjacent:* if the end-to-end `nested` is rescored honestly (S1-1), `REVIEW_GUIDE §2.7`,
`PHASE_RESULTS §5.1` and the corresponding registration verdicts (P3) all need re-derivation — that is a
statistics-review item.
