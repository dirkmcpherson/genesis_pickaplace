# Silent-Default / Fallback Audit — baselines/ + cluster/ (2026-08-17)

Scope: read-only sweep of the eval/aggregation path, dataset converters and demo
loaders, env-construction boundaries, and cluster shell env-var defaults, hunting
for lookups/branches that produce a plausible wrong answer instead of crashing.
Pattern reference: grip-column ×3, control-mode ×3, the `m.get('picked')`
aggregator (0/90 silently), wandb_eval `--uids` drop.

Files read in full: wandb_eval.py, eval_core.py, dp_runner.py, wandb_utils.py,
eval_watcher.py, convert_to_lerobot.py, train_rlpd.py, train_sacfd_full.py,
rlpd_sac.py, demo_buffer.py, full_env.py, genesis_can_env.py, cartesian_env.py,
ic_sampling.py, harvest_ai_demos.py, relabel_cartesian.py, pick_env.py (head),
build_obs_action_2x2_v2.py, backfill_evalcurve.py, sacfd_delta_gate.py (head),
to_dreamer_demos*.py (grep+excerpts), sbatch_rlpd.sh, sbatch_eval_x2x2.sh,
sbatch_r2dreamer.sh, sbatch_ouro_harvest.sh, launch_paper_week.sh (grep),
sbatch_paper_smoke_harvest.sh (grep), genesis_vec_env.py (excerpt),
collect_cartesian_dataset.py (excerpt).

---

## CRITICAL findings (could corrupt a paper number)

### C1. Grip-column bug, 6th sighting: CartesianFullTaskEnv NESTED PROXY reads `a_phys[4]` in 6-DOF modes
`baselines/rl/full_env.py:581`

```python
        if info.get('contact') and float(a_phys[4]) < 0.3 \
                and 'nested' not in self._granted:
```

The tip rule 20 lines below was fixed for exactly this (`full_env.py:603`,
`_grip = float(a_phys[6] if len(a_phys) >= 7 else a_phys[4])`, comment says
"5th sighting") — but the per-step nested proxy right above it was not. In
`abs6`/`delta6` control the physical action is 7-dim `[pos3, rotvec3, grip]`,
so `a_phys[4]` is a WRIST ROTATION component, routinely `< 0.3` (rotvecs live in
±1.6 and hover near 0). Concrete wrong answer: any cartesian 6-DOF FULL-scope
training or eval through `CartesianFullTaskEnv` (SACfD `--cartesian --control
abs6`, non-VEC dv3 cartesian) grants nested (+4) and TERMINATES the moment
`contact` + both-cans-upright holds, **while the gripper is still commanded
closed** — the "gripper commanded open" half of the proxy is never actually
tested. Rewards, returns, and termination stats in those runs are silently
wrong. 5-dim modes (`vel`/`delta`/`abs`) are unaffected (index 4 is grip
there). Fix shape: reuse the exact `_grip` expression from line 603 (compute it
once, before the proxy).

### C2. Missing `stage` field silently ranks a demo as `'contact'` in the SACfD/RLPD relabelers
`baselines/rl/train_sacfd_full.py:49` (relabel_full) and `:158` (relabel_hold_region)

```python
        ep_stage = str(d['stage']) if 'stage' in d.files else 'contact'
```

The stage gate exists because the per-frame geometric proxies false-positive;
its whole guarantee is "an episode that never reached contact can never grant
contact reward." An npz with NO `stage` field (harvests predating 07-26 stage
stamping; foreign/rebuilt sets; any collector that forgets the key) is silently
granted rank 3 — pick/place/contact rewards become grantable on proxy alone,
including on true FAILURE tapes. In `relabel_hold_region` a stage-less no-pick
tape is additionally counted as a POSITIVE, so it pays hold reward AND slips
past the `n_negative_rewarded_frames == 0` census assert (that assert only
covers rank<1 episodes). Nothing prints; training proceeds and the run reports.

Same missing key, three different silent answers across the sibling relabelers:
- `relabel_full` / `relabel_hold_region` → `'contact'` (over-grants — worst)
- `baselines/rl/to_dreamer_demos.py:94` → `'picked'` ("old harvests lack it")
- `baselines/rl/relabel_cartesian.py:36` → `'no-pick'` (fail-safe direction,
  but see M6: it silently ZEROES rewards instead)

Fix shape: assert `'stage' in d.files` (all current sets carry it), or at
minimum default to `'no-pick'` + a loud per-file warning, identically in all
three.

### C3. harvest_ai_demos SAC branch ignores `--action-space`, `--control`, and the action_mode sidecar
`baselines/harvest_ai_demos.py:91-99` (with the import at `:44`)

```python
    if teacher_type == 'sac':
        from stable_baselines3 import SAC
        model = SAC.load(checkpoint)
        print('[teacher] SAC loaded', flush=True)

        def policy_action(obs):
            a_norm, _ = model.predict(obs['state'], deterministic=True)
            return denormalize_action(a_norm)
```

`denormalize_action` here is `pick_env.denormalize_action` — JOINT ABSOLUTE.
Two silent-wrong invocations:

1. **delta_joint SAC teacher** (every current paper SACfD/RLPD arm:
   `launch_paper_week.sh` defaults `SACFD_ACTION_MODE=delta_joint`;
   sbatch_rlpd.sh trains delta_joint): its normalized per-step DELTAS are
   denormalized as absolute joint targets — the wrong MDP. The
   `<ckpt>.action_mode.json` sidecar that wandb_eval consults is never read
   here. `--verify` CANNOT catch this (it deterministically replays the same
   wrong actions), so any rollout where the flailing arm happens to game the
   pick predicate is kept, verified, and serialized as `stage='picked'` model
   demos. The `MIN_KEPT` gate and the fling-signature warning
   (short-rejects > kept) are the only backstops, and both are heuristic. A
   dSACfD "model demos" dataset harvested this way corrupts the paper's central
   demo-source comparison.
2. `--teacher-type sac --action-space cartesian`: only the RANDOM teacher gets
   the cartesian denorm dict (`:104-106`); a cartesian SAC checkpoint gets the
   7-dim JOINT denorm, and `CartesianCanEnv.step` then silently consumes
   joint radians as velocities and `a[4]` (a joint angle) as grip.

Fix shape: mirror wandb_eval — read the sidecar, assert
`action_mode == 'absolute'` (or implement the delta integrator), and route the
SAC denorm through the same action_space/control dispatch the random teacher
uses.

### C4. wandb_eval still silently drops `--uids` unless `--random 0` (known trap, NOT fixed)
`baselines/wandb_eval.py:256-264`

```python
_ic_sets = {}
if args.random:
    if args.ic_mode in ('demo', 'both'):
        _ic_sets['indist'] = ic_sampling.demo_ics(env, reps=1)[:args.random]
    if args.ic_mode in ('random', 'both'):
        _ic_sets['random'] = ic_sampling.sample_support_ics(
            env, args.random, seed=args.seed)
else:
    _ic_sets['indist'] = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)
```

`--random` defaults to 10, so `--uids 243` without `--random 0` evaluates the
first 10 demo ICs instead and writes them into a json the caller believes is
uid-243-specific. This is memory-documented as a trap
(wandb-eval-uids-silently-ignored.md) but the code has no assert and no
warning. The one automated caller (`cluster/sbatch_rlpd.sh:82`) protects itself
with `--random 0`; every ad-hoc invocation is one forgotten flag away from
per-uid sweep jsons containing the wrong episodes — data-corrupting, not just
run-wasting, because the output files are labeled by uid. Companion trap in the
same block: `demo_ics(env, reps=1)[:args.random]` silently caps the demo-IC
count at `--random`. Fix shape: `assert args.uids is None or args.random == 0`.

---

## MODERATE findings (could waste a run / produce a wrong intermediate)

### M1. train_sacfd_full periodic checkpoints get NO action_mode sidecar (fix applied only to RLPD)
`baselines/rl/train_sacfd_full.py:513` uses stock `CheckpointCallback`; only
`sacfd_final` gets a sidecar (`:525-526`). train_rlpd built
`SidecarCheckpointCallback` (`train_rlpd.py:313-333`) precisely because
`--action-mode auto` on a snapshot without a sidecar falls back to absolute and
"a delta policy then evals ~0.00 and the zero looks like a result"
(newbox_supp, 2026-08-13). Post-hoc curves over `sacfd_<N>_steps.zip` from a
delta_joint SACfD run hit exactly that fallback (a printed note is the only
tell). Also the final sidecar records ONLY `action_mode` — no
control/cartesian/scope — so a cartesian SACfD checkpoint's sidecar reads
`{"action_mode": "absolute"}` and the control mode still travels by operator
memory (see M2).

### M2. Control-mode `'vel'` fallbacks survive at four dict-dispatch sites + unvalidated constructor
- `baselines/cartesian_env.py:152-161`: `CartesianCanEnv.__init__` accepts ANY
  `control` string; `step()`'s dispatch ends in a bare `else:` = velocity
  (`:301-305`). A typo ('abs 6', 'velocity', future mode) silently runs the
  velocity integrator.
- `baselines/rl/full_env.py:571-576`: `.get(self.control, denormalize_action)`
  — unknown mode → velocity denorm.
- `baselines/wandb_eval.py:117-121`, `baselines/harvest_ai_demos.py:104-106`,
  `baselines/rl/to_dreamer_demos_cartesian.py:117-119`: same
  `.get(control, <velocity fn>)` shape (argparse `choices` currently guards the
  entry points; the library-level fallback remains).
Also `--control` defaults to `'vel'` on wandb_eval (`:67`),
train_sacfd_full (`:419`), harvest_ai_demos (`:243`), eval_watcher (`:29`) —
evaluating/harvesting a delta or abs6 policy without the flag silently runs the
wrong command type and reads ~0.00. Fix shape: assert
`control in {'vel','delta','abs','abs6','delta6'}` in
`CartesianCanEnv.__init__` and make the `step()`/denorm dispatch exhaustive
(`'vel'` an explicit key, unknown → KeyError).

### M3. Broad `except → grip_effort = 0.0` zeroes an obs dimension silently
`baselines/genesis_can_env.py:305-309` and the batched twin
`baselines/genesis_vec_env.py:311-315`:

```python
        try:
            grip_effort = float(np.abs(np_(w['kinova'].get_dofs_control_force(
                dofs_idx_local=bots))).sum())
        except Exception:
            grip_effort = 0.0
```

If the genesis API call fails (version drift, backend), every obs carries
grip_effort=0 forever with no message — the exact feature v4 datasets added.
Policies trained on real effort would silently degrade at eval (or vice versa),
and the single-env vs vec-env paths could diverge if only one throws. Fix
shape: warn once on first failure, or let it raise.

### M4. relabel_cartesian keys rewards off demo_manifest_auth.json by uid — harvested/model demos silently score `'no-pick'`
`baselines/rl/relabel_cartesian.py:36`:
`stage = man.get(str(uid), {}).get('stage', 'no-pick')`. Harvested tapes carry
uid ≥ 100000 (harvest_ai_demos STEM_BASE) which is never in the manifest, and
their own npz `stage` field is ignored — so a cartesian SACfD run pointed at a
model-demo dir gets ZERO rewarded demo transitions, silently (only visible if
the operator reads the `N rewarded` line). The fail direction is safe for
reward manufacturing but fatal for the run's purpose: "RL can't learn from
model demos" would be an artifact. Fix shape: prefer the npz `stage` field when
present, manifest as fallback, assert one of the two exists.

### M5. Demo-dir + condition defaults diverge between the sibling trainers (the paper's condition axis has defaults)
- `train_rlpd.py:48` `--demo-dir` defaults to the HUMAN set
  (`episodes_pick_phase_all`) — the demo-source axis of the paper defaulted; a
  model-demo launch that forgets the flag silently trains the human arm
  (memory: demo-dir naming trap). `train_sacfd_full.py:405` defaults to
  `episodes_all`.
- Same-named flags, different defaults: `--scope` = `'pick'` (rlpd) vs `'full'`
  (sacfd_full); `--action-mode` = `'delta_joint'` (rlpd) vs `'absolute'`
  (sacfd_full); `--project` = `genesis_paper` vs `genesis_pickaplace`;
  `--eval-max-steps` = 400 vs 1200. Muscle-memory across the two trainers
  produces different conditions silently (all are printed in `[cfg]` and most
  travel in the rlpd sidecar — visible, but only to a reader).
- `demo_buffer.find_demo_paths` (`demo_buffer.py:40-54`) still auto-falls
  through episodes_raw_rl → v4 → v3-with-zero-padded-effort when called without
  paths (legacy train_sacfd path only); it prints, but it *chooses a different
  dataset* rather than failing.
Mitigations that DO hold: sbatch_rlpd.sh requires `ARM` and provenance-checks
filename patterns against the claimed source (`:56-63`) — the model for how the
other launchers should treat these flags.

### M6. eval_watcher: cannot express 6-DOF control, and crashes on its own video log line
`baselines/eval_watcher.py:29` — `--control choices=['vel','delta']
default='vel'`: an abs6/delta6 DP/ACT run cannot be watched at all, and a delta
run watched without the flag produces a silently-zero curve (velocity env).
Separate hard bug at `:83-88`: `metrics` receives `wandb.Video` objects and is
then printed with `f'{v:.2f}'` — `TypeError: unsupported format string` on the
first checkpoint that records videos (verified format-spec behavior), killing
the watcher after its first `run.log`. Not silent, but it truncates live curves
right after they start.

### M7. Cluster env-var defaults that select real-but-possibly-wrong inputs
- `cluster/sbatch_ouro_harvest.sh:37`: `CTRL=${CTRL:-vel}`,
  `ACTIONS=${ACTIONS:-joint}`, `ALGO=${ALGO:-dp}`, `IC_MODE=${IC_MODE:-demo}` —
  a manual (re)submission for a cartesian/delta chain that omits CTRL harvests
  in vel (the chain's own resubmit at `:106` forwards them correctly).
- `cluster/sbatch_r2dreamer.sh:70-75`: DEMO_DIR chosen by glob on the CONFIG
  name (`*v5*` → delta25 set, `*delta*` → delta set, else non-delta) — a config
  whose name doesn't match the pattern (or matches the wrong one first; `*v5*`
  outranks `*delta*`) trains on mismatched demo encodings with only a
  dir-exists preflight.
- `cluster/launch_paper_week.sh:46`: `${SACFD_TEACHER:-baselines/outputs/paper/
  dH_SACfD_s0/sacfd_final.zip}` — a concrete default teacher; combined with C3
  (that checkpoint is delta_joint) the default dSACfD harvest is in the wrong
  MDP. `:105` `--action-mode ${SACFD_ACTION_MODE:-delta_joint}`.
- `${GENESIS_PICKAPLACE_ROOT:-$PWD}` / `:=$PWD` throughout: submitting from the
  wrong directory silently roots all relative dataset paths there (the
  preflights that check `[ -d "$DEMO_DIR" ]` catch most of it).

### M8. harvest_ai_demos full-scope cartesian without `--cap` can loop ~unboundedly
`harvest_ai_demos.py:137` `cap = PICK_CAP if scope == 'pick' else env.max_steps`
with the cartesian env built at `:276` with `max_steps=10**9`: a rollout that
PICKS but never contacts skips the NO_PICK_ABORT and iterates toward 1e9 steps
(hang, not wrong data). Cluster callers pass `CAP=1200` explicitly; local
full-scope cartesian harvests must too. The joint branch (`:279`) keeps the
default 1200-step env horizon, which bounds it.

---

## BENIGN (default genuinely safe — and why)

- `eval_core.py:45` `agg[k] += bool(info.get(k))`: GenesisCanEnv always sets
  picked/placed/contact and sets nested at its horizon, which is when eval_core
  reads it (`while not done`); missing-key scoring can only trigger if a
  foreign env is plugged in. Watch it if eval_core ever scores scope'd envs.
- `eval_core.py:30-31` recording silently disabled without a camera: absence of
  videos is immediately visible (standing directive makes videos expected).
- `wandb_eval.py:129-158` sidecar legacy fallbacks (absolute / repeat-1 /
  target): each prints its fallback, a `measured` sidecar is ALWAYS honored via
  assert, and `--delta-ref` conflicts refuse loudly. Deliberate, documented,
  and the dangerous direction is asserted.
- `wandb_eval.py:288,295` `_aggs.get(...) or first`: agg dicts are non-empty →
  always truthy; the `or` never mis-selects.
- `dp_runner.py:33` `.get('type', 'diffusion')`: lerobot always writes `type`;
  a mismatch fails loudly at `from_pretrained` (state-dict/config error).
- `dp_runner.py:50-51` raises when a vision checkpoint lacks a rig_provider —
  correct anti-silent behavior.
- `full_env.py:301` / `:551` `(options or {}).get('uid') or self.fixed_uid`:
  uid 0 does not exist (uids ≥ 232), so the falsy-zero hazard is unreachable.
- `genesis_can_env.py:86-88` `world_cfg.get('substeps', 1)` etc.: defaults
  equal the committed table's values; the table is the source of truth.
- `convert_to_lerobot.py`: positional-argv defaults are documented at the call
  site; `MIN_FRAMES` has an explicit 4th argv for pick-scope; `ds.finalize()` +
  the parquet-rows-vs-info.json integrity assert are present (fix holds).
- `wandb_utils.py` `getattr(self, 'control', 'vel')` etc.: attributes are
  always set in `__init__`; the getattrs are dead defense. `_poll` on rc!=0
  prints and leaves a gap in the curve rather than fabricating a point.
- `rlpd_sac.py`: sb3-version assert, `ent_coef` float() raises on typos,
  `set_demo_data` asserts non-empty, watchdog re-arms. Clean.
- `backfill_evalcurve.py`: regex non-matches are skipped, but it only ever
  ADDS missing rows from an explicit log; the source metrics still exist.
- `sbatch_rlpd.sh:101` and `dp_ckpt_curve.sh:31-35`: metrics readers hard-index
  `['metrics']['eval/picked']` — the `m.get('picked')` aggregator bug shape is
  gone from the active readers (fix holds; sbatch_rlpd even comments it).

---

## Verification of previously-fixed instances

| Known bug | Status |
|---|---|
| Grip column, joint tip rule (`full_env.py:487`) | HOLDS — `a_phys[6]`, comment marks 4th sighting |
| Grip column, cartesian tip rule (`full_env.py:599-603`) | HOLDS — mode-dependent `_grip`; **but the sibling nested proxy at `:581` was missed → C1** |
| Grip column, relabel_cartesian (`relabel_cartesian.py:52-55`) | HOLDS — `gidx = 6 if a.shape[1] >= 7 else 4` |
| Control-mode dispatch, CartesianFullTaskEnv (`full_env.py:566-576`) | HOLDS for known modes; unknown strings still fall to velocity → M2 |
| Control-mode at eval boundary (sbatch_eval_x2x2.sh) | HOLDS — explicit per-cell env/obs mapping, audit-commented |
| Aggregator nested-key (`['metrics']['eval/picked']`) | HOLDS in all current readers |
| wandb_eval `--uids` drop | **NOT FIXED** — documented only; still silent → C4 |
| `--action-mode auto` sidecar for RLPD snapshots (`train_rlpd.py:313-333`) | HOLDS — sidecar written next to every snapshot + startup snapshot dir; **not ported to train_sacfd_full → M1** |
| #26 inner-env horizon (`genv.max_steps = 10**9`) | HOLDS in FullTaskEnv (`:192`), CartesianFullTaskEnv (`:536-537`), harvest cartesian (`:276`); harvest joint env keeps 1200 but the rollout loop breaks on `done` in the same iteration |
| convert_to_lerobot finalize + integrity gate | HOLDS (`:95-109`) |
| P2 reset controller-target re-issue (`genesis_can_env.py:169-179`) | HOLDS |
| RLPD demo-RNG per-seed (`rlpd_sac.py:179-182`) + sbatch git gate | HOLDS |

---

## Full table

| # | File:line | Pattern | Wrong-answer scenario | Severity |
|---|---|---|---|---|
| C1 | rl/full_env.py:581 | magic grip index in 6-DOF | abs6/delta6 full-scope: nested +4/termination while gripping | CRITICAL |
| C2 | rl/train_sacfd_full.py:49,158 | missing-key default `'contact'` | stage-less npz relabeled up to contact; negatives paid; census assert bypassed | CRITICAL |
| C3 | harvest_ai_demos.py:91-99 | teacher denorm ignores mode/sidecar | delta_joint or cartesian SAC teacher harvested in wrong MDP; verify can't catch | CRITICAL |
| C4 | wandb_eval.py:256-264 | `--uids` dropped when `--random`>0 | uid-labeled jsons contain other episodes; demo-IC count capped by `--random` | CRITICAL |
| M1 | rl/train_sacfd_full.py:513,525 | no sidecar on snapshots; sidecar lacks control | delta SACfD snapshot curves eval absolute→0.00; cartesian ckpt control by memory | MODERATE |
| M2 | cartesian_env.py:152,301; full_env.py:571; wandb_eval.py:117; harvest_ai_demos.py:104; to_dreamer_demos_cartesian.py:117 | `.get(control, vel)` + unvalidated ctor | typo'd/new mode silently runs velocity control | MODERATE |
| M3 | genesis_can_env.py:305-309; genesis_vec_env.py:311-315 | broad except → 0.0 | grip_effort obs dim silently zeroed forever on API failure | MODERATE |
| M4 | rl/relabel_cartesian.py:36 | manifest miss → `'no-pick'` | harvested (uid≥100000) cartesian demos get zero reward silently | MODERATE |
| M5 | rl/train_rlpd.py:48; rl/train_sacfd_full.py:405,419,431,444; rl/demo_buffer.py:40-54 | semantic argparse defaults / dataset fallback chain | wrong arm/condition trained by default flags | MODERATE |
| M6 | eval_watcher.py:29,83-88 | control choices too narrow; Video in `:.2f` | delta/abs6 watch silently wrong or impossible; watcher dies after 1st video ckpt | MODERATE |
| M7 | cluster/sbatch_ouro_harvest.sh:37; sbatch_r2dreamer.sh:70-75; launch_paper_week.sh:46,105 | `${VAR:-…}` semantic defaults | vel harvest of a delta chain; config-glob demo dir; default delta teacher into C3 | MODERATE |
| M8 | harvest_ai_demos.py:137,276 | cap = env horizon = 1e9 | full-scope cartesian harvest w/o `--cap` loops on picked-no-contact | MODERATE |
| B1 | eval_core.py:45 | `info.get(stage)` | only misfires with a foreign env; current envs set keys | BENIGN |
| B2 | wandb_eval.py:129-158 | sidecar legacy fallbacks | printed; measured direction asserted | BENIGN |
| B3 | full_env.py:301,551 | `uid or fixed_uid` | uid 0 unreachable (uids ≥232) | BENIGN |
| B4 | dp_runner.py:33 | `.get('type','diffusion')` | mismatch fails loudly at load | BENIGN |
| B5 | genesis_can_env.py:86-88 | world_cfg `.get` defaults | defaults == committed table values | BENIGN |
| B6 | wandb_utils.py:33-47,109-118 | offline fallback; failed-eval gap | both print; no fabricated data | BENIGN |
| B7 | convert_to_lerobot.py:36,46,60 | argv defaults | documented, integrity-gated output | BENIGN |

## Suggested fix order
1. C1 — one-line index fix mirroring `full_env.py:603` (affects any live 6-DOF cartesian training).
2. C4 — one assert in wandb_eval (`uids ⇒ random==0`).
3. C2 — assert `'stage' in d.files` in relabel_full/relabel_hold_region; unify the three relabelers' missing-key behavior.
4. C3 — sidecar read + action-space dispatch in harvest_ai_demos' SAC branch (before any dSACfD harvest).
5. M1 — port `SidecarCheckpointCallback` to train_sacfd_full; add control/cartesian to its sidecar.
6. M2 — assert valid `control` in `CartesianCanEnv.__init__`; make dispatches exhaustive.
