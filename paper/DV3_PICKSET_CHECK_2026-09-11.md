# DV3 pick-set live-check audit — 2026-09-11 (lane DV3-4)

Answers the question left open by `paper/DV3_LOCAL_2026-09-11.md` §2 ("Live re-execution
check"): lane DV3-1 built `/home/j/data/genesis_pickaplace/demos_state/dHfull_pick_local/`
(74 tapes, cut from `demos_state_full/dHfull_all` at each tape's own recorded `picked` grant)
and then could not independently confirm the cut by replaying 5 tapes through a live
`FullTaskEnv(scope='pick')` — `picked` never fired, 0/5, even with the reset matching the
tape to sub-mm.

**Headline: the demo set is sound. DV3-1's checker was not.** It built the Genesis world
without applying the tapes' own sim variant (`gc_kp4_riser3_shelf6` — no `apply_pre`/
`apply_post`, no `GENESIS_SIM_VARIANT`), so it replayed a world with 1/4 the arm's commanded
joint stiffness, no gravity compensation on the arm, and the whole robot mounted 3 cm too low
relative to the world the tapes were recorded and are trained in. Actions tuned for that world
cannot be expected to reproduce a grasp in a materially different one. With the standard
sim-variant wiring restored (the exact wiring `relabel_reward.py`/D5 and `annotate_demos.py`
already use), the SAME 5 tapes fire `picked` at exactly the decisions the offline cut used —
5/5 — under **both** `scope='full'` and `scope='pick'`. Extending the check to all 74 tapes
(8 processes, 301 s wall) finds the cut exact on 63/64 "pick" tapes, off by one decision on
the 64th, and surfaces one genuine, narrow defect: one of the ten tapes the cut calls
"no-pick" (`genesis-102006-006-296.npz`) actually does pick under the live, current-code
predicate — independently confirmed by Lane 5's census. **The running 1M-step run's demo
buffer does not need to be rebuilt for training to continue**; the one misclassified tape is
a 1/74 (1.4%) blemish, not a systemic problem, and is called out below with the exact fix.

All work below is read-only against the repo (branch `ladder-unify-2026-09-11` @ `2a46abf`);
scripts live in this session's scratchpad (paths given per command) and nothing in the repo
was edited. Genesis backend `cpu` throughout — the GPU stays on the DreamerV3 1M run
(pid 579773), untouched.

---

## 1. Re-execution through the D5 path, both scopes, same 5 tapes

Same 5 tapes DV3-1 used (its doc's table): `genesis-100000-013-256.npz`,
`genesis-101008-029-392.npz`, `genesis-104000-044-196.npz`, `genesis-106001-059-601.npz`,
`genesis-107008-073-601.npz`, from the SOURCE set
`/home/j/data/genesis_pickaplace/demos_state_full/dHfull_all` (`sim_variant=gc_kp4_riser3_shelf6`,
per its `repeat.json`).

### Indexing note (checked before comparing anything)

Two different, both-correct conventions are in play and must be aligned before any number is
compared:
- `build_pick_from_full.py`'s cut (`scratchpad/build_pick_from_full.py:53-59`, DV3-1) finds
  `tstar` = the first **raw row index** in the tape's `reward` array with `reward>0`.
- The D5 loop (`relabel_reward.py`'s `tape_stream`, `relabel_reward.py:366-377`) reads the
  segment layout as **backward-shifted**: `action[0]==0`, and "decision `t` executes
  `action[t+1]` and lands on `state[t+1]`" — so the D5 **decision count** for a reward that
  lands on raw row `tstar` is `tstar - 1`.

This is not a bug in either file, just two valid zero-points; every comparison below converts
`tstar → tstar - 1` before comparing to a D5 "decision" number. The comparison itself proves
the conversion is right: every D5-side source (census, this lane's own re-execution) agrees
with `tstar - 1`, never with `tstar` itself.

### scope='full', D5 path (my own re-execution, not just citing Lane 5)

```
cd /home/j/workspace/genesis_pickaplace
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
  ~/workspace/genesis_sim2real/venv/bin/python <scratchpad>/d5_check.py --scope full \
  --out <scratchpad>/dv3-4/full_result.json
```
Builds the env exactly as `relabel_reward.build_env()` does (`relabel_reward.py:321-348`:
`GENESIS_SIM_VARIANT` exported, `sim_variant_hook.apply_pre`/`apply_post`, `scope='full'`
hardcoded there, `ladder='staged'`, `tip_guard='grip'`), then for each tape calls
`relabel_reward.reset_to_tape_ic()` (`relabel_reward.py:385-413`) and steps
`relabel_reward.tape_stream()`'s actions one decision at a time.

| tape | offline `tstar-1` | live `fired_at` | end (this run) |
|---|---|---|---|
| genesis-100000-013-256.npz | 70 | **70** | terminated (slide_success) @202 |
| genesis-101008-029-392.npz | 146 | **146** | stream_exhausted @391 |
| genesis-104000-044-196.npz | 52 | **52** | stream_exhausted @195 |
| genesis-106001-059-601.npz | 479 | **479** | truncated @600 |
| genesis-107008-073-601.npz | 467 | **467** | truncated @600 |

**5/5 exact match.** World build 22.5 s (this box had already compiled Taichi kernels from the
prior run this session — DV3-1's own 118.8 s / this lane's own scope='pick' run's 147.4 s are the
honest cold-start numbers). `d_can_mm=0.552`, `d_goal_mm=0.000` for every tape (deterministic
settle residual, matches DV3-1's report of a constant, not per-tape, reset error).

### scope='pick', D5 path (the corrected version of DV3-1's check)

```
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
  ~/workspace/genesis_sim2real/venv/bin/python <scratchpad>/d5_check.py --scope pick \
  --out <scratchpad>/dv3-4/pick_result.json
```
Same tape reader/IC-restore (`relabel_reward.tape_stream`/`reset_to_tape_ic`), same sim-variant
wiring, but `full_env.FullTaskEnv(..., scope='pick', ...)` built directly (`relabel_reward.py`'s
own `build_env()` hardcodes `scope='full'`, so there is no existing D5 entry point for
`scope='pick'` — this is the one piece of new code this lane wrote, mirroring
`relabel_reward.build_env()`'s kwargs line for line).

| tape | offline `tstar-1` | live `fired_at` | end (this run) |
|---|---|---|---|
| genesis-100000-013-256.npz | 70 | **70** | terminated (picked) @71 |
| genesis-101008-029-392.npz | 146 | **146** | terminated (picked) @147 |
| genesis-104000-044-196.npz | 52 | **52** | terminated (picked) @53 |
| genesis-106001-059-601.npz | 479 | **479** | terminated (picked) @480 |
| genesis-107008-073-601.npz | 467 | **467** | terminated (picked) @468 |

**5/5 exact match**, and the episode terminates immediately on the grant (scope='pick' pays
+1 and ends there — `full_env.py:1450-1476`, see §1.2). World build 147.4 s cold. Per-tape
wall time is the decisions to reach the pick only (6–44 s), far less than the full-tape
scope='full' run above, because `scope='pick'` stops the instant `picked` fires.

### (c) Census cross-check (Lane 5, `can_pos_recovery/videos_ladder_2026-09-11/census_human.json`)

Lane 5's census is independently generated (`baselines/annotate_demos.py census`, which
**imports** `tape_layout`/`tape_stream`/`reset_to_tape_ic`/`scalar`/`set_meta` from
`relabel_reward.py` rather than reimplementing them — `annotate_demos.py:89`) and runs
`scope='full'`:

| tape | census `grants.picked` |
|---|---|
| genesis-100000-013-256.npz | 70 |
| genesis-101008-029-392.npz | 146 |
| genesis-104000-044-196.npz | 52 |
| genesis-106001-059-601.npz | 479 |
| genesis-107008-073-601.npz | 467 |

Identical to both re-executions above. **(a) tape's own cut, (b) this lane's fresh `scope='full'`
and `scope='pick'` re-executions, and (c) Lane 5's census now all agree exactly, on every one of
the 5 tapes, once the row/decision indexing is aligned.**

### 1.2 Why `scope='pick'` fires wherever `scope='full'` does (code, not just measurement)

The `picked` flag is **not scope-aware at all** — it is computed once, unconditionally, inside
`GenesisCanEnv.step()` (`baselines/genesis_can_env.py:276-284`):
```
if bp[2] > w['pick_z'] and grip * 100.0 > GP_CLOSE and float(np.linalg.norm(ee - bp)) < PICK_EEF_DIST:
    ...
if self._pick_run >= PICK_SUSTAIN: self._picked = True
```
(`PICK_EEF_DIST=0.20`, `PICK_SUSTAIN=10`, `genesis_can_env.py:59-60`). `GenesisCanEnv` has no
`scope` attribute; `FullTaskEnv` is the only place `scope` exists. In `_step_once`
(`baselines/rl/full_env.py:1358-1476`):
- `scope='full'`'s EXTRA predicates (`placed_v2`, the StageTracker, `contact_push`,
  `slide_success`, …) are computed in `_full_scope_predicates`, called only
  `if self.scope == 'full':` (`full_env.py:1388-1389`) — none of that touches `picked`.
- `scope='pick'` reads the SAME `info['picked']` genv already set and terminates on it directly
  (`full_env.py:1472-1476`):
  ```
  terminated = bool(info.get('picked'))
  if terminated:
      truncated = False
      return (obs['state'].astype(np.float32), reward, True, False, info)
  ```
- The reward for it is gated by `LadderAccountant.pay_stages_for('pick')` →
  `frozenset({'picked'})` (`full_env.py:368-369`), so `scope='pick'` pays exactly +1 on this
  flag and nothing else, matching the ladder's own `picked` rung value.
- `pick_z`, `max_steps`: neither is scope-differentiated for the `picked` computation itself —
  `pick_z` is a world constant genv reads once (`self.pick_z = float(self.genv.w['pick_z'])`,
  `full_env.py:924`/`1637`); `max_steps` is an outer horizon (`full_env.py:854-856`) both checks
  here passed explicitly (`10**9` / `2400`), so it never bound either run.

So there is no "hardened pick differs by scope" branch to find, because there isn't one —
confirmed both by reading the code and by the 5/5 + 5/5 measurement above. Candidate (b) from
the task brief is **ruled out**.

---

## 2. Diff against DV3-1's check; reproduce, then show the fix

### 2.1 Where it differs

DV3-1's script: `scratchpad/verify_pick_cut.py` (still on disk this session, unmodified,
re-run below). The single load-bearing difference:

```python
# verify_pick_cut.py:30-32 (DV3-1)
env = full_env.FullTaskEnv(backend="cpu", max_steps=10**9, scope="pick",
                            action_mode="delta_joint", delta_cap=0.025,
                            delta_leash_mult=5.0, action_repeat=4)
```
— constructs `FullTaskEnv` **directly**, never calling `sim_variant_hook.apply_pre()` before
it or `apply_post()` after, and never setting `GENESIS_SIM_VARIANT`. Compare
`relabel_reward.build_env()` (`relabel_reward.py:321-348`), which every other re-execution
path in this repo (D5 relabel, D5 records, `annotate_demos.py`) goes through:
```python
# relabel_reward.py:326-339
os.environ['GENESIS_SIM_VARIANT'] = sim_variant
from sim_variant_hook import apply_pre, apply_post
from full_env import FullTaskEnv, refuse_legacy_gates
refuse_legacy_gates()
apply_pre(sim_variant)                       # <-- MUST run before FullTaskEnv() -- monkeypatches
                                              #     gs.Scene.add_entity, which the world build calls
env = FullTaskEnv(backend='cpu', max_steps=int(max_sim_steps), scope='full', ladder=ladder, ...)
apply_post(env, sim_variant)                 # <-- sets arm kp/kv AFTER the world exists
```
`dHfull_all/repeat.json` stamps `sim_variant=gc_kp4_riser3_shelf6`
(`sim_variants.py:67`: `dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0, effort='base', riser=0.03,
shelf_dz=0.06)`). What `apply_pre`/`apply_post` actually do to the world
(`sim_variants.py:107-153` install, `:156-194` post_build), and what DV3-1's env therefore
never got:
- **Arm joint kp ×4, kv ×2** — `post_build` sets `kp = BASE_KP * 4.0 = [800,800,600,400,240,240]`,
  `kv = BASE_KV * 2.0`; without it the arm runs at `replay_harness.build_world()`'s own default,
  `[200,200,150,100,60,60]`/`[20,20,15,10,6,6]` (`can_pos_recovery/replay_harness.py:172-174`) —
  a quarter of the commanded joint stiffness the tapes were driven with.
- **No gravity compensation** — `install()`'s monkeypatched `add_entity` attaches
  `gs.materials.Rigid(gravity_compensation=1.0)` to the Kinova URDF only when the variant is
  installed (`sim_variants.py:125-127`); without it the arm sags against its own weight under
  the (now also 4x-weaker) position controller.
- **No 3 cm riser** — `install()` adds `riser=0.03` m to the robot mount's Z position
  (`sim_variants.py:144-150`); without it the whole arm sits 3 cm lower than the world the demos
  were recorded and cut in.

None of this shows up as a reset-time error (`can/goal pose match to sub-mm` — DV3-1's own
number, reproduced below unchanged): `reset_to()`/`genv.reset()` sets the can, the goal and the
arm's *starting* joint angles directly regardless of variant. It only shows up once the arm
starts moving under a materially softer, unsupported, lower-mounted controller than the one the
recorded delta-joint commands were tuned against — which is exactly consistent with a **total**
0/5 failure (every tape, regardless of length) rather than a one- or two-decision timing shift.

Two secondary differences were checked and are **not** the cause:
- `verify_pick_cut.py:49-50` sets `goal_pos` from the tape's own recorded `goal_xy` rather than
  `replay_harness.STATIC_BOTTLE_POSITION` (what `relabel_reward.reset_to_tape_ic()` uses,
  `relabel_reward.py:402-407`). The goal object never enters the `picked` predicate
  (`bp`/`ee` in `genesis_can_env.py:278` are the picked-can and end-effector only) — ruled out
  by inspection, and moot in practice since the tape's recorded goal is close to the static one.
- `verify_pick_cut.py` never calls `env.genv._calib_tool_offset()` after `reset_to()`
  (`relabel_reward.reset_to_tape_ic()` does, `relabel_reward.py:409`); `GenesisCanEnv.reset()`
  already calibrates it on the world's first-ever reset when `self._tool_offset is None`
  (`genesis_can_env.py:217-218`), and every reset re-poses the arm to the same
  `HARDCODED_START` regardless of variant (`genesis_can_env.py:173-186`), so the cached offset
  is valid throughout either script. Confirmed by the pick-scope corrected run above still
  matching to 5/5 without ever calling this a second time — this candidate is also ruled out.

### 2.2 Reproduce the original 0/5, then show the fix passes

Re-ran DV3-1's **unmodified** `verify_pick_cut.py`:
```
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
  ~/workspace/genesis_sim2real/venv/bin/python <scratchpad>/verify_pick_cut.py
```
```
[verify] genesis-100000-013-256.npz: offline_tstar=71 live_fired_at=None reward=None arm_q_err=0.0008897 goal_err=0 can_pose_reset_err=5.52e-04
[verify] genesis-101008-029-392.npz: offline_tstar=147 live_fired_at=None reward=None arm_q_err=0.0008887 goal_err=0 can_pose_reset_err=5.52e-04
[verify] genesis-104000-044-196.npz: offline_tstar=53 live_fired_at=None reward=None arm_q_err=0.0008883 goal_err=0 can_pose_reset_err=5.52e-04
[verify] genesis-106001-059-601.npz: offline_tstar=480 live_fired_at=None reward=None arm_q_err=0.0008886 goal_err=0 can_pose_reset_err=5.52e-04
[verify] genesis-107008-073-601.npz: offline_tstar=468 live_fired_at=None reward=None arm_q_err=0.0008887 goal_err=0 can_pose_reset_err=5.52e-04
[verify] SUMMARY: 0/5 tapes match exactly
```
**Reproduced exactly** — same 0/5, same reset-error numbers to 4 significant figures as
DV3-1's doc (`8.9e-4` rad arm, `5.52e-4` m can). Not a fluke, not machine-specific drift: it is
this script's missing sim-variant wiring, deterministically.

The **corrected** check is §1's `scope='pick'` run above (same tapes, same `tape_stream`/
`reset_to_tape_ic`, `apply_pre`/`apply_post` added, otherwise identical settings) — **5/5
pass**, reproduced with the numbers already shown in §1.

---

## 3. Verdict on the demo set

**Yes — the tape's own cut point (first `reward>0` row, terminal reward 1.0) is where
`FullTaskEnv(scope='pick')` pays, for the overwhelming majority of tapes, and the running
1M-step demo buffer does not need to be rebuilt.** Per-tape offset (decisions,
`fired_at − (tstar−1)`) for the 5 sampled tapes: **0, 0, 0, 0, 0** (§1).

### All 74 tapes (cheap: 301 s wall, 8 processes)

```
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
  ~/workspace/genesis_sim2real/venv/bin/python <scratchpad>/dv3-4/d5_check_all74.py \
  --procs 8 --out <scratchpad>/dv3-4/all74_result.json
```
`scope='pick'`, same wiring as §1, re-executes all 74 `dHfull_all` tapes (8 worker processes,
one Genesis world each) and compares `fired_at` against `dHfull_pick_local`'s own
`cuts[tape]['tstar'] - 1`. **Launched 15:47:12, finished 15:52:13 — 301 s wall clock.**

```
n_tapes=74  n_pick_tapes=64  n_nopick_tapes=10
n_exact_match=63 / 64      n_fired_none_on_pick_tape=0
offsets over the 64 pick tapes: min 0, max 1, mean 0.016
```

**63/64 (98.4%) exact**, offset 0. **1/64** (`genesis-100009-022-601.npz`) is off by **+1
decision**: the tape's stamped reward puts the pick grant at raw row 178 (decision 177), but
this run's live `picked` (and, independently, Lane 5's census `grants.picked`, which agrees at
178 exactly) fires one decision later, at 178. The cut set truncates this one tape one decision
before the live env would — a single-decision early truncation on 1/74 tapes, i.e. it drops
the very last pre-pick approach frame and pays/terminates the demo one step early. Consistent
with this project's documented precedent that replay of the chaotic contact dynamics is not
bit-exact frame-for-frame (`relabel_reward.py`'s own `can_dev_max_m`/`joint_dev_max_rad`
fidelity columns exist for exactly this reason) — negligible in isolation.

**One genuine, narrow defect, independently confirmed:** `genesis-102006-006-296.npz` is one of
the ten tapes `dHfull_pick_local` calls **no-pick** (its stamped reward column is all zero, so
the tape was kept whole, unchanged, as a 295-decision negative example). This run's live
`scope='pick'` re-execution instead fires `picked` at **decision 165**, terminating there —
and Lane 5's independently-built census (`scope='full'`, a completely separate re-execution)
**agrees exactly**: `grants.picked=165`, going on to `end_reason=tipped` at decision 295. Both
live re-executions, on two different code paths, say this tape DOES pick; the tape's own
stamped reward (computed whenever `dHfull_all` was built, evidently against an earlier state of
this predicate) says it never did. **This is a demo-set defect, not a re-execution artifact**:
one of the 74 pick-scope training tapes is currently mislabeled as a 0-reward negative when it
should be a 165-decision positive (terminal reward 1.0) — the live env would grant and end the
episode there, so `scope='pick'` training never gets to see genesis-102006's approach-and-grasp
as a positive example. All ten "no-pick" tapes' live end behaviour:

| tape | live `fired_at` | end (this run) |
|---|---|---|
| genesis-101005-003-601.npz | None | truncated @600 |
| genesis-101006-004-601.npz | None | truncated @600 |
| genesis-102000-005-2.npz | None | terminated @1 (1-decision stub) |
| **genesis-102006-006-296.npz** | **165** | **terminated (picked) @166** |
| genesis-103004-007-202.npz | None | stream_exhausted @201 |
| genesis-103005-008-601.npz | None | truncated @600 |
| genesis-104005-009-601.npz | None | truncated @600 |
| genesis-105005-010-601.npz | None | truncated @600 |
| genesis-105007-011-2.npz | None | terminated @1 (1-decision stub) |
| genesis-106007-012-601.npz | None | truncated @600 |

Only this one of the ten disagrees; the other nine (including the two 1-decision stub tapes)
are confirmed genuine no-picks under the live predicate too.

### Is the running 1M run's demo buffer sound?

**Yes, sound enough to keep running, with one disclosed, low-impact defect.** 63/64 pick cuts
are exact; the 64th is off by one decision (negligible). One of the ten no-pick tapes
(`genesis-102006-006-296.npz`, 1/74 = 1.4% of the set) is misclassified — it should contribute
a 166-decision positive pick demonstration and currently contributes a 295-decision all-zero
negative one instead. This subtracts one real positive demo from a 64-positive set and replaces
it with an uninformative negative; it does not corrupt any of the other 73 tapes, does not
change any action stream, and does not indicate the cut *rule* is wrong — the rule (first
`reward>0` row, verified exactly against a live re-execution on 63/64 tapes) is right; only its
input (the tape's own stamped reward column) is stale on this one tape.

**Exactly what to rebuild, if/when this is worth fixing:** do not hand-patch this one tape.
Re-cut `dHfull_pick_local` from a **live** `scope='pick'` (or equivalently `scope='full'`)
re-execution instead of the source tape's stamped `reward` column — i.e. apply the same
methodology `relabel_reward.py` (D5) already uses for `scope='full'` re-labelling to build the
pick cut, rather than trusting `to_dreamer_native.py --reward-from-tape`'s stamp. Concretely:
extend `scratchpad/d5_check_all74.py` (or a permanent `baselines/rl/` script built the same
way) to write out a truncated tape at `fired_at` for every tape where the live `picked` fires,
in place of `build_pick_from_full.py`'s `nz = np.nonzero(d['reward'] > 0)[0]` cut. This would
be a **74-tape, one-world, ≤5-minute rebuild** (this run's own cost is the proof) and would
promote `genesis-102006-006-296.npz` from a 295-decision negative to a 166-decision positive,
and shift `genesis-100009-022-601.npz`'s terminal by one decision. Not urgent enough to
interrupt the current run; worth doing before this pick-scope demo set is reused for a future
run or cited in a results table.

---

## Commands index (all read-only against the repo; all Genesis backend='cpu')

| step | script | wall |
|---|---|---|
| scope='full', 5 tapes | `<scratchpad>/d5_check.py --scope full` | ~2.5 min (warm) |
| scope='pick', 5 tapes | `<scratchpad>/d5_check.py --scope pick` | ~4.7 min (cold build) |
| reproduce DV3-1's 0/5 | `<scratchpad>/verify_pick_cut.py` (unmodified) | ~4.9 min (cold build) |
| all 74, scope='pick', 8 procs | `<scratchpad>/dv3-4/d5_check_all74.py --procs 8` | 301 s (5 min 1 s) |

`<scratchpad>` = `/tmp/claude-1000/-home-j-workspace-genesis-pickaplace/14e7a131-c656-4890-a0e7-bcea8665917d/scratchpad`
(session-local; `verify_pick_cut.py`/`build_pick_from_full.py` are lane DV3-1's own files,
left as found; `d5_check.py`/`dv3-4/d5_check_all74.py` are this lane's). Raw JSON results:
`<scratchpad>/dv3-4/{pick_result,full_result,all74_result}.json`, logs alongside as `*.log`.
