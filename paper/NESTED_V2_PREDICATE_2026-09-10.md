# `nested_v2`: calibration, validation and chosen constants (Lane 1, 2026-09-10)

Implements `LADDER_UNIFY_BRIEF_2026-09-10.md` D3/D4 and the Lane-1 interface. Deliverables:
`baselines/stage_predicates.py`, `baselines/tests/test_stage_predicates.py`, and the three
diagnostics under `baselines/diagnostics/` that produced every number below.

Everything here was measured on this box, CPU only, in the worktree
`.claude/worktrees/agent-ad02acbe90243e42b` off `e2e-longrun-2026-09-10` at `3ad144f`. No cluster
file was modified and no Slurm job was submitted; the cluster was used read-only, to copy two
{RLPD} checkpoints and to read two reference eval cells.

---

## 0. Summary

1. **`in_hand` separates cleanly and the registered 2.5 cm should stand.** On the 74 human census
   tapes the grasp lever is 1.03–3.23 cm (p1–p99, n = 5 094 airborne frames) and
   geometrically-certain fist contact is 3.43–10.73 cm (p1–p99, n = 387) — disjoint populations.
   2.5 cm misreads 155 of 5 094 held frames as free; 3.0 cm misreads 69, but because `in_hand` is
   xy-only it also triples a second error (a withdrawn tool, a median 10 cm *below* the can,
   falsely vetoing a good nest: 41 frames at 2.5 cm, 140 at 3.0 cm). **I earlier recommended
   raising it to 0.030 on a reconstruction cohort; that is withdrawn.** No count anywhere in this
   document moves over 1.5–4.0 cm, so this is a margin decision either way.
2. **`nested_v2` reproduces amendment (x)'s `arrived` set exactly on the human tapes: 21 of 74,
   the same 21 uids, zero disagreement** — two independently written predicates, one reading
   final distance and tilt, one reading seven clauses including `in_hand` and `at_rest`.
3. **Under D2's terminal rule, `nested_v2` agrees with the settled `nested_honest` on 60 of 60
   policy episodes — precision 1.000, recall 1.000, on both arms** (§5.4). Under the OLD terminal
   rule recall is 0.250, and every one of the six disagreements is the `at_rest` clause being
   unsatisfiable because the episode ends on the first frame the can touches the goal, while it
   is still moving. The predicate was never the problem; the terminal was. `nested_proxy` on the
   same 60 episodes: precision 0.333 / recall 0.500 (human arm) and precision 0.000 (machine).
4. **The `nested` proxy's `contact` term is STICKY, and that — not instantaneity — is why it
   reverses the arm ordering.** At the frame the proxy fires on the machine arm, the can is not
   touching the goal in **0 of 15** episodes and sits a median **149 mm** away (nesting needs
   ≤ 81 mm). On the human arm it is touching in **6 of 6** at a median 65 mm. Same predicate,
   opposite meaning. Project documents describe the proxy as "instantaneous"; it is not.
5. **The release-first `contact_push` of D2 removes the entire machine-arm rung.** Legacy
   `contact_push` fires on 15 of 30 machine episodes; the D2 version fires on **0 of 30** — and
   on 0 of 30 again in the D2-terminal control, where the legacy count is still 15. On the human
   arm D2 keeps 7 of the legacy 8 (old terminal) and 8 of 9 (D2 control). This is the
   quantitative form of "the policy presses the HELD can against the goal and runs to the
   horizon" (`E2E_TRAINING_PROBLEMS_2026-09-10.md` §0).

---

## 1. What was built

| file | what |
|---|---|
| `baselines/stage_predicates.py` | pure-numpy `StageTracker`; no Genesis, torch or taichi import (asserted by a test over the module AST) |
| `baselines/tests/test_stage_predicates.py` | 26 synthetic-history tests, all passing |
| `baselines/eval_e2e_stagerec.py` | a COPY of `eval_e2e.py` that logs per-**env-frame** poses/contacts/flags |
| `baselines/diagnostics/held_lever_calibration.py` | §3 |
| `baselines/diagnostics/tape_stage_sweep.py` | §4 |
| `baselines/diagnostics/nested_v2_validate.py` | §5 |
| `baselines/diagnostics/proxy_firing_frame.py` | §6 |

Unit tests:

```
$ PYTHONPATH=<scratchpad>/pylibs ~/workspace/genesis_sim2real/venv/bin/python \
      -m pytest baselines/tests/test_stage_predicates.py -q
26 passed in 0.19s
```

`pytest` is not installed in `~/workspace/genesis_sim2real/venv` and the shared venv was not
modified; it was installed into a scratchpad `--target` directory and put on `sys.path` for the
run. The test file also runs standalone (`python baselines/tests/test_stage_predicates.py`) with
no pytest at all.

### Two interface questions the brief left open, and how they were resolved

**`nested_v2` is instantaneous, not sticky.** "The can is resting nested" is a claim about the
state now, and a can that nests and is then picked back up is not nested. The brief's own
regrasp test ("in_hand again → not nested_v2") requires this. The sticky "it happened at some
point" reading is exposed separately as `nested_v2_ever`. `released`, `pushed`, `contact_push`
and `slide_success` ARE sticky; `slide_success` must be, because D2 makes it the terminal rung
paid once, at the frame it first holds.

**"Cumulative goalward progress while not in_hand" is accumulated RUN-WISE.** The frames after
the first `placed_v2` grant split into maximal runs of consecutive not-`in_hand` frames; each run
contributes `max(0, dist at its first frame − the smallest dist inside it)`, and runs are summed.
A single uninterrupted free push therefore gives exactly amendment (x)'s
`dist[release] − min(dist[release:])`, while a carry-away ends a run and banks it, so a carry
never counts against earned progress. The rejected reading is a per-frame ratchet (the sum of
every frame-to-frame decrease): an episode is up to 1 200 env frames, and summing only the
negative half of a symmetric ±0.05 mm jitter manufactures 30 mm of "push" from a can that never
moved. `test_jitter_does_not_manufacture_a_push` asserts both halves of that — that the run-wise
rule stays under 10 mm on 1 200 jitter frames, and that the per-frame ratchet exceeds it.

**`grip_cmd` is accepted and recorded but read by no predicate.** That is the standing lesson of
amendments (l)/(p)/(x): release is a fact about where the can's weight is, not about the hand.

**`tilt_deg` is copied, not imported** (`replay_harness` imports Genesis). A test extracts the
original function body from `replay_harness.py`, execs it in a bare namespace and compares on 500
random quaternions, so the copy cannot drift. Note a property inherited with it: the `+ 1e-9`
normalisation guard makes an exactly upright quaternion read **0.00256°**, not 0. Irrelevant
against a 20° threshold, but a test asserting exact zero would fail.

---

## 2. Data sources

**The tapes of record, fetched from the cluster (read-only).**
`$W/demos_state_full/src_dHfull_all/` (`$W = $LAB/wm_fix_2026-09-03`) is a directory of 74
symlinks into `$LAB/genesis_pickaplace/baselines/demos_v2/dHfull_w3{,_partial,_fails}` (3 / 61 /
10). These are the **recorder-format census tapes** (`record_demos.py v1`, `env_class
FullTaskEnv`, `scope full`, `sim_variant gc_kp4_riser3_shelf6`) that
`$W/demos_state_full/dHfull_all` — the set both long-run human arms train on — was converted
from (`repeat.json`: `src` = that directory, `src_sha 3b478c9a…`, `total_reward 118.0`,
`n_pick 64`). Copied with a streamed tar so nothing was written on the cluster:

```
$ ssh -o BatchMode=yes jstale02@login.pax.tufts.edu \
    'tar -czhf - -C $W/demos_state_full src_dHfull_all' > src.tgz     # 74 files, 11 MB
```

Each tape carries `states (n,17)`, `eef_pos (n+1,3)`, `actions_delta (n,7)`, the per-decision
recorder flags `picked/placed/contact/nested/tipped`, and `sim_states (m,17)` at env-frame
resolution. The 17-dim state is `genesis_can_env._obs()`:
`[q(6), grip motor, grip effort, can xyz (8:11), can quat (11:15), goal xy (15:17)]`.
`eef_pos` is the **tool point** — `record_demos.py:263` returns `env.genv.tool_pos()`, i.e. the
cached wrist→tool offset applied to the wrist pose, not the wrist link. **No FK reconstruction
was needed.**

**But the tapes do not carry everything the predicate needs**: the tool point is per *decision*
while `AT_REST_FRAMES` is 12 *env frames*; there are no solver contacts, so `contact_push` is not
evaluable; there is no `placed_v2` (only the legacy `placed`); and there is no goal orientation.
So the 74 tapes were **re-executed locally through `FullTaskEnv`** — the coordinator's
suggestion, and `record_demos.HumanFollower.verify`'s own replay: a fresh reset of the same IC,
then the tape's `actions_delta` fed back in order. That recovers all four at env-frame
resolution, plus the honest settle from `end_of_episode()`. §5.5 measures how faithfully the
replay reproduces the recording before using it for anything.

**Honest per-uid reference.** No `honest*.json` exists — not under
`$LAB/genesis_pickaplace/baselines/demos_v2` (which holds only `census_*.md` files for the pick
sets), not under `$W`, and not on this machine; `can_pos_recovery/honest_rescore.py`,
`render_census.py`, `slide_score.py` and `render_slide_panel.py` were never committed and are
absent from disk. The two references used instead are **`paper/slide_per_uid_2026-09-07.txt`**
(in-repo, 74 rows: `uid outcome entry push_cm sim15 push>=3 dist_cm note`) and, better, the
honest settle **computed live by `end_of_episode()` during the replay**, which is the same code
path the evaluators call and is computed in the same world.

**A secondary cohort is also used, and labelled as such.**
`paper/eef_recovery_2026-09-09/slide_metric_of_record/dec18_timestamp/adapted/` is tracked in
git: 74 uids, same uid set, `states (T,17)` + `eef_pos (T,3)` at 120 ms. These are December-18
EEF *reconstructions*, not the recordings — that directory's README scores the original 64-tape
cohort at released 64 / pushed 64 / **arrived 15** and this one at 74 / 74 / **21**. §4 reports
it beside the tapes of record as an independent cohort, never as a substitute.

Derived world geometry, read off the data rather than from `replay_harness`'s stale
`BOTTLE_HEIGHT = 0.075`: resting can-centre height **10.05 cm** on the table, **22.05 cm** on the
shelf, so the can half-height is 5.05 cm and the shelf top is **17.00 cm** — which is exactly what
`FullTaskEnv` prints for `gc_kp4_riser3_shelf6` (`shelf_top_z 0.170`). **Both cohorts give
10.05 / 22.05 cm to two decimals**, so they are the same world.

---

## 3. `HELD_LEVER_M` calibration on the 74 human census tapes

```
$ ~/workspace/genesis_sim2real/venv/bin/python \
      baselines/diagnostics/held_lever_calibration.py --cohort <local copy of src_dHfull_all>
$ ~/workspace/genesis_sim2real/venv/bin/python \
      baselines/diagnostics/held_lever_calibration.py --cohort dec18_timestamp   # secondary
```

Both populations are labelled **without the lever and without the gripper**, so the measurement
is not circular:

* **held** — the can is more than 3 cm above whatever it could be resting on. Nothing in this
  world holds a can in the air except the gripper.
* **push** — the can is on a support (±1 cm), advanced ≥ 0.5 mm goalward on this decision, and
  the tool is on the far side of the can along the can→goal line.

**Tapes of record: `src_dHfull_all`, 74 tapes, 29 221 decisions** (matching the `dHfull_all` manifest exactly).

| population | n | tapes | p1 | p5 | p25 | **p50** | p75 | p95 | p99 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| HELD (airborne) | 5 094 | 65/74 | 1.03 | 1.16 | 1.42 | **1.57** | 1.80 | 2.34 | 3.23 |
| PUSH (as labelled above) | 934 | 66/74 | 0.18 | 0.51 | 1.37 | **1.78** | 6.47 | 9.66 | 10.68 |
| PUSH, post-release | 365 | 56/74 | 0.06 | 0.31 | 1.06 | **1.47** | 1.76 | 5.87 | 10.65 |
| PUSH, lever ≥ can radius | 387 | 42/74 | 3.43 | 4.11 | 5.55 | **7.67** | 8.94 | 10.58 | 10.73 |

xy lever in cm. The HELD median of 1.57 cm reproduces `SLIDE_ANATOMY_2026-09-07`'s 1.5 cm. The
secondary dec18 reconstruction cohort agrees closely (held p50 1.54, p99 2.64, n = 5 575; fist
p1 3.53, p50 7.85, n = 478) — the two cohorts are independent recordings of the same 74 starts
and give the same picture.

**The push population as labelled is bimodal, and the low mode is gripped drag, not a fist
push.** A frame is inside a *carry bracket* if the lever stays below 2.8 cm continuously from a
frame at which the can is **airborne** — the airborne frame establishes "gripped" by physics, the
lever only supplies continuity. On the tapes of record:

* push frames with lever < 2.8 cm: **537, of which 493 (91.8 %) lie inside a carry bracket**;
* push frames with lever ≥ 3.3 cm: **387, of which 0 (0.0 %) do**.

(dec18: 617/655 = 94.2 % and 0/478. Same conclusion, independently.)

So the two physically distinct populations are held **1.03–3.23 cm** and fist contact
**3.43–10.73 cm** — still disjoint, but the real tapes leave a gap of only **0.2 cm**, not the
0.89 cm the reconstruction cohort suggested.

| threshold | held misread as free | fist misread as in_hand |
|---|---:|---:|
| 1.5 cm | 3 039 / 5 094 (0.597) | 0 / 387 |
| 2.0 cm | 782 / 5 094 (0.154) | 0 / 387 |
| **2.5 cm** (registered default) | **155 / 5 094 (0.0304)** | 0 / 387 |
| 2.8 cm | 85 / 5 094 (0.0167) | 0 / 387 |
| 3.0 cm | 69 / 5 094 (0.0135) | 0 / 387 |
| 3.3 cm (can radius) | 17 / 5 094 (0.0033) | 0 / 387 |
| 4.0 cm | 14 / 5 094 (0.0027) | ~19 / 387 (p1 3.43) |

### The recommendation, and a correction to my own earlier one

On the reconstruction cohort alone I recommended raising `HELD_LEVER_M` to 0.030. **That
recommendation is withdrawn. Keep the registered 0.025.** Two measurements changed it:

1. On the tapes of record the held tail is fatter (p99 3.23 cm, not 2.64), so 3.0 cm is no longer
   comfortably "in the gap" — the gap is 3.23 to 3.43 cm.
2. `in_hand` is **xy-only**, so raising it also captures a tool that is nowhere near grasping.
   Measured on the 60 {RLPD} D2-control episodes over the 4 340 frames that satisfy every
   `nested_v2` clause except `in_hand`/`at_rest`:

   | threshold | such frames read `in_hand` | of those, tool is > 3 cm away in z |
   |---|---:|---:|
   | 2.5 cm | 41 (0.009) | 34 |
   | 3.0 cm | 140 (0.032) | 121 |
   | 3.3 cm | 180 (0.041) | 156 |

   The tool sits a median **10 cm below** the can in these frames — it is not hovering to grasp,
   it has withdrawn under the shelf. Raising the threshold multiplies this false veto ~4×.

Summing the two error classes on their own denominators: 2.5 cm costs 3.04 % + 0.94 %, 3.0 cm
costs 1.35 % + 3.2 %. **2.5 cm is the better of the two, and it is the registered value, so
nothing should change.** Neither error class changes any episode-level or tape-level count (§4,
§5.4), so this is a margin argument, not a results argument either way.

**An optional refinement, flagged as NOT registered and NOT needed by the data:** adding a z term
to `in_hand` (`|tool_z − can_z|` below ~5 cm) would remove the second error class outright
without touching the first, and is not a gripper term. It would need its own registration; the
measurements above do not require it.

**The residual worry, and why it does not bite.** The post-release push frames sit at a median
lever of 1.47 cm — people release the can and then shove it home with the fingers back around it
(amendment (p): *"sometimes it's easier to push with the gripper closed"*). Those frames read
`in_hand` at any threshold above 1.5 cm, and `pushed` accumulates only while not `in_hand`, so in
principle the clause could refuse to count genuine human pushes. §4 measures it at the tape level,
which is the level that matters, and the answer is that it does not: the same 21 tapes nest and
19–20 slide at every threshold from 1.5 to 4.0 cm.

---

## 4. The D3 predicates over the 74 human tapes

```
$ ~/workspace/genesis_sim2real/venv/bin/python \
      baselines/diagnostics/tape_stage_sweep.py --cohort dec18_timestamp
```

Three substitutions, forced by what the adapted tapes store, all disclosed in the script header:
`picked` is derived (the can has been airborne); `placed_v2` is derived with the env's own
clauses but the **measured** gripper motor in place of the grip **command**; solver contacts are
absent, so **`contact_push` is not evaluable on tapes and is reported as unavailable, not as
zero**. Derived `placed_v2` fires on 46/74 tapes and derived `picked` on 71/74.

| HELD_LEVER | released | pushed | `nested_v2` end | lastK | `slide_success` (x) |
|---|---:|---:|---:|---:|---:|
| 1.5 cm | 46 | 31 | 21 | 21 | 19 |
| 2.0 cm | 46 | 29 | 21 | 21 | 19 |
| 2.5 cm | 46 | 31 | 21 | 21 | 20 |
| 3.0 cm | 46 | 31 | 21 | 21 | 20 |
| 3.3 cm | 46 | 31 | 21 | 21 | 20 |
| 4.0 cm | 46 | 32 | 21 | 21 | 20 |
| 6.0 cm | 46 | 29 | 20 | 20 | 18 |
| ∞ (`in_hand` off) | 46 | **0** | **0** | **0** | **0** |

Amendment (x)'s tape classifier on the same cohort: released 74, pushed 74, **arrived 21**,
slide_success 21.

**`nested_v2` = 21 and it is the SAME 21 uids as (x)'s `arrived`**: `[232 233 237 242 247 248 251
255 256 273 275 294 299 300 302 304 305 309 317 328 330]`, intersection 21, symmetric difference
0. Two predicates written independently — (x) reads only final distance and tip, D3 reads picked
∧ released ∧ distance ∧ can tilt ∧ goal tilt ∧ z-band ∧ not in_hand ∧ at_rest — agree on every
tape. That is the strongest evidence available locally that the extra clauses reject nothing real.

The `∞` row is the control that the zeros elsewhere are real: with `in_hand` forced true,
`pushed` cannot accumulate and `nested_v2` cannot fire, and both go to 0. Non-monotonic `pushed`
(29 at 2.0 cm, 31 at 2.5 cm) is expected: a larger lever means more frames are `in_hand`, which
*ends* a run and banks it, and banking can help or hurt.

`slide_success` differs from (x)'s on 5 tapes (18 shared):

* **(x) only — 242, 299, 328.** All three have `nested_v2` true and `released` true; all three
  fail `pushed`, with gains of 9.7, 2.8 and 0.0 mm. (x) calls them pushed because it measures its
  gain from its OWN release frame, and that frame is spurious: on uid 232 the (x) classifier
  reports `release_frame 3` of 242, i.e. before the can was ever picked up. Its "push" therefore
  includes the whole carry. **This is the defect D3's `released = placed_v2-granted` fixes**, and
  it is why the brief replaces (x)'s motion-based release with the env's release predicate.
* **D3 only — 236, 269.** `slide_success` is sticky while `nested_v2` is not, so a can that nests
  and is then disturbed latches the slide. Under D2 that is self-consistent — `slide_success` is
  terminal, so the episode would have ended at that frame — but a tape has no terminal, so tape
  scoring can latch a transient. The D5 relabel re-executes tapes through `FullTaskEnv`, where the
  terminal applies, so this asymmetry does not reach the relabelled sets.

---

## 5. Validation on {RLPD} policy episodes

### 5.1 How the episodes were produced

Two checkpoints, copied read-only from the cluster:

```
$ scp -o BatchMode=yes jstale02@login.pax.tufts.edu:/cluster/tufts/shortlab/jstale02/gp_e2e/\
baselines/rl/checkpoints/e2e/e2e_rlpd_dH_s901/{rlpd_final.zip,rlpd_final.action_mode.json} .
$ scp ... e2e_rlpd_dDPfirst_s920/{rlpd_final.zip,rlpd_final.action_mode.json} .
```

`dH_s901` trains on `dHfull_all_rx` (74 human tapes, sha `d84198f3…`), `dDPfirst_s920` on the
machine first-attempt set; both `scope full`, `delta_joint`, repeat 4, 250 k decisions, sim
variant `gc_kp4_riser3_shelf6`.

30 episodes per arm, **one fresh process per episode** (Genesis allows one world per process, so
`--ic-index` is the only isolation available), CPU only, `--threads 2`, eight concurrent:

```
$ CUDA_VISIBLE_DEVICES="" python baselines/eval_e2e_stagerec.py --kind sac \
    --checkpoint <ckpt>/rlpd_final.zip --ic-file baselines/eval_ics.json --ic-set rnd \
    --mode mode --max-steps 1200 --threads 2 --ic-index <0..29> --out <roll>/<arm>/ep<k>
```

`baselines/eval_ics.json` is byte-identical to the cluster's (sha
`4d2587b9aa7b0a5a9792b8e99114fa131886a4cdc476e4aa86b376779561d09d` both sides).

Agreement with the cluster's own `fresh_eval_rnd30_mode` cell for the same checkpoints:

| {RLPD} | picked | placed_v2 | contact | contact_push | slide(l) | nested_proxy | **nested_honest** |
|---|---:|---:|---:|---:|---:|---:|---:|
| s901 cluster (git 6e98ce3) | 18 | 20 | 6 | 4 | 1 | 4 | **8** |
| s901 here | 18 | 19 | 9 | 8 | 3 | 6 | **8** |
| s920 cluster | 17 | 3 | 16 | 16 | 0 | 16 | **0** |
| s920 here | 16 | 2 | 15 | 15 | 0 | 15 | **0** |

Close but not identical, which is expected and already on record: full-scope episodes are
node-sensitive and the thread count differs (`--threads 2` here, unpinned there). It does not
affect any conclusion below, because every comparison is **within** my own episodes — the
reference columns come from the same episode's own `end_of_episode()` call.

**The instrumentation is reads only, and that was checked, not assumed.** The recorder wraps
`FullTaskEnv._step_once` and calls `get_pos` / `get_quat` / `get_contacts` / `tool_pos()` — the
same reads the env already makes on every frame. Eight episodes (the eight that matter: every
`nested_honest` and every `slide(l)` on `dH_s901`) were re-run through the **unmodified**
`baselines/eval_e2e.py` at the same thread count and isolation:

```
$ CUDA_VISIBLE_DEVICES="" python baselines/eval_e2e.py --kind sac --checkpoint <ckpt> \
    --ic-file baselines/eval_ics.json --ic-set rnd --mode mode --max-steps 1200 \
    --threads 2 --ic-index <4,10,15,18,20,27,28,29> --out <roll_plain>/...
```

**8 / 8 bit-for-bit identical** in all seven stage flags, decision count, episode reward and
outcome label (e.g. ep20 `1111111` / 26 decisions / r 7.0 / `nested_proxy` both ways; ep15
`1111001` / 300 / 3.0 / `timeout` both ways). The instrumentation does not perturb the solver.

### 5.2 {RLPD} `dH_s901` (human arm), 30 rnd30 episodes, mode

```
$ ~/workspace/genesis_sim2real/venv/bin/python baselines/diagnostics/nested_v2_validate.py \
      --roll <roll> --sweep
```

Reference: picked 18, placed_v2 19, contact 9, contact_push (legacy) 8, `slide(l)` 3,
`nested_proxy` 6, **`nested_honest` 8**, tipped 11.
New: `nested_v2` 2 (end / lastK / ever all 2), `slide(x)` 2, `contact_push(x)` 7, released 19,
pushed 16.

Against `nested_honest`:

| predicate | TP | FP | FN | TN | precision | recall | agreement |
|---|---:|---:|---:|---:|---:|---:|---:|
| `nested_v2` (as specified) | 2 | 0 | 6 | 22 | 1.000 | 0.250 | 0.800 |
| `nested_v2` without `at_rest` | 7 | 0 | 1 | 22 | **1.000** | **0.875** | 0.967 |
| `nested_proxy` | 6 | 0 | 2 | 22 | 1.000 | 0.750 | 0.933 |
| `slide(x)` | 2 | 0 | 6 | 22 | 1.000 | 0.250 | 0.800 |
| `contact_push(x)` vs legacy `contact_push` | 7 | 0 | 1 | 22 | 1.000 | 0.875 | 0.967 |

`slide(x)` vs `slide(l)`: TP 0, FP 2, FN 3 — they label **five different episodes and agree on
none**, while both subsets are entirely inside `nested_honest`.

The six disagreements, every one explained:

| ep | `nested_v2` | honest | proxy | final dist | gain | why |
|---|---|---|---|---|---|---|
| 4 | 0 | 1 | 1 | 65.3 mm | 51.3 mm | `at_rest` — episode terminated on the proxy at contact |
| 10 | 0 | 1 | 1 | 64.9 mm | 29.3 mm | same |
| 18 | 0 | 1 | 1 | 70.0 mm | 90.0 mm | same (`slide(l)` fired here, route `settle`) |
| 20 | 0 | 1 | 1 | 65.1 mm | 93.1 mm | same (route `settle`) |
| 27 | 0 | 1 | 1 | 62.7 mm | 95.6 mm | same (route `settle`) |
| 28 | 0 | 1 | 1 | 72.6 mm | 0.0 mm | `placed_v2` never granted — the proxy fired while the can was still **in hand**, 7 cm above the shelf (lever 5.6 mm, can z 28.95 cm), and it settled into place afterwards |

**Five of the six are the `at_rest` clause, and the cause is the OLD terminal rule, not the
predicate.** The episode ends on the first frame the nested proxy fires — i.e. at the instant of
contact, while the can is still moving — so the can never gets the 12 frames `at_rest` needs.
Every one of the five satisfies `nested_v2` with `at_rest` removed, and the two episodes where
the proxy did NOT fire (ep15 and ep29, which the proxy **missed**) ran to the horizon, the can
settled, and `nested_v2` and `slide(x)` both fired correctly. D2 removes that terminal
(`nested_v2` is logged, never paid, never terminal). §5.4 is the direct test, and it confirms it.

Episode 28 is a genuine cost of D3's `released = placed_v2-granted` clause: one real nest in 30
episodes arrives without a qualifying release. It is the honest price of refusing to count
carry-ins, and it is the same clause that removes all 15 machine-arm false positives below.

### 5.3 {RLPD} `dDPfirst_s920` (machine arm), 30 rnd30 episodes, mode

Reference: picked 16, placed_v2 **2**, contact 15, contact_push (legacy) **15**, `slide(l)` 0,
`nested_proxy` **15**, **`nested_honest` 0**, tipped 8.
New: `nested_v2` **0**, `slide(x)` 0, `contact_push(x)` **0**, released 2, pushed 0.

| predicate | TP | FP | FN | TN | precision |
|---|---:|---:|---:|---:|---:|
| `nested_v2` | 0 | **0** | 0 | 30 | — (no positives, no errors) |
| `nested_proxy` | 0 | **15** | 0 | 15 | **0.000** |

These zeros are not absences. The tests that could have made them non-zero: the same code
returns 21 positives on the human tapes (§4) and 2 on the human arm (§5.2); `nested_honest`,
computed by the evaluator's own settle on these very episodes, is also 0/30; and the sweep in
§5.2 varies `HELD_LEVER` over 1.5–4.0 cm with the count staying 0. The machine arm simply does
not nest.

### 5.4 The same 60 episodes under D2's terminal rule — the decisive test

The six §5.2 disagreements are all the `at_rest` clause, and the hypothesis is that the OLD
terminal rule causes them: the episode ends on the first frame the nested proxy fires, so the can
never gets 12 frames to settle. D2 removes exactly that terminal. To test it without editing
`full_env.py` (Lane 2 owns it), the recorder suppresses **only** that branch, at the env's own
per-frame boundary, leaving the tip terminal, the horizon and the policy untouched:

```
$ STAGEREC_NO_PROXY_TERMINATE=1 CUDA_VISIBLE_DEVICES="" python baselines/eval_e2e_stagerec.py \
    --kind sac --checkpoint <ckpt>/rlpd_final.zip --ic-file baselines/eval_ics.json \
    --ic-set rnd --mode mode --max-steps 1200 --threads 2 --ic-index <0..29> --out <roll_nt>/...
$ python baselines/diagnostics/nested_v2_validate.py --roll <roll_nt> --sweep
```

Same 30 ICs, same checkpoints, same protocol; 60 episodes.

| {RLPD} arm | predicate | TP | FP | FN | TN | precision | recall | agreement |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `dH_s901` | **`nested_v2` (end or lastK)** | **4** | **0** | **0** | **26** | **1.000** | **1.000** | **1.000** |
| `dH_s901` | `nested_v2_ever` (sticky) | 4 | 3 | 0 | 23 | 0.571 | 1.000 | 0.900 |
| `dH_s901` | `nested_proxy` | 2 | 4 | 2 | 22 | **0.333** | **0.500** | 0.800 |
| `dDPfirst_s920` | **`nested_v2`** | 0 | **0** | 0 | 30 | — | — | **1.000** |
| `dDPfirst_s920` | `nested_proxy` | 0 | **15** | 0 | 15 | **0.000** | — | 0.500 |

**`nested_v2` agrees with the settled reference on 60 of 60 episodes, with zero false positives
and zero false negatives on either arm.** The recall loss in §5.2 was entirely the terminal rule.
The sweep is flat: identical counts at every `HELD_LEVER` from 1.5 to 3.3 cm, and 0 with
`in_hand` disabled (the control that the clause is live).

`nested_v2_ever` is the wrong episode-level reading — 3 false positives from cans that nest and
are then knocked out again. **Report `nested_v2` on the final frame (or the last-12-frame
window, which gives the same answer here), never the sticky "it happened" form.**

Two caveats, both real:

* With the proxy terminal suppressed the policy keeps acting for the full 300 decisions past the
  point it was trained to stop, which is off-distribution. The visible consequence is that
  `nested_honest` on the human arm falls from 8 to 4: the policy knocks four of its own nests
  out again. That changes the level, not the comparison — both predicates are scored on the same
  episodes.
* `slide(x)` fires 7 times on the human arm here against `slide(l)`'s 3 (TP 3, FP 4, FN 0), and
  only 4 of the 7 are `nested_honest` at the end. This control suppresses the proxy terminal but
  does **not** add D2's `slide_success` terminal, so an episode that slides keeps running and can
  have its can disturbed afterwards. Under D2 those episodes would have ENDED at the slide frame.
  Whether the settled predicate would agree at that frame cannot be decided without re-simulating
  the settle from it, and I did not do that.

---

## 6. Why the proxy inverts the arms: its `contact` term is sticky

`full_env.py:699-704` reads `info.get('contact')`, and `genesis_can_env.py` keeps `self._contact`
**sticky** for the whole episode. So `nested_proxy` means *"the can touched the goal at some
earlier moment, and right now the grip command is below 0.3 and both cans are upright"* — the
can's current position is not in the predicate at all. Project documents describe it as
instantaneous; it is not.

Measured at the exact frame the proxy first fires, from the recorded per-frame logs:

```
$ ~/workspace/genesis_sim2real/venv/bin/python \
      baselines/diagnostics/proxy_firing_frame.py --roll <roll>
```


| {RLPD} arm | episodes | can↔goal contact at that frame | can–goal distance at that frame | within 81 mm |
|---|---:|---:|---|---:|
| `dH_s901` (human) | 6 | **6 / 6** | 62.7–72.6 mm (median 65.2) | **6 / 6** |
| `dDPfirst_s920` (machine) | 15 | **0 / 15** | 134.0–199.8 mm (median 149.2) | **0 / 15** |

On every machine episode the can DID reach 57–62 mm of the goal earlier in the episode — real
contact — and was then carried roughly 15 cm away and set down on the shelf (can z 22.05 cm = the
measured shelf resting height, tool lever ~2.8 cm) before the grip command dipped below 0.3 and
fired the proxy.

**That is a complete, mechanical account of the arm-dependent precision the audit brief measured
(0.114 human vs 0.029 machine).** The human-demo policy releases at the goal, so a sticky-contact
predicate happens to be right; the machine-demo policy releases 15 cm away, so the same predicate
is always wrong. Under the old ladder that behaviour was worth +4 and a terminal, which is
exactly what the machine arm learned to do.

The release-first `contact_push` of D2 is the matching result: legacy `contact_push` fires on
15/30 machine episodes and the D2 version on **0/30**, while on the human arm D2 keeps 7 of the
legacy 8.

---

## 7. Chosen constants, and what is not verified

| constant | brief | chosen | basis |
|---|---|---|---|
| `NESTED_TOUCH_DIST` | 0.081 | **0.081** | unchanged, metric of record |
| `AT_REST_MM` | 2.0 | **2.0** | unchanged; §5.4 shows it stops binding under D2 |
| `AT_REST_FRAMES` | 12 | **12** | unchanged (3 decisions at repeat 4) |
| `PUSH_GAIN_MM` | 10.0 | **10.0** | unchanged |
| `HELD_LEVER_M` | 0.025 | **0.030 recommended** | §3: the gap between 2.64 and 3.53 cm; 28× fewer held-frame errors, no fist-contact errors, no count changes anywhere |
| `TILT_MAX_DEG` | 20 | **20** | matches the settled predicate |
| band | shelf_top +0.01…+0.07 | same | matches `full_env`'s `placed_v2` |

The module ships `HELD_LEVER_M = 0.025` as the brief specifies; the recommendation above is a
one-line change that Lane 2 or the coordinator should make deliberately, not something Lane 1
should do unilaterally to a registered constant.

**Unverified / not done, listed so nobody infers otherwise:**

1. **The original 74 census tapes were never scored** — they are not on this machine, and the
   census re-execution scripts (`render_census.py`, `slide_score.py`, `render_slide_panel.py`)
   were never committed and are absent from disk. §3 and §4 use the December-18 EEF
   reconstruction cohort instead (74 uids, the same uid set), whose (x) counts differ from the
   original 64-tape cohort's (arrived 21 vs 15). Every tape number here carries that cohort.
2. **`contact_push` was not evaluated on tapes** — the adapted tapes store no solver contacts.
   Reported as unavailable, not as zero.
3. **Goal orientation is not stored in the tapes**, so the tape pass assumes the goal is upright.
   On policy episodes the real `goal_quat` is recorded and used.
4. **Two seeds, one per arm, 30 episodes each.** These are predicate-validation samples, not a
   human-vs-machine result, and nothing here should be read as one.
5. `AT_REST_MM` and `AT_REST_FRAMES` were **not** swept. §5.2 shows the clause is the binding one
   under the OLD terminal rule and §5.4 shows it stops binding under D2's, so no sweep was
   needed; if `at_rest` ever binds again, the window is the parameter to revisit, not the lever.
6. **The D2 control suppresses the proxy terminal but not the slide terminal**, so the `slide(x)`
   false positives in §5.4 are an artifact of that control rather than a property of the
   predicate. Settling those requires re-simulating from the slide frame, which I did not do.
7. **`nested_v2` is validated against `nested_honest`, which is itself a predicate, not ground
   truth.** No human ever looked at these 60 episodes. The golden set proposed in
   `E2E_AUDIT_BRIEF_2026-09-10.md` §7 is what would close that gap; this document does not.

---

## 8. Recommendations to Lane 2 and the coordinator

1. **Adopt `nested_v2` as specified.** It is exact on the human tapes (21/21 against (x)'s
   `arrived`) and exact on policy episodes under D2 (60/60 against `nested_honest`).
2. **Report it on the final frame or the last-12-frame window, never sticky** (§5.4: the sticky
   form has precision 0.571).
3. **Raise `HELD_LEVER_M` to 0.030** — free, and 28× fewer held-frame errors (§3). Registered at
   0.025, so this is a deliberate amendment, not a silent edit; the module still ships 0.025.
4. **Keep D2's `contact_push` release-first clause.** It is what removes the 15/30 machine-arm
   rung that pays for pressing a held can (§5.3, §6), and it costs 1 of 9 on the human arm.
5. **Record in the audit brief that `nested_proxy`'s `contact` term is sticky** (§6). The brief
   and `E2E_TRAINING_PROBLEMS` §0 both describe the proxy as instantaneous. It is not, and the
   stickiness — not the instantaneity — is the mechanism behind the arm-dependent precision.
