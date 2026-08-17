# Action/state normalization + demo-compatibility audit — 2026-08-17

Auditor: Fable session (local box, CPU only; read-only on code and data, no
running job touched). Repo HEAD at audit time: `7441d49`.
Interpreter for every number below: `/home/j/workspace/genesis_pickaplace/.venv-eval/bin/python`.

Scope: end-to-end encode/decode consistency for every (demo set -> trainer) pair
in active use, with the train-side encoder and the env/eval-side decoder required
to be exact inverses under identical constants.

Sets audited: `baselines/episodes_pick_phase_all` (dH, 91 human eps),
`baselines/episodes_champion_pick` (dR2D, 66 model eps),
`baselines/m1all_harvest` (dDP, 93 model eps), `baselines/episodes_all`,
`baselines/episodes_pick_phase_dppruned`, `baselines/episodes_delta_rerecord`,
`~/workspace/dreamerv3-torch/demonstrations/{genesis,genesis_pick_pruned,
genesis_pick_pruned_delta,genesis_pick_pruned_delta25,genesis_pick_msr_delta25_r4}`.

---

## 0. Headline

**The train-side encoder and the env/eval-side decoder ARE exact inverses.** The
mechanism is clean: replaying `episodes_champion_pick` through the encoder and then
through a byte-faithful replica of `wandb_eval`'s integrator reconstructs the
recorded absolute command to **1.19e-07 rad max over 121 frames — one fp32 ULP**.

**The defect is on the data side, and it is asymmetric between the two arms of the
live n=8 cluster contrast.** The human tape's command trajectory is *not
representable* in the delta_joint action space (cap 0.025 / leash 0.125), so the
encoder emits a lossy projection: on **13.29 % of dH transitions** the labelled
action, executed from the recorded state, does not reproduce the recorded command
(mean error 0.0073 rad, p99 0.152 rad, max 1.267 rad). On **dR2D the same figure is
0.000 % — exactly zero, every frame**, because those tapes were generated *by* the
delta mechanism. The dR2D demo buffer is on-kernel by construction; the dH demo
buffer is not. See **C1**.

---

## 1. Ranked findings

### C1 — CRITICAL. The live RLPD n=8 contrast confounds demo SOURCE with action-label FIDELITY

`cluster/sbatch_rlpd.sh` runs both arms with identical flags and one variable, the
demo dir (lines 56-72):

```bash
case "$ARM" in
  dH)   DEMO=baselines/episodes_pick_phase_all;  PAT='^[0-9]{3}\.npz$' ;;
  dR2D) DEMO=baselines/episodes_champion_pick;   PAT='^1[0-9]{5}\.npz$' ;;
...
python baselines/rl/train_rlpd.py \
  --steps "$STEPS" --scope pick --action-mode delta_joint --delta-ref target \
  --action-repeat 1 --gamma 0.998 ...
```

Both therefore hit the same encoder, `train_sacfd_full._delta_actions`
(train_sacfd_full.py:104-113), with the same constant `cap = env.delta_cap = 0.025`:

```python
    cmds = np.stack([t[1] for t in trans]).astype(np.float64)
    if delta_ref == 'measured':
        ref = np.stack([t[0][:6] for t in trans]).astype(np.float64)
        dq = np.clip((cmds[:, :6] - ref) / (5.0 * cap), -1.0, 1.0)
    else:
        ref = np.concatenate([trans[0][0][None, :6].astype(np.float64),
                              cmds[:-1, :6]])
        dq = np.clip((cmds[:, :6] - ref) / cap, -1.0, 1.0)
    grip = np.clip(cmds[:, 6], 0.0, 1.0) * 2.0 - 1.0
```

The encoder differences **pure commands with no leash**. The env applies the
delta and then **leashes the resulting target to the measured qpos**
(full_env.py:386-392):

```python
                d = np.clip(a[:6], -1.0, 1.0) * self.delta_cap
                sp = np.clip(self._dj_target + d, ARM_LO, ARM_HI)
            self._dj_target = self._dj_qmeas + np.clip(
                sp - self._dj_qmeas, -self.delta_leash, self.delta_leash)
```

Consequence: a demo frame is representable **iff** `|cmd_t - cmd_{t-1}| <= cap`
(0.025) for every arm dim **and** `|cmd_t - qmeas_t| <= leash` (0.125). Measured
single-step label error (no accumulated drift assumed — the friendliest possible
framing, and the one that matters for a critic that sees one transition at a time):

| set | arm | frames | frac frames one-step label err > 1e-3 | mean err (rad) | p99 | max | leash-exceeded | cap-exceeded |
|---|---|---|---|---|---|---|---|---|
| `episodes_pick_phase_all` | **dH (live)** | 83,556 | **13.293 %** | 0.00727 | 0.15243 | 1.26669 | 11.354 % | 4.402 % |
| `episodes_champion_pick` | **dR2D (live)** | 9,184 | **0.000 %** | 0.00000 | 0.00000 | 0.00000 | 0.000 % | 0.098 % |
| `m1all_harvest` | dDP (paper arm) | 70,028 | 11.294 % | 0.00395 | 0.08638 | 0.33488 | 5.185 % | 7.891 % |
| `episodes_pick_phase_dppruned` | dH-pruned | 39,398 | 14.102 % | 0.00457 | 0.09183 | 0.40304 | 11.818 % | 5.084 % |
| `episodes_all` | dH-full | 186,239 | 8.692 % | 0.00461 | 0.10545 | 1.26669 | 6.579 % | 4.101 % |
| `episodes_delta_rerecord` | dH re-record | 201,016 | 2.608 % | 0.00022 | 0.00710 | 0.03441 | 0.000 % | 3.242 % |

Cumulative (open-loop) view, same encoder + a replica of the eval integrator fed
the tape's own recorded qpos as reference: **90 of 91 dH episodes** contain frames
whose reconstructed target is >1e-3 off the recorded command; per-episode max
error median **0.1266 rad**, p90 0.2501, max 1.2667; 83.9 % of frames per episode.
**All 66 dR2D episodes reconstruct exactly (per-episode max error 0.0000).**

And the divergence lands *before* the thing being learned: for the 66 dH picked
episodes the first frame with target error >1e-3 has median index **66**, while the
recorded pick frame has median index **558** — the target has left the demonstrated
command **66/66 episodes before the demonstrated grasp**. For dR2D: **0/66**.

Why this is critical, not cosmetic. RLPD's demo buffer is permanently half of every
batch (`rlpd_sac.DemoData`, sampled 128/256). Those tuples are
`(s, a_delta, r, s', done)` where `s'` was produced by executing the *original
absolute command*, not `a_delta`. For dR2D the tuple is an exact sample from the
env's transition kernel; for dH, on 13.3 % of tuples it is not, and the error is
large (p99 0.152 rad ~ 8.7 deg on a joint). Half of every dH critic batch is drawn
from a buffer with that property; half of every dR2D batch is drawn from a clean
one. A difference in ignition rate between the arms is therefore **not attributable
to demo source alone**.

Status of prior knowledge: `paper/p1_delta_divergence_2026-08-13.md` established
target drift as the proximate killer of open-loop delta replay, and
`paper/cell_b_clean_demos_2026-08-15.md` §4(b) recorded 53 % (model) vs 47 %
(human) population re-earn rates — then explicitly set the issue aside: *"Neither
number is a training-validity claim: RLPD never replays demos open-loop. The
recorded (s, a, s') tuples are exact by construction."* That last sentence is true
of the model set and **false of the human set**, and the asymmetry is the confound.
The number above (13.29 % vs 0.000 %) is the single-step quantity that RLPD
actually consumes, so it does not depend on the open-loop-replay objection.

What it does NOT affect: the BC/DP arm. `convert_to_lerobot.py:84` writes
`'action': d['actions'][i]` — the raw absolute command — and `dp_runner.policy_action`
returns the un-normalized absolute action straight to `env.step`. No projection, so
dH and the model sets are equally faithful there and the BC demo-source comparison
is not confounded by this mechanism.

Cheapest de-confounding options, in ascending cost:
1. Report the contrast with this asymmetry stated, and quote the two fidelity
   numbers next to the rates (no new compute).
2. Re-run the dH arm on `baselines/episodes_delta_rerecord` (2.6 % vs 13.3 %) or
   with `--delta-ref measured` (measured-ref round-trip on the same uid-232 tape:
   max err 6.28e-03, 2/278 frames >1e-5, versus 6.75e-02 and 210/278 for target-ref).
   Neither is exact, but both cut the asymmetry ~5-25x.
3. Harvest a matched-fidelity human set (closed-loop re-record at cap 0.025 /
   leash 0.125), which is what `episodes_delta_rerecord` was for.

### C2 — CRITICAL (documentation, feeds C1). `full_env`'s stated calibration does not hold on the demo set in use

full_env.py:156-159 justifies both constants:

```python
        # Cap 0.025 = the demos' p99 per-frame commanded delta (44 deg/s
        # saturated); leash 5*0.025 = 0.125 covers the demos' |cmd-q| PD lead
        # (p99 0.126). MUST match the demo-buffer delta encoding
```

The calibration is real but **per-VALUE** (over all 6 dims x frames) and was taken
on `episodes_pick_pruned_img`; I reproduced it there exactly: per-value commanded
delta p99 **0.0249**, max **0.0594**, lead p99 **0.1255**. On the set the live dH
arm actually trains on (`episodes_pick_phase_all`) the same per-value figures are
delta p99 **0.0253** / max **0.0563** and lead p99 **0.1520** / max **1.3917** —
the lead p99 is already 1.22x the leash. And per-FRAME (the unit an action
occupies) the numbers are far worse: per-frame max-over-dims delta p99 **0.0412**,
lead p99 **0.2774**; **4.402 %** of frames have some dim over cap and **11.354 %**
have some dim over leash.

A per-value p99 target guarantees ~1 % value-level violation, which becomes
~11-13 % frame-level label corruption. The constants are not "wrong"; the comment's
implication that they cover the demos is what is wrong, and it is why C1 went
unnoticed. Same claim is repeated in `to_dreamer_demos.py:63-69` and in
`r2dreamer/configs/env/genesis_pick_v5d4c_delta.yaml:48-50`.

### M1 — MODERATE. `sacfd_delta_gate.py` — the gate that guards exactly this property — cannot run at HEAD

sacfd_delta_gate.py:201:

```python
    dd = DemoData(trans, action_transform=None, device=th.device('cpu'))
```

`DemoData.__init__` gained a required `seed` parameter in `2fbed2a` (the demo-RNG
seed audit), so this raises immediately:

```
$ .venv-eval/bin/python -c "... rlpd_sac.DemoData([], None, th.device('cpu'))"
TypeError: DemoData.__init__() missing 1 required positional argument: 'seed'
```

Signature is now `(self, transitions, action_transform, device, seed)`. The
tensor-equality half of the gate is dead, and the open-loop half never runs because
the gate exits at stage 1. Note `cluster/sbatch_rlpd.sh` does **not** invoke the
gate, so the n=8 wave launched with no encoder gate at all. One-line fix:
`DemoData(trans, action_transform=None, device=th.device('cpu'), seed=0)`.

### M2 — MODERATE. The gate's uid list is selection-biased away from the C1 failure, by its own admission

sacfd_delta_gate.py:33-44 documents the selection:

```python
# Gate uids empirically selected (2026-08-12) as gentle-speed, RESETTABLE pick demos
# whose delta-encoded replay reliably re-earns the env pick predicate. NOTE on why
# NOT 232/242/243 (an earlier guess): ... Fast demos (max joint step > cap 0.025)
# whose replay saturates the delta cap lose that razor-thin margin (uid 232 lifts to
# only 0.1456) ... Of the gentlest resettable pick demos 9/11 re-earn the pick
```

So the standing `GATE PASS 4/5` is measured on 5 demos hand-picked as the gentlest
of 11, i.e. chosen for *not* exhibiting the cap/leash violation. A gate whose pass
criterion is evaluated on the subset that cannot fail cannot detect C1. The
`--all-demos` sweep mode is the unbiased measurement and it reports 47 % (human) —
that number should be the gate, or at least be reported alongside it.

### M3 — MODERATE. Stamped provenance is written and never validated

`harvest_champion_demos.py` stamps `delta_ref`, `delta_scale=0.025`,
`teacher_action_repeat=4`, `tape_stride=1` into every npz (verified present in
`episodes_champion_pick/100000.npz`). **Nothing reads them at train or eval time.**
`grep -rn "delta_scale|teacher_action_repeat|tape_stride" --include=*.py .` outside
the harvester returns only `make_place_phase_datasets.py` and
`rerecord_delta_demos.py` (both writers). `train_rlpd.py` encodes with
`env.delta_cap` without ever comparing it to the tape's stamped `delta_scale`; the
values happen to agree today (0.025 == 0.025, verified) but a set harvested at a
different cap would be silently mis-scaled. This is the silent-default family with
the fix already half-built: the field exists, the assert does not. Two lines in
`train_rlpd.py` closes it.

### M4 — MODERATE. Demo-buffer composition differs by ~9x in reward density between the arms

Not an encoding bug, but it rides in the same one-variable claim and must be in the
paper. Both arms measured through `delta_encode_transitions(..., scope='pick')`:

| | dH | dR2D |
|---|---|---|
| episodes | 91 | 66 |
| transitions | 83,465 | 9,118 |
| rewarded transitions | 66 (**0.079 %**) | 66 (**0.724 %**) |
| episode length p50 / mean / max | 745 / 917 / 3,694 | 130 / 138 / 217 |
| no-pick negatives | 25 eps, 27,602 transitions (**33.1 % of buffer**) | 0 eps (**0 %**) |
| transitions after the pick terminal | 0 | 0 |

Under RLPD's fixed 128/256 demo half, the dH critic sees a demo stream that is
9.2x sparser in reward and one third zero-reward no-pick data; the dR2D stream is
all-success. `paper/cell_b_clean_demos_2026-08-15.md` §3 already tabulates the
per-set densities, so this is known — the point here is that "one variable, the
demo dir" is not one variable.

### B1 — BENIGN (fragile). The delta constants live in four places, not one

| location | cap | leash multiplier | source of truth? |
|---|---|---|---|
| `full_env.py:121-122` (`FullTaskEnv.__init__` defaults) | 0.025 | 5.0 | yes for training (`env.delta_cap` is passed into the encoders) |
| `train_sacfd_full.py:107` and `:393` | passed in | **`5.0` hardcoded** | no — multiplier is not read from the env |
| `wandb_eval.py:178` | **`0.025` hardcoded** | **`5.0 * 0.025` hardcoded** | no — not read from env or sidecar |
| `to_dreamer_demos.py:63` (`--delta-cap` default) | **0.04** | n/a | no — and the default differs from the value in use |
| `harvest_champion_demos.py:222-223` (fallbacks) | **0.04** | **3.0** | no — fallbacks if the teacher cfg lacks the keys |
| `r2dreamer/configs/env/genesis_pick_v5*.yaml:48-50` | 0.025 | 5 | per-config |

`wandb_eval.py:178` is the load-bearing duplicate:

```python
        DJ_CAP, DJ_LEASH = 0.025, 5.0 * 0.025
```

**Values match today** (`FullTaskEnv` defaults 0.025 / 5.0, champion manifest
`delta_cap: 0.025, leash_mult: 5.0`), and I verified the arithmetic is a
branch-for-branch mirror of `_step_once` including the first-step seeding. But a
run built with `FullTaskEnv(delta_leash_mult=3.0)` would train in one MDP and eval
in another with no assert firing, and `harvest_champion_demos.py`'s 0.04/3.0
fallbacks would silently produce a mis-scaled tape whose stamped `delta_scale`
nobody checks (M3).

### B2 — BENIGN (fragile). The r2dreamer cap<->demo-dir pairing is a bash filename glob

`cluster/sbatch_r2dreamer.sh:71-73`:

```bash
  *v5*)    DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta25 ;;  # cap 0.025 sets
  *delta*) DEMO_DEFAULT=$DV3_DIR/demonstrations/genesis_pick_pruned_delta ;;
```

Correct today — I confirmed the two dirs really do carry different caps by their
saturation signature (`genesis_pick_pruned_delta25`: 5.28 % of rows have a
saturated arm dim, matching cap 0.025 on this source; `genesis_pick_pruned_delta`:
1.23 %, matching cap 0.04). But the only thing tying a tape's cap to the env's cap
is the directory name and a `case` pattern; `demo_prefill` asserts
`demo_downsample == action_repeat` and does not assert anything about the cap.

### B3 — BENIGN. Two docstrings misdescribe their own code

1. `train_sacfd_full.py:344-351` — `delta_encode_transitions_measured` says
   *"Action = clip((cmd_t - qmeas_t)/cap, -1, 1)"*, but the code divides by
   `5.0 * cap` (the leash). The clarification is 9 lines further down (`:360-361`).
   Contradictory docstrings in a function whose whole purpose is scale agreement.
2. `harvest_champion_demos.py:28-33` claims the re-derived stride-1 delta *"is
   exactly the per-sim-step delta the teacher commanded."* It is not: the champion
   sits pinned at the leash (per-value `|cmd - qmeas|` p50 = p99 = max = **0.1250**,
   i.e. the clamp is active on most frames), so the recorded absolute command is
   the *clamped* target and the re-derived delta is the clamped increment. Only
   **0.50 %** of consecutive dR2D arm-action pairs are identical, where ~75 % would
   be expected if raw `a*cap` advance survived a repeat-4 window. Harmless — the
   round trip is exact either way (that is the point of C1's clean column) — but
   the stated reason is wrong.

### B4 — BENIGN. Verified non-issues (each a past bug in this repo, checked and clear)

* **Grip column / negative recorded grip.** Zero frames with `actions[:,6] < 0` in
  *all* sets checked (dH, dR2D, dDP, episodes_all, dppruned, delta_rerecord). The
  `[0,1]` clip in `genesis_can_env.step:218` and in `_delta_actions:112` is a
  no-op on today's tapes. dH grip range [0.000000, 0.994630] -> action range
  [-1.000000, 0.989260]; dR2D [0.000000, 1.000000] -> [-1.000000, 1.000000].
  Grip is column 6 in every 7-dim joint path (the 5-dim cartesian index-4 variant
  is a different code path and out of scope here).
* **Stale goal.** All sets carry `goal_xy = (0.672, -0.221)` at frame 0 — the
  current human-validated goal, one distinct value per set, no `(0.662, -0.057)`
  survivors.
* **Obs normalization.** None. No `VecNormalize`, no running mean/std anywhere in
  `baselines/`; SAC/RLPD consume raw 17-dim fp32 obs. Demo obs and env obs come
  from the same `GenesisCanEnv._obs()` (genesis_can_env.py:312-313), so column
  meaning and order are identical by construction: `q[:6]`, gripper motor 0..1,
  grip effort, can pos xyz, can quat wxyz, goal xy. The LeRobot DP path *does*
  normalize, but with stats baked into the checkpoint and applied identically at
  eval via `make_pre_post_processors` — demo stats and eval stats are the same
  object.
* **dtype boundaries.** npz `states`/`actions` fp32 in every set; `DemoData` casts
  to fp32; both integrators (encoder and eval) do the delta arithmetic in fp64 and
  hand fp32/fp64 absolute targets to the sim. Round-trip residual on the clean set
  is exactly one fp32 ULP (1.19e-07), which is the floor, not a leak.
* **Off-by-one.** Same convention in both sets, so no cross-set mis-pairing.
  `mean |a[t] - s[t+k]|` falls monotonically over k = -2..+2 in both
  (dH 0.02908 / 0.02697 / 0.02486 / 0.02282 / 0.02087; dR2D 0.07147 / 0.06641 /
  0.06131 / 0.05667 / 0.05209) — that is PD lag (the command leads the measured
  pose for many frames), not an index offset, and it is identical in shape across
  sets. The recording contract (`harvest_champion_demos.py:24-28`) is
  `states[t]` = obs BEFORE the step, `actions[t]` = command executed that step, and
  the encoder's row-0 reference (`trans[0][0][:6]`) is consistent with it.
* **Post-terminal demo transitions.** 0 in both live sets (measured): the pick-scope
  terminal is the last transition of every episode, so the critic never bootstraps
  through a state the env would not have continued from.
* **`#26` phantom settle.** Not reachable in the live eval: the cluster sweep runs
  `--max-steps 400` and `FullTaskEnv.__init__` sets `self.genv.max_steps = 10**9`
  (full_env.py:192).

### B5 — BENIGN. Row-0 encoder assumption is stale but harmless

`train_sacfd_full.py:96-98` and `:270-273` claim *"row 0 differenced against the
measured start pose (max |cmd_0 - q_0| over demos is 0.012 rad)"*. Measured:
dH max **0.05146**, 40/91 episodes over 0.012, and **3/91 episodes exceed the cap**
so their row-0 action clips. dR2D max **0.02168** with **66/66** episodes over
0.012 (p50 0.01890) but none over cap. Effect is confined to one frame per episode.

---

## 2. Full trace table, per pipeline

### 2.1 dH -> RLPD (live, `ARM=dH`)

| stage | artifact / code | representation | constants |
|---|---|---|---|
| record | `episodes_pick_phase_all/<uid>.npz` | `states (n,17) fp32` = obs BEFORE step; `actions (n,7) fp32` = ABSOLUTE `[6 joint targets rad, grip 0..1]`; `uid`=trial, `label`, `stage` | goal_xy (0.672,-0.221) |
| reward relabel | `train_sacfd_full.relabel_full` | pick proxy `can_z=s[:,10] > pick_z` AND `grip=a[:,6] > GRIP_CLOSED_FRAC`, gated by npz `stage` rank; `n = len(s)-1` | `pick_z=0.1505`, `GRIP_CLOSED_FRAC=0.3` |
| scope patch | `delta_encode_transitions` (scope='pick') | any transition with `r >= STAGE_REWARD['picked']` -> `done=True` | — |
| action encode | `_delta_actions(..., 'target')` | `a_arm = clip((cmd_t - cmd_{t-1})/cap, -1, 1)`, row 0 vs `s[0,:6]`; `a_grip = clip(cmd_grip,0,1)*2-1` | `cap = env.delta_cap = 0.025`; **no leash** |
| policy space | `RLPDSAC` on `FullTaskEnv.action_space` | `[-1,1]^7`, fp32; no obs normalization | — |
| env decode | `full_env._step_once` target branch | `sp = clip(target + clip(a,-1,1)*cap, ARM_LO, ARM_HI)`; `target = qmeas + clip(sp - qmeas, +-leash)`; grip `(clip(a6)+1)/2` | `cap 0.025`, `leash 0.125` (`5.0 x cap`) |
| eval decode | `wandb_eval.py:186-215` | byte-faithful mirror of the above, stateful across steps, reset per episode by `eval_core:36` | `DJ_CAP, DJ_LEASH = 0.025, 5.0*0.025` **hardcoded** |
| sidecar | `<ckpt>.action_mode.json` | `action_mode`, `action_repeat`, `delta_ref`, `scope`, `demo_dir`, git | written per checkpoint by `SidecarCheckpointCallback` |
| **verdict** | | encode/decode exact inverses; **demo tape not representable on 13.29 % of frames (C1)** | |

### 2.2 dR2D -> RLPD (live, `ARM=dR2D`)

Identical to 2.1 except:

| stage | difference |
|---|---|
| record | `harvest_champion_demos.py`; `uid` = rollout index, `ic_uid` = IC; stamps `delta_ref='target'`, `delta_scale=0.025`, `teacher_action_repeat=4`, `tape_stride=1`, `act_mode` (mode\|sample, mixed across shards), teacher ckpt + both git shas |
| teacher constants | `manifest.json`: `delta_cap 0.025`, `leash_mult 5.0`, `repeat 4`, `pick_z 0.15050000000000002` — all equal to the `FullTaskEnv` defaults the trainer uses |
| stamped fields | **read by nobody at train/eval time (M3)** |
| composition | 66 eps, all `label=success stage=picked`, no negatives |
| **verdict** | encode/decode exact inverses **and** tape exactly representable (0.000 % label error) |

### 2.3 dDP -> RLPD (`m1all_harvest`, paper arm, not in the 08-17 wave)

Same code path as 2.1. Same 17/7 layout, `uid` = rollout index, 93 eps
(63 picked / 30 no-pick). Label fidelity **11.294 %** frames >1e-3 — same regime as
dH, so the dH-vs-dDP contrast is *not* confounded by C1; only contrasts against
dR2D are.

### 2.4 dH / model sets -> LeRobot DP + ACT (BC arms)

| stage | code | representation |
|---|---|---|
| convert | `convert_to_lerobot.py:82-87` | `observation.state = states[i][:PROPRIO]` (default `sdim-9` = 8), `observation.environment_state = states[i][PROPRIO:]` (9), `action = actions[i]` **raw absolute** |
| train | LeRobot policy | normalization from dataset stats, baked into the checkpoint |
| eval | `dp_runner.policy_action` | same split from `obs['state']`, `pre(batch)` / `post(action)` from `make_pre_post_processors(pretrained_path=checkpoint)`, returns absolute action to `env.step` |
| **verdict** | no delta projection anywhere; demo stats == eval stats by construction; unaffected by C1/C2 |

### 2.5 dH / dDP -> r2dreamer (dv3 world-model arm)

| stage | code | representation | constants |
|---|---|---|---|
| convert | `to_dreamer_demos.py:108-122` | `action[t]` = `clip((cmd_t - cmd_{t-1})/delta_cap, -1, 1)` + grip `*2-1`, written BACKWARD-in-row with a zero action on `is_first` | `--delta-cap` **default 0.04**, passed 0.025 for the `*delta25` dirs |
| ingest | `r2dreamer/demo_prefill._load_episode` | asserts `action.shape == (T, act_dim)`, `is_first[0]`, `is_last[-1]`, zero action on `is_first`; downsamples by `demo_downsample` (asserted `== action_repeat`) taking reward=SUM and action=**MEAN** of the window; then shifts actions forward one row to match online convention | `demo_downsample 4 == action_repeat 4` |
| env | `genesis_pick_v5d4c_delta.yaml` | `delta_cap: 0.025`, `delta_leash_mult: 5`, `action_repeat: 4`, `time_limit: 400` | cap<->demo-dir paired only by the `case` glob in `sbatch_r2dreamer.sh:71-73` (B2) |
| **verdict** | conventions asserted where they matter (shape, is_first, downsample==repeat); **cap agreement is unasserted (B2)**; the human tapes carry the same non-representability as C1 (`genesis_pick_pruned_delta25`: 5.28 % of rows saturate an arm dim) |

---

## 3. Numerical round-trip results (verbatim)

Encoder = the real `train_sacfd_full` functions imported from the repo; decoder =
the arithmetic of `wandb_eval.py:186-215` replicated line for line and fed the
tape's own recorded `qmeas` as the measured reference.

```
dH  human  uid232 (232.npz, n=278 frames, dref=target):
  arm reconstruction err: max=6.747e-02 mean=3.555e-03 p99=1.680e-02
  frames with err>1e-5: 210/278; >1e-3: 210/278
  grip reconstruction err: max=1.490e-08
  worst frame 171: cmd=[ 0.541169 -1.420119  1.83883  -1.14733   1.675576 -1.597515]
                   rec=[ 0.532317 -1.420119  1.83883  -1.2148    1.678476 -1.597515]
    |cmd-qmeas| at that frame = 0.13128 (LEASH=0.125)
  first frame with err>1e-5: 68 (of 278)

dR2D champion 100000 (100000.npz, n=121 frames, dref=target):
  arm reconstruction err: max=1.192e-07 mean=1.511e-08 p99=1.192e-07
  frames with err>1e-5: 0/121; >1e-3: 0/121
  grip reconstruction err: max=0.000e+00

--- same tapes, measured-ref encoder/decoder pair ---
dH  human  uid232 (dref=measured):  max=6.276e-03  frames err>1e-5: 2/278
dR2D champion 100000 (dref=measured): max=1.192e-07  frames err>1e-5: 0/121

=== whole-set open-loop round trip (target-ref) ===
dH:   eps=91 per-ep max arm err: p50=0.1266 p90=0.2501 max=1.2667
      eps with any frame err>1e-3: 90/91 | mean frac of frames err>1e-3 = 0.8389
dR2D: eps=66 per-ep max arm err: p50=0.0000 p90=0.0000 max=0.0000
      eps with any frame err>1e-3:  0/66 | mean frac of frames err>1e-3 = 0.0000
```

Encoder output ranges (action-space sanity):

```
dH:   83,465 transitions, 66 rewarded (0.079%)
      dtype float32, max|a| = 1.000000
      arm-dim saturation frac 0.01088; frames with ANY saturated arm dim 0.04398
      arm |a| p50 0.00074  p99 1.00000
      grip a range [-1.000000, 0.989260]  (at -1: 7.80%, at +1: 0.00%, interior 92.20%)
dR2D: 9,118 transitions, 66 rewarded (0.724%)
      dtype float32, max|a| = 1.000000
      arm-dim saturation frac 0.00044; frames with ANY saturated arm dim 0.00186
      arm |a| p50 0.20133  p99 0.85365
      grip a range [-1.000000, 1.000000]  (at -1: 1.10%, at +1: 0.69%, interior 98.21%)
```

Both sets stay inside `[-1,1]` (that is guaranteed by the encoder's `clip`, so the
informative figure is the saturation fraction: dH clips an arm dim on **4.4 %** of
frames, dR2D on **0.19 %** — a 24x difference in how often the action label is
truncated).

Raw-tape statistics behind the above:

```
episodes_pick_phase_all (dH):  91 eps, 83,556 frames, fp32/fp32
  stages {picked: 66, no-pick: 25}   labels {success: 66, fail: 25}
  goal_xy frame0: {(0.672, -0.221)}
  grip cmd range [0.000000, 0.994630]; frames with grip<0: 0
  |cmd-qmeas| per-frame max-over-dims: p50 0.04181  p99 0.27743  max 1.39169  frac>leash 0.11354
  |cmd_t-cmd_(t-1)| per-frame max:     p50 0.000165 p99 0.041178 max 0.056308 frac>cap   0.04403
episodes_champion_pick (dR2D): 66 eps, 9,184 frames, fp32/fp32
  stages {picked: 66}   labels {success: 66}
  goal_xy frame0: {(0.672, -0.221)}
  grip cmd range [0.000000, 1.000000]; frames with grip<0: 0
  |cmd-qmeas|: p50 0.12500  p99 0.12500  max 0.12500  frac>leash+eps 0.000
  |cmd_t-cmd_(t-1)|: p50 0.011839 p99 0.022876 max 0.025000 frac>cap 0.00099
```

Other sets, same measurement:

```
episodes_all:                  91 eps 186,239 fr  grip<0 0  goal (0.672,-0.221)  lead p99 0.2305 frac>leash 0.0658  dcmd p99 0.0372 frac>cap 0.0410
episodes_pick_phase_dppruned:  66 eps  39,398 fr  grip<0 0  goal (0.672,-0.221)  lead p99 0.2168 frac>leash 0.1182  dcmd p99 0.0449 frac>cap 0.0509
episodes_delta_rerecord:       72 eps 201,016 fr  grip<0 0  goal (0.672,-0.221)  lead p99 0.1250 frac>leash 0.0003  dcmd p99 0.0321 frac>cap 0.0324
m1all_harvest:                 93 eps  70,028 fr  grip<0 0  goal (0.672,-0.221)  lead p99 0.1924 frac>leash 0.0519  dcmd p99 0.0698 frac>cap 0.0784
```

dv3 demonstration dirs (cap identification by saturation signature):

```
genesis_pick_pruned_delta:      67 eps 42,706 rows  any-dim sat 0.0123  arm|a| p99 0.6474   -> cap 0.04
genesis_pick_pruned_delta25:    67 eps 42,706 rows  any-dim sat 0.0528  arm|a| p99 1.0000   -> cap 0.025
genesis_pick_msr_delta25_r4:    67 eps 10,752 rows  any-dim sat 0.0452  arm|a| p99 0.9745   -> cap 0.025, repeat 4
genesis_pick_pruned / genesis:  67 / 91 eps        any-dim sat 0.0000                        -> absolute encoding
```

---

## 4. Recommended actions, in priority order

1. **(C1, no compute)** State the fidelity asymmetry wherever the dH-vs-dR2D
   contrast is reported: 13.29 % vs 0.000 % of demo transitions carry an action
   label that does not reproduce the recorded command. Retract the sentence
   *"The recorded (s, a, s') tuples are exact by construction"* as a claim about
   the human arm (`paper/cell_b_clean_demos_2026-08-15.md` §4(b)).
2. **(M1, one line)** `DemoData(..., seed=0)` in `sacfd_delta_gate.py:201`, then
   re-run the gate. Add it to `sbatch_rlpd.sh` as a pre-train self-gate next to the
   git-ancestor and provenance checks.
3. **(M3, two lines)** In `train_rlpd.py`, after loading `paths`, assert any
   stamped `delta_scale == env.delta_cap` and stamped `delta_ref == args.delta_ref`;
   print the stamp census. Cheap closure of a silent-default the harvester already
   pays for.
4. **(M2)** Make the unbiased `--all-demos` re-earn rate the reported gate number,
   or report both with the selection stated.
5. **(B1)** Have `wandb_eval` read `delta_cap` / `delta_leash` from the sidecar
   (write them in `train_rlpd`'s sidecar dict) instead of hardcoding
   `DJ_CAP, DJ_LEASH`, and have `_delta_actions` take the leash multiplier from the
   env rather than the literal `5.0`.
6. **(C1, with compute)** If the n=8 point estimates separate, the top-up should
   include a matched-fidelity human arm (`episodes_delta_rerecord`, or
   `--delta-ref measured`) so the separation can be attributed.

---

## 5. Reproduction

Four standalone scripts (kept out of the repo, in the session scratchpad; each
imports the real encoders from `baselines/rl/train_sacfd_full.py` rather than
reimplementing them):

```
$SCRATCH/audit_norm.py   set statistics, encoder output ranges, per-episode and
                         whole-set round trip, pairing, cross-set semantics
$SCRATCH/audit_norm2.py  pairing sweep over k, row-0 assumption, buffer
                         composition, first-divergence-vs-pick-frame, other sets
$SCRATCH/audit_norm3.py  single-step action-label fidelity (the C1 table)
$SCRATCH/audit_norm4.py  same, for m1all_harvest / dppruned / episodes_all
```

`$SCRATCH = /tmp/claude-1000/-home-j-workspace-genesis-pickaplace/8ea5848e-5db6-4a43-bc8e-7e682f9837fc/scratchpad`.
Run as `.venv-eval/bin/python $SCRATCH/audit_norm.py` from the repo root. All CPU,
no env build, ~1 min each; nothing writes to the repo.
