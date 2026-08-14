# Measured-ref integration: RLPD trainable + evaluable on the re-recorded tapes (2026-08-14)

Closes the blocking preconditions in FABLE_HANDOFF_2026-08-13 §10 (closed-loop
re-record) for training and evaluating on `baselines/episodes_delta_rerecord`
(72 npz, states (n,17) + `actions` (n,7) ABSOLUTE commands, `actions_delta`
leash-scaled measured-ref, stamped `delta_ref`/`delta_scale`/`action_repeat`).

All work CPU-only (3 RLPD nb/ln trainers held the GPU throughout).

---

## 1. Files changed (4) — all additive, all defaults preserved

| file | change | default behaviour |
|---|---|---|
| `baselines/rl/train_rlpd.py` | `--delta-ref {target,measured}` (default `target`); env built with it + asserted; measured-ref demo encoders selected in lockstep; `delta_ref`/`demo_dir`/`scope` added to the sidecar; passes `delta_ref` to the in-train eval callback | `target` = every pre-2026-08-14 run, unchanged |
| `baselines/wandb_eval.py` | `--delta-ref {auto,target,measured}` (default `auto`); measured branch added to the INLINE delta integrator; `delta_ref` added to the result json | `auto` + a sidecar with no `delta_ref` -> `target`; integrator arithmetic in the target branch is untouched |
| `baselines/rl/wandb_utils.py` | `VideoEvalCallback(delta_ref='target')`; passes `--delta-ref` to the eval subprocess ONLY when `action_mode == 'delta_joint'` | omitted for every existing caller (`train_sacfd_full` does not pass it) |
| `baselines/rl/sacfd_delta_gate.py` | `--demo-dir` (default `baselines/episodes_pick_phase_all`); env `delta_ref` asserted; richer `[gate]` header | default invocation is the original gate |

`baselines/rl/full_env.py` NOT touched — `delta_ref='measured'` already existed
there and is the reference implementation everything else mirrors.

### Shared-tree safety (newbox_supp runs CPU eval sweeps sporadically)
Every new flag has an explicit default equal to the previous hard-coded
behaviour. A concurrently launched eval using existing flags resolves
`delta_ref -> target` and executes the SAME code path, verified byte-wise in
gate (d) below. The only observable difference for legacy runs is one extra
printed line (`[eval] no delta_ref in sidecar -> target (legacy checkpoint)`)
and one extra key in the metrics json (`delta_ref`), which no consumer reads
(`VideoEvalCallback._poll` reads `metrics`/`videos`/`tiled` only).

---

## 2. The integrator mirror (the critical piece)

`wandb_eval.py`'s delta integrator was inline and hard-coded target-ref. It now
mirrors `full_env._step_once` branch-for-branch. Measured branch:

```python
sp = np.clip(q + np.clip(a[:6], -1.0, 1.0) * DJ_LEASH, ARM_LO, ARM_HI)
_dj['target'] = q + np.clip(sp - q, -DJ_LEASH, DJ_LEASH)
```

against `full_env.py`:

```python
d  = np.clip(a[:6], -1.0, 1.0) * self.delta_leash
sp = np.clip(self._dj_qmeas + d, ARM_LO, ARM_HI)
self._dj_target = self._dj_qmeas + np.clip(sp - self._dj_qmeas, -leash, leash)
```

Correspondences checked, not assumed:
- **Scale is the LEASH (5*cap = 0.125), not the cap** — the measured action is a
  normalized desired PD *error*; cap-scaling under-drives 5x (the documented
  0/5 smoke).
- **`q` == `self._dj_qmeas`**: the env stores measured qpos at the END of each
  `_step_once` and seeds it from measured qpos at reset (`_sync_dj_target`);
  eval reads `obs['state'][:6]`, which is that same measured qpos. Equal at
  every step including the first.
- **`action_repeat`**: the env re-reads `_dj_qmeas` on every inner `_step_once`;
  eval's `policy_action` runs once per env step and re-reads `q` while holding
  the same normalized action for `_repeat` steps. Same trajectory at any N.
- Grip mapping `(clip(a[6],-1,1)+1)/2` unchanged and shared.

### Resolution rules (the silent-fallback trap, closed)
1. A sidecar carrying `delta_ref='measured'` is AUTHORITATIVE. Passing
   `--delta-ref target` on such a checkpoint **asserts** — it does not fall back.
2. `auto` + sidecar without the field -> `target` (legacy), printed.
3. `delta_ref='measured'` with `action_mode != 'delta_joint'` or `--cartesian`
   asserts (it is a joint delta_joint concept).

---

## 3. Demo-dir flexibility + reward census (precondition 3)

`relabel_full` reads only `states`, `actions`, `stage` (and `label` via the
manifest elsewhere); the re-record extras (`actions_delta`, `rewards`, `dones`,
`delta_ref`, `delta_scale`, `action_repeat`, `stamp_backfilled`) are ignored.
`d['actions']` is the ABSOLUTE command column, which is exactly what
`delta_encode_transitions_measured` differences against `states[:, :6]`.

Census through the exact RLPD demo path (`FullTaskEnv(delta_ref='measured')`,
`pick_z=0.1505`, scope=pick):

```
[census] baselines/episodes_delta_rerecord n_npz=72
[census] recorded labels: {'success': 61, 'fail': 11}
[census] recorded stages: {'nested': 20, 'contact': 8, 'placed': 27, 'picked': 6, 'no-pick': 11}
[census] relabel_full: 193616 transitions | grants {'picked': 61, 'placed': 55, 'contact': 22, 'nested': 15}
[census] measured encoder (pick): 193616 transitions | n_rewarded=153 sum_reward=220.0
         n_done=153 | |a_arm| max=1.0000 sat_frac=0.0267
[census] round-trip vs stamped actions_delta: max err 9.537e-07 over 72 eps
[census] DemoData: n=193616 n_rewarded=153 act_dim=7 obs_dim=17
```

- **61 pick grants — the pre-registered expectation, hit exactly.** Matches the
  §10 stride-1 census (picked 61 / placed 55 / contact 28 / nested 20) at picked
  and placed; contact/nested come in lower (22/15) because `relabel_full` uses
  the *geometric proxies* (`TOUCH_XY`, tilt<20) gated by the recorded rank,
  which are stricter than the env's settled predicates. Pre-existing convention,
  identical to how `episodes_all` is relabeled — not a re-record artifact.
- Encoder round-trip vs the stamped `actions_delta` column: **9.537e-07** —
  reproduces the §10 figure, i.e. the tapes are bit-consistent with the encoder
  the trainer now uses.
- Action saturation 2.7% at stride 1 (leash is not binding).

### ⚠ FLAG FOR THE OPERATOR — a real confound, not a bug

The baseline dH set and the re-record set are different *kinds* of dataset:

```
episodes_pick_phase_all (dH baseline): 91 npz, stages {picked:66, no-pick:25},
    83465 transitions, n_rewarded=66, sum_reward=66.0, n_done=66   # TRUNCATED at the pick
episodes_delta_rerecord (measured):    72 npz, full task,
   193616 transitions, n_rewarded=153, sum_reward=220.0, n_done=153 # FULL-LENGTH
```

At `--scope pick` the encoders set `done=True` on any frame whose reward
`>= STAGE_REWARD['picked']`, which on full-length tapes also catches the
placed (+1) / contact (+2) / nested (+4) grants. So a measured wave on
`episodes_delta_rerecord --scope pick` puts **post-pick transitions worth up to
+4 with `done=True`** into the immutable demo buffer, while the env at
`scope=pick` terminates at the pick and can never produce such a transition.
That is off-MDP demo data and an inflated regression target for the critic.

This is pre-existing encoder semantics (it would bite `episodes_all --scope
pick` identically); it simply never fired because every prior pick-scope wave
used a pick-PHASE set. Consequences:

- A `dH_target (episodes_pick_phase_all)` vs `dH_measured
  (episodes_delta_rerecord)` comparison would confound **delta_ref** with
  **demo-set truncation**. Do not read it as a single-lever result.
- Matched-lever remedy (one command, not run here — operator's call):
  `./.venv-eval/bin/python baselines/make_pick_phase_datasets.py --src baselines/episodes_delta_rerecord --outdir baselines/episodes_delta_rerecord_pick`
  then point the wave at `baselines/episodes_delta_rerecord_pick`.
- Running on the full-length set is defensible on its own terms (more demo data,
  richer states) — just not as a clean single-lever measured-vs-target contrast.

---

## 4. Gates

### (a) Measured-ref open-loop replay, 5 gate uids, FROM THE RE-RECORD TAPES — **PASS 4/5**

`sacfd_delta_gate.py --delta-ref measured --demo-dir baselines/episodes_delta_rerecord`

```
[gate] env built | pick_z=0.1505 delta_cap=0.025 delta_leash=0.125 delta_ref=measured
       action_repeat=1 demo_dir=baselines/episodes_delta_rerecord
[gate] TENSOR-EQUALITY OK: 7911 transitions, 15 rewarded, action dim 7
[gate] uid 308: re-earned_lift=False canz_max=0.1245 (pick_z=0.1505) steps=1426 demo_len=1426
[gate] uid 325: re-earned_lift=True  canz_max=0.2672 steps=1417 demo_len=1417
[gate] uid 297: re-earned_lift=True  canz_max=0.2443 steps=1150 demo_len=1150
[gate] uid 326: re-earned_lift=True  canz_max=0.2363 steps=978  demo_len=978
[gate] uid 265: re-earned_lift=True  canz_max=0.2357 steps=1800 demo_len=2940 (env horizon truncation)
[gate] OPEN-LOOP REPLAY 4/5 re-earned the demonstrated lift (need >=4)
GATE PASS
```

Reading: the 4 passers reproduce the tape's own recorded lift closely
(replay `canz_max` 0.236–0.267 vs recorded 0.237–0.267) — near-exact
reproduction, as expected for tapes native to this space, and far above the
target-ref gate's razor-thin 0.151–0.163 margins. **uid 308 genuinely diverges**
(replay 0.1245 vs recorded 0.2122) — it loses the grasp; a per-uid P2-class flip
that the >=4/5 tolerance absorbs by design (§9 doctrine: thresholds with slack
survive P2, exact per-episode outcomes do not). Note the failing uid differs
between the two gates (measured/re-record fails 308; target/pick-phase fails
326), which is the signature of borderline-episode noise rather than a
systematic integrator defect.

### (b) Eval round-trip, auto mode — **PASS**

300-step CPU train smoke (`--delta-ref measured --demo-dir
baselines/episodes_delta_rerecord --device cpu --utd 1 --eval-freq 0
--no-wandb`) produced `rlpd_final.zip` + sidecar:

```json
{"action_mode": "delta_joint", "action_repeat": 1, "delta_ref": "measured",
 "backup_entropy": "off", "per_member_ln": "off", "git": "5bd6113",
 "demo_dir": "baselines/episodes_delta_rerecord", "scope": "pick"}
```

`wandb_eval.py --kind sac --checkpoint <smoke> --random 2 --ic-mode demo
--max-steps 400` (no `--delta-ref`, i.e. auto):

```
[eval] action_mode from sidecar: delta_joint
[eval] delta_ref from sidecar: measured
[eval] action_repeat=1 delta_ref=measured (1 policy query per 1 env step(s))
232 ep0: picked=False ...
234 ep1: picked=False ...
json: delta_ref='measured'  eval/n=2
```

Resolved measured from the sidecar, 2 episodes completed, no error. (0/2 picks
is the expected output of a policy that took zero gradient steps —
`learning_starts=1000` > 300; this gate tests the plumbing, not capability.)

### (c) Fail-loud guard — **PASS**

Same checkpoint with `--delta-ref target`: exit 1,
`AssertionError: checkpoint sidecar says delta_ref=measured but --delta-ref
target was requested: ... Refusing.` No silent fallback.

### (d) Target-ref regression, byte-wise — **PASS**

`git show HEAD:baselines/wandb_eval.py` vs the patched file, same checkpoint
(`baselines/rl/checkpoints/rlpd_nb_dH_s1/rlpd_100000_steps.zip`, legacy sidecar
with NO `delta_ref`), same flags (`--random 5 --ic-mode demo --max-steps 400
--seed 0`):

```
HEAD: 232 F | 234 F | 235 picked=True | 236 F | 237 F   -> eval/picked 0.2
NEW : 232 F | 234 F | 235 picked=True | 236 F | 237 F   -> eval/picked 0.2
metrics equal: True   |   new-only json keys: {'delta_ref'}
```

Episode-for-episode identical. Plus the default gate regression
(`sacfd_delta_gate.py`, no flags): **GATE PASS 4/5**, unchanged.

---

## 5. Launch command for the 3-seed measured-ref wave

100k, pick scope, backup-entropy off. `--per-member-ln` left as a **placeholder
the operator sets** from the ln-wave readout (`off` = the nb config that
ignited; `on` = audit bug 2 lever).

```bash
# operator: set PMLN to on|off per the ln-wave verdict
PMLN=off
DEMOS=baselines/episodes_delta_rerecord     # or ..._pick after the §3 truncation
for S in 0 1 2; do
  ./.venv-eval/bin/python baselines/rl/train_rlpd.py \
      --steps 100000 --scope pick --action-mode delta_joint \
      --delta-ref measured --demo-dir $DEMOS \
      --gamma 0.998 --utd 10 --backup-entropy off --per-member-ln $PMLN \
      --action-repeat 1 --train-max-steps 900 \
      --eval-freq 25000 --eval-max-steps 400 \
      --device cuda --seed $S \
      --out-dir baselines/rl/checkpoints/rlpd_mref_dH_s$S \
      --run-name dH_RLPD-mref_s$S --project genesis_paper &
  sleep 20
done; wait
```

Post-wave sweep: §4a-2 protocol (fixed 100k checkpoint, 15 fresh processes per
IC mode, explicit flags). `wandb_eval` resolves `delta_ref=measured` from the
sidecar automatically; pass `--delta-ref measured` explicitly if you prefer the
flag to be in the shell history — passing `target` will refuse.

## 6. Artifacts

- Logs / smoke checkpoint / census script:
  `<scratchpad>/mref/` — `gate_measured_rerecord.log`, `gate_target_default.log`,
  `census_rerecord_pick.log`, `census_pickphase.log`, `train_smoke_measured.log`,
  `eval_measured_auto.log`, `eval_failloud.log`, `eval_target_{head,new}.log`,
  `rlpd_mref_smoke/`.
- No wandb runs created (all evals `--no-wandb`).
