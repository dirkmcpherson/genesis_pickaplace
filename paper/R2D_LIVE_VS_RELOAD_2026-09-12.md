# The counter-vs-checkpoint discrepancy is a WINDOW MISMATCH, not a live-vs-reload defect

Lane DV3-9, local box (pop-os), 2026-09-12. Learner: **{dv3 local}** = DreamerV3 losses
(`model.rep_loss=dreamer`) inside the r2dreamer chassis. Scope **e2e (`scope=full`)**, ladder
`nested_ramp` v2, tip guard `not_in_hand`, human arm `dHfull_all_rnrh`, seed 0.

## 0. Verdict

**A reloaded checkpoint acts exactly like the live agent did. There is no missing state, no
second agent instance, and no defect in `eval_genesis.py`.** The "training record says 0.36,
the checkpoint reloads to 0.00" shape comes from comparing a checkpoint against a
**125-episode training window that the checkpoint's own weights did not produce**: this policy
moves fast enough that a 125-episode window spans ~4 700 gradient updates and straddles a
sharp collapse-and-recovery transient. Matched to the ~60 episodes that ended immediately
before the capture, the training record, a probe that re-runs the checkpoint through the
TRAINER's own data path, and the `eval_genesis` cells all agree inside their 95 % intervals.

Two independent checks rule out "the saved agent is not the acting agent":

1. In the checkpoint file, the frozen clones `act()` actually reads are **bit-identical** to
   their live counterparts — `_frozen_{actor,encoder,rssm,value,reward,cont,slow_value}` vs
   `{actor,encoder,rssm,value,reward,cont,_slow_value}`: 79/79 tensors, **0 differing,
   max |diff| = 0.000e+00**. (`clone_and_freeze()` re-points `param_new.data = param_orig.data`,
   so the clones share storage with the live parameters for the whole run; the optimizer's
   in-place updates keep the share intact.)
2. Every tensor reachable from the acting path (`_frozen_encoder`, `_frozen_rssm`,
   `_frozen_actor`: parameters **and** buffers) is present in the checkpoint —
   `acting-path tensors absent from the checkpoint: 0`, printed by both probe runs. There is
   no EMA copy, no running normalizer and no non-persistent buffer in `act()`:
   `Dreamer.preprocess` only rescales `image`, and this run is state-input
   (`encoder.mlp_keys=state`, `cnn_keys=$^`). `grep register_buffer` finds exactly two buffers
   in the whole model (`ReturnEMA.ema_vals`, `Dreamer._bc_lambda_t`), both used only in
   `update()`. There is no `Dropout`/`BatchNorm` anywhere, so `agent.train()` (trainer) vs
   `agent.eval()` (eval_genesis) cannot change an action.

## 1. What was compared, and the artefacts

Run under test (read-only; PID 878324 was still training throughout):
`~/runs_dv3_local/dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0_resume2M/` — the warm-restart
extension, launch record `~/runs_dv3_local/LAUNCH_e2e_ramp_s0_resume.txt`.
Checkpoint: `~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/latest.pt`
= `milestones/online_1000000.pt`, sha256 `2b6f0b38…`, `step` = **1 117 624**
(`prefill_counter_origin` 117 624 + 1 000 000 online env steps).

Code trees: r2dreamer `main` @ `0cf3d9e`; genesis_pickaplace `ladder-unify-2026-09-11`.
**The tree did not drift under the running process in any way that matters**: the training run's
`[ladder]` stamp says `git=…-934-g67157db` and the probe/eval stamp says `…-941-g326910b`, but
`git diff --stat 67157db..326910b` is 9 files — `CLAUDE.md`, four `paper/*.md`, and four
`cluster/*` scripts. No file the adapter imports changed, and the three stamped sha256s are
identical on both sides (`full_env=23fe428f222f genesis_can_env=40544bf73c8c
stage_predicates=a589b4f05632`). Both also carry
`[sim-variant] gc_kp4_riser3_shelf6` (the training workers print it six times, once per env).

New scripts (r2dreamer `main`, local only — upstream push is not possible for that repo):

- `probe_live_vs_reload.py` — reloads the checkpoint and runs it through the **trainer's own
  data path**: `make_envs(config.env)` → `ParallelEnv(env_num=6)` → `envs/genesis.py` adapter →
  `FullTaskEnv`, with the acting loop copied verbatim from `trainer.OnlineTrainer.begin`
  (same `done`/`is_first` handling, one persistent `agent_state`, `action_repeat=4`,
  `wrappers.TimeLimit` horizon 1200/4 = 300 decisions), minus the replay buffer and
  `agent.update()`. ICs come from the **training reset** (`FullTaskEnv.reset(uid=None)` →
  uniform over the 74 `success_uids`), so no IC pinning is involved. It refuses to run unless
  `R2D_SIM_VARIANT` / `GENESIS_SIM_VARIANT` / `GENESIS_PICKAPLACE_ROOT` are exported (a
  `R2D_SIM_VARIANT` default of `'base'` is a different world). It also has an `--path inproc`
  mode that swaps the IC source for the bisect; **it was not needed** (see §3).
- `probe_train_record_rates.py` — per-episode stage rates from `<logdir>/console.log`, with
  windows that bracket a milestone counter so the comparison carries its own drift estimate.

Probe outputs: `~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/probe_trainerpath/`
(`trainer_sample60/`, `trainer_mode30/`, `record_w125.json`, and the two `.log` files).

Reproduce (cwd `~/workspace/r2dreamer`):

```bash
env GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
    R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 \
    MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 \
  .venv/bin/python probe_live_vs_reload.py \
    --checkpoint ~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/latest.pt \
    --path trainer --episodes 60 --act sample --device cuda \
    --out ~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/probe_trainerpath/trainer_sample60
# ... --episodes 30 --act mode --out .../trainer_mode30

.venv/bin/python probe_train_record_rates.py \
  --console ~/runs_dv3_local/dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0_resume2M/console.log \
  --milestone 1117624 --window 60
```

Wall clock: 60 trainer-path episodes = **89 s** after a ~25 s six-world build, on the box that
was simultaneously training. The probe is cheap enough to run on any checkpoint of record.

## 2. The table — checkpoint `online_1000000` (counter 1 117 624)

Rates are per-episode; `length` is in DECISIONS (one decision = `action_repeat` 4 sim steps).
95 % intervals are Wilson. `farside`/`slide_event`/`home`/`nested_v2` are 0 in every cell below
and are omitted except `farside`.

| cell | n | picked | placed_v2 | farside | tipped | mean len | med len | mean score |
|---|---|---|---|---|---|---|---|---|
| training record, last **125** before capture (ctr 1 080 327–1 117 583) | 125 | 0.872 | **0.360** [.281,.447] | 0.224 | 0.880 | 76.4 | 35 | 1.335 |
| training record, last **60** before capture (ctr 1 108 460–1 117 583) | 60 | 0.867 [.758,.931] | **0.133** [.069,.242] | 0.033 | 0.967 | 39.6 | 28 | 1.007 |
| training record, last **30** (ctr 1 113 403–1 117 583) | 30 | 0.833 | 0.100 [.035,.256] | 0.033 | 0.967 | 40.3 | 28 | 0.933 |
| training record, last **15** (ctr 1 115 932–1 117 583) | 15 | 0.800 | 0.067 [.012,.298] | 0.000 | 1.000 | 33.2 | 27 | 0.867 |
| **probe, trainer path, `eval=False` (sampled)** | 60 | 0.733 [.610,.829] | **0.117** [.058,.222] | 0.017 | 0.983 | 36.5 | 28 | 0.850 |
| **probe, trainer path, `eval=True` (mode)** | 30 | 0.833 [.664,.927] | **0.000** [.000,.114] | 0.000 | 1.000 | 28.6 | 29 | 0.833 |
| `eval_genesis` hold15, mode | 15 | 0.733 [.480,.891] | 0.000 [.000,.204] | 0.000 | 1.000 | 29.0 | — | 0.733 |
| `eval_genesis` hold15, sample | 15 | 0.533 | 0.000 | 0.000 | 1.000 | 31.0 | — | 0.533 |
| `eval_genesis` rnd30, mode | 30 | 0.500 | 0.167 [.073,.336] | 0.067 | 0.900 | 57.5 | — | 0.533 |
| `eval_genesis` rnd30, sample | 30 | 0.533 | 0.167 | 0.000 | 0.833 | 77.0 | — | 0.567 |

Read the table this way: **every matched row agrees.** The probe's `placed_v2` 0.117 sits on top
of the record's own last-60 value 0.133 (CIs [.058,.222] vs [.069,.242]); the probe's median
episode length 28 equals the record's 28; the mode probe (n=30, `placed_v2` 0/30, `tipped`
30/30, mean length 28.6) equals the `eval_genesis` hold15 mode cell (0/15, 15/15, 29) and the
record's last-15 (1/15, 15/15, 33.2). The ONE row that does not fit is the 125-episode window
— the row the original puzzle was built on — and its interval [.281,.447] excludes all the
others. `picked` 0.867 (record last-60) vs 0.733 (probe, sampled) is the widest matched gap;
z ≈ 1.8, and the mode probe reads 0.833, so it is inside sampling noise.

`rnd30` is a different IC set (the support box, not demonstration starts) and is not expected
to match a training window at all — training resets draw uniformly from the 74
`success_uids`, of which `hold15` is a 15-subset. It is listed only for continuity with the
cells of record.

## 3. Why the bisect was not needed

The brief's fork was: if the trainer-path probe reproduces TRAINING-like rates then
`eval_genesis` is at fault and the IC source / leash sync / obs construction must be bisected;
if it reproduces EVAL-like rates then the live agent is not the saved agent. **The probe gave
eval-like rates while running the training env path with the training IC draw**, which
eliminates the whole `eval_genesis` branch in one step — the harness cannot be responsible for
a number the trainer's own path also produces. And §0's two checks eliminate the other branch.
What is left is the third branch the brief allowed for: the comparison itself.

For the record, the paths really are equivalent where it matters. `eval_genesis.reset_to_uid`
bypasses `adapter.reset` but re-does everything behaviourally relevant — it calls
`FullTaskEnv.reset(options={'uid': …})` (which runs `_reset_tracker`, `_sync_dj_target`,
`_granted = acct.reset()`, `_t = 0`) and then `env.sync_delta_target()`, and it rebuilds the
obs from `env._state_vec(env._env.genv._obs()['state'])`. The only thing it skips is
`self._emitted_stages = set()`, which gates the emission of the NON-sticky `log_*` twins and
cannot move a physical outcome; `eval_genesis` scores from `env._env._granted` / `info`, and
the probe reads the sticky `log_ep_*` keys, exactly as `trainer.begin` does.

## 4. The mechanism, measured

Rolling 60-episode windows of the training record through the capture (counter 1 117 624;
`ep_placed_v2`, `ep_farside`, mean/median length):

| window end (ctr) | picked | placed_v2 | farside | mean len | med len |
|---|---|---|---|---|---|
| 1 108 268 | 0.900 | 0.583 | 0.417 | 116.8 | 68 |
| 1 111 480 | 0.867 | 0.417 | 0.267 | 82.9 | 40 |
| 1 114 878 | 0.917 | 0.283 | 0.133 | 63.6 | 31 |
| **1 117 583** (capture at 1 117 624) | 0.867 | **0.133** | 0.033 | **39.6** | **28** |
| 1 120 983 | 0.783 | 0.133 | 0.017 | 40.2 | 28 |
| 1 131 486 | 0.750 | 0.217 | 0.067 | 56.2 | 31 |
| 1 142 784 | 0.683 | 0.300 | 0.133 | 102.5 | 42 |
| 1 153 788 | 0.767 | 0.517 | 0.217 | 134.2 | 79 |
| 1 170 886 | 0.900 | 0.867 | 0.633 | 114.6 | 75 |

The capture landed in the trough of a collapse-and-recovery cycle: `placed_v2` fell 0.583 →
0.133 and mean episode length 117 → 40 in **9 315 counter steps**, then recovered to 0.867
within a further ~53 000. At this recipe the trainer performs one `agent.update()` per **8**
counter steps (`_updates_needed = Every(batch_size·batch_length / train_ratio · action_repeat)`
= `Every(16·64/512·4)` = `Every(8)`), so:

- the 125-episode window spans 37 256 counter steps ≈ **4 657 gradient updates**;
- the 60-episode window spans 9 123 ≈ **1 140 updates**;
- the collapse itself took ≈ **1 160 updates**.

A 125-episode window is therefore wider than the transient it is being used to measure. The
policy is non-stationary on the same timescale as the window, and an immutable milestone is a
POINT SAMPLE of it.

## 5. Two confirmations on other checkpoints (same run family, cells already on disk)

Both are "the record and the cells agree when the window is matched", in the other direction —
these checkpoints are NOT in a trough.

**`online_500000` of the same resume run** (counter 617 628), record last-60
(ctr 590 966–617 584) vs its cells:

| | picked | placed_v2 | farside | mean len |
|---|---|---|---|---|
| record last-60 | 0.617 | 0.150 | 0.133 | 109.1 |
| `eval_genesis` hold15 sample (n=15) | 0.867 | 0.200 | 0.200 | 130.8 |
| `eval_genesis` rnd30 sample (n=30) | 0.533 | 0.333 | 0.167 | 123.0 |

**The ORIGINAL run's final 2M weights** (`~/runs_dv3_local/dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/`,
cells in `~/runs_dv3_local/dv3e2e_ramp_eval_final2M/`), record last-30 / last-60 vs its cells:

| | picked | placed_v2 | farside | mean len |
|---|---|---|---|---|
| record last-30 | 0.900 | 0.667 | 0.367 | 181.9 |
| record last-60 | 0.817 | 0.600 | 0.317 | 186.4 |
| record last-125 | 0.808 | 0.448 | 0.192 | 169.3 |
| `eval_genesis` hold15 mode (n=15) | 0.800 | **0.667** | 0.600 | 231.7 |
| `eval_genesis` hold15 sample (n=15) | 0.667 | 0.400 | 0.267 | 178.4 |
| `eval_genesis` rnd30 mode (n=30) | 0.467 | 0.500 | 0.367 | 183.8 |

`placed_v2` 0.667 in the hold15 mode cell reproduces the record's last-30 value exactly. Note
that here too the 125-episode window (0.448) understates the checkpoint, in the opposite
direction — further evidence that the window, not the harness, is what moves.

## 6. What this does and does not say about the cluster's s945

The cluster case in `paper/ROUTE_CENSUS_RC2_*.md` (`nested_v2` 0.79 in the training record,
0/90 on the reloaded checkpoint) was NOT re-examined here — the VPN is down and no cluster tree
was touched. This local reproduction shows the mechanism is real, is large, and is sufficient to
produce a record-vs-checkpoint gap of the observed SHAPE from correct code. It does not prove
that is what happened to s945; 0.79 → 0/90 is a bigger gap than anything measured here, and the
check that would settle it is mechanical and now written down (§7).

## 7. Recommendations (no code applied to anything the running process imports)

1. **Never compare a checkpoint's cells against a wide training window.** The comparable
   quantity is the ≤ 60 episodes that ended immediately BEFORE the capture; report the 60
   AFTER it alongside, as the drift estimate. `probe_train_record_rates.py --milestone <ctr>
   --window 60` prints both. Applied to this checkpoint the "discrepancy" disappears.
2. **Minimal fix, proposed not applied** (it needs `trainer.py`, which the running job imports;
   put it on a side branch and merge when no local job is live): have `Milestones.capture()`
   stamp the matched window into `milestones/online_<N>.json` at capture time — the last 60
   episodes' `score`, `length` and `train_ep_*` means. The trainer already holds every episode
   it logs; a 60-slot ring buffer in `OnlineTrainer` and one extra dict in `capture()`'s `meta`
   would make every milestone self-describing, and the comparison would stop depending on
   whoever later picks a window out of `console.log`.
3. **When a record-vs-checkpoint gap survives a matched window**, run
   `probe_live_vs_reload.py --path trainer` before suspecting the evaluator: it is ~90 s for
   60 episodes and it separates "the harness" from "the weights" by construction. If the probe
   agrees with the evaluator, the evaluator is exonerated.
4. A **milestone is a point sample of a non-stationary policy.** For a cell of record on this
   recipe, prefer either several nearby checkpoints or an explicit statement that the cell is a
   single point on a curve that moves by ±0.4 in `placed_v2` within ~1 000 gradient updates.
   The 09-12 `home` sighting in `dv3e2e_ramp_eval_final2M/fresh_eval_rnd30_mode` (one episode)
   should be read with that in mind.

## 8. Files

- Probe: `~/workspace/r2dreamer/probe_live_vs_reload.py`,
  `~/workspace/r2dreamer/probe_train_record_rates.py` (r2dreamer `main`, local only).
- Probe output: `~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/probe_trainerpath/`
  — `trainer_sample60/{summary,episodes}.json`, `trainer_mode30/{summary,episodes}.json`,
  `record_w125.json`, `trainer_sample60.log`, `trainer_mode30.log`.
- Cells of record used: `~/runs_dv3_local/dv3e2e_ramp_resume_eval_m1000k/fresh_eval_*/metrics.json`,
  `~/runs_dv3_local/dv3e2e_ramp_resume_eval_m500k/fresh_eval_*/metrics.json`,
  `~/runs_dv3_local/dv3e2e_ramp_eval_final2M/fresh_eval_*/metrics.json`.
- Training records: `.../dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0_resume2M/console.log`
  (2 560 episodes at the time of reading, counters 118 465–1 417 866) and
  `.../dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/console.log` (2 470 episodes).
