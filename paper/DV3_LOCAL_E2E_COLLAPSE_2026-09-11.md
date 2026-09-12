# {r2dreamer, `model.rep_loss=dreamer`} local e2e run — collapse characterisation — 2026-09-11 (lane DV3-6)

Characterises the apparent training collapse in
`/home/j/runs_dv3_local/dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/` (launched 20:10, still **TRAINING** at the
time every number below was pulled — a live-run snapshot, not a finished-run readout). Its files were only ever
READ; both evaluated checkpoints are COPIES (`~/runs_dv3_local/dv3e2e_ramp_eval_m500k/`,
`~/runs_dv3_local/dv3e2e_ramp_eval_latest_now/`). Every number carries the command that produced it. Nothing was
started as a consequence of this note (no retrain, no config change) — that is left to the caller per the task.

## 0. Run identity (`ladder_provenance.json`, `.hydra/config.yaml`)

`scope=full`, `ladder=nested_ramp` (`unified-2026-09-10`), `tip_guard=not_in_hand` (sustain 4 frames),
`far_release=false`, `return_clamp=9.0` (env AND model agree), `model.rep_loss=dreamer`, `actor_dist=bounded_normal`,
`act_entropy=3.0e-05`, `action_mode=delta_joint`, demo arm `dHfull_all_rnrh` (74 episodes), git
`known-good-2026-08-27-903-ga35534b`. Ladder chain: `placed_v2` requires `picked`; `farside` requires `placed_v2`;
`slide_event` requires `farside`; `home` requires `slide_event`; nominal `max_return=9.0` = 1(picked) + 1(placed_v2)
+ up to 3(slide ramp) + 4(home). `terminal_stages=[home, tipped]`.

**Command used throughout §1 (a stable snapshot, since the run is live):**
```
cp .../dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/metrics.jsonl  /tmp/.../scratchpad/metrics_snapshot.jsonl
cp .../dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/console.log    /tmp/.../scratchpad/console_snapshot.log
date   # Fri Sep 11 22:42:53 EDT 2026 — snapshot taken here
```
Snapshot covers **554 episodes** (step 124824-657261) and **108 training-scalar rows** (step 122640-687624, the
train-scalar stream ran ~30k steps ahead of the episode stream at snapshot time because it logs on a fixed
`update_log_every=5000` cadence independent of episode boundaries).

## 1. Timeline

### 1a. Per-50-episode window — outcome rates (`episode/train_ep_*`, `episode/length`, `episode/score`)

Command: filter `metrics_snapshot.jsonl` for lines carrying `episode/train_ep_picked`, in file order (554 rows),
chunk into consecutive windows of 50.

| ep window | step range | n | picked | placed_v2 | farside | tipped | timeout (len=300) | mean len | mean score |
|---|---|---|---|---|---|---|---|---|---|
| 0-49 | 124824-169129 | 50 | 0.060 | 0.000 | 0.000 | 0.220 | 0.780 | 248.5 | 0.060 |
| 50-99 | 169652-218894 | 50 | 0.080 | 0.000 | 0.000 | 0.260 | 0.740 | 237.7 | 0.080 |
| 100-149 | 219440-265089 | 50 | 0.020 | 0.000 | 0.000 | 0.320 | 0.680 | 233.9 | 0.020 |
| 150-199 | 265249-312718 | 50 | 0.080 | 0.000 | 0.000 | 0.300 | 0.700 | 242.2 | 0.080 |
| 200-249 | 313076-359988 | 50 | 0.100 | 0.000 | 0.000 | 0.380 | 0.620 | 237.8 | 0.100 |
| 250-299 | 360032-412616 | 50 | 0.120 | 0.000 | 0.000 | 0.240 | 0.760 | 259.4 | 0.120 |
| 300-349 | 413379-456285 | 50 | 0.120 | 0.000 | 0.000 | 0.420 | 0.580 | 226.2 | 0.120 |
| 350-399 | 457413-506700 | 50 | 0.060 | 0.000 | 0.000 | 0.300 | 0.700 | 250.0 | 0.060 |
| 400-449 | 507323-551048 | 50 | 0.020 | 0.000 | 0.000 | 0.440 | 0.560 | 211.5 | 0.020 |
| 450-499 | 551065-601654 | 50 | 0.060 | 0.000 | 0.000 | 0.220 | 0.780 | 256.1 | 0.060 |
| 500-549 | 601698-654756 | 50 | 0.040 | 0.000 | 0.000 | 0.100 | 0.900 | 274.6 | 0.040 |
| 550-553 | 655593-657261 | 4 | 0.000 | 0.000 | 0.000 | 0.750 | 0.250 | 127.2 | 0.000 |

**`placed_v2` and `farside` are 0.000 in EVERY window — 0/554 episodes total.** The training run has never once
chained a pick into a placement, at any point in its history, not only in the flagged final stretch. There is no
stage to "regress from": what the task description calls a collapse is a collapse of `picked`/`tipped` activity
back to near-zero, not a regression from a higher stage ever reached. `home`, `slide_event` are consequently also
0/554 (both require `farside`, which never fired).

### 1b. The specific dry spell flagged in the task, in the context of the run's full history

The run has never stabilised on picking: **long picked=0 dry spells recur throughout its ENTIRE history**,
including the very first 40 episodes. Command: scan the 554-length `train_ep_picked` series for maximal runs of
consecutive zeros.

| dry-spell rank | length (episodes) | ep range | step range | tip rate inside it | timeout rate inside it | mean len |
|---|---|---|---|---|---|---|
| 1 | 48 | 406-453 | 512291-552641 | 0.375 | 0.625 | 220.6 |
| 2 | 43 | 53-95 | 171871-213704 | 0.186 | 0.814 | 253.8 |
| **3 (task's "last 30")** | **42** | **505-546** | **604364-653905** | **0.024** | **0.976** | **295.0** |
| 4 | 40 | 0-39 | 124824-161892 | 0.125 | 0.875 | 267.0 |
| 5 | 39 | 97-135 | 214225-252444 | 0.282 | 0.718 | 248.3 |
| 6-10 | 31,30,30,27,19 | (scattered) | (scattered) | 0.185-0.421 | 0.579-0.774 | 218-263 |

**What is genuinely new about the flagged dry spell is not its length (it is the 3rd-longest, comparable to
several earlier spells the run recovered from) but its composition: tip rate collapses to 0.024 (essentially 1
tip in 42 episodes) against a baseline of 0.125-0.421 in every other dry spell, and timeout rate rises to 0.976**
(vs. 0.579-0.875 elsewhere). Every prior dry spell was "actively attempting and mostly failing via tip"; this one
is "almost pure inertia" (run out the clock without tipping the can either). That is the actual signature to
explain — not "picking stopped" (it had stopped and restarted repeatedly before) but "even tipping stopped."

**As of the snapshot the run has NOT gone fully silent going forward**: episode 547 (step 654215) is a pick
(`picked=1, tipped=1, score=1.0`, 26 steps — it picked, then later tipped, in the same episode), followed by
three more short tip events (69-91 steps) through step 657261. Calling this a terminal death as of the snapshot
would overclaim; it is at minimum a partial reactivation, and given the run's own history of repeated
dry-spell/recovery cycles, whether this is genuine recovery or noise inside a longer decline needs a later
re-check the caller was asked not to start here.

### 1c. Training scalars vs. the same windows (diagnosing a dead policy)

Command: bin the 108 rows carrying `train/opt/loss`/`train/action_entropy` into the §1a step windows and average.

| step window | picked | tipped | timeout | `train/con` | `train/val` | `train/ret_095` | `train/adv` | `train/adv_std` | `train/action_entropy` |
|---|---|---|---|---|---|---|---|---|---|
| 124824-169129 | 0.060 | 0.220 | 0.780 | 0.9991 | 4.616 | 6.004 | 0.0446 | 0.1125 | -0.885 |
| 169652-218894 | 0.080 | 0.260 | 0.740 | 0.9996 | 7.311 | 8.972 | 0.0312 | 0.0772 | -5.701 |
| 219440-265089 | 0.020 | 0.320 | 0.680 | 0.9998 | 5.952 | 8.771 | 0.0139 | 0.0662 | -5.794 |
| 265249-312718 | 0.080 | 0.300 | 0.700 | 0.9993 | 5.749 | 8.557 | 0.0106 | 0.0801 | -5.891 |
| 313076-359988 | 0.100 | 0.380 | 0.620 | 0.9997 | 5.504 | 8.329 | 0.0263 | 0.0785 | -6.018 |
| 360032-412616 | 0.120 | 0.240 | 0.760 | 0.9988 | 4.545 | 8.174 | -0.0008 | 0.0695 | -6.014 |
| 413379-456285 | 0.120 | 0.420 | 0.580 | 0.9981 | 3.920 | 7.321 | -0.0016 | 0.0808 | -6.050 |
| 457413-506700 | 0.060 | 0.300 | 0.700 | 0.9993 | 3.406 | 6.736 | 0.0049 | 0.0612 | -6.042 |
| 507323-551048 | 0.020 | 0.440 | 0.560 | 0.9985 | 4.114 | 7.127 | 0.0133 | 0.0875 | -6.025 |
| 551065-601654 | 0.060 | 0.220 | 0.780 | 0.9986 | 2.589 | 6.328 | -0.0027 | 0.0562 | -5.752 |
| **601698-654756 (flagged spell)** | **0.040** | **0.100** | **0.900** | **0.9990** | **3.621** | **6.465** | **0.0162** | **0.0775** | **-5.349** |

Full step-level tables (not just the 11-row window means) are in the scratch script output; reproduce with:
```python
# per line in metrics.jsonl: filter on 'episode/train_ep_picked' -> episode rows;
# filter on 'train/opt/loss' or 'train/action_entropy' -> train-scalar rows; bin by step range.
```

**Actor entropy** (`train/action_entropy`, r2dreamer's continuous differential entropy under `bounded_normal`,
`act_entropy=3e-5`): starts at **+9.1** at step 122640 (near prefill-init), falls through the next 4 logged rows
(9.2 -> 6.7 -> 1.4 -> **-1.8** at step 142600) then **-4 to -6** by step ~150-170k, and then sits in a **-5.3 to
-6.2 band for the
rest of the run with no further trend** — including the flagged final window (-5.349), which is *inside* that
same band, not lower than it. **The comparison run that "learned fine"
(`dv3pick_dHfull_pick_local_rlDreamer_s0`, pick-scope, same architecture/`act_entropy`) shows the IDENTICAL
pattern**: +9.1 at step 67268 -> -5.2 by step 117300 -> a -4.9 to -6.1 band for the rest of its run (step 997256),
while still reaching hold15 15/15 / rnd30 18/30 sampled (`paper/DV3_LOCAL_EVAL_2026-09-11.md`). **Conclusion: this
magnitude of actor-entropy collapse is a shared trait of the architecture/`act_entropy` setting, present
identically in a run that worked. It does not, by itself, discriminate the collapsed run from the healthy one,**
and its timing (complete by ~step 150-170k) is >400k steps before the flagged dry spell — it cannot be its
proximate trigger, however much it may be a background precondition.

**`train/adv` / `train/adv_std`** (imagined-rollout advantage fed to the actor loss): small (|mean| < 0.045,
std 0.056-0.113) in **every** window including the earliest active ones — there is no window where advantage
magnitude visibly implodes right before or during the flagged spell relative to its own noisy baseline. **A
"vanishing advantage caused the freeze" story is not supported by this series** — advantage was already this
small while the policy was still picking/tipping at its highest rates (e.g. window 413379-456285: adv=-0.0016,
tipped=0.420).

**`train/con`** (world-model continuation probability, i.e. `P(not done)` under the imagined rollout): pinned at
**0.998-1.000 in every single window of this run, independent of the REAL tipped rate, which itself ranges
0.024-0.440 across the same windows.** This is the sharpest contrast against the working pick run, where `con`
correctly tracks DOWN from 0.997 to **0.78-0.91** as real terminations (tip, `slide_success`) become common over
training (`dv3pick_dHfull_pick_local_rlDreamer_s0`, sampled every 10th of 187 rows):
```
step 67268: con=0.997  step 517300: con=0.898  step 717300: con=0.885  step 817300: con=0.782  step 997256: con=0.763
```
The full-scope/`nested_ramp` world model **never learns that a meaningful fraction of real episodes end early**;
its imagined rollouts behave as if every episode reaches full length regardless of the actual tip rate. This is
a genuine, run-specific defect (not shared with the working comparison run), and it holds for the ENTIRE run —
including the earlier, more active windows — so it is a chronic property of this run's world model, not something
that switched on right before the flagged dry spell.

**`train/val`** (critic value estimate, same units as return, ceiling `return_clamp=9.0`): starts near 0 (0.03 at
step 122640, matching an untrained critic), spikes to **6-8** by step ~150-170k (close to the clamp), then
DECAYS noisily over the rest of the run down to **2.6-4.6** by the end — **still 20-100x the actual mean episode
score in the same windows (0.02-0.12)**, and vastly above the only return this policy has EVER realised
end-to-end (1.0, for `picked` alone — `placed_v2` has never fired, so 1.0 is the true empirical ceiling, not the
ladder's nominal 9.0). **Contrast with the pick run**, where `return_clamp=1.0` matches the ladder's own scope
(pick-only, no downstream stages to hallucinate), and `train/val` settles at a realistic **0.5-0.8** — matching
its actual ~50-60% pick rate almost exactly. **The full-scope run's critic is chronically overvaluing states
relative to what the policy has ever actually achieved, for the entire run, not merely at the end** — consistent
with a return ceiling (9.0) that reflects the ladder's nominal design rather than anything this policy has come
close to earning, compounded by the `con` miscalibration above (an imagined rollout that never predicts early
termination has more "runway" over which to imagine reward it has never observed).

**Optimizer health**: `train/opt/grad_scale` fluctuates between 4096 and 262144 across the run with no trend
toward either extreme (`4096 -> 32768 -> 65536 -> 131072 -> 65536 -> 131072 -> 131072 -> 262144 -> 131072 ->
131072 -> 65536 -> 131072`, sampled every 10th row) — no sign of persistent gradient over/underflow.
`fps/fps` stays at **82-84** through the flagged window and beyond (last logged row: 83.5 at step 687624) — no
slowdown, hang, or restart (`grep -c '^Logdir' console.log` = 1, `grep -c 'Compiling update function'` = 1,
`grep -c 'Demo prefill'` = 1 — one continuous process, never restarted). `grep -ic nan console.log` returns 1, but
it is a false-positive substring match inside the word "prove**nan**ce" (`ladder_provenance.json`) — **zero real
NaN/Inf/Traceback events** in the log.

**Demo eviction (a real, timed event; not directly implicated by the pick-run counter-example, see below).** The
run's own startup line computes (once, at launch, as a static FIFO-eviction estimate, not a live re-check):
```
'eviction_note': 'FIFO: first demo frame evicted after 470594 online env steps, all demo frames gone by
                   500000 online env steps (buffer.max_size=500000)'
```
With `prefill_counter_origin=117624`, that puts the FIRST demo eviction at raw counter ~588218 and ALL 74 human
demos (`dHfull_all_rnrh`, 29406 transitions) fully evicted by raw counter ~617624 (confirmed independently: the
`milestones/online_500000.json` sidecar records the 500k-online-step checkpoint at raw counter **617640**,
`online_sim_steps: 500016`). **This eviction window (588218-617640) brackets the observed transition exactly**:
the last confirmed pick before the flagged dry spell is at step 604252 (inside the window), and the near-total
passivity spell runs 604364-653905 (starting inside the window, continuing well past it). **This is suggestive
but NOT confirmed causal**: the identical eviction dynamic (buffer `max_size=500000` with a prefill far below
that) also occurs in the WORKING pick run — `'first demo frame evicted after 484436 online env steps, all demo
frames gone by 500000'`, i.e. full eviction at raw counter ~562256 — at a comparable absolute step count, without
producing any comparable collapse there (the pick run continues to consolidate performance for another ~440k
steps past that point). **The same mechanism (buffer-size-driven demo eviction around counter ~560-620k) fires in
both runs; only the full-scope run shows a matching behavioural change nearby.** The honest reading is: eviction
removes the one clean source of successful-pick transitions from the buffer at a similar wall-clock point in
both runs, but by itself it is not sufficient to explain a collapse (the pick run's own history disproves that),
so if it contributes here it does so by removing a stabilising anchor from an ALREADY only marginally stable
policy (recurring 20-48 episode dry spells since episode 0) rather than by being a novel destabiliser on its own.

## 2. Checkpoint evals (SAMPLED, hold15 + rnd30)

**Checkpoint provenance, checked before evaluating anything:**
```
$ python -c "import torch; a=torch.load('.../latest.pt', map_location='cpu', weights_only=False); \
             b=torch.load('.../milestones/online_500000.pt', map_location='cpu', weights_only=False); \
             print(a['step'], b['step'])"
617640 617640
```
`latest.pt`'s `step` field (617640) and the `online_500000.pt` milestone's `step` field (617640) are **identical**
— both checkpoints are the SAME training step. A full tensor-by-tensor comparison of `agent_state_dict`
(170 tensors) found **0 tensors differing** (only the pickled file bytes differ, presumably optimizer-state /
serialization-order noise, not model weights). **As of this task's copy time (22:51), "the current `latest.pt`"
and "the 500k milestone" are the same policy** — the run's periodic full-checkpoint save (`save_every` ~100k
online steps) had not yet advanced past this point when the copies were made (the run was at raw counter ~657k-
690k, i.e. ~30-70k online steps past the milestone, still short of the next save boundary). Both copies were made
per the task's instructions:
```
cp .../milestones/online_500000.pt  ~/runs_dv3_local/dv3e2e_ramp_eval_m500k/latest.pt
cp .../.hydra/config.yaml            ~/runs_dv3_local/dv3e2e_ramp_eval_m500k/.hydra/config.yaml
cp .../latest.pt                     ~/runs_dv3_local/dv3e2e_ramp_eval_latest_now/latest.pt
cp .../.hydra/config.yaml            ~/runs_dv3_local/dv3e2e_ramp_eval_latest_now/.hydra/config.yaml
```
Given the proven weight-identity, and since `eval_genesis.py --mode sample` reseeds each process
(`tools.set_seed_everywhere(args.seed)`, `--seed 0` both times) before drawing any actor samples, running the
SAME weights through the SAME IC set with the SAME seed in two separate processes produces bit-identical episode
sequences — evaluating both copies would burn GPU time shared with the live training run for zero additional
information. **The two eval cells below were run ONCE, against `dv3e2e_ramp_eval_m500k` — the result applies
equally to the "current latest.pt" label by the tensor-equality proof above.** If a future re-check finds
`latest.pt`'s `step` has advanced, that would be a genuinely new checkpoint and would need its own eval.

### Commands
```
cd ~/workspace/r2dreamer
export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
RUN=/home/j/runs_dv3_local/dv3e2e_ramp_eval_m500k

.venv/bin/python eval_genesis.py --checkpoint $RUN/latest.pt --episodes 15 --mode sample --max-steps 1200 \
  --ic-file /home/j/workspace/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --seed 0 --device cuda \
  --out $RUN/fresh_eval_hold15_sample
# --ic-set rnd --episodes 30 -> $RUN/fresh_eval_rnd30_sample
```
`--config` was not passed explicitly; it defaulted to `$RUN/.hydra/config.yaml` (the copy of the training run's
own resolved config), confirmed live by the eval's own `[eval]`/`[ladder]` stamp (see below).

### Results

Eval build stamp (both cells): `[ladder] unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4
ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | ... | full_env=23fe428f222f
genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-910-g0c7dae3` —
sha256 stamps for `full_env`/`genesis_can_env`/`stage_predicates` are IDENTICAL to the training run's own
`ladder_provenance.json`, so the reward/predicate code path is unchanged between training and this eval (the
shared working tree moved 7 commits, `a35534b..0c7dae3`, none touching `baselines/genesis_can_env.py`,
`baselines/rl/full_env.py`, or `baselines/stage_predicates.py` — `git log --oneline a35534b..0c7dae3 -- <those
3 files>` returns nothing). `entries_pinned: true`, `pin_stats.n_enumerated=15, n_restored_match=15` on hold15
(rnd30 uses `rnd` ICs, not pinned entries, as expected).

| cell (checkpoint: step 617640, `dv3e2e_ramp_eval_m500k`, proven bit-identical to "current latest.pt") | n | `home` (success_key) | picked | tipped | timeout | mean steps | metrics.json |
|---|---|---|---|---|---|---|---|
| hold15 sample | 15 | 0/15 = **0.00** | 0/15 = **0.00** | 0/15 (0.00) | 15/15 (**1.00**) | 300 | `fresh_eval_hold15_sample/metrics.json` |
| rnd30 sample | 30 | 0/30 = **0.00** | 0/30 = **0.00** | 6/30 (0.20) | 24/30 (**0.80**) | 240.8 | `fresh_eval_rnd30_sample/metrics.json` |

rnd30 detail (`headline_stages` all-zero except reported; `outcomes_honest {"nested_honest":0,"proxy_only":0,
"tipped":6,"timeout":24}`; `slide_fails {"not_picked":30}` — every one of the 6 tips is a `not_picked` tip, i.e.
the can was knocked over before ever being grasped, exactly like the four short `tipped(2)`/`tipped(3)`/
`tipped(8)`/`tipped(8)` episodes seen in the training log's tail). The 6 tips all occurred in **1-8 steps**
(console: `ep2 tipped(2) ep5 tipped(2) ep8 tipped(8) ep19 tipped(3) ep25 tipped(2) ep26 tipped(8)`), i.e. an early
spasm during the approach, not a failed-but-purposeful reach — consistent with the "near-pure inertia,
occasional micro-collision, never a completed pick" character already read off the training log's final windows
(§1b/1c).

**hold15 is a clean, total confirmation that the collapse is in the policy weights, not an artifact of which
training episodes got logged.** This is the SAME 15-IC hold set on which `dv3pick_dHfull_pick_local_rlDreamer_s0`
(the working comparison run) scores **15/15 picked, 0 tipped, 0 timeout, mean 14.2 steps**
(`dv3pick_dHfull_pick_local_rlDreamer_s0/fresh_eval_hold15_sample/metrics.json`, read directly: `{'picked': 1.0,
'tipped': 0.0, 'timeout': 0.0, 'mean_steps': 14.2}`) — same demo ICs, same action space, same `act_entropy`
(`tip_guard` itself differs: `grip`/1-frame-sustain on the pick run vs. `not_in_hand`/4-frame-sustain here, both
under the same `tilt>60deg` rule — a run-config difference, not something this eval controlled for). The
full-scope checkpoint at step 617640 picks **zero** of them, every episode running the full
300-step horizon with no can contact event severe enough to register as `tipped` either (`slide_fails:
{"not_picked": 15}` on every episode; `outcomes_honest: {"nested_honest":0,"proxy_only":0,"tipped":0,
"timeout":15}`). The pick-run's own rnd30 (sampled) for reference: **picked 0.600, tipped 0.333, timeout 0.067,
mean 40.9 steps** (`{'picked': 0.6, 'tipped': 0.333, 'timeout': 0.067, 'mean_steps': 40.87}`).

## 3. Diagnosis

**The checkpoint eval closes the main open question: this is a real policy failure, not a training-log
artifact.** On hold15 — the identical 15-IC set on which the architecturally-identical pick-scope run gets
15/15 — this checkpoint gets 0/15, every episode running out the clock with no event at all
(`slide_fails: {"not_picked": 15}` on all 15). On rnd30 it is 0/30 picked, 80% pure timeout and 20% a 1-8-step
tip with no pick — i.e. even the "actively attempting and failing" mode from earlier in training (tip rates up
to 0.42 in the §1 windows) is mostly gone at this specific checkpoint; what remains is closer to a fixed,
near-motionless posture that occasionally gets bumped into a tip by residual noise rather than a directed reach.
This matches the training-log signature of the flagged dry spell (§1b: tip rate 0.024, timeout rate 0.976) far
better than it matches the run's earlier, more active windows (tip rate up to 0.42) — the checkpoint (step
617640) sits chronologically inside that dry spell, and its eval behaviour is a clean sample of it.

**Two run-specific defects, not shared with the working pick-scope run, are supported by the data and both point
the same direction — a critic/world-model calibration failure driven by a return ceiling far above anything the
policy has ever earned:**

1. **`train/con` (imagined-rollout continuation probability) is pinned at 0.998-1.000 for the entire run**
   (§1c), independent of the REAL tip rate, which itself ranges 0.024-0.440 across the same windows. The working
   pick run's `con` correctly tracks DOWN to 0.78-0.91 as real terminations become common. The full-scope world
   model's imagined rollouts behave as though every episode reaches full length, always — it never learns that a
   substantial fraction of real rollouts end early via `tipped`.
2. **`train/val` (critic value) sits chronically at 2.6-8.3 across the whole run** against `return_clamp=9.0` —
   20-100x the actual mean episode score in the same windows (0.02-0.12) and far above the only return this
   policy has EVER realised end-to-end (1.0, for `picked` alone; `placed_v2` has never fired once in 554+
   episodes, so the ladder's nominal 9.0 ceiling has never been within an order of magnitude of anything
   achieved). The pick run's own `return_clamp=1.0` matches its ladder's actual scope, and its `val` settles at a
   realistic 0.5-0.8, tracking its ~50-60% success rate.

**Mechanism consistent with both: an actor trained against imagined rollouts that (a) almost never predict early
termination and (b) are bootstrapped toward a value ceiling ~9x anything ever observed, receives a policy-gradient
signal (`train/adv`, |mean| < 0.045 throughout, §1c) that is small and only weakly informative about whether
picking-and-risking-a-tip is better than standing still — both look like "keep imagining a hopeful future" under
a critic that has not been disciplined by real termination statistics. With `act_entropy=3e-05` (a shared,
already-collapsed setting, see below) providing little corrective exploration pressure once the policy drifts,
a stochastic drift into a low-activity local optimum during a marginally-stable run (recurring 20-48 episode dry
spells since episode 0, §1b) is plausible and would not require any single triggering event.**

**Demo eviction (§1c) is a real, precisely-timed coincidence — first eviction at raw counter ~588218, complete by
~617640 (exactly the evaluated checkpoint's own step) — bracketing the run's last confirmed pick (step 604252)
and the onset of the near-total-timeout stretch (604364 on). But it cannot be assigned sole or even primary
causal weight: the SAME mechanism (buffer `max_size=500000` FIFO-evicting a << 500000-transition demo prefill)
fires at a closely comparable absolute step in the WORKING pick run (full eviction ~562256) with no comparable
behavioural change there — the pick run continues to consolidate for another ~440k steps past that point.** If
eviction contributes here, it does so by removing a stabilising anchor from a policy whose stability was already
only marginal (recurring dry spells since the very first 40 episodes), not by being a novel destabiliser on its
own; this is not something the current data can adjudicate further without either (a) a rerun that changes only
the buffer size/eviction schedule, or (b) waiting for this run to pass its own eviction-adjacent window a second
time (it has already partially reactivated once, at step 654215, since the flagged dry spell).

## 4. What the numbers do NOT support

- **"Actor entropy collapsed and killed the policy."** Entropy collapses to the same -5 to -6 band in BOTH runs,
  is complete by step ~150-170k (>400k steps before the flagged dry spell), and is *higher* (less negative,
  -5.35) in the final active window (601698-654756) than in several earlier, more productive windows (e.g.
  -6.05 at 313076-359988, which had picked=0.100, tipped=0.380). Entropy magnitude does not discriminate active
  from passive periods within this run, nor this run from the working one.
- **"The advantage signal vanished right before the collapse."** `train/adv`/`adv_std` are small and noisy in
  EVERY window of the run, including the most active ones (e.g. -0.0016 mean at 413379-456285, tipped=0.420).
  There is no visible pre-collapse decay in this series to point to.
- **"The policy regressed from a higher stage it once reached."** `placed_v2`/`farside`/`slide_event`/`home` are
  0/554 in EVERY window of training history and 0/45 in both eval cells. There is no later stage to have fallen
  from; the entire run has only ever oscillated between `picked` and `tipped`/`timeout` at the FIRST rung.
- **"The run has permanently died."** Episode 547 (step 654215, after the snapshot's flagged spell) is a
  successful pick (`picked=1, tipped=1, score=1.0`), followed by further short tip events through step 657261 —
  i.e. some renewed activity, not a hard flatline, as of the last read. Given the run's history of comparable
  20-48-episode dry spells that DID recover (§1b, ranks 1/2/4/5), declaring this one terminal would be an
  overclaim the timeline does not support on its own; the checkpoint eval shows the step-617640 snapshot is bad,
  not that every later step will be.
- **A GPU/optimizer/infra fault.** `grad_scale` fluctuates without a runaway trend, `fps` never drops, one
  continuous process (no restart), zero real NaN/Inf/Traceback events (the sole "nan" grep hit is a substring of
  "provenance"). The console log gives no evidence of a mechanical failure underlying the behavioural one.

## 5. Recommendation (not started — no config change, retrain, or job was started for this task)

**Try lowering `env.return_clamp`/`model.return_clamp` to something close to the empirically achievable ceiling
for this curriculum stage (e.g. ~1.0-2.0, matching what a policy that only reaches `picked`/`placed_v2` could
actually earn) rather than the ladder's full nominal chain value (9.0), and/or oversample terminal (tip) episodes
when constructing training sequences so the continuation head (`train/con`) is exposed to enough real termination
examples to learn `P(done)` away from ~1.0.** This is the lever the data in §3 most directly implicates. The two
runs differ in several respects at once (`scope`/`ladder` full vs. pick, `tip_guard` `not_in_hand`/4f vs.
`grip`/1f, and `return_clamp` 9.0 vs. 1.0), so this is not a clean single-variable contrast — but of those,
`return_clamp` is the one whose value this run's own data (§1c) shows tracking directly with the `con`/`val`
miscalibration (a ceiling 9x anything ever earned, vs. a ceiling that matched the pick run's actual achievable
return), whereas entropy shows an identical collapse pattern in both runs and advantage magnitude shows no
pre-collapse signature in either. **The task's two suggested alternatives are NOT what
this data points to**: an `act_entropy` change is not supported since the entropy trajectory here is
indistinguishable from the working run's; a `model.rep_loss` swap (dreamer -> r2dreamer's own rep loss) has no
comparison run in this dataset to implicate it one way or the other — it remains a plausible thing to try if the
clamp/curriculum fix does not resolve the `con` miscalibration, but nothing measured here singles it out over the
clamp explanation.
