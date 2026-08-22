# Adversarial implementation audit — RLPD / SACfD encoders / full_env / eval protocols
Auditor: independent read of the code only (sparse checkout: .py/.md/.sh/.json; no npz/data).
"CONFIRMED" = traced in source. "SUSPECTED" = mechanism traced but a value/file outside this
checkout is needed to close it.

---

## 1. RLPD trainer (train_rlpd.py, rlpd_sac.py)

### F1. CONFIRMED — CRITICAL — demo encoder has NO env-terminal guard whatsoever
`baselines/rl/train_sacfd_full.py:43-92` (`relabel_full`) and `:273-292`
(`delta_encode_transitions`, the encoder train_rlpd actually uses at
`train_rlpd.py:253-274`).

* `relabel_full` sets `done=True` **only** at the `nested` frame (`:87`). For a
  `stage='no-pick'` tape (`rank == 0`, `:53`) the reward array is all-zero and the done
  array is all-False, and `end = n` (`:89`) — **the entire tape is emitted, every
  transition `r=0, done=False`, including the last one.**
* `delta_encode_transitions` (`:286-288`) then flips `done=True` only on frames whose
  reward already `>= STAGE_REWARD['picked']`. A fail tape has none, so nothing is
  terminal and nothing is dropped.
* There is **no tip check, no `max_steps` check, no end-of-tape truncation flag**.
  `full_env.py:530-545` terminates the online env when `grip_cmd < GRIP_OPEN(0.3)` and
  `tilt > TIP_DEG(60)`; `full_env.py:546` truncates at `max_steps` (900 for training).
  Neither predicate is consulted anywhere in the encoders.
* The same file already contains the correct pattern — `relabel_hold_region`
  (`:187-211`) computes `end = j_term + 1` and **drops every later frame**, with the
  explicit comment "so the critic never bootstraps through a state the env would not
  have continued from". The pick-scope path does not use it.

Failure scenario (matches the observed dDP result): m1all_harvest contributes 30 fail
tapes × 1200 frames ≈ 36k transitions (51% of the demo buffer per the 08-22 doc), all
`r=0, done=False`, ~8 of them containing long post-tip chains. In `rlpd_sac.py:271`
`target_q = rewards + (1-dones)*gamma*next_q` these bootstrap forever; the chain is a
closed sub-MDP with spectral radius `gamma=0.998` (loop gain ≈ 1 over 500 steps, and the
chain is 1200 long). `rlpd_sac.py:284-295` runs the actor loss over the **mixed** batch
(`obs = cat(online, demo)`), so the actor is trained to maximise the ensemble-mean Q at
exactly those off-manifold states, and `rlpd_sac.py:257-260` evaluates the target at
`pi(s')` there. Online data can never correct it because the env terminates at those
states. With UTD 10 × 128 demo samples/step × 100k steps ≈ 1.3e8 demo-sample gradient
applications on a 70k-transition set (~1800 epochs) and **no conservatism term** (no CQL,
no BC loss, min-of-2-of-10 is weak pessimism when all members extrapolate the same way),
divergence is the expected outcome, not a surprise.
**dDP-vs-dH relevance: direct and primary.** dH tapes are pick-phase-truncated
(≈2 frames past the lift) and 33% no-pick; dR2D is 0% no-pick. The exposure ordering
0 / 33% / 51% matches the ignition ordering exactly.

### F2. CONFIRMED — HIGH — demo "pick" terminal uses a weaker predicate than the env, one frame early
`train_sacfd_full.py:58`:
```python
picked_f = (can_z[:-1] > pick_z) & (grip[:-1] > pick_env.GRIP_CLOSED_FRAC)
```
The env's `picked` (`baselines/genesis_can_env.py:258-265`) additionally requires
`|eef - can| < PICK_EEF_DIST (0.20)` **sustained `PICK_SUSTAIN = 10` consecutive frames**.
Two mismatches:
1. **No sustain / no eef-distance.** The demo grants `+1` and `done=True` up to ~10 frames
   before the env would, i.e. at a state where the online env pays 0 and keeps running.
   The anti-whack guard the env exists to enforce is absent from the demo labels.
2. **Frame offset.** The env pays step *i*'s reward from the state *reached*
   (`s[i+1]`); `relabel_full` evaluates the proxy at `s[i]`. `relabel_hold_region`
   documents this deviation explicitly (`:135-140`, "relabel_full … one frame late
   relative to the env") and fixes it for the hold path only. So the RLPD demo buffer
   attributes the `+1` to the wrong `(obs, action)` pair, and the transition that
   actually crosses `pick_z` is labelled `r=0, done=False`.

Failure scenario: the critic learns `Q(s_already_lifted, a) = 1` and `Q(s_lifting, a) = gamma·V`,
i.e. the reward is anchored one step downstream of the action that earns it. On a
`+1`-terminal-only sparse task this is the only reward signal in the buffer.
**dDP-vs-dH relevance: affects all RLfD arms roughly equally**, so it does not explain
the dDP null — but it biases every RLPD number in the paper and interacts with F3.

### F3. CONFIRMED — MEDIUM — post-terminal transitions are kept, not dropped
`train_sacfd_full.py:286-291` / `:363-370`: the scope='pick' pass sets `done=True` on the
pick frame but leaves every later transition of the same tape in the list
(`out.extend(... for i, ... in enumerate(trans))` over the full `trans`). The comment at
`:485-489` claims the opposite intent ("the pick grant ENDS the episode … or the critic
bootstraps through a state the env never continues from") — the code only half-does it.
Impact per success tape is small (dH pick-phase tapes are cut ~2 frames past the proxy
lift; m1all_harvest is cut 10 frames past it, so ≈9 extra `r=0, done=False` post-pick
transitions per tape), but combined with F2 it means the demo buffer teaches
"post-lift states are worth `gamma·V`, not 1".
**dDP relevance: mildly asymmetric** (10-frame vs 2-frame tails), second-order next to F1.

### F4. CONFIRMED (design) — MEDIUM — actor loss runs over demo states; matches the RLPD reference, but the reference's assumption is violated here
`rlpd_sac.py:284-295`. This is faithful to Ball et al. (the mixed batch drives both critic
and actor). The published method's implicit precondition is that demo states lie where
online data soon goes. The 08-22 measurements (fail-frame NN-distance to any success frame
p50 2.66 std / p90 42 std; 94% of dDP fail frames beyond the success set's own 99th-pct
self-distance) say that precondition is false for dDP. Not a bug to fix in isolation —
but it is the amplifier that converts F1 into divergence, and it is the reason the paper
should describe the dDP result as *demo-set × algorithm-assumption*, not "model demos are
worse".

### F5. CONFIRMED — LOW — the discriminator arm `dDPtiptrunc` does not restore the terminal
`baselines/make_dDPsucc.py:52-58, 99-107` cuts fail tapes at the first frame where the env's
tip rule would fire, using the env's exact predicate (grip<0.3 ∧ tilt>60) — good. But the
truncated tape then goes through `relabel_full`, which emits its last transition with
`done=False` (F1). The env *terminates* there (`full_env.py:544`), so the correct label is
`done=True, r=0` (target exactly 0). As built, `dDPtiptrunc` shortens the off-manifold
chain but leaves a dangling bootstrap at its end. Add a `done=True` at the cut frame or the
experiment under-tests its own hypothesis.
Also `make_dDPsucc.tilt_deg` (`:44-50`) is a **re-implementation** of `full_env.tilt_deg`
(norm-normalised vs `1-2(x²+y²)`); identical for unit quats, but this repo's own rule is
"both sides CALL THIS FUNCTION" (`full_env.py:60-67`).

### F6. CONFIRMED — LOW/NOT-A-BUG (verified clean)
* Online truncation is handled: SB3 2.8 `ReplayBuffer(handle_timeout_termination=True)`
  (default; `make_rlpd` passes no `replay_buffer_kwargs`) + `DummyVecEnv` setting
  `TimeLimit.truncated`. So online timeouts bootstrap correctly. The demo side is the
  only unguarded path.
* `backup_entropy=False` (`rlpd_sac.py:269`), min-of-Z on **target** critics only
  (`:258-260`), sum-of-E critic loss (`:275-276`), single actor+alpha update on the last
  UTD step (`:283`), polyak every grad step (`:303`) — all match RLPD/SB3 semantics.
* `DemoData._g` is per-seed (`:182`); `demo_batch=128/256` exactly 50/50.
* `q_watchdog` only prints; nothing clips or intervenes (`:318-324`). Given four dDP seeds
  reached Q 12k–26k, the watchdog fired ~10 times and changed nothing — expected, but it
  means "the watchdog didn't stop it" is not evidence of health.

### F7. CONFIRMED — MEDIUM — training horizon 900 is a silent, unrecorded default
`cluster/sbatch_rlpd.sh:197-203` never passes `--train-max-steps`, so
`train_rlpd.py:109` supplies 900. It is **not** in the sidecar (`train_rlpd.py:300-310`)
and **not** in `REG_KNOBS` (`sbatch_rlpd.sh:165-168`). Same for `--eval-max-steps 400`
and `--eval-freq`. Two runs differing only in training horizon would collide on the
registry key. This is exactly the silent-default family the repo audits for.

---

## 2. Shaping (full_env.py pick_shaping)

### F8. SUSPECTED — HIGH — r2dreamer's shaping gamma (0.999) may not match its own discount (0.997)
`cluster/r2dreamer_shaping_boundary_fix.patch` hard-codes `reward += 0.999*phi - phi_prev`
in the r2d adapter, and every doc repeats "r2dreamer gamma 0.999"
(`paper/ROUND_ROBIN_RUNNING_2026-08-19.md:75,103`; `REVIEW_GUIDE.md:66`;
`ROUND_ROBIN_RESULTS:118`; `ROUND_ROBIN_2026-08-20.md:155`).
But the champion config shipped in `cluster/r2dreamer_dense_lever.patch`
(`configs/env/genesis_pick_v5d4c_delta_shaped.yaml`) says:
```
horizon: 333             # discount 0.997; pick at ~100 agent steps -> 0.74 visibility
```
`1 - 1/333 = 0.997`, and `0.997^100 = 0.74` — the yaml's own arithmetic is self-consistent
at **0.997**, not 0.999 (`0.999^100 = 0.905`).
If the agent's discount is 0.997, the Ng policy-invariance claim ("γ matched per agent")
is **false for the r2dreamer dense arm** — the arm carrying the headline "dense dH 4/4,
p=0.007". With γ_shape > γ_agent the shaping is net-positive for hovering: at
reward_scale 100, a stationary agent at distance d earns `100·(1-0.999)·2d = 0.2d` per
agent step, worth `0.2d/(1-0.997) ≈ 67d` discounted vs a `100·0.997^T` terminal — at
d≈0.3 that is ≈20 vs ≈74, so completion still wins, but the reward is no longer
policy-invariant and the "dense reward changed only exploration" framing is not
supported.
**Closing action (one command on the cluster/r2dreamer checkout):** confirm whether
`discount` is `1-1/horizon` with `horizon: 333`, or an independent key set to 0.999. If
it is 0.997, the caption must drop the invariance claim for the r2d dense row.
RLPD (0.998 / `PICK_SHAPING_GAMMA = 0.998`, `full_env.py:129`) and dv3 (0.997/0.997) are
self-consistent; only r2d is in question.

### F9. CONFIRMED — MEDIUM — φ is not zeroed at the terminal state
`full_env.py:403-414`. Exact potential-shaping invariance in an episodic MDP requires
`φ(absorbing) = 0`; here the terminal transition receives `γ·φ(s_T) - φ(s_{T-1})` with
`φ(s_T) = -2‖eef-can‖ ≠ 0`. Effect: the pick terminal is worth `1 + γ·φ(s_T) ≈ 1 - 2·d_T`
instead of 1. At a genuine grasp `d_T` is small (a few cm) so the distortion is a few
percent; but the sign is *against* terminating, and it would become severe for any
"pick" achieved at large eef-can distance. The same applies to r2d/dv3 (scaled ×100).
LOW-MEDIUM in magnitude; it invalidates the literal "exactly policy-invariant" wording.

### F10. CONFIRMED (already registered as a caveat) — MEDIUM — demo rewards stay sparse while online rewards are shaped
`train_rlpd.py:74-79` ("Demos are NOT relabeled") + `rlpd_sac.py:249` concatenating
sparse-labelled demo rewards with shaped online rewards into one Bellman regression. The
critic's fixed point is the shaped value function `V_shaped(s) = V(s) - φ(s)`, but the demo
half of every batch regresses toward `r_sparse + γ V_shaped(s')` — a systematic
per-transition target error of `φ(s) - γφ(s')`, and a per-state value offset of up to
`|φ| ≈ 2d ≈ 1.0` in the same units as the entire task reward. The docs list this as an
open asymmetry; the magnitude (O(1) in Q units, i.e. 100% of max return) deserves to be
stated numerically in the caption rather than as "known asymmetry".
**dDP relevance: none** (dense ran only on dH).

### F11. CONFIRMED — clean — eval never sees shaping
`baselines/wandb_eval.py:112` builds a bare `GenesisCanEnv`, never `FullTaskEnv`, so
`pick_shaping` cannot leak; `pick_shaping` is a `FullTaskEnv`-only constructor arg, and
`full_env.py:197` asserts scope=='pick'. The in-train `VideoEvalCallback` likewise spawns
`wandb_eval.py`. Verified clean.

### F12. CONFIRMED — the shaping hover math in the comment is computed for the wrong horizon
`full_env.py:120-125` justifies SCALE=2 with "over any 400-step episode". Training episodes
are 900 sim steps (`train_rlpd.py:109`, never overridden — F7). Undiscounted, a hovering
agent accrues `900 × (1-γ)·SCALE·d = 900·0.004·d = 3.6d`, which exceeds the +1 terminal for
`d > 0.28 m`. The *discounted* comparison still favours picking (telescoping gives
hover = `-φ_0 = 2d` vs pick = `γ^T(1+γφ_T) - φ_0`, so picking wins for `2d_T < 1`), so this
is a comment error rather than a live pathology — but a reader checking the registered
math against the run config will find the horizon does not match. LOW.

---

## 3. Eval protocols (wandb_eval.py, sbatch_rlpd.sh, sbatch_dp.sh)

### F13. CONFIRMED — HIGH — DP and RLPD are NOT evaluated under the same protocol, contrary to the docs
`cluster/sbatch_dp.sh:384-386`:
```
python baselines/wandb_eval.py --kind dp --ic-mode both --checkpoint "$CKPT" --random "$EVAL_EPS" --seed 0 ...
```
No `--max-steps` ⇒ `wandb_eval.py:30` default **1200**. RLPD's sweep
(`sbatch_rlpd.sh:221-231`) passes `--max-steps 400`.
Also: DP runs all **30 episodes in ONE process**, while RLPD runs **one fresh process per
episode**. `REVIEW_GUIDE.md` and `ROUND_ROBIN_RUNNING` describe both as "fresh process per
episode".
Consequences:
* DP policies get a 3× longer horizon to achieve the pick than RLPD policies. Given the
  measured first-lift times (dH ~784-826 frames, dDP 512, dR2D ~120), a 400-step cap is
  binding for human-timed behaviour and a 1200-step cap is not. Any cross-algorithm
  statement ("dR2D_DP 0.96 is the best cell in the matrix", "RLPD ignition 0.02-0.28")
  compares numbers measured on different MDP horizons.
* The single-process DP eval reuses one Genesis world across 30 episodes — the exact
  carry-over risk the fresh-process rule was introduced to eliminate.
**Fix / disclosure:** either re-run DP with `--max-steps 400` fresh-per-episode, or state
the horizon per row in the results matrix. **dDP-vs-dH RLPD relevance: none** (both RLPD
arms use 400), but it affects headline claim #1 and #3.

### F14. CONFIRMED — MEDIUM — training horizon 900 vs eval horizon 400
`train_rlpd.py:109` (900 sim steps, terminate-on-pick) vs `sbatch_rlpd.sh:223,230`
(`--max-steps 400`) vs eval env = `GenesisCanEnv`, which has **no tip rule and no
terminate-on-pick** (`genesis_can_env.py:276-278`: `done = t >= max_steps` only).
So the eval MDP differs from the training MDP in three ways: shorter horizon, no tip
termination, no pick termination (`picked` latches, so scoring is still correct).
Direction: the horizon cut deflates every RLPD number uniformly; the missing tip
termination inflates slightly (a policy that tips the can may still pick later). Both are
constant across arms ⇒ **not a dDP-vs-dH confound**, but the reported RLPD ignition rates
are "pick within 400 sim steps", not "pick".

### F15. CONFIRMED — refutes an open question in ROUND_ROBIN_RESULTS §2
The doc asks "the dDP demo ICs are the harvest ICs; worth one look at which uids those
are." Answer from code: the demo-IC set is a **hard-coded, arm-independent list of 15
uids** (`sbatch_rlpd.sh:218`: `232 234 235 236 237 239 242 243 244 245 246 247 248 250 251`).
I verified against `can_pos_recovery/trial_placements.json`: these are exactly the first
15 `label=='success'` uids in sorted order (61 exist), i.e. identical to
`ic_sampling.demo_ics(env, reps=1)[:15]` used by the DP path. **Every RLPD arm and every
DP arm is evaluated on the same 15 demo ICs.** The odd random>demo result for dDP is not
an IC-set artifact.

### F16. CONFIRMED — MEDIUM — the 15 "random" ICs are the same 15 ICs in every RLPD run, and different from DP's 15
`sbatch_rlpd.sh:226-231` calls `--ic-mode random --random 1 --seed k` for k=0..14 ⇒
`ic_sampling.sample_support_ics(env, 1, seed=k)` = the *first* draw of `default_rng(k)`.
That is a fixed set of 15 ICs shared by every seed and every arm. Pooling 6 seeds × 15 as
90 independent Bernoulli trials (as `ROUND_ROBIN_RESULTS` does: "random 9/90 = 0.10")
understates the variance — the 15 ICs are a fixed, small, repeated design; the effective
n for a between-arm test is closer to 15 clusters, and the seeds are correlated through
the shared IC set.
Separately, DP uses `sample_support_ics(env, 15, seed=0)` = 15 draws from **one** rng ⇒ a
**different** IC set than RLPD's. The two algorithms' "random-IC" numbers are drawn from
the same distribution but not the same points.

### F17. CONFIRMED — MEDIUM — a crashed sweep episode silently shrinks the denominator
`sbatch_rlpd.sh:219-233`: each episode runs in a background subshell; `wait` with no
operands always returns 0 under bash, so `set -eo pipefail` does not catch a failed
episode. `sbatch_rlpd.sh:238-243` then counts only the json files that exist:
```python
for f in sorted(glob.glob(f'{sw}/{pref}_*.json')): ... n += 1; p += (v > 0)
```
A crashed episode (Genesis init failure, OOM under 5 concurrent worlds) vanishes from both
numerator and denominator. The `SWEEP-RESULT` line does print `d/dn`, so it is *visible* —
but only if someone checks that `dn == 15` on all 12 jobs. `sweep/demoIC` is pushed to
wandb as the string `"1/15"`, so this is auditable from wandb alone. **Action: verify every
dDP and dH-dense run has denominator 15.** If any dDP job lost episodes, its rate is biased
(dropping a would-be failure inflates; the reported 0/15-style numbers suggest not, but
check).

### F18. CONFIRMED — LOW — `DP-RESULT` uses top-level `.get()` defaults
`sbatch_dp.sh:392-399`: `m.get(f'{prefix}/n', 0)`, `m.get(f'{prefix}/picked', 0.0)`. This is
the same silent-zero pattern the REVIEW_GUIDE warns about (it *is* reading inside
`['metrics']`, so it is correct today), but a prefix change would print `indist=0/0`
rather than crash. The RLPD counter (`sbatch_rlpd.sh:241`) does it right with a bare
`['metrics']['eval/picked']`.

---

## 4. r2dreamer BEST-checkpoint protocol (sbatch_r2dreamer.sh)

### F19. CONFIRMED — MEDIUM/HIGH — the greppable `R2D-RESULT picked=` is the BEST-checkpoint MODE eval, not the final checkpoint
`sbatch_r2dreamer.sh:427`:
```
_pk=$(grep -h '"eval/picked"' "$LOGDIR/metrics.jsonl" | tail -1 | ...)
```
with the comment "picked from the LAST eval/* line in metrics.jsonl (the latest.pt seed-0
eval)". But the execution order is: final `latest.pt` eval (`:405-407`) → 3 confirmations
on `BEST_selected.pt` (`:413-416`) → 1 **mode** eval on `BEST_selected.pt` (`:417-418`) —
and **all of them pass `--append-metrics "$LOGDIR"`**. So `tail -1` is the mode eval of the
best checkpoint. Anyone reading `R2D-RESULT` as the final-checkpoint number gets the most
optimistic number in the job (the doc reports mode evals as 1.0 ×4 for the dense arm).

### F20. CONFIRMED — MEDIUM — the in-training checkpoint-lottery evals pollute the training run's `eval/picked` series
The archive loop (`:355-381`) also passes `--append-metrics "$LOGDIR"`, so every
archived-checkpoint score lands in the same `metrics.jsonl` that
`sync_runs_to_wandb.py` pushes to the *training* run. The wandb `eval/picked` curve for a
r2d training run is therefore not a function of training step — it interleaves scores of
snapshots taken at arbitrary wall-clock. This is very likely the source of the confusion
already recorded in `ROUND_ROBIN_RUNNING`: "r2d SHAPED dH s50-53: early evals already read
0.93/0.07/1.00/0.87 — checkpoint step NOT yet verified". Treat every r2d `eval/picked`
point as unattributed unless cross-referenced to `ckpt_scores.tsv`.

### F21. CONFIRMED — MEDIUM — selection and confirmation use different `--max-steps` than the final eval
* selection evals (`:371-373`): `--episodes 15 --mode sample --seed 0 --device cpu`, **no
  `--max-steps`** ⇒ `eval_genesis.py` default.
* confirmations (`:414-418`): `--episodes 15 --mode sample --seed 1|2|3`, **no
  `--max-steps`**.
* final `latest.pt` eval (`:405-407`): `--max-steps "${EVAL_MAX_STEPS:-400}"`.
So the headline confirmations and the "final-3M column" in the results table were measured
on **different horizons** unless `eval_genesis.py`'s default equals 400. `eval_genesis.py`
is not in this checkout — **verify the default before publishing the
"best 0.91 / final 0.00" contrast**, since that contrast is the whole argument for the
BEST protocol. (`time_limit: 400` in the config suggests the default is 400, but the flag
being passed in exactly one of the three call sites is the classic silent-default shape.)

### F22. CONFIRMED — the selection/confirmation independence claim is *partially* true
Selection uses `--seed 0`; confirmation uses seeds 1-3. That does make the confirmation an
independent draw over the policy's action-sampling noise. Two caveats:
* Whether the seed also redraws the **initial conditions** is decided inside
  `eval_genesis.py` (not in this checkout). If ICs are the fixed demo-IC list regardless of
  seed, the confirmation does not remove IC-luck from the max-over-~30-checkpoints
  selection, and the headline is optimistic by the IC-selection component.
* The reported statistic is by construction `max over ~30 checkpoints` — honest as
  declared ("best checkpoint"), but it is not an estimate of the agent's end-of-training
  performance and must never be compared against a single-checkpoint number from another
  algorithm (dv3's periodic eval, RLPD's final checkpoint) without saying so. The results
  table currently places `dH_R2D dense 0.90` in the same column as `dDP_RLPD 0.02`
  (RLPD final checkpoint) and `dH_DV3 0.28` (best of 2-3 periodic evals) — three different
  selection protocols in one column.
* `sort -k2 -rn | head -1` (`:409`) resolves score ties by reverse whole-line order, i.e.
  the newest timestamped filename. Harmless.
* `set +e +o pipefail` (`:404`) means a crash in all three confirmations produces a job
  that still prints "done" with no confirmation lines. Check `confirm.log` exists for each
  of s50-53 before trusting the 4/4.

---

## 5. run_registry.py — does it refuse duplicates as described?

### F23. CONFIRMED — yes for the sequential case; MEDIUM gap for the concurrent case
`cmd_check` (`:151-190`) refuses (exit 2) on a full-key match unless `DUPLICATE_OK` is set,
and warns (exit 0) on a git-only difference. That is exactly as documented.
Gaps:
1. **TOCTOU / concurrency.** `check` runs *before* training (`sbatch_rlpd.sh:183`);
   `register` runs *after* (`:205`) — hours later. Two identical jobs submitted in the
   same batch (or a resubmit while the first is still training) both see an empty registry
   and both run. The protection is against *sequential* re-execution only. Every duplicate
   in the audit that motivated this tool was sequential, so this is a real but partial
   guard; the REVIEW_GUIDE's blanket "RUN_REGISTRY refuses duplicate (config,seed,demo)
   keys" overstates it.
2. **Concurrent appends to one JSONL on a shared FS.** `:204-205` opens in `'a'` mode from
   up to 12 simultaneous jobs. `read_registry` (`:113-117`) *skips* malformed lines with a
   stderr WARN — an interleaved/torn line silently removes a run from duplicate detection
   and from the provenance record. Appends are one short line so tearing is unlikely, but
   the failure mode is silent by construction.
3. **Knobs are asserted, not observed.** `REG_KNOBS` (`sbatch_rlpd.sh:165-168`) is a
   hand-maintained mirror of the training command line. Nothing checks that they agree, and
   several real knobs are absent (F7: `train_max_steps`, `eval_max_steps`, `eval_freq`).
4. On a preemption requeue after `register` succeeded, the check is skipped
   (`sbatch_rlpd.sh:182`) and a **second** register line is appended for the same logical
   run. `sbatch_dp.sh:360` guards its register with `SLURM_RESTART_COUNT -eq 0`;
   `sbatch_rlpd.sh` does not. Minor inconsistency; makes the registry over-count RLPD runs.
5. `git_short_hash` falls back to `'unknown'` on any failure (`:70-77`), which downgrades a
   true duplicate from REFUSE to WARN (exit 0, job proceeds).

---

## Ranked top 5

1. **F1 (CRITICAL) — the RLPD demo encoder emits fail tapes whole with `done=False` and no
   env-terminal guard (no tip check, no `max_steps` check, no end-of-tape terminal).**
   `train_sacfd_full.py:43-92, 273-292`. This is the confirmed mechanism behind the dDP_RLPD
   0/6 hypothesis: 36k transitions (51% of the demo buffer) of un-anchored, off-manifold
   bootstrap targets at γ=0.998, with the actor trained to maximise Q on exactly those
   states. The correct pattern already exists in the same file (`relabel_hold_region`,
   `:187-211`). *Fix:* truncate fail tapes at the env's tip predicate **and set
   `done=True` there** (the current `dDPtiptrunc` build does the first half only — F5).

2. **F13 (HIGH) — DP and RLPD are evaluated under materially different protocols
   (max-steps 1200 vs 400; one process for 30 episodes vs fresh process per episode)**,
   while the review guide asserts they are identical. `sbatch_dp.sh:384-386` vs
   `sbatch_rlpd.sh:221-231`. Any cross-algorithm number in the results matrix (including
   "dR2D_DP is the best cell ever") carries an undisclosed 3× horizon advantage.

3. **F8 (HIGH, needs one grep to close) — r2dreamer's shaping γ is hard-coded 0.999 while
   its own champion config says `horizon: 333 # discount 0.997`.** If the discount really is
   0.997, the "γ-matched, exactly policy-invariant" claim fails precisely on the arm
   carrying the dense-reward headline (dH_R2D 4/4, p=0.007). RLPD and dv3 are self-consistent.

4. **F2 (HIGH) — the demo `picked` terminal uses a strictly weaker predicate than the env
   (no `PICK_EEF_DIST`, no `PICK_SUSTAIN=10`) and is evaluated one frame late.**
   `train_sacfd_full.py:58` vs `genesis_can_env.py:258-265`. The single reward signal in
   every RLfD demo buffer is attached to the wrong `(s,a)` and fires ~10 frames before the
   env would pay it. Affects all RLPD/SACfD numbers in the paper.

5. **F19+F21+F22 (MEDIUM/HIGH) — the r2dreamer BEST protocol has three reporting hazards:**
   `R2D-RESULT picked=` is actually the best-checkpoint *mode* eval (F19); selection and
   confirmation omit `--max-steps` while the final eval passes it (F21); and the headline
   column mixes three different selection protocols across algorithms (F22). The protocol
   itself is defensible (selection seed 0, confirmation seeds 1-3), but the artifacts it
   emits are mislabelled.

### Also worth fixing before submission (not top-5)
* F16 — the 15 random ICs are a fixed shared set; 90 pooled episodes are not 90 independent
  trials, and DP's random set is a *different* 15 points than RLPD's.
* F17 — verify every sweep denominator is 15 (a crashed episode silently drops from both
  numerator and denominator; `wait` returns 0 regardless).
* F10 — quantify the shaped/sparse demo-reward asymmetry: the target error is O(|φ|)≈1.0,
  i.e. the same order as the entire task reward.
* F23.1 — the registry cannot catch concurrently-submitted duplicates (check-before /
  register-after). Weaken the REVIEW_GUIDE wording or move `register` to job start.
* F7 — `--train-max-steps 900` is a silent default absent from both sidecar and registry key.
* F15 — settles the open question in ROUND_ROBIN_RESULTS §2: the demo-IC set is a hard-coded,
  arm-independent list of the first 15 success uids, identical for RLPD and DP.
