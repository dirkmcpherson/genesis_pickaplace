# E2E training problems, 2026-09-05 → 09-10: the recurring classes, and one unreconciled result

Handoff note for the next agent. Read `paper/E2E_AUDIT_BRIEF_2026-09-10.md` for the evidence and
commands; this note groups the week's problems by class and flags one contradiction the brief
does not resolve. §1–§5 come from repo history; §0 was measured on the cluster from existing
eval cells (no new evaluation run).

## 0. FIRST: an unreconciled within-learner result that may overturn the slide conclusion

The brief (§4b, §8 item 5) concludes that the slide is limited by the WORLD, not by the incentive,
because {r2dreamer} was paid +4 for `slide_success` and produced 0 in 390 episodes, while {RLPD}
was paid 0 and produced 7. That argument is a CROSS-learner comparison, and the brief itself
(§2) says cross-learner comparisons are confounded by the ladder split.

The within-learner evidence points the other way. **Measured on the cluster 2026-09-10 evening**
from the existing `fresh_eval_rnd30_mode/metrics.json` cells (no new evaluation). Old-ladder runs:
`$W/long_run_candidate_2026-09-08/runs/r2long_{h,m}_*`. (x) runs: `$W/runs/full_r2d_state_*_rx_s9*`
(the trained seeds only; the rest are still training). Both batches use the same evaluator:
eval-fixes `j`, identical `stage_notes`, `slide_success` present in 30/30 per-episode records, and
no None-valued stages. The zeros are real, not missing keys.

| {r2dreamer}, e2e, rnd30, mode | seeds | eps | picked | placed_v2 | contact | contact_push | nested_proxy | **nested_honest** | **slide** |
|---|---|---|---|---|---|---|---|---|---|
| OLD ladder, 4.1M, human | 4 | 120 | 0.650 | 0.250 | 0.517 | 0.175 | 0.242 | **0.167 (20)** | **12** |
| OLD ladder, 4.1M, machine (best-of-3 set) | 4 | 120 | 0.583 | 0.150 | 0.408 | 0.167 | 0.133 | **0.075 (9)** | **6** |
| (x) ladder, 4M, human | 6 | 180 | 0.578 | 0.117 | 0.450 | 0.444 | 0.044 | **0.017 (3)** | **0** |
| (x) ladder, 4M, machine (first-attempt set) | 7 | 210 | 0.576 | 0.095 | 0.438 | 0.419 | 0.029 | **0.010 (2)** | **0** |

Per seed, human arm, OLD versus (x): nested_honest {0.40, 0.03, 0.03, 0.20} versus
{0.03, 0, 0, 0.07, 0, 0}, with exact one-sided permutation p = 10/210 = 0.048. Slides
{8, 0, 1, 3} versus all zeros, with p = 7/210 = 0.033. The human trajectories are identical across
the two ladders (`_rx` changes only the reward column), and so are the learner, the budget and the
evaluator. The machine rows agree in direction, but the machine SET also changed (best-of-3 →
first attempt), so use the human row.

**Mechanism, verified in the code (gp_root, the tree the (x) batch ran).** My first guess
("`contact_push` is unreachable") was wrong. The verified chain is worse:

1. `contact_push` needs no release (`genesis_can_env.py:311-326`: picked ∧ can↔goal contact ∧
   tool on the far side ∧ no gripper↔goal contact). Under (x) it fires in 0.444 of episodes, in
   80 of the 81 episodes that reach `contact`. The policy earns picked 1 + contact_push 2 by
   pressing the HELD can against the goal, then runs to the horizon (s900: 22/30 `timeout`).
2. **The +4 `slide_success` rung cannot be paid during training.** Full scope terminates on the
   nested proxy — `terminated = bool(info.get('nested'))` (`full_env.py:978`), proxy = contact ∧
   grip cmd < 0.3 ∧ both cans upright (`full_env.py:699-704`), checked every env frame (the
   adapter repeats the action 4× and breaks on `terminated`, `envs/genesis.py:289-301`).
   In-episode `slide_success` needs picked ∧ contact ∧ grip < 0.3 ∧ in footprint ∧ upright on
   **12 consecutive frames** (`SLIDE_SUSTAIN = 3 × 4`). The proxy's conditions are a subset of
   the slide's, so the proxy ends the episode on the FIRST frame of the slide window. The
   'settle' route runs in `end_of_episode()`, which the adapter calls AFTER the terminal reward
   is taken, logging only (`envs/genesis.py:325-345`); `FullTaskEnv` gets `max_steps=1e9`, so the
   env's own done branch never runs. The reward loop (`full_env.py:705`) never sees
   `slide_success=True`. Confirmed by the old-ladder eval cells: every slide there has
   `slide_route='settle'`, none `'sustained'`.
   So the (x) ladder in training was **picked 1 / placed_v2 1 / contact_push 2, max 4**, and the
   episode ends, unpaid, on the event the old ladder paid +4 for.
3. **Demo buffer and env disagree.** The `_rx` prefill pays the +4 row in 13 of 74 human tapes
   (2 tapes carry a 6.0 row). The online env can never pay it. Amendment (y) was written to
   remove exactly this mismatch.
4. **Registered ≠ implemented.** Amendment (x) registers `slide_success` as three state
   conditions with no gripper term. The env's `slide_success` (both in training and in every
   eval cell) is still the (l) predicate with `grip < 0.3`, which (p) withdrew. The (x) classifier
   exists only in `can_pos_recovery/slide_predicate.py` and scores tapes, not episodes.

The old ladder paid `nested_proxy` (+4, terminal), which needs grip < 0.3 with the can touching
the goal. That forced a release at the goal, and honest nests and slides followed as a side
effect (old s0: 12 nests, 13 tips, 4 timeouts). Under (x) `nested_proxy` falls from 0.242 to
0.044: the behaviour changed, not only the episode length.

**Consequence.** The (x) ladder made {r2dreamer} e2e WORSE at every stage the paper cares about,
and the cause is structural, not a tuning problem. "Slide is world-limited" (brief §4b, §8
item 5) cannot stand: the same learner on the same human demos slides 12/120 under the old
ladder. Any redesign must (a) pay the top rung INSIDE the episode, (b) never terminate on an
unpaid event, (c) make the push rung require a release first, and (d) use one `slide_success`
definition in the env, the demo relabel and the evaluator.

**What is the same across the two batches (verified).** Old runs: `release_v4` (isolated
release, manifest with `release_sha256`). (x) runs: `gp_root` + `r2dreamer_fix`. Resolved config
of the old run matches the (x) launcher recipe: `bounded_normal`, `act_entropy 3e-5`,
`return_clamp 8`, `env_num 6`, `train_ratio 512`, 500k FIFO buffer, 4M steps, `delta_joint`
0.025, horizon 333; `genesis_full_state.yaml`, `eval_genesis.py` and `dreamer.py` are
byte-identical. Code drift: `genesis_can_env.py` (121 diff lines) is logging plus
`end_of_episode()` replacing `_nested()` with the same step budget; `full_env.py` (178) is the
gate, `placed_v2`/`contact_push` bookkeeping in full scope, phase scopes and a shelf assertion.
Nothing in the drift changes full-scope dynamics or the old ladder's payments.
**Correction (Lane 3 audit, 2026-09-11):** the budgets were NOT identical. `release_v4` counts
the budget as ONLINE steps (`R2_LONG_RUN=1`: 4,000,000 online, counter target 4.117M human /
4.150M machine); `r2dreamer_fix` counts TOTAL steps, so the (x) runs got 3,970,594 (human) /
3,962,512 (machine) online. The gap is 0.74 % / 0.94 % — far too small to carry the effect above,
but "same budget" was overstated. Also: the (x) batch has no 2M milestone checkpoints, and
`release_v4/gp` never computed `placed_v2` in full scope, so the old runs' `placed_v2` column
comes from the post-hoc re-score against `gp_root`, not from the tree they trained in. And the
09-09 `contact_push` guard in `full_env.py:723-734` is a no-op today (`genesis_can_env.py:336`
already writes `info['contact_push']` in both live trees), so the cause of the earlier
`contact_push = 0.000` reading is NOT established by that hunk. Full detail:
`paper/TREE_RECONCILIATION_2026-09-10.md`.

Remaining caveats: n = 4 v 6 seeds; old-ladder seed variance is large (two of four old seeds sit
at 0.03); the (x) batch is only 13 of 32 seeds trained — recompute when the rest land.

## 1. The objective was not the thing we scored

- The e2e reward paid `nested_proxy` / bare `contact` while the scorer moved to `slide_success` /
  `nested_honest` (commit 5f7b4d9). Drift, not design.
- `nested_proxy` precision is 0.114 (human) v 0.029 (machine) and REVERSES the arm ordering
  (brief §4a). {RLPD} trained on exactly this proxy. **Mechanism (Lane 1, 2026-09-11): its
  `contact` term is sticky** — machine-arm firings happen with the can a median 149 mm from the
  goal (0/15 touching), after an early touch, a carry-away and a set-down with the gripper
  opened. It also misses 12 of 14 real nests on the human tapes. Replaced by `nested_v2`.
- The two learners got different ladders because `FULLENV_REWARD_X=1` was inert in `gp_e2e`
  (brief §2).
- Predicates with names that do not mean what they say: `timeout` is a residual label; legacy
  `placed` never fires; `contact`'s ee clause was vacuous (ee = wrist); the (l) slide grip clause
  passed 2/74 demos and was withdrawn (p); `placed_v2` fires at reset on 4 of the 30 rnd30 ICs
  (brief §3). Open question: do TRAINING resets also start inside the shelf footprint and pay +1
  for free?

## 2. The budget was never a convergence criterion

- 2M was "2x the single-phase budget", with no stopping rule (a6f128c, 39790de). The arms ignite
  at different times, so a fixed budget gives them unequal post-ignition training. The direction
  goes against the machine arm.
- 2M → 4.1M multiplied honest completion ~9x (§5.6). The 4M budget is also a heuristic.
  Register a stopping rule (e.g., a plateau on the final-quarter slope) before the next batch.

## 3. The arms were not built the same way

- Best-of-3 selection drove most of the machine arm's advantage (Σ reward 131 → 206). Fixed with
  the first-attempt set (v).
- Episode length: machine median 601 (the cap) v human 389; idle fraction 0.007 v 0.365.
- DP and RLPD read the same sets in opposite directions (DP −0.258, RLPD +0.048; cc6d38b). The 10
  no-pick human tapes are the probable cause: an imitator copies them, an online learner explores
  past them.
- Smaller items: the `one_per_ic_first` manifest stamp was false (fixed b756259); the tape-reward
  validator was pick-scope only (y); `R2D_EOE` did not exist when the human arm of record trained;
  `dHfull_w3` and the census are different recordings (24/74 action streams differ).

## 4. Infrastructure made fixes unreachable or failures invisible

- Three code trees (`gp_e2e`, `gp_root`, `genesis_pickaplace`) diverged in both directions. An env
  var gate is silently inert in a tree that lacks it. **No run stamps the code it loaded.**
- Slurm spools batch scripts at submission. A launcher fix does not reach queued jobs.
- `-J` renames broke two log lookups: a post-train check died after a full 12 h run, and
  `full_eval_sweep.sh` skipped every renamed run and reported success.
- The requeue handler sat behind an existence guard that exited first. Preemption killed 5
  {r2dreamer} seeds (brief §8, defect 4). Jobs queued before the fix still carry the old guard.
- Monitors: a malformed `sacct` call reported "zero failures" three times; a health check globbed
  r2dreamer paths only, so it passed {RLPD} vacuously; the checker lived in per-node `/tmp`.
- The 09-07 disk-full incident killed every job on the cluster.

## 5. The recurring reasoning error

Six times this week an ABSENT value was read as a confident zero, or a code-reading inference was
reported before a built environment was probed: the "hollow" {RLPD} record (16 runs relaunched,
then disproven); "slide never observed" (twice); "zero failures" (three times); the `contact_push`
"missing field" (a real zero, §5.6 correction); a figure labelled {r2dreamer} that held {RLPD}
data. Three times, code written to handle a case was made unreachable by an earlier line.

**Rule for the next agent:** before reporting a zero, show the per-episode test that could have
produced a non-zero. Before trusting a gate, print what the loaded code resolved (for example the
`STAGE_REWARD` dict and the sha256 of `full_env.py`) into the run directory.

## 6. Suggested order of work

1. §0 is measured (4 v 6 seeds). Recompute it when the remaining (x) seeds land. Do not start a
   ladder redesign or the scripted-push probe until the "rung-3 plateau" reading is accepted or
   refuted. A per-episode look at the (x) videos (does the policy hold and press?) is the cheap
   confirmation.
2. Provenance guard: every run writes the resolved ladder and the `full_env.py` hash; the launcher
   refuses if `FULLENV_REWARD_X` is set and the tree has no gate. Then collapse to one tree.
3. Register a convergence rule before any new long run.
4. Only then decide the ladder redesign, the {RLPD} retrain under (x), and the golden set (user
   decisions, brief §8).
