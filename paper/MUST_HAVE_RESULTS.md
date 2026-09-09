# Must-have results to publish (opened 2026-09-09)

*The minimum set of cells the paper cannot go without, in the user's priority order. Living document: update status in place, never delete a row.*

## Scope and terms

Three learners throughout: **Diffusion Policy** (offline imitation), **RLPD** (online RL from demonstrations), **world model** (r2dreamer). Two demonstration sources per cell: human and machine.

The user's stage split maps onto our predicates as:

| user's term | predicate | note |
|---|---|---|
| pick | `picked` | env grant |
| place | `placed_v2` | corrected set-down; the stale `placed` is never granted and must not be used |
| **can-contact** | `contact_push` | pick-can touches the goal can, tool on the far side — not bare `contact`, which counts carrying the can in (84–86 % of policy grants) |
| **can-settle** | `slide_success` (amendment x) | released, pushed, arrived settled and upright |

Every cell needs **both** a success rate and a learning curve. **4 seeds per arm** for now (8 is the target; 4 until runtime is settled) — this is underpowered and every reported null must carry its detectable-effect figure.

## 1. End-to-end (highest priority)

| stage | Diffusion Policy | RLPD | world model |
|---|---|---|---|
| pick — rate | preview cells exist | **have** (0.648 / 0.600 / 0.641) | **have** (8v8) |
| pick — curve | **impossible** (offline: no rollouts, ever) | possible — logging added to queued runs | **have** |
| place — rate | preview | needs read-out | **have** |
| place — curve | impossible | **blocked** — see (w) | **blocked** — not a reward rung |
| can-contact — rate | preview | needs read-out | **have** |
| can-contact — curve | impossible | **blocked** | **blocked** |
| can-settle — rate | preview | needs read-out | **have** |
| can-settle — curve | impossible | **blocked** | **blocked** |

**All of these are on datasets being rebuilt** (matched successful data, both sources) so the current numbers are superseded as source comparisons regardless of their status above.

## 2. Phase runs

| stage | Diffusion Policy | RLPD | world model |
|---|---|---|---|
| pick | **have** (0.878 / 0.873) | **have** (0.858 / 0.842) | **have** (0.617 / 0.608) |
| place | runs completed, table blocked on read-out | 2 of 32 read out | **have** (0.715 / 0.652) |
| can-contact | **held** | **held** | re-scored 0.581 / 0.598 on bare `contact`; needs `contact_push` |
| can-settle | **held** | **held** | ≈ 0 for every arm — the finding is that no arm learns it |
| curves, any stage | **impossible** | needs the (w) wires | **have** for pick/place/slide |

## 3. The blocker behind every "blocked" cell

**Learning curves for place and the two contact stages do not exist for any learner, and cannot be produced from the data on disk.** In full scope the per-step stage flags are written only when an episode terminates *inside* the adapter; a horizon truncation logs all zeros even for an episode that picked (1,198 of 2,911 episodes on one run, every one at exactly the horizon, 608 of them having scored). That leaves the accumulated reward as the only truncation-proof channel — so **we can only curve what the reward pays for**, and neither `placed_v2` nor the slide predicate is a rung.

Amendment (w) fixes it and is **half implemented**: the environment now emits sticky cumulative stage flags from the single exit path both termination and truncation pass through (commit `7096fe6`), gated off by default. Two wires remain, specified in `paper/HANDOFF_episode_record_2026-09-08.md`: the adapter must copy those flags onto the *transition* as `log_`-prefixed keys (the trainer reads `trans`, not `info`), and the launcher must export the gate.

**Diffusion Policy can never contribute a learning curve** — it is offline and produces no rollouts. Any cross-learner curve figure is a two-learner figure and must say so.

## 4. Periodic checkpoints for dynamic evaluation — cost

Measured on this project, weights only (optimiser state is pruned after training and is not needed to evaluate):

| learner | per checkpoint | 10 milestones × 1 seed | × 4 seeds × 2 arms |
|---|---|---|---|
| RLPD | ≈ 18 MB | 0.18 GB | **1.4 GB** |
| world model | ≈ 131 MB | 1.3 GB | **10.5 GB** |
| Diffusion Policy | **949 MB** | 9.5 GB | **76 GB** |

Across all four scopes (end-to-end, pick, place, contact) that is ≈ 6 GB for RLPD, ≈ 42 GB for the world model, and **≈ 304 GB for Diffusion Policy** — which does not fit alongside current usage.

**Recommendation.** Keep 10 milestones for RLPD and the world model; for Diffusion Policy keep **4** (≈ 30 GB across all scopes), since it is the learner whose curve matters least — it has no acquisition dynamics to plot, only a checkpoint-versus-performance sweep. Always prune the optimiser state, which is two thirds of a Diffusion Policy checkpoint. Note also that milestone checkpoints only give an *evaluation-protocol* curve; the cheap per-episode log gives an *acquisition* curve and the two are not interchangeable — the log's endpoint is not the table number.

## 5. What is already implemented

- **Periodic saving exists in all three**: RLPD `--ckpt-every`, Diffusion Policy `save_freq`, world-model milestone checkpoints (the latter built with online-step accounting and content hashes by the Codex lane).
- **Evaluators exist** for every stage: `contact_push`, `nested_honest` and `slide_success` all compute today, and the settled predicate is `can_pos_recovery/slide_predicate.py`.
- **A curve builder exists** (`HRI_results/curves/learning_curves.py`) with the nested-stage assertion.
- **Missing:** the two (w) wires; a standing decision on milestone cadence; and the rebuilt matched datasets, which supersede every source comparison above.
