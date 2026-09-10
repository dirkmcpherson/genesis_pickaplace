# End-to-end runs: what is wrong with them, and how to check it yourself

**Written 2026-09-10 for an auditor with NO prior context.** Every claim below carries the
command that produced it. Do not take any of it on trust — several statements in the project's
other documents are things I asserted and later disproved, and those are listed in §6.

`$LAB = /cluster/tufts/shortlab/jstale02`

---

## 1. What the runs are

64 training runs of the end-to-end (full task: pick → place → slide the can into the goal):

| learner | n | budget | code tree | arms |
|---|---|---|---|---|
| **{r2dreamer}** (world model) | 32 | 4M env steps | `$LAB/wm_fix_2026-09-03/gp_root` | human `dHfull_all_rx` (74 tapes) / machine `dDPfull_first_rx` (72) |
| **{RLPD}** (online RL from demos) | 32 | 250k decisions | `$LAB/gp_e2e` | same two arms |

Seeds: human 900–915, machine 920–935 (16 per arm per learner). Diffusion Policy is **not** in
this batch. The machine set is the **de-selected** first-attempt set (PHASE_PLAN (v)) — the
best-of-3 selection was removed because it, not demonstration source, drove earlier results.

Run dirs: `$LAB/wm_fix_2026-09-03/runs/full_r2d_state_*_s9*` and
`$LAB/gp_e2e/baselines/rl/checkpoints/e2e/e2e_rlpd_*_s9*`.

Eval cells in those dirs are stamped `role: preview`. **The cells of record come from a pinned
re-score pass that has NOT been run for this batch.** No number here is final.

---

## 2. Defect 1 — THE TWO LEARNERS TRAINED ON DIFFERENT REWARD LADDERS (most serious)

    $ grep -n "^STAGE_REWARD" $LAB/gp_e2e/baselines/rl/full_env.py
    57:STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)     # hardcoded, NO gate
    $ grep -n "^STAGE_REWARD" $LAB/wm_fix_2026-09-03/gp_root/baselines/rl/full_env.py
    72:STAGE_REWARD = _STAGE_REWARD_X if os.environ.get('FULLENV_REWARD_X','')=='1' else _STAGE_REWARD_OLD
    $ grep -n "GENESIS_PICKAPLACE_ROOT" $LAB/gp_e2e/cluster/sbatch_rlpd_e2e.sh
    41:cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"          # = gp_e2e, the tree with no gate
    $ grep -n "GENESIS_PICKAPLACE_ROOT" $LAB/wm_fix_2026-09-03/wmfix_full.sbatch
    27:export GENESIS_PICKAPLACE_ROOT=${GP_ROOT:-$LAB/wm_fix_2026-09-03/gp_root}

Every e2e job was submitted with `FULLENV_REWARD_X=1`. **It was a no-op for all 32 RLPD runs**,
because that clone's `full_env.py` predates amendment (x) and has no gate at all. So:

* **{RLPD}** was paid `picked 1 / placed 1 / contact 2 / nested 4` — and `placed` never fires,
  so effectively `picked 1 / contact 2 / nested 4`.
* **{r2dreamer}** was paid `picked 1 / placed_v2 1 / contact_push 2 / slide_success 4`.

**Independent check that does not rely on reading code:** run one episode and look at the reward.
An episode with `placed_v2`, `contact_push` and `slide_success` all true pays **8** under (x) and
**7** (= 1+2+4) under the old ladder:

    sbatch $LAB/annotate_episodes.sbatch     # prints "[gate] FULLENV_REWARD_X=1" then r=7.0

**Consequences.** Within-learner human-vs-machine contrasts are unaffected (both arms of a
learner ran one tree, one ladder). **Any cross-learner e2e comparison is confounded** — the
learners optimised different objectives.

It also inverts the naive reading of the phase results: **RLPD reached placement 69/240 and
produced 7 slides while being paid for NEITHER; r2dreamer sat at ~0.05 placement and ~0 slide
while being paid +1 and +4 for exactly those.** Whatever drives placement and slide here, it is
not the reward term.

**Not actioned:** whether RLPD is retrained under (x). Not needed for within-learner assessment;
required before any cross-learner claim.

---

## 3. Defect 2 — the `rnd30` start set OVERLAPS the goal region

    $ grep -n "BOX_POS\|BOX_SIZE" baselines/sim_variants.py
    185:BOX_SIZE = (0.4, 0.75, 0.12); BOX_POS = (0.75, -0.1875, 0.05); BOX_TOP_Z = 0.11
    → shelf footprint x ∈ [0.55, 0.95], y ∈ [-0.5625, 0.1875]
    $ python3 $LAB/ic_inspect.py         # prints each rnd IC's can_pos

ICs 6/13/19/26 have can x = 0.556 / 0.591 / 0.575 / 0.561 — **inside the footprint**. Every other
IC has x < 0.55. Perfect separation at the shelf's near edge.

Those starts have the can in-footprint, upright, gripper open, so after `PLACE_SUSTAIN`=10 frames
`placed_v2` grants **with the arm still at home**. Verified — both arms' ic13 episodes score
`stages={'placed_v2': 1}` and nothing else, reward 0.0, full 300 decisions:

    $ python3 $LAB/placed_without_pick.py

**This is NOT a predicate bug.** `placed_v2` correctly reports a true state that the task setup
created. Measured over 390 eval episodes from 13 fully-trained RLPD runs: 26 spurious firings,
**symmetric across arms (13 human, 13 machine)** — it inflates both arms' level without biasing
the contrast. Net of it:

| arm | episodes | placed_v2 raw | spurious | net | rate |
|---|---|---|---|---|---|
| human | 240 | 82 | 13 | **69** | **28.8 %** |
| machine | 150 | 14 | 13 | **1** | **0.7 %** |

The raw 82-vs-14 badly understates the gap. Those far-x starts are also outside training support
(`paper/DP_PRUNED_GAP_2026-09-07.md`: 0/280 picks at x ≥ 0.52), so they were never winnable.

Two fixes exist and they are **not** equivalent: requiring `picked ∧ placed_v2` changes a REWARD
term; excluding the overlapping ICs changes the frozen eval start set and moves every prior
`rnd30` number in the project. Neither is actioned.

---

## 4. Defect 3 — predicates that share a name but not a meaning

    $ python3 $LAB/timeout_anatomy.py

Over the same 390 episodes:

* `nested_proxy` fires **83** times, `nested_honest` **15**, and they agree on only **7**. The
  proxy misses 8 of 15 real successes and fires 76 times without one. Project docs describe the
  proxy as over-counting "~2.5x"; measured here it is far worse than that.
* `timeout` is a **residual** label — `'nested_proxy' if … else ('tipped' if … else 'timeout')` —
  meaning "neither nested nor tipped". Empirically all 179 did run the full 300 decisions, so the
  name happens to be accurate, but **64 of them picked the can and 66 reached `placed_v2`**, and
  **8 have `nested_honest = 1`**. Filtering "successes" by outcome label discards most placements.
* `placed` (legacy) never fires at all.
* `slide_success` and `nested_honest` are decided by a **post-episode settle**, so they cannot
  exist as per-decision flags. That part is inherent, not a bug.

---

## 5. What is safe to say, and what is not

**Safe.** Within-learner human-vs-machine contrasts. The symmetry of the `placed_v2` artefact
(13/13) means it does not bias those. Checkpoints exist for every run, so any predicate can be
re-scored without retraining ({RLPD} 3 ckpts/run, 0.6 GB total; {r2dreamer} `latest.pt`, 2.6 GB).

**Not safe.** (a) Any cross-learner comparison (§2). (b) Any absolute placement level on `rnd30`
without netting out §3. (c) Any statement built on `nested_proxy` as if it were nesting. (d) Any
number from these run dirs described as final — they are `role: preview`.

**Cannot be recovered by re-scoring:** the training-time phase curves. They were logged as the
episodes happened; a new predicate cannot relabel them without re-simulating.

---

## 6. Claims I made in this project's documents and then DISPROVED

An auditor reading the other docs chronologically will meet these as assertions before meeting
the corrections. All four were mine.

1. **"The RLPD episode record is hollow — it captures only the stage that ended the episode."**
   I relaunched 16 runs over this. **Disproven**: per-episode comparison gives
   `both=6, ep_only=0, term_only=0` — zero disagreement. `info['picked']` is already sticky at the
   done step. `picked` read 0 because an untrained policy had not picked yet. An absent signal
   from a policy that cannot yet produce it is not evidence of a broken recorder.
2. **"slide_success is never observed."** **Wrong, twice.** It is 7 in the human arm and 0 in the
   machine arm across 13 fully-trained runs. The training curve reads 0.00 because 7 events is
   below the line's resolution — "flat at this scale" is not "never occurs".
3. **"Zero failures"** — reported three times off a monitor whose `sacct` call was malformed
   (`%%`-escaped format strings → `Invalid time specification` → zero rows). It was structurally
   incapable of reporting a failure. A real job had died.
4. **A figure labelled `{r2dreamer}` that contained {RLPD} data** (argv off-by-one). Corrected.

The pattern in all four: a value that was *absent* being read as a confident zero.

---

## 7. Why the golden set is proposed, and what it does NOT fix

There is no set of episodes with verified labels, so no predicate test can fail, and every defect
above was found by a number failing to reconcile rather than by a check. The proposal: 30–40 e2e
episodes, stratified over the disputed classes (the 26 spurious `placed_v2`, the 8
`timeout`-with-`nested_honest`, the 76 proxy-without-honest, all 7 slides, plus clear positives
and negatives), stored as **trajectories** rather than checkpoint+IC so validation never depends
on sim determinism, labelled **blind from clean video** against a rubric written first.

**It would catch §3 and §4. It would NOT have caught §2** — the ladder confound needs a
provenance check that the code actually loaded is the code intended. That is a different guard,
and it is the one this project most lacks: three code trees are in play
(`gp_e2e`, `gp_root`, `genesis_pickaplace`) and they have silently diverged in both directions.

---

## 8. State and open decisions

Running at time of writing: ~13/32 {r2dreamer} trained, ~15/32 {RLPD} done; monitors auto-recover
stranded evals. Note {r2dreamer} runs **exit non-zero after training completes** — a launcher
check greps its own Slurm log by an assumed job name, which any `-J` rename breaks. Training
artefacts are intact; `cluster/e2e_posthoc_sweep.sh` recovers the evals.

Open, all requiring a human decision:
1. Build the golden set? (I can do everything except supply labels.)
2. Retrain {RLPD} under the (x) ladder? Only needed for cross-learner claims.
3. Fix `placed_v2` by predicate (changes a reward term) or by IC set (moves every prior number)?
4. DP on `dHfull_pruned` — still unstarted; the convergence runs used the RAW human set.
