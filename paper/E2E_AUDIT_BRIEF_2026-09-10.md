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

### 4a. THE PROXY INVERTS THE HUMAN-VS-MACHINE ORDERING (measured 2026-09-10)

    $ python3 $LAB/nested_asymmetry.py     # rnd30 mode cells, all trained {RLPD} runs

| arm | episodes | `nested_proxy` | `nested_honest` | proxy-only | proxy precision | proxy recall |
|---|---|---|---|---|---|---|
| human | 300 | 44 (**0.147**) | 12 (**0.040**) | 39 | 0.114 | 0.417 |
| machine | 240 | 70 (**0.292**) | 3 (**0.013**) | 68 | 0.029 | 0.667 |

**On `nested_proxy` the machine arm leads 0.292 to 0.147. On `nested_honest` the human arm leads
0.040 to 0.013. Same episodes, same runs, opposite conclusion.**

The proxy is not merely noisy, and its error is **not symmetric across arms**: its precision is
0.114 for the human arm and 0.029 for the machine arm — a 4x difference. The machine arm produces
68 proxy firings that the settled predicate rejects, against 3 real nests.

This matters more than the `placed_v2` artefact in §3, which was symmetric (13/13) and therefore
left the contrast unbiased. **This one biases the contrast itself.** Any result stated on
`nested`, `nested_proxy`, or a reward ladder that pays for the proxy (which is what {RLPD}
trained on — see §2) inherits an arm-dependent error large enough to reverse the sign.

### 4b. The same census for {r2dreamer}, and the strongest evidence on slide

    $ python3 $LAB/r2_select.py      # rnd30 mode cells, 390 episodes, 13 runs

| arm | episodes | `nested_proxy` | `nested_honest` | `slide_success` |
|---|---|---|---|---|
| human | 180 | 8 | 3 | **0** |
| machine | 210 | 6 | 2 | **0** |

**Zero slides in all 390 episodes — from the learner that was paid +4 for exactly that.**
{RLPD}, paid nothing for sliding (§2), produced 7. Slide occurrence is therefore uncorrelated
with slide reward across the two learners, which is the strongest single piece of evidence that
the binding constraint on the top rung is the WORLD (§10b: 2-4 cm systematic under-transfer of
the push), not the incentive. Raising the slide reward is very unlikely to produce slides.

> **UNRECONCILED (added 2026-09-10 at handoff; see `paper/E2E_TRAINING_PROBLEMS_2026-09-10.md` §0).**
> The argument above is cross-learner, which §2 says is confounded. The WITHIN-learner evidence
> disagrees (measured from existing cells, same evaluator): {r2dreamer} human arm, OLD ladder
> 4.1M (4 seeds) nested_honest 0.167 (20/120) and 12 slides, against (x) ladder 4M (6 seeds)
> 0.017 (3/180) and 0 slides; per-seed exact p 0.048 / 0.033 (one-sided). Under (x),
> `contact_push` fires 0.444 WITHOUT release, so the policy stalls at rung 3 pressing the held can
> against the goal. **Do not act on "world, not incentive".** Full table and mechanism:
> `paper/E2E_TRAINING_PROBLEMS_2026-09-10.md` §0.

{r2dreamer} also nests far less than {RLPD} at every level (proxy 8/180 vs 44/300; honest 3/180
vs 12/300) **despite the better pick rate** (~0.88 vs ~0.80). It picks more and finishes less.
Real, but confounded by the ladder split (§2) — do not report it as a learner result.

The proxy inversion in §4a is {RLPD}-specific: at n=8 and n=6 the r2dreamer proxy counts are too
small to reverse anything. The inversion is driven by {RLPD}'s 68 machine-arm proxy-only firings,
and {RLPD} is also the arm whose reward paid for the proxy.

**Note the two evaluators differ in honesty.** `eval_genesis.py` (r2dreamer) already records
`outcome` AND `outcome_honest` and labels 12 episodes `proxy_only` outright; `eval_e2e.py`
({RLPD}/DP) does not. Both share one trap: episodes that are `nested_honest` but labelled
`timeout` (3 in r2dreamer, 8 in RLPD).

Practical consequence: **do not report any nesting comparison on the proxy.** The settled
predicate is available for every episode via the post-episode `end_of_episode()` call, costs one
extra settle, and is already recorded in the eval cells as `nested_honest`.

* `slide_success` and `nested_honest` are decided by a **post-episode settle**, so they cannot
  exist as per-decision flags. That part is inherent, not a bug.

---

## 5. What is safe to say, and what is not

**Safe.** Within-learner human-vs-machine contrasts. The symmetry of the `placed_v2` artefact
(13/13) means it does not bias those. Checkpoints exist for every run, so any predicate can be
re-scored without retraining ({RLPD} 3 ckpts/run, 0.6 GB total; {r2dreamer} `latest.pt`, 2.6 GB).

**Not safe.** (a) Any cross-learner comparison (§2). (b) Any absolute placement level on `rnd30`
without netting out §3. (c) **Any nesting comparison stated on `nested_proxy` — it reverses the
arm ordering (§4a), and its error is arm-dependent, so it biases the contrast rather than the
level.** (d) Any
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

### Defect 4 (found 2026-09-10, late) — PREEMPTION IS FATAL to {r2dreamer} runs

`wmfix_full.sbatch` has a requeue handler that clears a partial logdir on restart (line ~43,
keyed on `SLURM_RESTART_COUNT`), but the existence guard ran FIRST and exited:

    26: [ -e "$LOGDIR" ] && { echo "FATAL: $LOGDIR exists"; exit 2; }
    43: if [ "${SLURM_RESTART_COUNT:-0}" -gt 0 ] && [ -d "$LOGDIR" ]; then ... clearing ...

The handler was therefore unreachable. Every preempted r2dreamer run died instantly on restart —
`FAILED 2:0`, elapsed `00:00:00` — despite `--requeue` and despite code written for exactly this.
**Cost: 5 seeds (dH s905/s906/s913, dM s929/s932), all preempted together on pax111.**

Fixed (`cluster/fix_requeue_guard.py`): the guard now yields when `SLURM_RESTART_COUNT > 0`. The
5 seeds were resubmitted (3515822-26). **Slurm spools batch scripts at submission, so the fix
reaches only NEW submissions** — jobs already queued or running still carry the old guard and will
still die if preempted. Watch for `FAILED 2:0` with elapsed `00:00:00`.

This is the third instance of the same class today: code written to handle a case, made
unreachable by an earlier line. (The others: the launcher's log-name assumption, §9; and
`full_eval_sweep.sh` skipping every `-J`-renamed run.)

### Defect 5 (verified in code 2026-09-10, late) — under the (x) ladder the +4 slide rung CANNOT be paid in training

    $ sed -n 978p $W/gp_root/baselines/rl/full_env.py        # terminated = bool(info.get('nested'))  (the PROXY)
    $ sed -n 699,704p $W/gp_root/baselines/rl/full_env.py    # proxy = contact ∧ grip<0.3 ∧ both upright, per env frame
    $ sed -n 328,333p $W/gp_root/baselines/genesis_can_env.py # slide = same clauses + footprint, 12 CONSECUTIVE frames
    $ sed -n 325,345p $W/r2dreamer_fix/envs/genesis.py       # end_of_episode() runs AFTER the terminal reward; logging only

The proxy's conditions are a subset of the slide's, so the episode terminates on the first frame
of the 12-frame slide window. The settle route is post-terminal and unpaid. The reward loop never
sees `slide_success=True`. **The (x) training ladder was picked 1 / placed_v2 1 / contact_push 2,
max 4, with an unpaid terminal.** The `_rx` demo prefill still pays +4 in 13/74 human tapes, so
buffer and env disagree on the objective. And the env's `slide_success` is the (l) predicate
(`grip < 0.3`, withdrawn by (p)), not the (x) definition that was registered.

Measured effect, {r2dreamer} human arm, rnd30 mode: OLD ladder 4.1M (4 seeds) nested_honest
0.167, slides 12/120; (x) 4M (6 seeds) 0.017, 0/180. Full table and the equal-config check:
`paper/E2E_TRAINING_PROBLEMS_2026-09-10.md` §0. This is the reason for the {r2dreamer} numbers in
§4b, and it removes the basis for "world, not incentive".

Note the {r2dreamer} launcher DOES stamp the ladder (`[gates] STAGE_REWARD in force:` in every
`e2eL_r2_*` Slurm log); the {RLPD} launcher stamps nothing.

### Phase annotators (both learners, added 2026-09-10)

Overlay per-decision sticky stage chips + firing decision + terminal verdict on eval videos.
Both are deliberate COPIES — the originals are read at runtime by in-flight eval stages and must
not be mutated mid-batch — and both snapshot the SAME union the scorer uses
(`_granted | {k for k in STAGES if info[k]}`), so the overlay shows what was scored, never a
reimplementation.

* {RLPD}/DP: `baselines/eval_e2e_annot.py` (in this repo). Driver: `cluster/annotate_episodes.sbatch`,
  `cluster/annotate_push.sbatch`. Single episode via `--ic-index N`.
* {r2dreamer}: `$W/r2dreamer_fix/eval_genesis_annot.py`, generated by `cluster/make_r2_annotator.py`.
  Driver: `cluster/annotate_r2.sbatch`. That evaluator has no `--ic-index`, so a single episode is
  selected with `--episodes N+1 --ic-skip 0..N-1`; skipped episodes still write placeholder
  `*_hang.mp4` files, so **filter those out** when collecting results.

Note `slide_success` and `nested_honest` are decided by a POST-EPISODE settle, so they appear only
as a terminal verdict in the panel — there is no decision at which they can light. That is
inherent to the predicates, not a limitation of the overlay.

Open, all requiring a human decision:
1. Build the golden set? (I can do everything except supply labels.)
2. Retrain {RLPD} under the (x) ladder? Only needed for cross-learner claims.
3. Fix `placed_v2` by predicate (changes a reward term) or by IC set (moves every prior number)?
4. DP on `dHfull_pruned` — still unstarted; the convergence runs used the RAW human set.
5. The reward-ladder redesign the user proposed 2026-09-10: drop the proxy (agreed), raise the
   slide reward, and make the ladder sparser. Evidence in 4b argues the last two will not work as
   intended -- slide occurrence is uncorrelated with slide reward across the two learners, and at
   a ~1% occurrence rate a sparser ladder pays approximately never. Recommended order: first
   measure whether a slide is reachable AT ALL from a good placement (scripted-push probe, a few
   CPU-hours); if it is, reward goalward can displacement during contact rather than the terminal
   event. Any ladder change means RETRAINING -- it cannot be re-scored from checkpoints.

---

## 9. Exactly how these runs were launched

`$W = $LAB/wm_fix_2026-09-03`, `$E = $LAB/gp_e2e`.

### {r2dreamer} — 4M steps

    cd $W
    for i in 0 1 2 3 4 5 6 7; do
      FULLENV_REWARD_X=1 sbatch -J e2eL_r2_dH_s$i $W/wmfix_full.sbatch dHfull_all_rx      $((900+i)) 4000000
      FULLENV_REWARD_X=1 sbatch -J e2eL_r2_dM_s$i $W/wmfix_full.sbatch dDPfull_first_rx   $((920+i)) 4000000
    done
    # extension, same form with i = 8..15  → seeds 908-915 / 928-935

`wmfix_full.sbatch <ARM> <seed> <steps>`; ARM is the directory name under
`${DEMO_ROOT:-$W/demos_state_full}`. The launcher exports
`GENESIS_PICKAPLACE_ROOT=${GP_ROOT:-$W/gp_root}` (line 27) — **this is why r2dreamer got the (x)
ladder**. Logdir `$W/runs/full_r2d_state_<ARM>_s<seed>`; the launcher refuses if it exists, and
clears it on requeue.

### {RLPD} — 250k decisions

    cd $E
    for i in 0 1 2 3 4 5 6 7; do
      ARM=dH        SEED=$((900+i)) DEMO=$W/demos_state_full/dHfull_all_rx     FULLENV_REWARD_X=1 \
        sbatch -J e2eL_rl_dH_s$i cluster/sbatch_rlpd_e2e.sh
      ARM=dDPfirst  SEED=$((920+i)) DEMO=$W/demos_state_full/dDPfull_first_rx  FULLENV_REWARD_X=1 \
        sbatch -J e2eL_rl_dM_s$i cluster/sbatch_rlpd_e2e.sh
    done
    # extension, same form with i = 8..15

`sbatch_rlpd_e2e.sh` reads ARM/SEED/DEMO/STEPS from the environment and does
`cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"` (line 41) — since it is invoked from `$E`, it runs the
**gp_e2e** tree. **`FULLENV_REWARD_X=1` here is inert** (§2). Run dir
`$E/baselines/rl/checkpoints/e2e/e2e_rlpd_<ARM>_s<seed>`.

### `-J` matters, and broke things

Both launchers were given explicit `-J` names so the queue is readable. Two scripts locate a
run's Slurm log by an **assumed job name** and therefore broke:

* `wmfix_full.sbatch` looked for `$W/slurm/wmfix_full_${SLURM_JOB_ID}.out`; with `-J` the log is
  `e2eL_r2_*_<id>.out`, so the post-training check died (`FATAL: no [sim-variant] line`) **after
  a complete 12h training run**. Fixed to resolve by job id.
* `full_eval_sweep.sh` (the project's standing recovery tool) globs `slurm/wmfix_full_*.out` for
  the same reason and **silently skips every `-J`-renamed run** — it would report success having
  recovered nothing. `cluster/e2e_posthoc_sweep.sh` was written to replace it for this batch.

### Generations — several seeds were launched more than once

`sacct` shows far more jobs than 64 (`e2eL_rl_dM` alone has 56 records). **The live run for a seed
is the highest job id bearing that job name**; earlier ones are FAILED or CANCELLED. To trace one:

    sacct -S 2026-09-09 -u $USER -X -n --name e2eL_rl_dM_s2 --format=JobID%14,State%12,Elapsed

| generation | ids (approx) | what changed |
|---|---|---|
| RLPD gen 1 | 3484565–66 + | machine arm FAILED 1:0 — the launcher's `one_per_ic_first` manifest gate correctly refused a relabelled set whose manifest did not attest the de-selection |
| RLPD gen 2 | 3484659–67 | after the manifest gained verified provenance (72/72 keys match, all 72 action streams byte-identical by sha256; only reward differs) |
| RLPD gen 3 | 3484712–27 | relaunch for the sticky episode-record change — **13 REFUSED by the run registry**, correctly: the fix was still uncommitted, so `(script, arm, seed, git)` was unchanged and it looked like a duplicate run |
| **RLPD gen 4** | 3484741–56 | after committing gp_e2e `6e98ce3`. **This is the live generation for seeds 0–7** |
| RLPD gen 5 | 3488574+ | extension seeds 8–15 |
| **r2 gen 1** | 3484549–3484601 | seeds 900–907 / 920–927. **Spooled the PRE-FIX launcher**, so each exits non-zero after training; evals recovered post hoc |
| r2 s906 | 3484598 → 3486259 | bus error at 1.1M steps → resubmitted → later preempted → auto-requeued |
| **r2 gen 2** | 3491309–10 + | extension seeds 908–915 / 928–935, cancelled and resubmitted against the **fixed** launcher |

**Within-arm code difference an auditor must know:** r2 gen 1 and gen 2 ran *different launcher
versions*. The difference is confined to the post-training log check — the training command,
tree, ladder and demo sets are identical — so it does not affect what any policy learned. But
gen-1 runs die at the end and gen-2 runs do not, and only gen 1 needs the post-hoc eval sweep.

Slurm spools the batch script at submission, so a launcher fix **cannot** reach already-submitted
jobs, pending ones included. That is why the pending r2 extension seeds had to be cancelled and
resubmitted rather than simply left to pick up the fix.

### Datasets the runs consume

`$W/demos_state_full/dHfull_all_rx` (74 tapes) and `.../dDPfull_first_rx` (72). Both are
reward-relabelled copies (`baselines/rl/relabel_reward.py`) of the corresponding
`demos_state_full/` sets; `dDPfull_first_rx` derives from the first-attempt (de-selected) machine
set. Manifests are asserted by both launchers before training starts — sim_variant, scope=full,
`with_state`, `action_repeat==4`, `reward_from_tape`, `delta_cap`, tape count, and the
`one_per_ic_first` / `one_per_ic_best` selection flags.

---

## 10. The world these runs use, and what is known to be wrong with it

    $ grep -n "gc_kp4_riser3_shelf6'" baselines/sim_variants.py
    'gc_kp4_riser3_shelf6': dict(kp_mult=4.0, kv_mult=2.0, gravity_comp=1.0,
                                 effort='base', riser=0.03, shelf_dz=0.06)

Asserted in every demo manifest and every eval cell, so all 64 runs share it. Four world-level
facts bear on how the e2e results should be read:

**(a) The declared contact-solver fix is NOT in this world.** `sim_variants.py` carries an
explicit comment: `gc_kp4_riser3_shelf6` was *"DECLARED world of record on 2026-08-26 (commit
7c8195d) but NEVER RUN as one: every frozen w3 set, harvest and reported arm was built on
gc_kp4_riser3_shelf6 WITHOUT this fix (dropped by omission)"*. The variant dict has no
`grasp_timeconst` key; a `_ts5` sibling exists and is unused here. Consequence, per
`paper/TS5_DROPPED_FIX_2026-09-02.md`: **8–10 mm of finger-into-can penetration is live in every
arm of this batch**, and over-squeeze can eject the can. It is symmetric across arms, so it does
not bias human-vs-machine, but it caps absolute grasp quality.

**(b) The slide is control-limited by the simulator, not by the policy.**
`paper/SLIDE_ANATOMY_2026-09-07.md` measured that at set-down the sim tool tracks the real tool to
**0.2 cm median**, but the *can* does not follow: engaged strokes transfer only ~0.65 of the
goalward motion and lose ~2.4 cm laterally per tape, a systematic **2–4 cm under-transfer**. This
is why the same human demonstrations yield ~15 completed slides in sim against the ~18 a human
counts from the real footage. **Any e2e slide number is therefore a lower bound set partly by
world fidelity**, which matters a great deal given slide is the top rung of the ladder and reads
near zero (§4, §6.2).

**(c) Contact-constraint creep.** `paper/CONFOUNDS.md` row 49: Genesis regularises friction
constraints so a pinched can under a sub-limit tangential load creeps linearly in time (59°
droop/s at the engine default impedance). This is a real engine property; the registered
impedance ladder was tested and **did not** fix the recorder path, so w3 was kept.

**(d) Two initial conditions are unwinnable by construction.** CONFOUNDS row 51: uids 234 and 318
are the only 90°-lying-can entries in `trial_placements.json` (an artifact of a 2026-07-20
placement pass); the tip rule fires at decision 1 and both tapes are one decision long. Real
footage shows both cans upright at t0. They sit in every n=74 denominator.

See also CONFOUNDS row 50 (the `og4` fast-open release filter was adopted for the *recorder*
path; machine-arm harvests do not receive it — check before comparing grip dynamics across arms).

---

## 11. The demonstration sets

Verified from the manifests (`$W/demos_state_full/<set>/repeat.json`):

| | human `dHfull_all_rx` | machine `dDPfull_first_rx` |
|---|---|---|
| tapes | 74 | 72 |
| Σ reward (relabelled) | 238.0 | 237.0 |
| decisions min / median / max | 2 / **389** / 601 | 2 / **601** / 601 |
| sim_variant | gc_kp4_riser3_shelf6 | gc_kp4_riser3_shelf6 |
| action_repeat / delta_cap | 4 / 0.025 | 4 / 0.025 |
| selection | none (every attempt kept, no-picks included) | `one_per_ic_first=True`, `one_per_ic_best=False` |

**The episode-length asymmetry is large and easy to miss:** the machine median is 601 — the cap —
while the human median is 389. Machine demonstrations mostly run to the horizon; human ones end
when the person finishes. Any per-decision statistic (idle fraction, action magnitude) inherits
this.

**Provenance of the human set.** These are teleoperated Kinova gen3-lite pick-and-place trials
replayed in sim. The 74 uids are **all from a single afternoon** (the 2024-12-18 session). A
further 120 trials from three earlier days were recovered on 2026-09-08
(`paper/EARLY_TRIALS_ROTATION_2026-09-08.md`, 88/89 solved) but are a **disclosed separate pool**
and are NOT in these sets.

**Participant identity is unknown, and this is a real limitation for a paper about "human
demonstrations".** Per `CLAUDE.md`: no identifier exists anywhere — `config.yaml` is byte-identical
across all 224 trials except `data_dir`, `user_NNN` is a trial counter, no bag topic carries one,
and METHODS states no participant count. One person over four days and four people over four days
are **equally consistent with the data**. If it is one person, "human demonstrations" describes a
single individual's style, which is worth stating explicitly rather than leaving to the reader.

**Provenance of the machine set.** Harvested from a Diffusion Policy teacher trained on the human
demonstrations, then reduced to the **first** attempt per initial condition. The earlier
best-of-3 selection was removed deliberately (PHASE_PLAN (v)) because selection, not
demonstration source, produced most of the machine arm's apparent advantage: Σ reward
131 → 206 (+57 %) and completions 8 → 16 under selection, and `best ≠ first` on 24 of 72 starts.
**The de-selected set is the honest comparison and is what these runs use.**

**Lineage caveat.** `paper/SLIDE_CLAUSE5_LINEAGE_2026-09-07.md` found that `dHfull_w3` and the
honest census are *different recordings of the same 74 ICs* — action streams differ on 24 of 74
and contact flags on 11 — with the census re-executing bit-exactly and `dHfull` not. Check which
lineage any given number came from before combining.

**One asymmetry that is NOT a confound:** `convert_to_lerobot` drops 2 one-decision tapes per arm,
so DP datasets hold 72/74 and 70/72 while RLPD and the world model train on all. It is symmetric
and 0.007 % of rows, and is recorded in the manifests.
