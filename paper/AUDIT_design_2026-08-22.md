# Independent design review — "human vs model demonstrations" (Genesis Kinova pick)
Reviewer stance: NeurIPS/CoRL AC with RL-from-demos experience. Scope: experimental DESIGN and
DIRECTION. Repo read: paper/*.md, PAPER_PLAN.md, cluster/*.sh, baselines/rl/*.py (sparse checkout,
no data). Nothing was edited. All p-values quoted below were recomputed independently and the
project's arithmetic is correct; the problems are identification, protocol, and multiplicity — not
arithmetic.

Bottom line up front: the engineering hygiene here is far above average (registered bars, dated
corrections, adversarial audits, fresh-process evals, run registry). The *experimental design* is
not: **no cell in the matrix is a single-variable comparison of demonstration source**, and two of
the five headline claims are confounded by protocol changes rather than by the independent
variable. The paper is currently a collection of well-instrumented observations in search of an
identified claim. The good news is that the strongest available thesis is already in the data and
costs little more compute to secure.

---

## TOP 8 ISSUES, RANKED BY THREAT

### 1. "Demo source" is never the only variable — every arm differs on 4+ axes at once
The three sources differ simultaneously in fail-tape share, episode length, frame count, action
representability, and reward density. Measured (paper/AUDIT_normalization_2026-08-17.md:223-241,
:87-94, :475-480; paper/ROUND_ROBIN_RESULTS_2026-08-22.md §"Why dDP_RLPD < dH_RLPD"):

| RLPD demo buffer | dH | dDP | dR2D |
|---|---|---|---|
| episodes | 91 | 93 | 66 |
| transitions | 83,465 | 70,028 | 9,118 |
| fail/no-pick share of buffer | 33.1% | 51% | 0% |
| rewarded-transition density | 0.079% | 0.090% | 0.724% |
| frames with label err >1e-3 (delta encoding) | 13.29% | 11.29% | **0.000%** |
| frames over the 0.025 action cap | 4.40% | 7.89% | 0.098% |
| episode length p50 | 745 | ~1200 (fails) / 512 to first lift | 130 |

A 9.2x difference in demo-buffer size and a 9.2x difference in reward density between dH and dR2D
is not a nuisance parameter in an algorithm whose every minibatch is 50% demo data. The RLPD
"null" (dH 8/16 vs dR2D 10/16) is therefore not evidence that source doesn't matter; it is evidence
that two very different datasets happen to land at the same ignition rate. Amendment 2
(CLUSTER_ROUND_2026-08-17.md) acknowledges the fidelity/density asymmetry and correctly re-scopes
the wave to "which demo SET works better as-encoded" — but the REVIEW_GUIDE headline still says
"no demo-set effect", and the 08-22 docs still frame dDP-vs-dH as "the unconfounded contrast"
(true only for the fidelity axis; false for fail share, tape length, and buffer size).

**Fix.** Declare which single channel each cell isolates and build sets that vary only that
channel: (a) success-only everywhere OR fails-included everywhere; (b) idle-pruned everywhere or
nowhere; (c) frame-matched (subsample the big sets to ~9k transitions), not episode-matched. The
"matched N=66" rule in PAPER_PLAN §3 matches episodes only — that is the wrong unit for every
learner in the paper.

### 2. The world-model arms' dH set looks like the PRUNED, SUCCESS-ONLY human set — which would
confound the flagship WM result with exactly the variable the RLPD investigation calls causal
paper/ROUND_ROBIN_2026-08-20.md §4.3 and REVIEW_GUIDE state "dH means UNPRUNED for every RL/WM
row". But the r2dreamer dH demo dir is `genesis_pick_pruned_delta25`, 67 eps / 42,706 rows, and dv3's
is `genesis_pick_msr_delta25_r4`, 67 eps (paper/AUDIT_normalization_2026-08-17.md:485-489;
paper/ROUND_ROBIN_RUNNING_2026-08-19.md:14-19). The RLPD dH set is 91 eps / 83,465 transitions
including 25 no-pick negatives; the pruned human set is 66 eps / 39,398 frames
(AUDIT_normalization:475-478). `baselines/rl/to_dreamer_demos.py:26` has a `--pick-only` flag that
drops no-pick negatives. So the WM "dH" arm is very likely 67 pruned SUCCESSES, while its "dDP"
arm is 93 episodes *including 30 whole 1200-step failure tapes* and "dR2D" is 52 successes.
If so, the paper's flagship world-model claim — r2dreamer ignites 8/34 on human demos and 0/23 on
model demos (p=0.016) — is confounded by fail-tape inclusion and idle-frame density, i.e. by the
very mechanism the 08-22 note (PAPER_NOTES N1) proposes as the cause of dDP's RLPD failure. It
would also mean the r2d/dv3 rows are not comparable with the RLPD rows that share the "dH" label.

**Fix (verify first, today, zero GPU).** Count zero-reward episodes and per-episode idle-frame
share in `genesis_pick_pruned_delta25`, `genesis_pick_msr_delta25_r4`, `genesis_m1all_delta25`,
`genesis_r2dchamp_delta25`; publish that census as a table. If the asymmetry is real, either
downgrade the WM source claim to "human-success-only tapes vs model tapes-with-failures", or
rebuild one side (a success-only dDP pixel set is the cheap direction) and rerun ≥6 seeds.

### 3. RLPD has no positive control anywhere, and REVIEW_GUIDE claim 4 overstates what the controls
show
REVIEW_GUIDE: "Every RL/WM implementation is positively controlled on ManiSkill; the two RLPD MS
failures are sparse-reward-specific." The record says otherwise: our RLPD is 0/3 on sparse
PickCube; the reference RLPD is also 0/3 (partial absolution, RESULTS_MATRIX_2026-08-15.md swap
test); and on *dense* PickCube — the discriminating setting — "reference grasps 0.3-0.6, ours ~0"
(ALGORITHM_STATE_2026-08-18.md §1 RLPD controls). Ours being worse than the reference on the one
setting designed to discriminate is the opposite of a passed control. Additionally
AUDIT_ms_chain_2026-08-16.md:160-181 finds only 3/12 of the "exact ManiSkill knobs" actually match.
A reviewer will read the RLPD rows (a third of the paper) as uncontrolled.

**Fix.** Delete claim 4 as written. Replace with the precise statement: "our RLPD and the authors'
reference RLPD fail identically on sparse PickCube at 300k; on dense PickCube the reference reaches
0.3-0.6 grasp and ours ~0, so our RLPD implementation is not positively controlled." Then either
(a) run the one control that could pass — reference-recipe RLPD on a task/budget where RLPD is
published to work (Adroit sparse at 1M, not 300k) — or (b) present RLPD results as "SAC+demo
batches, as implemented" and never as a statement about RLfD in general.

### 4. The r2dreamer ignition rate is a function of how many checkpoints were scored — and the
dense-vs-sparse comparison changed that protocol mid-stream
Waves 1-3 scored ~5 fresh checkpoint evals per seed (paper/figs/FIGNOTES_ignition_20260818.md
T-tables: "5" evals per seed for s10-s39). The 08-19 round scores **every `latest.pt` write**
(cluster/sbatch_r2dreamer.sh:355-380, 15 eps, seed 0, best-2+newest retained) and takes the max.
The project's own data show ignition detection scales with coverage: the local runs with 23-34
evals per seed read 2/4 ignited, the cluster waves with 5 evals read 6/30 — and the champion's
documented bistability is "~1-in-7 checkpoints good". So "dense 4/4 vs sparse 8/34 (p=0.007)" and
even "vs same-era wave-3 2/10 (p=0.015)" confound the treatment (shaping) with a change in the
detection protocol. The dR2D 0/3 in the same round is *not* affected (full coverage, still zero),
so that cell is the trustworthy one.

**Fix.** Rescore wave-3 checkpoints at matched coverage if the logdirs survive; otherwise restrict
the dense claim to a coverage-matched statistic (e.g. best of 5 evenly spaced checkpoints per seed
in both arms), and report the *time-to-first-training-pick* (209-430k vs ~1M) which is
coverage-independent and is the stronger evidence anyway.

### 5. Statistical adequacy: uncorrected multiplicity over a 14-cell matrix, thresholded ignition,
n=3-6 headline cells, and a paired design analyzed as unpaired
- Multiplicity: at least 8 nominal p-values in 0.007-0.05 are quoted across the docs (dDP vs dH
  0.051, dDP vs dR2D 0.015, dH vs dDP r2d 0.020, model-vs-human pixel 0.016, dense r2d 0.007/0.015,
  BC 0.994 posterior, ...). No family is declared and no correction applied. Under BH over the
  declared family most of these sit at the boundary.
- Ignition is a threshold on a 15-episode binomial: a seed with true rate 0.10 clears ≥3/15 18% of
  the time. The binary destroys information that the per-seed rates already carry. And the bar is
  applied to demo-IC only: dDP has random-IC 9/90 vs demo-IC 2/90, with one seed at 4/15 random and
  0/15 demo — under the other regime dDP would read 1/6, not 0/6. That regime asymmetry is
  unexplained and unremarked.
- The dense RLPD arm used seeds 3-8, which are *the same seed values* as sparse-dH seeds 3-8 with
  the same demo set — a matched-pair design that is being analyzed as two independent groups
  (Fisher p=0.65). ROUND_ROBIN_2026-08-20 even records the rationale as "fresh seeds avoid any
  re-execution argument", which inverts the value of the pairing. A paired per-seed analysis on the
  same six seed indices is free and roughly doubles the power.
- Pooled per-episode rates ("0.278 vs 0.221") are quoted alongside seed-level tests. The project's
  own results_significance.md correctly labels pooled Beta-Binomial as anti-conservative; that
  discipline has not been carried to the new cells.
- Best-checkpoint selection protects the *value* (independent confirmation seeds) but not the
  *indicator*: "ignited" is a max over checkpoints, so its expectation rises with checkpoint count
  (see issue 4). Nowhere acknowledged.

**Fix.** Declare one primary hypothesis and one primary statistic before the next run; BH-correct
the rest and label them exploratory. Report per-seed rates with hierarchical CIs (analysis/
bayes_source_effect.py already does exactly this — extend it to the RLPD/r2d cells) rather than
ignition counts as the headline. Analyze the dense arms paired.

### 6. The dDP off-manifold hypothesis is correlational, rests on a proxy human set, and may
describe a pipeline defect rather than a property of model demonstrations
The evidence (ROUND_ROBIN_RESULTS_2026-08-22 §"Why dDP_RLPD < dH_RLPD"; PAPER_NOTES N1) is: dDP
critics diverge earlier (median actor_q 32 vs 0.5/0.6 at 25k) and harder (12,900 vs 1,090/8 at
100k); dDP fail tapes are long (1200 steps), numerous (51% of buffer), and far from the success
manifold (NN-distance p50 2.66 vs 0.88 std). That is a good, cheap forensic pass. It is not yet
evidence of causation, for four reasons:
1. **Dose-response within arms is untested and free.** dH seeds also reach Q 1-4k and still ignite
   8/16 — so high Q is not sufficient for failure. Whether early Q predicts non-ignition *within*
   the dH and dR2D arms is answerable today from 38 existing wandb runs at zero compute. If it
   doesn't, the mechanism story is dead as stated.
2. **The human comparison is a proxy.** The tape geometry table compares dDP against
   `episodes_delta_rerecord_pick_all` (72 tapes, closed-loop re-recorded, 2.6% label error), not
   against the actual dH training set (91 tapes, 13.3% error). The doc says so; the paper cannot.
3. **It is confounded with tape length, buffer size, and action saturation.** dDP fails are 1200
   steps while the *online* episode limit is 900 (`baselines/rl/train_rlpd.py:109`) — every dDP
   fail tape is longer than any online episode, independent of where its states lie. dDP also has
   the highest over-cap rate (7.9%), i.e. the most saturated actions, which alone can push a
   critic to extrapolate at the action bounds.
4. **The proximate cause is arguably a labeling bug, not a demo property.** `delta_encode_transitions`
   (baselines/rl/train_sacfd_full.py:273-292) sets done=True only on the pick reward; the env's
   tip-termination rule is never applied to demo tapes. So post-tip states enter the buffer with
   done=False in *every* arm — dDP just has the most of them. A reviewer will call that an
   implementation defect in the demo pipeline; "model teachers fail differently from humans" is
   then a second-order observation about how much a defect bites per dataset.

**Minimal set that would establish or refute it** (all RLPD, 100k, 6 seeds each, matched machine):
- **dDPsucc** (63 successes, fails dropped) — necessity. Already built (`baselines/make_dDPsucc.py`,
  ARM=dDPsucc in cluster/sbatch_rlpd.sh). Prediction: ignition ≈ dH.
- **dDPtiptrunc** (fails cut at the env's tip rule) — isolates post-termination chains from failure
  states. Already built.
- **dR2D+DPfails** (the clean 66-tape set with the 30 DP fail tapes injected) — **sufficiency; this
  is the missing arm.** Without it, dDPsucc alone cannot distinguish "the fails poison it" from
  "the successes are also weak". Prediction under the hypothesis: dR2D drops from 10/16 to ~0.
- **dDPlen900** (fail tapes truncated to 900 steps, tip states retained) — separates "longer than
  any online episode" from "off-manifold in state space". Cheap to build alongside tiptrunc.
- **Free analyses, do first:** within-arm Q-vs-ignition correlation on the 38 existing runs; the
  same tape-geometry census computed on the *actual* dH set rather than the re-record proxy.
Alternative explanations that none of the above rules out and that should be named in the paper:
action saturation/over-cap (test: the registered `delta_rerecord`-style re-encode of dDP), buffer
size (test: subsample dH/dDP to 9k transitions), and the possibility that the DP teacher's *success*
tapes are simply weak demonstrations (dDPsucc is the falsifier and it is registered — good).

### 7. Eval protocols are not comparable across algorithms, and one of them is shorter than the
demonstrations themselves
RLPD evals at 400 sim steps (`--eval-max-steps` default 400, sweep uses `--max-steps 400`); DP evals
at wandb_eval's default 1200; r2dreamer at 400 sim steps / 100 agent steps; dv3 at 600. Meanwhile
first-lift time is ~784-826 frames for human tapes, 512 for dDP, ~120 for dR2D
(ROUND_ROBIN_RESULTS_2026-08-22 follow-up table). **A policy that picks at human pace scores 0 under
the RLPD eval and non-zero under the DP eval.** RESULTS_MATRIX's "measured effect on the strongest
checkpoint: nil" is a single-checkpoint check, not a protocol validation. Add the r2d
best-of-many-checkpoints protocol vs dv3's 2-3 six-episode periodic draws vs RLPD's fresh-process
15+15, and the round-robin table's columns cannot be compared to each other at all — yet
ROUND_ROBIN_RESULTS §7 and PAPER_NOTES already read across them ("BC prefers model demos; RLPD works
on dH and dR2D; r2dreamer only on dH").

**Fix.** One eval harness, one horizon (1200, or 400 for everyone with the limitation stated), ≥30
episodes, applied to every archived best/final checkpoint of every cell. If that is not affordable,
state explicitly in the paper that cross-algorithm cells are not comparable and confine every claim
to within-algorithm source contrasts.

### 8. Compute is about to disappear and the proposed program does not fit in it — while the
artifacts the headline results rest on are still only on the cluster
Cluster shutdown ~08-24 (2 days from the newest doc). Outstanding: r2d `BEST_selected.pt` +
`ckpt_scores.tsv` for s50-53 (the entire dense 4/4 result), dv3 `latest.pt` for rr_dH_s2 and
rr_dR2D_s1 (the entire "dv3 sign of life"), the `.out` result lines, and RUN_REGISTRY.jsonl
(ROUND_ROBIN_RESULTS_2026-08-22 "Gaps"). Meanwhile the proposed next work — dense × all sources
(9+ cells), multi-policy-per-WM (a 1-day build plus K×WM training), dv3 at 600k — is several
GPU-weeks. Two of those results currently exist only as a number in wandb with no checkpoint behind
them; if the disks go, the paper loses the ability to re-evaluate its own headline cells.

**Fix, in order:** (1) pull artifacts today; (2) freeze the matrix — no new *cells*, only
identification arms; (3) spend remaining local GPU on issue 6's four RLPD arms (100k each, the
dense verdict shows 3 local seeds are affordable) and on a coverage-matched rescore of surviving r2d
checkpoints; (4) spend zero-GPU time on the distributional analysis (below), which is a promised
contribution that has never been delivered.

---

## 1. Confound inventory (acknowledged vs not)

**Acknowledged in the docs (with pointers):**
- Pruned vs unpruned human set for BC; `dHallpruned_DP` registered as the density control
  (ROUND_ROBIN_2026-08-20 "NOTED FOLLOW-UP"; RESULTS_MATRIX notation row).
- Action-label fidelity 13.3% vs 0.000% (CLUSTER_ROUND Amendment 2; AUDIT_normalization C1).
- Reward density 9.2x and no-pick share 33% vs 0% (AUDIT_normalization:223-241).
- dR2D = 52 pixel demos for WM vs 66 state tapes for RLPD/DP (ROUND_ROBIN_2026-08-20 §4.1).
- dDP DP row trains on 63 successes; RL/WM consume all 93 (§4.2).
- Same-machine rule for cluster vs local absolute rates (§4.4).
- State-based (RLPD/DP) vs pixel-only (r2d/dv3) (ROUND_ROBIN_RUNNING §"Observation and action spaces").
- Pooling across E1/E2 env boundaries and across code eras (RESULTS_MATRIX "code-era caveats").
- Demo-RNG defect, pair-dH re-execution, cell-B scoring bug (AUDIT_rng, AUDIT_run_identity,
  AUDIT_ms_chain) — all struck or corrected.
- Shaped runs: demo transitions keep sparse labels while online transitions carry shaping
  (ROUND_ROBIN_RUNNING §dense, registered pre-data).

**NOT acknowledged, or acknowledged in one doc and contradicted in the headline:**
1. **Demo-buffer size 83k / 70k / 9k transitions** treated as a nuisance; it is a 9x difference in
   the effective dataset for a learner whose batches are 50% demo data. Never appears in a caption.
2. **Fail-tape share as a design variable** (33% / 51% / 0%). Only surfaced on 08-22 as a
   *finding*; it was an uncontrolled difference in every RL/WM cell from the start.
3. **WM dH set appears pruned/success-only (67)** while the docs assert it is unpruned — issue 2.
4. **Eval horizon 400 (RLPD) < demo time-to-pick (~800)**, and 400 vs 600 vs 1200 across algorithms
   — issue 7.
5. **Teacher-competence confound in the BC row.** Student rank (dR2D 0.96 > dDP 0.80 > dHpruned 0.62)
   is monotone in teacher consistency (champion 0.91-1.00 > DP teacher ~0.67 > human teleop replays
   with 13% unrepresentable labels). "Model demos are better demonstrations" and "demonstrations
   from a more consistent, in-simulator, same-embodiment policy are easier to clone" are not
   separated by any run in the matrix.
6. **The newest BC headline is cross-machine.** dHpruned_DP/dDP_DP evals date 08-02..08-09;
   dR2D_DP ran on the cluster 08-20. The project's own standing rule is "official numbers only from
   same-machine baselines" (METHODOLOGY §1) and replay is documented as machine- and load-dependent
   (METHODOLOGY §2). The 0.96/0.80/0.62 ordering is presented in ROUND_ROBIN_RESULTS §3 and
   PAPER_NOTES N3 with only a passing "same-machine rule" note.
7. **DP evals are nondeterministic** — wandb_eval never seeds torch, so diffusion denoising noise is
   unseeded (AUDIT_rng F4). Per-seed DP numbers carry an unquantified eval-noise component.
8. **Random-IC eval sets differ per seed** (AUDIT_rng F5), so cross-arm random-IC comparisons at
   different seed counts (n=8 vs n=3) are on different episode sets.
9. **r2d checkpoint-scoring coverage changed between waves** — issue 4; nowhere noted.
10. **dR2D_R2D is self-distillation** (r2dreamer trained on its own champion's rollouts). FIGNOTES
    earlier called this cell "not meaningful"; it is now cited as evidence about model demos in
    general. Whatever it shows is entangled with self-distillation.
11. **No demo-free control anywhere.** No SAC-without-demos, no r2dreamer-without-prefill. The paper
    cannot currently claim demonstrations help at all, only that different demonstrations differ.
12. **The demo pipeline does not apply the env's tip-termination rule to demo tapes**
    (train_sacfd_full.py:273-292) — an MDP inconsistency affecting every fails-consuming arm.
13. **PAPER_PLAN's decision log stops at 2026-08-10** while calling itself "the paper's single
    source of truth". Registered decisions from 08-15 onward live in six other files. For a project
    whose integrity machinery is a stated contribution, that is a hole a reviewer will find.

## 2. Statistical adequacy — claim by claim

REVIEW_GUIDE "headline claims", graded:

1. *BC: model demos beat human in-dist at matched N (0.80 vs 0.62, P=0.994, n=8/arm); no
   generalization gain (0.23 both).* — **Supported as a descriptive contrast**, over-stated as a
   source claim. "Matched N" is episodes, not frames; the human arm is pruned (a stated conservatism
   for the comparison, but it changes the action-density of the training set); the causal reading is
   confounded with teacher consistency (issue 1, confound 5). The "no generalization gain" half is
   now falsified by dR2D_DP random 0.76 — which itself is n=3 and cross-machine.
2. *RLfD (RLPD): no demo-set effect at n=16/arm (0.50 vs 0.62, p=0.72).* — **Over-stated.** It is a
   null between two datasets differing 9x in size and density; power to detect 0.50→0.80 at n=16/arm
   is ~50%. And it is now contradicted by dDP 0/6. Correct statement: "dH and dR2D produce
   indistinguishable ignition rates despite large differences in dataset composition; dDP does not."
3. *World models: r2dreamer 0.91-1.00 on human demos via seed+checkpoint lottery (8/34); model-demo
   arm 0/20 (p~0.02); dv3 null with both MS controls positive.* — **Partly over-stated.** The
   pooled 8/34 crosses E-boundaries and budgets (1M for waves 1-2, 3M for wave 3), which the
   project's own ledger rule forbids for citable numbers; same-era per-wave contrasts are
   3/10-vs-0/10 (p=0.21) and 1/10-vs-0/10 (p=1.0). Direction is consistent across four waves, which
   is the honest thing to report. The dv3 "null" is retired by the 08-22 transients (two 5/6 draws),
   so claim 3's last clause must change.
4. *Every RL/WM implementation is positively controlled on ManiSkill; RLPD's MS failures are
   sparse-specific.* — **False as written** (issue 3).
5. *Collapse-after-ignition seen independently in r2dreamer and RLPD.* — **Supported as an
   observation** (2 r2d waves + RLPD clean-long + 2/4 dense seeds). No mechanism is established;
   keep it descriptive and do not let it carry the multi-policy proposal's weight.

Other statistical exposure: seeds per cell are 8/8/3 (DP), 16/16/6/6 (RLPD), 34-pooled/20/3-4/4
(r2d), 3/2/2/3 (dv3). Four headline cells are n≤4. Two cells have no eval at all (r2d dR2D s43, dv3
dR2D s0) and two dv3 cells finished with training picks that no eval ever measured — those must be
reported as missing data, not as zeros.

## 3. Direction — what to run with the compute that remains

**Keep (in priority order):**
1. Artifact rescue from the cluster (not an experiment; the paper's headline cells depend on it).
2. The dDPsucc / dDPtiptrunc / **dR2D+DPfails** / dDPlen900 RLPD identification set (issue 6). This
   is the only proposed work that converts an observation into a mechanism, it runs locally at 100k
   steps, and it either produces the paper's central causal claim or kills it cleanly.
3. Free analyses: within-arm Q-vs-ignition dose-response; tape-geometry census on the real dH set;
   the demo-set census that settles issue 2; **the H2 distributional analysis** (action entropy,
   smoothness, DTW diversity, time-to-pick) — promised as contribution 2 since 2026-07-31, never
   delivered, needs no GPU, and is exactly what makes the "which tape property matters" thesis
   quantitative rather than anecdotal.
4. A coverage-matched rescore of surviving r2d checkpoints (issue 4).

**Cut or defer:**
- **Multi-policy-per-world-model (ALGORITHM_STATE §5).** It is a new research program (a day of
  build, K× training runs) aimed at a phenomenon — post-ignition collapse — that is not this
  paper's question. It also cannot be finished before the cluster dies. Defer to the follow-up
  paper; the collapse observation stands on its own as a limitation.
- **dv3 at 600k.** dv3 has produced two 6-episode transients in the entire project. A longer budget
  buys, at best, a cell that still cannot carry a claim. The defensible move is to report dv3 as a
  documented negative result with the MS-HEAD control, plus the two transients as an anecdote, and
  stop spending on it. If anything is spent, spend it on a fresh-process re-eval of the surviving
  rr_dH_s2 checkpoint — that is a 30-minute job that determines whether the dv3 row says "null" or
  "transient", and it is currently the difference between an honest cell and an unverifiable one.
- **Dense × all sources (the §3.5 round-robin framing).** Attractive but it multiplies the matrix at
  a moment when no cell is identified. If any dense cell is run, run exactly one: dense on dR2D for
  r2dreamer (registered in ROUND_ROBIN_RESULTS §5 as a cheap add) — it tests whether the model-demo
  WM null is basin-entry or something else, i.e. it is an identification arm, not a new cell.
- **The noise-injection stretch arm (H3).** Moot; drop it from the plan explicitly.

**Missing that should be added if anything is:** a demo-free RL control (one seed each of RLPD and
r2dreamer with no demos) — a reviewer will ask "do demonstrations help at all here?" and there is
currently no answer; and a frame-matched (not episode-matched) BC arm.

## 4. Framing — the most defensible thesis

The pre-registered arc ("BC inherits demo-source differences; world models route around them", H4)
is **refuted by the project's own data, in the opposite direction**: BC is the arm that is
*indifferent-to-favorable* toward model demos (0.62 → 0.80 → 0.96), and the world model is the arm
that discriminates hardest (8/34 human vs 0/23 model). Say so. An inverted pre-registered hypothesis
with the data to show it is a better paper than a confirmed one.

**Recommended thesis:** *Demonstration-source effects are learner-specific and are driven by
measurable tape properties — idle-frame density, action representability in the learner's action
space, and the geometry and termination-labeling of failure tapes — not by "human vs model" per se.*
Sub-claims it supports, all within-algorithm:
- BC improves monotonically with demonstrator consistency (human < DP teacher < champion), and part
  of that gap is idle-frame density (dHallpruned is the control that quantifies it).
- Learners that consume failures inherit the failure *geometry* of their teacher: human failures are
  near-misses adjacent to the success manifold; a policy teacher's failures are long divergent
  excursions that leave the state distribution the training MDP ever visits. This is a genuinely
  new, checkable statement about self-training data — and it is the most interesting thing in the
  project.
- Under sparse reward every RL/WM learner on this task is a seed lottery; potential-based shaping
  raises basin entry without raising the ceiling and without touching post-ignition collapse.

**Claims to drop or downgrade:**
- "Model demos beat human demos" as a quality claim (confounded; keep as the descriptive BC row with
  the density and teacher-consistency caveats, and only after the same-machine re-run).
- H4 as stated — report as refuted, with the direction reversed.
- "No demo-source effect for RLfD" — replace with the composition-specific statement.
- "Every implementation is positively controlled" — false for RLPD.
- "dense collapses the r2d lottery (4/4)" — hold until coverage-matched; lead instead with
  time-to-first-pick, which is protocol-independent.
- "dv3 ignites on genesis" — not claimable from two 6-episode draws with no archived checkpoint.
- Any cross-algorithm quantitative comparison (different obs spaces, budgets, horizons, eval
  protocols). Present the matrix as four within-algorithm experiments that share a task and three
  datasets, not as one 4x3 factorial.

## 5. Two process notes
- PAPER_PLAN.md claims to be the single source of truth and its decision log ends 08-10. Fold the
  registered decisions from CLUSTER_ROUND amendments, ALGORITHM_STATE §3, and the ROUND_ROBIN docs
  back into it, or demote the claim. The integrity machinery is a stated contribution; its ledger
  should not be the least current document in the repo.
- Several audit items remain OPEN and touch cited numbers: the encoder gate (`sacfd_delta_gate.py`)
  has been broken at HEAD since 2fbed2a, so the entire n=16 RLPD wave launched with no encoder gate
  (AUDIT_normalization M1); the re-aggregation of nb/hold/mref sweeps from raw logs was flagged as
  needed "before the paper cites them" (AUDIT_ms_chain C5) and is not confirmed done. Close or
  disclose both before submission.
