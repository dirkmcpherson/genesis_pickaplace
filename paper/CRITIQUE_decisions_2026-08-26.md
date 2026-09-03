# ADVERSARIAL CRITIQUE — the assistant's decisions and interpretations, 2026-08-25 → 08-26

Reviewer stance: hostile reviewer of the **judgment exercised in the last ~24 h**, not of the
codebase and not of the historical results. Read-only: nothing was launched, cancelled, ssh'd or
modified except this file. Every claim cites a file:line, a manifest key, an artifact value, or an
arithmetic re-derivation done here with `~/.wandb-venv/bin/python`.

Scope read: `paper/PAPER_NOTES.md` (N1–N13, N12a), `paper/UPDATE_2026-08-25.md`,
`paper/POSTMAINT_COMMANDS.md` (top section), `paper/CRITIQUE_launch_plan_2026-08-25.md`,
`paper/gripper_lab_2026-08-25.md`, `paper/real2sim_follower_lab_2026-08-23.md` §9,
`paper/PREREG_final_round_robin_2026-08-23.md` §5/§6/§8/§11 + amendments,
`paper/SESSION_LOG_2026-08-23_cluster.md` (through the 08-26 07:50 row), `analysis/results_table.py`,
`paper/RESULTS_TABLE_2026-08-25.md`, `baselines/sim_variants.py`, `cluster/sbatch_r2dreamer.sh`,
`cluster/sbatch_rlpd.sh`, `~/workspace/dreamerv3-torch-genesis/{dreamer.py,models.py}`,
`~/workspace/final_rr_artifacts_2026-08-24/matched_v2/{census.md,MATCHED_SETS.json,r2d/*/repeat.json}`.

---

## VERDICT

The last 24 hours contain the project's best piece of physics (the contact-solver diagnosis), its
best piece of claim hygiene (publishing the 17× exposure arithmetic in the same breath as the null),
and its worst piece of experimental design (the exposure-matched arm). The self-correction culture is
real and working — N9, N11 and N12a are genuine, fast, unprompted retractions — but it is being
applied **backwards in time only**. Every note the assistant corrects is one the *user* challenged or
that the assistant re-derived from artifacts; the notes written in the same session as the correction
(N12, N13) reproduce the exact failure modes the corrections were about: an unstated readout with a
structural ceiling (N11's own rule, violated one day later), a cross-learner comparison run on a
different IC set from the row it is compared against (PREREG §5, violated silently), and a density
confound already ruled disqualifying for the same arm in N9. The headline mechanism sentence in N12
is **false as written** against the source — demo states are the roots of every imagined rollout and
do receive actor *and* critic gradient. And the 28 GPU jobs launched in a 12-minute window on
08-26 07:34–07:46 include one arm (BUFFER_MAX=40000) that cannot answer its own question and omit the
one arm (share-matched fails) that could, while the three top-ranked items of the critique the
assistant accepted 24 h earlier were never submitted and were then written off by the world change.

The world decision itself is **right for the wrong stated reason**: the physics argument (timeconst
8× the engine's stability floor, penetration ∝ timeconst²) and the free-tip-over result (10 recoveries
vs 2 losses, McNemar p ≈ 0.039) carry it; the "+1/66 pick recreation" that leads the decision table is
2 vs 1 discordant demos, McNemar p = 1.0, on a metric the report itself says flips between AVX2 and
AVX-512. The timing — freezing `shelf6` into the world of record while §7 of the same document says
the shelf height is unmeasured and undecided — is the weakest call of the window.

---

## ERRORS (wrong, and they change a conclusion)

### E1. N12's two rows are measured on **different evaluation IC sets**, and the WM row selects and reports on the same one

- RLPD `0.55 (8,14,4,7 /15)` and `0.15 (3,1,5,0 /15)` are the **`hold`** column
  (`paper/RESULTS_TABLE_2026-08-25.md`, rows *RLPD old sparse dR2D / dR2DDPfails*;
  `cluster/sbatch_rlpd.sh:275` confirms on `--sets hold,rnd`).
- The r2dreamer numbers are **`sel`**. `cluster/sbatch_r2dreamer.sh:237` sets
  `R2D_EVAL_EXTRA=${R2D_EVAL_EXTRA:---ic-file .../eval_ics.json --ic-set sel}` for the `DEMOSET=v2`
  branch, and the launch line (SESSION_LOG 08-25 16:36:40) does not override it.
- `sel` and `hold` are **disjoint uid sets**: `sel = [232,234,235,236,237,239,242,243,244,245,246,
  247,248,250,251]`, `hold = [252,254,256,265,273,276,284,294,295,302,311,325,327,331,335]`
  (`baselines/eval_ics.json`).

Three consequences:

1. **PREREG §5 violation.** "*headline per seed = the selected checkpoint on `hold` (15) + `rnd`
   (30)*". The WM cells report `sel` and have **no `rnd` at all**. This is the first r2dreamer
   results readout of the final block, which PREREG §11 says triggers a **Results audit** whose first
   required answer is "headline = selected-on-hold+rnd". It was not run.
2. **Selection and reporting on the same ICs.** `sbatch_r2dreamer.sh:524` picks `BEST` by
   `sort -k2 -rn ckpt_scores.tsv` — scores produced by the in-run archiver evaluating on `sel` — then
   :528-535 "confirms" that checkpoint on `sel` again with fresh *action-sampling* seeds. Fresh
   sampling seeds do not remove **IC-level** selection bias: a checkpoint chosen because it happens to
   solve those 15 ICs will re-solve them. RLPD selects on `sel` and reports `hold`. The two rows in
   N12's table are therefore not on the same footing even before the reward regime is considered.
3. **The reference arm is at its structural ceiling and N12 does not say so.** `sel` contains
   uid 234, which N11 established one day earlier is unachievable by construction (lying can, tip rule
   at decision 1, picked **0 times in 430 policy-evals**) — so 14/15 = **0.933 is the maximum**. Two of
   three dR2D seeds scored exactly 0.93. N11's own adopted rule (b) — "*any 'parity' statement must
   name the readout and its headroom*" — is violated by the note written the next day.

**Fix is nearly free:** the `BEST_selected.pt` files exist; `eval_genesis.py` already takes
`--ic-file/--ic-set`. Re-score on `hold` and `rnd` before N12 is quoted again.

### E2. The stated mechanism is false against the source: demo states **are** imagination roots and **do** receive critic gradient

N12: "*RLPD trains its actor and critic directly on demo transitions; a world model consumes them as
dynamics data and trains its actor in imagination, so those states never enter a TD target directly.*"

In `~/workspace/dreamerv3-torch-genesis`:
- `dreamer.py:259` — `start = post`, i.e. the **posterior states of the replayed batch**, demo rows
  included.
- `models.py:412-415` — `_imagine` does `start = {k: flatten(v) …}` over `(batch, time)`, so **every
  timestep of every replayed sequence, demo timesteps included, is an imagination root**.
- `models.py:384-393` — `value_input = imag_feat`; `value = self.value(value_input[:-1].detach())`
  and `value_loss = -value.log_prob(target.detach())`. Index 0 of `imag_feat` **is the demo state**.
  The actor loss (`:371-379`) likewise runs over `imag_feat[:-1]`, index 0 included.

So a demo state receives actor **and** critic gradient at exactly its sampling share of the replay
ring — not zero. The real distinction is narrower and should be stated instead:

> At a demo state, RLPD's critic target is built from the **recorded off-policy action and recorded
> next state** through a bootstrapped Q with a target network — the extrapolation channel N1
> identified. A Dreamer-family critic's target at the same state is a λ-return over an **on-policy
> imagined** rollout whose rewards come from a **supervised reward head** (`dreamer.py:260-262`), which
> is regressed on those same fail frames and predicts ≈0 there. There is no max/argmax over an
> off-distribution recorded action anywhere in the backup, so there is no channel for value inflation
> at those states — which is precisely why the fail tapes are free here and were not free to RLPD.

Caveat on this finding: **r2dreamer's own tree is not on this box** (only
`cluster/patches/r2dreamer_final_rr.patch`). The above is verified in `dreamerv3-torch-genesis`, the
same family; `paper/AUDIT_sources_2026-08-23.md` §4 records the r2d critic as trained in imagination
with `imag_horizon 15 / λ 0.95`. Verify against the cluster r2dreamer source before this sentence
enters the paper.

### E3. The exposure-matched arm cannot answer its own question, and its "matching" arithmetic compares two different quantities

Launched 08-26 07:34 as `BUFFER_MAX=40000`, seeds 90-93, both arms. Measured inputs (from
`matched_v2/r2d/*/repeat.json` and `matched_v2/census.md`, at `demo_duplicate 4`):

| | demo rows in ring | demo share @450k | demo share @40k | fail share of ring @40k |
|---|---|---|---|---|
| dR2D (control) | 4 × 1010 = 4,040 | 0.90 % | **10.1 %** | 0 |
| dR2DDPfails | 4 × 3,418 = 13,672 | 3.04 % | **34.2 %** | **24.5 %** |
| RLPD (for reference) | — | demos **50 %** of every batch | — | fails **35.8 %** of every batch |

**(a) The two arms are put at different demo shares.** 10.1 % vs 34.2 % — a 3.4× asymmetry. A drop in
the fails arm at 40k is confounded with *total demo share* independent of fail content. (The same
asymmetry exists at 450k, but at 1/11 the magnitude, which is why it did not bite in N12.)

**(b) "34 % matches RLPD's ~36 %" (SESSION_LOG NOTE 08-26 07:34) compares a demo share to a fail
share.** RLPD's 35.8 % is the **fail** share of every batch (`0.716 × demo_batch 128/256`); the
exposure arm's 34.2 % is the **demo** share of the ring, whose fail component is 24.5 %. To reach
RLPD's fail exposure you need `BUFFER_MAX ≈ 9,600/0.358 ≈ 26,800` — and the prefill assert
`dup × raw_frames < 0.5 × max_size` (13,672 < 13,400) **forbids it**. The arm is not exposure-matched
to the mediator it exists to match.

**(c) Shrinking the ring 11× is not a single manipulation.** It simultaneously changes: the online
replay window (staleness / recency bias), the **online capacity per arm** (26.3k rows for the fails
arm vs 36.0k for the control — a second asymmetry), and the magnitude of demo **reinjection shock** —
which this repo has already documented as destabilising: `MORNING_REPORT_2026-08-11.md:94` records
"*picks vanish right after 750k/900k reinjections — consistent with reinject-triggered value shock*".
At 40k a reinjection replaces 34 % of the ring in the treatment arm and 10 % in the control.

**(d) The clean manipulation was available, cheap, and already exposed as a knob.** Hold
`BUFFER_MAX=450000` and raise `DUPLICATE` (`sbatch_r2dreamer.sh:54,299`) so each arm reaches the same
demo share: `dup ≈ 45` for the fails arm (45×3,418 = 154k, assert 154k < 225k ✓) and `dup ≈ 151` for
the control (151×1,010 = 152k ✓). That matches exposure **without** touching staleness, online
capacity or shock magnitude. The cost is correlated duplicate rows — a far smaller confound than the
four introduced above.

**(e) And the arm that separates content from density was not launched at all.** N5 registered a
"share-matched fails arm (subsample fail transitions to ~26 %)"; the accepted critique promoted it
(BL-6, revised plan R5, "*only if R4's pair reads*"). R4 read. R5 was not run. Without it, N12's null
and the exposure arm's result both remain confounded with the **3.4× reward-density dilution**
(5.87 % → 1.67 % of rows rewarded, `matched_v2/census.md`).

### E4. Half of N13 carries exactly the confound N9 ruled disqualifying, and the note does not say so

N13: "*matched_v2 (N-matched 56, IC-matched, success-only, same tapes the other learners used) … the
first time the three sources are compared on a world model with matched sets and no fail-tape
confound.*" From `matched_v2/census.md` and the `repeat.json` stamps:

| | dH | dDP | **dR2D** |
|---|---|---|---|
| tapes | 56 | 56 | 56 |
| transitions | 7,002 | 6,909 | **954** |
| tape len p50 (decisions) | 115 | 114 | **17** |
| reward density | 0.00800 | 0.00811 | **0.05870** (7.3×) |
| dreamer-dir `total_reward` | 56.0 | 56.0 | **64.0** |
| demo share of the 450k ring @dup 4 | 6.2 % | 6.1 % | **0.90 %** (7×) |

- N9 already ruled the dR2D half of the RLPD source spread density-confounded on exactly these
  numbers. The same tapes are now being fed to a world model.
- The accepted critique's **W-4** says the +8 reward mass "*must not travel into any cross-source WM
  comparison without the footnote*". N13 is that comparison. The footnote is absent.
- The dR2D arm also differs 7× in **demo exposure** — the very mediator N12's own caveat calls
  decisive. So if dR2D wins or loses, it is uninterpretable as a source effect.

**What is clean, and should be the claim:** `dH` vs `dDP` are N-matched (56/56), IC-matched
(`MATCHED_SETS.json: matching.ic_multiset_identical = true`), length-matched (115 vs 114 p50),
density-matched (0.0080 vs 0.0081) and exposure-matched (7,002 vs 6,909 transitions). **That pair is
the first genuinely clean human-vs-model world-model contrast in the project.** N13 should have scoped
its prediction to it and labelled the dR2D arm exploratory.

### E5. The world change is not "one parameter on the grasp pair"; it moves every contact the can makes

`baselines/sim_variants.py:148-155` writes `sol_params` on the four finger geoms **and** the picked
can. Since Genesis averages a pair's params (`collider_decomp.py:1170-1181`, per gripper_lab §1.2),
the intervention is:

| pair | before | after |
|---|---|---|
| finger ↔ picked can | 0.020 | **0.005** (intended) |
| picked can ↔ **goal can** | 0.020 | **0.0125** |
| picked can ↔ shelf / table | 0.020 | **0.0125** |
| finger ↔ shelf / table | 0.020 | **0.0125** |

The picked-can ↔ goal-can pair is **exactly what the `contact` predicate tests**
(`genesis_can_env.py:271`, `bottle.get_contacts(goal)`), and the can ↔ shelf pair is what `nested`
and `placed` settle on. So "nested 17→20, contact 22→20, tipped 38→23" are **not** purely downstream
consequences of a better grasp; part of them is a direct 1.6× stiffening of the metric-bearing
contacts. The report's explanation ("*with less penetration the can rests a couple of mm further from
the goal and the strict test samples through fewer touching frames*", §5.3) is at best half the story,
and the TL;DR's "*Nothing else changes*" is not true. Everything else in the code is correct — see
CORRECT CALLS C2.

### E6. N12a still uses the wrong critical value, in the note whose point is the critical value

Re-derived here (`~/.wandb-venv/bin/python`, from counts 14/15, 9/15, 14/15 vs 12/15, 11/15, 12/15,
which reproduce the note's quoted means and sds exactly):

```
m1 = 0.8222  sd1 = 0.1925 | m2 = 0.7778  sd2 = 0.0385
diff = 0.0444  se = 0.11331  Welch df = 2.1597          <- all as stated, correct
exact two-sided permutation p = 20/20 = 1.000            <- correct
minimum attainable two-sided p at 3v3 = 2/20 = 0.100     <- correct
t(0.975, df=2)      = 4.3027   ->  CI [-0.4431, +0.5320]  <- what the note used
t(0.975, df=2.1597) = 4.0117   ->  CI [-0.4101, +0.4990]  <- the correct interval
```

4.303 is the **df = 2** critical value, not the df = 2.16 one. The note's interval is ~7 % wider than
correct, so it is conservative and **the conclusion is unchanged** — but the note closes with "*use
the RIGHT critical value for the Welch df*" while not doing so. Listed as an error because it is
factually wrong twice in a row in the same note; it does not flip the verdict.

Separately: **is Welch even the right instrument here?** No, and the note should say so. The per-seed
statistic is a proportion out of 15 bounded ICs, one of which is structurally unachievable (E1.3), and
it is a **max over archived checkpoints** — a biased, skewed order statistic whose sampling
distribution is not remotely normal. With n = 3 the variance estimate itself has a 95 % interval
spanning roughly 0.5–6× the point estimate. The permutation test (correctly reported, p = 1.00) and
the raw per-seed values are the honest readouts; the interval is decoration until the seeds land.

---

## OVERREACHES (stated more strongly than the evidence supports)

**O1. "PRIMARY RESULT" and "The WM is unmoved".** At n = 3/arm with the interval spanning
[−0.44, +0.53], this is a point estimate. N12a, written 13 minutes later, concedes exactly that
("*only the direction … NOT 'the effect is excluded'*"). The N12 heading and the word PRIMARY should
be demoted to match its own follow-up note.

**O2. "best-checkpoint evals at time_limit 400".** 400 is the **training** horizon
(`env.time_limit`). The evals ran at `EVAL_MAX_STEPS = 1200` sim steps
(`sbatch_r2dreamer.sh:187`, not overridden in the launch) — **3× the horizon the policy was ever
trained under**. That is a favourable, undisclosed protocol asymmetry.

**O3. "r2dreamer ignited 6/6 across both arms here, vs 8/34 sparse and 4/4 dense historically".**
6/6 counts only the seeds that had reported; 2 of the 8 launched were still running (one after a CUDA
crash and resubmission). And the historical comparators are not comparable: the audit caveat under N2
flags that ignition counts scale with checkpoint-scoring coverage, and the 4/4 was produced under the
**shaping-γ 0.999 vs discount 0.997 mismatch** that was patched out on 08-24 — the accepted critique's
§3 warned in terms that "*it is entirely possible that the fix removed the thing that produced 4/4*".

**O4. "Demos are ~3 % of r2dreamer's 450k replay ring".** 3.04 % is the **fails arm's** share
(13,672/450,000). The control arm is **0.90 %**. The two arms of the null differ 3.4× in demo exposure
even at the default buffer — a fact that matters for reading the null and is not stated.

**O5. N13's "that old result is now UNEXPLAINED".** Before that stands, check whether the 08-19 WM
`dDP` dir carried the same missing-terminal defect N1 found in the RLPD encoder. Those sets came from
the legacy `convert_genesis_demos_repeat.py` path, which `AUDIT_sources_2026-08-23.md` §4(b) records as
placing `is_terminal` **12 decisions after** the grant. More broadly, N12 (dense, terminal-guarded,
8 tapes at 3 % exposure, `sel`, 1200-step eval) and the 08-19 result (sparse, legacy encoder, 30 tapes,
different demo pipeline, different checkpoint coverage) differ on six axes; "N12 makes that WORSE, not
better" is a chain of inference across all six.

**O6. N1 still reads as a live finding whose central claim has been overturned.** Its "Framing for
the paper" says the dDP catastrophe "*IS a human-vs-model demonstration property — of the failures*".
`UPDATE_2026-08-25.md` §3c now says: "*The 08-19 dDP_RLPD catastrophe … was an **encoding bug**, not a
property of model demos … With terminals labelled, dDP ignites 4/4.*" N1 carries no SUPERSEDED banner.
A reader of PAPER_NOTES today takes N1 at face value.

**O7. N3 ("dR2D_DP is the best BC cell; model demos beat human in BC by another step") is the exact
claim N6/N11 attribute to a translation artifact.** No banner. Same for **O8: N2's 4/4** result, which
was measured under a reward function that no longer exists.

**O9. N6's "the corrected-world DP is the strongest policy of the program (rnd up to 19/30)".** That
is a single-seed max. On `rnd` **means**, N11's own table gives old-world `dR2D_DP` 0.61 > corrected
`dH_DP` 0.55. Direction asserted from a maximum, contradicted by the note two entries down.

**O10. N12 names no readout and no headroom** — the rule N11 adopted the previous day. Two of three
dR2D seeds are at 14/15, the structural ceiling.

**O11. "+4 seeds/arm … CI half-width should fall to ~0.20."** Verified: at n = 7 with the *observed*
sds, half-width = **0.178** (se 0.0742, Welch df 6.48, t 2.404). But sd₁ = 0.1925 is estimated from
three points and is driven by one 0.60 seed, and r2d ignition is a documented lottery (8/34 sparse).
If one of the four new dR2D seeds returns 0.0, sd₁ → 0.341 and the half-width → **0.315**. State the
projection conditionally on the variances holding.

**O12. Asymmetric weighting of two identically-noisy paired counts in the world decision.**
`UPDATE §4` leads its decision table with "*pick recreation (cluster) 57/66 → 58/66*" and justifies
adoption on "*it improves the metric we actually optimise (pick recreation)*". Paired, that is **2
recoveries (246, 301) vs 1 loss (316 or 254) — McNemar exact p = 1.00** — on a metric whose marginal
demos gripper_lab §5.1 itself says "*flip between this laptop (AVX2) and the cluster (AVX-512)*".
Meanwhile full-task open-loop `picked` moved the **other** way on a larger base (71 → 69; uids 255,
303, 316 lost, 331 gained) and was filed as a "*small unexplained regression*". Same statistic,
opposite treatment. The genuinely load-bearing count is free tip-overs: **10 recoveries vs 2 losses,
McNemar exact p ≈ 0.039** — that is the number the recommendation should have led with, alongside the
timeconst² physics.

---

## QUESTIONABLE DECISIONS (defensible, suboptimal — with what I'd have done)

**Q1. The accepted launch plan was never executed, and then the world change wrote it off.**
`POSTMAINT_COMMANDS.md`'s top section — written *after* accepting the critique — lists 56 jobs. From
the session log, only **STEP 1a's 8 N5 jobs** were ever submitted (16:36:40, 08-25). Never submitted:
- **1b, r2d disentangle (2 jobs)** — the critique's #1 item and the gate on the corrected-world dR2D
  teacher;
- **2a, 24 RLPD seed top-ups** — the critique's "*single largest increase in claim strength per
  GPU-hour available*", taking N6's null and N10's vanished spread from n = 4 to n = 10;
- **2b, 10 DP top-ups**; **2c, 12 N7 density jobs** — "*blocks any 'demo quality over idle density'
  claim*".

At 07:48 on 08-26 the world flip made all 46 of them obsolete (the UPDATE says so itself: "*46 of the
48 staged jobs run on the world this changes*"). Between 16:36 08-25 and 07:34 08-26 the queue held
8 long jobs against a plan that assumed ~13 concurrent GPUs. **What I'd have done:** submit 2a + 2b +
2c at 16:40 on 08-25. They are ~3 h each, were DRYRUN'd, and — critically — a *within-world* result
stays valid after a world change; `matched_w3` n = 10 would be a publishable "RLPD source parity in
world W" row regardless of what world came next. They would have banked overnight. Then flip the world
in the morning.

**Q2. 28 GPU jobs in a 12-minute window, no read-gates, and the justifying analysis was wrong at
launch time.** 07:34 exposure arm (8), 07:41 N13 (12), 07:46 N12 top-up (8) — and the N12a power
analysis that justifies the third batch was committed at 07:46 **with the wrong critical value** and
corrected at 07:55, after launch. The accepted critique's organising principle ("run WAVE 2 first
because it gates the others; require the success-only arm to ignite before reading anything") was
abandoned within 24 h of being adopted.

**Q3. Freezing `shelf6` while the shelf height is explicitly undecided.** `UPDATE §7`: "*One physical
measurement still outstanding: the real shelf surface height above the table, which decides `shelf6`
vs `shelf10`.*" The assistant's own argument for adopting the timeconst fix now — "*paying the rebuild
bill now is cheaper than discovering later that 48 jobs ran on a superseded world*" — applies with
equal force to the shelf, and `shelf10` scored the **best `nested` of any variant** in real2sim §9.1
(22/75 vs shelf6's 17). Because the provenance gate keys on the **variant name**, a later shelf
decision forces a third re-record even though pick physics is unaffected. **What I'd have done:** ask
the user for the tape-measure reading (minutes of user time) before starting `dH_w4` at 07:50.

**Q4. Choosing `g_stiff5` on differences the report itself says are noise.** The four stiffened
configs separate on `placed` (60 vs 51/52), `nested` (20 vs 18) and `tipped` (23 vs 27/28) — paired
counts on 75 demos, the same magnitude as the AVX2/AVX-512 flips §5.1 disclaims. On the two defensible
axes (penetration, monotone and physics-predicted; free tip-overs, p ≈ 0.039) `g_stiff5` and
`g_padcanstiff` are hard to separate, and `g_stiff_f10` uniquely holds `obs[7]` at the base world's
value (20.0 vs 18.4) — which matters because **`obs[7]` is in the learners' state vector** and every
policy that reads it must be refit (§6.4, §5.1 obs-distribution note). Not wrong; under-argued.

**Q5. N13's dR2D arm duplicates N12's dR2D arm knob-for-knob.** Seeds 100-103 and 80-87 are the same
cell (`ARM=dR2D DEMOSET=v2 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400`, default
`BUFFER_MAX`). That is efficient **only if pooled** — pooled it takes the dR2D arm to n ≈ 12 against
the fails arm's n ≈ 8, which is precisely what N12a says it needs. Treated as two separate cells under
two note numbers, it buys nothing extra.

**Q6. No PREREG amendment since A8 (2026-08-23).** Since then: `TIME_LIMIT` moved to 400 for the WM
block (which is *literally* PREREG §8's prescribed on-fail action after the 08-24 pilot missed its bar,
so it needs an amendment rather than silence); the WM headline moved off `hold+rnd` onto `sel` (§5
violation); the world of record changed twice; and N13 registered a prediction that sits across §6's
`P-R2D`. PREREG §2 ("*one commit hash for the block; a fix restarts that learner's block*") and §11's
**Change** and **Results** audit gates both apply, and neither was invoked.

**Q7. N12/N13 are outside the rule N9 created two days earlier.** `analysis/results_table.py` parses
only `SWEEP-HEADLINE` (RLPD) and `HEADLINE.txt` (DP) — no r2d path — and its world inference
(`10-14 old, 20-29 corrected`, :18) labels every N12/N13 seed `?`. So the first primary result after
"*results are henceforth quoted only from `analysis/results_table.py` output, never from
recollection*" is hand-greped from cluster `.out` files. Extending the tool to parse `R2D-RESULT` and
`confirm.log` is ~20 lines and would have caught E1 immediately (it would have had nowhere to put a
`sel`-only row).

**Q8. `sim_variants.post_build` prints its geom count but does not assert it** (`:156`). If link
naming or `geom_start/geom_end` ever changes, the variant silently degrades to
`gc_kp4_riser3_shelf6` while every manifest still stamps `_ts5`. Given six catalogued
silent-default/fallback bugs in this repo, `assert n_set == 5` is the obvious guard.

---

## CORRECT CALLS (so the user can calibrate)

**C1. The contact-solver diagnosis is the strongest single piece of work in the window.** `timeconst
0.02 s` against an engine floor of `2·substep_dt = 0.0025 s`, penetration ∝ `timeconst²`, plus the
correction that *released* genesis-world 0.2.1 overwrites `sol_params[0]` unconditionally while this
0.2.1+270 tree only `max()`-clamps it — that is a first-principles finding with a measured mechanism
and it, not any count delta, justifies the change. Adopting a stiffer contact is right.

**C2. The implementation is correct on every point I checked.** Setting `sol_params` on **both** sides
is necessary (Genesis averages and has no `priority`; `g_padstiff`, fingers-only, lands at the
predicted 0.011 s pair and only reaches 4.8 mm). **5 geoms is the right count** — four finger links
with exactly one collision geom each (`right/left_finger_prox_link`, `right/left_finger_dist_link`,
verified in `gen3_lite_2f_robotiq_85.urdf`) plus the picked can's single `Cylinder` geom. The **goal
can is correctly not touched** (`w['bottle']` only). The manual `tmin = 2*solver._substep_dt` clamp is
right and fails loudly rather than silently reverting, and the comment about
`set_global_sol_params()` being broken in this build is a useful piece of institutional memory.
(`describe()` omits `grasp_timeconst`, but it is dead code — nothing in the repo calls it.)

**C3. Running the N5 WM cells on the *same* old-world `matched_v2` tapes the RLPD numbers were
measured on.** Rebuilding them in the corrected world would break the pairing, the corrected-world
fails sets do not exist (`matched_w3/MATCHED_SETS.json: fails_arm_n_fails = 0`), and building them
means re-running the RLPD fails arms too. Correct, and correctly scoped as a mechanism result.

**C4. `TIME_LIMIT=400` and the ≥2/4 ignition read-gate.** Both came from the critique, both were
adopted, and both paid off: 6/6 ignition against a 0/2 pilot at 1200.

**C5. Publishing the exposure arithmetic in the same note as the null, unprompted.** "*So the null is
'a WM at 3 % demo exposure is unharmed', NOT 'world models are intrinsically robust'*" — with the
caveat marked pre-registered rather than discovered after the fact. This is the single best piece of
claim hygiene in the window and it is what makes N12 readable at all.

**C6. The N13 framing correction.** N12's two arms *are* both model demos; saying so to the user
unprompted, and refusing to let the 08-19 result stand as evidence for a WM source effect, is exactly
right. And the launched `dH` vs `dDP` pair genuinely is the first clean human-vs-model world-model
contrast in the project (E4).

**C7. N9 / N10 / N11 as a set.** Moving the BC claim onto `rnd`; refusing to present either the
old-world RLPD gap *or* its disappearance as a finding until one candidate is measured; identifying
uid 234 as a structural 14/15 cap from per-episode `sweep.json` records; and creating
`results_table.py` after a hand re-derivation went wrong. Model self-correction. The failure is that
N11's own rule was not carried forward one day.

**C8. Catching the N12a error in 13 minutes and stating plainly that the earlier claim "must not be
repeated".** Every quantity in the correction except the critical value re-derives exactly here: mean
0.8222 / sd 0.1925, mean 0.7778 / sd 0.0385, diff 0.0444, se 0.11331, Welch df 2.1597, exact
permutation p = 1.000, minimum attainable two-sided p at 3v3 = 2/20 = 0.100.

**C9. Declining N8 for the stated reasons** (no stride-1 *pixel* demos exist; `ACTREP` is hard-coded in
`sbatch_genesis_final_rr.sh`; B and C collide on the registry full key because `discount` is not in
`REG_KNOBS`). The critique's BL-4/BL-5 were right and were accepted **fully**, not partially — as were
BL-3 (ARM tables), BL-7 (the `matched_w3/dR2D` byte-identical placeholder, purged at 16:34 08-25),
W-1/W-2 (N7 onto `rnd`, seeds 20-22) and W-3.

---

## WHAT TO DO NEXT, ranked by value

1. **Re-score the WM cells on `hold` and `rnd`.** The `BEST_selected.pt` files exist and
   `eval_genesis.py` takes `--ic-file/--ic-set`. ~30 CPU-min per checkpoint. Until this runs, N12 is
   not comparable to any other row in the paper and violates PREREG §5. *Do this before N12 is quoted
   again.* (Fixes E1.)
2. **Pool seeds 80-87 with 100-103 for the dR2D cell** — they are the same configuration. That gives
   n ≈ 12 vs n ≈ 8, which is the one thing N12a says it needs; recompute the interval with
   `t(0.975, Welch df)`, and report the permutation p and the per-seed values alongside it.
3. **Launch the share-matched fails arm** (N5's own registered follow-up; BL-6/R5). Subsample fail
   transitions to ~26 % of demo rows at `BUFFER_MAX=450000`. Without it neither the null nor the
   exposure arm separates off-manifold content from the 3.4× reward-density dilution.
4. **Re-cast the exposure manipulation as `DUPLICATE` at fixed `BUFFER_MAX`** — `dup ≈ 45` (fails) /
   `≈ 151` (control) both satisfy the prefill assert and put the arms at a *matched* ~34 % demo share
   without changing staleness, online capacity or reinject-shock size. If the 8 running
   `BUFFER_MAX=40000` jobs come back with a drop, it will not be interpretable; consider re-purposing
   the allocation now rather than after 24 h.
5. **Get the shelf measurement before the `w4` rebuild finishes.** If it says `shelf10`, the `dH_w4`
   recording started 07:50 is wasted and a third rebuild follows.
6. **Census the 08-19 WM `dDP` demo dir for terminal flags** before "the 08-19 result is unexplained"
   goes anywhere near the paper (O5).
7. **Add SUPERSEDED banners to N1, N2 and N3, and the point-estimate qualifier to N6's "strongest
   policy of the program".** Three of the thirteen notes currently assert as live what later notes
   overturned. This is cheap and it is the highest-risk item for the paper text.
8. **Extend `analysis/results_table.py`** to parse `R2D-RESULT` / `confirm.log` / `DV3-RESULT`, to
   record the **IC set** each number came from, and to take the world from the demo dir's
   `sim_variant` rather than the seed block. The tool as written cannot represent the error in E1.
9. **`assert n_set == 5` in `sim_variants.post_build`**, and add `grasp_timeconst`/`shelf_dz` to
   `describe()` (dead today, but it is the obvious future provenance path).
10. **Thread `w['shelf_top']` into the env's `placed` predicate.** `genesis_can_env.py:266-267` still
    tests `BOX_TOP_Z + 0.01 < z < BOX_TOP_Z + 0.07` against the un-shifted 0.11 while the shelf top
    under `shelf6`/`_ts5` is 0.17 — a can correctly resting on the shelf sits at z ≈ 0.22 and scores
    `placed = False`. Inert in pick scope (the lab tools use `sim_variants.shelf_top()` correctly, so
    the gripper-lab numbers are sound), but it is a silent zero the moment place scope opens.
    `real2sim_follower_lab_2026-08-23.md` §9.3 already specifies the patch and it never landed.
    (`genesis_vec_env.py:203` has the same defect for the goal-can spawn height, but that module is
    reachable only from the unused `bridge_batched_gpu.py` — no action needed.)
