# Item 12 — Does the paper confuse demonstration QUALITY with demonstration SOURCE?

**Status: DONE** (Fable lane, 2026-09-17/18). **Short answer: yes, systematically.** The thesis (C1/C2 in the intro) is
about *source* (human vs machine, and which machine), but roughly half of the sentences that carry the argument are
*quality* sentences (noise, pauses, mistakes, "precise and clean", success rate). The four robot datasets differ on both
axes at once, and the paper never says which of its comparisons holds which axis fixed. Below: (1) the confound structure
of the data as it actually is, (2) a sentence-by-sentence classification, (3) a defining paragraph for the intro/method
that names both axes and the controls, (4) before→after rewrites, (5) a Limitations paragraph, (6) discrepancies.
`\llm{}` notes inserted at the structural locations only.

## 1. What actually varies between the datasets (the facts the prose must respect)

| set (paper name) | source | success at `home` | idle / pauses | filtering | starts |
|---|---|---|---|---|---|
| Human | 40 untrained people, teleop, re-executed in the twin | 12/74 state, 13/74 pixels (**≈17 %**) — *attenuated by re-execution: every tape succeeded in reality* | idle fraction **0.365** (0.393 pre-pick) | every recoverable success-labelled tape kept (74/75); the 16 operator-labelled failures excluded by label | 74 real starts |
| DP | DP policy trained on the **pruned** human set (no-pick tapes dropped, idle collapsed), rolled out at the human starts, **first attempt** per start (de-selected) | 12/72 state, 14/72 pixels (**≈18 %**) | idle fraction **0.007** | none post hoc (first attempt kept) | 72 of the human starts |
| Planner | scripted motion planner in the twin | 67–68/72 (**94 %**); all failures pre-pick | ~0 | none | same 72 starts |
| R2 (teacher) | R2Dreamer policy trained on the human set, rolled out | 68/72 (**94 %**) per the paper's Table (`04:87`) | ~0 | (not stated in the paper) | same 72 starts |

Sources: `HRI_results/DEMO_SETS_2026-09-11.md` §1 (provenance, idle 0.365 at line 35), §"What the contrast therefore is"
("The arms differ in source AND in two pruning effects (idle collapse; the 10 no-pick tapes absent from the teacher's
data)"), §3 (phase counts); CLAUDE.md e2e-agent status 2026-09-07 ("idle fraction 0.365 human (0.393 pre-pick) vs 0.007
machine"); `paper/CONFOUNDS.md` row 26 ("Machine generator = self-distillation of the human set"), row 15 (idle_frac human
vs machine 0 is partly *provenance* — the DP teacher emits continuous deltas so exact zeros are impossible — not only
behaviour), row 21 (attempt/verify asymmetry); `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md` (67/72 complete; all
five failures before pickup); `appendix/a-research-methods.tex:26–29` (re-execution attenuates the human set);
`HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §3 (5) ("tapes are ~80 % failures" for human/DP, "94 % successes" for
planner/teacher).

**Consequently the four sets form two quality tiers:** {Human, DP} at ≈17 % complete and {Planner, R2} at ≈94 %.

| comparison | source differs? | success rate differs? | idle/style differs? | what it can show |
|---|---|---|---|---|
| Human vs DP | yes (human vs learned imitator) | **no** (17 v 18 %) | yes (0.37 v 0.01; partly recorder provenance) | the paper's cleanest **source** contrast — but the DP set is a self-distillation of the pruned human set, so "machine" here means "a re-execution of the human behaviour by a policy" |
| Planner vs R2 | yes (scripted vs learned) | **no** (94 v 94 %) | no | a **machine-source** contrast at matched quality |
| DP vs Planner / R2 | yes | **yes** (18 v 94 %) | yes | *confounded*: any gap is source × quality |
| Human vs Planner / R2 | yes | **yes** | yes | *confounded* (and this is the pair the Discussion argues from) |

**Which results are therefore source effects and which are quality effects (HANDOFF §3 eval numbers):**
- **DP** 0.03 / 0.02 / 0.57 / 0.60 (Human/DP/Planner/R2): flat within each quality tier, a step between tiers →
  **quality**, not source. The paper's "DP does better with the precise and clean demonstrations" is right only if
  "precise and clean" means "94 % successful"; the within-tier nulls (0.03 v 0.02; 0.57 v 0.60) say source does not
  matter to DP once success rate is fixed.
- **DfD** 0.31 / 0.35 / 0.04 / 0.28: Planner and R2 have identical success rates, yet DfD gets 0.04 v 0.28 → a genuine
  **source** effect that quality cannot explain (n = 1 seed on R2, so hold it lightly). Human v DP null → no source effect
  at matched quality.
- **RLPD** 0.00 / 0.16 / 0.00 / 0.00: it succeeds only on the *low-success* DP set and fails on the *high-success*
  planner and R2 sets → a **source** effect running *against* quality (candidate mechanism: RLPD needs failures /
  coverage near its own rollouts; the planner set has no post-pick failures — PLANNER_MATCHED_STARTS: "provides little
  evidence about recovery from failed placement or slide starts").
- **R2Dreamer** 0.59 / 0.53 / 0.54 / 0.56: indifferent to both axes.
- **PushT** (Table `tbl:pusht`): DP\_DP 43.9 % > DP\_DfD 3.0 % although the DfD demonstrations are *more* successful
  (81 % v 56 %) → **source over quality**, the cleanest such row in the paper; DfD\_Human 81 % v DfD\_DfD 92 % with 86 %
  v 81 % demo success → indifferent to both.
- **PinPad4**: IBC/VMAIL improve with the converged DfD data; the text attributes this to "pauses and mistakes"
  (quality). Human and DfD data differ in both axes there and no matched-quality control exists, so it is *not
  separable* in the sim tasks.

So the paper's *evidence* for C1 ("source is a critical factor") is actually: (a) RLPD's preference for the DP set over
better planner data; (b) DfD's planner failure at matched quality; (c) PushT DP\_DP > DP\_DfD. Its *prose* for C1 instead
leans on DP's human→planner jump, which is the one result that is a quality effect. That is the confusion to fix.

## 2. Sentence classification

Legend: **S** = source claim; **Q** = quality claim; **S/Q** = conflates (asserts one while the evidence or wording is the
other); **✓** = already separates the two correctly (keep, reuse).

| loc | sentence (abridged) | class | note |
|---|---|---|---|
| `abstract.tex:3` | "dependence … on the **quality** of their demonstrations ('garbage in, garbage out'), little work has compared different demonstration **sources**' impact" | S/Q | Opens by equating the two axes in one breath. Say: quality is well studied; source, *holding quality fixed*, is not. |
| `abstract.tex:3` | "World Model RLfD … learns as well from **noisy** human demonstrations as any other source" | S/Q | "noisy" is a quality attribute attached to the human source; the result being cited is a source null at matched quality (Human v DP). |
| `abstract.tex:3` | "**Noisy** demonstrations from untrained humans are significantly easier to obtain than demonstrations from trained human beings or converged models" | Q | Motivation; fine if the paper then names noise as one axis. |
| `main.tex:49` | same "quality … sources" opening | S/Q | as above |
| `main.tex:49` | "LfD algorithms differ in performance when trained on demonstrations that come from trained models rather than human beings" | S | ok |
| `01:24` | "LfD models have proven performant with **expert** demonstrations, but less work has been done with **untrained humans**, trained models, and classical robotics approaches" | S/Q | expert/untrained is a quality axis; trained-model/planner is a source axis; the sentence lists them as one axis. |
| `01:25` | "between demonstration sources **of similar quality**" | S/Q | **False** for the robot study (17 % v 94 %). Either restrict to Human-v-DP and Planner-v-R2 or delete. |
| `01:36` | "**Temporally correlated noise** and unobserved context can create spurious relationships" | S-mech | a hypothesised *mechanism* by which source acts; fine as hypothesis. |
| `01:36` | "Human and AI demonstrations may consequently differ in ways that are **not apparent from task-success metrics alone**" | ✓ | This is the correct framing (source ≠ success). Promote it to the defining paragraph. |
| `01:40` | "Do current LfD methods work with the data available to **regular users**?" | S/Q | "regular users" = untrained = low quality *and* human. Acceptable as HRI motivation if the axes are named first. |
| `01:47` | C1 "Demonstration-source is a critical factor" | S | but the evidence cited later for it is DP's quality jump (see §1). |
| `02:6` | "demonstrations … can also insert bias or **destructive noise**" | Q | fine |
| `02:10` | "Simulating **noisey** human decision making … Boltzmann" | Q | quality-as-noise model; not used in the paper. |
| `02:10` | "Human demonstrations have been shown to have **non-markovian noise** that may effect how LfD algorithms perform" | S-mech | hypothesis, fine. |
| `02:10` | "Mandlekar … considered demonstration **quality and provenance** … proficient-human, multi-human, and **mixed-quality** SAC … offline RL performed much better with **machine** demonstrations" | S/Q | The cited work *separates* the axes (PH v MH = quality; human v MG = provenance) and the summary collapses them into "machine". Also our own robomimic replication found the RLPD MG advantage was a quantity/coverage effect (CLAUDE.md 09-08: MG718s ≈ human, MGall > human) — worth one clause. |
| `03:33` | "Following standard practices DP trains on **successful demonstrations only** and **prunes out zero-actions**" | Q (design) | This is a quality *manipulation* applied to one learner's human set only; stated as an aside. See Discrepancy D1: HANDOFF says the pixel DP human arm trains **raw**. |
| `03:50` | "RLPD benefits from both **successful and failed** demonstrations" | Q | fine; it is the mechanism behind RLPD's DP-set preference. |
| `04:17` | "Both datasets retain **unsuccessful attempts**" | ✓ | good — a quality control, stated. |
| `04:24` | "All three algorithms beat **human performance** when trained on … DfD" | Q | "human performance" = demo success; ok. |
| `04:30` | "When trained on **85.8 % successful** human trajectories, DfD … 81 % … DfD did even better when trained on those 81 % successful trajectories" | ✓ | Reports demo success beside each source — the right practice. |
| `04:30` | "DP performed better on its own demonstrations than on DfD's demonstrations" | S | **and** the DfD demos are higher quality, so this is source over quality — say it. |
| `04:30` | "the human demonstrations tend to attain high coverage even when they're **not successful**" | Q | characterization; fine. |
| `04:67` | "forty **non-expert** users" | S(+Q) | fine; expertise is a quality co-variate of the human source. |
| `04:71` | "recreated in sim with **attenuated success**: 65/74 picked, 40/65 placed, 12/40 slid" | Q | Crucial: the human set's low success is largely a re-execution loss ("every one did in reality", appendix). The Discussion's "human demonstrations contain mistakes" reads the attenuation as human error. |
| `04:75` | "R2Dreamer because it was our **best performing** RLfD algorithm" | Q (design) | generator chosen by performance → the R2 set is high-quality by construction. |
| `04:78` | "removing **failed** demonstrations and **zero-actions** … Once trained on the **cleaned** human dataset, DP predictably performed similarly to its demonstration distribution" | S/Q | Reveals that the "DP" *source* is a quality-filtered re-execution of the human source. Must be said where the sets are defined, not in a DP aside. |
| `04:80–90` phase table | success by phase per source | Q | This **is** the quality axis. Unlabelled, unreferenced. |
| `04:110` caption | "DP and RLPD benefit from **machine** demonstrations (learned or planned) while WMs benefit similarly from all" | S/Q | DP's gain is the quality tier; RLPD's is DP-set-only (and anti-quality); DfD fails on the planner. |
| `05:5` | "DP does better with the **precise and clean** demonstrations that come from motion planning and a performant converged model" | S/Q → Q | "precise and clean" is a quality attribution presented as the source finding. Within a quality tier DP is source-indifferent. |
| `05:5` | "RLPD can only complete the task when given DP demonstrations" | S | ✓ and it is *against* quality — the paper's best C1 evidence, unremarked. |
| `05:7` | "DfD … learns equally well from the human, DP, and R2Dreamer demonstrations, but does not learn well from motion planning" | S | ✓ (Planner = R2 in quality, so this is pure source). Say so. |
| `05:15` | "DP … accommodates … distributions that include **pauses**" | Q-mech | fine as mechanism |
| `05:15` | "Human demonstrations have **less consistent behavior and more noise**, and this disrupts learning for one-step learners" | S/Q | asserted, not measured (the characterization subsection is empty); and on the robot task DP's failure on human data co-occurs with a 17 % success rate, which is the simpler explanation. |
| `05:17` | "VMAIL and IBC's steady-state reward is **limited by the quality** of the input demonstrations … The human demonstrations contain **pauses and mistakes** … converged DfD … **more ideal** behavior distribution" | Q | Explicit quality claim. Legitimate — but then PinPad4 is evidence for "quality matters to BC", which is the "garbage in, garbage out" proverb the abstract says is already known, not for C1. |
| `05:17` | "while it learns **faster** from the higher-scoring demonstrations, DfD attains similar rewards on both sources" | ✓ | separates rise-time (quality-sensitive) from asymptote (source-indifferent). Good; generalise to the robot (R2 on planner ignites late, catches up — HANDOFF §3 (2)). |
| `05:17` | "learns enough … from the **non-ideal** human demonstrations" | Q | fine |
| `05:20` | "baselines that work well on both **sources**" | S | ✓ |
| `05:24` | "There's value in **human-noise**" | Q | unsupported — nothing in §5 isolates noise. |
| `05:33` | conclusion | S | ok; inherits the DP-quality evidence problem. |
| `appx:94,101` | "IBC performs better with trained-policy demos than with Human demos" | S | ok (sim: axes not separable, say "trained-policy demos, which are also more successful"). |
| `appx:26–29` | "Re-execution is not lossless … 12 reach the goal, although every one did in reality" | ✓ | The single most important quality fact; should be in the main text next to the phase table. |

Tally: 9 sentences conflate (S/Q), 12 are quality claims presented inside a source argument, 5 already separate the axes
correctly. Title "The Noise is a Signal" is a quality-axis title on a source-axis paper.

## 3. Where to separate the two — a defining paragraph

Place after `01:24` (replacing the "that variation" sentence) or as the first paragraph of `03-method.tex` §3.1. It names
the axes, states which comparisons control which, and pre-empts the "garbage in, garbage out" objection.

```latex
Demonstrations vary along two axes that are easy to run together. \emph{Quality} is what the tapes achieve:
their task success rate, and the pauses, detours and mistakes along the way. \emph{Source} is who or what
produced them: an untrained person, a policy trained by imitation, a policy trained by reinforcement learning,
or a scripted planner. Quality is the axis the ``garbage in, garbage out'' proverb refers to and is well
studied; source is not, and the two are confounded in practice because machine generators are usually chosen
for being good at the task. We separate them by construction. The four robot datasets share the same 72
starting configurations and fall into two quality tiers: the human set and the DP set complete the task on
about 17\% and 18\% of tapes, while the planner and R2Dreamer sets complete it on 94\%
(Table~\ref{tab:phases}). Human-versus-DP and planner-versus-R2Dreamer are therefore \emph{source} contrasts
at matched success rate; DP-versus-planner and human-versus-planner vary source and quality together and we
read them only as a joint effect. The DP set is not an independent generator: it is a DP policy trained on a
cleaned copy of the human set and replayed at the human starts, so the human-versus-DP contrast measures what
a learner loses or gains when human behaviour is re-executed by an imitator with the pauses removed.
```

If the intro is too long, a two-sentence version for `01:25`:

```latex
Demonstrations vary in \emph{quality} (how well the tapes do the task) and in \emph{source} (who or what made
them), and the two are usually confounded because machine generators are chosen for being good at the task.
We hold quality fixed where we can---human versus DP demonstrations complete the task at the same rate, as do
planner versus R2Dreamer demonstrations---and read the remaining contrasts as joint source-and-quality effects.
```

## 4. Sentences to rewrite (before → after)

1. `abstract.tex:3` — "Despite the near-proverbial dependence … on the quality of their demonstrations (``garbage in,
   garbage out''), little work has compared different demonstration sources' impact on LfD algorithms."
   → "The dependence of LfD on demonstration *quality* is proverbial (``garbage in, garbage out''); the dependence on
   demonstration *source*—who or what produced the tapes, at a given quality—has had little attention."
2. `abstract.tex:3` — "World Model RLfD, which learns as well from noisy human demonstrations as any other source"
   → "world-model RLfD, which at matched success rate learns as well from untrained-human demonstrations as from
   machine demonstrations, and (for R2Dreamer) as well from low-success as from near-perfect sets".
3. `01:24` — "LfD models have proven performant with expert demonstrations, but less work has been done with untrained
   humans, trained models, and classical robotics approaches"
   → "LfD has proven performant with expert human demonstrations. Less is known about demonstrations from untrained
   people, and about whether a learner cares *who* produced its demonstrations—a person, a trained policy or a
   planner—once their quality is accounted for."
4. `01:25` — "between demonstration sources of similar quality" → "between demonstration sources, with quality matched
   where the data allow (Sec.~\ref{sec:algos})".
5. `02:10` — "they found that offline RL performed much better with machine demonstrations"
   → "they found offline RL did better on the SAC-generated set than on the multi-human set; source and quality vary
   together in that comparison, and a later quantity-matched replication attributes most of the gap to coverage".
6. `03:33` — "Following standard practices DP trains on successful demonstrations only and prunes out zero-actions to
   prevent state-action aliasing."
   → "Following standard practice, the DP *teacher* that produces the DP demonstration set was trained on the human set
   with failed tapes and zero-action segments removed. The DP *learner* in the comparison trains on the same raw sets as
   the other learners [**verify**: HANDOFF §3 (5) says the pixel human arm is raw; `04:78` implies cleaned]."
7. `04:78` — "Once trained on the cleaned human dataset, DP predictably performed similarly to it's demonstration
   distribution."
   → "Trained on the cleaned human set, the DP teacher reproduces the human set's success rate almost exactly (18\% v
   17\% at the goal, Table~\ref{tab:phases}) while removing its pauses (command-idle fraction 0.007 v 0.365). The DP
   set is therefore a re-execution of the human behaviour by an imitator, at matched quality."
8. `04:89` caption — "Demonstration phase success (\%) broken down by task phase. Each source contains 72 demonstrations."
   → "Success by task phase for each demonstration set (\%). The human and DP sets form a low-success tier and the
   planner and R2Dreamer sets a high-success tier; comparisons within a tier hold quality fixed. [fix n: human = 74]"
9. `04:110` caption — "DP and RLPD benefit from machine demonstrations (learned or planned) while WMs (R2Dreamer and
   DreamerV3) benefit similarly from all demonstration data."
   → "DP follows the success rate of its data (near zero on the human and DP sets, 0.57--0.60 on the planner and R2 sets)
   and is indifferent to source within a tier. RLPD completes the task only from the DP set, not from the more
   successful planner or R2 sets. R2Dreamer is indifferent to both source and quality; DfD is indifferent to source
   between human, DP and R2 data but fails on planner data of the same quality."
10. `05:5` — "DP does better with the precise and clean demonstrations that come from motion planning and a performant
    converged model, while RLPD can only complete the task when given DP demonstrations."
    → "DP's performance tracks the success rate of its demonstrations rather than their source: it is equally poor on
    the human and DP sets and equally good on the planner and R2 sets. RLPD shows the opposite pattern—it completes the
    task only from the DP set and not from the more successful planner or R2 sets—which is a source effect that quality
    cannot explain."
11. `05:7` — add after "…but does not learn well from motion planning demonstrations.":
    "Because the planner and R2 sets have the same success rate, this is a source effect, not a quality effect."
12. `05:15` — "Human demonstrations have less consistent behavior and more noise, and this disrupts learning for one-step
    learners like IBC and VMAIL."
    → "We hypothesise that the human demonstrations' pauses and less consistent behaviour disrupt one-step learners such
    as IBC and VMAIL; in the simulated tasks the human and machine sets also differ in success rate, so the two
    explanations are not separated there."
13. `05:17` — "This result cleanly demonstrates the benefit of using MBRL over LfD"
    → "This shows the benefit of MBRL over behavior cloning on *low-quality* data: the model learns the dynamics and
    reward from non-ideal demonstrations and then improves on them in imagination." (Label it as the quality result it
    is.)
14. `05:24` — "There's value in human-noise" → cut, or: "The only place we can separate noise from success rate is
    PushT, where DP learns more from its own lower-success demonstrations than from DfD's higher-success ones; whether
    human noise itself carries signal remains open."
15. Title — if the results stand, "The Noise is a Signal" promises a noise (quality) result the paper does not deliver;
    "World Models Prefer Human Demonstrations" is also not what the tables show (R2: 0.59 v 0.53 v 0.54 v 0.56). Flag
    for the authors; no suggestion offered beyond noting the mismatch.

## 5. Limitations paragraph (for `05:29`, replacing "Your text.")

```latex
\paragraph{Source and quality are only partly separable.} Our four robot datasets fall into two quality tiers, so
only two of the six pairwise contrasts hold success rate fixed. The DP set is a self-distillation of the human
set (a DP policy trained on a cleaned copy of the human tapes and replayed at the same starts), not an independent
machine generator; it differs from the human set in pauses and idle time as well as in source, and part of that
idle gap is an artefact of how the two recorders emit zero commands. The human set's low simulated success rate
(17\%) is largely a re-execution loss---every recorded trial succeeded in reality---so ``human demonstrations
contain mistakes'' should be read as ``human demonstrations, re-executed in the twin, mostly fail at the
slide.'' The DP learner's human set was cleaned for the teacher; whether the learner itself trained on cleaned
or raw tapes must be stated per experiment. All learners were trained and evaluated in the digital twin.
```

## Discrepancies

| id | paper says | notes/code say | evidence |
|---|---|---|---|
| D1 | `03:33`, `04:78`: DP trains on successful demonstrations only, zero-actions pruned ("cleaned human dataset") | pixel 4×4: "The human arm trains on raw demonstrations … a pruned pixel set has not been built"; the *teacher* (state DP) was trained on pruned human data | `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §3 (5); `paper/CONFOUNDS.md` row 26; `DEMO_SETS` §1 machine-set provenance |
| D2 | `01:25`: sources "of similar quality" | 17 % v 94 % `home` between tiers | `DEMO_SETS` §3; `04:84–87` |
| D3 | `05:17`, `05:15`: human demos contain "mistakes", "more noise" | human tapes all succeeded in reality; sim failure is re-execution attenuation concentrated in the slide | `appendix/a-research-methods.tex:26–29`; `DEMO_SETS` §1 |
| D4 | `04:78`, `04:110` (todo): idle/zero-action contrast presented as a behavioural difference | idle_frac human v machine 0 is partly recorder provenance (DP teacher emits continuous deltas; human follower re-issues reached waypoints) | `paper/CONFOUNDS.md` row 15 |
| D5 | `02:10`: Mandlekar — "offline RL performed much better with machine demonstrations" | our robomimic replication: RLPD MG200s < human, but MG718s ≈ human and MGall > human → quantity/coverage, not source | CLAUDE.md 09-07/09-08 robomimic entries; `tables/robomimic.tex` (not \input) |
| D6 | `04:87`: R2Dreamer set 94.4 at every phase | not verified by me; the R2 teacher handoff is `HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md` (not read this lane) | — |

## \llm notes inserted
1. `sections/01-introduction.tex:25` (shared with item 8) — "sources of similar quality" is false; two axes; pointer to §3 paragraph.
2. `sections/01-introduction.tex:47` — C1's evidence is a quality effect for DP; the source evidence is RLPD/DfD-planner/PushT DP\_DP.
3. `sections/02-related-work.tex:10` (shared with item 8) — Mandlekar summary collapses quality and provenance.
4. `sections/03-method.tex:33` — DP cleaning = quality manipulation on one learner; D1 discrepancy.
5. `sections/04-evaluation.tex:78` — the DP set is a matched-quality re-execution of the human set; say so here.
6. `sections/04-evaluation.tex:89` — the phase table is the quality axis; two tiers; caption n.
7. `sections/05-conclusion.tex:5` — "precise and clean" is quality; RLPD row is the source evidence.
8. `sections/05-conclusion.tex:17` — quality claim; label it.
9. `sections/05-conclusion.tex:29` — Limitations draft (shared with item 8).

## Sources
- Live `.tex` files (all, read in full).
- `HRI_results/DEMO_SETS_2026-09-11.md` §1 (line 35 idle fraction 0.365), §"What the contrast therefore is", §2 predicates, §3 counts.
- `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §3 (numbers, verdicts 1–6), §4.
- `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md` (67/72 completions, failures pre-pick).
- `paper/CONFOUNDS.md` rows 15, 21, 26 (`grep -nE '^\| (15|21|26) \|'`).
- `/home/james/workspace/genesis_pickaplace/CLAUDE.md` Agent Status entries: e2e DP+RLPD agent 2026-09-07 21:20 (idle 0.365 v 0.007), robomimic 09-07/09-08, sim-box 09-09.
- `appendix/a-research-methods.tex:22–34`.
