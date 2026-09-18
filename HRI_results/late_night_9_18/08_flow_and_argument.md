# Item 08 — Flow, transitions, and reiteration of the central points

**Status: DONE** (Fable lane, 2026-09-17/18). Whole paper read in order; each section is annotated below with (a) where a
paragraph lacks a transition or the argument is not carried, (b) where the prose has drifted from the current results
(`HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §3, reproduced below), (c) orphan/placeholder text, and (d) a short
rewrite suggestion per location. `\llm{}` notes were inserted at the structural locations only (list at the end).
Item 12 (quality vs source) is a separate file; where a flow problem is *caused* by that conflation I say so and defer.

## The two central points (from `01-introduction.tex:45–49`)
- **(i)** Demonstration source is a critical factor in LfD performance; algorithms meant to be trained by humans must be
  evaluated with human demonstrations.
- **(ii)** World-model RLfD is robust to demonstration source and is therefore a good neutral baseline for comparing source
  effects.

## The numbers the prose has to agree with (HANDOFF §3, eval chart = 30 random starts, sampled actions, mean `home`)

| learner | Human | DP (machine) | Planner | R2 teacher |
|---|---|---|---|---|
| DfD (DreamerV3 losses) | 0.31 (8) | 0.35 (8) | 0.04 (7) | 0.28 (1) |
| R2Dreamer | 0.59 (8) | 0.53 (8) | 0.54 (7) | 0.56 (2) |
| RLPD | 0.00 (8) | 0.16 (8) | 0.00 (6) | 0.00 (1) |
| DP | 0.03 (4) | 0.02 (4) | 0.57 (3) | 0.60 (2) |

HANDOFF §3 verdicts that matter for the prose: (1) human v machine, both WMs: no difference (p 0.64 / 0.29; registered
16v16: p 0.97 / 0.09). (2) **R2Dreamer learns equally well from all four sets** (planner catches up by 1.5–2M; the
earlier "planner hurts R2" statement was withdrawn). (3) **DfD does poorly on planner data** (0.04 v 0.31, p 0.01).
(4) **RLPD: human gives zero on every seed**; machine 3/8 seeds; planner ~never; teacher fastest in training.
(5) **DP tracks the success rate of its data** (human/DP tapes ≈80 % failures → 0.03/0.02; planner/R2 ≈94 % successes →
0.57–1.00). (6) Observation control: state never reaches `home`, pixels ignite 64/64 WM seeds.

So: "world models are agnostic" is true of **R2Dreamer** and true of **DfD for human-vs-machine**, but **false for DfD on
planner data** (0.04). "Learns as well from noisy human demonstrations as any other source" is true for R2Dreamer and
for DfD (human 0.31 is its second-best source). Every sentence below is checked against this.

---

## Abstract (`main.tex:49` inline; `frontmatter/abstract.tex:3` file) — two versions, both drift

- **Duplication.** Two abstracts are typeset (item 7 A1). They make *different* claims: the inline one says "LfD algorithms
  differ in performance when trained on demonstrations that come from trained models rather than human beings … DfD can
  effectively learn regardless of the demonstration source"; the file one says "do not uniformly benefit from one or the
  other … World Model RLfD learns as well from noisy human demonstrations as any other source". Pick one.
- **Drift.** "regardless of the demonstration source" / "as any other source" — DfD on planner = 0.04. Only R2Dreamer
  supports the unqualified sentence. → *"…a world-model RLfD system (R2Dreamer) learns comparably from every source we
  tested, and a second (DfD) from every source but the motion planner, so world-model learners can serve as a
  source-neutral baseline."*
- **Missing (i)-half.** Neither abstract states the practical consequence (evaluate with human data if meant for humans),
  which the intro and conclusion call the first implication. Add one clause.
- The inline abstract's dangling sentence (item 7) breaks the opening.

## 1 Introduction

- `01:24→25` **No transition into the thesis.** Para 1 ends on "…World Models." and the next sentence starts "In this
  paper, we examine how that variation shows up" — *that variation* was never introduced. Insert a bridge sentence that
  names the two axes (source; quality) — see item 12's defining paragraph — then "In this paper we ask how *source*…".
- `01:25` "between demonstration sources of similar quality" — **false for this study** (planner 94 % vs human 18 % at
  `home`; item 12). If kept, the intro promises a control the paper does not have. → *"between demonstration sources,
  holding the task, starts and learner fixed"*.
- `01:34` Para 3 argues that human and machine data may differ; `01:36` para 4 argues *why* (Markov / context); `01:38`
  para 5 states what we do; `01:40` para 6 states what we find. Paras 3–4 are fine. **Para 5 → 6 lacks the link "why HRI
  cares"**: `01:40` opens "For HRI, this work addresses a practical concern." with no connective. → *"This matters for
  HRI because the demonstrations a deployed robot will actually receive come from untrained people, not converged
  policies. Do current LfD methods work with that data?"*
- `01:40` **Drift.** "world model systems are agnostic to every task-dataset combination we tested" — DfD/planner 0.04
  refutes it; `\todo{Is this still true…}` already flags it. → *"and that one world-model learner, R2Dreamer, is
  indifferent to source on every task–dataset combination we tested, while the other, DfD, is indifferent on all but
  one."*
- `01:45–49` The two implications are the paper's spine but are stated once here and never echoed with the same wording
  again until the conclusion. Recommend (a) numbering them **C1/C2** here and (b) closing every results subsection with
  one sentence that scores C1/C2 (see §5 notes).
- `01:47` implication (i) — "Demonstration-source is a critical factor … and they should be evaluated with human
  demonstrations" — is the *quality/source* conflation point: as written, the evidence for "critical factor" is DP 0.03
  → 0.57, which is a *quality* effect (item 12).
- Commented block `01:43` ("The current results show that the observed human-versus-machine contrast depends on the
  learner…") is more accurate than the live `01:40`; consider reviving it.

## 2 Related Work

- `02:6` One paragraph defines BC and RLfD, then ends "…can also insert bias or destructive noise into the training
  process." with no forward link. → add *"Which sources introduce which biases is the question we take up."*
- `02:8–10` "Demonstration Provenance and Human Noise" — lists (a) Boltzmann noise models, (b) non-Markovian human noise,
  (c) Mandlekar et al. Three unconnected sentences; the paragraph never says what *this* paper adds beyond "did not look
  at machine demonstrations from different sources or MBRL". → end with *"We extend that comparison to two further
  machine sources (a motion planner and a world-model policy) and to world-model learners."* Also: the Boltzmann sentence
  is about *simulated* human noise, which this paper does not use (`04:136` todo says a noise injection was *not* done) —
  either cut it or say why it is relevant.
- `02:10` "they found that offline RL performed much better with machine demonstrations" is a quality/provenance mix
  (Mandlekar's SAC data is "mixed-quality"); item 12.
- `02:12–13` World-model paragraph closes on "…useful comparison with other offline imitation learning methods that may be
  more affected by demonstration source" — this **is** central point (ii) and is the one place in Related Work the thesis
  appears; good. But the claim "does not learn the multi-step value function … and so is not subject to the bias" is
  wrong for Dreamer (it has a λ-return critic) and the *actual* mechanism the paper later invokes (`05:17`: the WM learns
  dynamics+reward from non-ideal data and then improves on it in imagination) is a different argument. Make the two
  agree.

## 3 Method

- `03:8–17` No section lead-in before the figure; the opening paragraph states the design cleanly. `03:17` "Second, we
  compare the demonstrations themselves" — the *first* of the two comparisons is never labelled "First"; and the
  demonstration characterization it promises is one sentence in §5 (`04:130–139`). Either deliver it or drop the promise.
- `03:19–23` Demonstration pipeline: says "we record a trained algorithm's rollouts to produce a machine-demonstration
  set" — but §5.2 uses **three** machine sets (DP teacher, planner, R2 teacher) and the sim tasks use a DfD teacher.
  Figure 3 shows DP→DP only. → one sentence: *"On the robot task the machine sets come from three generators: a DP
  policy trained on the human set, an R2Dreamer policy trained on the human set, and a motion planner; in the simulated
  tasks the generator is a converged DfD."*
- `03:23` The naming rule (RLPD\_DP) is stated but **never used** in §5 (tables say "Human / DP / Motion Planning /
  R2Dreamer"; PushT table says "DfD / Human"). Lane 15's item; flagged here because it breaks the reader's mapping.
- `03:25–27` "Each environment uses the simplest algorithms that can make progress on their task" — good, but the
  paragraph does not say *why the sets differ* (IBC/VMAIL in sim, RLPD/R2 on the robot) in terms of the thesis. → *"The
  simulated tasks let us compare pure imitation learners; the robot task adds two RLfD learners because there a strong
  non-imitation baseline is available through the twin's reward."*
- `03:33` DP paragraph ends "…trains on successful demonstrations only and prunes out zero-actions" — this is the single
  most consequential design fact for the quality/source question (item 12) and it is buried as a "standard practice"
  aside. It should be stated with its consequence: DP's human set is not the same set the other learners see.
- `03:50` RLPD paragraph: "where a strong non-imitation baseline is needed and dense reward is not available" — the
  reward *is* available (sparse +10) and RLPD is not "non-imitation" (half of every batch is demonstrations). Rewrite:
  *"RLPD is the model-free RLfD arm; it uses the same sparse reward as the world models."*
- `03:53–71` Robot figure sits under the RLPD paragraph with no prose referencing it (only `04:63` does, via the
  duplicated label).

## 4/5 Experiments (`04-evaluation.tex`)

- `04:11–16` **Orphan outline** ("Sec. 5 Experiments / Sec. 5.1 …") typesets as prose. Delete.
- `04:17` Sim data paragraph names "DfD, RLPD, and Diffusion Policy" — RLPD is robot-only (`03:50`), IBC/VMAIL are the
  sim learners; also "100 human demonstrations from paper authors" vs the abstract's "inexperienced participants" (the
  participants are the robot task only — say so). → *"In both simulated tasks the human set is 100 demonstrations from the
  authors; the machine set is 100 rollouts of a converged DfD trained on that human set. DfD, IBC, VMAIL (PinPad4) and DP
  (PushT) train on the same fixed sets, eight seeds each."*
- `04:19–22` Empty `Tasks` / `Data Collection` subsubsections; the intro figure caption is the only task description.
- `04:23–26` PinPad4: results reported as a p-value list with no sentence that returns to C1/C2. → close the paragraph:
  *"PinPad4 therefore shows both effects: the imitation learners' reward moves with the source (C1), and DfD's does not
  (C2)."*
- `04:26` "did not have a meaningfully significant impact on DfD (p=0.011)" — under any conventional α this *is*
  significant; the sentence undercuts C2 unless the corrected threshold is stated.
- `04:28–30` PushT: "Pusht shows a similar pattern" is a good transition. But the paragraph ends on coverage-reward
  detail and never says what the DP/DP self-distillation row *means* for the thesis (DP learns better from its own
  source than from a better-performing DfD source — that is a *source* effect that is *not* a quality effect, and it is
  the cleanest evidence for C1 in the paper). → add: *"DP\_DP (43.9 %) versus DP\_DfD (3.0 %) is the clearest source
  effect we observe: the DfD demonstrations are more successful (81 % vs 56 %) yet DP learns far less from them, so the
  effect is not explained by demonstration quality."*
- `04:61–63` Restocking subsection opens on task mechanics with no bridge from the sim results. → *"The simulated tasks
  use author demonstrations and one machine generator. The robot task adds untrained demonstrators and three machine
  generators."*
- `04:70–71` Digital-twin paragraph: first sentence is garbled (item 7); the funnel "65/74 picked, 40/65 placed, 12/40
  slid" is a *quality* statement about the human set and should be tied to the phase table two paragraphs later (they
  currently give different numbers — item 7 Discrepancies).
- `04:73–78` Machine demonstrations: "R2Dreamer because it was our best performing RLfD algorithm" — the result is
  being used before it is reported; fine if signposted ("Sec. 5.2.5"). The DP paragraph `04:78` ends mid-sentence and
  contains the crucial fact that the DP machine set is distilled from the *cleaned* human set (item 12).
- `04:80–90` Phase table: never referenced, no lead-in sentence, caption does not say what it is for. It is the paper's
  **quality axis** (item 12) and should be introduced as such: *"Table X characterizes each set by phase success; the
  two learned sources match the human set (≈16 % complete), the planner and R2 sets are near-perfect (94 %). Source and
  success rate are therefore confounded between the {Human, DP} and {Planner, R2} pairs but not within them."*
- `04:92–93` Results paragraph is a settings paragraph; there is **no results prose at all** for the robot task — the
  table and figure are dropped in with `\todo{UPDATE}` and never discussed. The reader gets no sentence like "RLPD never
  completes the task from human data" or "DfD fails on planner data". Minimum text (from HANDOFF §3):
  ```latex
  Table~\ref{tab:phase-success-pixel-sparse10-sampled-eval} and Figure~\ref{fig:pickaplace_perf} give the eval
  success by phase. R2Dreamer completes the task from every source at a similar rate (0.53--0.59), and DfD from
  the human, DP and R2 sets (0.28--0.35) but not from the planner set (0.04). RLPD completes it only from the DP
  set (0.16; three of eight seeds) and never from human data. DP tracks the success rate of its data: near zero on
  the human and DP sets, whose tapes are mostly failures, and 0.57--0.60 on the planner and R2 sets. Human versus
  DP demonstrations, the two sets with matched success rates, show no difference for either world model
  ($p=0.64$, $p=0.29$).
  ```
- `04:108–112` **Figure caption drift.** "WMs (R2Dreamer and DreamerV3) benefit similarly from all demonstration data"
  — DfD/planner 0.04 vs 0.31. → *"R2Dreamer benefits similarly from all four sets; DfD from all but the planner set."*
  Also "DP and RLPD benefit from machine demonstrations (learned or planned)" — RLPD gets **0.00** on planner and R2
  teacher data; only the DP set works. → *"DP benefits from the high-success machine sets (planner, R2); RLPD only from
  the DP set."*
- `04:121–125` `fig:training_phases` (ignition/rise-time) — not referenced, not discussed; `03:17` promised
  "performance rise-time, and likelihood of learning" as two of three comparison axes. Either add a paragraph (HANDOFF
  §3: R2 on planner ignites late but catches up by 1.5–2M; RLPD ignites in 13–27k decisions then loses `home`) or drop
  the promise.
- `04:130–139` "Characterizing the Demonstration Sets": one sentence, two author notes, two unreferenced figures. This is
  where the *quality* axis would be described (idle fraction, duration, coverage) and it is empty. The `\elaine{}` note
  gives the framing; a three-sentence draft is in item 12.
- **Missing closer for §5.2.** No sentence returns to C1/C2 before the Discussion.

## 6 Discussion / Limitations / Conclusion (`05-conclusion.tex`)

- `05:5` Opening paragraph is the best statement of the thesis in the paper. Two drifts: "DP does better with the
  precise and clean demonstrations" is a quality attribution (item 12); "World Model approaches are much less sensitive"
  is right for R2, qualified for DfD — the very next paragraph says so, so the two paragraphs should be one.
- `05:7` "DfD … learns equally well from the human, DP, and R2Dreamer demonstrations, but does not learn well from motion
  planning demonstrations. R2Dreamer … is demonstration-source agnostic" — **correct** and the most accurate sentence in
  the paper; but it contradicts the abstract/intro "agnostic to every combination". Propagate this qualification upward.
- `05:7→9` "…interesting and useful for several reasons." followed by the fragment "makes it helpful for characterizing
  the performance of LfD algorithms." The list of reasons was never written. Draft:
  ```latex
  World Model RLfD's robustness to demonstration source is useful for two reasons. First, a learner that scores the
  same from human and machine data can be run on whichever is cheaper, which for a new task is usually a handful
  of untrained human demonstrations. Second, such a learner is a neutral yardstick: when a source-sensitive
  algorithm is evaluated beside it on the same sets, any gap can be attributed to the algorithm rather than to
  the demonstrations. DfD and R2Dreamer provide that baseline here.
  ```
- `05:12` Truncated paragraph ("…results are repo"). Its intended point ("this is typically not examined when results are
  reported") is repeated at `05:20`; merge and delete one.
- `05:15` Mechanism paragraph: **internal contradiction** with `01:36`. Intro: machine trajectories "are not necessarily
  Markovian from the learner's observed state" (because policies use context/recurrence). Discussion: "IBC learns better
  from trained-policy demonstrations because they are actually generated by markov processes with respect to the
  observable task state." Both cannot stand. The DfD teacher *is* recurrent (RSSM), so the intro is right for it;
  resolve by saying the machine policies are *stationary* and *consistent* rather than Markovian. Also "DP is
  theoretically less affected by non-Markovian demonstrations because it's a sequence model … We speculate that Diffusion
  may be harmed by the Markovian nature of trained-policy demonstrations" — this predicts DP prefers human data, which
  matches PushT (55.8 vs 3.0) but is the **opposite** of the robot result (DP 0.03 human vs 0.57 planner). The paragraph
  needs the reconciliation: on the robot the human/DP sets are 80 % failures, so the quality axis dominates (item 12).
- `05:17` PinPad4 paragraph: good mechanism story; "limited by the quality of the input demonstrations" — quality, not
  source (item 12). Ends "benefit of using MBRL over LfD" (category error, item 7).
- `05:20` Restates C2 well ("baselines that work well on both sources … like DfD"). This is the reiteration the results
  sections lack; move a one-line version of it to the end of §5.1 and §5.2.
- `05:24` "Of course more data wins…" — informal register, no evidence in the paper (no data-quantity experiment is
  reported; the robomimic MGall control that *did* test quantity is not in the paper). "There's value in human-noise" —
  the paper's title claim ("The Noise is a Signal") rests on this sentence and nothing in §5 measures noise. Either cut,
  or tie to a result: the PushT DP\_DP > DP\_DfD row and the RLPD human-ignites-then-loses pattern are the only
  noise-adjacent evidence.
- `05:26–29` Limitations = "Your text." The HANDOFF §4 and `DEMO_SETS` §"What the contrast therefore is" list the
  limitations a reviewer will ask for: (1) DP trained on cleaned sets while others train raw; (2) machine sets are
  self-distillations of the human set (CONFOUNDS row 26); (3) evaluation in the twin only; (4) 8 seeds, some cells
  interim; (5) RLPD recipe gaps (no frame stack, 10 not 20 critic updates); (6) one human session, participant identity
  unknown (CLAUDE.md sim-box note 09-09: no participant identifier exists). Draft in item 12 for (1)–(2).
- `05:33` Conclusion restates C1 and C2 faithfully. It says "world model RLfD algorithms exhibit particular robustness"
  — plural, but only R2 is fully robust. → "…R2Dreamer is robust to every source we tested and DfD to all but one".

## Appendix

- `appx:36–48` "Training" has one sentence ("We clamp imagined returns to stabilize training.") and five empty
  subsections; items 2–5 lanes are filling these.
- `appx:50–86` Statistical appendix describes the OLD state-based study (T20/T50rel timing, "14 main performance
  comparisons", `dHfull`/`dDPfull` run tags, commit hashes) — none of which is in §5 now. The sensitivity table's rows
  ("Pick / DP", "E2E Settled nesting / R2", "T20") do not correspond to any live table. Either re-run the stats for the 4×4
  and regenerate the table, or cut this section to the permutation-test paragraph.
- `appx:89–103` Training curves: captions carry the thesis ("IBC and VMAIL perform better with trained-policy demos…
  DfD performs similarly with both") — good; the main text never points at `fig:results_pusht_train_return`.

## Summary of drift (prose vs HANDOFF §3)

| location | prose | result | fix |
|---|---|---|---|
| `abstract.tex:3`, `main.tex:49`, `01:25`, `01:40`, `04:110`, `05:33` | WMs agnostic to every source | DfD planner 0.04 v 0.31 (p 0.01) | qualify: R2 fully, DfD all but planner |
| `04:110` | RLPD benefits from machine demonstrations (learned or planned) | RLPD 0.00 on planner and R2; 0.16 on DP only | "only from the DP set" |
| `05:15` | DP harmed by Markovian machine data (PushT logic) | robot: DP 0.57/0.60 on machine sets | add the quality reconciliation |
| `01:25` | sources of similar quality | 18 % vs 94 % complete | drop "similar quality" or restrict to Human-vs-DP |
| `05:24` | "more data wins", "value in human-noise" | no quantity or noise experiment in the paper | cut or cite the PushT DP\_DP row |

## Discrepancies
See the table above and item 7's Discrepancies (demo-set counts). One more: `04:17` says the sim human set is from
"paper authors" while the abstract says "inexperienced participants" without scoping it to the robot task.

## \llm notes inserted
(anchors are short unique strings in the current tree; each note points here)

1. `sections/01-introduction.tex:25` after "demonstration-source agnostic." — drift + missing antecedent.
2. `sections/01-introduction.tex:40` after the `\todo{Is this still true…}` — the answer (R2 yes; DfD all but planner).
3. `sections/02-related-work.tex:10` end of the provenance paragraph — no "what we add" sentence.
4. `sections/03-method.tex:21` end of pipeline paragraph — three machine generators, not one.
5. `sections/04-evaluation.tex:16` after the outline lines — orphan outline.
6. `sections/04-evaluation.tex:17` — RLPD is not a sim learner.
7. `sections/04-evaluation.tex:30` end of PushT paragraph — DP\_DP v DP\_DfD is the cleanest source effect; say so.
8. `sections/04-evaluation.tex:93` end of Results settings paragraph — no results prose; draft supplied.
9. `sections/04-evaluation.tex:110` inside the figure caption todo — caption drift (DfD/planner; RLPD/planner).
10. `sections/05-conclusion.tex:9` at the fragment — orphan; draft of the "several reasons".
11. `sections/05-conclusion.tex:12` at the truncation — merge with `05:20`.
12. `sections/05-conclusion.tex:15` — Markov contradiction with `01:36`; DP prediction opposite to robot result.
13. `sections/05-conclusion.tex:24` — "more data wins" unsupported.
14. `sections/05-conclusion.tex:29` — Limitations placeholder; list of six.
15. `appendix/a-research-methods.tex:77` — stats appendix describes the old study.

## Sources
- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/` live `.tex` files (all, `cat -n`).
- `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §2, §3, §4 (numbers and verdicts quoted above).
- `HRI_results/DEMO_SETS_2026-09-11.md` §1, §"What the contrast therefore is", §3.
- `HRI_results/late_night_9_18/00_BRIEF.md`.
- `paper/CONFOUNDS.md` rows 15, 21, 26 (for the limitations list).
- CLAUDE.md sim-box status 2026-09-09 (participant-identifier note).
