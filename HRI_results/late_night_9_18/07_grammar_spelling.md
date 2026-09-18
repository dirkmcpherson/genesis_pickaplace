# Item 07 — Grammar, spelling, typo and LaTeX-defect check

**Status: DONE** (Fable lane, 2026-09-17/18). Every live `.tex` was read in order (`main.tex`, `frontmatter/abstract.tex`,
`sections/01…05`, `appendix/a-research-methods.tex`, `appendix/b-online-resources.tex`, `tables/*.tex`, `back/*.tex`,
`frontmatter/ccs.tex`, `preamble.tex`). Ignored as instructed: `04-evaluation-bk.tex`, `06-todo.tex`,
`dataset_characterization.tex`, `appendix/c-llm-generated.tex`. Only obvious errors are listed; no restyling. No `.tex`
was edited for this item (the brief allows `\llm{}` notes only for items 8/12). Lines are as of the working tree at
the time of reading (`git log -1` = `c50d276`); other lanes are inserting `\llm{}` notes concurrently, so line numbers
may drift by a few lines — anchor on the quoted text, not the number. Two entries overlap other lanes' items
(11 "AI"→"machine", 13 "restocking", 14 "DfD", 15 naming) and are marked *(lane NN)*; they are listed here only where
the inconsistency is a plain error, not a naming policy.

## A. Defects that will BREAK or MISRENDER the compile (fix first)

| # | file:line | defect | fix |
|---|---|---|---|
| A1 | `main.tex:47–50` | **Two abstracts.** `\input{frontmatter/abstract}` (line 48) is immediately followed by a second, inline abstract (line 49). Both typeset inside `\begin{abstract}`, back to back. | Keep one. The inline one carries the `\elaine{cut by 9-10 lines}` note; the file one carries `\todo{still true?}`. Decide which is current, delete the other. |
| A2 | `sections/05-conclusion.tex:41` | `\jss{...}` — undefined control sequence (only `\js` exists in `preamble.tex:36`). Hard compile error. | `\js{...}`, or delete the line. |
| A3 | `appendix/a-research-methods.tex:87` | `\FloatBarrier` — needs `\usepackage{placeins}`; `preamble.tex` does not load it and `acmart.cls` does not either (`grep -c placeins acmart.cls` = 0). Undefined control sequence. | Delete the line (acmart discourages float nudging anyway) or add `placeins` to `preamble.tex`. |
| A4 | `sections/04-evaluation.tex:139` | `See appendix \label{appx:characterization_definitions} for precise definitions` — `\label` where `\ref` was meant; typesets as "See appendix  for precise definitions". | `See Appendix~\ref{appx:characterization_definitions}` — **and** the label does not exist anywhere in the appendix, so after the fix it is an undefined reference until that appendix section is written. |
| A5 | `sections/01-introduction.tex:67` and `sections/03-method.tex:70` | Duplicate `\label{fig:envs}` (the PinPad4/PushT figure and the robot/digital-twin figure). LaTeX warns "multiply-defined label"; `\ref{fig:envs}` at `04-evaluation.tex:63` ("The restocking task has three parts (Fig.~\ref{fig:envs})") resolves to whichever label was defined last, i.e. the wrong (sim) figure with `\input` order intro→method reversed. | Rename the method one to `fig:restock_envs` (or `fig:robot_twin`) and update `04-evaluation.tex:63`. |
| A6 | `sections/03-method.tex:17` | `Sec.~\ref{sec:experiments}` and `Sec.~\ref{sec:restock}` — neither label exists (defined labels are `sec:eval:results`, `sec:dig_tw`). Prints "??". | `\ref{sec:eval:results}` and a new `\label{sec:restock}` on the `\subsection{Real-World Experiment: Restocking}` at `04-evaluation.tex:61`. |
| A7 | `sections/03-method.tex:29` | `Appendix~\ref{appx:hyperparameters}` — label undefined. | Point at `appx:training` or add the label when the hyperparameter appendix (items 2–5) is pasted in. |
| A8 | `sections/04-evaluation.tex:80–90` | `\label{tab:phases}` sits **before** `\caption` (line 81 vs 89). A label before the caption picks up the enclosing section counter, so any `\ref{tab:phases}` would print "5.2.3", not the table number. (Currently nothing references it — see B-list.) | Move `\label{tab:phases}` to after `\caption{...}`. |
| A9 | `sections/04-evaluation.tex:93` | `can be found in \ref{appx:training}` — bare `\ref` prints just "B" with no "Appendix". | `in Appendix~\ref{appx:training}`. Same at `04-evaluation.tex:71` (`appendix \ref{appx:real2sim}` → `Appendix~\ref{appx:real2sim}`). |
| A10 | `sections/05-conclusion.tex:15` | `...distributions that include pauses~\citet{chi2024diffusionpolicy}.` — `\citet` used parenthetically; typesets "…pauses Chi et al. (2023)." | `\citep`. |
| A11 | `main.tex:65–67` | Template `\received{20 February 2007}` / `\received[revised]{12 March 2009}` / `\received[accepted]{5 June 2009}` left in; the comment above them says delete for a conference paper. Prints fake dates in the footer. | Delete. |
| A12 | `frontmatter/ccs.tex:13–31` | Placeholder CCS concepts ("Do Not Use This Code, Generate the Correct Terms for Your Paper") and keywords ("Do, Not, Use, This, Code, …") will print in the PDF. | Generate real CCS terms and keywords. |
| A13 | `back/acks.tex:12–14`, `back/ethics.tex:22–29`, `appendix/b-online-resources.tex:6`, `sections/05-conclusion.tex:29` | Template placeholder prose ("Identification of funding sources…", "Discuss the potential societal risks…", "Your text." ×2). | Write or delete. |
| A14 | `sections/04-evaluation.tex:11–16` | Section outline notes ("Sec. 5 Experiments / Sec. 5.1 Simulation Experiments / Subsections for Tasks, Data Collection, Results / …") are live prose and typeset as a paragraph under 5.1. | Delete or comment out. (Also item 8.) |
| A15 | `appendix/a-research-methods.tex:42` | `\subsubsection{Implicit Behavior Cloning}` nested under `\subsection{DreamerV3 and R2Dreamer}` while its siblings (DP, RLPD, VMAIL) are `\subsection`s — wrong level; IBC appears as a sub-part of the Dreamer entry. | `\subsection`. |
| A16 | `sections/04-evaluation.tex:139` (same sentence as A4) | "See appendix" — lowercase + missing `~`. | "See Appendix~\ref{…}". |
| A17 | `sections/02-related-work.tex:6,10` | ` ~\citep{…}` / ` ~\citet{…}` with a space *before* the tie, and at line 10 the citation starts a sentence (`. ~\citet{mandlekar2021matters} looked`) → stray leading space/non-breaking space. Cosmetic, but visible. | Remove the space before `~`; at the sentence start use `\citet{...}` with no tie. Same pattern `03-method.tex:50` `(RLPD) ~\citep`. |
| A18 | `sections/02-related-work.tex:13` | `imitation loss\cite{staley2024agent}` — no tie/space before `\cite` (glues the citation to the word). | `loss~\cite{...}`. |

## B. Cross-reference hygiene (not errors, but will draw referee comments)

- `04-evaluation.tex` defines `tab:phases` (demo-phase table), `tab:phase-success-pixel-sparse10-sampled-eval` (main results
  table), `fig:pickaplace_perf`, `fig:training_phases`, `fig:coverage`, `fig:tool` — **none is referenced in the prose**.
  The main results table and the main results figure are never pointed to from the text.
- `tables/e2e_performance.tex`, `phase_performance.tex`, `robomimic.tex`, `secondary_performance.tex`, `timing.tex` are only
  `\input` from the ignored `04-evaluation-bk.tex`; only `tables/sensitivity.tex` is live. Its caption
  (`a-research-methods.tex:84`) says "no timing comparison survives its six-comparison correction" but no timing table
  is in the paper any more; the sensitivity table's rows (Pick/DP, Place/RLPD, E2E …, T20, T50rel) are the OLD
  state-based study, not the 4×4 pixel study reported in Section 5. (Content point → also in item 8.)
- `\cite` vs `\citep` are mixed (`02:13`, `03:37`, `04:71`, appendix use `\cite`; elsewhere `\citep`). acmart+natbib
  makes them equivalent, so no misrender — consistency only.
- Table label prefix `tbl:pusht` vs `tab:*` elsewhere — cosmetic.

## C. Spelling, grammar, typos — `file:line — "before" → "after"`

### main.tex (inline abstract) and frontmatter/abstract.tex
- `main.tex:49` — "Learning from Demonstration (LfD) methods, where an agent is shown example trajectories from expert users or trained policies, ." → dangling sentence; finish it, e.g. "…or trained policies, are a leading approach to this problem."
- `main.tex:49` — "a world model based reinforcement learning (MBRL) system" → "a world-model-based reinforcement learning (MBRL) system" (and the acronym MBRL expands "model-based RL", not "world model based RL"; say "a model-based reinforcement learning (MBRL) system that learns a world model").
- `main.tex:49` — "DreamerV3 from Demonstrations (DfD)" vs `03-method.tex:37` "Dreamer from Demonstrations (DfD)" vs `03-method.tex:27` "DreamerV3 from Demonstrations (DfD)" → one expansion *(lane 14)*.
- `frontmatter/abstract.tex:3` — "We conduct two simulated environments and one real-world robot task" → "We run experiments in two simulated environments and on one real-world robot task".
- `frontmatter/abstract.tex:3` — "as any other source\todo{still true?}" → resolve the todo (see item 8: it is not true for DfD on planner data).
- `main.tex:49` / `abstract.tex:3` / `01:38` — "collected in-the-wild" (adverbial) → "collected in the wild"; keep the hyphens only attributively ("in-the-wild data collection"). `03-method.tex:58` has the opposite error: "in the wild data collection" → "in-the-wild data collection".

### sections/01-introduction.tex
- `01:24` — "latent-space dynamics systems i.e. World Models" → "latent-space dynamics systems, i.e., world models".
- `01:24–25` — "less work has been done with untrained humans, trained models, and classical robotics approaches … In this paper, we examine how **that variation** shows up" → "that variation" has no antecedent (nothing before it names a variation). Suggest: "…World Models. Demonstrations from these sources differ in ways we describe below. In this paper we examine how those differences show up…".
- `01:25` — leading space at line start (" In this paper") — harmless in LaTeX; the sentence is also a run-on with three "and"s; split after "demonstrations".
- `01:34` — "but  LfD" double space (harmless).
- `01:38` — "R2Dreamer (R2) and DreamerV3 (DV3)" → the paper elsewhere calls these R2Dreamer and DfD; "DV3" appears only here and in `04:93`, `04:99` *(lane 14)*.
- `01:40` — "do not consistently prefer a single demonstrations source" → "a single demonstration source".
- `01:40` — "world model systems are agnostic to every task-dataset combination" → "are agnostic to demonstration source on every task–dataset combination" (as written, it says the systems are agnostic to the combination itself).
- `01:47` — "Demonstration-source is a critical factor determining LfD model performance, and **they** should be evaluated" → "Demonstration source is … , and LfD models should be evaluated" (no plural antecedent).
- `01:47` — "trained with human beings" → "trained on human demonstrations" (a model is not trained "with" human beings).
- `01:66` — caption "Simulated environments for preliminary results." fine.

### sections/02-related-work.tex
- `02:6` — "demonstrated observations-action pairs" → "demonstrated observation–action pairs".
- `02:6` — "Learning from Demonstrations (LfD)" vs "Learning from Demonstration (LfD)" (intro, abstract ×2, main) → pick one (singular is the usual expansion).
- `02:6` — "a broader category which includes … (RLfD) where a model uses" → "a broader category that includes … (RLfD), in which a model uses".
- `02:10` — "noisey" → "noisy".
- `02:10` — "using a Boltzmann distribution \todo{…} Human demonstrations" → missing full stop before "Human" (the `\todo` swallowed it).
- `02:10` — "non-markovian" → "non-Markovian" (proper noun; also `05:15` "markov processes" → "Markov processes").
- `02:10` — "that may effect how LfD algorithms perform" → "affect".
- `02:10` — "perform  \todo" double space.
- `02:10` — "mixed-quality Soft Actor-Critic (SAC) generated trajectories" → "mixed-quality trajectories generated by Soft Actor-Critic (SAC)" (or "SAC-generated").
- `02:13` — "World Models (WM) learn" → "World models (WMs) learn"; "The WM's dynamics model" → "A WM's dynamics model".
- `02:13` — "the multi-step value function that is common in RL and so is not subject to the bias that such a value function induces" — grammatical; but the claim is contestable (Dreamer's critic *is* a multi-step λ-return value function). Content, not grammar — flagged for the authors.

### sections/03-method.tex
- `03:13` — caption "A LfD algorithm" → "An LfD algorithm" ("el-eff-dee").
- `03:17` — "We then compare each … pair … Second, we compare the demonstrations" → "Second" with no "First": "We compare … First, … Second, …" or drop "Second,".
- `03:17` — `$\{algorithm\}\_\{demonstration\_source\}$` typesets the words in math italics with implicit multiplication spacing → `\texttt{\{algorithm\}\_\{demonstration\_source\}}` or plain "algorithm\_source".
- `03:21` — "we record a trained algorithms rollouts" → "we record a trained algorithm's rollouts".
- `03:21` — "that can do the task to use as an alternate source" → "that can perform the task, as an alternative source".
- `03:27` — "the simplest algorithms that can make progress on their task" → "on its task".
- `03:27` — "Reinforcement Learning **from** Prior Data (RLPD)" vs `03:50` and `appendix:46` "Reinforcement Learning **with** Prior Data" → "with" (Ball et al.).
- `03:33` — "Following standard practices DP trains" → "Following standard practice, DP trains".
- `03:37` — "We also train a variant, R2Dreamer, a decoder-free DreamerV3-family variant" → "variant … variant"; "We also train R2Dreamer, a decoder-free DreamerV3-family method with…".
- `03:37` — "\cite{staley2024agent} and refer" → tie: "training~\cite{staley2024agent}".
- `03:50` — "Reinforcement Learning with Prior Data (RLPD) ~\citep{ball2023efficient}." → sentence has no verb: "We use Reinforcement Learning with Prior Data (RLPD)~\citep{ball2023efficient}, which trains a stochastic actor-critic with an entropy bonus (as in SAC) and three additions: half of every batch…".
- `03:50` — "RLPD samples a stochastic actor-critic" → "trains" (one does not "sample" an actor-critic).
- `03:50` — "trains with an entropy reward (like SAC\todo{cite}) with additions" → "with … with"; see rewrite above.
- `03:50` — "the critic updates ten times per environment step" — the HANDOFF §4 says 10 critic updates per *decision*, and the RLPD paper uses 20 (`HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §4). Grammar fine; content → item 2's lane / Discrepancies below.
- `03:58` — "Real world set-up for in the wild data collection." → "Real-world set-up for in-the-wild data collection."

### sections/04-evaluation.tex
- `04:11–16` — outline placeholder lines (see A14).
- `04:17` — "100 machine-generated trajectories collected from a DfD policy trained on the human dataset, using sampled actions" — fine; but "DfD, RLPD, and Diffusion Policy each use the same fixed human and machine datasets" — RLPD is not run in simulation (`03:50` "We use RLPD on the robot task only") and IBC/VMAIL are missing → content contradiction (item 8).
- `04:23`, `04:30`, `05:17`, `appendix:101` — "Pinpad Four" (8×) and `appendix:89` "Pinpad" → "PinPad4" (7× elsewhere, incl. figure captions).
- `04:28`, `04:30` (×1), `04:50` — "Pusht" (6×) → "PushT" (9× elsewhere).
- `04:24` — "Figure \ref" → "Figure~\ref" (also `04:30` "Table \ref", "figure \ref" → "Figure~\ref"; `03:21` "Figure \ref").
- `04:26` — "When it came to only human demonstrations, DfD performed significantly better" → "With human demonstrations only, DfD attained a significantly higher mean episode reward than IBC…".
- `04:26` — "$p=0.474486$" → round ($p=0.47$); "$p=0.000003$" → "$p<10^{-5}$" (also `04:26` "$p=0.00009$").
- `04:26` — "but did not have a meaningfully significant impact on DfD ($p=0.011$)" → "meaningfully significant" is not a category; p=0.011 is below 0.05, so either state the corrected threshold ("not significant after Bonferroni correction, $p=0.011$") or drop "meaningfully".
- `04:26` — "AI-sourced", "AI-generated" → "machine-sourced", "machine-generated" *(lane 11)*.
- `04:30` — "When trained on 85.8\% successful human trajectories" → "When trained on human trajectories that were 85.8\% successful".
- `04:30` — "achieving 92\% success to DP's 3.03\%" → "compared with DP's 3.03\%".
- `04:30` — "We trained DP one more time on it's own output" → "its own output".
- `04:30` — "Coverage reward is also listed displayed in figure \ref{fig:demo_char_pusht}" → "Coverage reward is also shown in Figure~\ref{fig:demo_char_pusht}" (and the next sentence but one repeats "Figure~\ref{fig:demo_char_pusht} shows the coverage reward distribution" — merge).
- `04:30` — "Table \ref{tbl:pusht} shows the performance of a set of 100 demonstrations" but the table's `n episodes` column says 400 → fix one.
- `04:30` — "when they're not successful" → "when they are not successful".
- `04:50` — caption "Pusht trained algorithm demonstration stats by \textit{algorithm / source} e.g. DfD / Human is DreamerV3 trained on" → "PushT: statistics of the demonstrations produced by each trained algorithm, listed as \textit{algorithm / source}, e.g., DfD / Human is DfD trained on human demonstrations."
- `04:42`, `04:45` — table rows "IBC - Dreamer / Human" → "IBC–Dreamer / Human" (en dash) or just "IBC / Human" to match the prose.
- `04:56` — caption "\textit{DP\_ DP}" → "\textit{DP\_DP}" (stray space); "Labels are like \textit{algorithm\_src} e.g." → "Labels follow \textit{algorithm\_source}, e.g.,".
- `04:63` — "\textit{Slide} the can into contact with goal can already on the shelf" → "with the goal can".
- `04:63` — "The can started in one of three marked spots, and goal can was static" → "and the goal can was static".
- `04:67` — "in an IRB approved public-space study" → "IRB-approved".
- `04:68` — "We recorded 74 successful trials." vs appendix "74 of 75" successes and 16 labelled failures — say "The session yielded 75 success-labelled trials, 74 of which could be recovered (Appendix~\ref{appx:real2sim})".
- `04:71` — "we trained learners that required online sampling for a statistically significant number of seeds" → garbled: "Several of our learners require online interaction, and the comparison needs enough seeds per condition to support statistical tests."
- `04:71` — "converted the demos to work within the sim, and all training and evaluation happen in the twin" → parallelism: "converted the demonstrations to run in the simulator, and ran all training and evaluation in the twin"; "demos"/"sim" → "demonstrations"/"simulator".
- `04:71` — "Due to differences between the simulator and true embedded arm controller" → "the true embedded arm controller"; "resulting in less than 0.1 radian difference" → "resulting in a difference of less than 0.1\,rad" (appendix reports median 0.004 rad / 95th pct 0.030 rad — quote those instead).
- `04:71` — "12/40 slid the can into contact with the goal" — the denominators chain (65/74, 40/65, 12/40) but the appendix says "12 reach the goal" of 74 and `DEMO_SETS` §3 gives `slide_event` 25 / `home` 12 — "slid into contact" is `home`, not `slide`; the phase table at `04:84` says Slide 33.3 % (=24/72). Say which predicate (item 8 / Discrepancies).
- `04:75` — "a motion-planning  algorithm" double space; "…a classical robotics approach, details about the motion planning algorithm can be found at \todo{…}" → comma splice: "approach; the motion-planning algorithm is described in Appendix~\ref{appx:planner}".
- `04:75` — "All demonstrations sets contain" → "All demonstration sets contain".
- `04:75` — "R2Dreamer because it was our best performing RLfD algorithm" → "best-performing".
- `04:78` — "similarly to it's demonstration distribution" → "its".
- `04:78` — "to show a like-to-like comparisons  " → "to give a like-for-like comparison." (missing full stop; paragraph ends without punctuation).
- `04:78` — "pixel-demonstrations" → "pixel demonstrations"; "numeric privileged state" → "privileged numeric state".
- `04:89` — caption "Each source contains 72 demonstrations." — the human set is 74 tapes (`04:68`, appendix, `DEMO_SETS`). Either the table is computed on 72 human tapes (say which two are dropped) or the caption is wrong.
- `04:93` — "We train 8-seeds" → "We train eight seeds".
- `04:93` — "to investigate effects on these algorithms from demonstration source" → "to measure the effect of demonstration source on each algorithm".
- `04:93` — "break out results to the individual phases" → "break results out by task phase".
- `04:93` — "RL policies (RLPD, DV3, R2)" → "(RLPD, DfD, R2Dreamer)" *(lane 14)*.
- `04:99` — table row "DreamerV3" → "DfD" *(lane 14)*; `04:110` "WMs (R2Dreamer and DreamerV3)" likewise.
- `04:104` — caption "Pixel sparse +10: Sampled-action evaluation. Demonstration source order: Human / DP / Motion Planning / R2Dreamer." → write it as a sentence: "Success rate by task phase for each learner (rows) and demonstration source (Human / DP / Planner / R2, in that order within each cell); pixel observations, sparse +10 reward, sampled actions, 30 random starts, mean over eight seeds."
- `04:110` — "(learned or planned)  while" double space.
- `04:130–139` — the "Characterizing the Demonstration Sets" subsection has one sentence of prose (`04:139`) and two author notes; both figures (`fig:coverage`, `fig:tool`) are unreferenced.

### sections/05-conclusion.tex
- `05:7` — "DfD, which uses pixel reconstruction loss, learns equally well … but **do** not learn well" → "but does not learn well".
- `05:7` — "even more robust due its decoder-free loss" → "due to its".
- `05:7` — "\todo{cite TDMPC2 and R2Dreamer}" — R2Dreamer is already cited as `morihira2026r2d` at `03:37`.
- `05:9` — "makes it helpful for characterizing the performance of LfD algorithms." → sentence fragment (orphan; the subject was lost). See item 8 for a rewrite.
- `05:12` — "…when algorithm performance results are repo" → truncated ("reported.").
- `05:15` — "because it's a sequence model" → contraction is grammatically correct ("it is") but formal register: "because it is a sequence model"; likewise "doesn't" → "does not", `05:24` "it's easier", "There's".
- `05:15` — "~\citet{chi2024diffusionpolicy}" → `\citep` (A10).
- `05:15` — "high dimensional distributions" → "high-dimensional".
- `05:15` — "generated by markov processes" → "Markov".
- `05:15` — "IBC learns better from trained-policy demonstrations because they are actually generated by markov processes with respect to the observable task state" contradicts `01:36` "AI-generated trajectories are therefore not necessarily Markovian from the learner's observed state" → content (item 8), but as written one of the two sentences is false.
- `05:17` — "In Pinpad Four" → "In PinPad4".
- `05:17` — "these algorithms perform near optimal" → "perform near-optimally".
- `05:17` — "This result cleanly demonstrates the benefit of using MBRL over LfD, the model learns" → comma splice; and MBRL-from-demonstrations is itself LfD by the paper's own definition (`02:6`): "…the benefit of MBRL over behavior cloning: the model learns…".
- `05:17` — "learn a policy that does better than the input demonstrations" fine.
- `05:20` — "A demonstration-agnostic baseline act as a neutral comparison" → "A demonstration-source-agnostic baseline acts as".
- `05:24` — "Of course more data wins, when you can do the task as many times as you want." → informal; "There's value in human-noise" → "There is value in human noise" (no hyphen). Register issue rather than grammar; see item 8.
- `05:33` — "Our results that the source of demonstrations must be considered" → "Our results show that".
- `05:33` — "demonstration-sources", "the demonstration-source" → "demonstration sources", "the demonstration source" (hyphen only when used attributively, e.g., "demonstration-source effects").
- `05:37` — "the python code" → "the Python code"; "format latex tables" → "LaTeX".

### appendix/a-research-methods.tex
- `appx:18` — "over 66 demonstrations and 39{,}332 frames" vs "74 of the 74 tapes" three lines later → the tracking-error figure was measured on the older 66-tape frozen set; say so or re-measure (Discrepancies).
- `appx:33` — "2-7\%" → "2--7\%" (en dash, as at line 15 and 28).
- `appx:38` — "DreamerV3 and R2Dreamer" → "DfD and R2Dreamer" *(lane 14)*.
- `appx:42` — `\subsubsection` → `\subsection` (A15).
- `appx:48` — "Visual Adversarial Imitation Learning" vs `03:27` "Variational Model-based Adversarial Imitation Learning (VMAIL)" → Rafailov et al. is "Variational Model-based Adversarial Imitation Learning"; fix the appendix heading.
- `appx:64` — "the existing repository's hierarchical … factors" → reader-facing text should not refer to "the existing repository"; also `appx:79` "HRI document commit \texttt{7b227e7}", "\texttt{rnd30\_v2}/\texttt{hold15\_v2}", "\texttt{all\_bnormclampS8ent5}" are internal run tags (item 8).
- `appx:84` — caption "no timing comparison survives its six-comparison correction" — refers to a table no longer in the paper (B-list).
- `appx:89` — "Pinpad and PushT Training Curves" → "PinPad4 and PushT Training Curves".
- `appx:94` — "Shading is +/- 1 standard deviation of n=4 trials" → "Shading is $\pm 1$ standard deviation over $n=4$ seeds" (also `appx:101` "n = 8 trials"); "than with Human demos" → "than with human demonstrations" (×4 in the two captions); "*DP is trained…" asterisk footnote inside a caption → make it a sentence.
- `appx:101` — "IBC and VMAIL perform better with trained-policy demos than with Human demos. While DfD performs similarly with both." → fragment: "…than with human demonstrations, while DfD performs similarly with both."

### tables/*.tex (only `sensitivity.tex` is live)
- No spelling defects. Column "Holm $p$" and "BF range" fine.

## Discrepancies (paper vs code/notes surfaced while checking grammar)

| where | paper says | notes/code say | source |
|---|---|---|---|
| `04:89` caption | each source has 72 demonstrations | human set = 74 tapes | `HRI_results/DEMO_SETS_2026-09-11.md` §1 ("Human set — `dHfull_all` (74 tapes)"); `04:68`; `appx:25` |
| `04:71` | "12/40 slid the can into contact" (=`home`) and `04:84` "Slide 33.3 %" | `home` 12/74 state, 13/74 pixels; `slide_event` 25/74 state, 26/74 pixels; `placed_v2` 40 (state) / 42 (pixels) | `DEMO_SETS` §3 table |
| `04:84` Human row "Nested 15.3" | 15.3 % | 11/72 = 15.3 % — but the human set has 74 tapes and `nested_v2` = 14/74 (18.9 %) / `home` 12/74 (16.2 %) | `DEMO_SETS` §3; the table's denominator and predicate are unstated |
| `04:30` | "a set of 100 demonstrations" | table n = 400 | `04:40–47` |
| `03:50` | critic updates 10× per environment step | HANDOFF §4: "10 critic updates per decision, where the paper uses 20" (i.e. the *RLPD paper*); also "no frame stack", "two-layer critics" | `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §4 |
| `appx:18` | tracking error measured over 66 demonstrations | training set is 74 | `appx:25`; `DEMO_SETS` §1 |
| `04:63` | reward +10 "when they slid the can into contact with the goal can" | `home` = `slide_event` ∧ `nested_v2` (upright, at rest, not in hand, within 8.1 cm) — contact alone does not pay | `DEMO_SETS` §2 predicate table |

## \llm notes inserted
None for this item (spelling/grammar notes are not inserted, per the brief).

## Sources
- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/{main.tex,preamble.tex,frontmatter/abstract.tex,frontmatter/ccs.tex,sections/01-introduction.tex,sections/02-related-work.tex,sections/03-method.tex,sections/04-evaluation.tex,sections/05-conclusion.tex,appendix/a-research-methods.tex,appendix/b-online-resources.tex,tables/*.tex,back/acks.tex,back/ethics.tex}` (read in full, `cat -n`).
- `grep -c placeins acmart.cls` → 0; label/ref/figure cross-check: `grep -hoE '\\label\{[^}]+\}'` / `'\\(ref|Cref|cref)\{[^}]+\}'` / `includegraphics` over the live files; `ls figures/`.
- Spelling-variant counts: `grep -rhoE 'Pin[Pp]ad ?(4|Four)?|Push[Tt]'`, `'(Variational|Visual)[A-Za-z -]*Imitation Learning'`, `'Reinforcement Learning (with|from) Prior Data'`, `'(DreamerV3|Dreamer) from Demonstrations \(DfD\)'`.
- `HRI_results/DEMO_SETS_2026-09-11.md` §1–§3; `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §4.
