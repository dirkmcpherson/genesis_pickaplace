# Item 13 — the robot-arm task is called "restocking" everywhere

**Status: done (audit complete; no .tex text changed, per the brief).** The word "restocking" already appears in 5
places (intro figure caption, intro:38 "shelf-restocking task", the `04-evaluation` subsection title and its first
sentence, and the method's parenthetical "robot shelf restocking"). Everywhere else the task is "the robot task",
"real-world robot task", "the task", "the study scene", "full-task", or unnamed. There is **no** literal
"pick-and-place"/"pick→place→slide" in any `\input`'d file (only in the un-input `04-evaluation-bk.tex`, 3 hits, and the
label `fig:pickaplace_perf`). Below: every occurrence with a before → after, the labels that can stay, and the figure
files whose titles would need "Restocking" if the user wants the task name inside the images.

All paths are relative to `~/workspace/overleaf/6a96f0e5337340dfd4edea88/` (working copy at `c50d276`, 2026-09-17 23:20).

## A. Already correct (no change)

| file:line | text |
|---|---|
| `sections/01-introduction.tex:30` | caption "Restocking task using a Kinova Gen3 Lite for human demonstration collection…" |
| `sections/01-introduction.tex:31` | `\label{fig:restock_full}` |
| `sections/01-introduction.tex:38` | "a shelf-restocking task built from real-world robot demonstrations" (fine; could shorten to "the restocking task") |
| `sections/04-evaluation.tex:61` | `\subsection{Real-World Experiment: Restocking}` |
| `sections/04-evaluation.tex:63` | "The restocking task has three parts…" |
| `frontmatter/abstract.tex:9` | commented TW draft "shelf-restocking task" |

## B. Changes — the task named as something else

| # | file:line | before | after |
|---|---|---|---|
| 1 | `frontmatter/abstract.tex:3` | "We conduct two simulated environments and one real-world robot task where we collect data from inexperienced participants in-the-wild." | "We conduct experiments in two simulated environments and on one real-world robot **restocking** task, for which we collect data from inexperienced participants in-the-wild." |
| 2 | `main.tex:49` | "…two simulated environments and one real-world robot task where we collect data…" | same as #1 — **but this line is a duplicate stale abstract; delete it** (item 11 file, Discrepancy D1). |
| 3 | `sections/01-introduction.tex:34` | "when a novel, unsolved, real-world task has no trained model available" | keep (generic statement, not our task). |
| 4 | `sections/03-method.tex:17` | "two simulated tasks (PinPad4 and PushT, Sec.~\ref{sec:experiments}) and one real-world robot task with a digital twin (robot shelf restocking, Sec.~\ref{sec:restock})" | "two simulated tasks (PinPad4 and PushT, Sec.~\ref{sec:experiments}) and one real-world **restocking** task with a digital twin (Sec.~\ref{sec:restock})". **Both `\ref`s are undefined** — see Discrepancies. |
| 5 | `sections/03-method.tex:27` | "and the robot task uses DP, Reinforcement Learning from Prior Data (RLPD), R2Dreamer with demonstrations, and DfD." | "and the **restocking** task uses DP, Reinforcement Learning from Prior Data (RLPD), R2Dreamer with demonstrations, and DfD." |
| 6 | `sections/03-method.tex:50` | "We use RLPD on the robot task only, where a strong non-imitation baseline is needed and dense reward is not available." | "We use RLPD on the **restocking** task only, …" |
| 7 | `sections/03-method.tex:67-69` (caption of `fig:envs`, the system/twin figure) | "A robot arm picks up a can of soup and places it on the shelf next to a can already in place. …" | "**The restocking task.** A robot arm picks up a can of soup and places it on the shelf next to a can already in place. …" |
| 8 | `sections/04-evaluation.tex:11-16` | outline scaffolding "Sec 5.2 Real-World Experiment: Restocking …" | leftover outline text that will print in the PDF — delete (flow lane), naming is already right. |
| 9 | `sections/04-evaluation.tex:71` | "Therefore we built a digital twin of the task in the Genesis simulator" | "…a digital twin of the **restocking** task…" (optional; the subsection is already titled Restocking). |
| 10 | `sections/04-evaluation.tex:104` (caption, `tab:phase-success-pixel-sparse10-sampled-eval`) | "Pixel sparse +10: Sampled-action evaluation. Demonstration source order: …" | "**Restocking**, pixel observations, sparse $+10$ reward: sampled-action evaluation. Dataset order in each cell: Human / DP / Planner / R2." (item 15 wording) |
| 11 | `sections/04-evaluation.tex:110` (caption, `fig:pickaplace_perf`) | "Performance of LfD algorithms on human vs learned vs planned demonstrations." | "**Restocking** performance (nested success) of each learner trained on the Human, DP, Planner and R2 datasets." |
| 12 | `sections/04-evaluation.tex:123` (caption, `fig:training_phases`) | "How many online samples are required before the algorithm can perform a phase of the task." | "…before the algorithm can perform a phase of the **restocking** task." (same text is in the commented-out `:117`) |
| 13 | `sections/04-evaluation.tex:130` | `\subsubsection{Characterizing the Demonstration Sets}` | `\subsubsection{Characterizing the Restocking Demonstration Sets}` (optional; it sits under the Restocking subsection) |
| 14 | `sections/04-evaluation.tex:143-148` (caption, `fig:coverage`) | "(b) … full-task nominal duration … (c) Full-task command inactivity … (d) Full-task command-inactivity distributions." | "full **restocking-task** nominal duration" once in (b); (c)/(d) can keep "Full-task" after that. |
| 15 | `sections/05-conclusion.tex:5` | "RLPD can only complete the task when given DP demonstrations" | "RLPD can only complete the **restocking** task when given the DP dataset" |
| 16 | `sections/05-conclusion.tex:7` | "R2Dreamer … is demonstration-source agnostic on this task." | "…on the **restocking** task." |
| 17 | `sections/05-conclusion.tex:24` | "when you can do the task as many times as you want" | keep (generic). |
| 18 | `appendix/a-research-methods.tex:7` | `\section{Bringing real demonstrations into the Genesis simulator}` | `\section{Bringing the real restocking demonstrations into the Genesis simulator}` |
| 19 | `appendix/a-research-methods.tex:9` | "We rebuilt the study scene in the Genesis simulator" | "We rebuilt the **restocking** scene in the Genesis simulator" |
| 20 | `appendix/a-research-methods.tex:24` | "The full-task human set keeps every success-labelled trial" | "The full **restocking-task** human set keeps every success-labelled trial" |
| 21 | `sections/01-introduction.tex:38` | "a shelf-restocking task built from real-world robot demonstrations collected in-the-wild" | optional: "the restocking task, built from …" — keeps one canonical name ("restocking", not "shelf-restocking"). Same for `03-method.tex:1-3` comments (ignore, comments). |

Ready-to-paste versions of the sentences that change meaning-free:

```latex
% frontmatter/abstract.tex:3 (one sentence)
We conduct experiments in two simulated environments and on one real-world robot restocking task, for which we collect data from inexperienced participants in-the-wild.

% sections/03-method.tex:17 (one sentence; the two \ref targets still need labels, see Discrepancies)
We run this comparison in three environments: two simulated tasks (PinPad4 and PushT, Sec.~\ref{sec:experiments}) and one real-world restocking task with a digital twin (Sec.~\ref{sec:restock}).

% sections/03-method.tex:27 (tail of the sentence)
and the restocking task uses DP, Reinforcement Learning from Prior Data (RLPD), R2Dreamer with demonstrations, and DfD.

% sections/03-method.tex:50 (one sentence)
We use RLPD on the restocking task only, where a strong non-imitation baseline is needed and dense reward is not available.

% sections/03-method.tex:67 (caption opening)
\caption{The restocking task. A robot arm picks up a can of soup and places it on the shelf next to a can already in place. The width of the gripper prevents the robot from placing the cans directly next to each other, so the user must slide one can into contact with the other.}

% sections/04-evaluation.tex:123
\caption{How many online samples are required before the algorithm can perform a phase of the restocking task.}

% sections/05-conclusion.tex:5 / :7
… while RLPD can only complete the restocking task when given the DP dataset. …
… is demonstration-source agnostic on the restocking task.

% appendix/a-research-methods.tex:7
\section{Bringing the real restocking demonstrations into the Genesis simulator}\label{appx:real2sim}
```

## C. Labels and identifiers (can stay; listed so nobody "fixes" a cross-reference by accident)

| where | identifier | note |
|---|---|---|
| `sections/04-evaluation.tex:111` | `\label{fig:pickaplace_perf}` | No `\ref` to it anywhere in the `\input`'d files (only `04-evaluation-bk.tex`). Keep or rename to `fig:restock_perf`; harmless. |
| `sections/01-introduction.tex:31` | `\label{fig:restock_full}` | referenced at `01-introduction.tex:38`. |
| `sections/04-evaluation.tex:105` | `tab:phase-success-pixel-sparse10-sampled-eval` | internal name; keep. |
| `appendix/a-research-methods.tex:7` | `appx:real2sim` | referenced at `04-evaluation.tex:71`. |
| figure file names `pickaplace_perf.png`, repo name `genesis_pickaplace` | — | file names, keep. |

## D. Figure files whose pixels name the task (LaTeX cannot change)

None of the included figures prints "pick-and-place". If the user wants "Restocking" inside the images:

| figure | used at | in-image title/labels | generator |
|---|---|---|---|
| `figures/success_sample_home.png` | `04-evaluation.tex:109` | title "Pixels · sparse +10 / Sampled evaluation"; y "Nested success (%)" | `HRI_results/pixel_4x4/export_charts.py` → `paper/figures/px_phase_2026-09-14/fig_results_4x4_sampled_home.png` (`EXPORT_MANIFEST.json` lists source `HRI_results/artifacts/refresh_2026-09-17/pixel_sparse10`); upstream plotting `baselines/diagnostics/px_results_4x4_plot.py` |
| `figures/training_phases.png` | `04-evaluation.tex:122` | title "Pixels · sparse +10 · training phases"; panels Picked/Placed/Slide/Nested | `baselines/diagnostics/px_phase_analysis.py` via `HRI_results/pixel_4x4/upstream_phase_analysis.py` |
| `figures/phase_ignition.png` | commented out (`:116`) | "Pixels · sparse +10 · phase ignition" | same |
| `figures/success_mode_home.png`, `success_training_home.png` | unused | "Pixels · sparse +10 · MODE evaluation" / "· Sampled training" | `HRI_results/pipeline/paper_success_violin.py` |
| `figures/coverage_and_inactivity.png` | `04-evaluation.tex:142` | "(b) Task duration", "(c) Full-task inactivity", "(d) Full-task command inactivity", x "Full-task nominal duration (s)" | `HRI_results/dataset_dynamics_four_sources/coverage_figure.py` |
| `figures/tool_trajectories.pdf` | `04-evaluation.tex:152` | "Genesis: top view / side view" | `HRI_results/dataset_dynamics_four_sources/tool_figure.py` |
| `figures/pickaplace_perf.png` | unused | "One world, three learners … pick success, 30 random ICs" | stale (state-obs pick study) |
| `figures/e2e_performance.pdf` | unused | "R2Dreamer end-to-end … Awaiting validated results" | placeholder |

Suggested in-image title if regenerated: "Restocking · pixels · sparse +10 · sampled evaluation".

## Discrepancies

- **Undefined cross-references.** `sections/03-method.tex:17` refers to `\ref{sec:experiments}` and `\ref{sec:restock}`; neither label exists (`grep -rn 'label{sec:' sections/` gives `sec:intro, sec:related, sec:eval:results, sec:dig_tw, sec:eval:discussion, sec:limitations, sec:conclusion`). The restocking subsection at `04-evaluation.tex:61` has no `\label`. Also `03-method.tex:29` `\ref{appx:hyperparameters}` (no such label; the appendix uses `appx:training`) and `04-evaluation.tex:139` writes `\label{appx:characterization_definitions}` where a `\ref` is meant (and no such section exists). These print as "??".
- **`\label{fig:envs}` is defined twice**: `01-introduction.tex:67` (PinPad4/PushT panels) and `03-method.tex:70` (real set-up / digital twin). `04-evaluation.tex:63` "The restocking task has three parts (Fig.~\ref{fig:envs})" intends the robot figure; LaTeX will warn "multiply defined" and resolve to whichever is last. Rename the robot one to `fig:restock_envs`.
- **Task description vs code.** Paper: "The can started in one of three marked spots" (`04-evaluation.tex:63`) and "We evaluate on 30 new can positions chosen randomly from the demo support" (`:93`). Code/notes: the three clusters are visible in `coverage_and_inactivity.png` panel (a); the evaluation bank is `rnd30` (`HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`; `00_BRIEF.md`). Consistent. Reward: paper "+10 when they slid the can into contact" (`:63`, `:93`); brief: `nested_sparse10` pays +10 once for `home` (= slide_event ∧ nested_v2), terminal — "into contact" under-describes "nested" but is not wrong. `:71` "65/74 picked, 40/65 placed, and 12/40 slid" vs appendix `:26` "65 of the 74 tapes pick in simulation and 12 reach the goal" — consistent with each other; `DEMO_SETS_2026-09-11.md` is the source to re-check the 40 (not re-verified here; item 6 lane).
- `04-evaluation.tex:1-16` still contains the section outline as body text ("Sec. 5 Experiments … Sec. 5.3 or Sec. 6: Discussion"); it will typeset.

## \llm notes inserted

None specific to this item; the caption notes inserted for item 15 (`04-evaluation.tex` tables at lines 89 and 104) mention this file for the "Restocking" caption wording.

## Sources

- All in-scope .tex files read in full (`cat -n`), root `~/workspace/overleaf/6a96f0e5337340dfd4edea88/`.
- `grep -n -i -E 'pick-and-place|pick and place|pickaplace|robot task|shelf task|shelf-restocking|shelf restocking|restock|real-world robot|the task|Real-World Experiment'` over the in-scope files; `grep -rn 'label{' sections appendix` for the label census; `grep -rn includegraphics --include=*.tex .` for figure usage.
- Figures viewed: see item 11 file §D (same set). Generators: `HRI_results/pixel_4x4/{export_charts.py,publish.py,upstream_phase_analysis.py}`, `paper/figures/px_phase_2026-09-14/EXPORT_MANIFEST.json`, `baselines/diagnostics/{px_results_4x4_plot.py,px_phase_analysis.py}`, `HRI_results/pipeline/paper_success_violin.py`, `HRI_results/dataset_dynamics_four_sources/{coverage_figure.py,tool_figure.py}`.
- `HRI_results/late_night_9_18/00_BRIEF.md` (task definition, reward ladder, eval bank).
