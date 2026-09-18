# Item 09 — Start the appendix with a table of contents

**Status: done.** Recommendation is option (a), a *partial* table of contents from the `titletoc` package
(`\startcontents`/`\printcontents`), which lists only what comes after `\appendix`. It was compiled locally against the
project's own `acmart.cls` (sigconf, pdfLaTeX, hyperref + cleveref loaded as in `preamble.tex`, TeX Live 2022) and
renders a hyperlinked "A / B / B.1 / B.2 / C / D" list with page numbers; the test files are in
`/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/30232c01-75cf-49d2-b2e3-edb5866cd994/scratchpad/toctest/`
(`t_titletoc.tex`, rendered `page-1.png`). The etoc variant (option b) compiled but its `\localtableofcontents` at
document level did not scope to the appendix in the test; the hand-written itemize (option c) needs a `\label` on every
section and other lanes are adding unlabelled subsections tonight, so it is the fallback, not the recommendation.
Two compile-blocking things surfaced while testing and are listed under Discrepancies (`\FloatBarrier` without
`placeins`; `\ref{appx:hyperparameters}` with no such label).

## 1. Preamble line(s) — `preamble.tex`, in the "Safe additional packages" block

```latex
\usepackage{titletoc}       % partial TOC for the appendix (\startcontents/\printcontents); no titlesec needed
\usepackage{placeins}       % \FloatBarrier is already used in appendix/a-research-methods.tex:87 but nothing loads it
```

`titletoc` is not on acmart's forbidden list (that list is geometry/setspace/font overrides — `preamble.tex` header);
it only hooks `\addcontentsline`, which acmart and hyperref use normally (acmart writes standard `\contentsline`
records; `\l@section`/`\l@subsection` are acmart's own `\@tocline` styles, so the appendix TOC is typeset in the
class's own TOC style, no extra formatting needed). The partial TOC goes to `main.ptc`; two pdfLaTeX passes as usual.

## 2. Paste-ready block — `main.tex`, replacing the three lines after the bibliography

```latex
%% \appendix goes AFTER the bibliography. Sections become lettered.
\appendix
%% Appendix-only table of contents (titletoc partial TOC: lists only what follows \startcontents).
%% If the appendix is compiled as its own document, keep exactly these lines; nothing else changes.
\startcontents[appendix]
\printcontents[appendix]{}{1}{\section*{Contents of the Appendix}\setcounter{tocdepth}{2}}
\input{appendix/a-research-methods}
\input{appendix/b-online-resources}
```

Notes on the arguments: `{}` = no prefix for the `.ptc` file; `{1}` = start at level 1 (`\section`); the last argument
runs before the list — `\section*{…}` gives the heading in the class's section font, `tocdepth 2` lists
sections + subsections (set to 3 to include `\subsubsection`s, 1 for sections only). The heading is unnumbered and is
itself excluded from the list (starred).

**If the appendix is submitted as a separate PDF** (`\jss{DO NOT INCLUDE THE APPENDIX IN THE MAIN DOCUMENT, SUBMIT
SEPARATELY.}`, `sections/05-conclusion.tex:41`): make a second root file (e.g. `main-appendix.tex`) that is a copy of
`main.tex` with the body/back-matter `\input`s removed and the block above kept. `\Cref`/`\ref` to main-text labels
(`sec:experiments`, `sec:restock`, `fig:…`) will then print `??` — grep the appendix for `\ref{sec:` / `\ref{fig:`
before the split. In that standalone file a plain `\tableofcontents` also works (the document then contains only the
appendix), but the titletoc block works in both configurations, so use it everywhere.

## 3. Fallback (option c) — hand-written list, only if titletoc is rejected

```latex
\appendix
\section*{Contents of the Appendix}
\begin{itemize}\setlength{\itemsep}{0pt}
  \item \Cref{appx:real2sim}: \nameref{appx:real2sim}
  \item \Cref{appx:planner}: \nameref{appx:planner}
  \item \Cref{appx:training}: \nameref{appx:training}
  \item \Cref{app:stats}: \nameref{app:stats}
  \item \Cref{appx:pinpad_pusht_training_curves}: \nameref{appx:pinpad_pusht_training_curves}
  \item \Cref{app:resources}: \nameref{app:resources}
\end{itemize}
```
(`\nameref` comes with hyperref; every listed section needs a `\label` on the same line as its `\section`.)

## 4. Expected appendix contents once tonight's lanes are pasted

Current files (`appendix/a-research-methods.tex`, `appendix/b-online-resources.tex`) plus the sections the other lanes are
drafting (`01_…` proposes `\subsection{Motion-planning demonstrations}\label{appx:planner}`; lanes 02–05 are writing
RLPD / Diffusion Policy / R2Dreamer / DfD hyperparameter subsections; lane 06 audits the real2sim section). Suggested
final order, with the labels the main text already references:

| # | `\section` / `\subsection` | label | source |
|---|---|---|---|
| A | Bringing real demonstrations into the Genesis simulator | `appx:real2sim` (referenced `04-evaluation.tex:71`) | exists; lane 06 audit |
| B | Demonstration sources | — | new container (or fold B.1 into A as lane 01 suggests) |
| B.1 | Motion-planning demonstrations | `appx:planner` (referenced by `04-evaluation.tex:75` once the `\todo{reference appendix}` is replaced) | lane 01 |
| B.2 | (optional) DP-teacher and R2Dreamer-teacher demonstration sets | — | lanes 03/04 if they provide it |
| C | Training | `appx:training` (referenced `04-evaluation.tex:93`) **and** `appx:hyperparameters` (referenced `03-method.tex:29`, currently undefined — put both `\label`s on this `\section`) | exists |
| C.1 | Diffusion Policy | — | lane 03 |
| C.2 | Reinforcement Learning with Prior Data | — | lane 02 |
| C.3 | R2Dreamer | — | lane 04 |
| C.4 | DfD (DreamerV3 losses) | — | lane 05 |
| C.5 | Implicit Behavior Cloning | — | exists as a `\subsubsection` under "DreamerV3 and R2Dreamer" — promote to `\subsection` |
| C.6 | Visual Adversarial Imitation Learning | — | exists (empty) |
| D | Statistical calculations | `app:stats` | exists |
| E | PinPad4 and PushT training curves | `appx:pinpad_pusht_training_curves` | exists (title says "Pinpad") |
| F | Plot definitions and calculation methods (dataset characterization) | `appx:characterization_definitions` (referenced — wrongly, with `\label` instead of `\ref` — at `04-evaluation.tex:139`) | `appendix/c-llm-generated.tex`, **not `\input`'d by `main.tex`** |
| G | Online Resources | `app:resources` | `b-online-resources.tex` (placeholder "Your text.") |

With `tocdepth 2` the TOC prints all of the above except `\subsubsection`s.

## Discrepancies

1. **`\FloatBarrier` is used (`appendix/a-research-methods.tex:87`, `appendix/c-llm-generated.tex:133`,
   `sections/04-evaluation-bk.tex:168`) but no file loads `placeins`** (`grep -rn placeins` over `*.tex`, `acmart.cls`:
   no hits). pdfLaTeX reports `Undefined control sequence`; the preamble line above fixes it.
2. **`Appendix~\ref{appx:hyperparameters}`** (`sections/03-method.tex:29`) has no matching `\label` anywhere
   (`grep -rn 'label{appx:hyperparameters}'`: none). Add `\label{appx:hyperparameters}` next to
   `\section{Training}\label{appx:training}` (two labels on one section are fine).
3. **`See appendix \label{appx:characterization_definitions}`** (`sections/04-evaluation.tex:139`) is a `\label`, not a
   `\ref`, and the real label lives in `appendix/c-llm-generated.tex:18`, which `main.tex` does not `\input`. Either
   `\input{appendix/c-llm-generated}` (its top is an orphan `\paragraph{Provenance…}` that needs a section above it)
   or drop the sentence.
4. Naming: the appendix title "Pinpad and PushT Training Curves" vs the main text "PinPad4"; "Training" section
   subsection "DreamerV3 and R2Dreamer" vs the paper's "DfD" convention (`00_BRIEF.md`: DreamerV3 → "DfD").
5. The compile test needed `\microtypesetup{expansion=false}` on this machine only because the local Debian TeX Live
   lacks scalable Libertine fonts (baseline acmart without any of my additions fails the same way); that line is **not**
   part of the recommendation and is not needed on Overleaf.

## \llm notes inserted

- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/appendix/a-research-methods.tex`, anchor
  `\section{Bringing real demonstrations into the Genesis simulator}\label{appx:real2sim}` — inserted immediately
  before it:
  `\llm{Item 09: appendix TOC = titletoc partial TOC (\usepackage{titletoc} in preamble.tex; \startcontents[appendix] + \printcontents in main.tex right after \appendix); paste-ready block, expected section list and two compile fixes (placeins for \FloatBarrier; missing label appx:hyperparameters) in genesis_pickaplace/HRI_results/late_night_9_18/09_appendix_toc.md.}`

## Sources

- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/{main.tex,preamble.tex,acmart.cls}` (acmart `\@starttoc`, `\l@section`,
  `\RequirePackage` list at lines 291–878; no `placeins`, `titletoc`, `etoc`)
- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/appendix/{a-research-methods,b-online-resources,c-llm-generated}.tex`
- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sections/{03-method,04-evaluation,05-conclusion}.tex` (appendix `\ref`s)
- `HRI_results/late_night_9_18/00_BRIEF.md`, `01_motion_planner_appendix.md` (proposed `appx:planner` subsection), `02–06` (claimed stubs at the time of writing)
- Compile tests: `pdflatex`/`bibtex` (TeX Live 2022/dev/Debian), `kpsewhich titletoc.sty etoc.sty placeins.sty`;
  files under the scratchpad `toctest/` directory listed in the status paragraph.
