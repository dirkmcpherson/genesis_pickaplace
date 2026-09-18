# Item 15 — dataset / run naming convention

**Status: done (audit complete; no .tex text changed except four `\llm{}` anchors).** The convention sentence at
`sections/03-method.tex:23` is the only place the paper follows itself. It is contradicted in **three different ways**
elsewhere: (i) run names written as "Algorithm / Source" with a slash (PushT table), as "algorithm\_src" with `_Hu`
suffixes (PushT/PinPad figures), and as "$\{algorithm\}\_\{demonstration\_source\}$" (method:17); (ii) datasets named
"Motion planning", "Motion Planning", "R2Dreamer", "R2-generated", "DP-generated", "Machine", "learned/planned"
instead of one name each; (iii) the sentence itself says "by **source and learner**" but its example `RLPD_DP` is
**learner\_source** order. Below: the canonical set, a rename table, every occurrence, and the figures that must be
regenerated because their labels are baked in.

All paths are relative to `~/workspace/overleaf/6a96f0e5337340dfd4edea88/` (working copy at `c50d276`, 2026-09-17 23:20).

## A. Recommended canonical form (one form, used everywhere)

**Datasets** are named by the source that produced them:

| dataset | what it is | internal set name (for provenance appendix only) |
|---|---|---|
| **Human** | in-the-wild teleoperation; 74 tapes (restocking) / 100 (PinPad4, PushT) | `dHfull_all_rns10h_img` (`HRI_results/DEMO_SETS_2026-09-11.md`) |
| **DP** | rollouts of DP\_Human (state-observation teacher); 72 tapes | `dDPfull_first_rns10h_img` |
| **Planner** | rollouts of the motion planner; 72 tapes | `planner_matched72…` (`HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`) |
| **R2** | rollouts of R2\_Human; 72 tapes | r2teacher set (`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`) |
| **DfD** (simulated tasks only) | rollouts of DfD\_Human in PinPad4/PushT | — |

**Learners**: DfD, R2, RLPD, DP, IBC, VMAIL. (R2 = R2Dreamer, defined at `01-introduction.tex:38`; the learner name in
prose can stay "R2Dreamer", but in run names and table "Learner" columns use the short form so `R2_R2` is legible.)

**Runs**: `learner_dataset`, typeset `\textsc{}`-free as `RLPD\_DP`, `DfD\_Human`, `DP\_Planner`, `R2\_R2`, `IBC\_DfD`.
No slashes, no "Hu", no "-generated", no "Machine" as a dataset name.

The convention sentence should therefore read:

```latex
% sections/03-method.tex:23 (replace the paragraph)
We name datasets by their source (Human, DP, Planner, R2) and trained runs by learner and dataset. For example, RLPD\_DP is RLPD trained on the DP dataset, the machine demonstrations that a DP teacher produced; DfD\_Human is DfD trained on the human demonstrations.
```

(The only edit of substance is "by source and learner" → "by learner and dataset", so the words match the `RLPD_DP`
order, plus listing the four names once.)

## B. Rename table (before → after), every occurrence

### B1. The convention and its restatement

| file:line | before | after |
|---|---|---|
| `sections/03-method.tex:17` | "We then compare each $\{algorithm\}\_\{demonstration\_source\}$ pair by task success rate…" | "We then compare each learner\_dataset run by task success rate…" (same vocabulary as line 23: "learner", "dataset"; drop the math-mode braces, which typeset as `{algorithm}_{demonstration_source}`). |
| `sections/03-method.tex:23` | "We name datasets by their source, and trained runs by source and learner. For example, RLPD\_DP is RLPD trained on machine demonstrations that a DP teacher produced." | see §A. |
| `sections/03-method.tex:21` | "Next, we record a trained algorithms rollouts to produce a machine-demonstration set." | "Next, we record a trained run's rollouts to produce a machine dataset named after that learner (e.g. the DP dataset)." (optional; ties the pipeline to the names) |

### B2. Simulated experiments (PinPad4 / PushT) — `sections/04-evaluation.tex`

| file:line | before | after |
|---|---|---|
| `:17` | "100 human demonstrations from paper authors and 100 machine-generated trajectories collected from a DfD policy trained on the human dataset" | "the Human dataset (100 demonstrations from paper authors) and the DfD dataset (100 trajectories collected from DfD\_Human with sampled actions)" — **see Discrepancy D2 (this paragraph names RLPD and 8 seeds; it looks like restocking text).** |
| `:24` | "with human demonstrations versus DfD demonstrations" / "demonstrations from a fully trained DfD" | "with the Human dataset versus the DfD dataset" (optional; already uses the dataset name "DfD"). |
| `:26` | "When comparing the AI-sourced demonstrations…" / "The addition of the AI-generated demonstration set…" | "When comparing runs trained on the DfD dataset…" / "The addition of the DfD dataset…" (item 11 §A rows 4–5). |
| `:30` | "DfD performs similarly when trained on human demonstrations and DfD demonstrations" … "We trained DP one more time on it's own output (55.8\% successful) and found that DP performed better on its own demonstrations than on DfD's demonstrations" | add the run names once: "DfD\_Human and DfD\_DfD perform similarly…"; "DP\_DP (43.9\%) performed better than DP\_DfD (3.03\%)". |
| `:36` (table header) | `\textbf{Algorithm / Source}` | `\textbf{Run (learner\_dataset)}` |
| `:40` | `\textit{Human / -}` | `\textit{Human dataset}` |
| `:41` | `DfD / Human` | `DfD\_Human` |
| `:42` | `IBC - Dreamer / Human` | `IBC\_Human` (see item 14 row 12: do not write "IBC-DfD"; say in the method that IBC uses the DfD world model) |
| `:43` | `DP / Human` | `DP\_Human` |
| `:44` | `DfD / DfD` | `DfD\_DfD` |
| `:45` | `IBC - Dreamer / DfD` | `IBC\_DfD` |
| `:46` | `DP / DfD` | `DP\_DfD` |
| `:47` | `DP / DP` | `DP\_DP` |
| `:50` (caption) | "Pusht trained algorithm demonstration stats by \textit{algorithm / source} e.g. DfD / Human is DreamerV3 trained on human demonstrations. Demonstrations were produced after 200k steps of training." | "PushT: success of the demonstrations produced by each trained run, named \textit{learner\_dataset} (DfD\_Human is DfD trained on the Human dataset; the first row is the Human dataset itself). Demonstrations were produced after 200k steps of training." |
| `:56` (caption `fig:demo_char_pusht`) | "Labels are like \textit{algorithm\_src} e.g. \textit{DP\_ DP} is Diffusion Policy trained on demonstrations produced from Diffusion Policy." | "Labels are \textit{learner\_dataset}, e.g. \textit{DP\_DP} is DP trained on the DP dataset." (also removes the stray space in `DP\_ DP`) |
| `appendix/a-research-methods.tex:94` (caption) | "IBC performs better with trained-policy demos than with Human demos … All trained-policy demonstrations came from DfD trained on human demonstrations" | "IBC\_DfD performs better than IBC\_Human … The DfD dataset came from DfD\_Human." (optional; captions currently readable) |
| `appendix/a-research-methods.tex:101` (caption) | same pattern | same. |

PushT table, ready to paste:

```latex
\begin{table}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{|l|l|l|l|}
\hline
\multicolumn{1}{|c|}{\textbf{Run (learner\_dataset)}} &
  n episodes &
  \multicolumn{1}{c|}{\begin{tabular}[c]{@{}c@{}}Mean\\ Success \%\end{tabular}} &
  \multicolumn{1}{c|}{\begin{tabular}[c]{@{}c@{}}Mean \\ Reward\end{tabular}} \\ \hline
\textit{Human dataset} & \textit{400} & \textit{85.8} & \textit{203.8} \\ \hline
DfD\_Human   & 400 & 81.0          & 132.7          \\ \hline
IBC\_Human   & 400 & 0.25          & 36.3           \\ \hline
DP\_Human    & 400 & 55.8          & 164.5          \\ \hline
DfD\_DfD     & 400 & \textbf{92.0} & \textbf{171.7} \\ \hline
IBC\_DfD     & 400 & 16.8          & 64.5           \\ \hline
DP\_DfD      & 400 & 3.03          & 48.5           \\ \hline
DP\_DP       & 400 & 43.9          & 143.3          \\ \hline
\end{tabular}
}
\caption{PushT: success of the demonstrations produced by each trained run, named \textit{learner\_dataset} (DfD\_Human is DfD trained on the Human dataset; the first row is the Human dataset itself). Demonstrations were produced after 200k steps of training.}\label{tbl:pusht}
\end{table}
```

### B3. Restocking — `sections/04-evaluation.tex`

| file:line | before | after |
|---|---|---|
| `:75` | "We generated three datasets from different machine sources: R2Dreamer because it was our best performing RLfD algorithm, DP trained on human demonstrations to represent supervised imitation learning, and a motion-planning algorithm to represent a classical robotics approach" | "We generated three machine datasets: **R2**, from R2\_Human, our best performing RLfD run; **DP**, from DP\_Human, representing supervised imitation learning; and **Planner**, from a motion-planning algorithm, representing a classical robotics approach" — this is where the four dataset names get defined for the restocking study. |
| `:77` | `\paragraph{DP}` | `\paragraph{The DP dataset}` (the paragraph is about the dataset, not the learner). |
| `:78` | "to produce the DP demonstrations. We then train DP on pixel-demonstrations…" | "to produce the DP dataset. We then train DP\_Human, DP\_DP, DP\_Planner and DP\_R2 on pixel observations…" (sentence is also unfinished — grammar lane). |
| `:83` (table header) | `Demonstration source` | `Dataset` |
| `:86` | `Motion planning` | `Planner` |
| `:87` | `R2Dreamer` | `R2` |
| `:89` (caption) | "Demonstration phase success (\%) broken down by task phase. Each source contains 72 demonstrations." | "Phase success (\%) of each restocking dataset. Each dataset contains 72 demonstrations." — **see Discrepancy D3 (Human has 74 tapes)**; also `\label{tab:phases}` at `:81` precedes `\caption` — move it after the caption or it will reference the wrong counter. |
| `:93` | "of DP, RLPD, R2Dreamer, and DV3" / "RL policies (RLPD, DV3, R2)" | "of DP, RLPD, R2, and DfD" / "(RLPD, DfD, R2)" (item 14). |
| `:98` (4×4 table header) | `Algorithm` | `Learner` |
| `:99` | `DreamerV3` | `DfD` |
| `:100` | `R2Dreamer` | `R2` |
| `:104` (caption) | "Pixel sparse +10: Sampled-action evaluation. Demonstration source order: Human / DP / Motion Planning / R2Dreamer." | "Restocking, pixel observations, sparse $+10$: sampled-action evaluation of every learner\_dataset run. Each cell lists the Human / DP / Planner / R2 datasets in that order." |
| `:110` (caption `fig:pickaplace_perf`) | "on human vs learned vs planned demonstrations. … DP and RLPD benefit from machine demonstrations (learned or planned) while WMs (R2Dreamer and DreamerV3)…" | "trained on the Human, DP, Planner and R2 datasets. … DP and RLPD benefit from the machine datasets (DP, Planner, R2) while the world-model learners (R2 and DfD)…" |
| `:143` (caption `fig:coverage`) | "all sources share their starts" | "all four datasets share their starts" (optional). |
| `:153` (caption `fig:tool`) | "for each source" | "for each dataset" (optional). |
| `sections/05-conclusion.tex:5` | "DP does better with the precise and clean demonstrations that come from motion planning and a performant converged model, while RLPD can only complete the task when given DP demonstrations." | "DP does better with the precise and clean Planner and R2 datasets, while RLPD can only complete the restocking task when given the DP dataset." |
| `sections/05-conclusion.tex:7` | "learns equally well from the human, DP, and R2Dreamer demonstrations, but do not learn well from motion planning demonstrations" | "learns equally well from the Human, DP and R2 datasets, but does not learn well from the Planner dataset" |
| `appendix/a-research-methods.tex:79` | "human \texttt{dHfull} with \texttt{all\_bnormclampS8ent5}, and machine \texttt{dDPfull} with …" | keep the internal names but map them once: "the Human dataset (\texttt{dHfull}…) and the DP dataset (\texttt{dDPfull}…)". (Whole paragraph is from the state-observation study; item 6/2 lanes decide whether it survives.) |

4×4 table, ready to paste (numbers unchanged):

```latex
\begin{table*}[htbp]\centering\small\setlength{\tabcolsep}{3pt}
\begin{tabular}{lcccc}\toprule
Learner & Picked & Placed & Slide & Nested \\\midrule
DfD  & 0.65 / 0.65 / 0.39 / 0.61 & 0.64 / 0.65 / 0.33 / 0.62 & 0.31 / 0.35 / 0.06 / 0.44 & 0.31 / 0.35 / 0.04 / 0.42 \\
R2   & 0.71 / 0.67 / 0.69 / \textbf{0.74} & 0.71 / 0.69 / \textbf{0.71} / 0.71 & 0.59 / 0.55 / 0.57 / 0.57 & 0.59 / 0.53 / 0.54 / 0.57 \\
RLPD & 0.13 / 0.38 / 0.00 / 0.00 & 0.10 / 0.33 / 0.06 / 0.07 & 0.00 / 0.17 / 0.00 / 0.00 & 0.00 / 0.16 / 0.00 / 0.00 \\
DP   & 0.34 / 0.31 / 0.63 / 0.65 & 0.21 / 0.24 / 0.69 / 0.70 & 0.04 / 0.03 / \textbf{0.61} / 0.60 & 0.03 / 0.02 / 0.57 / \textbf{0.60} \\
\bottomrule\end{tabular}
\caption{Restocking, pixel observations, sparse $+10$: sampled-action evaluation of every learner\_dataset run. Each cell lists the Human / DP / Planner / R2 datasets in that order (e.g. the second R2 entry is R2\_DP).}
\label{tab:phase-success-pixel-sparse10-sampled-eval}\end{table*}
```

Dataset-phase table, ready to paste (numbers unchanged; label moved after caption):

```latex
\begin{table}[htbp]\centering\small
\begin{tabular}{lrrrr}\toprule
Dataset & Picked & Placed & Slide & Nested \\\midrule
Human   & 88.9 & 54.2 & 33.3 & 15.3 \\
DP      & 88.9 & 56.9 & 31.9 & 16.7 \\
Planner & 94.4 & 94.4 & 94.4 & 94.4 \\
R2      & 94.4 & 94.4 & 94.4 & 94.4 \\
\bottomrule\end{tabular}
\caption{Phase success (\%) of each restocking dataset. Each dataset contains 72 demonstrations.}
\label{tab:phases}
\end{table}
```

### B4. `tables/*.tex` (state-observation study; only `sensitivity.tex` is `\input`)

These tables use "Endpoint / learner" rows (`Pick / DP`, `Place / R2`) with `H (%)` / `M (%)` columns, i.e. the *learner*
is named and the two datasets are abbreviated H/M. That is compatible with the convention (they compare `DP_Human` vs
`DP_DP`, etc.) provided the caption defines H = Human dataset, M = DP dataset. Required edits:

| file:line | before | after |
|---|---|---|
| `tables/sensitivity.tex:9` | `Pick / DV3 sample` | `Pick / DfD sample` (item 14) |
| `tables/phase_performance.tex:9` | `Pick / DV3$^{\dagger}$` | `Pick / DfD$^{\dagger}$` |
| `appendix/a-research-methods.tex:84` (sensitivity caption) | "…NE indicates a degenerate within-arm variance estimate." | append "H and M denote runs trained on the Human and DP datasets." |
| `tables/robomimic.tex:3-9` | `Learner / machine set`, `DP: MG200s`, `MH200`… | robomimic's own dataset names (PH/MH/MG); not `\input`, leave. |
| `tables/timing.tex:3` | `R2 outcome` | fine (learner R2). |

### B5. Figures with baked-in names (regenerate)

| figure | used at | labels now | should be | where the string lives |
|---|---|---|---|---|
| `figures/success_sample_home.png` | `04-evaluation.tex:109` | x: "Human / **Machine** / **Motion planning** / **R2-generated**"; panel titles "DreamerV3, R2Dreamer, RLPD, DP" | "Human / DP / Planner / R2"; "DfD, R2, RLPD, DP" | `HRI_results/pixel_4x4/run.py:19` `LABELS=['Human','Machine (DP)','Planner','R2 teacher']`; export via `HRI_results/pixel_4x4/export_charts.py` (→ `paper/figures/px_phase_2026-09-14/fig_results_4x4_sampled_home.png`); `baselines/diagnostics/px_results_4x4_plot.py:17` |
| `figures/training_phases.png` (and unused `phase_ignition.png`) | `:122` | legend "Human / **Machine** / **Motion planning**" (no R2 line), learners "R2Dreamer / DreamerV3 / RLPD / Diffusion Policy (offline)" | "Human / DP / Planner / R2"; "R2 / DfD / RLPD / DP" | `baselines/diagnostics/px_phase_analysis.py:119-143` |
| `figures/success_mode_home.png`, `success_training_home.png` | unused | same "Machine / Motion planning" | same | `HRI_results/pipeline/paper_success_violin.py` |
| `figures/coverage_and_inactivity.png` | `:142` | panels b/c: "Human / **DP-generated** / **Motion planning** / **R2-generated**"; panel d: "Human / DP / Planner / R2" (**inconsistent within one figure**) | "Human / DP / Planner / R2" in all panels | `HRI_results/dataset_dynamics_four_sources/coverage_figure.py:13` `LABELS = ['Human','DP-generated','Motion planning','R2-generated']` |
| `figures/tool_trajectories.pdf` | `:152` | column titles "Human / DP-generated / Motion planning / R2-generated" | "Human / DP / Planner / R2" | `HRI_results/dataset_dynamics_four_sources/tool_figure.py:10` |
| `figures/Ep Reward by Algorithm and Demonstration Source.png` | `:55` | x: `DP_DP, DP_Hu, DP_DfD, DfD_Hu, DfD_DfD, IBC_Hu, IBC_DfD, Human`; x-axis "Algorithm_Src"; title "…by Algorithm and Demonstration Source" | `DP_DP, DP_Human, DP_DfD, DfD_Human, DfD_DfD, IBC_Human, IBC_DfD, Human dataset`; axis "learner_dataset" | PushT analysis notebook/script (not found in this repo or `~/workspace/dreamerv3-torch` by `grep -rl 'IBC_Hu'`; ask the user where the PushT plots were made) |
| `figures/pusht_scalars_psuccess.png` | appendix `:93` | legend `DP_DfD*, DP_Hu*, DP_DP*, IBC_Hu, IBC_DfD, DfD_Hu, DfD_DfD, Human` | `_Hu` → `_Human` | same |
| `figures/pinpad_scalars_train_return.png` | appendix `:100` | `DfD_Hu, VMAIL_Hu, IBC_Hu, DfD_DfD, IBC_DfD, VMAIL_DfD, Human` | `_Hu` → `_Human` | same |
| `figures/training_procedure.png` | `03-method.tex:12` | cylinders "Human Demonstrations", "Model Demonstrations"; arrows $\pi^{DP}_H$, $\pi^{DP}_{DP}$ | "Human dataset", "DP dataset"; arrows "DP\_Human", "DP\_DP" | drawn by hand (no generator found) |
| `figures/three_training_procedure.png` | unused | adds "Motion Planned Demonstrations", $\pi^{MP}$, $\pi^{DP}_{MP}$ | "Planner dataset", "DP\_Planner" | same |

Caption of `fig:training_procedure` (`03-method.tex:13`) can be aligned now without touching the image: "DP\_Human is
trained on the Human dataset; its rollouts form the DP dataset, on which DP\_DP is trained."

## C. Where the stated convention is contradicted (summary)

1. **Order words vs example**: `03-method.tex:23` "by source and learner" but `RLPD_DP` is learner\_source.
2. **Separator**: slash "Algorithm / Source" (`04-evaluation.tex:36-47`, caption `:50`) vs underscore "algorithm\_src" (`:56`) vs math `$\{algorithm\}\_\{demonstration\_source\}$` (`03-method.tex:17`).
3. **Dataset names**: "Motion planning" (`:86`), "Motion Planning" (`:104`), "motion planning demonstrations" (`05-conclusion.tex:5,7`), "Planner" (figure panel d) — four spellings for one dataset; "R2Dreamer" (`:87`, `:104`, `05-conclusion.tex:7`), "R2-generated" (figures), "R2" (panel d) for another; "Machine" (figures) vs "DP" (`:85`) vs "DP-generated" (figures) for a third; "Hu" vs "Human" in the sim figures.
4. **"Machine" ambiguity**: prose uses machine = all non-human sources; figures use Machine = DP dataset (see item 11 file D2).
5. **Learner column header** "Algorithm" (`:98`) vs "learner" (`:23`, `tables/*.tex` "Endpoint / learner").

## Discrepancies

- **D1 — `03-method.tex:23` vs its own example** (above, C1).
- **D2 — `04-evaluation.tex:17`** ("DfD, RLPD, and Diffusion Policy each use the same fixed human and machine datasets across eight training seeds per source"; "100 human demonstrations from paper authors") sits under *Simulated Experiments*, but RLPD is restocking-only (`03-method.tex:50`) and the sim learners are DfD/IBC/VMAIL/DP (`:27`). The PinPad/PushT captions say n=8 / n=4 seeds (`appendix:94,101`). Either the paragraph is misplaced or the seed counts disagree.
- **D3 — dataset sizes.** `04-evaluation.tex:89` "Each source contains 72 demonstrations" and `:75` "All demonstrations sets contain the same initial conditions", but the Human restocking dataset has **74** tapes (`04-evaluation.tex:68` "We recorded 74 successful trials"; `appendix:24-26` "74 of 75"; `00_BRIEF.md`: human `dHfull_all_rns10h_img` 74, machine 72, planner 72, r2 72). The per-source phase table may have been computed on the 72 shared starts — the head session should confirm against `HRI_results/artifacts/dataset_phase_success_layout_2026-09-17/datasets` (path from `HRI_results/pixel_4x4/README.md`) before the caption says 72 for Human.
- **D4 — `\label{tab:phases}` before `\caption`** (`04-evaluation.tex:81`): the label binds to the previous float/section counter, so any `\ref{tab:phases}` would be wrong. Fixed in the paste-ready block above.
- **D5 — the figure vs the table disagree on what "Machine" is.** `success_sample_home.png` shows "Machine" n=8 (DP dataset) next to "Motion planning" and "R2-generated"; the 4×4 table (`:104`) calls the same column "DP". Same data, two names.
- **D6 — sim-figure seed counts** in the images (`pinpad n=8`, `pusht n=4`) vs `:17` "eight training seeds per source" — as D2.

## \llm notes inserted

- `sections/03-method.tex`, anchor "…that a DP teacher produced." — note: convention says "source and learner" but example is learner\_source; rename table + canonical names in `HRI_results/late_night_9_18/15_naming_convention.md`.
- `sections/04-evaluation.tex`, caption of `tbl:pusht` (anchor "…after 200k steps of training.") — note: rows to `learner\_dataset` form (`DfD\_Human`, `IBC\_Human`, …), "DreamerV3" → "DfD"; paste-ready table in §B2.
- `sections/04-evaluation.tex`, caption of `tab:phases` (anchor "Each source contains 72 demonstrations.") — note: header "Dataset", rows Planner/R2, label after caption, Human = 74 tapes?; §B3.
- `sections/04-evaluation.tex`, caption of `tab:phase-success-pixel-sparse10-sampled-eval` (anchor "…Motion Planning / R2Dreamer.") — note: "Learner" header, DfD/R2 rows, "Human / DP / Planner / R2" order wording, "Restocking" in caption; §B3 and items 13/14.
- `appendix/a-research-methods.tex`, caption of `tab:sensitivity` (anchor "…within-arm variance estimate.") — note: input row `Pick / DV3` → `Pick / DfD`; define H/M.

## Sources

- All in-scope .tex read in full (`cat -n`), root `~/workspace/overleaf/6a96f0e5337340dfd4edea88/`.
- `grep -n -E 'RLPD\\_DP|\\_\{demonstration|algorithm\\_src|Algorithm / Source|/ Human|/ DfD|/ DP|Demonstration source order|Motion planning|Motion Planning|Planner|R2Dreamer &|Learner / machine|Endpoint / learner|H \(\\%\)|MG200s|dHfull|dDPfull|\bR2\b'` over the in-scope files; `grep -rn -E '\\input\{tables|includegraphics' --include=*.tex .` for what is actually compiled.
- Figures viewed (Read tool / `pdftotext`): all listed in §B5.
- Label strings: `HRI_results/pixel_4x4/run.py:16-19, 288-342`, `HRI_results/pixel_4x4/export_charts.py:88-113`, `baselines/diagnostics/px_results_4x4_plot.py:17`, `baselines/diagnostics/px_phase_analysis.py:119-143`, `HRI_results/pipeline/paper_success_violin.py:19,97`, `HRI_results/dataset_dynamics_four_sources/{coverage_figure.py:13,tool_figure.py:10}`, `HRI_results/pixel_4x4/revise_success_figure.py:18`; `grep -rl 'IBC_Hu' --include=*.py --include=*.ipynb ~/workspace/dreamerv3-torch .` (no generator found for the sim figures).
- Dataset sizes/names: `HRI_results/late_night_9_18/00_BRIEF.md` ("Datasets" bullet), `HRI_results/DEMO_SETS_2026-09-11.md` (referenced, not re-read), `HRI_results/pixel_4x4/README.md` (dataset-dir path).
