# Item 14 — DreamerV3 → "DfD" everywhere

**Status: done (audit complete; no .tex text changed except two `\llm{}` anchors shared with item 15).** The body
already uses "DfD" in most of the results/discussion prose (26 occurrences). What remains: "DreamerV3"/"DV3" as the
*learner name* in the intro (`DV3`), the method paragraph heading and its first sentences (`Dreamer`), the PushT table
caption, the 4×4 restocking table row, one figure caption, the appendix subsection title, and two `tables/*.tex` rows
(`DV3`). Plus a naming inconsistency inside the paper itself: DfD is expanded as "**DreamerV3** from Demonstrations"
(abstract, method:27) and as "**Dreamer** from Demonstrations" (method:37). R2Dreamer is a different learner and is
listed below only so it is not touched. Five figure files print "DreamerV3" and need regeneration.

All paths are relative to `~/workspace/overleaf/6a96f0e5337340dfd4edea88/` (working copy at `c50d276`, 2026-09-17 23:20).

## A. The rule and its two exceptions

1. **DreamerV3 → DfD** for the learner we train (the PinPad4/PushT arm and the restocking "dreamer-loss" arm).
2. **Exception (a) — the citation instance.** The sentence that cites `hafner2023mastering` and defines the acronym must
   say DreamerV3 once: "DreamerV3 from Demonstrations (DfD)". Pick **one** definition site (recommended: the intro,
   `01-introduction.tex:38`, where R2 is also defined) and one expansion ("DreamerV3 from Demonstrations", matching the
   abstract). The method paragraph keeps the citation but should not re-define the acronym with a different expansion.
3. **Exception (b) — R2Dreamer is not renamed.** It is the other world-model learner (`morihira2026r2d`,
   `rep_loss=r2dreamer`); "R2Dreamer (R2)" stays. "Dreamer-family" (`02-related-work.tex:13`) and "a decoder-free
   DreamerV3-family variant" (`03-method.tex:37`, describing R2Dreamer) also stay — they name the family, not our learner.

## B. Every occurrence (grep + context), with the required action

| # | file:line | occurrence (context) | action |
|---|---|---|---|
| 1 | `main.tex:49` | "…world model based reinforcement learning (MBRL) system, DreamerV3 from Demonstrations (DfD), can effectively learn…" | Correct form (definition), **but this line is a duplicate stale abstract — delete the whole paragraph** (item 11 file, D1). `frontmatter/abstract.tex` does not mention DfD/DreamerV3 at all; if the abstract should name the baseline, add "DreamerV3 from Demonstrations (DfD)" there once. |
| 2 | `sections/01-introduction.tex:38` | "world-model learning from demonstrations using R2Dreamer (R2) and DreamerV3 (DV3)" | → "…using R2Dreamer (R2) and DreamerV3 from Demonstrations (DfD)". This becomes the single definition site; **`DV3` is never used again in the paper after this**, so nothing else depends on it. |
| 3 | `sections/02-related-work.tex:13` | "Dreamer-family methods train on imagined rollouts to optimize a policy~\citep{hafner2023mastering}, and demonstrations can bootstrap their learning…\cite{staley2024agent}" | keep (family + citation). |
| 4 | `sections/03-method.tex:4` | `% … IBC/VMAIL/DfD are NOT stale` | comment; ignore. |
| 5 | `sections/03-method.tex:27` | "PinPad4 uses DreamerV3 from Demonstrations (DfD), Implicit Behavior Cloning (IBC), and …" | → "PinPad4 uses DfD, Implicit Behavior Cloning (IBC), and …" (already defined in the intro). |
| 6 | `sections/03-method.tex:35` | `\paragraph{Dreamer}` | → `\paragraph{DfD}` (or `\paragraph{DreamerV3 from Demonstrations (DfD)}` if the user prefers the citation instance to live here instead of the intro). |
| 7 | `sections/03-method.tex:37` | "Dreamer is a model-based RL algorithm~\citep{hafner2023mastering}. It learns a recurrent world model…" | → "DreamerV3 is a model-based RL algorithm~\citep{hafner2023mastering}." — **this is the one place "DreamerV3" must remain** (the cited algorithm, before demonstrations are added). |
| 8 | `sections/03-method.tex:37` | "We follow previous work that inserts demonstrations directly into Dreamer's initial replay buffer at the start of training \cite{staley2024agent} and refer to it as Dreamer from Demonstrations (DfD)." | → "…into DreamerV3's initial replay buffer … and refer to the result as DreamerV3 from Demonstrations (DfD)." (fixes the "Dreamer from Demonstrations" vs "DreamerV3 from Demonstrations" inconsistency). |
| 9 | `sections/03-method.tex:37` | "We use a forked PyTorch implementation~\citep{dreamerv3torch2024}. We also train a variant, R2Dreamer, a decoder-free DreamerV3-family variant with a redundancy-reduction representation loss~\citep{morihira2026r2d}." | keep wording; see Discrepancy D1 (the restocking DfD is *not* the `dreamerv3torch2024` code) and D3 (`morihira2026r2d` is not in the .bib). |
| 10 | `sections/03-method.tex:42` | "We replace the actor-critic inside DreamerV3 with IBC, because we find IBC performs better on Dreamer's learned representations than on raw pixels through a CNN." | → "We replace the actor-critic inside DfD with IBC, because we find IBC performs better on DfD's learned representations than on raw pixels through a CNN." (The IBC arm runs on the DfD world model; see item 15 for what to call that row — "IBC", not "IBC - Dreamer".) |
| 11 | `sections/04-evaluation.tex:17, 24, 26, 30` | "DfD" ×17 | already correct. |
| 12 | `sections/04-evaluation.tex:42, 45` (PushT table rows) | "IBC - Dreamer / Human", "IBC - Dreamer / DfD" | → "IBC\_Human", "IBC\_DfD" (item 15 form). Do **not** write "IBC-DfD / DfD": under the run-naming convention `IBC_DfD` already means "IBC trained on the DfD dataset", so tagging the world model with "DfD" too would make the row ambiguous. State in the method text (row 10) that IBC uses the DfD world model. |
| 13 | `sections/04-evaluation.tex:50` (PushT table caption) | "e.g. DfD / Human is DreamerV3 trained on human demonstrations" | → "e.g. DfD\_Human is DfD trained on the Human dataset". |
| 14 | `sections/04-evaluation.tex:71` | `\cite{wu2023daydreamer, …}` | citation key, not a name. |
| 15 | `sections/04-evaluation.tex:93` | "We train 8-seeds … of DP, RLPD, R2Dreamer, and DV3 …" | → "…of DP, RLPD, R2, and DfD…". |
| 16 | `sections/04-evaluation.tex:93` | "RL policies (RLPD, DV3, R2) train with a sparse reward of $+10$" | → "RL policies (RLPD, DfD, R2)". |
| 17 | `sections/04-evaluation.tex:99` (4×4 table row, `tab:phase-success-pixel-sparse10-sampled-eval`) | `DreamerV3 & 0.65 / 0.65 / 0.39 / 0.61 & …` | → `DfD & 0.65 / 0.65 / 0.39 / 0.61 & …`. Header "Algorithm" → "Learner" (item 15). |
| 18 | `sections/04-evaluation.tex:100, 104` | "R2Dreamer" row / caption | learner name; keep (or "R2" for consistency with the run names — item 15 §B recommends "R2" in the Learner column). |
| 19 | `sections/04-evaluation.tex:110` (caption `fig:pickaplace_perf`) | "while WMs (R2Dreamer and DreamerV3) benefit similarly from all demonstration data" | → "while the world-model learners (R2 and DfD) benefit similarly…". |
| 20 | `sections/05-conclusion.tex:7, 9, 17, 20` | "DfD" ×6; "R2Dreamer" ×3 | already correct; R2Dreamer stays. |
| 21 | `appendix/a-research-methods.tex:38` | `\subsection{DreamerV3 and R2Dreamer}` | → `\subsection{DfD and R2Dreamer}`. |
| 22 | `appendix/a-research-methods.tex:94, 101` (curve captions) | "DfD" ×5, "All trained-policy demonstrations came from DfD" | already correct. |
| 23 | `tables/phase_performance.tex:9` | `Pick / DV3$^{\dagger}$ & sample & 2/2 & …` | → `Pick / DfD$^{\dagger}$`. (File is only `\input` by the un-input `04-evaluation-bk.tex`; fix anyway so it is not pasted back stale.) |
| 24 | `tables/sensitivity.tex:9` | `Pick / DV3 sample & 2/2 & 1.000 & NE` | → `Pick / DfD sample`. This table **is** `\input` at `appendix/a-research-methods.tex:83`. |
| 25 | `tables/e2e_performance.tex`, `secondary_performance.tex`, `timing.tex`, `robomimic.tex` | no Dreamer/DV3 tokens ("R2" only) | nothing. |
| 26 | `back/*.tex`, `appendix/b-online-resources.tex`, `frontmatter/abstract.tex` | none | nothing. |

R2Dreamer occurrences that must NOT change (for the record): `01-introduction.tex:38`; `03-method.tex:27, 37`; `04-evaluation.tex:75, 87, 93, 100, 104, 110`; `05-conclusion.tex:7 (×3)`; `appendix/a-research-methods.tex:38`.

Ready-to-paste LaTeX:

```latex
% sections/01-introduction.tex:38 (definition site)
We test offline imitation learning with Diffusion Policy (DP) and Implicit Behavior Cloning (IBC), RL from demonstrations using RLPD, and world-model learning from demonstrations using R2Dreamer (R2) and DreamerV3 from Demonstrations (DfD).

% sections/03-method.tex:27
PinPad4 uses DfD, Implicit Behavior Cloning (IBC), and Variational Model-based Adversarial Imitation Learning (VMAIL). PushT uses DfD, IBC, and Diffusion Policy (DP), and the restocking task uses DP, Reinforcement Learning from Prior Data (RLPD), R2Dreamer with demonstrations, and DfD.

% sections/03-method.tex:35-37
\paragraph{DfD}

DreamerV3 is a model-based RL algorithm~\citep{hafner2023mastering}. It learns a recurrent world model that predicts future latent states from past observations and actions. An actor-critic agent then trains inside imagined rollouts of that model. We follow previous work that inserts demonstrations directly into DreamerV3's initial replay buffer at the start of training~\cite{staley2024agent} and refer to the result as DreamerV3 from Demonstrations (DfD). We use a forked PyTorch implementation~\citep{dreamerv3torch2024}. We also train a variant, R2Dreamer, a decoder-free DreamerV3-family variant with a redundancy-reduction representation loss~\citep{morihira2026r2d}.

% sections/03-method.tex:42
We replace the actor-critic inside DfD with IBC, because we find IBC performs better on DfD's learned representations than on raw pixels through a CNN.

% sections/04-evaluation.tex:93
We train 8 seeds of DP, RLPD, R2, and DfD to investigate effects on these algorithms from demonstration source. … RL policies (RLPD, DfD, R2) train with a sparse reward of $+10$ for a successful slide.

% sections/04-evaluation.tex:99 (table row)
DfD & 0.65 / 0.65 / 0.39 / 0.61 & 0.64 / 0.65 / 0.33 / 0.62 & 0.31 / 0.35 / 0.06 / 0.44 & 0.31 / 0.35 / 0.04 / 0.42 \\

% appendix/a-research-methods.tex:38
\subsection{DfD and R2Dreamer}

% tables/phase_performance.tex:9 and tables/sensitivity.tex:9
Pick / DfD$^{\dagger}$ & sample & 2/2 & 70.0 & 63.3 & +6.7 [NE] & 0.333 & NE \\
Pick / DfD sample & 2/2 & 1.000 & NE \\
```

## C. Figure files that print "DreamerV3"/"DV3" (regenerate)

| figure | used at | in-image text | generator / label source |
|---|---|---|---|
| `figures/success_sample_home.png` | `04-evaluation.tex:109` | top panel title **"DreamerV3"** (others: R2Dreamer, RLPD, DP) | `HRI_results/pixel_4x4/export_charts.py:88` maps to `'{DreamerV3 losses}'`; upstream `baselines/diagnostics/px_results_4x4_plot.py:17` `LEARNERS = ["{DreamerV3 losses}", …]`; `HRI_results/pixel_4x4/run.py:16` `ALGS=['DreamerV3 losses',…]`. Change the display string to "DfD". |
| `figures/training_phases.png` | `04-evaluation.tex:122` | legend **"DreamerV3"** | `baselines/diagnostics/px_phase_analysis.py:119, 143` (`LEDGER_LEARNER = {"dreamer": "{DreamerV3 losses}", …}`) via `HRI_results/pixel_4x4/upstream_phase_analysis.py` |
| `figures/phase_ignition.png` | commented out (`:116`) | legend **"DreamerV3"** | same as above; also `HRI_results/ignition_review/run.py:27` `label(...)` returns `'DreamerV3'` |
| `figures/success_mode_home.png`, `success_training_home.png` | unused | panel title **"DreamerV3"** | `HRI_results/pipeline/paper_success_violin.py:19, 97` (`'DreamerV3 losses'`, display `'DreamerV3\nlosses'`) |
| `figures/pusht_results.png` | unused | title "Results (**DreamerfD** 100 smoothing)", legend "DfD" | stale; if used, retitle. |
| `figures/Ep Reward…png`, `pusht_scalars_psuccess.png`, `pinpad_scalars_train_return.png` | `04-evaluation.tex:55`, appendix `:93, :100` | already "DfD" (`DfD_Hu`, `DfD_DfD`) | fine for item 14 (item 15 wants `_Hu` → `_Human`). |
| `figures/e2e_performance.pdf`, `pickaplace_perf.png`, `coverage_and_inactivity.png`, `tool_trajectories.pdf`, `training_procedure.png` | — | no Dreamer text (only "R2Dreamer"/"R2-generated"/"WM") | nothing. |

## Discrepancies

- **D1 — "DfD" is two codebases.** `03-method.tex:37` says DfD is "a forked PyTorch implementation~\citep{dreamerv3torch2024}" (NM512 `dreamerv3-torch`, `~/workspace/dreamerv3-torch`). That is true for PinPad4/PushT. For the restocking task the "DfD" arm is the **r2dreamer chassis with `rep_loss=dreamer`** (`00_BRIEF.md` "Learners" bullet; `baselines/diagnostics/px_phase_analysis.py:163` `algorithm="dv3 (DreamerV3 losses in the r2dreamer chassis), pixels"`; cluster tree `$W/r2dreamer_px`, launcher `cluster/wmfix_full.sbatch`). Renaming both "DfD" is fine, but the method/appendix must say which implementation each environment used (item 5 lane owns the hyperparameter appendix; flagged here because the name change hides the difference).
- **D2 — inconsistent expansion.** "DreamerV3 from Demonstrations" (`main.tex:49`, `03-method.tex:27`) vs "Dreamer from Demonstrations" (`03-method.tex:37`). Recommend the former everywhere (also matches `staley2024agent`'s usage).
- **D3 — bib.** `morihira2026r2d` (cited `03-method.tex:37`) has **no entry** in `sample-base.bib` or `software.bib` (`grep -i morihira` → nothing) — will typeset as `[?]`. `dreamerv3torch2024` is defined **twice** in `sample-base.bib` (lines 308 and 622, identical) — BibTeX "repeated entry" warning. Both belong to items 10/16 but are load-bearing for the DfD/R2 sentences.
- **D4 — the intro defines "DV3", the abstract defines "DfD", the method defines "DfD" again.** After the change there is exactly one definition (intro:38) and one citation instance (method:37).

## \llm notes inserted

- `sections/04-evaluation.tex`, anchor "Demonstration source order: Human / DP / Motion Planning / R2Dreamer." (caption of `tab:phase-success-pixel-sparse10-sampled-eval`): note says the row "DreamerV3" → "DfD", learner column names, dataset order wording — see this file and `15_naming_convention.md`.
- `sections/04-evaluation.tex`, anchor "Demonstrations were produced after 200k steps of training." (caption of `tbl:pusht`): note covers "DreamerV3" → "DfD" in the caption and the `IBC - Dreamer` rows (shared with item 15).
- `appendix/a-research-methods.tex`, anchor "NE indicates a degenerate within-arm variance estimate." (caption of `tab:sensitivity`): note that the input row `Pick / DV3 sample` → `Pick / DfD sample`.

## Sources

- All in-scope .tex read in full (`cat -n`), root `~/workspace/overleaf/6a96f0e5337340dfd4edea88/`.
- `grep -n -o -E '[^ ]*[Dd][Rr][Ee][Aa][Mm][Ee][Rr][^ ]*|\bDV3\b|\bdv3\b|\bDfD\b|dreamerv3torch2024|hafner2023mastering'` over the in-scope files (counts per line, then read in context).
- `sample-base.bib:41-46` (`hafner2023mastering`), `:306-318` and `:620-632` (duplicate `dreamerv3torch2024`), `grep -n -i 'morihira|r2dreamer' sample-base.bib software.bib` (none).
- Figures viewed (Read tool): `success_sample_home.png`, `success_mode_home.png`, `success_training_home.png`, `training_phases.png`, `phase_ignition.png`, `pusht_results.png`, `pusht_scalars_psuccess.png`, `pinpad_scalars_train_return.png`, `Ep Reward by Algorithm and Demonstration Source.png`.
- Generators: `HRI_results/pixel_4x4/{run.py:16-19,export_charts.py:88}`, `baselines/diagnostics/{px_results_4x4_plot.py:17,px_phase_analysis.py:119-168}`, `HRI_results/pipeline/paper_success_violin.py:19,97`, `HRI_results/ignition_review/run.py:27`.
- `HRI_results/late_night_9_18/00_BRIEF.md` ("Learners" bullet: which code each DfD arm ran).
