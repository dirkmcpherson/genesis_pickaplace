# Item 11 — "AI" → "machine" everywhere

**Status: done (audit complete; no .tex text changed, per the brief).** Every in-scope .tex file was read in full
(not just grepped) so that `VMAIL` / `MAIN` and the LLM-use statement are not mangled. There are **4 sentences (6
tokens)** where "AI" names a demonstration source; all are in `sections/01-introduction.tex` and
`sections/04-evaluation.tex`. One further "AI" (the *Generative AI Use Statement* heading) refers to LLMs, not
demonstrations, and must stay. Optional synonyms ("model-generated", "trained-policy", "learned", "synthetic") are
listed separately with a keep/change recommendation. Figure files that print "Model Demonstrations" are listed at the
end because LaTeX cannot change them.

All paths below are relative to `~/workspace/overleaf/6a96f0e5337340dfd4edea88/`. Line numbers are from the tree as of
2026-09-17 23:20 (git `c50d276`, working copy).

## A. Mandatory changes (the user's rule: "AI" → "machine")

| # | file:line | before | after |
|---|---|---|---|
| 1 | `sections/01-introduction.tex:34` | "some LfD research uses demonstrations that come from trained AI models." | "some LfD research uses machine demonstrations that come from trained models." (literal alternative: "…from trained machine models" — reads worse; the rewrite keeps the noun phrase "machine demonstrations" that the rest of the paragraph already uses) |
| 2 | `sections/01-introduction.tex:36` | "AI-generated trajectories are therefore not necessarily Markovian from the learner's observed state." | "machine-generated trajectories are therefore not necessarily Markovian from the learner's observed state." |
| 3 | `sections/01-introduction.tex:36` | "Human and AI demonstrations may consequently differ in ways…" | "Human and machine demonstrations may consequently differ in ways…" |
| 4 | `sections/04-evaluation.tex:26` | "When comparing the AI-sourced demonstrations, there was no significant difference…" | "When comparing runs trained on the DfD dataset, there was no significant difference…" (the "AI-sourced" set in PinPad4 is specifically the DfD-teacher dataset; "machine-sourced demonstrations" is the literal replacement, but the dataset name is what item 15 asks for) |
| 5 | `sections/04-evaluation.tex:26` | "The addition of the AI-generated demonstration set significantly improved…" | "The addition of the machine-generated (DfD) demonstration set significantly improved…" |

Ready-to-paste LaTeX for the two intro sentences and the PinPad4 sentence:

```latex
% sections/01-introduction.tex:34 (replace the one sentence)
Despite LfD's intuitive focus on human demonstrations, some LfD research uses machine demonstrations that come from trained models.

% sections/01-introduction.tex:36 (two sentences)
A learned policy may use a single observation, a context window, or a recurrent internal state to determine the next action; machine-generated trajectories are therefore not necessarily Markovian from the learner's observed state.
% ...
Human and machine demonstrations may consequently differ in ways that are not apparent from task-success metrics alone.

% sections/04-evaluation.tex:26 (two sentences)
When comparing runs trained on the DfD dataset, there was no significant difference ($p=0.116$) between DfD and IBC, no significant difference ($p=0.113$) between DfD and VMAIL, and no significant difference ($p=0.412$) between IBC and VMAIL. The addition of the machine-generated (DfD) demonstration set significantly improved the average episode reward for IBC ($p=0.000003$) and VMAIL ($p=0.00009$), but did not have a meaningfully significant impact on DfD ($p=0.011$).
```

## B. "AI" tokens that must NOT change (false positives checked by reading context)

| file:line | token | why it stays |
|---|---|---|
| `sections/05-conclusion.tex:35` | `\paragraph{Generative AI Use Statement}` | Refers to LLM use in writing/code, not to demonstrations. |
| `sections/03-method.tex:27, 44, 46 (×2)`, `sections/04-evaluation.tex:24, 26 (×5)`, `sections/05-conclusion.tex:15, 17`, `appendix/a-research-methods.tex:101` | `VMAIL` | Algorithm name (Variational Model-based Adversarial Imitation Learning). `grep -w AI` does not match these, but a naive `sed s/AI/machine/` would. |
| `sections/05-conclusion.tex:41` | `MAIN` (inside `\jss{DO NOT INCLUDE THE APPENDIX IN THE MAIN DOCUMENT…}`) | Editor note. |
| `sections/03-method.tex:4` | `% … IBC/VMAIL/DfD are NOT stale` | LaTeX comment. |

No occurrences of "Airtable", "AI-" inside citation keys, or "AI" in `tables/*.tex`, `back/*.tex`, `frontmatter/*.tex`, `main.tex`, or `appendix/b-online-resources.tex`.

Out of scope but worth knowing: `sections/04-evaluation-bk.tex` (not `\input`) has 4 more "AI" hits; `appendix/c-llm-generated.tex` (not `\input`) has none.

## C. Optional candidates for consistency (not "AI"; user's call)

The paper's own vocabulary elsewhere is "machine demonstrations" (abstract, intro:34, method:17/21, eval:17/75/110). These are the remaining synonyms:

| file:line | phrase | recommendation |
|---|---|---|
| `main.tex:49` | "example trajectories from expert users or trained policies" / "demonstrations that come from trained models rather than human beings" | **Change** to "…from expert users or machines" / "…that come from machines rather than human beings" — **but see Discrepancy D1: this whole paragraph is a second, stale abstract pasted inline after `\input{frontmatter/abstract}`; deleting it is the better fix.** |
| `sections/01-introduction.tex:24` | "less work has been done with untrained humans, trained models, and classical robotics approaches" | **Keep.** Here "trained models" and "classical robotics" are the two *kinds* of machine source, which the paper deliberately distinguishes. |
| `sections/01-introduction.tex:38` | "models trained on human, policy, or classically planned demonstrations" | **Change** to "trained on human, trained-policy, or classically planned demonstrations" (grammar) or, using item 15's dataset names, "trained on the Human, DP, Planner and R2 datasets". "policy" alone is ambiguous. |
| `sections/04-evaluation.tex:17` | "100 machine-generated trajectories collected from a DfD policy" | Already "machine". Keep (but see item 15 file: this paragraph is misplaced — it names RLPD in the *simulated* section). |
| `sections/04-evaluation.tex:110` (fig caption) | "human vs learned vs planned demonstrations … machine demonstrations (learned or planned)" | **Optional**: "learned" → "DP- or R2-generated". Already uses "machine". |
| `sections/04-evaluation.tex:134` | `\elaine{… how synthetic demonstrations differ …}` | Editor note; leave. |
| `sections/05-conclusion.tex:12` | "do not learn the same way from model-generated demonstrations" | **Change** → "machine-generated demonstrations" (pure synonym; no reason to keep two words for one thing). Note the sentence is also truncated ("…results are repo") — grammar lane. |
| `sections/05-conclusion.tex:15` | "the Markovian nature of trained-policy demonstrations" ×2 | **Keep.** The argument is specifically about *policies* being Markov in the observed state; the Planner dataset is also "machine" but is not a learned policy, so "trained-policy" is the precise term here. |
| `sections/05-conclusion.tex:20` | "treat human and model-generated demonstrations the same" | **Change** → "human and machine demonstrations". |
| `appendix/a-research-methods.tex:94, 101` (fig captions) | "trained-policy demos" ×4, "All trained-policy demonstrations came from DfD" | **Optional**: "machine (DfD-generated) demos". The captions already say the demos came from DfD, so the meaning is unambiguous either way. |
| `frontmatter/abstract.tex:8` | "% … pre-trained policy rollouts …" | Commented-out TW draft; ignore. |

## D. Figures whose pixels say "model"/"AI" (LaTeX cannot fix; regenerate)

Opened with the Read tool (PNG) / `pdftotext` (PDF):

| figure | used by | text in the image | action |
|---|---|---|---|
| `figures/training_procedure.png` | `sections/03-method.tex:12` (Fig. `fig:training_procedure`) | cylinders "Human Demonstrations", **"Model Demonstrations"**; arrows $\pi^{DP}_{H}$, $\pi^{DP}_{DP}$ | Regenerate: "Model" → "Machine" (item 15 would prefer the cylinders be named "Human dataset" / "DP dataset" and the arrows DP\_Human / DP\_DP). |
| `figures/three_training_procedure.png` | not included anywhere | "Human Demonstrations", **"Model Demonstrations"**, "Motion Planned Demonstrations"; $\pi^{DP}_H$, $\pi^{DP}_{DP}$, $\pi^{MP}$, $\pi^{DP}_{MP}$ | Same, if it is ever used ("MP" would become "Planner"). |
| `figures/Ep Reward by Algorithm and Demonstration Source.png` | `sections/04-evaluation.tex:55` | x-labels `DP_DP, DP_Hu, DP_DfD, DfD_Hu, DfD_DfD, IBC_Hu, IBC_DfD, Human`; axis "Algorithm_Src" | No "AI" text. Item 15: `_Hu` → `_Human`, axis → "learner\_dataset". |
| `figures/pickaplace_perf.png` | not included (only in `04-evaluation-bk.tex`) | "human / machine" columns; title "…DP ≈ indifferent, RLPD null, world model prefers human" | Already "machine". Stale (pick-only, state world). |
| `figures/success_sample_home.png`, `success_mode_home.png`, `success_training_home.png`, `training_phases.png`, `phase_ignition.png` | `success_sample_home` at `04-evaluation.tex:109`; `training_phases` at `:122`; others unused | Source labels "Human / **Machine** / Motion planning / R2-generated" | No "AI". But "Machine" here means the DP dataset only, while the text uses "machine" for all three non-human datasets — see item 15 file §D. |
| `figures/coverage_and_inactivity.png`, `figures/tool_trajectories.pdf` | `04-evaluation.tex:142, 152` | "Human / DP-generated / Motion planning / R2-generated" (panel d of coverage: "Human / DP / Planner / R2") | No "AI". Item 15 naming. |
| `figures/pusht_scalars_psuccess.png`, `pinpad_scalars_train_return.png` | `appendix/a-research-methods.tex:93, 100` | legends `DfD_Hu, IBC_Hu, VMAIL_Hu, DfD_DfD, IBC_DfD, VMAIL_DfD, DP_Hu*, DP_DfD*, DP_DP*, Human` | No "AI". Item 15: `_Hu` → `_Human`. |
| `figures/pusht_results.png` | unused | title "Results (**DreamerfD** 100 smoothing)", legend "Human, DP_DP, DfD" | No "AI"; stale title (item 14). |
| `figures/e2e_performance.pdf` | unused | "Human demos / Machine demos … Awaiting validated results" | Placeholder; already "machine". |

## Discrepancies

- **D1 — two abstracts.** `main.tex:47-50` does `\input{frontmatter/abstract}` *and then* contains a second full abstract paragraph inline (line 49, ending `\elaine{cut by 9-10 lines}`). The compiled abstract is therefore both paragraphs back to back. The inline one is the older text ("expert users or trained policies", "trained models rather than human beings", "DreamerV3 from Demonstrations (DfD)"). Recommend deleting `main.tex:49` rather than editing its wording. (Evidence: `main.tex:47-50`; `frontmatter/abstract.tex:3`.)
- **D2 — "machine" has two meanings.** Prose: machine = every non-human source (`03-method.tex:17,21`; `04-evaluation.tex:75` "three datasets from different machine sources"). Figures and the 4×4 analysis code: "Machine" = the DP-teacher dataset only (`success_sample_home.png` x-axis; `HRI_results/pixel_4x4/run.py:19` `LABELS=['Human','Machine (DP)','Planner','R2 teacher']`; `baselines/diagnostics/px_phase_analysis.py:134-135` keys `("{DreamerV3 losses}", "machine")` = the `dDPfull_first_rns10h_img` set). Replacing "AI" with "machine" makes this collision more visible; item 15's dataset names (Human / DP / Planner / R2) resolve it.
- **D3 — `04-evaluation.tex:17` (Simulated Experiments)** says "DfD, RLPD, and Diffusion Policy each use the same fixed human and machine datasets across eight training seeds per source", but `03-method.tex:27,50` say RLPD is used on the robot task only and PinPad4/PushT use DfD/IBC/VMAIL/DP. This paragraph appears to be robot-task text left under the simulated heading.

## \llm notes inserted

None for this item (the brief says sparingly; the two anchors used for items 14/15 in `03-method.tex` and the table captions point at this directory). The PinPad4 sentence (`04-evaluation.tex:26`) is covered by the item-15 note on the PushT table caption a few lines below it.

## Sources

- `~/workspace/overleaf/6a96f0e5337340dfd4edea88/{main.tex, frontmatter/abstract.tex, sections/01-introduction.tex, 02-related-work.tex, 03-method.tex, 04-evaluation.tex, 05-conclusion.tex, appendix/a-research-methods.tex, appendix/b-online-resources.tex, tables/*.tex, back/*.tex, preamble.tex}` — read in full with `cat -n`.
- `grep -n -E '\bAI\b'`, `grep -n -o -E '\w*AI\w*'` (false-positive census), `grep -n -i -E 'model-generated|trained-policy|policy demonstrations|learned demonstrations|synthetic|trained models|policy rollouts'` over the in-scope files.
- Figures opened: `figures/{training_procedure,three_training_procedure,Ep Reward by Algorithm and Demonstration Source,pickaplace_perf,success_sample_home,success_mode_home,success_training_home,training_phases,phase_ignition,coverage_and_inactivity,pusht_scalars_psuccess,pinpad_scalars_train_return,pusht_results}.png` (Read tool); `pdftotext figures/{tool_trajectories,e2e_performance}.pdf`.
- Label definitions in generators: `HRI_results/pixel_4x4/run.py:16-19`, `HRI_results/dataset_dynamics_four_sources/{coverage_figure.py:13,tool_figure.py:10}`, `baselines/diagnostics/px_phase_analysis.py:119-143`.
- `HRI_results/late_night_9_18/00_BRIEF.md` (naming convention the paper wants).
