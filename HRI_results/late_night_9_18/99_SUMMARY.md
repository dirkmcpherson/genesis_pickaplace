# Late-night list: cross-lane summary (2026-09-18 ~01:00)

All 16 items of `HRI_results/Late_Night_LLM_Work.md` were done by nine parallel lanes. Every deliverable is a numbered
MD file here (see `README.md`); the only edits to the Overleaf sources are 43 `\llm{...}` notes across six `.tex`
files (verified: after stripping the notes every changed line equals the original, whitespace aside). No figure,
table, bib or prose was changed. Nothing was pushed to Overleaf; see "How to apply" at the end.

## Status

| item | status | one-line result |
|---|---|---|
| 1 planner appendix | done | `\subsection{Motion-planning demonstrations}` + `tab:planner-hparams` written from `experiments/full_state_planner_2026-09-12/run.py` (RRT-Connect only for reset→pre-grasp; Cartesian DLS-IK servoing elsewhere; every offset in metres) |
| 2 RLPD | done | 28-row hparam table; all four datasets' runs print the identical `[cfg]` line on the cluster; actions/states byte-identical to source sets |
| 3 Diffusion Policy | done | hparam table from the runs' `train_config.json`; action columns byte-identical on all four datasets |
| 4 R2Dreamer | done | 48/48 run stamps verified (`rep_loss=r2dreamer`, tree `0b1b9d8`, clamp 10, shift4); appendix + eviction paragraph |
| 5 DfD | done | 32/32 robot runs verified (`rep_loss=dreamer`); separate tables for the robot task and the PinPad/PushT fork (`~/workspace/fastrl-pusht`) |
| 6 sim / real2sim | done | 50-sentence verdict table; corrected LaTeX for four §5.2 paragraphs and the whole real2sim appendix |
| 7 grammar/spelling | done | 18 compile/misrender defects + ~90 wording fixes as `file:line before → after` |
| 8 flow | done | per-section transitions, results-vs-prose drift table, paste-ready results paragraph for §5.2 |
| 9 appendix TOC | done | `titletoc` partial TOC, compiled locally against the project's `acmart.cls` |
| 10 BibTeX for code | done | one paste-ready block (R2Dreamer, lerobot, dreamerv3-torch, Genesis, SB3, RLPD, SAC, DrQ/DrQ-v2, TD-MPC2, DP IJRR, DreamerV3 Nature, Kinova, ROS) |
| 11 AI → machine | done | 6 tokens in 4 sentences; optional synonyms listed |
| 12 quality v source | done | 40-sentence S/Q classification, defining paragraph, 15 rewrites, Limitations paragraph |
| 13 restocking | done | 21 occurrences |
| 14 DreamerV3 → DfD | done | 26 occurrences; one citation instance to keep; R2Dreamer untouched |
| 15 naming convention | done | canonical names {Human, DP, Planner, R2} × learner\_dataset; rename tables for PushT, phase and 4×4 tables |
| 16 citation check | done | 4 undefined keys (`ball2023efficient`, `morihira2026r2d`, `rouder2009`, `rafailov2021vmail`→rename to `rafailov2021visual`); all six `\todo{cite}` sites answered |

## Findings that change what the paper says (ranked)

1. **DP claims are false for the reported runs.** Every DP run has `pretrained_backbone_weights: null` (no ImageNet
   backbone), and the four pixel DP arms train on the RAW sets (human incl. 10 no-pick tapes, idle 0.365). Only the
   state-based teacher that produced the DP dataset used the pruned, success-only data. (03, 12, 08)
2. **"Forty participants" has no source.** No participant identifier exists; all 74 trials are one afternoon
   (2024-12-18). IRB and the Xbox controller are in no project note. (06)
3. **The phase table's human row is not the training set.** 88.9/54.2/33.3/15.3 = 64/39/24/11 of a start-matched
   72-tape subset scored on the state re-execution; the learners trained on all 74 pixel tapes (65/42/26/13). The
   planner row (94.4 = 68/72) IS the training corpus and is correct; it counts attempts (two second attempts at failed
   starts, one succeeded), not first attempts (67/72), which the caption should say. (01, 03, 06, 15)
4. **World-model prose has drifted from the 4×4 results.** "WMs agnostic to every combination" holds for R2Dreamer
   only; DfD scores 0.04 on Planner v 0.31 on Human; RLPD is 0.00 on Planner and R2 (the `fig:pickaplace_perf`
   caption says the opposite); the DfD/R2-teacher cell has n=2. (05, 08)
5. **Quality and source are confounded two ways** and the paper argues source from the quality contrast: the sets form
   two tiers (Human/DP ≈17 % complete, Planner/R2 94 %); DP's gains are flat within tiers. The genuine source evidence
   is RLPD (only the DP set works), DfD (fails on Planner but not on equal-quality R2), and PushT DP\_DP > DP\_DfD. (12)
6. **Both robot-task world models run on the R2Dreamer chassis** (`rep_loss=dreamer|r2dreamer`); the cited
   `dreamerv3torch2024` fork was used only for PinPad/PushT. R2's loss is Barlow-Twins redundancy reduction, NOT
   "contrastive" as our own results docs say. (04, 05)
7. **Replay eviction unit:** rows are decisions (4 frames), so `max_size=5e5` = 2.0M frames; demos leave only in the
   last ~0.1M of the 2M budget. `AUDIT_R2D_NESTED_SPARSE_HvM` §7.1 ("gone by 0.5M") is the ×4 error. (04, 05)
8. **The +10 `home` row is non-terminal in every demo set** (planner/teacher terminals deliberately cleared to match)
   while online `home` terminates: RLPD and both world models bootstrap through the demonstrated reward. Consistent
   across datasets; must be disclosed. (02, 04, 05)
9. **RLPD paragraph:** entropy is actor-only (no backup entropy), LayerNorm affine shared across the 10 critics, UTD 10
   per decision (Ball et al. use 20), "dense reward not available" is not why RLPD is robot-only, "benefits from
   failed demos" is uncited and contradicted by our own all-data control (0.554 v 0.600, p 0.57). (02, 16)
10. **Simulated-task numbers are unverified locally:** scripts show 15+15 demos (PinPad), 70–78 v 100 (PushT), mode
    (not sampled) machine actions, `-ts 60000` not 200k. Which trials fed the figures is in wandb only. (05)
11. **Evaluation starts:** the 30-start set is one seed-0 draw over the July bounding box; 10/30 lie beyond the demo
    range and 5/30 inside the shelf footprint (CONFOUNDS 82 says 4; IC 8 was missed). `home` is unaffected. (06)
12. **"Machine" means the DP dataset only in every figure but all non-human sources in prose**; the AI→machine rename
    makes this worse unless dataset names {Human, DP, Planner, R2} are adopted. (11, 15)

## Will not compile / misrender as-is (fix before anything else)

- Two abstracts print back to back (`main.tex:48–49` inline copy after `\input{frontmatter/abstract}`).
- `\jss{...}` undefined (`05-conclusion.tex:41`; only `\js` exists).
- `\FloatBarrier` used in the appendix but `placeins` is not loaded.
- `\ref{appx:hyperparameters}`, `\ref{sec:experiments}`, `\ref{sec:restock}` undefined; `04-evaluation.tex:139` has
  `\label{appx:characterization_definitions}` where a `\ref` was meant and the target file is never `\input`.
- Duplicate `\label{fig:envs}` (intro and method) so the restocking text points at the sim figure.
- `\label{tab:phases}` before its `\caption`; template `\received{...}` dates and placeholder CCS terms print.
- `sample-base.bib` defines `dreamerv3torch2024` twice (BibTeX "Repeated entry"); four cited keys undefined (item 16).
- `@software` entries lose their URLs under `ACM-Reference-Format.bst`; `@misc` forms supplied (item 10).

## Figures that need regeneration (labels baked into the image)

`success_sample_home.png`, `training_phases.png`, `phase_ignition.png`, `coverage_and_inactivity.png`,
`tool_trajectories.pdf`, `training_procedure.png`, the PushT/PinPad curves and `Ep Reward … .png` (generators listed in
`11_ai_to_machine.md` / `14_dfd_naming.md`; no generator for the PushT/PinPad plots exists in this repo).

## Decisions only the user can make

- Whether the pixel DP arms should be re-run on pruned data (the paper's stated protocol) or the text changed to raw.
- The participant sentence (one session, unknown count) and the IRB/controller wording.
- Which phase-table numbers are of record (pixel sets, n=74/72) and whether planner is quoted first-attempt (67/72).
- Whether to keep "world models are source-agnostic" as a claim for R2Dreamer only.
- Reporting the sampled-vs-deterministic RLPD choice (sampled chosen after seeing it scores higher).

## `\llm` notes inserted (43)

`sections/01-introduction.tex` 4 · `02-related-work.tex` 3 · `03-method.tex` 10 · `04-evaluation.tex` 14 ·
`05-conclusion.tex` 8 · `appendix/a-research-methods.tex` 10 (counted from `git diff -U0 | grep -c '\\llm{'` in the
Overleaf checkout; each file's own list is in its `## \llm notes inserted` section). Each note is one line and names
its item file.

## How to apply

The Overleaf checkout (`~/workspace/overleaf/6a96f0e5337340dfd4edea88`) has the notes committed locally on top of
`c50d276 Update on Overleaf.`; `git push` from there sends them to Overleaf. The paste-ready LaTeX lives only in
these MD files; apply it section by section after the decisions above.
