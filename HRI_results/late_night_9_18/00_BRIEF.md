# Late-night paper work, 2026-09-17/18 — shared brief for every lane

Source list: `HRI_results/Late_Night_LLM_Work.md` (16 numbered items). Each lane claims its items by creating
`HRI_results/late_night_9_18/NN_<slug>.md` (NN = item number, zero-padded) FIRST, then fills it with the deliverable.

## Rules (from the user)
- The LaTeX project is at `~/workspace/overleaf/6a96f0e5337340dfd4edea88/` (a git repo). Main file `main.tex`; body in
  `sections/01-introduction.tex … 05-conclusion.tex`; abstract `frontmatter/abstract.tex`; appendix
  `appendix/a-research-methods.tex` (+ `b-online-resources.tex`, `c-llm-generated.tex` not \input'd); tables in `tables/`;
  bibliography `sample-base.bib` (+ `software.bib`). Compiler pdfLaTeX + BibTeX, acmart sigconf.
- **Do NOT edit any .tex except to add notes of the form `\llm{...}`** (macro exists in preamble.tex). Insert notes with the
  Edit tool only (small unique anchors, never rewrite a file) — other lanes edit the same files concurrently. Keep notes
  short and point at your MD file for the full text. Every LaTeX you write (paragraphs, tables, appendix sections) goes
  in your MD file inside a ```latex fence, ready to paste.
- Cluster (`ssh pax`, `$LAB=/cluster/tufts/shortlab/jstale02`, `$W=$LAB/wm_fix_2026-09-03`) is READ-ONLY for you: no
  sbatch/scancel/rm, no python that opens a Genesis world on the login node. `cat`/`grep`/`ls`/`sha256sum`/`head` are fine.
- Every result or hyperparameter you quote must carry the file/command it came from. If the paper's prose and the code
  disagree, say so explicitly (a "DISCREPANCIES" section), do not silently pick one.
- Do not commit. I (the head session) commit at the end.

## What "the paper" currently reports (so you know which runs matter)
- Robot task = shelf **restocking**: pick → place → slide into contact with a goal can. Genesis 0.2.1 digital twin, world
  `gc_kp4_riser3_shelf6`, action_repeat 4, 6-d delta joint targets (cap 0.025) + gripper, horizon 1200 sim steps.
- The primary robot study is now **PIXEL observations** (two 64×64 cameras top+wrist, plus 8-d proprio: 6 joints, gripper
  motor, grip effort; NO object/goal pose), reward ladder **`nested_sparse10`** (+10 once for `home`, terminal; tip
  terminal, 0), **4 demonstration datasets × 4 learners**, target 8 seeds/cell. Read `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`
  and `HRI_results/STATE_PIXEL_4x4_2026-09-16.md` first, then `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`
  (how every run was launched), `paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md`.
- Datasets (`HRI_results/DEMO_SETS_2026-09-11.md`): human `dHfull_all_rns10h_img` (74 tapes), machine (DP teacher)
  `dDPfull_first_rns10h_img` (72), planner `planner_matched72…` (72; `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`,
  code under `experiments/full_state_planner_2026-09-12/`, cluster campaign `cluster/planner72_px_2026-09-15/`),
  r2dreamer-teacher set (72; `HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`, `cluster/r2teacher_px_2026-09-15/`).
- Learners: {DreamerV3 losses} = the r2dreamer PyTorch chassis with `rep_loss=dreamer` (reconstruction) — the paper calls
  this DfD/DreamerV3; {r2dreamer} = same chassis, `rep_loss=r2dreamer`; {RLPD} = our SB3 reimplementation
  (`baselines/rl/train_rlpd.py`, `rlpd_sac.py`, `rlpd_pixel.py`); {Diffusion Policy} = lerobot fork
  (`~/workspace/lerobot`, launchers `cluster/sbatch_dp_px.sh`, `cluster/submit_ah_dp_px.sh`, `cluster/sbatch_dp_e2e.sh`).
  World-model code: local `~/workspace/r2dreamer` (configs/ incl. `genesis_full_pixel.yaml`), cluster tree of record
  `$W/r2dreamer_px`; launcher `cluster/wmfix_full.sbatch`, `cluster/submit_px_batch.sh`. The simulated-environment
  (PinPad4/PushT) DfD/IBC/VMAIL runs used `~/workspace/dreamerv3-torch` (fork) — separate code from the robot task.
- Registrations/amendments: `paper/PHASE_PLAN_2026-09-04.md` (amendments (ad)…(ai), P-MP-20260915). Confound ledger:
  `paper/CONFOUNDS.md`. Real2sim: `paper/E2E_AUDIT_BRIEF_2026-09-10.md`, `paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md`,
  CONFOUNDS row 46 (frame-rate/timing), `paper/EEF_ACTION_DIST_2026-09-07.md`, `can_pos_recovery/`, `CAN_STARTING_POSITION.md`.
- Naming convention the paper wants: datasets named by source (Human, DP, Planner, R2), runs by learner_source
  (e.g. RLPD_DP = RLPD trained on the DP-teacher machine demonstrations). "AI" → "machine" everywhere. DreamerV3 → "DfD".
  Task → "restocking".

## Output format for each item file
1. `# Item NN — <title>` and a one-paragraph status (done / partial / blocked, and why).
2. The deliverable (LaTeX in fences; tables; lists of corrections as `file:line — before → after`).
3. `## Discrepancies` (paper vs code/notes) with evidence paths.
4. `## \llm notes inserted` — list of file + anchor + note text.
5. `## Sources` — every file/command consulted.
