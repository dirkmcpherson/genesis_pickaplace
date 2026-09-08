# Audit guide — how to check this project's writing for fidelity (2026-09-08)

*For an agent or person with the code and a cluster login, auditing prose that cites these results. It contains **no
numbers**: every number in this directory is generated, and a hand-copied figure in a guide is exactly the failure this
guide exists to catch. It contains the **procedure** for verifying any number, the **registration chain** a claim must sit
inside, the **traps** that have actually fired in this project, and the **phrasings** each class of result can and cannot
support.*

Companion documents: `WHAT_STANDS_2026-09-08.md` (what to rely on today), `results.md` (the numbers),
`METHODOLOGY.md` (how each learner was made to work), `../paper/REVIEWER_ENTRY_2026-09-08.md` (the adversarial entry
point, including where we think we are most likely to be wrong).

---

## 1. The rule that makes an audit possible

**No number in a manuscript may exist only in prose.** Every figure must be traceable to a line of `results.md`, and
every line of `results.md` to an evaluation artefact on the cluster. The chain is three hops and each is mechanical:

```
manuscript sentence
   -> results.md row              (id in the leftmost column)
   -> seed_counts.csv             (one row per seed, with the `path` column)
   -> a file on the cluster       (the evaluator's own output; the ground truth)
```

If a sentence cannot be walked back to a cluster file this way, it is not yet a result — it is a recollection. Several
sentences in this project's older drafts failed that test and were wrong; that is the empirical basis for the rule.

## 2. Walking the chain — the exact commands

```bash
cd HRI_results
V=~/workspace/genesis_sim2real/venv/bin/python

# (a) which comparisons exist, and their status
$V make_tables.py && sed -n '1,60p' results.md

# (b) the per-seed data behind one comparison, with the artefact path for every seed
grep '^place_polE' seed_counts.csv | column -t -s,

# (c) the artefact itself, on the cluster -- this is the ground truth
ssh pax 'cat <the path column from (b)>'

# (d) rebuild from the cluster rather than from the committed CSV
$V make_tables.py --refresh        # re-harvests, then rebuilds

# (e) prove the pipeline is deterministic: a rebuild with unchanged data is byte-identical
$V make_tables.py && git status --short      # any diff here means the DATA moved
```

A rebuild that changes `results.md` when you did not intend to change data is itself a finding — report it rather than
committing it.

**Statistics self-test:** `$V hri_stats.py` re-derives the permutation test, MDE and posterior against cells whose values
are known from the published documents. If it fails, distrust every inferential column until it passes.

**Prose-vs-table check:** `doc_checks.py` holds the numbers as they appear in the documents of record, and every rebuild
compares them against the regenerated cells. Disagreements are printed into `results.md` under *Doc disagreements*.
**Read that section before trusting any sentence quoted from an older paper document** — it is the mechanised form of
this entire audit, and it has caught real errors (a double-counted seed set among them).

## 3. Was the claim registered before it was measured?

This project's honesty protocol is pre-registration. A result whose prediction was written after the readout is not a
confirmation, and a manuscript must not present it as one.

- **The registry** is `../paper/PHASE_PLAN_2026-09-04.md`, a chain of lettered amendments. Each names its arms, its
  statistic, its predictions and its disconfirming branches.
- **The check is `git log`,** because the file is committed before the jobs are submitted:
  ```bash
  git log --oneline --follow paper/PHASE_PLAN_2026-09-04.md      # when each amendment landed
  git show --stat <commit>                                        # and what else moved with it
  ssh pax 'sacct -u jstale02 -S <date> --format=JobID,JobName,Submit,State | head'   # when the jobs were submitted
  ```
  **Registration commit time must precede job submit time.** If it does not, the result is exploratory — say so.
- **Which amendment governs which cell:** (d) end-to-end arm, (e) matched-N, (f) the contact entry bank, (g)/(g′)
  `contact_push`, (h) DP + RLPD place, (i) the symmetry control, (j) the evaluation fixes, (l) `slide_success`,
  (p) the withdrawal of (l)'s grip clause, (u) ignition replication, **(v) the de-confounded end-to-end machine arm**.
- **A withdrawn amendment does not vanish.** (l) is implemented, computed and reported — and withdrawn by (p). Prose may
  cite `slide_success` as a diagnostic and must not cite it as the task outcome. The code enforces the same distinction.

## 4. The traps — each one has fired here at least once

Check each against any sentence before it ships. These are not hypothetical; every entry is a mistake this project
actually made and corrected.

| # | trap | how to check it |
|---|---|---|
| 1 | **A predicate name that means something else.** `nested` in the full scope is the *training proxy*; `nested_honest` runs the 100-step settle. `contact` counts carrying the can without releasing. `placed` in the full scope is a stale base-world band and is structurally unearnable. | Never quote a predicate without checking which one the cell computed. `grep -n 'def _nested\|def _contact\|placed_v2' baselines/rl/full_env.py baselines/genesis_can_env.py` |
| 2 | **A tape's own `stage` field is stale.** The recorder's label disagrees with the honest metric on 13 w3 tapes. | Use the evaluator's JSON, never the tape's `stage`. |
| 3 | **Selection asymmetry.** The end-to-end machine set of record keeps the best of up to three attempts per start; the human set keeps every attempt including failures. | `python3 -c "import json;print(json.load(open('.../matched_w3/dDPfull/manifest.json'))['one_per_ic_best'])"` — and see §6 below, this is now being corrected rather than only disclosed. |
| 4 | **`hold15` is not held out.** 14 of its 15 starts are training starts. | Any sentence saying "held-out" must name `rnd30`/`spots60`, or say "in-distribution". |
| 5 | **Hardware class changes end-to-end results.** Cells reproduce bit-exactly within a CPU class and not across it. | Every cell stamps its node set; `harvest_nodes.py` reads `/proc/cpuinfo`, never Slurm's advertised features, which are wrong on this cluster. |
| 6 | **A silently substituted start.** The old evaluator replaced an entry that failed to restore instead of counting a failure. | Cells of record report zero restore failures because they could not report one; the fixed evaluator reports them. Check the cell's `restore_failed`. |
| 7 | **Bank entries drawn with replacement**, so a "148-entry" bank was not 148 distinct starts. | Check the bank's `bank_version`; anything without one predates the fix. |
| 8 | **Truncated episodes logged as all-zero**, which made flag-based learning curves measure termination rather than stages. | End-to-end curves are rebuilt from `episode/score`; see `curves/learning_curves.py`. |
| 9 | **A seed set that is not seeds.** One DP cell's "15 seeds" were 10 policies double-counted. | The harvester now asserts on collisions; keep the assertion, do not deduplicate silently. |
| 10 | **A convenient label trusted over the artefact that recorded the fact.** The root cause of two separate wrong conclusions (Slurm node features; training-node attribution of evaluation cells). | When a claim rests on a label, find the artefact that measured it. |

## 5. What each class of result can support — defensible and indefensible phrasing

The single most common fidelity failure in writing about this project is **reporting a null as an equivalence**.

| the result is | you may write | you may not write |
|---|---|---|
| a null whose **MDE exceeds ±0.10** | "no effect of demonstration source was detectable at this sample size"; "the comparison could only have detected a difference of about `<MDE>`" | "the arms are equivalent"; "source does not matter"; "we found no difference" without the MDE |
| a null whose **MDE is below ±0.10** *and* whose `verdict` is `equivalent at +/-0.10` with `prior = stable` | "equivalent within the pre-registered ±0.10 margin" | the same sentence when the verdict says `PRIOR-SENSITIVE` or `borderline` |
| a **floor** (both arms near zero) | "neither arm learned this" | any comparison between the arms |
| a **difference detected** in an uncontrolled arm (e.g. the robomimic 200-tape draw) | "a difference on this data source, not yet controlled for quantity" | "machine demonstrations are worse" |
| a **learning-speed** effect | "one learner, exploratory, and flat on the isolated phases" | "human demonstrations train faster" as a general claim |
| anything from an **end-to-end** cell | the number, plus that stages are underpowered and the machine set was outcome-selected | a source interpretation, until (v) reads out |

Two standing wording corrections: say **"mode"**, not "deterministic", for world-model action selection (it is accurate
only for RLPD); and keep the note that the isolated-phase and end-to-end results point in **opposite directions** —
dropping it makes either one read as the project's finding.

## 6. What is in motion right now — do not audit a moving target

| in flight | what it will change | how to check it landed |
|---|---|---|
| **PHASE_PLAN (v)**, the de-confounded end-to-end machine arm: 4 seeds × {world model, RLPD, Diffusion Policy} trained on `dDPfull_first` (first attempt per start, no outcome selection) | **Every end-to-end human-vs-machine sentence.** Until it reads out, trap 3 stands and no end-to-end number supports a claim about demonstration *source* | `ssh pax 'squeue -u jstale02 -o "%i %j %T" \| grep v1st'`; then `cd $LAB/gp_e2e && python3 baselines/e2e_table_all.py --strat` |
| the last carrycontact and end-to-end re-score cells | rows marked `SUPERSEDED - re-score in flight` in `results.md` | the banner at the top of `results.md` clears itself |
| robomimic quantity controls | whether the robomimic source separation survives its own control | `../paper/ROBOMIMIC_LOG_2026-09-06.md` |

## 7. If you find something wrong

Say so plainly and name the artefact. This project's convention is that corrections are **commits with reasoning**, not
silent edits, and that a retraction is written where the original claim was made — including inside this directory
(`INDEPENDENT_REVIEW_2026-09-08.md` carries a response appended to the review itself). A finding that a number is wrong
is worth more than a finding that it is right, and nothing here is defended.
