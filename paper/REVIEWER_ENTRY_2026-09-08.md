# Entry point for an independent reviewer (2026-09-08)

> **RESOLVED 2026-09-08 — the hardware account stands, and the arm asymmetry is a scheduling accident.** Two lanes disagreed on which machine produced each cell; the disagreement is settled by attribution method. Attributing every cell to the job that logged writing *that specific directory* gives **perfect separation by record hardware: 16 of 16 cells with a 36-core original moved, and 0 of 176 from every other class did** (32-core 0/8, 48-core 0/56, 64-core 0/108, 96-core 0/4). The movers' originals are two named 36-core Xeon E5-2695 v4 machines. The earlier "not one has a 36-core original" was exactly inverted — all of them do — and came from a first-matching-log heuristic rather than a per-directory attribution.
>
> **Why it looked arm-directional:** no machine-arm cell was *ever* evaluated on that class. The minority hardware happened to host only human-arm evaluations, so the arm correlation is scheduling, not bias. Re-scoring is **not** directionally biased and the running 256-cell pass is sound. The inflated "26 versus 0" count also conflated real movement with 49 cells where only a structurally-zero column became earnable under a separate fix — that one is not arm-directional at all (human 19, machine 30).
>
> **What remains true and still matters:** the published human arm is partly evaluated on a hardware class that produces different results, which is a genuine confound requiring the re-score, and end-to-end seeds 2 and 3 will legitimately move. Preemption and patch boundaries were both checked and excluded. This is the fifth revision of this question; it is offered with more confidence than the previous four because it explains *both* the separation and the apparent asymmetry, and because 16/16 against 0/176 is not a coincidence.
>
> **Independently reproduced 2026-09-08, and the bug is named.** The second lane rebuilt its join and reproduces 16/16 against 0/176 exactly, tracing the movers to the same two machines. The defect: cells were attributed to the node in the *training* run's event-file name, but the evaluations ran later as **separate CPU jobs**, so every post-hoc cell was credited to the wrong machine. Attribution is now taken from each job's own "wrote this path" log line joined to the scheduler's record. **The collinearity claim also stands** — the `/proc/cpuinfo` probe is authoritative over Slurm's advertised features, and it finds 24 AVX2 nodes, every one exactly 36 cores, with no 36-core node on the other instruction set. Both of that lane's errors had one root, worth generalising: **a convenient label was trusted over the artefact that actually recorded the fact.**

*Written for an agent or person auditing this project for robustness. It says where the claims live, what is currently in motion, and — most usefully — where I think we are most likely to be wrong. Adversarial reading is the point; nothing here is defended.*

## 1. How to watch what is happening

- **The running record is the git log** on branch `4dof-cartesian`. Every change carries its reasoning in the commit message, including every retraction. Corrections are legible as corrections rather than silent edits.
- **The claims of record**, in precedence order — later supersedes earlier where they conflict:
  1. `paper/REVIEW_GUIDE_2026-09-07.md` — results with power and threats. **Read §8 and every dated addendum first**; they overturn statements in the body.
  2. `paper/CELL_STATUS_2026-09-07.md` — the phase × learner matrix: what has a number, what is provisional, what is empty.
  3. `paper/CONFOUNDS.md` — the standing ledger. Check every claim against it.
  4. `paper/PHASE_RESULTS_2026-09-05.md`, `paper/RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` — primary results, **some now known stale** (see §3).
  5. `paper/OVERNIGHT_STATE_2026-09-07.md` — opens with a brief; the chronology below it contains superseded entries by design.
- **To ask questions or demand a re-run:** message the session `genesis-pickaplace-04`. Push back; do not assume a number is settled because it is written down.

## 2. The single most important methodological fact

**Almost every result here is a null**, and the registered predictions are equivalence claims of the form |Δ| < 0.10. A p-value cannot establish equivalence. Any null quoted without its minimum detectable effect is uninterpretable, and at least one — the RLPD pick, MDE 0.345 — is weak enough that it would have missed a very large effect. A Bayesian treatment reporting the posterior probability that Δ falls inside the ±0.10 region is being built in `HRI_results/` precisely because the frequentist column cannot carry the claim we want to make.

## 3. Numbers currently known to be stale or provisional

- **`PHASE_RESULTS` §5.1 (end-to-end) will move.** Those cells were produced on the minority hardware class; the re-score puts everything on the majority class. The only cross-class measurement shows the same cell differing on 24 of 30 episodes, contact shifting by 0.100.
- **§2.y place (0.703 v 0.647) will move, for TWO independent reasons**, and 144 cells are re-scoring now. (a) It was scored on the superseded entry bank, where 30 of 148 entries sat in a state with the fingers commanded more open than measured. (b) **When an entry failed to restore, the evaluator silently substituted a different start rather than counting a failure.** Every place cell of record reports 0 of 148 restore failures; the fixed evaluator reports 5. So the published place rates are inflated by up to five episodes per cell, independently of the bank. The two effects are being separated in the re-score rather than reported as one number.
- **§3 contact and §4 carrycontact** — same bank problem, re-scoring in the same pass.
- **In-distribution Diffusion Policy 0.878** currently reproduces from 5 seeds, not the 10 behind the quoted figure; five are archived pending pinned re-runs.
- **Robomimic Diffusion Policy (0.863 v 0.095) and BC-RNN (0.927 v 0.393)** rest on a 200-tape machine draw whose RLPD equivalent has already been overturned by its own control. Treat both as uncontrolled until run on the larger machine arms.

## 4. Where I think we are most likely to be wrong — attack these first

1. **The hardware axis.** I revised this three times in one night: instruction set, then core count, then a two-class split whose mechanism is unidentifiable because the two candidate causes are perfectly collinear on this cluster (all 24 AVX2 machines have exactly 36 cores). The *operational* rule — hold class fixed — is safe under every account. The *causal* story is not established and should not be asserted.
2. **Selection asymmetry in the end-to-end sets — NOW BEING CORRECTED (2026-09-08, PHASE_PLAN (v)).** The machine demonstration set is best-of-3 per start on 89 % of starts; the human set is every tape including failures. Σ demonstrated reward 206 v 118, completed demonstrations 16 v 3, and the best attempt differs from the first on **24 of 72** starts. It was disclosed but not corrected, and it is the reason no end-to-end number — the `nested_honest` gap and the ignition effect included — currently separates demonstration *source* from our own tape filtering. Selection cannot be matched upward (each human start has one recorded attempt), so the machine arm is being **de-selected** instead: a first-attempt-per-start set (Σ 131, completions 8, picked 63 — now *below* the human arm on picked and contact) is training on all three learners at 4 seeds, jobs `v1st_*`. Registered before the build, with the decisive reading fixed in advance: whether the human ignition advantage survives de-selection or was substantially our filtering. **Until it lands, treat every end-to-end source claim as confounded.**
3. **`hold15` is not held out** — 14 of its 15 starts are training starts. Anything called in-distribution should be checked for what it actually holds out.
4. **Post hoc statistic selection.** The sampled-versus-mode action statistic was chosen after seeing results (disclosed, both reported). Roughly 60 uncorrected tests exist; 3 at p < 0.05 is the null expectation, and the significant cells belong to the all-data arm.
5. **Two initial conditions are unwinnable by construction** (uids 234, 318 — cans lying on their side). Every n=74 denominator carries them.
6. **Human tapes are replayed on a clock that is not theirs** (a beat artefact between two 40 Hz sources; 2–7 % speed error per demonstration). Disclosed, not fixed.
7. **Neither world-model port has been validated on a reference task.** No standard-benchmark run exists in any log. "Working configuration" is claimed; "validated implementation" must not be.
8. **Silent substitution is the recurring defect class in this evaluator, now seen twice.** A failed entry restore used to be replaced by a different start instead of being counted (inflating every place rate of record); and separately, **a silent data-loss mode has appeared twice** — completed evaluation cells discarded when a sweep parent is killed, presenting as absent data rather than an error. Both times recovered without recomputation. Assume it may have happened elsewhere and check `n_present == n_expected`.

## 5. Things that were already caught, as calibration

The reviewer should know the failure rate here is non-trivial and mostly self-caught: a comparability defect where world-model cells were scored on a different entry bank than every new cell, invisible because those cells carry no bank stamp; a contact predicate that scored carrying the can in rather than sliding it, in 84–86 % of policy grants; a slide success rule that passed 2 of 74 human demonstrations; a robomimic source claim overturned by its own registered control; and a training-proxy metric over-counting a published figure by 2.5×. Two patterns to be suspicious of. **Metrics whose name describes the intended behaviour rather than what they measure** — that covers the contact predicate, the slide rule and the training proxy. And **a convenient label trusted over the artefact that recorded the fact** — that covers a node census taken from the scheduler's advertised CPU features (wrong on this cluster) and cells attributed to a training run's event file when the evaluation ran later as a separate job. Both label errors produced confident, coherent, wrong conclusions that survived review until someone joined to the primary record instead.


## 6. What the independent audit found — read before writing any sentence

An audit of every claim against its registration and its cells (2026-09-08) produced the artefacts in `HRI_results/`: a claims ledger keyed to sentences, a per-row verification guide, and a registration outcomes table. **35 registered predictions: 9 met, 11 FAILED, 6 not evaluable, 1 demoted, 3 withdrawn, 5 pending.** Our record is not clean and must not be described as such. Eight specific exposures:

1. **"All registered predictions met" was written in two documents and is false.** The margin fails on the in-distribution release predicate — 0.083 human against 0.242 machine, with the machine arm above the trigger that makes the prediction apply. Corrected at both sites.

2. **The registered equivalence procedure (TOST) has never been run — on any contrast, anywhere.** Every equivalence-flavoured sentence in this project currently rests on something other than the test registered for it. Either run it or stop using equivalence language.

3. **The paper's original headline hypothesis, H4, is recorded as FAILED**, with a directional human preference. The null that now stands was registered *after* that failure. **Presenting the current null as H4 confirmed would invert the record** — this is the single most dangerous sentence the paper could contain.

4. **The in-training-distribution amendment has numbers but no met/not-met verdict written anywhere**, and its world-model clause was never run.

5. **The slide statistic is recorded as "met" twice for a predicate the project withdrew.** It must never be counted among satisfied predictions.

6. **Registration-timing exposure, self-disclosed and easy to omit:** one amendment was registered after 3 of the 8 seeds it was scored on, another after 2, and a third after the full frozen comparison. These are disclosed in our own logs and must survive into the write-up.

7. **One registration is stamped 7 minutes after the job it governs was submitted.** Unexplained; the neighbouring gates in the same series are correctly ordered. Investigate or disclose.

8. **"World models are source-indifferent" is not supported as a general claim.** It rests on one task, and the independent-task replication produced a world model that scored 0 of 400 — it never learned the task at all.

**Two rules are enforced mechanically in the ledger rather than left to judgement**: a null may not be described as equivalence where the detectable effect exceeds the margin, and no lean may be attributed to demonstration source while the set-construction confound is unresolved. The first rule fired on the auditor's own entry, downgrading a claim it had marked established.
