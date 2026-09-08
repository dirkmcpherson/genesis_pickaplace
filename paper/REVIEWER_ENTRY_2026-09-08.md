# Entry point for an independent reviewer (2026-09-08)

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
- **§2.y place (0.703 v 0.647) was scored on the superseded entry bank** and may move; 144 cells are re-scoring now.
- **§3 contact and §4 carrycontact** — same bank problem, re-scoring in the same pass.
- **In-distribution Diffusion Policy 0.878** currently reproduces from 5 seeds, not the 10 behind the quoted figure; five are archived pending pinned re-runs.
- **Robomimic Diffusion Policy (0.863 v 0.095) and BC-RNN (0.927 v 0.393)** rest on a 200-tape machine draw whose RLPD equivalent has already been overturned by its own control. Treat both as uncontrolled until run on the larger machine arms.

## 4. Where I think we are most likely to be wrong — attack these first

1. **The hardware axis.** I revised this three times in one night: instruction set, then core count, then a two-class split whose mechanism is unidentifiable because the two candidate causes are perfectly collinear on this cluster (all 24 AVX2 machines have exactly 36 cores). The *operational* rule — hold class fixed — is safe under every account. The *causal* story is not established and should not be asserted.
2. **Selection asymmetry in the end-to-end sets.** The machine demonstration set is best-of-3 per start on 89 % of starts; the human set is every tape including failures. Σ demonstrated reward 206 v 118. This favours the machine arm by construction and is disclosed, not corrected.
3. **`hold15` is not held out** — 14 of its 15 starts are training starts. Anything called in-distribution should be checked for what it actually holds out.
4. **Post hoc statistic selection.** The sampled-versus-mode action statistic was chosen after seeing results (disclosed, both reported). Roughly 60 uncorrected tests exist; 3 at p < 0.05 is the null expectation, and the significant cells belong to the all-data arm.
5. **Two initial conditions are unwinnable by construction** (uids 234, 318 — cans lying on their side). Every n=74 denominator carries them.
6. **Human tapes are replayed on a clock that is not theirs** (a beat artefact between two 40 Hz sources; 2–7 % speed error per demonstration). Disclosed, not fixed.
7. **Neither world-model port has been validated on a reference task.** No standard-benchmark run exists in any log. "Working configuration" is claimed; "validated implementation" must not be.
8. **A silent data-loss mode has appeared twice** — completed evaluation cells discarded when a sweep parent is killed, presenting as absent data rather than an error. Both times recovered without recomputation. Assume it may have happened elsewhere and check `n_present == n_expected`.

## 5. Things that were already caught, as calibration

The reviewer should know the failure rate here is non-trivial and mostly self-caught: a comparability defect where world-model cells were scored on a different entry bank than every new cell, invisible because those cells carry no bank stamp; a contact predicate that scored carrying the can in rather than sliding it, in 84–86 % of policy grants; a slide success rule that passed 2 of 74 human demonstrations; a robomimic source claim overturned by its own registered control; and a training-proxy metric over-counting a published figure by 2.5×. The pattern to be suspicious of: **metrics whose name describes the intended behaviour rather than what they measure.**
