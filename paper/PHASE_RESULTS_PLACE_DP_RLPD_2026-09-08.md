# Place phase, Diffusion Policy and RLPD — results (SKELETON, 2026-09-08)

**NO NUMBERS ARE ENTERED BELOW.** Every cell is filled from tool output by
`python3 baselines/place_table_all.py` (learner × source, exact permutation tests, bank-provenance footer) once the 32
runs land; a cell that has not been computed prints `—` and must stay `—`. This file exists so the readout is a
fill-in, not a fresh piece of writing. Design and registration: `PHASE_PLAN_2026-09-04.md` amendment (h) and its
checkpoint-retention addendum. It becomes §2.z of `PHASE_RESULTS_2026-09-05.md` when complete.

## 1. What was run
Phase 2 = Place, repeated for the two learners the world model was compared against, so the paper has a learner × source
table at this phase. Entry = a banked pick-grant state; success = `placed_v2` (grip cmd < 0.45 ∧ can inside the shelf
footprint ∧ can-centre z in the world's shelf band 0.18–0.24 m ∧ tilt < 20°, sustained 10 frames); tips terminate with no
penalty (`phase_sparse`); horizon 600 sim steps (150 decisions); corrected world `gc_kp4_riser3_shelf6`.

| | human arm | machine arm |
|---|---|---|
| demonstrations | `dH_place`, 39 segments / 39 ICs / 4265 decision rows, sha `38f986be…` | `dDP_place_n39`, 39 segments / 39 ICs / 4727 rows, sha `5473ed08…` (amendment (e) matched subsample, seed 0, of the 63 one-per-IC machine segments) |
| RLPD | the r2dreamer-native segment rows verbatim (`place_demos.segment_transitions`) | same |
| DP | the same cuts of the full contract-v1 tapes, absolute joint targets, lerobot fps 7.5 | same |

Cross-check performed before training: `place_demos.py check` reproduced both segment sets row-for-row from the full
tapes (first/final state, `actions_delta`, one +1 per tape) — `CHECK-OK` for 39/39 and 39/39.

**Recipes (not tuned).** RLPD: recipe of record (UTD 10, E10/Z2, LN critics, γ 0.99, 50/50 demo batches, delta_joint
cap 0.025 / leash 0.125, action_repeat 4), 250k decisions = 1e6 sim steps, LAST checkpoint. DP: recipe of record
(state-only, 100k grad steps, batch 64), LAST (100k) checkpoint, executed hold-4 through the env's delta integrator.
Both arms of RLPD reset from the HUMAN pick-grant bank (64 entries) — disclosed asymmetry, see §4.

## 2. Result — FILL FROM `place_table_all.py`
| learner | arm | holdE sample | holdE mode | polE sample | polE mode |
|---|---|---|---|---|---|
| r2dreamer (of record, §2.y) | human 39 | — | — | — | — |
| r2dreamer (of record, §2.y) | machine 39 | — | — | — | — |
| RLPD | human 39 | — | — | — | — |
| RLPD | machine 39 | — | — | — | — |
| DP | human 39 | — | n/a | — | n/a |
| DP | machine 39 | — | n/a | — | n/a |

Per-seed counts and the exact two-sided permutation test per learner and cell are printed by the same script and are
pasted verbatim. Registered prediction: |Δ(human − machine)| < 0.10 on polE for both learners (P1); DP polE ≥ 0.5 (P2);
RLPD clears the learnability floor on both arms (P3).

## 3. Provenance stamped in every cell
Entry bank path, `bank_sha256`, `bank_used_sha256`, `bank_n_entries` and `bank_version`; the polE cells use the rebuilt
physical-grip bank (`bank_version=physgrip_2026-09-07`). `place_table_all.py` prints a bank-provenance footer and
refuses to let rows scored on different versions of one bank name be read as a single table. Node identity and physical
core count (`node.cores`) are stamped per episode: node class is a known divergence source in this project, so any
comparison can be checked for core homogeneity after the fact rather than re-litigated.

## 4. Asymmetries and caveats (disclosed before the numbers exist)
1. **Reset distribution.** Both RLPD arms reset from the human pick-grant bank, unlike the world-model machine arm
   (which used its own 179-entry bank, §6.2 of the WM results). This removes that asymmetry at the cost of putting the
   machine demonstrations slightly off their own reset distribution.
2. **Budget units differ by learner** (DP grad steps, RLPD decisions, WM sim steps) — a like-for-like "compute" claim is
   not available and is not made.
3. **DP trains on absolute joint targets** and is converted to the delta MDP only at evaluation (hold-4).
4. **Evaluation protocol differs from the WM cells in one respect, deliberately:** this evaluator runs every bank entry
   exactly once, in order, and counts a failed restore as a failure; the r2dreamer place evaluator drew entries with
   replacement and substituted failures (adversarial review 2026-09-07 S1-3). Restore-failure counts are reported per cell.
5. Bare `contact` is not used anywhere in this phase; the statistic is `placed_v2`.
