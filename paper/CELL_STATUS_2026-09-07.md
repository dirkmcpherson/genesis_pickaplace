# Which cells have numbers, which are still coming (2026-09-07, offline build)

*Assembled from the documents of record while the cluster was unreachable, so every cell cites its source rather than being recomputed. Where a source doc and this table ever disagree, the source doc wins. Cells marked pending were pending as of 22:37 on 2026-09-07 and may have landed since.*

## The comparison, by phase and learner

Each cell is **human vs machine**, then Δ (human − machine), the exact two-sided permutation p, and n as seeds per arm. Every phase statistic is the mode-action cell on its registered bank.

| phase | Diffusion Policy | RLPD | world model (r2dreamer) |
|---|---|---|---|
| **Pick** (random starts) | 0.520 v 0.467, Δ +0.053, p 0.123, n 10 | 0.600 v 0.517, Δ +0.083, p 0.485, n 8 | 0.617 v 0.608, Δ +0.008, p 0.875, n 8 |
| **Place** (matched 39 demos) | pending — 32 runs queued | pending — same 32 runs | 0.703 v 0.647, Δ +0.056, p 0.227, n 8 |
| **Contact** (matched 11, sub-floor) | not run | not run | 0.593 v 0.602, Δ −0.009, p 0.841, n 8 |
| **Carrycontact** (matched 21) | not run | not run | 0.807 v 0.796, Δ +0.011, p 0.348, n 8 |
| **End-to-end**, picked | pending — 32 runs queued | pending — same 32 runs | 0.500 v 0.537, Δ −0.037, p 0.643, n 8 |
| **End-to-end**, contact | pending | pending | 0.379 v 0.338, Δ +0.042, p 0.639, n 8 |
| **Slide** | **held** | **held** | not run |

Sources: pick from `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §2; place §2.y, contact §3, carrycontact §4, end-to-end §5.1 of `PHASE_RESULTS_2026-09-05.md`.

**A second world-model implementation now exists.** dv3 with state input and the same return clamp reads 0.700 vs 0.633 on the pick, but at two seeds per arm that is not testable; the four-seed completion was running when the link dropped. Its value is that the clamp diagnosis reproduces on an independent port, not the number itself.

## How to read this table honestly

**The three-learner comparison exists for exactly one phase.** Pick is the only row where all three learners have been run on the same question. Every other Genesis row is a world-model result with two empty columns, and those columns are what the queued runs fill. Until they land, "no source effect across learners" is a claim supported at the pick and nowhere else on this task.

**Every Genesis cell is a null, and nulls are only as good as their power.** The tightest is the world-model pick, where the per-seed spread is about two picks in thirty, so a true gap of 0.10 would have shown. The weakest is the RLPD pick, whose minimum detectable effect is 0.345 — it would have missed a very large effect. The end-to-end row sits near 0.21. Reporting these as "no difference" without the detectable-effect figure would overstate all of them.

**The end-to-end `nested` row is deliberately absent.** The published world-model figure for it is the training proxy, which over-counts by about two and a half times against the honest settled predicate, while the two queued learners report the honest one. Putting them in one row would compare two different quantities. The re-score that fixes this is prepared and pinned.

**Slide is held on a definition, not on compute.** Its success predicate required an open gripper at contact, which passes 2 of 74 human demonstrations because people push the can home with the fingers still partly closed. Both the predicate and the phase's entry gate are corrected; what remains is your decision on which human tapes the phase trains on.

## The independent generator (robomimic Can)

This is a different task with a different machine source — policy rollouts from a reinforcement-learning agent rather than distilled demonstrations — and it behaves nothing like the Genesis task.

| learner | human (mixed-skill) | machine (200 rollouts) | Δ | p |
|---|---|---|---|---|
| BC-RNN | 0.927 | 0.393 | 0.53 | — |
| Diffusion Policy | 0.863 | 0.095 | 0.77 | 0.00016 |
| RLPD | 0.455 | 0.147 | 0.31 | 0.008 |

The ordering is the one the thesis predicts: the pure imitators lose most, the learner that interacts with the environment loses least. **Do not write that up yet.** The quantity control — the same learner on all 3,900 rollouts rather than the 200-tape subsample — reads 0.74 and 0.60 on its first two seeds, above the human arm. If that holds, these three gaps are a property of how the subsample was drawn and not of machine provenance, and the ordering means something else entirely. The control was registered before the result precisely so it could overturn it.

## What was in flight when the link dropped

Queued or running: the 32 place runs, the 32 end-to-end runs, the four-seed dv3 completion, the robomimic controls, the contact re-score, and the in-distribution evaluations. Held deliberately: the 32 slide runs, pending your decision, and the pinned end-to-end re-score, which was prepared but never launched.
