# Which cells have numbers, which are still coming (2026-09-07, offline build)

*Assembled from the documents of record while the cluster was unreachable, so every cell cites its source rather than being recomputed. Where a source doc and this table ever disagree, the source doc wins. Cells marked pending were pending as of 22:37 on 2026-09-07 and may have landed since.*

## The comparison, by phase and learner

Each cell is **human vs machine**, then Δ (human − machine), the exact two-sided permutation p, and n as seeds per arm. Every phase statistic is the mode-action cell on its registered bank.

| phase | Diffusion Policy | RLPD | world model (r2dreamer) |
|---|---|---|---|
| **Pick** (random starts) | 0.520 v 0.467, Δ +0.053, p 0.123, n 10 | 0.600 v 0.567, Δ +0.033, p 0.646, n 8 | 0.617 v 0.608, Δ +0.008, p 0.875, n 8 |
| **Pick** (`spots60`, in-training-distribution) | 0.878 v 0.873, Δ +0.005, p 0.845, n 10 | **0.869 v 0.865, Δ +0.004, p 0.873, n 8** | pending |
| **Place** (matched 39 demos) | pending — 32 runs queued | pending — same 32 runs | 0.703 v 0.647, Δ +0.056, p 0.227, n 8 |
| **Contact** (matched 11, sub-floor) | not run | not run | 0.593 v 0.602, Δ −0.009, p 0.841, n 8 |
| **Carrycontact** (matched 21) | not run | not run | 0.807 v 0.796, Δ +0.011, p 0.348, n 8 |
| **End-to-end**, picked | pending — 32 runs queued | pending — same 32 runs | 0.500 v 0.537, Δ −0.037, p 0.643, n 8 |
| **End-to-end**, contact | pending | pending | 0.379 v 0.338, Δ +0.042, p 0.639, n 8 |
| **Slide** | **held** | **held** | not run |

Sources: pick from `RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md` §2; place §2.y, contact §3, carrycontact §4, end-to-end §5.1 of `PHASE_RESULTS_2026-09-05.md`.

**A second world-model implementation now exists.** dv3 with state input and the same return clamp reads 0.700 vs 0.633 on the pick, but at two seeds per arm that is not testable; the four-seed completion was running when the link dropped. Its value is that the clamp diagnosis reproduces on an independent port, not the number itself.

**The in-training-distribution row is the strongest null we have.** On `spots60` both learners sit near 0.87 with differences of 0.004 and 0.005 — the arms are indistinguishable *at high performance*, which is a much better null than one taken near a floor where any method looks alike. It is also the set you asked for specifically. Note that the RLPD random-start figure has been revised from 0.517 to 0.567 for the machine arm by the completed re-evaluation, shrinking that gap from 0.083 to 0.033.

**Provisional, for a good reason.** The place agent found both of its comparisons were skewed by machine size — the human pick arm had a seed on a 36-core node and the Diffusion Policy arm had five seeds on 48-core, each against all-64-core machine arms. Rather than disclose the skew it archived those 16 cells and resubmitted them pinned to one size. The numbers above will be re-issued when those land, and are expected to move little given the audit's measured movement of at most 0.058. **Be precise about what the in-training-distribution Diffusion Policy figure rests on:** the 0.878 was computed on the pre-archive tree with 10 human seeds; five of those are now archived pending their pinned re-runs, so it currently reproduces from five. It is not wrong, but it is not yet the number of record.

**Pruning still matters more than source.** Trained on raw rather than pruned human data, Diffusion Policy drops to 0.688 — a deficit of 0.19 at p 0.000, which is far larger than any source difference anywhere in this project, and it now reproduces in-distribution as well as out.

## How to read this table honestly

**The three-learner comparison exists for exactly one phase.** Pick is the only row where all three learners have been run on the same question. Every other Genesis row is a world-model result with two empty columns, and those columns are what the queued runs fill. Until they land, "no source effect across learners" is a claim supported at the pick and nowhere else on this task.

**Every Genesis cell is a null, and nulls are only as good as their power.** The tightest is the world-model pick, where the per-seed spread is about two picks in thirty, so a true gap of 0.10 would have shown. The weakest is the RLPD pick, whose minimum detectable effect is 0.345 — it would have missed a very large effect. The end-to-end row sits near 0.21. Reporting these as "no difference" without the detectable-effect figure would overstate all of them.

**The end-to-end `nested` row is deliberately absent.** The published world-model figure for it is the training proxy, which over-counts by about two and a half times against the honest settled predicate, while the two queued learners report the honest one. Putting them in one row would compare two different quantities. The re-score that fixes this is prepared and pinned.

**Slide is held on a definition, not on compute.** Its success predicate required an open gripper at contact, which passes 2 of 74 human demonstrations because people push the can home with the fingers still partly closed. Both the predicate and the phase's entry gate are corrected; what remains is your decision on which human tapes the phase trains on.

## The independent generator (robomimic Can) — the source reading is DEAD as of 2026-09-08

The registered quantity control has read out at full seeds and it overturns the headline. RLPD, 8 seeds, 50 shared starts, last checkpoint, mode:

| machine arm | tapes | score | vs MH200 (0.455) |
|---|---|---|---|
| MG200s (the 200-tape subsample) | 200 | **0.147** | −0.308, p 0.008 |
| MG200s at 3× budget | 200 | 0.287 | −0.168, budget helps +0.140 (p 0.115, n.s.) |
| MG718s (all successes) | 718 | **0.475** | **+0.020, p 0.886 — indistinguishable** |
| MGall (all rollouts) | 3900 | **0.610** | **+0.155 — above the human arm** |

Within the machine arms the differences are large and significant: MGall over MG200s **+0.463 (p 0.0002)**, MG718s over MG200s **+0.328 (p 0.0033)**. **The registered "quantity/coverage, not source" rule fires.**

**What is now true:** the RLPD gap is a property of the 200-tape draw, not of machine provenance. At their natural size, machine demonstrations match or beat human ones on this task. **What does not follow:** that demonstration count alone explains it — see the open question below. The Diffusion Policy (0.863 v 0.095) and BC-RNN (0.927 v 0.393) separations rest on the *same* 200-tape draw and inherit the same doubt until they are run on the larger machine arms.

**The open question, and the experiment that settles it.** The 200-tape subsample was not a neutral draw: 89 % of its rows come from the last four SAC checkpoint blocks, so it is both small and narrow. Two readings survive, and they differ in what the paper can claim:
- *Sampling artefact*: that particular draw was pathological, and a neutral 200-tape draw would match the human arm.
- *Per-demonstration quality*: machine demonstrations carry less information each, so ~3.5× as many are needed to match — which would be a genuine source effect visible only at matched count.

**A random 200-tape draw from the 718 successes distinguishes these and costs about 8 GPU-hours.** Until it runs, the honest statement is: at matched tape count the human draw wins on this task; at natural size the machine data does not lose; and we do not yet know which of the two readings is right.

## What was in flight when the link dropped

Queued or running: the 32 place runs, the 32 end-to-end runs, the four-seed dv3 completion, the robomimic controls, the contact re-score, and the in-distribution evaluations. Held deliberately: the 32 slide runs, pending your decision, and the pinned end-to-end re-score, which was prepared but never launched.
