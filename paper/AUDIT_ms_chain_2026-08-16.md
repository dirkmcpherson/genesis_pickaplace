# ADVERSARIAL AUDIT: MS chain claims C1-C5 (2026-08-16)

Auditor: independent Fable session, read-only on code, no running process touched.
Standard applied: the 2026-07-08 panel standard (verify against primary artifacts;
name every overstatement). Style: Simplified Technical English where possible.

Verdicts at a glance:

| claim | verdict |
|---|---|
| C1 hover-incentive mechanism | **PART AFFIRMED / PART REFUTED** — real at gamma 0.99; does NOT vanish at gamma 0.8; the gamma attribution is wrong |
| C2 "ManiSkill's exact proven knobs" | **OVERSTATED** — 3 knobs match, ~9 load-bearing ones do not; the sparse-demo-in-dense-run defense is unsupported |
| C3 warmed-process finding | **REFUTED BY ITS OWN ARTIFACTS** — the fresh-process sweep shows 12/45 demo-IC picks, not 0/45; the mechanism story explains a discrepancy that does not exist |
| C4 swap-test equivalence | **AFFIRMED WITH A POWER CAVEAT** — numbers verified; a 0-vs-0 outcome cannot separate "both fine" from "both defective" |
| C5 record consistency | **ONE MAJOR FAILURE** (cell B row), rest verified or locally unverifiable |

---

## C3 — THE HEADLINE FINDING. The cell-B "warmed-process" claim is refuted by the primary artifacts. (Read this first; it flips a recorded verdict.)

The claim (handoff §29, RESULTS_MATRIX cell-B row, commit 3104df7): "0/45 demo-IC,
0/45 random-IC fresh-process; 5/10 sequential; therefore solver-state coupling."

**The named artifacts do not contain a 0/45 result.** I read every json and every
log in the session scratchpad `cellb_sweep/` (90 jsons + 90 logs, mtimes 08-15
23:3x-23:47, i.e. BEFORE §29 was written at ~00:30). Fresh single-episode
processes (one `wandb_eval.py --uids <u> --ic-mode demo --random 0 --reps 1`
process per episode, CPU) show:

| seed | fresh demo-IC picks | uids | fresh random-IC |
|---|---|---|---|
| s0 | 0/15 | — | 2/15 |
| s1 | **9/15** | 236,237,239,242,243,244,247,248,250 | 1/15 |
| s2 | **3/15** | 235,236,245 | 2/15 |

Spot-checked against raw stdout, not just jsons: `clean_d_1_236.log` line
"236 ep0: picked=True", same for 247, 250, s2 235, 245; picked videos exist
(`v_clean_d_1_236/sac:indist_0_picked.mp4`). These are the exact files the
verdict cites as its evidence.

Consequences, in order of severity:

1. **The pre-registered cell-B bar (cell_b_clean_demos §1: ">=2 of 3 seeds reach
   >=3/15 demo-IC" -> DATASET verdict) is MET: s1 9/15, s2 3/15.** By the
   registration, cell B resolves DATASET, not METHODS/TASK — the opposite of the
   recorded verdict. s1's 9/15 also exceeds every human-wave signature (5-7/45
   pooled); the matrix line "clean demos moved picks DOWN" is refuted.
2. The "warmed-process / solver-state coupling" mechanism has nothing to
   explain. Fresh-process picks are a SUPERSET of the sequential picks for s1
   (all five sequential uids pick fresh, plus 242 — which FAILED sequentially —
   plus 247/248/250, which the 10-episode sequential run never visited).
   Process warmth, if anything, removed a pick.
3. Downstream text built on the false null is now unsupported: "clean demos is
   evidence that demo narrowness has its own failure mode" (§29), the §27/§29
   "shared methods defect firmed" chain, and the RESULTS_MATRIX controls-table
   entry.

Probable cause of the error (hypothesis, stated as such): a metric-key mismatch
at aggregation time. Single-mode runs (`--ic-mode demo --random 0`) write
`eval/picked`; two-mode runs (the in-train eval, the seqrepro) write
`eval_indist/picked` + `eval_random/picked`. Every wandb-side helper in the
scratchpad (`explore_wandb.py`, `pull_matrix.py`, `wandb_watch.py`) reads
`eval_indist/picked`; applied to the sweep jsons that key is absent and a
`.get()` default of 0/None reads as zero for all 90 files — which reproduces
"0/45 demo-IC, 0/45 random-IC" exactly. No aggregator script for cellb_sweep
was preserved, so this cannot be proven, only noted as the simplest mechanism.

What in §29 DID verify:

* In-train evals: s1 5/10 (uids 236/237/239/243/244), s2 3/10, s0 0/10 —
  confirmed in `baselines/rl/checkpoints/rlpd_clean_s{k}/wandb_eval/eval.log`.
* Sequential repro: `scratchpad/seqrepro_s1.log` picks the same 5 uids, 5/10.
  Real replication (n=2, different `--seed`, demo-IC identical; the random-IC
  phase differed — expected, seed-driven, but it means "deterministic" is shown
  for the demo-IC phase only, from two runs, not from a repeated-run test).
* Checkpoint identity: I re-hashed the SB3 zips myself — all five `.pth`
  members of `rlpd_clean_s1/wandb_eval/snapshot.zip` and
  `rlpd_clean_s1/rlpd_100000_steps.zip` are md5-identical. The 28/28-tensors
  claim holds.
* Launch gates: `scratchpad/cellb/rlpd_clean_s1.log` shows 66 eps -> 9118
  transitions, 66 rewarded, explicit flags. Matches the cell-B build doc.

Answers to the specific attack questions:

* Other invocation differences besides process lifetime: (a) `--uids` +
  `--ic-mode demo --random 0` vs no-uids `--random 10`; (b) sweep exported
  `CUDA_VISIBLE_DEVICES=""` while the in-train subprocess inherits a GPU-visible
  env — this is a NON-difference because `wandb_eval.py` hardcodes
  `SAC.load(..., device='cpu')` and `GenesisCanEnv(backend='cpu')` (lines
  103-124); (c) sweep ran 5-way parallel (known load effect flips borderline
  demos) while the repro ran solo. None of these matter now: there is no
  discrepancy left to attribute.
* n=10 vs n=15: the 15-uid sweep set is a strict superset of the 10-uid
  sequential set; moot given the refutation.

**Required actions:** re-aggregate the sweep from the logs (one grep suffices),
correct handoff §29 + RESULTS_MATRIX + commit 3104df7's message claim, re-score
cell B against its §1 bars, and tell the user the cell-B verdict flipped. The
"minimal warm-up probe" held for user priority is pointless as framed.

---

## C1 — hover-incentive mechanism: right at gamma 0.99, wrong that it vanishes at gamma 0.8, and the gamma attribution is wrong.

Checked against `~/workspace/ManiSkill/mani_skill/envs/tasks/tabletop/pick_cube.py`
(compute_dense_reward), `baselines/rl/ms_env.py` (termination), SB3 buffers
(`handle_timeout_termination=True` default) and `rlpd_ref/swap_test/train_swap.py`
(mask=1.0 on TimeLimit.truncated). Facts:

* Dense reward (normalized = /5): reaching (0..1] + is_grasped {0,1} +
  place*is_grasped (0..1) + static*is_obj_placed (0..1); `reward[success]=5`
  where success = NATIVE (placed AND static). Positive everywhere (measured min
  0.0112 in the gate). Max non-success hover value: grasped, tcp at cube, just
  outside goal_thresh 0.025 m -> reaching ~0.93-1.0, grasp 1, place
  1-tanh(0.125)=0.876, static term 0 -> **r_hover ~ 0.56-0.58/step**.
* Our wrapper terminates on RELAXED success (grasped AND placed). At that cut
  the arm is mid-motion, so the native `=5` override usually does NOT apply:
  terminal reward ~0.58-0.8, at best 1.0.
* BOTH trainers bootstrap through truncation and cut at termination. So a
  hover policy's value is r_hover/(1-gamma); a completing policy's value is the
  one terminal payment.

The numbers:

| gamma | V(hover) | V(complete) | ratio |
|---|---|---|---|
| 0.99 (msdense) | ~57 | 0.6-1.0 | **~60-95x** |
| 0.8 (msknobs) | ~2.9 | 0.6-1.0 | **~3-5x** |

So: **at gamma 0.99 the perverse incentive is real and enormous — AFFIRMED.**
**At gamma 0.8 it shrinks ~20x but does NOT vanish — hover still out-earns
completion ~3-5x — REFUTED.** Break-even is gamma ~0.42 even granting the
full 1.0 terminal; no plausible discount rescues terminate-on-success dense
PickCube from this incentive.

The attribution error: ManiSkill's own SAC baseline is immune NOT because of
gamma 0.8 but because of its termination semantics. Its README PickCube command
leaves `partial_reset` at the default False, so `ManiSkillVectorEnv(...,
ignore_terminations=True)`: **episodes never terminate on success**; they run a
fixed 50 steps, and every post-success step pays the override 1.0/step > hover
0.575/step. Completion strictly dominates hovering at ANY gamma in their
config. (They also default `bootstrap_at_done='always'`.) Our wrapper keeps
terminate-on-relaxed in dense mode (ms_env.py step()), so msknobs carries the
incentive their baseline does not have.

Consistency check against data: ref-dense finished 0/3 with grasp rising to
0.2-0.6 and mean_len pinned at 100 (grasp-without-success — the observation the
hypothesis was built for). This is CONSISTENT with hover-farming but does not
prove it; a policy that grasps and simply fails to place looks identical in
these logs. obj_to_goal telemetry at eval would separate them.

Implication for msknobs (C2 overlap): if hover-farming is the binding failure,
msknobs is predicted to fail TOO (incentive persists at 0.8). A msknobs failure
therefore does NOT add evidence for "core broken", and a msknobs success would
refute the hover story. The run is a valid experiment but not the one the
gamma-vanishes framing says it is.

---

## C2 — "RLPD-ours-MSknobs runs at ManiSkill's exact proven knobs": OVERSTATED.

Their proven baseline (`ManiSkill/examples/baselines/sac/sac.py` + README
PickCube command) vs the live msknobs runs (sidecar
`rlpd_msknobs_s0/rlpd_ms.sidecar.json`, log `scratchpad/msknobs/s0.log`):

| knob | theirs | ours (msknobs) | match |
|---|---|---|---|
| gamma | 0.8 | 0.8 | YES |
| horizon | 50 | 50 | YES |
| reward | normalized_dense | normalized_dense (online) | YES |
| total steps | 500k ACROSS 32 envs (~15.6k/env) | 500k in ONE env | count only |
| UTD | 0.5 | 1 | **NO (2x)** |
| critics | 2, min-of-2, 3x256 | 10-ensemble, min-of-random-2-of-10, actor on mean-of-10, 2x256 | **NO** |
| demos | none | immutable 50%-of-every-batch, SPARSE-relabeled | **NO** |
| batch | 1024 | 256 (128+128) | NO |
| tau | 0.01 | 0.005 | NO |
| learning_starts | 4000 global | 1000 | NO |
| target_entropy | -4 (-dim) | -2 (-dim/2) | NO |
| termination | none on success (partial_reset=False -> ignore_terminations) | terminate on relaxed success | **NO — flips the reward structure (see C1)** |
| bootstrap | 'always' (even at termination) | SB3: cut at termination, bootstrap at truncation | NO |
| eval | success_once, 16 envs, fixed 50 steps, native metric | terminate-on-relaxed, n=10 | NO |

Three knobs match. "Exact proven knobs" is false labeling. Reading risk, both
directions: a FAILURE is over-determined (termination-incentive per C1, sparse
demo labels, single-env correlation, UTD/batch mismatches) and says little
about the core; a SUCCESS would show the core can learn dense PickCube but
would NOT establish "their proven config works in our code" because it is not
their config. Also: **no pre-registration document or bar exists for msknobs**
(nothing in paper/, nothing in the scratchpad msknobs dir; the run launched
08-16 08:04 after commit d24f109). Every neighbouring run has one. Deviation
from the project's own registration discipline — flag it before results exist,
which is now.

Sparse-demos-inside-dense-run sub-claim ("the mixture is exactly the March dv3
positive's mixture, so it is proven workable" — ms_dense_pair doc): the analogy
does not carry. In dv3 the demo tapes feed a LEARNED reward/world model and
dilute into a growing online buffer; in RLPD the demo half is 50% of every
critic minibatch forever, and ~3390 of 3440 demo frames carry r=0 at states
where the online dense stream pays ~0.1-0.6. The critic is trained on
systematically contradictory reward labels for demo-covered states — plausibly
repelling the policy from demonstrated trajectories, which is the opposite of
the demos' purpose. Not proven harmful here, but "proven workable" is
unsupported; the dense pair carries this confound in BOTH arms, so it cannot
detect it. A cheap ablation (dense pair with --demo-batch 0 or dense-relabeled
demos) would.

Minor log-hygiene bug found en route: `load_ms_demos` computes `n_over_horizon`
against the module constant HORIZON=100, not `--horizon`. Under msknobs
(horizon 50) the census prints "0 exceed the ... env horizon" while the median
cut tape is 69.5 steps — most demo tapes are LONGER than the online episode
horizon, and the census line hides it. Silent-default family; the fact itself
(demo terminals unreachable inside a 50-step online episode) is worth a line in
any msknobs writeup.

---

## C4 — swap-test equivalence: AFFIRMED with a stated power caveat.

Numbers verified in primary logs (`rlpd_ref/swap_test/checkpoints/
RLPD-ref-MS_s{0,1,2}/train.log`): success AND grasp 0.00 at all seven decision
points (0-300k), 3/3 seeds. "Fails identically" is accurate for sparse.

The gates do close the mechanical alternative-failure-mode gaps: census through
the reference loader is an exact match (50/3440/50/1.453488%); demo frames
reach reference batches at expectation — I re-derived it independently:
45k trained steps x UTD 20 x (128 demo x 1.4535%) = 1.674M rewarded-frame
draws by 50k, and the log reads 1,674,952. Action space asserted Box(-1,1);
tapes replay 3/3 through the adapter; negctl 0. The D1-D11 deviation list is
honest, and D9 (byte-identical +1 relabel) removes the most likely
"fails-for-its-own-reason" candidate.

The caveat the paper must keep: a 0.00-vs-0.00 comparison has no discriminating
power. A defective our-trainer and a correct our-trainer both produce 0/3 on a
task nothing has ever passed at this budget (§30's own reframe). So the swap
test proves "the reference does not rescue this setting", and only weakly
"our trainer is not the distinguishing defect". RESULTS_MATRIX already words
this correctly ("implementation equivalence on this setting, not soundness");
handoff §30's headline sentence is the strong version and should carry the
matrix's qualifier. The dense pair is the right next discriminator — with the
demo-label confound noted in C2.

---

## C5 — record consistency spot-checks (rows touched since 08-15).

| number | artifact | verdict |
|---|---|---|
| MS control 0/3, 0.00 at every 50k point, transient s0 0.10/0.10 @150k | `rlpd_msctl_s{0,1,2}/train.log` | **MATCHES** (note: s0 also grasp 0.10 @200k with success 0.00 — "single transient" is slightly generous; "grasp ~0 throughout" still fair) |
| MS control end diagnostics critic_loss ~5e-4 | s0 train.log tail: 0.00055 | MATCHES |
| swap test 0/3, seven decision points, grasp 0.00 | RLPD-ref-MS_s* train.log | **MATCHES** |
| cell B row: "0/45 demo-IC, 0/45 random-IC, 0/3 at bar" | cellb_sweep jsons+logs | **REFUTED** — actual 12/45 demo-IC (9+3+0), 5/45 random-IC; 2/3 seeds at the pre-registered bar (see C3) |
| cell B row: in-train s1 5/10, s2 3/10; same-5-uid repro; bit-identical ckpt | eval.log, seqrepro_s1.log, zip hashes | MATCHES |
| dense-pair launch record: 6 PIDs | live ps table | MATCHES (ours 2953049/52/54 live; ref dense trio completed 300k in-log; msknobs 3013822/3/4 live, logs in scratchpad/msknobs/) |
| dense-pair gates (a)-(c) values | gate_a_regression.py, dense_smoke_ours.log, census jsons | spot-checked census + smoke exist; not re-executed (read-only audit) |
| ignition table: RLPD nb/pair/hold/mref 1/3 each; r2d per-wave; dDP 0/20; msrecipe 3/3-0/3 | peer-box (newbox_supp) sweeps + cluster wandb | **UNVERIFIABLE from this box**; no contradiction found, but the cell-B failure shows sweep aggregation is a live error class — the four "1/3" wave sweeps used the same single-uid wandb_eval pattern and deserve one re-aggregation pass from raw logs before the paper cites them |
| ref-dense verdict (not yet in matrix) | RLPD-ref-MSdense_s*/train.log | 0/3 at 300k, grasp 0.3-0.6 rising, mean_len 100 — when written up, the pre-registered "both fail -> wrapper suspect" branch should cite C1's termination analysis |

---

## Summary of required corrections

1. **Cell B: re-aggregate, re-score, correct §29 + matrix + inform the user.**
   The recorded null is an artifact error; the pre-registered DATASET verdict
   fired. This also weakens "four-wave invariance + fifth shared-defect
   candidate" as currently written.
2. Retire or reword the "gamma 0.8 makes the incentive vanish" line; the
   correct statement is "ManiSkill's baseline never terminates on success, so
   its dense reward has no hover incentive at any gamma; ours does at every
   gamma above ~0.4."
3. Stop calling msknobs "their exact proven knobs"; write its deviation table
   into a registration note (it has none) before its first decision point.
4. Re-check the other waves' sweep aggregations from raw logs (same error
   class as cell B).
5. Keep the matrix's hedged wording for the swap test; do not upgrade it to
   "trainer absolved".

No running process was touched. No code file was modified.
