# ADVERSARIAL AUDIT — 2026-08-29 (22 h to blackout, ~3 weeks to deadline)

Reviewer stance: hostile. Read: NEXT_2026-08-28, PAPER_PLAN §1-3, PREREG A9-A15, AUDIT_results_2026-08-28 §0-1/§7,
LEARNER_HEALTH_2026-08-28, WM_CANDIDATES §3, SESSION_LOG tail (08-28 18:27 → 08-29 05:34), train_rlpd.py demo/watchdog,
rlpd_sac.py:308-325. No cluster access; nothing here is a measurement, everything is a hole or a test.

## 0. The one thing to internalise
The paper has ONE healthy learner (DP) and ONE finding with a p-value (DP rnd dH 0.547 vs dDP 0.487, p 0.041 unadjusted,
AUDIT_results §4). Everything else is either (a) a learner whose recipe changed mid-block and therefore has NO valid
numbers yet (RLPD: PREREG §2 "a fix restarts that learner's block"), or (b) a learner with no healthy recipe at all (both
WMs). Three launcher bugs + three silent pack deaths in 24 h means the operator's launch error rate is ~1 per 4 h of
attended work; the blackout removes the attendance. Plan accordingly: fewer, chained, self-checking jobs.

---
## 1. RLPD "human vs DP demos" divergence (γ 0.99: dH hold 14/14/14, dDP 10/13/14, 2/3 dDP seeds diverge)

**Strongest case against:** n=3 vs 3 on a BINARY event. Fisher exact for 0/3 vs 2/3 is p = 0.40. At γ 0.998, 69/74 runs
diverged REGARDLESS of source (LEARNER_HEALTH §1) — so γ 0.99 sits at the edge of stability and the source contrast is
being read off the knife-edge of a phase transition, where any nuisance difference between tape sets tips seeds over.
"Near-identical length/action statistics" is a marginal-distribution check; divergence is driven by tails and by
(s, a) pairs the actor cannot produce.

Artefact explanations NOT ruled out (in my order of likelihood):
1. **Action saturation / normalisation of DP tapes.** DP emits absolute window-end targets executed through the
   integrator (PREREG §2); the native loader sets `norm = None` and trusts the tape ("actions already in normalized
   delta space", train_rlpd.py:356). If the recorder writes the pre-clip command, dDP rows can have |a| > 1 (or sit
   exactly at ±1 far more often than human follower actions). Q(s, a_demo) at/beyond the tanh boundary is the classic
   critic-extrapolation blow-up. Nobody has printed the |a|≥0.999 fraction per source.
2. **Terminal bookkeeping on DP tapes.** Native tapes carry `terminated`/`truncated` (train_sacfd_full.py:534);
   the pre-08-28 +2 double grant hit "8/56 dR2D demos" — the dDP count is not quoted anywhere. Loader normalises to +1
   for jobs after 08-28 13:25 only; g99 jobs launched before that stamp trained on a max return of 2 with a watchdog
   threshold of 2.0 (= 1× max return, not 2×), so trip counts across the 13:25 boundary are not comparable. Also: a DP
   teacher picks FAST, so dDP tapes have far more `terminated=True` rows per row than dH (short tapes). Different
   terminal density = different bootstrap-free fraction of the demo half of every batch; that is a learning-dynamics
   difference, not a "source" difference in the paper's sense.
3. **Demo-state distribution vs online.** DP demos (hold ~0.9) cluster around one solution manifold; 50 % of every batch
   is those states with a 0.99 critic that must also fit random-exploration states. dH covers more of state space
   (regrasps). This is a legitimate mechanism AND a confound of the "source" claim — the paper must decide which.
4. **IC count / N mismatch.** dH = 51 executable of 66 (A1); dDP recorded 64/66 ICs (A5). Is the RLPD dDP set the
   N-matched subset (§3.2) or the 64? Not stated in the health doc or the log. If unmatched, the 50/50 demo half is
   drawn from a different number of distinct ICs.
5. **Watchdog itself:** warn-only (rlpd_sac.py:318-324), does NOT alter training — ruled out. But its threshold is
   wrong for DENSE arms: train_rlpd.py:274-276 rescales only for hold reward; potential shaping φ = −2‖eef−can‖ shifts
   Q by up to ~+1 at the start state, so a healthy dense critic can legitimately exceed 2.0. "Trip count" is therefore
   an invalid health criterion in the dense half of the g99 matrix (NEXT item 1, N18 criterion). Use max critic_loss
   and final≈selected only.
6. **delta_ref / action_repeat mismatch:** the loader asserts the tape stamp equals the run's args (train_rlpd.py:309-310)
   — ruled out unless the stamp lies.
7. **Node heterogeneity:** pax050 was slow enough to kill s30's final eval (LOG 00:10); CPU-bound sim + 18 h limit means
   slow nodes silently truncate the confirmation readout. Not a divergence cause, but it makes "final" missing per node.
8. **Seed pairing:** seed ids are shared across sources (§1), but the env/actor init shares the seed while the demo
   sampler `DemoData(..., seed=args.seed)` (train_rlpd.py:365) draws from different-sized buffers — same seed ≠ paired
   noise. Fine, but do not call the design "paired".

**ONE control that kills artefacts 1, 2, 6 at once (0 GPU-h, CPU partition, ~2 h):** open-loop REPLAY the dDP and dH
tapes through `FullTaskEnv` with the recorder's own executed-action path and diff the re-recorded tape against the
stored one (actions after clip, terminated/truncated flags, reward rows). The tape contract (PREREG §4) says
recorded-as-executed; if the diff is zero for both sources, the format family is dead. Add to the same script the
per-source census: |a| saturation fraction, rows per tape, terminated rows per 1000, n_double_grant, obs ranges. Ship
the script output to a fixed file so it can be read after the blackout.

**Is n=8/arm enough?** For the HOLD readout, no: dH is AT the ceiling (14/15 in 6/6 runs) so hold cannot resolve
anything beyond "dDP sometimes diverges", and A8 says hold is training ICs anyway. Pre-register NOW (before s33-37
read out) that the RLPD source statistic is (i) divergence rate (Fisher exact; at 0/8 vs 5/8 p≈0.013, at 1/8 vs 4/8
p≈0.28) and (ii) rnd 30-IC success of the LAST checkpoint, not the selected one. With 8/arm and the observed 0/3 vs
2/3 rates, the expected power for (i) is roughly 50 %. Budget n=10 if any GPU frees; nothing else in RLPD matters more.

---
## 2. The world-model programme

**Strongest case against continuing dv3:** the torch port's critic runaway is now reproduced with NO demos on the
simplest task after it had already learned (LOG 20:45), ×3 on the 5M baseline by 300k (LOG 02:25). That is a port
defect (WM_CANDIDATES §2: NM512/dreamerv3-torch predates the replay-critic/EMA-regulariser revisions), not a property
of the task. A recipe that "holds" only with a non-standard clamp is exactly what A15 forbids as a matrix arm. The std
arm (s20-22 @1M) is the only dv3 job with matrix eligibility; 3M std ≈ 64 h > the 2-day cap (LOG 03:40) and cannot be
resumed during the blackout unless a dependency chain is submitted NOW.

**Fallback timing:** the MoDem/DEMO3 adapter is 4-6 person-days of CODING, which needs no cluster. The blackout is
1.5 days of enforced no-cluster time. Starting the adapter after the blackout throws that time away. Start it now, in
parallel; the "gate" in NEXT ("do not wait for 3M") is already satisfied by the 5M baseline runaway and touchgoal
collapse. Decision cost of being wrong: zero (if dv3-std ignites, the adapter is an appendix).

**Minimal WM evidence the paper needs:** for (1a) one healthy recipe, dH vs dDP success-only, ≥4 seeds each, protocol
readouts (hold+rnd) of the LAST checkpoint alongside BEST, last ≈ best. For (1b) the same two arms + own fails. For
(2) sparse vs dense: r2dreamer sparse is an exploration null (0 picks / 7,700 episodes, LEARNER_HEALTH §2) — that IS
the sparse result if the sparse arm is by-the-book; do not spend more GPU trying to make sparse WM ignite. Total
minimum: 4 arms × 4 seeds × 3M steps ≈ 16 jobs (packed 3-4/GPU = 4-6 GPU-slots × 2 days). That fits after the blackout
only if the recipe is decided BEFORE it.

**Corrected-vs-old world for the WM: NOT load-bearing.** A11 already makes the old world primary for (1a)/(1b); A13's
flip rule requires the corrected-world WM gate to ignite AND then re-running the WM fails arms there — a week of the
cap. The s80-83 w3 gate reads out tonight anyway (let it finish, it is sunk); but under A15 it runs the CLAMPED recipe,
so even if it ignites it cannot be a matrix arm. Conclusion: A13 should be amended — the world question is a sim-fidelity
contribution with DP+RLPD replicates only; the WM arm lives in one world.

**H4 as written is untestable with any candidate.** PAPER_PLAN §2 H4 says WMs consume demos "as dynamics data"; the
audit shows r2dreamer roots imagination at demo states and backs a λ-return along demo tapes (AUDIT_results §1,
dreamer.py:479-484, :539-563), the official DV3 does the same (repval 0.3), and MoDem BC-initialises the actor. Rewrite
H4 to a mechanism-free prediction ("the WM's dH−dDP gap is smaller than DP's/RLPD's") or drop it.

---
## 3. Matrix design (A12: {dH,dDP} × {succ, +own fails} × {DP,RLPD,r2dreamer,dv3})

- **DP × dense is meaningless** (no reward); **DP × +fails is forbidden** by the paper's own no-IL-on-failures rule
  (PAPER_PLAN §3). DP therefore has 2 cells, not 4. Check the 35 held DP jobs contain no fails/dense cell; if they do,
  cancel them — they would be a reviewer's first "you didn't understand your own design" comment.
- **Fails arms are not row-matched.** Fail share 0.30 is by TAPE; fail tapes are cap-truncated 301 rows vs ≤25 for
  successes, so fail ROWS were 70 % of the demo rows in the one fails arm measured (AUDIT_results §1). The A10 controls
  (dup13, DPsucc) exist only for the dropped dR2D lineage. For (1b) the matrix needs the same controls on dH/dDP or
  the result is "long tapes vs short tapes". Cheapest fix: build `dHsucc_dup` / `dDPsucc_dup` sets (duplicate successes
  to the fails arm's row count, 0 GPU) and run them as RLPD sparse ×4 (4 × 5 h = 20 GPU-h). This is the only new arm
  worth launching before the blackout.
- **Story-changing cells:** (a) dDP_RLPD+DPfails vs dH_RLPD+Hfails at γ 0.99 — queued; (b) the row-matched control above
  — NOT queued; (c) dH vs dDP under a healthy WM — no healthy WM. Everything else is replicate or diagnostic.
- **Cells that cannot change the story — cut them:** dv3 5M baselines ×3 (runaway confirmed by 300k, no A15
  eligibility); dv3 clamp/fix 1M and clamp 3M packs beyond ONE seed each (diagnostic only); tg_none_s1 (exploration
  null, teaches nothing); corrected-world RLPD fails top-ups s20-23 (A13 itself says RLPD sits at the floor there —
  a source contrast at 0.2 with 15 ICs has no power); corrected-world RLPD dense. That frees ≥ 12 GPU-slots × 18-48 h.
- **Dense relabel γ:** PREREG §2 table still says "RLPD 0.998"; dense demo rows are relabelled with `--gamma` at load
  (train_rlpd.py:311) so γ 0.998 and γ 0.99 dense runs have DIFFERENT demo rewards and cannot share a table.
  Amendment required.

---
## 4. Blackout risk

Capacity arithmetic: 38 RLPD × 18 h = 684 GPU-h; the cap is 10 GPUs → 68 h of the whole cluster. The 22 h + 36 h window
is 580 GPU-h. RLPD alone over-subscribes it before any WM job runs; with the held DP (35) and r2d (29) queues released
at any point, nothing finishes in order. Queue order must be set by job priority NOW, not by human hand later.

Must exist before the blackout starts:
1. **Dependency chains** for every job that outlives the 2-day cap: std3m/clamp3m packs → `sbatch --dependency=afterany:
   <jobid>` resubmit with DUPLICATE_OK (dreamer.py:407 re-enters latest.pt). Without it those packs die at 48 h and
   sit dead for the rest of the blackout.
2. **Self-retry for packs:** every pack gets an `--dependency=afternotok:<jobid>` clone (one retry). Three packs died in
   40 s unnoticed; a retry clone would have caught two of the three (the hydra-key one would fail twice — fine, it's
   logged).
3. **Scripted readouts:** an sbatch on the batch partition at `--begin=now+20h`, `+44h`, `+62h` that runs the
   squeue/grep loops from the LOG (RLPD headlines, dv3 metrics, r2d ckpt_scores, pack rc=) into
   `paper/harvest_<date>.md` on the cluster FS. Post-blackout the first read is one file, not 40 .out files.
4. **RLPD 12 h casualty:** job 2984556 refused the 18 h update (LOG 00:05) — it will die at 12 h; resubmit it under a
   fresh seed id per A9 now, or accept n−1.
5. **Pre-registration timestamps:** A16 (RLPD statistic = divergence rate + LAST-ckpt rnd), A17 (γ 0.99 restart of the
   RLPD block; all 0.998 numbers superseded), A18 (WM fallback = MoDem, actor-BC confound disclosed, H4 reworded),
   A19 (A13 amended: WM in one world) — committed BEFORE the s33-37 readouts are looked at.

---
## 5. Internal inconsistencies a reviewer rejects on sight
- **Best-of-K on a bistable learner.** r2dreamer BEST = max over 30 checkpoints of a process that flips between 0/15 and
  ≥10/15 (LEARNER_HEALTH §2); the "×3 confirmations" are the same 15 ICs (17/17 identical). Sel = training ICs (A8).
  This is checkpoint cherry-picking by construction. The LAST-ckpt re-score is "to be queued" — make it the headline
  and BEST the appendix, for every learner, or the WM row is unpublishable.
- **hold quoted as headline** in the 08-28/29 notes ("dH 14/14/14"). A8 forbids calling it held-out and uid 234 caps
  it at 14/15. Only rnd (30 novel ICs) is a generalisation number; quote rnd first.
- **Mixed worlds in one story:** the DP finding (p 0.041) is corrected-world n=10; the RLPD contrast is old-world; A13
  may flip the primary world tonight. Decide once, write A19, never mix in a table without a world column.
- **Watchdog census contaminated:** pre-08-28 max return was 2 (LEARNER_HEALTH §0) while the watchdog fired at 2.0
  (train_rlpd.py:267-276 comment says "2× max task return" — it was 1×). "Trips by 10-30k in nearly every run" partly
  counts healthy critics. critic_loss > 100 in 61/74 stands; the trip counts do not.
- **PREREG §10 disclosure "RLPD has no passing positive control"** is now stale in the good direction (dH γ 0.99 rnd
  20-26/30) but under a mid-block recipe change; say both.
- **PAPER_PLAN §1** still sells the ouroboros/noise-injection paper; §2 H4 is mechanistically false (see §2 above);
  §3 says N=66 while A1 says 51 executable. A reader of the plan and the PREREG gets two different papers.
- **A9 grey zone:** tg_clamp s2/s3 were wiped and resubmitted under the same ids (LOG 02:25). Defensible (no readout
  existed) but write the exception into A9 explicitly or a fresh audit will flag it.
- **Exposure/mechanism prose** ("1-3 %", "no recorded transition enters the WM critic") is still in PAPER_NOTES and
  NOTES_IN_PLAIN_ENGLISH per AUDIT_results §5 — the ranked change list items 1, 6, 9 are not marked done in NEXT.

---
## 6. 22-hour schedule (hour 0 = now)
| hours | action | GPU-h |
|---|---|---|
| 0-1 | `grep rc=` on every pack .out; sacct on every R/PD job; kill: dv3 5M baselines ×3, tg_none_s1, dv3 clamp/fix beyond one seed, corrected-world RLPD fails top-ups + dense (§3) | frees ~12 slots |
| 1-3 | submit `afterany` resume chains for std3m/clamp3m; `afternotok` retry clones for every pack; 3 scheduled readout jobs (+20/+44/+62 h) writing `paper/harvest_*.md` | 0 |
| 3-5 | CPU-partition tape replay-consistency + census script (§1 control); output to a fixed file | 0 |
| 5-7 | build row-matched `dHsucc_dup`/`dDPsucc_dup` sets; submit RLPD sparse ×4 each at the FRONT of the rlpd queue (hold the rest of the 38 behind them) | 40 |
| 7-9 | write A16-A19; commit; resubmit 2984556's seed under a fresh id | 5 |
| 9-12 | r2d w3 gate 3M readout (s80-83): record, apply A19 regardless of outcome; queue LAST-ckpt hold re-scores for every completed r2d run (CPU) | 0 |
| 12-16 | start the MoDem/DEMO3 adapter locally (env registry stub + npz→pkl converter); no cluster needed | 0 |
| 16-20 | dv3 std s20 ≈1M readout: value bounded AND any train success → keep std chain; else mark dv3-torch NEGATIVE in LEARNER_HEALTH and stop feeding it GPU | 0 |
| 20-22 | final squeue snapshot into the log; verify every running job has a dependency successor or is < 2 d; verify the 3 readout jobs are PD with correct `--begin`; commit | 0 |
Everything after hour 5 that needs a GPU should already be queued; the last 17 h are for chains, docs, and code.

## 7. Claims the paper can make TODAY
1. Methodology: one recorder / tape contract / one harness; run-identity rule; six silent-default bugs and how they
   were caught (this is the most defensible contribution in the repo).
2. DP (healthy, n=10, corrected world): DP-teacher demos ≈ human demos at matched N; hold flat (+0.013), rnd human edge
   +0.06 (CI [+0.006, +0.114], p 0.041, single unadjusted test). Old world n=5 replicate: no spread.
3. RLPD (as published, γ 0.99): human demos → stable critic 3/3, DP demos → 2/3 diverge with partial performance;
   n=3, a recipe-sensitivity observation, not a source effect. At γ 0.998 (the deviation) 69/74 runs diverge for both
   sources.
4. Adding 8 DP fail tapes: RLPD −73 %, r2dreamer −25-30 % (n 4 v 3, CI spans 0, min attainable p 0.057), confounded by
   demo share ×2.7 and tape length ×12.
5. World models: r2dreamer (non-standard return clamp) learns pick with dense reward + demos but is checkpoint-bistable;
   sparse is an exploration null (0/7,700 episodes). dreamerv3-torch: critic runaway past the attainable return ×3 with
   demos and once without — a negative result for that port under the standard recipe (pending std s20-22).
Nothing about (1a)/(1b) for RLPD or any WM is claimable yet; nothing about (2) beyond "sparse WM never ignites".
