# Morning report — 2026-08-11 (overnight autonomous shift)

## Headline
The delta-joint world-model arm (r2dreamer `pick_delta25d4_s0`) **genuinely learns
the pick** — train pick rate peaked at **0.81** (667 honest picks, mode ~20-28
agent steps, faster than demo replays) — but training is violently **bistable**:
a diagnosed value-explosion cycle destroys and rebuilds the policy every ~170k
steps, and every checkpoint written so far has landed in a destroyed phase. The
run continues (~1.3M/3M). One targeted, non-confounding fix is ready for your
go: **clamp lambda-return targets at the known max return** (see Mechanism).

## The night's eval-confirmed curve (15 eps, sampled actions, demo ICs)
| ckpt step | eval picked | notes |
|---|---|---|
| ~350k | 0.07 | first valid delta eval ever |
| ~500k | 0.13 | curve rising; tips fading |
| 772k / 972k / 1172k / 1275k | 0.00 | all four = collapse-phase checkpoints (see below) |

## Overnight discoveries (in order)
1. **Silent-default bug #7 (fixed)**: eval_genesis never passed action_mode/
   delta_cap -> ALL prior delta evals ran the policy in the WRONG MDP (absolute
   mode). Void: delta_s0 x2, d4 x1. After fix: the 0.07/0.13 curve above.
2. **delta_s0 record closed**: under correct semantics its checkpoint is 0/15
   (all timeouts) -> killing it was right on the true facts.
3. **Train metric certified honest**: 667 picked train episodes, all within the
   100-step env cap (no demo-episode contamination), lengths 11-100.
4. **Bistable collapse mechanism (the big one)**: 23 entropy-spike onsets, EVERY
   one preceded by lambda-return targets > the true max of 100 (observed 130-640
   in a gamma=.997 single-+100-terminal MDP) = bootstrap-through-terminal
   compounding via the cont head's small error at pick terminals. Amplitude
   GROWS with skill (more terminals/batch = more compounding): 0.56->0.03,
   0.81->0.00 swings. AMP grad-scale halvings (inf grads) accompany each spike.
5. **Checkpoint aliasing**: write cadence (~200k) vs oscillation period (~170k)
   -> all 4 archived checkpoints sampled dead phases (ckpt_1277432's write sits
   ON a spike onset). A/B probe (same weights, training-style vs eval-style
   resets, 0/10 both) rules out reset-path and harness explanations.
6. **Seed 1 NOT launched**: your gate was "if it continues to be successful" --
   evals never confirmed >=0.13 after the gate was armed (all archived ckpts
   were collapse-phase). Cloning a known-unstable recipe would replicate the
   pathology; seed the STABILIZED variant instead (your call, below).

## Recommended next actions (your morning decisions)
1. **Approve the return-target clamp** (clip lambda-return targets to
   reward_scale*1.0 = 100; optionally force discount=0 through is_terminal
   rows). One line in the return computation; numerically principled (true max
   is known); non-confounding (demos still consumed as data only). Then
   relaunch d4 recipe as s0/s1 -- likely rides the 0.81-level skill without the
   collapse cycle.
2. Cluster: launch the **SACfD -dj wave** (commands in last night's message /
   launcher defaults now delta+suffix). The 16 absolute zeros are the control.
3. Cluster r2dreamer v5 seeds via runbook (needs rsync incl. delta25 demos +
   d4 overrides: env.demo_duplicate=4 env.demo_reinject_every=150000).

## Cluster tallies (final)
- Hardened RETRAINS (absolute): **16/16 = 0.00** (n=8 per source; control row).
- Old-checkpoint re-evals: 15/16 = 0.00, one 0.07 (dH_s3). Fling narrative confirmed.
- dSACfD wave-2 (DOA teacher): all finished 0.00, closed.
- dv3 (absolute, both sources): flat 0 with sporadic 1/6 noise through 3.3M.
- DP rows unchanged (dDP_DP mean 0.80 / dH_DP 0.62 in-dist).

## Assets banked overnight
- Step-stamped checkpoint archive (ckpt_*.pt) + banked_peak.pt in the run dir.
- Full diagnostic ledger in r2dreamer/GENESIS_PORT_STATUS.md (bug #7, A/B
  probe, spike-onset table method).
- Grid videos + all evals in wandb `r2dreamer_genesis` (runs *-eval-step*).

---
## ADDENDUM (~06:45): the train-vs-eval contradiction, hunted to its last hypothesis

After the report above was written, later checkpoints kept evaling 0/15 even when
written at instantaneous train rates of 0.39-0.57. Systematic elimination
(all on identical weights, ckpt_974910 = 57% instantaneous train rate):

| hypothesis | test | verdict |
|---|---|---|
| eval-harness reset path | A/B probe, training-style vs eval-style resets | dead (0/10 + 0/10) |
| stale checkpoint save | weight diffs across checkpoints | dead (~4%/200k drift) |
| phase timing / metric lag | instantaneous windowed rates at write times | dead (0.57 at a dead ckpt; 0.038 at the 0.13-eval ckpt — inverted) |
| frozen-clone lag | frozen vs raw weights in ckpt | dead (identical) |
| warm RSSM state carry-over | obs_step is_first audit | dead (state zeroed correctly) |
| demo episodes in the metric | length histogram (81% of picks <=40 steps; shortest demo 55) + reinject timing (anti-correlated) | dead |
| CPU-vs-GPU numerics | GPU probe | dead (0/12) |
| **compiled-act path (cudagraphs)** | full 6-worker collection replica: compile=False -> **0/42**; compile=True -> RUNNING | **pending — the last one standing** |

If the compiled replica picks ~0.5: the "policy" that picks exists only inside
torch.compile's captured graph (an aliasing/staleness bug in r2dreamer's act
path) — the honest skill of the saved weights is ~0, train metrics were real
picks by a ghost policy no checkpoint contains, and the entire local d4 result
chain needs the compile bug fixed before any claim survives. If it does NOT
pick: the contradiction is unresolved and the trainer's live process holds
state no reproduction reaches — either way DO NOT cite d4 numbers beyond the
two early evals (0.07/0.13) until this closes.

Also confirmed overnight: reinject anti-correlation (picks vanish right after
750k/900k reinjections) — consistent with reinject-triggered value shock
feeding the lambda-return explosion (the clamp recommendation stands).

## FINAL (~07:20): compiled replica 0/42 — all eight hypotheses dead; live behavior VIDEO-VERIFIED real

The compiled-act replica also failed (0/42), and the checkpoint load is
byte-perfect (0 missing / 0 unexpected keys; frozen==raw). Meanwhile the live
trainer's own train_video (worker 0, step ~1.45M) SHOWS the arm approaching,
grasping, and carrying the can — the collection behavior is real, the metric is
honest, the predicate is honest. Cumulative extraction attempts: ~0/104
episodes across 8 controlled variants (harness, resets, mode, CPU/GPU, batch,
spawn workers, compile on/off).

**Bottom line: the picking policy exists only inside the live training process.
Something the state_dict does not carry (or an act-path heisenbug the replicas
don't trigger) is load-bearing. This is now a code-level reproducibility bug in
the r2dreamer port/upstream — not a physics, predicate, eval, or metric issue.**

### What this means for the paper, as of this morning
- Citable world-model numbers remain ONLY: d4 evals 0.07 (350k) and 0.13 (500k),
  and the instrumented-null absolute rows.
- The d4 run (still training, ~1.5M/3M) is a live specimen — I recommend NOT
  killing it until we decide whether to introspect it (py-spy / live-probe) —
  the ~10 step-stamped checkpoint archives are all banked regardless.
- The lambda-return explosion diagnosis + clamp recommendation stand unchanged.
- SACfD -dj cluster wave unaffected by any of this (SB3 stack, no shared code).

### Debug leads for the extraction bug (fresh-eyes list)
1. Diff the LIVE act outputs vs loaded-agent act outputs on identical obs
   (needs live introspection: py-spy dump, or a debug hook + restart).
2. Audit dreamer.py clone_and_freeze copy_ pattern vs torch.compile caching
   (line 176/196/258) -- the replicas construct FRESH agents; the live agent's
   frozen clones have been copy_'d thousands of times.
3. Check whether act() behavior depends on train-mode side effects (norm layers,
   dropout-like paths) -- replicas ran agent.eval(); the live trainer may act
   with modules in train() mode. **<- CHEAPEST: rerun replica WITHOUT
   agent.eval()** (not yet tried -- do this first).

---
# RESOLUTION (~05:15 actual): CHAMPION FOUND — 15/15 EVAL. No extraction bug.

**ckpt_1576820 (1.58M steps) evals PERFECT: picked 15/15, mean 33 agent steps,
sampled actions, demo ICs, hardened predicate.** Banked as
runs/pick_delta25d4_s0/CHAMPION_1576820.pt (+step-stamped copy). x3-rep
confirmation chain (2 more sampled seeds + mode eval) running; results land in
wandb r2dreamer_genesis.

**The night mystery resolves the boring way**: collapse onsets are <5k steps
wide; my +-15k instantaneous-rate windows mislabeled write moments; six
checkpoints genuinely landed on dead phases, the seventh landed on a peak. The
eval stack, save path, and metrics were healthy all along — the eight
"eliminated hypotheses" were all correctly eliminated, including the apparent
phenomenon itself. The addendum's extraction-bug framing is RETRACTED.

**Standing results for the paper (world-model arm):**
- r2dreamer + delta-joint (cap=demo-p99) + bounded actor samples + 4x demo
  density: eval curve 0.07 -> 0.13 -> **1.00** (15/15) at 1.58M steps. First
  fully-eval-confirmed world-model policy in the project.
- The lambda-return explosion + bistable oscillation remain REAL and diagnosed
  (targets 130-640 > max 100): the run flickers between champion-level and
  destroyed. The clamp fix would stabilize; the champion checkpoint already in
  hand reduces its urgency for the pick phase.
- Seed 1 launched ~01:10 (same recipe, verified config, seed=1), ~860k steps in.
- Random-IC eval of the champion still to run (queued after the x3 reps).

Next: dDP twin under the same recipe = the H4 comparison the paper wants.

## Confirmation protocol COMPLETE (~05:45)
CHAMPION_1576820 (r2dreamer, delta-joint, 1.58M steps), demo ICs, hardened
predicate, 15 eps/rep:
- sampled actions x3 seeds: **1.00 / 0.87 / 0.87 (mean 0.91, n=45)**
- mode actions: **1.00 (15/15)**
- zero timeouts in all 60 episodes; mean ~35 agent steps (~4.7 s)
- grid videos: wandb r2dreamer_genesis (runs 4-digit suffixes irr8okoj,
  975kzneb, ovrhyzry + the original discovery eval)
