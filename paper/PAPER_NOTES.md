# PAPER NOTES — findings and claims destined for the paper text

Started 2026-08-22. One entry per finding: the claim at the strength the evidence supports,
the evidence pointer, the caveats that must travel with it, and what would change it.
(PAPER_PLAN.md stays the coordination/decision log; RESULTS_MATRIX is the per-cell ledger;
this file is the prose-ready layer.) Notation: d{demo_source}_{algorithm}, e.g. dHpruned_DP.

## N1 (2026-08-22). Under RLPD the "demo-set effect" is about the FAIL tapes' relation to the training MDP, not human-vs-model origin

> **SUPERSEDED IN PART (2026-08-26).** The mechanism (missing env-terminal guard -> unanchored
> bootstrapping) is CONFIRMED and stands. The framing sentence — "that IS a human-vs-model
> demonstration property — of the failures" — does NOT: with terminals labelled, dDP ignites 4/4
> (UPDATE_2026-08-25 §3c), so the 0/6 catastrophe was an ENCODING BUG, not a property of model
> demos. Read N12/N12a for what failure tapes actually do, and N9/N10 for the source question.

**Claim (current strength: mechanism shown, causal test pending).** dDP_RLPD ignites 0/6 (max 1/15
demo-IC; pooled 0.02) while dH_RLPD 8/16 and dR2D_RLPD 10/16 (Fisher vs dH p=0.05, vs dR2D p=0.015).
The dDP stream drives RLPD's critic into overestimation from ~25k steps — before any online pick —
(median actor_q 32 vs 0.5/0.6 for dH/dR2D at 25k; 12,900 vs 1,090/8 at 100k; ent_coef 9.4 vs
0.88/0.02), after which the policy is entropy-saturated and never picks. Two dDP seeds whose
critics stayed sane (Q 5, 819) simply failed to ignite, the ordinary ~50% non-ignition.

**Why dDP specifically (measured on the tapes):** its 30 FAIL tapes are DP-teacher rollouts kept
whole at the 1200-step cap (51% of the 70k-transition buffer; dH no-pick share 33%; dR2D 0%). They
are off-manifold in kind, not just in share: 8/30 tip the can; ~10% of the whole buffer carries a
tipped can and 6% meets the env's strict tip-termination predicate (tilt>60° with grip open) at which
FullTaskEnv(pick) TERMINATES online (3 fails start from lying-can ICs) — states no success tape
and no online rollout visits; NN-distance of fail frames to any success frame p50 2.7 / p90 42 std
(human re-recorded fails: 0.9 / 4.8); 81% of dDP fail frames lie beyond the success set's 99th-pct
self-distance (human 43%; census tool numbers); longest post-tip chain = a full 1200-frame tape, ending done=False.
Mechanism: SB3-SAC's actor loss runs over the mixed batch, so the actor maximises Q at those
ungrounded states, its actions feed the targets there, and at gamma=0.998 a 1200-step chain with
no terminal anchor is a closed bootstrapping loop — the offline-RL extrapolation loop that RLPD
normally escapes because demo states lie where online data soon goes. The inflated values leak
into online states (actor_q is measured on online states). Human fails are near-misses close to
the success manifold (grounded); dR2D has no fails. The DP SUCCESS tapes show no destructive
signature (their only oddity is a 2x over-cap rate, a bounded bias shared by their fails).

**Framing for the paper:** the human set's fails are benign because humans fail like near-misses;
a BC teacher fails like a divergent policy. That IS a human-vs-model demonstration property — of
the failures, not of the successes — and it matters only for learners that consume fails (RL/WM);
BC rows train on successes only and are untouched.

**Caveats that travel with it:** n=6 (dDP) vs 16 (dH, dR2D); human comparison above uses the
re-recorded human set as a proxy (live dH set not on the analysis box); cluster-vs-local
same-machine rule; the dDP set is one DP teacher (ouro gen0 dp_joint, 19.7% harvest yield).

**Pre-registered tests (built 08-22; ARM=dDPsucc / dDPtiptrunc in cluster/sbatch_rlpd.sh,
sets from baselines/make_dDPsucc.py):**
- dDPsucc (63 success tapes, fails dropped) x6 seeds: PREDICTION — critic stays in the dH/dR2D
  regime (actor_q < 100 at 50k) and ignition >= dH's 0.5; if so the fail tapes are the cause.
- dDPtiptrunc (fails cut at the env's tip rule) x6: PREDICTION — intermediate; if it recovers
  most of dDPsucc's ignition the post-termination chains are the dominant channel, if not the
  DP failure states themselves (oscillation, can shoved 10-25 cm) are.
- Falsifier: dDPsucc also 0/6 with sane critics -> the success tapes (slow 512-step picks,
  over-cap 8%) are the problem and this note is wrong.
Evidence: paper/ROUND_ROBIN_RESULTS_2026-08-22.md (§ Why dDP_RLPD < dH_RLPD + follow-up);
census tool analysis/characterize_demo_sets.py.

## N2 (2026-08-22). Potential-shaped reward collapses the r2dreamer ignition lottery on human demos; raises RLPD ignition modestly; does not touch post-ignition collapse

> **SUPERSEDED (2026-08-26).** These numbers were measured under a shaping implementation that no
> longer exists: the r2dreamer adapter hard-coded gamma 0.999 against a config discount of 0.997,
> patched out on 08-24 (AUDIT_sources_2026-08-23 §5). The 4/4 dense result may have depended on the
> distance-proportional bonus the fix removed. Do not cite the ignition counts without that caveat.
dH_R2D dense 4/4 seeds ignite (best-ckpt confirmations 0.76-0.98, mode 1.0 x4; first training pick
209-430k env steps) vs sparse dH 8/34 (p=0.007; vs same-era wave 3 2/10, p=0.015); ceiling
unchanged (0.91-1.00 = champion). Two of four collapse by 3M (final-ckpt 0.00), one drifts, one
sustains 0.93 — the BEST-checkpoint protocol is what makes the cell readable. RLPD dense 4/6 vs
8/16 sparse (p=0.65), pooled 0.28 vs 0.22, best seed unchanged (9/15). dv3 dense 0/3 at eval.
Caveats: dense ran only on dH, so "does the source effect appear under dense reward" (ALGORITHM_
STATE §3.5) is still open; shaping is training-only, all headlines are sparse fresh-process pick;
the r2d result rests on BEST_selected.pt files on the cluster (shutdown ~08-24).

## N3 (2026-08-22). dR2D_DP is the best BC cell; model demos beat human in BC by another step

> **SUPERSEDED (2026-08-26).** "Model demos beat human in BC by another step" is the exact claim
> N6/N9/N11 attribute to a TRANSLATION ARTIFACT in how human demos were encoded. Recorded through
> one recorder the gap vanishes; and the readout used here (hold/in-dist) carries a structural
> 14/15 ceiling (N11). Superseded by N9 (source parity is BC-only, stated conditionally) and N11
> (BC claims move to `rnd`).
dR2D_DP in-dist 0.93/0.93/1.00 (0.96), random-IC 0.73/0.80/0.73 (0.76) vs dDP_DP 0.80/0.23 and
dHpruned_DP 0.62/0.23. Caveats: n=3; 66 champion tapes vs 63/8-seed arms; same-machine rule;
the action-density control dHallpruned_DP (all-zero-frame pruning) is still the registered
follow-up before "demo quality" is claimed over "idle-frame density".

## N4 (2026-08-22). dv3 on genesis: transient takeoffs, no confirmed ignition
First nonzero genesis evals ever: rr_dH_s2 5/6 at 320k (its last eval), rr_dR2D_s1 5/6 at 140k
(then 0 x4). Rollout videos confirm real approach-grasp-lift. MS-HEAD control 0.9 x2 stands.
Retire "dv3 never leaves 0 on genesis"; replace with "takeoffs at 140-320k not sustained inside
the 300k budget" — needs a fresh-process re-eval of the surviving checkpoint and a longer-budget
run with checkpoint archiving before any claim.

## Audit caveats attached 2026-08-22 (paper/AUDIT_impl_2026-08-22.md, paper/AUDIT_design_2026-08-22.md)
- N1: the mechanism is the encoder's missing env-terminal guard (confirmed in code; applies to all arms — dDP
  supplies the longest/most post-terminal chains). Third identification arm added: dR2Dfails (dR2D + DP fails;
  PREDICTION: ignition drops from 10/16-class toward 0 if fails are sufficient). dDPtiptrunc leaves one
  dangling bootstrap per cut tape (no done=True) — it under-tests; a `--demo-terminal-guard` encoder flag is
  the clean version. The "fails are benign for humans" half rests on a re-recorded proxy set until the live
  dH set is censused (analysis/characterize_demo_sets.py on the cluster).
- N1/N2 (WM rows): the r2dreamer/dv3 "dH" sets are the PRUNED SUCCESS-ONLY human set (67) while their dDP
  set carries the 30 DP fail tapes — the r2d human-vs-model contrast (8/34 vs 0/20) is confounded by the
  fail-tape variable; only dR2D_R2D (0/3, fail-free) is the clean model-demo WM point. Census the six WM
  dirs before writing that claim; a success-only dDP pixel set is the cheap fix.
- N2: r2dreamer shaping γ=0.999 vs a config comment "horizon 333 # discount 0.997" — verify; if mismatched,
  say "approximately policy-invariant". Checkpoint-scoring coverage may differ between the dense round and
  earlier sparse waves — lead with time-to-first-training-pick (coverage-independent) and rescore survivors.
- N3: DP evals run 1200 steps in one process; RLPD 400 fresh-process. State horizons per row; no
  cross-algorithm "best cell" language without it.
- Global: per REVIEW_GUIDE claim 4, "every implementation positively controlled" is false for RLPD — drop.

## N5 (2026-08-24). PINNED (user): the fails-arm question rides on the WM cells
P-MECH (corrected sets, sparse RLPD): dR2D+DPfails 0.55->0.15 on hold; dDP+fails == dDP
success-only (0.25). Mediator identified: 8 cap-truncated 300-decision fail tapes vs ~17-decision
dR2D successes -> fails = 72% of the demo buffer (71.6% by tape rows; audit 08-28) under per-transition sampling (vs 26% in the dDP (25.8%)
arm, no harm); no critic divergence (terminals labelled). PINNED per user: do not litigate
"fairness of fails" further on the RL side. The decisive cell is the WM arms on the SAME
fails-included sets (post-maintenance): if the WMs are flat-or-better where RLPD drops, the thesis
is learner-specific demo-property effects on one matched dataset. Registered follow-ups if needed
AFTER that readout: share-matched fails arm (subsample fail transitions to ~28%); per-episode
demo sampling as the practical RLPD fix. Fairness condition: identical fail tapes to every
fail-consuming learner; share arithmetic disclosed in any claim.

## N6 (2026-08-24, end of window). Corrected-world pick block + pilots
World = gc_kp4_riser3_shelf6 (sim tracks the real arm 15x better; adopted by user; shelf6 pending
the real shelf-height measurement). Sets matched_w3: dH_w2 58/66, dDP_w2 58 (teacher = new-world
DP pilot dH s0@100k, hold 14/15), identical IC multisets. Results (selected-on-hold means):
- DP: dH 0.90 (13,13,14,14,14 /15), dDP ~0.87 -- source parity holds; the corrected-world DP is
  the strongest policy of the program (rnd up to 19/30).
- RLPD: dH sparse 0.18 / dense 0.22 (one 10/15 seed); dDP sparse 0.20 / dense 0.08. WORSE than
  the old world (0.42/0.25 sparse) at the same budget: RLPD's recipe (gamma, caps, UTD, horizons)
  was tuned under old-world dynamics -- the R8 disclosure cuts both ways. Source parity again.
- r2dreamer corrected-world pilot: 0/2 seeds at 2e6 dense -- but conflated world + time_limit
  400->1200 + the gamma fix; disentangling run (new world, native time_limit 400) is first in the
  post-maintenance list. No new-world dR2D teacher yet.
Emerging paper shape: the apparent demo-source effect for BC was an artifact of how HUMAN demos
were translated into the learners' action space (re-encoding a 30 Hz absolute-command teleop tape
into decision-rate deltas); recorded through one pipeline, the source contrast vanishes for BC and
stays absent for RLPD. STATE IT CONDITIONALLY: DP is not source-agnostic by nature -- badly
translated human demos DO make it worse (0.62 vs 0.96, 08-19); the claim is that the effect is
attributable to translation, not to provenance. The contribution is the DIAGNOSIS, of which the
null is a consequence. What remains: LEARNER x WORLD/RECIPE effects (DP held 0.81->0.90 across a
world change; RLPD's tuned recipe lost half its ignition) and the pinned N5 fails question,
decided by the WM cells.

## N7 (2026-08-25, PRE-REGISTERED, not yet run). Naive action-density pruning should HURT dH_DP
The action-density control registered on 08-20 (N3), finally specified. Two corrections during
specification, the second from the user:
 (a) "prune all zeros" is a NO-OP on contract-v1 tapes (closed-loop follower -> ~0% exact zeros),
     so the rule must be a threshold on |a_arm|inf;
 (b) the rule must NOT ignore the gripper column. a_grip is an ABSOLUTE command in [-1,1], so
     |a_grip|~0 means "half open", not "no change": an arm-still decision during grasp CLOSURE
     would have been pruned, manufacturing the predicted result. Corrected rule: prune only if
     |a_arm|inf < eps AND |delta a_grip| <= grip_eps. |delta a_grip| is bimodal (p50 1.6e-5,
     p75 2.1e-3, p90 0.066) -> grip_eps = 5e-3 sits in the gap (noise pruned, closure protected).
Measured on matched_w3 (eps 1e-3, grip_eps 5e-3): 11.7% of dH decisions prunable (811/6927) vs
~0% of dDP; 51% of the prunable dH decisions hold a CLOSED grip (vs 31% of all decisions); idle
runs median 2 decisions, p90 10, max 33 (4.4 s). So most genuine idle is the hold that seats the
grasp -- which the hardened pick predicate requires.
PREDICTIONS (BC only; deleting decisions breaks the (s,a,s') chain -- invalid for RLPD/WM):
  (1) dHallpruned_DP hold < dH_DP (0.90): direction predicted, effect >= 0.05;
  (2) dose-response: larger eps (more removed) monotonically worse;
  (3) dDPallpruned_DP == dDP_DP within seed noise (null control: the RULE is harmless; the
      REMOVED CONTENT is what matters -- the same rule removes ~nothing from a model teacher);
  (4) failure mode is picks, not placement -- the removed frames are grasp-seating holds.
FALSIFIER: dHallpruned >= dH would mean idle density was a handicap and the leading-idle rule was
too conservative -- reviving "density" as the explanation for the 08-19 dR2D_DP > dH_DP gap that
N6 attributes to translation. Either way this cell must run before the paper claims "demo quality"
over "idle-frame density". Cost: 4 variants x 3 seeds = 12 DP jobs (~3 h each).

## N8 (2026-08-25, PRE-REGISTERED, not yet run). Does the WM arm need action_repeat 4?
USER CHALLENGE: repeat 4 was adopted because 1200-decision episodes "broke credit assignment", but
a world model trains on fixed-length subsequences (16x64) -- its DYNAMICS learning is indifferent
to episode length, so the WM should still learn what it needs. That is correct as stated; the
assistant's original framing was loose. What actually degrades at repeat 1, with gamma held fixed:
  - decisions-to-pick (dH median) 113 -> ~452; discount weight at episode start 0.997^113 = 0.70
    -> 0.997^452 = 0.24 (terminal reward mostly discounted away before value reaches the start);
  - rewarded frames per 1024-frame batch ~8.8 -> ~2.2 (the density lever METHODOLOGY §6.5 calls
    load-bearing for this arm).
Both are properties of the CRITIC's credit propagation, not of the model, and both are retunable.
DESIGN (dv3, dH only, 2 seeds each, 250k decisions = update-matched to a 1M-step repeat-4 run):
  A repeat 4, gamma 0.997      (current recipe, baseline)
  B repeat 1, gamma 0.99925    (= 0.997^(1/4): discount horizon matched in PHYSICAL TIME)
  C repeat 1, gamma 0.997      (unmatched: separates clock from horizon)
CAVEAT THAT SHAPES THE READOUT: dv3 has NEVER confirmed ignition (N4), so binary ignite/not may be
0/0/0 and uninformative. PRIMARY readout is therefore the graded diagnostic from the new tool
analysis/dv3_interrogate.py: reward-head error on demo frames, value-head profile vs
decisions-to-terminal, `value_reach` (largest decisions-to-terminal at which value > 10% of its
terminal value) REPORTED IN SIM STEPS so clocks are comparable, imagined-return decomposition, and
observed rewarded-frames-per-batch.
PREDICTIONS: (1) value_reach in SIM STEPS is comparable for A and B and shorter for C -- i.e. the
horizon, not the temporal resolution, is what governs propagation; (2) B nonetheless trails A on
any ignition/pick metric, because rewarded frames per batch are ~4x lower; (3) if (1) and (2) both
hold, the barrier at native resolution is REWARD DENSITY, which is cheaply fixable (demo
duplication / reinjection at repeat 1) -- register that as the follow-up arm rather than concluding
"WMs need repeat 4". FALSIFIER for the whole premise: B matches or beats A, in which case the WM
arms should move to native 30 Hz and the human demos stop being windowed at all (the design the
user prefers on validity grounds).

## N9 (2026-08-25, CORRECTION to N6). "Source parity" is a BC result, NOT an RLPD result
User challenge on the P-MECH table: it shows dR2D 0.55 vs dDP 0.25 -- i.e. source DOES matter --
which contradicts how the assistant had been summarizing. Re-derived from artifacts
(analysis/results_table.py; the first hand re-derivation was itself wrong, having swept in the
3k-step seed-0 SMOKE runs as real seeds -- hence the tool):
  RLPD old world SPARSE:    dR2D 0.55 > dH 0.42 > dDP 0.25   <- a real spread
  RLPD old world DENSE:     dR2D 0.18 ~ dH 0.15 ~ dDP 0.17   <- no spread
  RLPD corrected SPARSE:    dH 0.18 ~ dDP 0.20               <- no spread (both near floor)
  DP corrected:             dH 0.91 ~ dDP 0.85               <- parity, high absolute
CORRECT STATEMENT: BC shows source parity at high performance (n=5/cell, corrected world). RLPD
shows a source spread in exactly one condition (old world, sparse), n=4 with per-seed values
0.13-0.93, which vanishes under dense reward and cannot be tested in the corrected world because
every cell sits near the floor. The spread is ALSO PARTLY confounded with the same mediator as the
fails result -- but only for dR2D: its episodes are ~17 decisions vs ~115 for dH/dDP, so its reward
density is ~7x higher (5.5% vs 0.80% of rows rewarded). The dH-vs-dDP half of the spread is NOT
density-confounded: those sets are N-matched (56/56), IC-matched, and length-matched (median 115.0
vs 114.5 decisions). See N10.
CONSEQUENCE: do not write "demo source does not matter" anywhere. The defensible claims are
(a) BC: no source effect once translation is matched (N6, conditionally stated);
(b) RLPD: an apparent source effect exists but is confounded with reward density and is
    condition-fragile -- separating them needs a length/density-matched arm, which is the SAME
    control N5 already registered for the fails question. Register it once, use it for both.
PROCESS NOTE: results are henceforth quoted only from analysis/results_table.py output
(paper/RESULTS_TABLE_2026-08-25.md), never from recollection.

## N10 (2026-08-25, OPEN QUESTION -- no measurement yet). Why did the RLPD source spread vanish in the corrected world?
Nothing in N6/N9 explains this; N9 said "both near floor" parenthetically and that is an assertion,
not a measurement. The fact needing explanation, from paper/RESULTS_TABLE_2026-08-25.md:
  old world, sparse:        dH 0.42  vs dDP 0.25   (gap 0.17, favouring HUMAN demos)
  corrected world, sparse:  dH 0.18  vs dDP 0.20   (gap gone; every cell near floor)
This contrast is NOT explained by the density mediator: the two sets are N-matched (56/56 old,
58/58 corrected), IC-matched, and length-matched (old 115.0 vs 114.5 decisions p50; corrected 109.5
vs 119.5). So the old-world gap is close to a clean source effect -- and it favours human demos,
which is the OPPOSITE of the 08-19 BC story and of H4's framing.
CANDIDATE EXPLANATIONS, untested:
 (1) FLOOR COMPRESSION. Corrected-world per-seed maxima are 0.33; old-world dH had seeds at 0.73
     and 0.53. A 0.17 gap may be unresolvable once the achievable range collapses. DIRECT TEST:
     the registered seed top-ups (n=4 -> 10, revised plan R2). If the gap is real but compressed,
     more seeds should recover a smaller version of it.
 (2) THE COMPARISONS USE DIFFERENT DATASETS. dH_w2 was recorded with the `arrival=either` rule
     (58 tapes, incl. 7 demos the old rule dropped -- the dwell-stall hard cases), and dDP_w2 comes
     from a DIFFERENT teacher retrained in the corrected world. Both arms changed identity when the
     world changed. STRUCTURALLY HARD TO REMOVE: changing the world necessarily changes the demos
     (old-world tapes cannot be fed to a corrected-world env -- the sim_variant gate refuses, and
     correctly so). This is a stated limitation, not a fixable confound.
 (3) RECIPE MISTUNING (N6): RLPD's gamma/caps/UTD/horizons were tuned under old-world dynamics, so
     every corrected-world cell may be noise-dominated. TEST: the single-knob retune probe.
UNTIL ONE OF THESE IS MEASURED, the paper must not present either the old-world gap or its
disappearance as a finding -- both are n=4 observations in an unexplained regime change.

## N11 (2026-08-25). The BC "source parity" claim rests on a CEILINGED readout; use `rnd`
User question: did the corrected world introduce a DP performance ceiling? NO -- the cap predates
it. Across 22 DP runs (both worlds, three sources) no run has EVER exceeded 14/15 on `sel` or
`hold`. Verified from per-episode sweep.json records:
  - `sel` ceiling is STRUCTURAL: index 1 = uid 234, a lying-can IC where the tip rule fires at
    decision 1 -- picked 0 times in 430 policy-evals. 15/15 is unreachable by construction.
  - `hold` has NO structural wall (every one of its 15 ICs is picked by some policy) but no policy
    has swept it; DP at 13.6/15 sits at ~97% of anything ever achieved -> ~1 count of headroom.
CONSEQUENCE: on `hold` the dH-vs-dDP ordering FLIPS between worlds (old dDP 13.4 > dH 12.2;
corrected dH 13.6 > dDP 12.8) -- the signature of ceiling-compressed noise, not a source effect.
On `rnd` (30 ICs, observed 14.8-18.2, real headroom) the ordering is small but CONSISTENT:
  old:       dR2D 0.61 > dH 0.57 > dDP 0.55
  corrected: dH 0.55 > dDP 0.49            (no dR2D arm exists in the corrected world yet)
i.e. dDP is lowest in BOTH worlds and dR2D best where it exists (n=5/cell, gaps 0.02-0.06).
RULES ADOPTED: (a) BC source claims are made on `rnd`, with `hold`/`sel` reported as
ceiling-limited; (b) any "parity" statement must name the readout and its headroom; (c) this is
the SAME defect flagged for N7 (whose falsifier was untestable on a ceilinged hold) -- N7's primary
readout is already `rnd`. (d) The 14/15 sel cap should be stated once in the methods: one of the 15
selection ICs is unachievable under the tip rule, so 0.93 is the effective maximum.

## N12 (2026-08-26) — **OVERTURNED 2026-08-27 by its own re-score. See N15.**

> The numbers below were measured on `sel`, which (a) is not the readout the RLPD rows use, and
> (b) carries a structural 14/15 ceiling (N11). Re-scored on `hold`/`rnd` per PREREG §5, the
> "unmoved" finding does not survive: the world model loses 30% relative. Read N15 instead; this
> note is retained only as the record of the error.

### (original N12 text, superseded) The WM is unmoved by the fail tapes that broke RLPD
The pinned N5 decision cell (PAPER_NOTES N5, launched per the revised plan) has reported. Same
tapes, same world (old-world matched_v2), two learners:

  learner              dR2D (success-only)        dR2D + the 8 DP fail tapes     effect
  RLPD  (sparse)       0.55  (8,14,4,7 /15)       0.15  (3,1,5,0 /15)            -0.40, -73%
  r2dreamer (dense)    0.82  (0.93,0.60,0.93)     0.78  (0.80,0.73,0.80)         -0.04, within noise

n=3/4 per WM arm (dR2D s81 and dR2DDPfails s82 still running); RLPD n=4. WM numbers are
best-checkpoint evals at time_limit 400, the horizon the launch critique insisted on -- and it
matters: r2dreamer ignited 6/6 across both arms here, vs 8/34 sparse and 4/4 dense historically.
The pre-registered read-gate (success-only arm must ignite in >=2/4) passed 3/3.
CLAIM THIS SUPPORTS: demonstration-set defects are LEARNER-SPECIFIC. The same 8 no-pick tapes are
near-fatal to an off-policy actor-critic that trains its actor and critic directly on demo
transitions, and are ~free to a world model that consumes them as dynamics data and trains its
actor in imagination.
CAVEAT THAT MUST TRAVEL WITH IT (pre-registered, not discovered after the fact): the two learners
do not see the fail tapes equally. Demos are ~3% of r2dreamer's 450k replay ring once it fills,
vs ~36% of every RLPD gradient batch (demo_batch 128 of 256) -- a ~17x difference in exposure to
the mediator. Adding the fails also cuts dR2D's reward density 3.4x (5.5% -> 1.6% of rows). So the
null is "a WM at 3% demo exposure is unharmed", NOT "world models are intrinsically robust". The
registered disambiguation is the exposure-matched arm (BUFFER_MAX=40000 -> demos ~34% of the ring)
and/or the share-matched fails arm (subsample fail transitions to ~26%); N5 already registered the
latter. Until one runs, report the exposure arithmetic in the same breath as the result.
ALSO NOTED: dR2D s83's best checkpoint is at 421k steps while every other arm peaked at 2.4-2.9M --
the bistability the BEST-checkpoint protocol exists to catch (N2).

## N13 (2026-08-26, LAUNCHED). The human-vs-model WM comparison has never actually been run
User framing check: N12 is NOT a human-vs-model result -- both its arms are model demos (dR2D with
and without 8 DP FAIL tapes). The belief that "human demos help world models" traces to 08-19
(WM dH 8/34 ignited vs WM dDP 0/20), which was confounded: the WM dH set was pruned success-only
(67) while the WM dDP set carried 30 fail tapes. N12 makes that WORSE, not better -- if fail tapes
are free to a world model, they cannot explain the 0/20 either. That old result is now UNEXPLAINED,
and must not be cited as evidence for a source effect in world models.
State of the human-vs-model question by learner, before this run:
  RLPD (old, sparse): dH 0.42 > dDP 0.25  -- human better, n=4, unexplained (N10)
  DP / BC:            dH ~ dDP, dDP lower on rnd -- underpowered at n=5 (N11)
  World model:        NO CLEAN COMPARISON EXISTS
LAUNCHED to close it: r2dreamer, dense, matched_v2 (N-matched 56, IC-matched, success-only, same
tapes the other learners used), TIME_LIMIT=400, arms dH / dDP / dR2D x seeds 100-103 = 12 jobs.
This is the first time the three sources are compared on a world model with matched sets and no
fail-tape confound. PREDICTION (registered before results): given N12 (fail tapes free to the WM)
and N11 (BC source effects vanish once translation is matched), the WM source spread should be
SMALL -- if instead dH >> dDP reappears, the 08-19 result was real and something other than fail
tapes drives it, which would be a genuine finding and would need its own mechanism.

## N12a (2026-08-26). **SUPERSEDED 2026-08-28** — its seed sets no longer exist (fails s83 destroyed) and its corrected critical value is still wrong (t(0.975, df 2.16) = 4.012, not 4.303 → CI [-0.410, +0.499]). Kept for the record.
## N12a (original). Power on the N12 null -- it is thinner than the headline suggests
**CORRECTED 2026-08-26 (assistant's own error, caught on re-check).** The first version of this note
used t=2.776 (the df=4 critical value) when Welch's df here is 2.16, which requires t=4.303. The
CORRECT interval is:
  dR2D 0.822 (sd 0.192) vs +fails 0.778 (sd 0.039); difference +0.044, se 0.113,
  95% CI [-0.443, +0.532]  <-- does NOT exclude an RLPD-sized effect (-0.400).
At 3v3 the interval spans from a large benefit to a harm larger than RLPD's: the data support the
POINT ESTIMATE of no difference and nothing else quantitative. The earlier claim ("excludes an
RLPD-sized effect by 0.04") was wrong and must not be repeated. Exact
permutation p = 1.00, but the MINIMUM attainable p at 3v3 is 0.10: the design cannot reach
significance even under perfect separation.
DEFENSIBLE CLAIM AS OF NOW: only the direction -- "the point estimate shows no harm to the world
model from the tapes that cost RLPD 73%". NOT "the effect is excluded", NOT "failure demos are
harmless to world models". Any interval statement must wait for the added seeds.
Variance is lopsided (dR2D sd 0.192, driven by one 0.60 seed; +fails sd 0.039), so seeds buy a lot:
+4 seeds/arm (84-87, LAUNCHED) are now NECESSARY, not merely strengthening: at n=7-8/arm and the
observed spreads the CI half-width should fall to ~0.20, which would finally let the interval
speak. Report the CI, not just the means -- and use the RIGHT critical value for the Welch df.

## N14 (2026-08-26, LAUNCHED). Is the result robust to WHICH human demos we drew?
USER (the sharpest framing of the week): "all that matters is the result is robust to the human demo
version. Otherwise this is bad science."
Existing evidence, across a version change that altered the follower rule (meas -> either), the tape
count (51 -> 58) AND the world:
  DP rnd:       0.57 -> 0.55   STABLE
  DP hold:      0.81 -> 0.91   ceilinged (N11), uninformative
  RLPD sparse:  0.42 -> 0.18   NOT stable -- and N10 records that world and version cannot be
                               separated by re-recording, because demos ARE recordings made in a world.
THE TEST THAT DOES SEPARATE THEM: split-half. Same world, same recorder, same follower rule, same
teacher; only WHICH tapes you hold differs. baselines/make_split_halves.py cuts an IC-stratified
disjoint pair (whole tapes, so both halves stay valid for RL as well as BC).
LAUNCHED: matched_w3 dH and dDP each split 29/29 (58 ICs, alternating whole ICs); DP on
dH_A/dH_B/dDP_A/dDP_B x seeds 40-42 = 12 jobs.
READOUT: NOT absolute performance (29 demos will underperform 58) but whether the dH-vs-dDP
ORDERING is the same in half A as in half B. Same ordering in both -> the finding is not an artifact
of the particular draw. Ordering flips -> it is draw-specific and must be reported that way.
PREDICTION (registered): ordering preserved and small in both halves, consistent with N11's reading
that BC source effects are below the resolution of this design.

## N15 (2026-08-27, SUPERSEDES N12). Failure tapes DO hurt the world model — about half as much as RLPD
The E1 re-score (CRITIQUE_decisions_2026-08-26; `cluster/r2d_rescore.sh`, fresh process per episode,
PREREG §5 readouts) changes the primary result. Same tapes, same world (old-world matched_v2):

  readout   dR2D (success-only)      dR2D + 8 DP fail tapes    diff      95% CI            perm p
  hold      0.767 (n=4)              0.533 (n=3)               +0.233    [-0.244,+0.710]   0.286
  rnd       0.642 (n=4)              0.478 (n=3)               +0.164    [-0.234,+0.562]   0.286
  (RLPD, hold, same tapes: 0.550 -> 0.150, diff -0.400)

WHAT CHANGED AND WHY: N12 reported `sel` — 0.822 vs 0.778, "within noise". `sel` contains uid 234,
unachievable by construction (N11: picked 0/430), so its ceiling is 14/15, and the r2dreamer runs
both SELECTED and REPORTED on it. Moving to the protocol readouts multiplies the measured gap by 5x.
A ceilinged, selection-contaminated readout hid a real effect. This is the most consequential
methodological error of the project so far, and it was caught by an adversarial review of the
assistant's own decisions, not by the assistant.
CORRECTED CLAIM: failure tapes hurt BOTH learners; the world model loses ~30% relative where RLPD
loses ~73%. The learner-specificity is a MATTER OF DEGREE, not a presence/absence dichotomy. Neither
difference is significant at these n (perm p = 0.286 for both readouts, and the CIs span zero) --
the DIRECTION is consistent across two independent readouts and the point estimates differ by ~2x,
which is suggestive and nothing more. Seeds 84-87 and 100-103 are running and must be pooled before
any interval statement.
DEAD CLAIMS: "the WM is unmoved"; "demonstration-set defects are learner-specific" in the
presence/absence form; the framing given to the user that this was a publishable dichotomy.
STILL LIVE: the mechanism asymmetry is real in direction and plausibly explains a 2x difference --
but the mechanism SENTENCE in N12 was also false (E2: demo states ARE imagination roots and DO take
actor/critic gradient; models.py:412, :384-393). The defensible version: at a demo state RLPD backs
up a recorded off-policy action through a bootstrapped target network, while a Dreamer critic backs
up an on-policy imagined rollout scored by a supervised reward head that predicts ~0 there -- no
max over an off-distribution recorded action, hence a weaker inflation channel, not no channel.
ALSO: the exposure caveat still applies (demos ~1-3% of the r2d ring vs ~36% of every RLPD batch),
and now cuts the other way -- the WM shows a 30% drop at 1/17th the exposure.

### N15 — AUDIT ADDENDUM 2026-08-28 (gate 2/3; full text `AUDIT_results_2026-08-28.md`)
The body above is left as written on 08-27. The audit found:
 (1) Point estimates reproduce. CIs used the wrong critical values; correct Welch: hold [-0.259, +0.726]
     (df 4.52), rnd [-0.228, +0.556] (df 4.17). rnd relative loss is 25.5%, not "~30%". Minimum
     attainable perm p at 4v3 is 0.057 — this design cannot reach 0.05. Episode-pooled tests are invalid.
 (2) "A ceilinged readout hid a 5x effect" is FALSE. On these seed sets `sel` already shows +0.211 vs
     hold +0.233. N12's 0.822/0.778 were on seed sets {80,82,83}/{80,81,83}; the change is that fails
     s82 (sel 0.13) landed and fails s83 (sel 0.80, COMPLETED) was overwritten in place by the 08-27
     relaunch and can never be re-scored. Attribution: seed bookkeeping, not the readout.
 (3) The mechanism paragraph is still false. r2dreamer (not dreamerv3-torch; `models.py` citations are
     the wrong tree) has TWO critic losses: imagined (`dreamer.py:514-522`, on-policy, supervised reward
     head `:470`, no max) AND a replay loss (`dreamer.py:539-563`, weight 0.3) that backs up a lambda-return
     along the RECORDED trajectory, fail tapes included. Recorded rewards/next-states DO enter a critic
     target at demo states. There is no argmax channel; there is a direct one.
 (4) Exposure is wrong by ~10x. `demo_reinject_every 150000` x `demo_duplicate 4` re-injects 19 times per
     run; ring share at steady state is ~9.8% (dR2D) and ~26.7% (fails arm; fail rows ~18.8%) versus
     RLPD 50%/35.8% per batch. Ratio ~2x, not 17x. The "evicted by 450k" log line is prefill-only.
 (5) Design confound: the fails arm differs from the control in fail content, total demo share (2.7x)
     and tape length (301 vs <=25 rows, 70% of that arm's demo rows). "~30% vs ~73%" cannot be attributed
     to fail content or to a mechanism until the share-matched control (N5) runs.
SUPPORTABLE NOW: direction only — fail tapes lowered the WM's protocol readouts ~25-30% relative (CIs
span zero) vs ~73% for RLPD on the same 8 tapes. Re-scores for dR2D s86,87 / fails s84-86 and N13's
seeds are in flight (08-28); pool ONLY after applying the run-identity rule (a seed id = one run).

## N16 (2026-08-28, PRE-REGISTERED, launched). N15's confound is BOUNDED, not removed
USER (on the audit's "three-way confound"): fails are longer than successes BY NATURE in an episodic
task, so "adding fail tapes" cannot be separated from "adding long tapes" -- a success tape as long
as a fail tape does not exist. Correct. The whole-treatment observation in N15 ("adding these 8 fail
tapes lowered the WM ~25-30% relative, RLPD ~73%") stands as stated; what is at stake is the READING:
fail content vs. long/voluminous same-teacher data. Two controls, old world, matched_v2, dense,
TIME_LIMIT 400, 4 seeds each, launched 08-28 (jobs after n15ctl_build 2981314):
  dR2Ddup13   = dR2D tapes at demo_duplicate 13 -> demo row share of the 450k ring ~27% (= fails arm)
                with short success content only. Tests "row volume".           seeds 200-203
  dR2DDPsucc  = dR2D + the 8 LONGEST dDP success tapes (161-233 rows, 1521 rows total = 63% of the
                fails' 2400; success-terminated). Tests "long same-teacher tapes". seeds 300-303
                (baselines/make_succ_control_set.py; manifest records the 63% length match)
Readout: RESCORE hold+rnd (protocol), pooled with dR2D 80-91 and dR2DDPfails 80-91 under the
run-identity rule. Compare each control to dR2D (control-of-controls) and to dR2DDPfails.
PREDICTIONS (registered before any control result):
  (a) dR2Ddup13 ~ dR2D (within seed noise): row volume alone does not move the WM.
  (b) dR2DDPsucc <= dR2D by less than half the fails-arm drop: long success tapes cost something
      (imagination roots far from the policy's own distribution; replay critic on 200-row tapes) but
      less than fails.
  (c) If (a) and (b) hold, the fails-arm drop is attributable to fail content up to the untestable
      residual (length beyond ~233 rows, truncation vs termination). If EITHER control drops as much
      as the fails arm, "fail tapes hurt the WM" is NOT separable from "long/voluminous tapes hurt the
      WM" and must be reported in that form.
NOTHING here changes the RLPD side: its 73% was measured with the same intrinsically-long fails.

## N17 (2026-08-28, PRE-REGISTERED, launched). Same-source fails arms — the 1b question asked properly
USER RULE: never mix demo sources within an arm. So 1b ("does the learner x source picture change when
the source's OWN failures are included?") is dH vs dH+Hfails, dDP vs dDP+DPfails, dR2D vs dR2D+R2Dfails.
N15/N16 (dR2D + DP fails, and its mixed control) are demoted to mechanism cells.
SETS (old world, matched_v2; baselines/make_samesource_fails_arm.py; W5 lying-can exclusion; share 0.125 all three):
  dHHfails      = dH 56 + 8 human fails (dH_either_fails; 2 lying-can excluded)   sha 0dd7300e46fd927d
                  fail rows 2145 (mean 268; 7 cap-truncated no-picks, 1 tip)
  dDPfails      = dDP 56 + 8 DP fails (existing)                                    fail rows 2400 (8 x 300 cap-truncated)
  dR2DR2Dfails  = dR2D 56 + 8 r2d fails (dR2Dprov_fails; 1 success-labelled tape dropped)  sha 248d0ade864424de
                  fail rows 108 (mean 14; ALL 8 tip-terminated)
FINDING BEFORE ANY TRAINING (disclose in the paper): "failure" is not one thing across sources. Human and
DP failures are long no-picks that run to the cap; r2dreamer failures are fast knock-overs ended by the tip
rule at ~14 decisions. A same-source fails contrast therefore also varies fail LENGTH and fail TYPE with the
source. This is a property of the sources, not a confound to remove (user, 08-28: fails are long by nature).
LAUNCHED 08-28: RLPD sparse dHHfails/dR2DR2Dfails x seeds 10-13 (8; pairs with the existing dDPfails and the
old-world success rows); r2dreamer dense dHHfails/dR2DR2Dfails x seeds 80-83 (8); r2dreamer SPARSE dH/dDP/dR2D
x seeds 80-83 (12) for question 2 (dense vs sparse has NO world-model contrast so far). dv3 dense block (9)
queued; dv3 fails/sparse arms wait on ignition. dDP+DPfails for r2dreamer still to launch (4).
PREDICTIONS (registered): (a) RLPD: dH+Hfails and dDP+DPfails both drop from their success-only cells
(long unterminated-by-success tapes dominate the buffer, N1/N5 mechanism); dR2D+R2Dfails drops little or not
at all (108 rows cannot move a buffer; tip-terminated tapes carry a true terminal). (b) r2dreamer: same
ordering, smaller magnitudes (N15 direction). (c) Sparse vs dense for r2dreamer: dense does NOT show RLPD's
collapse (0.55 -> 0.18) -- the shaping enters a supervised reward head, not a bootstrapped Q. If dense
collapses r2dreamer too, question 2 has a learner-independent answer and N2's retraction was premature in
the other direction.
