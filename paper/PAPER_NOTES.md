# PAPER NOTES — findings and claims destined for the paper text

Started 2026-08-22. One entry per finding: the claim at the strength the evidence supports,
the evidence pointer, the caveats that must travel with it, and what would change it.
(PAPER_PLAN.md stays the coordination/decision log; RESULTS_MATRIX is the per-cell ledger;
this file is the prose-ready layer.) Notation: d{demo_source}_{algorithm}, e.g. dHpruned_DP.

## N1 (2026-08-22). Under RLPD the "demo-set effect" is about the FAIL tapes' relation to the training MDP, not human-vs-model origin

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
dH_R2D dense 4/4 seeds ignite (best-ckpt confirmations 0.76-0.98, mode 1.0 x4; first training pick
209-430k env steps) vs sparse dH 8/34 (p=0.007; vs same-era wave 3 2/10, p=0.015); ceiling
unchanged (0.91-1.00 = champion). Two of four collapse by 3M (final-ckpt 0.00), one drifts, one
sustains 0.93 — the BEST-checkpoint protocol is what makes the cell readable. RLPD dense 4/6 vs
8/16 sparse (p=0.65), pooled 0.28 vs 0.22, best seed unchanged (9/15). dv3 dense 0/3 at eval.
Caveats: dense ran only on dH, so "does the source effect appear under dense reward" (ALGORITHM_
STATE §3.5) is still open; shaping is training-only, all headlines are sparse fresh-process pick;
the r2d result rests on BEST_selected.pt files on the cluster (shutdown ~08-24).

## N3 (2026-08-22). dR2D_DP is the best BC cell; model demos beat human in BC by another step
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
dR2D successes -> fails = 74% of the demo buffer under per-transition sampling (vs 28% in the dDP
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
every cell sits near the floor. The spread is ALSO confounded with the same mediator as the fails
result: dR2D episodes are ~18 decisions vs ~115 for dH/dDP, so dR2D's reward density is ~7x higher
(5.5% vs 0.80% of rows rewarded). For RLPD, "demo source" and "reward density / episode length"
are not separable in the current design.
CONSEQUENCE: do not write "demo source does not matter" anywhere. The defensible claims are
(a) BC: no source effect once translation is matched (N6, conditionally stated);
(b) RLPD: an apparent source effect exists but is confounded with reward density and is
    condition-fragile -- separating them needs a length/density-matched arm, which is the SAME
    control N5 already registered for the fails question. Register it once, use it for both.
PROCESS NOTE: results are henceforth quoted only from analysis/results_table.py output
(paper/RESULTS_TABLE_2026-08-25.md), never from recollection.
