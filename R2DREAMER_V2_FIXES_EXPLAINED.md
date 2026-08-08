# The r2dreamer v2 fixes, in plain terms

Four problems compounded. Each: what went wrong → how we saw it → the fix.

## 1. The entropy ratchet (why the policy got WORSE with more training)
Dreamer's actor is trained to maximize (advantage + entropy bonus). With a
sparse reward the advantage signal collapsed to ~0 (nothing to prefer), so the
only surviving gradient was "be more random" — entropy rose 20× over 3M steps
and steadily randomized an initially-decent policy. The 20%-picks phase existed
mid-run; by the time the final checkpoint saved, it was destroyed. **We never
had an eval bug — we were evaluating the corpse.**
**Fix:** entropy bonus 3e-4 → 3e-5, and evaluate periodic checkpoints, not just
the final one.

## 2. Critic poisoning (why value learning pointed nowhere)
The reward head is a classifier that never outputs exactly zero — it leaks a
tiny ~0.005 "maybe reward" per state. Harmless on short tasks; over our
~1000-step effective horizon the critic summed it into a predicted return of
~11 on a task whose true max is 1. The real +1 pick was a rounding error inside
that hallucinated value — the actor had no reason to chase it.
**Fix:** horizon 1000 → 333 (leak shrinks 3×; the true reward becomes visible),
which is also their own working-benchmark setting.

## 3. Demo starvation (why the demos stopped helping)
Replay is a FIFO buffer. The 95k demo frames — the ONLY frames containing
reward early on — were fully evicted by ~700k steps, after which batches
averaged 0.04 rewarded frames. The world model slowly forgot reward exists.
**Fix:** re-inject the full demo set every 300k steps (plus the earlier
stale-action alignment fix, so demos condition the model correctly at all).

## 4. Timescale mismatch (why everything was 4× harder than needed)
We ran 1200 decisions per episode at 30Hz because the demos are long — but the
demos are long because HUMANS dither: the learned policy picks in ~80 raw steps
(2.7s). Their benchmarks use ~250-500-step episodes. Long episodes stretch
credit assignment, shrink the fraction imagination can see (15 of 1200), and
inflate pathology #2.
**Fix:** action_repeat 4 (one decision per 4 sim steps → 300-decision episodes,
a pick ≈ 20 decisions), with demos downsampled by the same rule — verified by
paired open-loop replay: downsampled demos succeed exactly where full-rate ones
do, per-demo identical.

## Result
v1 final checkpoint: ~0/15 eval. **v2 final checkpoint: 3/15 (0.20)** — first
working world-model policy in the project, and the final checkpoint now RETAINS
its skill instead of decaying (pathology #1 gone).

## Do I expect further improvement?
**Yes, modestly — with calibrated expectations:**
- **Likely gains**: more steps (the v2 curve was not saturated; one 3M run is
  one draw), 2-3 seeds (world-model seeds vary like SACfD's), and one small
  tuning pass (entropy between 3e-5 and 3e-4; train_ratio) informed by v2's
  logged diagnostics. Reaching SACfD's band (~0.3-0.5) is a realistic target.
- **Not expected**: DP parity (0.8). BC directly imitates 66 curated successes;
  a world model must also learn dynamics + reward + planning from scratch.
- **For the paper it may not matter**: H4 is about SOURCE PARITY, not absolute
  skill. If dH and dDP twins land within noise of each other at ANY nonzero
  level, the thesis (imitators inherit source differences, model-based learners
  don't) is demonstrated. The dDP twin is blocked only on the genesis_m1all
  rsync from the cluster.
