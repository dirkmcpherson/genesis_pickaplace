# Draft text: positioning against robomimic (background + discussion)

Written 2026-09-02 so the point is not lost; REVISED 2026-09-02 pm per AUDIT_approach_2026-09-02 findings 1 and 28
(the earlier draft misstated the robomimic MG result and described our machine arm as if it were an independent
generator). Plain technical English; numbers are the ones of record on this date (corrected-world numbers from
A20/§1/§3.1; coverage from fig6/7). Bracketed items are placeholders for numbers still landing (A31 machine arms,
A33 dense, A27) or for facts to verify against the cited paper before submission. Old-world numbers appear ONLY as a
sensitivity disclosure (PREREG A34), never as headline rows.

---

## Background (candidate paragraph, related work)

The most cited evidence on demonstration source is the robomimic study (Mandlekar et al., 2021). It compared three
sources of demonstrations for the same tasks: a single proficient human (PH), several humans of mixed skill (MH),
and machine-generated data (MG) — rollouts collected from reinforcement-learning agents (SAC) at several training
checkpoints, i.e. an independent generator with its own, partly unconverged behaviour. Its finding on source was
learner-dependent, not a single ordering: behaviour cloning (BC, BC-RNN) did far worse on MG than on PH on the two
tasks with MG data (Lift, Can), whereas the offline RL method BCQ held its performance on MG and was the best
learner there, while the same offline RL methods struggled on the human datasets [verify the exact success rates in
robomimic Table 1 / §5 before submission]. In other words, a source × learner interaction — imitation prefers human
data, value-based learning tolerates or prefers machine data — was already published; what has been carried forward
since is the shorthand "human demonstrations are better than machine-generated ones", and several data-generation
systems justify a human-in-the-loop step on that basis.

That study did not vary the environment, did not match the two sources on initial conditions or count, and did not
separate two things that differ between human and machine data: the states the demonstrations visit (coverage) and
how the demonstrator moves between them (motion statistics such as pauses, speed profiles and corrections). Its
machine generator was also unrelated to its human data. We ask a narrower question with those factors controlled:
**does re-executing a set of human demonstrations through a policy trained on them change their value to a learner?**
Our machine demonstrations are rollouts of a diffusion policy trained on the human set, at the same initial
conditions, success-filtered and matched per initial condition and per count to the human set — a distillation of the
human data, not an independent generator. This is the setting of practical interest for data-augmentation and
re-execution pipelines (DAgger-style relabelling, MimicGen-style regeneration, self-distillation), and it is the
weakest possible test of "source" in the robomimic sense: any effect we find is a lower bound on what an independent
generator would produce, and a null is a null for distillation, not for machine data in general. We measure both
coverage and motion statistics directly so that a difference in learner performance can be attributed to one or the
other.

## Discussion (candidate paragraph)

With demonstration sets matched per initial condition and per count, the reinforcement-learning learner (RLPD)
reached the same success from policy re-executed demonstrations as from the human originals in our final environment
(difference [−0.02], n = 8 seeds per arm, pre-registered null; detectable effect ≥ [0.36] at this n). The
behaviour-cloning learner (diffusion policy) was likewise indifferent once human data were pruned of leading idle
time [+0.06, not significant after the registered Holm correction] — expected, since the machine arm is a
self-distillation of the same tapes. Only the world-model learner showed a directional benefit from human data, and
that benefit is an *ignition-rate* effect: conditional on a seed learning the task at all, human- and machine-trained
world models score alike [WM: +0.25 on the mean, n = 8 v 8, p = 0.13; ignition [6 or 7]/8 v 3/8 — criterion to be
fixed per PREREG A35; A27 n = 12 v 12 pending].

This is consistent with robomimic's interaction rather than in tension with it: the learner that imitates is the one
most sensitive to what the demonstrator did, the value-based learner is not, and in our design the machine data are
by construction close to the human data. We observed the same learner-dependence earlier in the project in a way that
sharpens the reading: in an earlier version of the simulator, with stiffer contact and gripper parameters, the same
RLPD learner trained on human demonstrations beat the re-executed arm by +0.21 success on random starts, in three
independent preparations (n = 8 v 8, p = 0.001). The gap vanished when the simulator was corrected. What changed with
the correction was not the human data but the re-execution: in the earlier world the re-executed demonstrations
covered 21% less of the end-effector workspace than the human set, and in the corrected world only 5% less (Fig. 6,
7). The learner's advantage from human data tracked the coverage deficit of the machine data, not any property of the
human motion itself.

This gives a coverage account that also fits robomimic's MG result: machine-generated demonstrations hurt an
imitation learner when the generator visits a different or narrower set of states than the humans did, and help or
do not hurt a value learner, which only needs the visited states to be informative. When a re-execution's coverage
matches the human set's, the source stops mattering for the RL and behaviour-cloning learners we tested. The
practical recommendation is therefore to measure the coverage of regenerated data against a human reference, rather
than to assume a human-in-the-loop step is required — with the explicit caveat that we tested distillation, not
independent machine generators.

Two limits apply. First, the world-model learner's directional preference for human data [is/is not] confirmed at
n = 12 v 12, and our burstiness ablation (retimed, smoothed and noised demonstrations) [does / does not] isolate a
motion-statistics cause; the world-model result is also obtained under a critic whose return target was mis-scaled
(PREREG A32) and under best-of-K checkpoint selection, so it is a claim about learnability under selection, not about
the trained endpoint. Second, the earlier-world result is a sensitivity observation from a superseded simulator,
reported here only to explain why a null on n = 8 should not be read as low power alone: the same design detected a
+0.21 effect when one was present.

---

Notes for the writer:
- Keep the old-world result to this one paragraph. Do not add old-world rows to any results table (A34).
- The coverage percentages are from fig6/fig7 (EEF-voxel coverage, matched_v2 vs matched_w3 sets). Cite the figure, not the CSV.
- robomimic facts to verify before submission: MG = SAC rollouts from multiple checkpoints (Lift, Can only); BC/BC-RNN
  drop on MG; BCQ holds on MG; offline RL underperforms on MH/PH. Do not quote percentages from memory.
- Never call the machine arm "machine-generated demonstrations" without the qualifier "re-executed / distilled from
  the human set" (AUDIT_approach finding 1).
- If A33 (RLPD on dense reward) is null as well, add one clause: "and this held under both sparse and shaped reward".
- If the audit of the stops-vs-creep metric stands (AUDIT_zero_action_2026-09-02.md), do NOT claim humans "stop" and machines "creep"; say "human and machine demonstrations dwell equally often; human rests are exact, machine rests jitter at sub-millimetre scale".
- Ignition count: RESULTS §3.1 quotes 7/8 v 3/8 under "BEST hold ≥ 8/15" but its per-seed list gives 6/8 v 3/8 under that
  criterion (7/8 is BEST rnd ≥ 8/30) — analysis/DECOMPOSITION_2026-09-02.md; use whichever RESULTS settles on.
