# Draft text: positioning against robomimic (background + discussion)

Written 2026-09-02 so the point is not lost. Plain technical English; numbers are the ones of record on
this date (old-world RLPD from the frozen matched_v2 block, corrected-world from A20; coverage from fig6/7).
Bracketed items are placeholders for numbers still landing (A31 machine arms, A33 dense). Old-world numbers
appear ONLY as a sensitivity disclosure (PREREG A34), never as headline rows.

---

## Background (candidate paragraph, related work)

The most cited evidence on demonstration source is the robomimic study (Mandlekar et al., 2021). It compared
three sources of demonstrations for the same tasks: a single proficient human, several humans of mixed skill,
and a machine generator (rollouts sampled from trained RL agents). Across several offline and demo-augmented
learners, policies trained on proficient-human data performed best, mixed-quality human data next, and
machine-generated data worst, often by a large margin. Since then, "human demonstrations are better than
machine-generated ones" has been the default expectation, and several data-generation systems justify a
human-in-the-loop step on that basis.

That study did not vary the environment, and it did not separate two things that differ between human and
machine data: the states the demonstrations visit (coverage) and how the demonstrator moves between them
(motion statistics such as pauses, speed profiles and corrections). We match the two sources on task, initial
conditions, number of demonstrations and success, and we measure both coverage and motion statistics
directly, so that a difference in learner performance can be attributed to one or the other.

## Discussion (candidate paragraph)

Our results do not reproduce the robomimic ordering. With demonstration sets matched per initial condition
and per count, the reinforcement-learning learner (RLPD) reached the same success from machine-generated
demonstrations as from human ones in our final environment (difference [−0.02], n = 8 seeds per arm,
pre-registered null). The behaviour-cloning learner (diffusion policy) was likewise indifferent to the source
once human data were pruned to the same length distribution [+0.06]. Only the world-model learner showed a
directional benefit from human data [WM: +0.25, n = 8 v 8, p = 0.13; A27 n = 12 v 12 pending].

We do not read this as evidence that the robomimic finding is wrong. We observed the same gap earlier in
the project: in an earlier version of the simulator, with stiffer contact and gripper parameters, the same
learner trained on human demonstrations beat the machine-demonstration arm by +0.21 success on random
starts, in three independent preparations (n = 8 v 8, p = 0.001). The gap vanished when the simulator was
corrected. What changed with the correction was not the human data but the machine generator: in the
earlier world its demonstrations covered 21% less of the end-effector workspace than the human set, and in
the corrected world only 5% less (Fig. 6, 7). The learner's advantage from human data tracked the coverage
deficit of the machine data, not any property of the human motion itself.

This gives a simpler account of the robomimic ordering. Machine-generated demonstrations are worse when the
generator explores less of the state space than the humans did, which is common when the generator is a
policy trained on narrow data or in an environment where it fails often. When the generator's coverage
matches the humans', the source of the demonstrations stops mattering for the RL and behaviour-cloning
learners we tested. The practical recommendation is therefore to measure coverage of machine-generated data
against a human reference, rather than to assume a human-in-the-loop step is required.

Two limits apply. First, the world-model learner's directional preference for human data [is/is not]
confirmed at n = 12 v 12, and our burstiness ablation (retimed, smoothed and noised demonstrations) [does /
does not] isolate a motion-statistics cause. Second, the earlier-world result is a sensitivity observation
from a superseded simulator, reported here only to explain why a null on n = 8 should not be read as low
power: the same design detected a +0.21 effect when one was present.

---

Notes for the writer:
- Keep the old-world result to this one paragraph. Do not add old-world rows to any results table (A34).
- The coverage percentages are from fig6/fig7 (EEF-voxel coverage, matched_v2 vs matched_w3 sets). Cite the figure, not the CSV.
- If A33 (RLPD on dense reward) is null as well, add one clause: "and this held under both sparse and shaped reward".
- If the audit of the stops-vs-creep metric stands (AUDIT_zero_action_2026-09-02.md), do NOT claim humans "stop" and machines "creep"; say "human and machine demonstrations dwell equally often; human rests are exact, machine rests jitter at sub-millimetre scale".
