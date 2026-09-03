# The findings, in plain English

A readable companion to `paper/PAPER_NOTES.md`, which is written densely for reviewers and agents.
Same content, same status labels, no jargon. Updated 2026-08-28 after the gate-2/3 audit (`AUDIT_results_2026-08-28.md`).

## The cast

- **The task.** A simulated Kinova arm picks a can off a table. "Success" means a hardened lift:
  the can is up, the gripper is closed on it, and it stays that way.
- **Three demonstration sources.** **dH** = recordings of a *human* teleoperating the real robot.
  **dDP** = recordings of a *diffusion-policy* robot that was itself trained on the human data.
  **dR2D** = recordings of a *world-model* robot, also descended from the human data. The paper's
  original question was whether human or machine demonstrations teach better.
- **Four learners.** **DP** (diffusion policy) just imitates. **RLPD** is reinforcement learning that
  mixes demonstrations into its own trial-and-error. **r2dreamer** and **dv3** are *world models*:
  they learn to predict what the simulator will do, then practise inside their own imagination.
- **Three test sets.** `sel` (15 starting positions used to pick the best checkpoint), `hold`
  (15 different ones, used to report the score), `rnd` (30 random positions). Reporting on the same
  set you selected on is cheating, which turns out to matter a lot below.

---

## Live findings

**N15 — Bad demonstrations hurt every learner, but roughly twice as much for reinforcement learning.**
*(This supersedes N12 — see the retraction section.)*
We took the best demonstration set and deliberately added 8 recordings of the robot *failing*. RLPD
lost about 73% of its performance. The world model lost about 30%. So failure recordings are not
harmless to world models, as we briefly believed — but they hurt considerably less. **Confidence:
moderate on the direction, none on the size (audit 2026-08-28).** The gap is measured on 4 and 3
random seeds; with that few seeds no outcome could have reached p < 0.05 (the floor is 0.057; we got
0.29). Three things the audit corrected: the "world models were unharmed" reading did not change
because we re-measured properly — it changed because one seed landed low and one high seed was
accidentally destroyed by a relaunch; the world model sees the demonstrations far more than we said
(about a quarter of its replay buffer in this arm, not 1–3 %); and adding the failure recordings also
tripled the amount of demonstration data and made the tapes 12× longer, so "failure content" is not
the only thing that differs between the arms. More seeds and a volume-matched control are needed.

**N9 / N11 — "Which demonstration source you use doesn't matter" is true for imitation learning
only, and only on the test set that can actually measure it.**
The original headline (model demos beat human demos 0.96 to 0.62) disappeared once we recorded all
sources through the same pipeline. But two caveats emerged. First, that's an imitation-learning
result — reinforcement learning *does* show a source difference. Second, our main scoreboard was
saturated: one of the 15 test positions is impossible by construction (the can starts lying down and
the safety rule ends the episode immediately), so 14/15 is the real maximum and almost everything
scored 13–14. On the test set with actual headroom, human demos come out slightly ahead of the DP
robot's in both worlds — small, and not resolvable at 5 seeds. **Confidence: high that the original
gap was an artifact; low on any remaining small difference.**

**N1 (mechanism half) — The catastrophic failure we saw in August was a bookkeeping bug, not a fact
about machine demonstrations.**
Recordings of failed attempts were being written without an "episode ended here" flag. The learning
algorithm then treated those dead ends as if the robot might still recover from them, and its value
estimates exploded (to ~12,900 where the true maximum is 1). Fixing the flag fixed the learner.
**Confidence: high** — verified in the code and reproduced.

**N10 — Something changed when we fixed the simulator, and we don't know what.** Reinforcement
learning showed a clear human-vs-machine difference in the old simulator (0.42 vs 0.25) that vanished
in the corrected one (0.18 vs 0.20), where everything performs near the floor. Three explanations are
possible — the scores got squashed too low to see a difference; the datasets aren't literally the same
datasets; or the algorithm's settings were tuned for the old physics. **Confidence: this is an open
question, deliberately not reported as a finding.**

**N7 — Trimming the "dead time" out of human demonstrations changes nothing.** Humans pause and
dither; machine demonstrators don't. A standing theory was that this padding is why human data
looked worse. We cut out the still moments (12% of decisions at the first threshold) and imitation-learning
performance didn't move — 0.55 before, 0.58 after (3 seeds vs 10). **I had predicted this would hurt,
and it didn't**: predictions (1) and (2) of the pre-registration failed, prediction (3) is half run.
The pre-registered reading of this outcome was "idle density is back on the table as an explanation",
not "translation is confirmed" — an earlier version of this paragraph said the latter, which was a
post-hoc reversal (audit 2026-08-28). Honest version: no effect resolvable at this n; density is
neither confirmed nor excluded. **Confidence: low.**

**N4 — One of the two world-model implementations (dv3) has never reliably learned this task.**
Occasional flashes, nothing sustained. We report it as a documented negative, not as a comparison.

## Running now

- **N13** — the human-vs-machine question for world models, which has *never actually been run
  cleanly*. The version that suggested "human demos help world models" compared a cleaned-up human
  set against a machine set that still had failure recordings in it, so it was measuring the wrong
  thing.
- **N14** — the robustness check you asked for: split the human demonstrations into two halves and
  train on each. If the answer changes depending on which half you got, the finding is an accident of
  our particular recordings.

## Retracted or superseded

- **N12 — "World models are immune to bad demonstrations." Wrong, and it was my error.** I measured
  the world model on the saturated test set *and* picked its best checkpoint using that same set.
  Re-measuring properly multiplied the effect fivefold and produced N15. This was caught by an
  adversarial review of my own decisions, one day after I had personally documented the saturation
  problem and written a rule against exactly this.
- **N3 — "Machine demonstrations beat human ones for imitation learning."** The gap was in how human
  recordings were converted into the robot's control format, not in the demonstrations.
- **N2 — "Shaped rewards fix the world model's reliability."** Measured under a reward implementation
  that contained a bug and no longer exists.
- **N12a** — the statistics attached to N12. Corrected twice, both times by me, both times because
  I used the wrong lookup value in a confidence-interval calculation.

## Deferred

- **N8** — whether world models need to run at a coarser time step. Needs recordings that don't
  exist yet (camera images were only saved once per decision, not per simulation step).
- Place / nested phases — scoped out until the pick phase is finished.

---

## The one-paragraph version

The project set out to compare human against machine demonstrations. Most of what it found is that
the apparent differences were artifacts — of how human recordings were translated into the robot's
control language, of a saturated scoreboard, and of a simulator with three genuine physics defects
(no gravity compensation on the arm, a shelf modelled half-buried, and a contact stiffness eight
times softer than the engine's own stability limit, which let the gripper sink a centimetre into the
can). Once those are removed, the demonstration *source* mostly stops mattering. What does still
matter is the **learner**: the same 8 bad recordings cost reinforcement learning about 73% of its
performance and a world model about 25–30% — but at seed counts that cannot reach significance, with
arms that also differ in demonstration volume, and with a mechanism story that the 2026-08-28 audit
found incomplete (the world model's critic also trains directly on the recorded failure tapes). The
direction of that asymmetry is the result most likely to survive review; its size is not yet a result.
