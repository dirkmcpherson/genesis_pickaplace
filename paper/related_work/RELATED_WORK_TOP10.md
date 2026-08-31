# Top-10 Related Work — selection and justification (2026-08-20)

PDFs live in this directory (gitignored — they do not enter git history; re-download
via the arXiv links below if missing). Selection criterion: the ten papers a
reviewer of *this* paper — human vs. model demonstrations at matched N across three
learner classes, with the no-collapse generational result and the
verified-harvesting methodology — would most expect us to engage, weighted toward
papers that either (a) anticipate a headline claim and force differentiation, or
(b) supply the mechanism/theory a claim rests on. One slot per role; near-misses
listed at the end with reasons.

The single most important framing fact from this sweep: **published precedent
already exists for machine-generated demos beating human demos** (RoboCasa 47.6%
vs 28.8%; MimicGen parity). Our novelty is therefore NOT "model demos can win" —
it is the controlled *attribution* (matched N/IC/recipe, closed-loop harvest from
a policy trained on the same human set, hierarchical seed statistics) and the
*learner-class factorization* (BC-only source effect; RLfD/WM indifferent). The
intro should concede the precedent explicitly and claim the factorization.

> **REVISED 2026-08-31.** The paragraph above is the 08-20 reading and its
> factorization is now WRONG in the interesting direction. The final-round-robin
> results (RESULTS_for_writing_2026-08-30.md) show the source effect is *largest*
> exactly where the 08-20 sweep expected indifference: RLPD prefers human demos by
> +0.21 on random ICs (LAST ckpt, n=8v8, p≈0.001) and DP-teacher demos destabilise
> its critic (6/8 vs 1/8 divergence); the world model ignites from human demos on
> 4/4 seeds vs 1/4 (n=8 pending); DP (the IL arm) is the one that is nearly
> indifferent (+0.06, p 0.041). The synthetic-beats-human precedent (RoboCasa /
> MimicGen / DemoGen) is an IL-only phenomenon and survives intact in our DP arm.
> The new framing and its grounding: see "GROUNDING THE 2026-08-31 FINDINGS" below.
> Five papers were added on 08-31 (entries 11–15).

---

## 1. Mandlekar et al., *What Matters in Learning from Offline Human Demonstrations for Robot Manipulation* (CoRL 2021)
`mandlekar2021_robomimic_what_matters.pdf` — https://arxiv.org/abs/2108.03298

**Role: the closest prior comparison; the paper we are the controlled version of.**
robomimic is the canonical study with both machine-generated (MG) and human (PH/MH)
datasets for the same tasks, and its finding — offline learners cope with
mixed-quality *machine* data better than mixed-quality *human* data — is the
nearest existing "demo source × learner interaction" result. But their MG data
comes from an RL agent unrelated to the human demonstrators, at different N, with
no IC matching and no seed statistics. Our design closes exactly those confounds:
the model source is trained *on* the human set it is compared against, N=66
everywhere, demo-IC everywhere, hierarchical Beta-Binomial over seeds. Also the
source for the human-proficiency (Worse/Okay/Better operators) framing our
"human data is heterogeneous" discussion leans on. Must-cite; likely first
paragraph of related work.
**08-31 role upgrade: this is now the canonical finding we REVERSE.** Their
verbatim claim — batch RL is "excellent at learning from suboptimal
machine-generated datasets but much worse at learning from suboptimal human
datasets" (BCQ 91% on machine Lift vs 7% on human Transport) — is the received
wisdom our RLPD result inverts (+0.21 human, machine demos diverge the critic).
The reconciliation is the generating process: their MG data is noisy RL
*exploration* (broad state coverage); our dDP data is a converged, success-filtered
IL policy (narrow coverage). Goes from "closest prior" to the intro's foil.

## 2. Nasiriany et al., *RoboCasa* (RSS 2024)
`nasiriany2024_robocasa.pdf` — https://arxiv.org/abs/2406.02523

**Role: the published precedent for our sign flip — the paper we must
differentiate from or be scooped by.** Reports MimicGen-generated trajectories
beating human demonstrations for multi-task policies (47.6% vs 28.8%). A reviewer
who knows this number will read our H1-inversion as confirmation, not surprise —
so we cite it and sharpen: their generated data is (i) trajectory-transform
augmentation, not closed-loop policy rollouts, (ii) at much larger N than the
human set (unmatched), (iii) single learner class. Our result isolates *why* and
*for whom* (BC in-distribution only; nothing for generalization; nothing for
RLfD). This is the paper that most changes how the intro must be written.
**08-31:** still the IL-side precedent, but it no longer threatens the headline —
the final results put the source effect in RLPD/WM, where RoboCasa says nothing.
Its role narrows to grounding the DP-indifference arm.

## 3. Mandlekar et al., *MimicGen* (CoRL 2023)
`mandlekar2023_mimicgen.pdf` — https://arxiv.org/abs/2310.17596

**Role: the dominant synthetic-demonstration paradigm; poses our research
question and leaves it open.** 50k demos from ~200 human seeds via
segment-transform-replay; explicitly observes that policies trained on generated
data can match equal amounts of human data and asks "when is human data actually
necessary?" — nearly verbatim our research question, unanswered there because
generation is geometric augmentation rather than a learned policy, and no
matched-N controlled comparison is run. Anchors the "automated demo generation"
paragraph (with SkillMimicGen / Isaac Lab Mimic as trailing cites).

## 4. Shumailov et al., *The Curse of Recursion / AI models collapse when trained on recursively generated data* (Nature 2024)
`shumailov2023_curse_of_recursion_model_collapse.pdf` (arXiv version) — https://arxiv.org/abs/2305.17493 · Nature: https://www.nature.com/articles/s41586-024-07566-y

**Role: the model-collapse anchor our generational result speaks against.**
Defines early/late collapse (tail loss, then distributional drift) for recursive
training on generated data. Our ouroboros curve (0.67 → 0.87 → 0.87, no collapse
through two generations under success-filtered, verified harvesting) is a
robotics-side data point in this debate; no robotics paper we found makes that
bridge, so it is ours to claim — but only if we frame it against the canonical
collapse citation. Cite the Nature version in the paper; PDF here is the arXiv
text.

## 5. Gerstgrasser et al., *Is Model Collapse Inevitable? Breaking the Curse of Recursion by Accumulating Real and Synthetic Data* (2024)
`gerstgrasser2024_is_model_collapse_inevitable.pdf` — https://arxiv.org/abs/2404.01413

**Role: the other side of the collapse debate — and the side our evidence lands
on.** Shows collapse follows from *replacing* data with model output; when data
*accumulates* alongside real data, test error plateaus instead of diverging. Our
no-collapse finding has exactly this structure (each generation is harvested
fresh from a teacher trained on verified, success-filtered data with the real
demos retained in the lineage's provenance), so citing only Shumailov would
misframe our result as anomalous when it is in fact the predicted accumulate-
regime outcome. The pair (4+5) gives the discussion section its frame.

## 6. Bousmalis et al., *RoboCat: A Self-Improving Generalist Agent for Robotic Manipulation* (2023)
`bousmalis2023_robocat.pdf` — https://arxiv.org/abs/2306.11706

**Role: the flagship self-improvement loop in robot manipulation.** Demonstrate →
fine-tune → self-generate ~10k rollouts → filter → retrain, at foundation-model
scale; integrating self-generated data measurably improves the generalist. It is
the existence proof that policy-harvested data works in-the-large; it isolates
nothing (no matched N, no IC control, no source factorization, no per-learner
analysis). Our ouroboros is the small-scale, hypothesis-driven, controlled
version. Also the natural citation for "verified/success-filtered harvest" as
standard practice whose *consequences* nobody measured in a controlled way.

## 7. Luo et al., *Geometric Entropy: When Trajectory Diversity Helps and Hurts in Imitation Learning* (IROS 2026)
`luo2026_geometric_entropy.pdf` — https://arxiv.org/abs/2606.20871

**Role: the H2/H3 theory paper; the inverted-U we pre-registered, published.**
An entropy metric over demonstration trajectories (normalizing away extrinsic
variation) with an inverted-U between diversity and IL performance, and the
optimal-diversity threshold *decreasing* with model proficiency/data/priors.
Directly frames our distributional analysis (H2: model demos have lower
entropy/higher smoothness) and gives the H3 noise arm — even unrun — a published
theoretical shape. Our entropy/smoothness/DTW measurements should cite and
compare against their metric; the "success-filtered demos are cleaner imitation
targets in-distribution but buy no generalization" claim is a specific instance
of their proficiency-dependent tradeoff. The most recent must-read on the list.

## 8. Belkhale, Cui & Sadigh, *Data Quality in Imitation Learning* (NeurIPS 2023)
`belkhale2023_data_quality_imitation.pdf` — https://arxiv.org/abs/2306.02437

**Role: the formal vocabulary for WHY human demos underperform in-dist.**
Formalizes dataset quality as action divergence + transition diversity and shows
noisy/multimodal demonstrations produce averaged, hesitant policies. This is the
mechanism behind our central BC finding (human demos carry corrections, regrasps,
idle pauses → higher conditional action variance → worse in-dist imitation), and
the frame for the idle-frame-pruning result (the 2.5× preprocessing effect is an
action-divergence intervention; model demos need none because closed-loop
teachers emit no idle frames — itself a source property, as PAPER_PLAN notes).
**08-31:** the unpruned-human DP control quantifies this — the first raw-tape seed
lands at 0.43 rnd vs the pruned band's 0.50–0.63 (n=1, two seeds finishing) —
i.e. DP's source-indifference is conditional on a human-only cleaning step.

## 9. Laskey et al., *DART: Noise Injection for Robust Imitation Learning* (CoRL 2017)
`laskey2017_dart_noise_injection.pdf` — https://arxiv.org/abs/1703.09327

**Role: the intellectual ancestor of H3 and of the "human noise as feature"
hypothesis.** Injecting noise into the *supervisor's* demonstrations forces
recovery behavior into the dataset and closes most of the BC-vs-DAgger gap
off-policy. H3 (noise on model demos recovers the diversity human demos have
naturally) is DART's logic applied to a learned demonstrator, and the null/
inverted H1 outcome is partly a statement about how much those recovery
behaviors were (not) worth at the pick phase. Belongs in related work whether or
not the noise arm ever runs.

## 10. Ha, Florence & Song, *Scaling Up and Distilling Down* (CoRL 2023)
`ha2023_scaling_up_distilling_down.pdf` — https://arxiv.org/abs/2307.14535

**Role: precedent for the student EXCEEDING its data-generation process.**
LLM+planner-generated demos distilled into a diffusion policy that beats its own
data-collection success rate by +33% — the same amplification shape as our gen-0
0.67 teacher → 0.87 student (self-distillation amplifies once, then plateaus).
Also the closest published "generated demos → diffusion policy" pipeline, i.e.
the same learner class as our BC arm. Differentiation: their generator is a
planner with privileged state, not a policy trained on the human data under
comparison; no human-vs-generated control.

---

## Bonus (already ours): Staley, Goel, Shukla & Short, *Agent-Centric Human Demonstrations Train World Models* (RLC 2024)
`staley2024_agent_centric_demos_world_models.pdf` — https://aabl.cs.tufts.edu/papers/rlc2024_james.pdf

Not counted in the ten (own prior work), but it is the documented origin of H4 —
human data guiding *world-model* learning rather than policy learning — and must
be cited as such; H4's "strong prior from earlier sim work" in PAPER_PLAN §2 is
this paper.

## Near-misses (kept in the background list, not the top ten)

- **MoDem (Hansen et al., 2212.05698) / MoDem-V2** — the strongest demos-in-MBRL
  cite for the world-model arm's methods section; lost its slot because Staley
  2024 already carries the H4 lineage and MoDem doesn't compare demo *sources*.
- **Lin et al., Data Scaling Laws in IL (ICLR 2025, 2410.18647)** — diversity >
  quantity; justifies the matched-N/matched-IC control axes. Methods cite, not a
  positioning threat.
- **Gandhi et al., Compatible Demonstrations (CoRL 2022, 2210.08073)** and
  **Learning to Discern (2310.14196)** — human heterogeneity hurts BC; subsumed
  in the paper by Belkhale's formalism + robomimic's MH data.
- **RLPD / IBRL / SERL / HIL-SERL** — already fully covered with primary-source
  detail in `paper/rlpd_literature_comparison_2026-08-13.md`; duplicating them
  here would waste slots.
- **PLD (2511.00091) and the 2025-26 VLA self-improvement wave (SRPO, πRL)** —
  timeliness cites for the intro ("when does policy-generated data substitute
  for human data" is now a scaling question); none run controlled source
  comparisons.
- **When a Robot is More Capable than a Human (2510.09096)** — inverse framing
  (capability-mismatched demonstrators); good discussion cite for reading our
  in-dist advantage as an embodiment/style-match effect.
- **Skalse et al., Defining Reward Hacking (NeurIPS 2022) + Krakovna et al.
  specification-gaming list + Amodei et al. 2016** — underwrite the
  instrument-failure ledger ("every reward-optimizing learner found the exploit
  before the task"); methods-section cites, verify before use (not downloaded).
- **DART-descendant smoothness/quality metrics (2604.23000) and the diversity
  benchmark (2402.14606)** — comparison baselines for the H2 measurements.

---

# ADDED 2026-08-31 (post-results sweep; downloaded same day)

## 11. Orsini et al., *What Matters for Adversarial Imitation Learning?* (NeurIPS 2021)
`orsini2021_what_matters_adversarial_il.pdf` — https://arxiv.org/abs/2106.00672

**Role: the other canonical source-matters study, and the methodological warning we
extend.** >500k trained agents across AIL design choices, with both synthetic and
human demos; verbatim conclusion: "artificial demonstrations are not a good proxy
for human data," and evaluating imitation algorithms only on synthetic demos
"may lead to algorithms which perform poorly in the more realistic scenarios with
human demonstrations." Like robomimic it frames HUMAN data as the hard case (for
AIL hyperparameters); our RLPD result shows the sign of the difficulty flips with
the generating process. We extend their warning beyond AIL: recipe conclusions
drawn on one demo source do not transfer — the same published RLPD recipe that is
stable on human demos diverges 6/8 on distilled machine demos.

## 12. Gao et al., *Reinforcement Learning from Imperfect Demonstrations* (2018/19)
`gao2018_rl_from_imperfect_demonstrations.pdf` — https://arxiv.org/abs/1802.05313

**Role: the mechanism ancestor for the critic-divergence finding.** Their
Normalized Actor-Critic exists precisely because Q-functions overestimate on
actions *unseen in the demonstration data*, and they fix it by normalizing Q down
on those actions. Our divergence result is the demo-SOURCE version of the same
phenomenon: success-only distilled demos have the narrowest state-action support,
the critic extrapolates outside it, and max critic loss runs 10²–10⁵. The fails
arm is the causal demonstration — restoring coverage with the teacher's own
failure tapes cures divergence (6/8 → 1/8) without touching the algorithm.

## 13. Xue et al., *DemoGen: Synthetic Demonstration Generation for Data-Efficient Visuomotor Policy Learning* (2025)
`xue2025_demogen.pdf` — https://arxiv.org/abs/2502.16932

**Role: the current front of the MimicGen line (one human demo → spatially
augmented synthetic set, visuomotor).** Freshest evidence that synthetic demos
substitute for human demos *for imitation learners* — consistent with, and
differentiated by, our DP arm exactly as MimicGen/RoboCasa are: generation by
transform, not by a policy trained on the compared human data; IL-only; no
value-learning or world-model consumer. Trailing cite alongside SkillMimicGen in
the "automated demo generation" paragraph.

## 14. Garrett et al., *SkillMimicGen: Automated Demonstration Generation for Efficient Skill Learning and Deployment* (CoRL 2024)
`garrett2024_skillmimicgen.pdf` — https://arxiv.org/abs/2410.18907

**Role: MimicGen's successor (skill-segmented generation + motion planning);
24× human-demo leverage.** Same citation duty as #13: the demo-generation
literature measures its synthetic data ONLY through IL students. Our
learner-class factorization is the missing experiment for this entire line: the
same synthetic-vs-human question, asked of a value learner or a world model,
gets the opposite answer.

## 15. Fu et al., *A Theoretical Perspective: How to Prevent Model Collapse in Self-consuming Training Loops* (2025)
`fu2025_prevent_collapse_self_consuming.pdf` — https://arxiv.org/abs/2502.18865

**Role: theory bridge between the collapse pair (#4/#5) and our coverage
mechanism.** Formalizes when self-consuming loops collapse and shows collapse is
governed by how much fresh/real signal each iteration retains. Read next to our
results: success-filtered distillation is a support-narrowing operation (the
collapse-adjacent step), and both of our machine-data pathologies — RLPD critic
divergence on dDP, and the ouroboros question — are downstream of that
narrowing; harvesting WITH failures is the accumulate-regime move that keeps
support wide. Lets the discussion connect the source effect and the
no-collapse generational result under one mechanism instead of two anecdotes.

---

# GROUNDING THE 2026-08-31 FINDINGS (draft paragraphs for related work / discussion)

**(1) The reversal.** The received wisdom on demonstration sources comes from
robomimic [1]: offline value-based learners are "excellent at learning from
suboptimal machine-generated datasets but much worse at learning from suboptimal
human datasets." Our central RLPD result inverts this: with the published recipe
at matched N, ICs, and seeds, human demonstrations outperform machine
demonstrations by +0.21 success on random initial conditions (n=8v8, p≈0.001)
and the machine-demo arm's critic diverges on 6/8 seeds against 1/8. The two
findings are reconciled by the process that generated the machine data. In
robomimic, machine data is the noisy exploration of an RL agent — broad
state-action coverage with mixed returns; ours is the output of a converged,
success-filtered imitation policy trained on the very human set it is compared
against — narrow coverage with uniform returns. Value backup is exactly the
learner that cares: Q-functions overestimate where demonstrations are silent
(Gao et al. [12]), and a distilled teacher is silent almost everywhere off its
own trajectory. Our failure-tape arm makes the mechanism causal rather than
interpretive: adding the same teacher's failed rollouts — data no
success-filtered pipeline would keep — restores coverage and cures the
divergence (6/8 → 1/8) while the human>machine performance gap persists
unchanged (+0.21, p 0.017), and a row-matched duplication control shows the cure
is not a dataset-size effect. "Machine-generated" is not one thing; what
transfers across studies is the coverage of the generating process, not the
species of the demonstrator.

**(2) Where the precedent survives.** The synthetic-demonstration literature —
MimicGen [3], RoboCasa [2], SkillMimicGen [14], DemoGen [13] — reports synthetic
data matching or beating human data, and our imitation arm agrees: Diffusion
Policy is nearly source-indifferent at matched N (+0.06 on random ICs, p 0.041,
and flat in-distribution). But that literature evaluates its data exclusively
through imitation students, and our factorization shows the equivalence is a
property of the learner class, not of the data: the same two datasets that tie
under DP are separated by +0.21 under RLPD and by +0.41 (n=4, n=8 pending) under
a world model. One asterisk runs the other way: DP's indifference required a
human-only preprocessing step (pruning idle frames — an action-divergence
intervention in Belkhale et al.'s [8] terms), and on raw human tapes DP's first
seed drops below the machine-demo band. Cleaning was trivial here; it will not
always be.

**(3) The evaluation-practice warning.** Orsini et al. [11] showed that
adversarial IL algorithms tuned on synthetic demonstrations mislead about
performance on human data. Our results generalize the warning to demo-augmented
online RL and world models, and sharpen it: the transfer failure is symmetric.
A recipe study run only on distilled machine demos would conclude RLPD is
divergence-prone and needs stabilization; run only on human demos, that
pathology is invisible (1/8). Any benchmark whose demonstrations come from
converged policies — including every pipeline that harvests, filters for
success, and retrains — is silently selecting for narrow-support data and will
mischaracterize learners that back up value through it.

**(4) World models and self-consumption.** The world-model arm extends our own
prior finding that human demonstrations are particularly effective for training
world models (Staley et al., RLC 2024): in the corrected world the
return-clamped DreamerV3 ignites from human demos on 4/4 seeds but from machine
demos on 1/4 (BEST-checkpoint hold 0.77 vs 0.33; n=8 in flight), while both
unmodified torch ports fail on this task under the published recipe regardless
of source — a disclosed, protocol-grade negative, not a comparison cell.
Finally, the coverage mechanism connects the source effect to the model-collapse
debate (Shumailov et al. [4]; Gerstgrasser et al. [5]; Fu et al. [15]):
success-filtered distillation is precisely the support-narrowing operation that
collapse theory worries about, and our two machine-data results — value
divergence downstream of narrow demos, and its reversal when the loop retains
failures — are a robotics-scale instance of the accumulate-vs-replace
distinction. What protects a self-consuming pipeline is not that its data is
"real," but that its data keeps covering the states the learner will ask about.

*(Numbers as of 08-31: RLPD sparse n=8v8 LAST-ckpt rnd; fails n=8v8; dup n=4v4
with n=8 running; DP corrected-world n=10v10; r2d W3 n=4v4 with n=8 due ~09-01;
sources: RESULTS_for_writing_2026-08-30.md §§1–4.)*
