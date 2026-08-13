# RLPD Literature Comparison: Is Our Sparse-Reward Manipulation Behavior Typical?

**Date:** 2026-08-13
**Scope:** Compare our RLPD results on the 4-DOF Kinova gen3-lite can-pick task (Genesis sim) against published RLPD (Ball et al. 2023) and RLPD-descendant literature (IBRL, SERL, HIL-SERL) to judge whether our ignition timing, seed reliability, and unstable 0.2-0.4 peaks are in-family or anomalous.

---

## Our setup and results (baseline for comparison)

- Task: 4-DOF-constrained Kinova gen3-lite, pick a can. **Terminal-only sparse reward**: +1 once, on sustained-hold lift, then episode ends. Horizon <=900 steps at ~30 Hz, delta-joint actions, **no action repeat**.
- Obs: 17-dim privileged state (joints, gripper, gripper effort, can pose GT, goal xy).
- RLPD faithful to Ball et al. 2023: symmetric 50/50 demo/online (128/128), 10 LayerNorm critics, target = min over random 2-of-10, UTD=10, SAC auto-alpha, **gamma=0.998**.
- Prior data: 91 human teleop demos = 83,465 transitions, but **only 66 rewarded frames total** (one terminal reward per successful demo; demos truncated ~2 frames past the pick).
- Results, 7 seeds x 200k env steps: 6/7 seeds reach nonzero eval success; first nonzero at **75k-175k** env steps; peak snapshot success **0.2-0.4** (n=10 eval eps/snapshot); success **oscillates** snapshot-to-snapshot (e.g. 0.40@150k -> 0.00@175k -> 0.00@200k in one seed; others still rising at 200k). One seed 0.3 on randomized ICs.
- Contrast: plain SACfD (single FIFO mixed buffer, no ensemble, UTD 1) = 0 success in ~30 runs on the same task.

---

## RQ1 — RLPD on sparse manipulation: budgets, success, demo/reward density

**Ball et al. 2023, "Efficient Online Reinforcement Learning with Offline Data"** (ICML 2023). arXiv: https://arxiv.org/abs/2302.02948 ; PMLR: https://proceedings.mlr.press/v202/ball23a/ball23a.pdf

- **Sparse manipulation domain = Adroit** (dexterous 28-DoF hand): three tasks — **pen, door, relocate** (Section 5). Hammer is *not* evaluated.
- **Offline data**: "a small set of human demonstrations and a large set of trajectories from a behavior-cloned policy trained on that human data." So the prior buffer is padded to large size with BC-rollout trajectories, not just the handful of human demos.
- **Reward density (critical difference)**: Adroit "sparse" is **binary +1 at EVERY timestep the task is in the solved state**. Return = "percentage of total timesteps in which the task is considered solved" (following Kostrikov et al. 2022). Example: Pen horizon = 100 steps; Normalized Score 80 means the goal was held for 80 of 100 steps (D4RL sparse-Adroit reward definition). **This is NOT terminal-only** — a single successful trajectory contributes tens of rewarded frames, and the episode continues accumulating reward after first success rather than terminating.
- **Horizon**: short (Pen = 100 steps; Adroit tasks are ~100-200 step horizons), not 900.
- **Online budget**: Adroit online phase = **300k env steps** (some tasks converge earlier). RLPD reaches strong performance "in the order of just 10k online samples" and beats the best prior reported Sparse-Adroit **Door** score by **2.5x**.
- **Hyperparameters** (Table 1/2): gamma = **0.99**; ensemble E = **10**; UTD/gradient-steps G = **20** (state-based); target-min subset Z is environment-specific (AntMaze Z=1); **10 seeds** per task.
- AntMaze (sparse navigation, sub-optimal offline data only): RLPD is "first to effectively solve all AntMaze tasks" in "less than a third the time-step budget of prior methods."

**Is our 75k-175k-to-first-success in-family?** Yes, in-family with the *envelope*, but on the slow end. RLPD ignites Adroit in ~10k online samples, but Adroit gives *repeated* reward per solved step over a 100-step horizon, so a single online success injects many rewarded transitions. Our task gives one reward frame per success on a 900-step horizon (~9x longer). Normalizing for reward-per-success and horizon, 75-175k steps to ignition on the harder reward structure is not out of family — it is the expected slowdown.

---

## RQ2 — Is oscillation/instability reported, or are curves monotonic?

- **Published RLPD curves are 10-seed means with std shading** (Fig. 7), which *smooths away* per-seed, per-checkpoint churn. The paper's own stability claim is about the **LayerNorm ablation**: removing LayerNorm "results in significantly higher variance across seeds and reduces mean performance," and on expert-only Adroit data without LayerNorm "no progress [is] made on any task" (Fig. 7 discussion). So the paper explicitly documents that RLPD sits near an instability boundary that LayerNorm barely tames — instability is a first-class concern, not absent.
- **General deep-RL with high UTD**: high replay/UTD ratios are known to damage network plasticity and produce "nonmonotonic and/or overly noisy" reward curves (Scaling-DRL survey, https://arxiv.org/html/2508.03194 ). RLPD specifically uses LayerNorm to prevent Q-value blow-up under high UTD; it operates in exactly the regime where non-monotonic curves are expected.
- **IBRL (Hu et al. 2023, https://arxiv.org/abs/2311.02198 , Fig. 3)** plots RLPD/RLPD+ learning curves on Robomimic (Lift/Can/Square) and shows RLPD **converging slowly and lagging**, with the gap widening on harder tasks — i.e., RLPD curves are *not* the clean fast monotonic ones; IBRL was built partly because RLPD's curves are slow/unstable on sparse manipulation from limited demos.
- **Key methodological note on OUR oscillation**: with n=10 eval episodes per snapshot, a true success rate of p=0.30 has a binomial SD of sqrt(0.3*0.7/10) = **0.145**. A 0.40 -> 0.00 swing between adjacent snapshots is well within pure eval sampling noise; it is not necessarily policy collapse. Published RLPD hides this by (a) short horizons + repeated reward giving lower-variance return estimates and (b) averaging 10 seeds. Our snapshot-to-snapshot swings are largely an *evaluation-noise artifact* layered on genuine near-threshold policy churn — both are consistent with the literature, not anomalous.

---

## RQ3 — Real-robot RLPD descendants (SERL, HIL-SERL): what do they actually run?

**SERL (Luo et al. 2024, https://arxiv.org/abs/2401.16013 ; https://serl-robot.github.io/ ):** RLPD-based. Learns PCB assembly, cable routing, object relocation to near-perfect / perfect success in **25-50 min** of real training per policy. Uses a **learned reward classifier** on images (not hand-shaped, but also not pure terminal state), forward-backward controllers.

**HIL-SERL (Luo et al., Science Robotics 2024, https://arxiv.org/abs/2410.21845 ; https://www.science.org/doi/10.1126/scirobotics.ads5033 ):** RLPD core + human interventions.
- **Demos**: 20-30 human demos to seed the demo buffer.
- **Reward**: **sparse binary classifier** (~200 positive + ~1000 negative examples ≈ 10 trajectories, ~5 min to collect) that grants +1 on completion, 0 otherwise, plus a small gripper-action penalty. The classifier labels **many** frames as positive per success (~200 positive examples), not one.
- **Control frequency**: **10 Hz** (paper notes 50 Hz is problematic for the system). Actions = 6D Cartesian twist + discrete gripper. **Low-frequency, Cartesian, effectively short-horizon.**
- **Training time**: **1-2.5 hours** real-world to reach **100% success** (vs 49.7% BC baseline).
- **Human interventions are injected into the online buffer** throughout training; intervention rate decays toward zero as the policy improves (Fig. 4). This continuously feeds *near-success and corrective* transitions — a dense stream of informative data that pure autonomous RLPD does not get.

**Takeaway:** Every real-robot RLPD success runs at **low control frequency (~10 Hz), short effective horizons, and a reward source far denser than one terminal frame** — either a classifier that fires on many frames or human interventions that continuously seed good transitions. None run 900-step, 30 Hz, no-action-repeat episodes with a single terminal reward per success.

---

## RQ4 — Known RLPD failure modes / sensitivities

- **LayerNorm dependence**: without it, RLPD has high seed variance and can make zero progress (Ball et al. Fig. 7). We use it (good).
- **Replay ratio**: RLPD is *not* sensitive to demo/online mix; 50/50 is the best compromise (Ball et al.). Our 50/50 is correct.
- **High UTD ↔ plasticity loss**: high UTD (we use 10) is the regime where non-monotonic/noisy curves and value churn appear; LayerNorm + ensemble are the only guards (Scaling-DRL survey https://arxiv.org/html/2508.03194 ).
- **Discount/horizon coupling**: gamma sets the effective planning horizon ~ 1/(1-gamma). At **gamma=0.998, effective horizon ≈ 500 steps**, which is **shorter than our 900-step episode** — a terminal reward emitted late in the episode is discounted by 0.998^k and, for k approaching/exceeding 500, is heavily attenuated at the start states the policy must learn from. Published RLPD uses gamma=0.99 (effective horizon ~100) on ~100-step tasks — a *matched* horizon. Discount-factor sensitivity (higher gamma -> slower convergence; the reward must remain visible through gamma^(steps-to-reward)) is well documented (e.g. https://milvus.io/ai-quick-reference/how-does-the-discount-factor-gamma-affect-rl-training ). This is also flagged in our own memory note "discount-horizon-vs-action-space."
- **Reward-frame density in the prior buffer**: RLPD's symmetric sampling assumes the demo half of each batch actually carries reward signal. With **66 rewarded frames in 83,465 transitions (0.079%)**, a 128-sample demo minibatch contains **~0.10 rewarded transitions on average** — i.e., the vast majority of demo minibatches carry **zero** reward signal; the demos function almost purely as coverage/BC-style state distribution, not as reward propagation. Published RLPD/HIL-SERL prior buffers are orders of magnitude denser in positive-reward frames (Adroit: every solved step; HIL-SERL: ~200 classifier positives + intervention transitions).
- **Truncated demos (~2 frames past pick)**: the demo buffer contains almost no examples of the *sustained-hold* success state or its immediate successors, so critic bootstrapping toward the goal has minimal in-distribution support for the rewarded region — a demo-coverage gap exactly where value must be highest.

---

## RQ5 — VERDICT

**Our behavior is broadly WITHIN the normal RLPD envelope for a sparse manipulation task, given how much harder our reward structure and horizon are than any published RLPD benchmark. It is not bizarre.** Specifically:

- **6/7 seeds igniting** is *good* reliability for RLPD near a reward-signal threshold; the paper itself shows RLPD flips to "no progress on any task" when its stabilizers are perturbed.
- **Ignition at 75-175k env steps** is on the slow side of, but consistent with, the family once you normalize for our ~9x longer horizon and terminal-only (vs per-step) reward. HIL-SERL's 1-2.5 h at 10 Hz is tens of thousands of steps with continuous human-seeded reward; our budget is comparable in robot-time terms.
- **0.2-0.4 unstable peaks rather than clean convergence** is expected: (a) the reward signal in the prior buffer is ~1000x sparser than published setups, so value propagation is weak and slow; (b) n=10 eval gives SD ≈ 0.145, so most snapshot-to-snapshot "collapses" are eval noise, not policy failure; (c) published clean curves are 10-seed means over short-horizon, reward-dense tasks — a presentation that would also smooth our curves.
- Plain SACfD scoring 0 in ~30 runs while RLPD ignites 6/7 is exactly the RLPD-vs-baseline gap the paper advertises — a strong sanity check that our implementation is behaving as RLPD should.

**Top explanatory deltas for our lower/unstable success vs published RLPD (ranked):**

1. **Terminal-only reward density: 66 rewarded frames / 83,465 transitions (0.08%).** This is the single largest deviation. Published RLPD "sparse" tasks emit +1 on *every solved timestep* (Ball et al. 2023, Adroit return = % of steps solved; https://arxiv.org/abs/2302.02948 ), and HIL-SERL's classifier + human interventions seed hundreds of positive/near-positive frames (Luo et al. 2024, https://arxiv.org/abs/2410.21845 ). At our density, most 128-sample demo minibatches contain zero reward, so the demo buffer provides coverage but almost no reward propagation — directly predicting slow ignition and weak, churny asymptotic value estimates. The ~2-frame demo truncation compounds this by starving the buffer of the sustained-hold success region.

2. **900-step horizon at 30 Hz with no action repeat, coupled to gamma=0.998.** Every published RLPD/descendant success runs short horizons (Adroit ~100 steps, gamma 0.99) or low-frequency control (SERL/HIL-SERL 10 Hz, ~short effective horizons). Our 900-step/30 Hz/no-repeat episodes give an effective horizon (~1/(1-0.998) ≈ 500) *shorter than the episode itself*, so a late terminal reward is discounted away from the states that need it, and no-action-repeat inflates the step count between decision and reward — stretching gamma^(steps-to-reward) toward invisibility (our own "discount-horizon-vs-action-space" note; discount sensitivity literature). This most plausibly explains slow ignition and the failure to cleanly plateau by 200k.

These two deltas — not any implementation error — are the parsimonious explanation. Mitigations the literature supports: densify the reward frames (per-step hold reward or a HIL-SERL-style classifier that labels the whole hold region rather than one frame; keep demos un-truncated through the hold), shorten the effective horizon (action repeat / lower control rate toward ~10 Hz, or lower gamma to match the true steps-to-reward), and continue reporting seed-mean curves with per-seed bands and larger-n eval to distinguish eval noise from real churn.

---

### Sources

- Ball, Smith, Kostrikov, Levine. *Efficient Online Reinforcement Learning with Offline Data* (RLPD), ICML 2023. https://arxiv.org/abs/2302.02948 , https://proceedings.mlr.press/v202/ball23a/ball23a.pdf (Sec. 5, Fig. 7, Tables 1-2; Adroit pen/door/relocate, sparse binary per-step reward, gamma 0.99, E=10, G=20, 10 seeds, 2.5x Door).
- Hu et al. *Imitation Bootstrapped Reinforcement Learning* (IBRL). https://arxiv.org/abs/2311.02198 , https://arxiv.org/html/2311.02198v3 (Fig. 3; RLPD slow/lagging on Robomimic; IBRL 6.4x RLPD on PickPlaceCan @10 demos/100k online).
- Luo et al. *SERL: A Software Suite for Sample-Efficient Robotic RL*. https://arxiv.org/abs/2401.16013 , https://serl-robot.github.io/ (RLPD-based; 25-50 min real training; classifier reward).
- Luo et al. *Precise and Dexterous Robotic Manipulation via Human-in-the-Loop RL* (HIL-SERL), Science Robotics 2024. https://arxiv.org/abs/2410.21845 , https://www.science.org/doi/10.1126/scirobotics.ads5033 (20-30 demos; ~200 pos/1000 neg classifier; 10 Hz; 6D twist; 1-2.5 h to 100%; interventions decay).
- *Scaling DRL for Decision Making* survey. https://arxiv.org/html/2508.03194 (high UTD/replay -> plasticity loss, non-monotonic/noisy curves).
- Discount-factor / horizon sensitivity: https://milvus.io/ai-quick-reference/how-does-the-discount-factor-gamma-affect-rl-training
- D4RL sparse-Adroit reward definition (binary reward per solved timestep). https://arxiv.org/abs/2302.02948 (Sec. 5, following Kostrikov et al. 2022).
