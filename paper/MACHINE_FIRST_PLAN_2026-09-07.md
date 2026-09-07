# Machine-first demonstrations for our task — plan and registration (2026-09-07, written before any run)

*User (2026-09-07 13:40): "this 'with machine demonstrations that were distilled from the human ones' needs to be tested. We need a machine-first demonstration for our task." Context: every machine set so far (`dDP`, the phase harvests, `dDPfull`) descends from a Diffusion Policy trained on pruned human demonstrations; on robomimic Can, where the machine set is independent (SAC rollouts), RLPD dropped from 0.455 (mixed-human) to 0.147 (p 0.008) and BC-RNN from 0.93 to 0.39. Whether our Genesis nulls survive an independent generator is therefore an open question, not a footnote.*

## 1. Definition

A **machine-first** demonstration set is one produced by a policy whose lineage contains no human data: trained from reward alone (dense shaping allowed, potential-based), or scripted. Three candidate generators, in order of preference, each with a gate; the first that passes its gate becomes the set of record:

- **T1 — reward-only world model.** r2dreamer, the recipe of record (`bnormclamp1ent5`, state obs, corrected world w3, action_repeat 4, 1200 sim steps) with **no demonstration prefill** (`env.demo_dir` unset) and the **pick-scope potential shaping** the adapter already carries (`pick_shaping=true`, potential from `full_env.pick_shaping_phi`, shaping discount equal to the agent's — the adapter asserts this; the agent sets `pick_shaping_gamma` = 1 − 1/horizon). Because shaping raises the attainable return above 1, `return_clamp` must be re-registered at the shaped maximum (the potential is bounded; the agent measures max φ over the support box and sets the clamp to 1 + max φ, disclosed). 4 seeds × 1e6 sim steps.
- **T2 — reward-only SAC (RLPD without demonstrations).** `train_sacfd_full.py` recipe of record (γ 0.99, UTD 10, LN critics) with an empty demonstration buffer and the same shaping, 4 seeds × 4e5 decisions. If the trainer cannot run without demos, a `--no-demos` flag is added (disclosed).
- **T3 — scripted demonstrator** (only if T1 and T2 both fail their gates): a waypoint controller in the commanded Cartesian path (above the can → descend → close → lift), IK to joint targets under the recorder's caps; registered separately before use.

**Teacher gate (same bar as the distilled teacher had):** fresh-process `rnd30` MODE picked ≥ 0.45 on at least one seed (the DP teacher that produced `dDP` scored 14/30 = 0.467 on the same set; matching teacher quality is part of the design). The teacher seed of record = the lowest seed index that passes; every seed's rate is reported.

## 2. Harvest and set (mirrors how `dDP` was made)

- `record_demos.py --teacher r2d --checkpoint <T1 seed> --ic-mode demo --attempts 3 --mode sample --scope pick --verify` on the **66 human raw ICs** (the same starts `dDP` was harvested on; contract-v1 tapes, symmetric plain recorder, w3). For T2 the recorder gets a `sac` follower (the `harvest_ai_demos.py` SAC loader lifted into a follower class; disclosed).
- **`dRL`** = one success tape per IC, the first success by attempt order; N = the teacher's yield on the 66 starts (dDP: 58). No subsampling of the human arm; if the yield is below 50, the comparison is still run and the count asymmetry disclosed, as for `dDP`.
- Conversions exactly as for `dDP`: `to_dreamer_native.py --repeat 4 --terminal-reward 1 --with-state --state-only` → `demos_state/dRL`; RLPD reads the contract-v1 tapes natively (`cluster/sbatch_rlpd.sh` allow-list gains `dRL`).
- Characterization with the existing scripts (`paper/characterize_*.py`): idle fraction, speed, path length, approach geometry, saturation, nudges — `dRL` vs `dHv2raw` vs `dDP`; expected: near-zero idle, higher saturation and jitter than `dDP`. Rows per set reported (the quantity confound is disclosed, not corrected, exactly as for `dDP`).

## 3. Learners and evaluation (the existing protocol, unchanged)

- r2dreamer recipe of record on `dRL`, 8 seeds × 1e6 (`wmfix_s2.sbatch dRL <seed> 1000000 env.actor_dist=bounded_normal env.return_clamp=1.0 env.act_entropy=3e-5`); RLPD recipe of record on `dRL`, 8 seeds (`GAMMA=0.99 IC_FILE=baselines/eval_ics_v2_w3.json`). Both compared with the existing 8-seed `dHv2raw` arms (and `dDP` as the three-way).
- Statistic: LAST checkpoint, fresh process, `rnd30` MODE picked (SAMPLE alongside; hold15, rnd300 post hoc), exact two-sided permutation on per-seed counts, n = 8 v 8.

## 4. Registered predictions

- **P1 (world model):** |Δ(dHv2raw − dRL)| < 0.10 on rnd30 MODE. Prior: even. Falsifier: dRL below dHv2raw by ≥ 0.15 with p < 0.05 — "the world-model null depended on the machine demos being distilled from the human ones".
- **P2 (RLPD):** dRL below dHv2raw by ≥ 0.15 with p < 0.05 (prior ≈ 60 %, from the robomimic RLPD result); the alternative, |Δ| < 0.10, would mean the robomimic RLPD gap is about Can/SAC rather than about independence.
- **P3 (three-way):** dDP within 0.10 of dRL for the world model (distillation did not matter) — or not (it did).
- **Disconfirm branch:** no teacher passes the gate at the registered budgets → reported as "machine-first demonstrations are not obtainable from reward alone on this task at 1e6 steps"; T3 (scripted) is then registered and run; no teacher re-tuning without a new registration.

## 5. Budget and order

T1 (4 × ≈ 5 h) and T2 (4 × ≈ 4 h) in parallel → gate → harvest (CPU, ≈ 1 h) → build → 16 learner runs (≈ 5 h each) → readout. ≈ 120 GPU-h, about 36 h wall-clock at the current concurrency. Every job under the `mf_` prefix; ≤ 12 GPU jobs in flight for this arm while the robomimic matrix and its controls run.

## 6. Disclosures by construction

`dRL` is the teacher's own behaviour, not a re-execution of human intent: rows, idle, speed and saturation will differ from both `dHv2raw` and `dDP`; the pick-shaping potential is training-only for the teacher (the learners on `dRL` see the sparse +1, as every arm does); the recorder path and world are identical to the human and distilled sets; the teacher-quality bar is matched to the distilled teacher's rnd30 rate, not to the human demonstrator's.

## Amendment (a) — teacher configurations of record (registered 2026-09-07 14:20, before any T1/T2 submission)

**Order (user priority 13:55): T2 first, T1 immediately after; both on QOS normal (`-p gpu`, coordinator's queue note — the preempt QOS is saturated by the robomimic matrix); every job `mf_`-prefixed; ≤ 12 GPU jobs.**

**T2 — reward-only SAC.** Trainer = `baselines/rl/train_rlpd.py` (the RLPD trainer of record behind `cluster/sbatch_rlpd.sh`; §1 named `train_sacfd_full.py`, its SACfD ancestor — corrected, no recipe change) with the new `--no-demos` flag (demo_batch forced to 0, every batch 100 % online, no demo buffer; `rlpd_sac.py` keeps the 50/50 path byte-identical when demo_batch > 0). Everything else is the w3 RLPD recipe: γ 0.99, UTD 10, E = 10 / Z = 2 LayerNorm critics, auto entropy, delta_joint / delta_ref target / cap 0.025 / repeat 4, train horizon 1200 sim steps (300 decisions), online buffer 3e5 FIFO. Shaping: `--pick-shaping on` (φ = −2‖eef − can‖ from `full_env.pick_shaping_phi`, shaping γ = the agent's 0.99, φ(terminal) = 0). Budget as registered in §1: **4 seeds (0–3) × 4e5 decisions = 1.6e6 sim steps**; the §5 "≈ 4 h" line assumed the 1e5-decision RLPD budget — wall-clock is ≈ 14–18 h per seed (RLPD 1e5 decisions took 4–6 h incl. evals, `sacct` 3085499/3170398/3170402); the 3e5 FIFO evicts the first 1e5 decisions (recipe, disclosed). Checkpoints archived at 25/50/75/100 %. **Gate read: LAST checkpoint, `cluster/eval_sweep.sh sac` (one fresh process per episode, deterministic policy = MODE), hold15 + rnd30 of `baselines/eval_ics.json`**; the archived checkpoints scored on rnd30 as a disclosed secondary (the 25 % one = the RLPD budget of record) — descriptive, never the gate. Q watchdog stays at 2.0 (the shaped max return 1 + max|φ| ≈ 1.67 sits below it). Launcher `cluster/mf/mf_sac_teacher.sbatch` (`SEED=<s> sbatch -p gpu …`; smoke `STEPS=2000` first), run dirs `baselines/rl/checkpoints/mf_sac_t2_s<seed>` in the machine-first checkout `$LAB/mf_gp`.

**T1 — reward-only world model.** r2dreamer tree of record `$W/r2dreamer_fix` (unpatched), config `genesis_pick_state` + the recipe overrides `env.actor_dist=bounded_normal env.act_entropy=3e-5`, plus `env.pick_shaping=true` (adapter-side shaping once per agent step, shaping γ = 1 − 1/333 asserted equal to the agent's) and **`env.demo_dir` unset → no prefill** (the buffer starts empty; trainer start step 0). **`env.return_clamp=1.6714` = 1 + max|φ(s₀)|**, measured by `baselines/mf_measure_phi.py` (world w3, tool output 13:45): training-uid resets (74 success uids) max 0.4530 (uid 247, can (0.482, −0.183)); the 300 `eval_ics_rnd300.json` starts max 0.6640; the 4 support-box corners max **0.6714** at (0.592, −0.237); eef home (0.367, 0.011, 0.090) at every reset. Caveat: a mid-episode state farther from the can than any start has a shaped value above the clamp, which then truncates the target there (a flattening far from the can, no bias near it) — disclosed. 4 seeds (0–3) × 1e6 sim steps; launcher `$W/mf_teacher.sbatch` = `wmfix_s2.sbatch` minus the demo gate and the `env.demo_dir` argument (copy kept at `cluster/mf/mf_teacher.sbatch`); in-job fresh-process evals `EVAL_SETS="hold rnd"` (sample + mode); **the gate reads `fresh_eval_rnd30_mode/metrics.json` `picked`**. Run dirs `$W/runs/mf_r2d_t1_s<seed>`.

**Slip disclosed:** the worktree branch had been cut from an ancient base (895 commits behind `4dof-cartesian`, no `baselines/rl`, no `cluster/`); it was rebased onto `4dof-cartesian` (dd9942b) before any work — the registration commit is unchanged in content.

## Amendment (b) — review findings applied (registered 2026-09-07 15:15, before any teacher readout)

(1) **hold15 is in-distribution, not held out** (14 of its 15 starts are `dHv2raw` training starts — `paper/ADVERSARIAL_REVIEW_*_2026-09-07.md`, REVIEW_GUIDE §8): every hold15 cell in this arm is labelled "in-distribution"; **rnd30 MODE stays the out-of-distribution statistic of record** for the teacher gate and for P1–P3. (2) **Harvest selection stays the §2 rule literally**: per IC the recorder rolls out at most 3 attempts and stops at the FIRST success (attempt order = rollout-id order); `dRL` keeps that first success — never a best-of-N choice; attempts-per-IC and the yield are recorded in the log from the shard manifests. (3) Pick scope only: this arm builds no entry bank and evaluates no full-scope policy (the r2dreamer full-scope `nested` proxy and the `--dump-entries` grip-range mismatch found in review do not touch it).
