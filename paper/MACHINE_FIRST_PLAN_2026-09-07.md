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
