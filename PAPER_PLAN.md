# PAPER PLAN — Human vs. Model Demonstrations (living document)

**Convention:** both of us edit this file; it lives in git so every change is
attributed. Add decisions to the Decision Log with a date. Anything marked
`[PROPOSED]` is not agreed yet. Status tables get updated as runs land — treat the
numbers here as the paper's single source of truth, superseding wandb archaeology.

---

## 1. Research question

**Does training an imitation policy on model-generated demonstrations differ from
training on human demonstrations — and if so, how, and can the gap be closed by
adding noise to model demonstrations?**

Contributions we can honestly aim for:
1. A controlled human-vs-model-demo comparison at matched dataset size, matched IC
   distribution, matched training recipe, ×3 seeds (the ouroboros gen-0 vs gen-1).
2. Distributional characterization of *how* model demos differ (entropy, smoothness,
   trajectory diversity, IC coverage) — mechanism, not just outcome.
3. **Stretch (time-permitting):** if a gap exists, a noise-injection intervention
   on model demos (σ-sweep calibrated from the measured entropy gap) testing
   whether the gap is an entropy/diversity artifact.
4. The methodology itself: verified harvesting (open-loop replay guard), negative
   controls, matched-size targeting — a template for honest self-training studies.

Venue target: ICRA (CoRL if timing fits). HRI framing deferred to a follow-up with
a human-perception study (see Decision Log 2026-07-29).

## 2. Pre-registered hypotheses (locked before gen-1 results are read)

- **H1:** DP trained on model demos underperforms DP trained on human demos at
  matched N, primarily in the *downstream* stages (place→contact), because model
  demos are success-filtered and lower-diversity.
- **H2:** The model-demo action distribution has measurably lower entropy and
  higher smoothness than human demos (which contain corrections/regrasps).
- **H3 (contingent):** moderate action noise on model demos recovers part of the
  gap; excessive noise degrades. (Inverted-U.)
- **Null outcome is publishable:** "no difference at matched N" is a real finding
  given the community's assumption that human data is special.

## 3. Experimental design

**Fixed choices** (= the validated stack; see Decision Log):
- Learner: Diffusion Policy, joint obs + joint actions, 100k steps, batch 64.
  (The ONLY configuration with a confirmed positive control: 0.67 in-dist,
  audit-replicated on the rebuilt dataset.)
- Eval: dual — demo-IC (headline) + random-IC (generalization), 15 eps each,
  identical `wandb_eval --ic-mode both` protocol, videos kept.
- Seeds: ×3 for every trained policy. NO single-seed claims (measured joint DP
  seed spread 0.07–0.53).
- Demo source ICs: the demos' own ~3 can positions (`IC_MODE=demo`) so IC coverage
  cannot confound source.
- Dataset size: 67 everywhere (`TARGET_KEPT=67` = the human demo count). Any
  `short_of_target` harvest is reported, not hidden.

**Condition matrix (core):**

| id | demos | learner | status |
|----|-------|---------|--------|
| H-DP ×3   | 67 human            | DP  | 1 seed done (0.67); seeds 1,2 NEEDED |
| M1-DP ×3  | 67 gen-1 model      | DP  | ouroboros lineage running → harvest pending |
| H-ACT ×3  | 67 human            | ACT | lineage running |
| M1-ACT ×3 | 67 gen-1 model      | ACT | pending gen-1 harvest |
| M2-DP ×3  | 67 gen-2 model      | DP  | pending (chain MAXGEN=3) |

**Stretch matrix — TIME-PERMITTING ONLY (Decision Log 2026-07-31). Runs only if
(a) the core matrix is complete with ×3 seeds, (b) a gap exists, and (c) there is
GPU-week left. Not on the critical path; the paper stands without it.**

| id | demos | noise |
|----|-------|-------|
| M1N-σ ×3 each | gen-1 model | Gaussian on actions, σ ∈ {0.5×, 1×, 2×} the human-vs-model entropy gap (calibrated from H2 measurement, not guessed) |

**Distributional measurements (no training needed, run on the demo sets):**
- per-dim action entropy; velocity/accel smoothness; DTW pairwise trajectory
  diversity; grasp-pose variance at pick frame; time-to-pick distribution.
- Same measurements on human vs gen-1 vs gen-2 → the "what is lost" figure.

## 4. Current assets and their status (updated 2026-07-31)

- **Positive control:** joint DP 0.67 in-dist / 0.27 random (seed 0, rebuilt data,
  audit-confirmed). Seeds 1–2 queued as part of H-DP.
- **Ouroboros pipeline:** end-to-end verified on the cluster (gen-0→gen-1→gen-2
  smoke completed). DP + ACT lineages running with TARGET_KEPT=67, IC_MODE=demo.
- **Verified-harvest guards:** negative control (rate-gated), open-loop verify,
  MIN_KEEP_FRAMES, dual-representation recording. Audit-clean.
- **2×2 obs×action study:** running on cluster (12 runs). NOT part of this paper's
  core claim — feeds the "why joint" justification and a possible appendix.
- **dv3:** local pick-scope run in progress; cluster segfault unresolved. dv3 is
  NOT in this paper's critical path `[PROPOSED: drop dv3 from the paper entirely]`.
- **Known-broken history:** see AUDIT_REQUEST_Fable.md — any number generated
  before its fixes must be re-derived, not quoted.

## 5. Division of labor (this week)

**Assistant builds/runs:**
- [ ] H-DP seeds 1,2 (local GPU, tonight)
- [ ] Distributional measurement script (`analysis/demo_distributions.py`) + the
      human-demo half of the H2 figure — runnable now
- [ ] Noise-injection tooling: DEPRIORITIZED to last -- build only after the core
      matrix + distributional analysis + paper skeleton are done
- [ ] Per-condition results table auto-generated from eval logs (`analysis/collect_results.py`)
- [ ] Paper skeleton in `paper/` (LaTeX), methods section drafted from the
      pipeline docs — methods can be written NOW, they don't depend on results
- [ ] Monitor cluster lineages; report gen-1 harvest quality (kept, short_of_target,
      rejected_by_verify) the moment it lands

**User decides/handles:**
- [ ] Gap threshold for triggering the noise arm (proposal above: Δpicked ≥ 0.15)
- [ ] Whether M2 (gen-2) is in the core paper or appendix
- [ ] Cluster babysitting: requeues, the two demo-dir rsyncs, scancel calls
- [ ] Venue/deadline pick (drives how hard we run the contingent arm)
- [ ] Framing pass on the intro once the core table has 3 rows

## 6. Risks, stated now

1. **Teacher strength.** Gen-0 DP at ~0.67 in-dist yields gen-1 harvests at maybe
   ~500–900 rollouts for 67 kept. If a seed lands at the bottom of the spread
   (0.07), that lineage's harvest starves → `short_of_target`. Mitigation: the
   launcher trains gen-0 fresh; if its eval < 0.3 in-dist we re-seed rather than
   harvest garbage. `[PROPOSED]`
2. **Two-way comparisons need identical everything.** The eval-meaning change
   (07-30) and the dataset rebuild mean ONLY runs started after 07-30 are
   comparable. All paper numbers get regenerated inside this window.
3. **Verify-rejection rate on real teachers** is still unconfirmed at scale; if the
   first real harvest shows rejected_by_verify > ~5% we stop and diagnose before
   trusting any generated dataset (July30th_Fable.md §1).
4. **Seed budget.** 3 seeds × ~4h × (2 learners × ≥2 sources + noise arm) ≈ several
   GPU-days. Cluster A100s can carry all DP/ACT training (no genesis needed).

## 7. Paper skeleton `[assistant drafts, user reshapes]`

1. Intro — demonstration data economics; the self-training question
2. Related work — LfD, self-distillation/model-collapse, demo quality studies
3. Method — env, demos, verified harvesting, matched-N protocol
4. Experiments — core matrix; distributional analysis; (noise arm)
5. Results — outcome table + "what is lost" figure + (inverted-U)
6. Limitations — sim-only, one task, pick-scope emphasis, teacher ceiling
7. Conclusion

## Decision Log

- 2026-07-31: Noise arm is a STRETCH goal, time-permitting only -- core paper is
  the human-vs-model comparison + distributional analysis. σ calibration from the
  measured entropy gap agreed if it runs. (user)
- 2026-07-31: Paper scope = human vs model demos; noise arm contingent on a gap.
  Living doc created. (user + assistant)
- 2026-07-30: Demo can positions are the default IC distribution everywhere. (user)
- 2026-07-30: TARGET_KEPT=67 matched dataset sizes. (user: "similar AMOUNTS of data")
- 2026-07-30: IL never trains on failed demonstrations. (user)
- 2026-07-29: HRI venue deferred; ICRA/CoRL framing. (user + assistant)
- 2026-07-29: All headline claims require ×3 seeds. (standing honesty protocol)

## Changelog (newest first)

- 2026-07-31 (assistant): noise arm demoted to stretch per user; priorities
  reordered (core matrix > distributional analysis > skeleton > noise tooling).
- 2026-07-31 (assistant): initial draft.
