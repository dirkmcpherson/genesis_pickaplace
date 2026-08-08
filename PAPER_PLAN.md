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
- **H4 (world-model arm):** DreamerV3-from-demos benefits ~equally from human and
  model demonstrations — |H-DV3 − M1-DV3| ≪ |H-DP − M1-DP| — because the world
  model consumes demos as DYNAMICS data (any competent trajectory teaches the
  transition function) while the policy improves against reward in imagination, not
  by imitating the demonstrator. Strong prior from our earlier simulation work.
  The paper's arc: BC inherits demo-source differences; world models route around
  them.
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
- Dataset size: **66** everywhere (`TARGET_KEPT=66` = the human demo count; the
  plan previously said 67, which is the cartesian-graded count -- one episode
  grades differently across replay modes). Any `short_of_target` harvest is
  reported, not hidden.

**The human demo set (N=66), by furthest stage reached:**

| stage reached | count | share |
|---|---|---|
| picked only          |  2 | 3%  |
| placed (set down)    | 38 | 58% |
| contact (slide to goal can) | 6 | 9% |
| **nested (full task)** | **20** | **30%** |

Provenance / why 66 is the ceiling: 93 in-the-wild trials recorded -> 2 stubs
(gripper never closes) excluded -> 91 graded -> 75 success-labeled + 16 fail ->
9 of the successes never achieve a pick under replay -> **66 usable for IL**
(the no-IL-on-failures rule, Decision Log 07-30). More human demos would require
new collection on the real robot; the 16 fail + 9 no-pick episodes remain
available as negative/RL data but never enter IL training sets.

**PHASE SCOPE (Decision Log 2026-08-01): the core paper is PHASE 1 — PICK.**
Rationale: matched-N is only feasible at pick (place demos are unharvestable from
a 0.00-place model policy; contact ~5x dearer); the pick predicate is the
hack-resistant one (the z-band `placed` credits a knocked-on-shelf can — observed);
and all existing cross-source results are pick-shaped. Phase 2 (place) follows
LATER as specialists trained FROM PICKED ENTRY STATES (reset to a banked
post-pick state; `placed_v2` release predicate gates its inclusion). Slide/nest
leave the controlled comparison; full-task H-DP numbers remain as descriptive
context. Fairness rule: BOTH demo sources truncate at the pick grant (same rule)
— previously H trained full-length vs M1 pick-truncated.

**Naming (user, 2026-08-01): `d{source}_{algorithm}`** — dH_DP (human demos → DP),
dDP_DP (DP-harvested demos → DP), dDP2_DP (gen-2), dACT_ACT, dH_DV3, dDP_DV3.
Old H-*/M1-*/M2-* names appear only in wandb archaeology before this date.

**Condition matrix (core, PICK phase):**

Three learner CLASSES (user 08-01; ACT dropped — duplicates DP's class):
pure imitation (DP) / RL-from-demos (SACfD) / world model (DV3).
**CROSS-ALGORITHM DEMO MATRIX (user 08-01, supersedes generational DP): each
learner trains on each source's demos. SACfD/DV3 also consume FAILED demos
(zero-reward negatives: human set 66+25, harvests --keep-fails 30); DP only ever
sees successes.** Sources: dH (human), dDP (gen-0 DP harvest), dSACfD (harvest
from the best dH_SACfD, wave 2), dDV3 (wave 3: needs the H4 gate to pass + a
dv3-teacher mode in harvest_ai_demos). 3×3(+diag) conditions ×3-5 seeds via
`cluster/launch_paper_week.sh` — rerun it as teachers appear; it fills in.

| id | class | demos | status |
|----|-------|-------|--------|
| dH_DP ×3    | BC   | 66 human pick-trunc  | week launcher (NB: full-demo predecessors hit 0.67/0.73/0.80 in-dist, 07-31 — kept as descriptive context) |
| dDP_DP ×3   | BC   | gen-0 DP harvest     | week launcher. Cap-600 lineage predecessor s0: **picked 0.87 in-dist / 0.33 random — above all human seeds** (self-distillation amplification) |
| dDP2_DP ×3  | BC   | gen-1 DP harvest     | launcher auto-harvests from the 0.87 teacher |
| dH_SACfD ×3 | RLfD | 66 human pick-trunc  | week launcher (train_sacfd_full --scope pick; smoke-verified 66/66 grants) |
| dDP_SACfD ×3| RLfD | gen-0 DP harvest     | week launcher (same m1_full harvest as dDP_DP) |
| dH_DV3 ×3   | WM   | 91 human (image)     | RUNNING (hdv3_pick s0/s1/s2, 5M budget) — also the H4 learnability gate |
| dDP_DV3 ×3  | WM   | gen-0 harvest (image)| launcher, after m1_full harvest lands (same demos as dDP_DP/dDP_SACfD) |

Archival: the ouro ACT lineage (gen0-2 trained, datasets banked) stays as
appendix/backup material; its conditions left the core when ACT was dropped.

**Stretch matrix — TIME-PERMITTING ONLY (Decision Log 2026-07-31). Runs only if
(a) the core matrix is complete with ×3 seeds, (b) a gap exists, and (c) there is
GPU-week left. Not on the critical path; the paper stands without it.**

| id | demos | noise |
|----|-------|-------|
| M1N-σ ×3 each | gen-1 model | Gaussian on actions, σ ∈ {0.5×, 1×, 2×} the human-vs-model entropy gap (calibrated from H2 measurement, not guessed) |

**Stage-wise matrix (Decision Log 2026-07-31; user-proposed):** evaluate each
stage in ISOLATION in addition to end-to-end, so the demo-source effect is
localized per stage rather than inferred through the funnel.

- Stages: pick (start -> grasp+lift), place (post-pick -> RELEASE-in-band, the
  placed_v2 predicate, not the mid-lift proxy), slide (post-place -> nested).
- Entry states for isolated eval: `reset_to_frame(uid, frame)` restores arm qpos +
  can pose from recorded stage boundaries; matched entry sets across sources.
- Specialists: DP only (ACT stays end-to-end). 3 stages x {human, model} x 3 seeds.
- Chained eval: pick->place->slide specialist switching = compounding measurement
  AND our best shot at a high-performing full-task teacher for later generations.
- Known data thinness, reported not hidden: slide has only 26 human segments;
  downstream model-demo harvests are stage-asymmetric in cost (a 0.67-pick teacher
  demonstrates picks cheaply, contact ~5x dearer) -> per-stage matched-N =
  min(human, harvestable).

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
- **2×2 obs×action study: VOID as built (07-31 night).** The control cell
  (jobs_jact = the positive-control format) reads 0.13/0/0 on cluster and 0.07 in
  a local replication — the `episodes_cartesian_dual` SOURCE is the problem
  (inconsistent time base, ~20% frame inflation vs the proven replay; see
  July30th_Fable.md §2 update). Dataset provenance alone moves joint DP 0.73→0.07,
  which also confounds the entire historical "cartesian BC fails" result. Rebuild:
  record the dual representation during the PROVEN vel-cmd replay, re-emit cells,
  rerun. The corrected eval mapping + `--obs` adapters remain valid tooling.
- **dv3:** IN the paper as the world-model arm (H4) — Decision Log 2026-07-31.
  Cluster segfault RESOLVED (poisoned env; verify_env.sh passes incl. H200). Local
  pick-scope run validating learnability. M1-DV3 needs a dedicated image harvest.
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
- [ ] Stage tooling: `reset_to_frame`, placed_v2 stage slicer, scope='place'/'slide'
      envs, per-stage eval protocol -- IN PROGRESS
- [x] Image harvest from the gen-0 teacher for M1-DV3 — folded into the paper
      smoke (job A of `cluster/launch_paper_smoke.sh`)
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
4. **H4 is conditional on dv3 learning at all.** If dv3 cannot reach nonzero
   picked even at pick-scope, the world-model arm is untestable and H4 drops to
   qualitative discussion. The cluster pick-scope smoke runs are the gate.
   **Contingency (user, 07-31): r2dreamer** (github.com/NM512/r2dreamer — same
   author as our dv3 fork; ICLR 2026; decoder-free, TD-MPC2-style latent-only
   rep learning, ~1.6x faster). Well-matched to the suspected failure mode:
   reconstruction loss burning capacity on the static scene while the can is a
   few dozen pixels (image_loss ~70 dominates our runs). Port = genesis env
   adapter + VEC facade + log_* keys + demo prefill into its Hydra structure,
   ~a day. TRIGGER: hdv3_pick seeds flat (log_picked 0) at ~2M steps.
5. **Seed budget.** 3 seeds × ~4h × (2 learners × ≥2 sources + noise arm) ≈ several
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

- 2026-08-01: PICK-PHASE CORE (user). Place phase later via specialists trained
  from banked picked entry states; build tooling with that in mind. Phase-matched
  truncation for both demo sources. Naming convention d{source}_{algorithm}.
- 2026-08-01: dv3 evals must show truth/model reconstruction for both cameras
  (train_openl-style); scope-matched eval mandatory (flail-place reward theater
  observed under full-scope eval of pick-scope policies). (user)
- 2026-07-31: Stage-wise evaluation matrix added (isolated per-stage + chained
  specialists); DP-only for specialists to bound scope. (user proposal, assistant
  scoping)
- 2026-07-31: dv3 is IN the paper: the world-model arm, H4 = world models benefit
  ~equally from all demo sources (strong prior from earlier sim work). Supersedes
  the proposed drop. (user)
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

## Deferred maintenance (not urgent, tracked so it isn't lost)

- Git history carries ~862MB of accidentally-committed datasets (2026-07-31).
  Cosmetic: fresh clones are slow. Fix = `git filter-repo` + coordinated
  force-push across all checkouts, some quiet day between runs. (user: "fine for now")

## Changelog (newest first)

- 2026-07-31 (assistant): PAPER SMOKE defined (user: one smoke before the week,
  across all algorithms). DP/ACT legs = the already-running ouroboros lineages
  (their gen-1 manifests + train curves are the pass criteria — no duplicate jobs).
  The launched piece covers the only zero-precedent paths:
  `cluster/launch_paper_smoke.sh` = verified `--images` harvest from gen-0 →
  pick-scope dreamer demos (M1-DV3 data path) → one dv3 multi job (H-DV3 pick ×2
  seeds + M1-DV3 pick ×1, full 5M budget — doubles as the H4 learnability gate and,
  if healthy, continues as real seeds). New tooling: `to_dreamer_demos --scope pick`
  (truncate at pick grant, terminal, +1; validated on all 91 human demos → 79
  pick-terminated, 12 zero-reward dynamics eps, `demonstrations/genesis_pick`),
  `DEMODIR`/`SCOPE` keys in `sbatch_genesis_multi.sh`. Pass criteria in the
  launcher header. Discovered en route: the local dv3 pick-scope gate run never
  existed (silent launch failure), and H-DP seed 1 lands picked 0.73 in-dist.

- 2026-07-31 (assistant): demo breakdown added; N corrected 67->66 (66 is the
  joint-graded human IL set; TARGET_KEPT follows). Running lineages used
  TARGET_KEPT=67 -- a 1-demo difference, noted as harmless but recorded.
- 2026-07-31 (assistant): noise arm demoted to stretch per user; priorities
  reordered (core matrix > distributional analysis > skeleton > noise tooling).
- 2026-07-31 (assistant): initial draft.

**Pruning note for the paper (user, 08-01):** DP requires idle-frame pruning
(collapse action-delta<1e-3 teleop pauses, pre-pick; make_dp_pruned) to be
functional. REPORT BOTH RATES: pruned human DP = 0.67/0.73/0.80 picked in-dist;
UNPRUNED same-frames control (jobs_jact_v2 s0) = 0.27/0.13 — a ~2.5x
preprocessing effect. dH_DP trains on the PRUNED pick-phase set (29.6% frames
dropped); SACfD/DV3 consume unpruned (idle frames are honest dynamics/negative
data there). Model harvests contain no idle frames (closed-loop teachers), which
is itself a source property the distributional analysis should quantify.

- 2026-08-02 (assistant): archival lineage generational curve COMPLETE at pick
  scope: gen-0 0.67 -> gen-1 0.87 -> gen-2 0.87 picked in-dist (single seeds,
  cap-600 harvests). Self-distillation amplifies once then PLATEAUS -- no
  model-collapse through two generations. Feeds the discussion section.

- 2026-08-02 (assistant): **dH_DP x3 COMPLETE (matched-rule, pruned pick-phase
  data): picked 0.80/0.60/0.60 in-dist (mean 0.67), random 0.13/0.33/0.20.**
  First full matrix row. dH_SACfD s0 official: 0.40 in-dist / 0.33 random -- gen
  gap 0.07, the smallest of ANY policy so far (RL generalizes; BC memorizes --
  echoes the 07-18 SACfD-vs-DP pattern at phase-1 scope).

- 2026-08-02 (assistant): **CORE RESULT LANDS — dDP_DP_s0 (matched-rule cap-1200
  harvest): picked 0.87 in-dist / 0.27 random. Model demos BEAT the human DP row
  (0.80/0.60/0.60) under the fairest protocol**, replicating the lineage 0.87.
  Seeds 1-2 training. SACfD rows COMPLETE (official evals): dH_SACfD =
  0.40/0.53/0.13 (mean 0.35, random 0.33/0.33/0.07); dDP_SACfD = 0.07/0.13/0.60
  (mean 0.27, random 0/0.2/0.2). RLfD verdict at n=3: source effect within seed
  noise — human and model demos serve RL comparably (H4-adjacent), while BC
  slightly FAVORS model demos. Seed variance in SACfD (0.07-0.60) is the
  dominant uncertainty; N=5 would sharpen it if the week allows.

- 2026-08-03 (assistant): **CENTRAL TABLE COMPLETE — BC head-to-head at matched
  everything (N=66, same truncation rule, same ICs, same recipe, x3 seeds):**
    dH_DP  = 0.80/0.60/0.60 in-dist (mean 0.67) | random 0.13/0.33/0.20 (0.22)
    dDP_DP = 0.87/0.73/0.67 in-dist (mean 0.76) | random 0.27/0.20/0.33 (0.27)
  **Model demonstrations BEAT human demonstrations on both distributions at the
  pick phase — H1 is refuted (inverted) here.** Consistent with the SACfD rows
  (no source effect) and the generational plateau (0.67->0.87->0.87): for this
  task/phase, success-filtered teacher demos are simply cleaner imitation
  targets, and nothing collapses across generations. The paper's arc holds with
  the sign flipped: "human data is special" is not supported at the grasp phase.

- 2026-08-08 (assistant): **CLAIM RECALIBRATION (Bayesian analysis,
  paper/results_significance.md): hierarchical P(model>human) = 0.76 BC in-dist
  / 0.64 BC random / 0.35, 0.21 SACfD — all indecisive at n=3.** The earlier
  "H1 refuted/inverted" language OVERCLAIMED: the defensible statement is
  "consistent direction favoring model demos for BC (~3:1 posterior odds), no
  detectable source effect for RLfD; n=5 in flight decides." Supersedes the
  08-03 phrasing wherever they conflict.

- 2026-08-09 (assistant): **FIRST WORKING WORLD-MODEL POLICY: r2dreamer v2
  (repeat-4, horizon-333, entropy 3e-5, demo re-inject) evals picked 0.20
  (3/15, full horizon, sample mode) on human demos** -- vs ~0 for every prior
  world-model eval in the project. The diagnosed pathologies (entropy ratchet,
  critic leak, timescale) were the blocker, not the architecture. dv3's null
  now carries the config asterisk. The H4 source comparison is LIVE pending the
  dDP twin (needs genesis_m1all rsync from cluster).
