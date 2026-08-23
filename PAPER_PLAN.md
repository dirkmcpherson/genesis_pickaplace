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
| dH_RLPD ×7  | RLfD | 66 human pick-trunc  | **FIRST NONZERO MODEL-FREE ROW.** 6/7 seeds ignite; best seed s0 @150k = **0.40 demo-IC / 0.16 random**. Post-hoc best-checkpoint-of-best-seed — NOT a seed-mean; see 08-13 decision-log caveat |
| dDP_RLPD    | RLfD | gen-0 DP harvest     | NOT RUN — required to make the RLPD row a source comparison rather than a single-source result |

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

> **BLOCKED 2026-08-13 — delta-joint replay does not reproduce downstream demo
> phases.** All three learner arms now train on delta_joint actions, but the
> delta open-loop replay loses the demonstrated downstream outcomes: canonical
> VELOCITY-replay labels say contact 9 / nested 22, while delta replay through
> FullTaskEnv measures contact 5 / nested 4 at repeat-1. This blocks Phase 2
> (place/slide specialists) and every full-task delta number until resolved.
> The PICK phase — the core paper's scope — is unaffected. See the 08-13
> decision-log entry and FABLE_HANDOFF_2026-08-13.md §8.

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

- 2026-08-23 14:50 EDT (assistant, cluster session): **FINAL-RR WINDOW BLOCK LAUNCHED** on
  `baselines/matched_v2` (dH = recorder `--arrival either` 56/66; dDP 56 of 64 from the
  DP-r4 pilot s0 ckpt 020000; dR2D 56 of 64 from r2dreamer dH-dense s51 BEST (A2); identical
  IC multiset; fails arms 8 teacher fails). DP 3×5 seeds (10-14), RLPD 3×{sparse,dense}×4
  seeds (10-13) = 39 sbatch jobs; pre-launch audit passed with amendments A4-A8
  (paper/AUDIT_prelaunch_2026-08-23.md); dense smoke clean. DP-r4 pilots: hold 11-13/15,
  rnd 15/30 (vs stride-1 dHpruned_DP 0.62/0.23). Follower lab (paper/real2sim_follower_lab_
  2026-08-23.md): tier 1 (arrival test) adopted; tier 2 = SIMULATOR defects (no gravity comp,
  soft PD, robot mounted >=3 cm too low, gripper driven ~10 N·m into the can -> 7-8 mm
  penetration) with a real-data-justified fix (gravity comp + kp×4 + 3 cm riser: sim-vs-real
  joint error 0.047 -> 0.004 rad) DEFERRED to the full rerun (every teacher must be retrained
  in the new world). Session log: paper/SESSION_LOG_2026-08-23_cluster.md.

- 2026-08-23 (user): **Independent-audit schedule LOCKED** — milestone-gated (pre-launch /
  results readout / claim / change / reproducibility), fresh-context read-only auditors citing
  file:line; table in PREREG_final_round_robin_2026-08-23.md §11. First due: pre-launch audit
  2026-08-24 before the DP/RLPD block.

- 2026-08-23 (cluster session, assistant on pax020 via ssh; user decisions inline):
  **FINAL-ROUND-ROBIN PREREQUISITES STARTED.** (1) dH re-recorded through the learners'
  MDP with the contract-v1 recorder (`baselines/record_demos.py --teacher human`,
  repeat 4, cap 0.025, tip rule, 1200 sim steps): **51/66 kept** (bar >=50). The 15
  losses are deterministic (re-run byte-identical): 2 lying-can ICs (tip at decision 1),
  1 knock-over, 5 human demos longer than the horizon (source tapes 1260-2530 frames),
  7 where the human moved faster than the learners' cap (follower lags, dilation 1.1-3.0).
  Kept dilation p50 1.04 (max 1.36) — the follower tracks the human in real time.
  12 of the 66 have no recovered placement; `--ic-from-tape` resets to the tape's own
  frame-0 can pose (10 of those 12 kept). (2) dR2D teacher: CHAMPION_1576820.pt is NOT on
  the cluster; a PROVISIONAL dR2D set (64 tapes / 64 of 66 ICs) was recorded from the
  round-robin dH-dense s51 BEST checkpoint (1.00 on sel at selection); becomes the teacher
  of record (amendment) unless the champion is rsynced in time. (3) DP-r4 dH pilot (2
  seeds) training on pax020 — its median seed becomes the dDP teacher per PREREG §3.1.
  (4) USER PLAN: after this window, give a Fable an unlimited budget to improve the
  real->sim translation of the human demos (the recorder's kept/dilation/frac-at-cap on
  the pruned 66 is the scoreboard; baseline 51/66, 1.04, 5.5%); if fruitful, the whole
  chain (recorder -> matched sets -> sbatch) reruns mechanically. Reading list for that
  effort: paper/p1_delta_divergence_2026-08-13.md, paper/measured_ref_integration_2026-08-14.md,
  baselines/rl/rerecord_delta_demos.py, CAN_STARTING_POSITION.md.

- 2026-08-23 (user direction + assistant audit): **TIME-BASE STANDARDIZATION OF THE
  DEMO SETS — proposal, decision pending.** User: "we did action_repeat=4 to help the
  WM and RL algorithms, but it doesn't make sense to then give it DP demos that don't
  have the action repeat." Audited state of record (round robin 08-19):
  learners — RLPD action_repeat **1** (sbatch_rlpd.sh; the ar4/ar8 waves of 08-14 were
  pre-T1 and are dead), DP stride 1 (30 Hz frames, 1200-step eval), r2dreamer **4**,
  dv3 **4**. Tapes — dH: human 30 Hz joystick, stride 1; dDP: DP teacher acting at
  stride 1 (m1all_harvest, 1200-step cap), stride-1 tape; dR2D: r2dreamer champion
  acting at repeat 4, recorded per SIM step (stride-1 tape whose 4-frame windows are
  the teacher's own decisions). Consumption — RLPD/DP take the stride-1 tapes as-is;
  dv3 gets stride-4 windows via convert_genesis_demos_repeat.py (`*_msr_delta25_r4`);
  r2dreamer downsamples the stride-1 `*_delta25` dirs at load by the same window rule.
  So for the repeat-4 learners the stride-4 re-encoding is EXACT for dR2D by
  construction, near-exact for dH, and lossy for dDP: census tool (new "Time-base"
  table, analysis/characterize_demo_sets.py --stride 4, dev box 08-23): windows exactly
  representable dDP 0.1% vs human-proxy 54%; command-vs-ramp deviation p50 0.25 cap
  (6 mrad) vs 0.01 cap; windows with an intra-window direction reversal 94% vs 5%
  (the DP teacher's absolute-target command flips sign on ~40% of consecutive frames
  — diffusion jitter around a smooth measured path; the PD filters it, so the
  measured-state consequence is small, but the (s, a_window, s') triples the WM learns
  from are not the ramp the learner would execute). Conclusion: the model-demo arms
  are NOT on a common time-base with their learners — dR2D is the only natively
  repeat-4 model set; dDP is a stride-1 teacher's tape re-windowed. Options, cheapest
  first: (A) re-harvest dDP with the SAME DP teacher held at repeat 4 (policy queried
  once per 4 sim steps, target held) — needs a yield check first (the teacher was
  trained on 30 Hz data; hold-4 changes its closed loop); (B) make DP a repeat-4
  learner too (train on the stride-4 windowed human tapes, eval hold-4 at 300
  decisions) and harvest dDP from THAT teacher — every learner and every tape on one
  decision clock, dDP natively representable like dR2D; requires retraining the DP
  rows and the dDP set; (C) keep the learners, report the time-base per (set, learner)
  with the census table and treat dDP's re-windowing as a stated confound of the WM
  human-vs-model contrast (on top of the fail-tape confound). Recommendation: B as
  the paper's standard (one clock), with A as the immediate cluster test of whether
  a hold-4 DP teacher still picks. RLPD stays stride 1 unless re-positively-controlled
  at repeat 4. Numbers: scratchpad census_stride.md; to be re-run on the cluster
  over all five sets (dR2D expected ~1.0 exact).

- 2026-08-10 (night): ACTION-SPACE FIX STACK for the r2dreamer arm (user
  directive: "restrict the range of the env's diff_joint action space so it's
  not so different from the demonstrations"). Forensics on pick_delta_s0's
  learn-then-decay (train picked 0.24 -> 0.08, lunge-tips in 1-2 agent steps)
  found (1) r2dreamer's bounded_normal actor bounds the MEAN only — samples hit
  +-4, env executed clip(a) but buffer/imagination stored the raw value
  (fictional-action dynamics, the unnormalized-demo bug family on the policy
  side); (2) delta_cap 0.04 = 1.5x demo p99 speed (demo median |a| 0.002!);
  (3) v4_delta silently inherited stock entropy 3e-4 (10x v2 recipe). Fixes:
  bounded_normal_clipped dist (sample+rsample projected), cap 0.025 == demo p99
  speed (re-encoded demo set delta25, replay gate 3/3, permanent replay_gate.py),
  entropy 3e-5. dv3 audited NOT affected (ContDist absmax=1.0). msparity
  absolute-control killed at 850k: 0/2479 episodes, no stable entropy collapse
  — the MS recipe does not transplant onto absolute joints. v5_delta running
  local full-GPU. Details: r2dreamer/GENESIS_PORT_STATUS.md 2026-08-10 night.
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

- 2026-08-09 (assistant): **FINAL n=8 VERDICT (supersedes all earlier claim
  language): BC in-dist P(model>human) = 0.994 — DECISIVE. dH_DP 0.62
  (0.40-0.80) vs dDP_DP 0.80 (0.67-0.93). The advantage is IN-DISTRIBUTION
  ONLY (random: 0.23 vs 0.23, P=0.45). SACfD: indecisive both distributions
  (P=0.78/0.44; nominal direction flipped between n=3 and n=8 — pure seed
  noise), source-indifference stands. Paper claims: (1) model demos make
  measurably better in-distribution imitators at matched N; (2) they buy NO
  generalization; (3) RL-from-demos is source-indifferent; (4) no generational
  collapse. Full stats in paper/results_significance.md.**

- 2026-08-10 (assistant): **dSACfD 0/320 HARVEST MYSTERY SOLVED — the dH_SACfD
  teacher is a FLING policy, and its official 0.40 was predicate-gaming, not
  picking.** Local repro on identical demo ICs (paper_dH_SACfD_s0, both paths
  bit-audited: only seed difference is harvest's float32 action cast, 7.7e-7
  rad, chaotic scatter with EQUAL aggregate yield): the old single-instant
  predicate fires at step 6-11 FROM RESET (can whacked airborne, max_z up to
  0.48 ballistic) — the same exploit the r2dreamer v4 audit caught. wandb_eval's
  sticky per-episode `picked` counts those flings -> 0.40; harvest truncates at
  picked_at+PICK_TAIL (~20 frames) and MIN_KEEP_FRAMES=100 rejects EVERY one ->
  kept 0/320 (old-predicate emulation locally: rollout "successes" 2/8, all
  short-rejected, kept 0). The harvest guard was RIGHT; the eval number was the
  artifact. Under the hardened predicate (aa762ac) both paths agree: picked
  0.00 on demo ICs. CONSEQUENCES: (1) the dH_SACfD row (0.40/0.53/0.13) and
  dDP_SACfD row predate the hardened predicate and need re-eval before the
  paper cites them; (2) do NOT relaunch the dSACfD harvest from this teacher —
  it has no real picks to harvest; (3) harvest_ai_demos now writes
  rejected_short to the manifest + warns on the fling signature
  (short-rejects > kept), so this failure mode is self-diagnosing.

- 2026-08-10 (assistant): **PREDICATE CERTIFIED, v2 CLAIM RETRACTED.** DP
  positive control under the hardened pick predicate: 0.67/0.53/0.20/0.07 —
  byte-identical to its historical numbers, so the predicate change does NOT
  move genuine grasps and ALL paper BC/SACfD numbers stand unchanged. r2dreamer
  v2's 0.20 does not survive it (0/15 on recheck, reproduced deterministically):
  the world-model arms (dv3 AND r2dreamer, all configs) have produced NO
  working policy — every apparent success (v1 20% train, v2 0.20 eval, v4 30%
  train) was a measurement artifact caught by paired controls. H4 = a
  rigorously instrumented null; the diagnosis ledger (entropy ratchet, critic
  leak, timescale, predicate gaming) is the paper's methods contribution.

- 2026-08-10 (night, assistant): **RLfD ROW FINAL — SACfD IS ZERO UNDER THE
  HONEST PREDICATE.** The full hardened-predicate retrain wave (n=8 per source,
  200k steps, cluster; runs dH_SACfD_s0-7 / dDP_SACfD_s0-7 created 21:44Z)
  completed with rollout/ep_rew_mean flat at 0 (one seed 0.01) for the entire
  200k and eval 0.00 in-dist AND random for every seed of BOTH sources. The
  concurrent re-evals of the OLD checkpoints under the hardened predicate agree
  (0/0/0/0.07 so far): the previously reported SACfD band (0.07-0.60) was fling
  exploit top to bottom. PAPER FRAMING: at this budget the RLfD source
  comparison is degenerate (0 vs 0) — the honest claim is "every
  reward-optimizing learner in the study (SACfD, dv3, r2dreamer) found the
  predicate exploit before the task; success-filtered imitation is the only
  approach that performs at the pick phase without reward-integrity
  engineering." The old 07-18 'RL leads pick' narrative inverts. CONSEQUENCES:
  (1) wave-2 dSACfD conditions are DOA — the hardened dH_SACfD_s0 teacher
  picks 0.00, harvest would yield nothing; scancel the 7 pending dSACfD jobs
  (recommended to user). (2) Bayesian source analysis for RLfD is moot at n=8
  all-zeros. (3) A nonzero honest RLfD row would need a different experiment
  (longer budget / shaping / demo-speed action caps a la r2dreamer v5) — out of
  scope unless the user opts in.

- 2026-08-11 (user decision): **NO actor-BC in the world-model arm** — it
  imitates demonstrator actions and confounds the H4 design (WM learners must
  consume demos as dynamics/reward data only). The bc-run (picked 0.08 @153k,
  entropy collapsed from start) was aborted on this rule and is retained only
  as a diagnostic: an action prior makes the recipe learn quickly, so the WM
  bottleneck is exploration/credit assignment, not world-model capacity.
  Non-confounding levers now in play: demo_duplicate=4 + reinject 150k
  (pick_delta25d4_s0, running).

- 2026-08-11 (user: "do 1"): **SACfD MOVES TO DELTA-JOINT ACTIONS.** The
  r2dreamer action-geometry fix ported to the RLfD arm: FullTaskEnv gains
  action_mode='delta_joint' (per-step joint-target deltas, cap 0.025 = demo p99
  speed, leash 5*cap; target re-seeded on every reset variant), train_sacfd_full
  --action-mode delta-encodes the demo buffer with the same converter math
  (66/66 pick grants preserved), wandb_eval integrates deltas statefully and
  auto-detects the mode from a .action_mode.json sidecar (legacy zips ->
  absolute). Replay gate 3/3 (232/242/243 re-earn picks open-loop, 3-8 steps
  PD catch-up); 400-step train smoke + sidecar eval smoke PASS. Launcher:
  SACfD defaults to delta with '-dj' suffixed names/dirs; the 16 absolute-mode
  zeros of 08-10 stand as the matched control row. Rationale: SB3's actor is
  properly tanh-bounded, but absolute joint targets make its exploration
  thrash, which the hardened sustained-hold predicate can never reward -- DP
  works on absolute joints precisely because it does not explore (action
  geometry interacts with exploration, not imitation).

- 2026-08-11 (~05:45, assistant): **WORLD-MODEL ARM LANDS — CHAMPION 0.91/1.00.**
  r2dreamer + delta-joint actions (cap = demo p99 speed) + bounded actor
  samples + 4x demo density (demos as DATA only, no BC): checkpoint at 1.58M
  steps evals picked **0.91 sampled (1.00/0.87/0.87, n=45) / 1.00 mode (15/15)**
  on demo ICs under the hardened predicate — the first fully confirmed
  world-model policy in the project (banked:
  r2dreamer/runs/pick_delta25d4_s0/CHAMPION_1576820.pt). Training is bistable
  (diagnosed lambda-return explosion past the 100 max; checkpoint = phase
  lottery, 1-in-7 hit rate) — the return-target clamp remains the
  reliability fix. Seed 1 running. H4 unblocked: the dDP twin under this exact
  recipe is the paper's comparison. dv3-absolute rows stand as the
  action-geometry ablation; SACfD-dj cluster wave = the RLfD leg of the same
  story.

- 2026-08-11 (evening, assistant): **clamp hypothesis refuted by pilot** —
  return_clamp=100 eliminates target overshoot entirely (ret_max>105: 0.85 ->
  0.00 of logs) yet entropy-collapse cycles persist unchanged (15 vs 12 spike
  onsets over matched 1.1M). The 08-11 morning claim that clamping would
  stabilize training is retracted; bistability cause remains open (AMP
  inf-grads / reinject shocks are the live suspects). Stability for the paper
  rests on the best-checkpoint + independent-confirmation protocol, which is
  measurement-side and hypothesis-free.

- 2026-08-12 (assistant): **single-lever gamma pilot FAILED** — SACfD with
  delta actions AND gamma 0.998 (fixing the silent 0.98 that made the demo
  terminal ~1.6e-6 from start states) still scores 0.00 at 200k (train reward
  flat zero; both eval modes 0). The RLfD failure is therefore attributable to
  neither action geometry alone nor discount alone: with matched geometry,
  visible reward, and demo-seeded replay, one-step TD from a single critic
  still never consolidates. Remaining candidate differences vs the working
  world-model learner: value-propagation machinery (RLPD's ensemble +
  symmetric sampling + high UTD) and/or imagination-based credit assignment
  itself. The RLPD implementation plan (agent-audited 08-12; includes the
  measured cost table and pre-registered pilot criteria) is the motivated next
  step. In-train eval action-mode bug (snapshot evals of delta runs ran
  absolute) found and fixed en route — dj in-train CURVES are unreliable;
  final evals unaffected.

- 2026-08-13 (assistant, newbox_supp): **RLPD IS THE FIRST HONEST NONZERO
  MODEL-FREE RESULT** (~30 SACfD zeros precede it). Ball-2023-style: E=10
  LayerNorm critic ensemble, min over a random 2, symmetric 128/128 demo/online
  batches, UTD 10, delta-joint actions, gamma 0.998. 7 seeds x 200k local:
  **6/7 ignited**, first nonzero 75-175k. Best seed s0 @150k checkpoint:
  **picked 0.40 demo-IC / 0.156 random-IC**, placed/contact/nested 0.00
  throughout (a pick-phase-only policy). Videos sent to user 08-13.

- 2026-08-13 (assistant, newbox_supp): **TWO CORRECTIONS to the RLPD confirm
  numbers as first reported.**
  (1) **The "0.40 confirmed across 3 independent eval seeds" is ONE
  deterministic measurement repeated three times, not three samples.** es1/es2/
  es3 return a byte-identical per-episode outcome pattern (P.P...P..P..P.P over
  the same 15 demo uids, 232/234/235/...). The demo-IC eval holds ICs fixed and
  the SAC eval policy is deterministic, so the eval seed cannot perturb this
  arm; the 3x agreement carries no more information than a single run and is NOT
  evidence of robustness. Honest uncertainty on the headline stays binomial:
  6/15 = 0.40 +- 0.13 (1 SD). The random-IC arm IS genuinely stochastic
  (patterns differ every seed) and is where eval-seed replication has meaning.
  **Protocol consequence: for demo-IC evals, "x3" must mean 3 TRAINING seeds or
  3 stochastic-policy samples — repeating a deterministic rollout is a null
  operation.** This is a measurement-layer error of exactly the kind the 08-13
  poller-filter and 08-11 syncer incidents were: the runs were fine.
  (2) **Random-IC is 0.156, not ~0.30.** Per-seed 0.20/0.20/0.07 (3/15, 3/15,
  1/15). The 0.30 figure in SESSION_STATE_2026-08-13 was a single noisy
  snapshot; the x3 mean supersedes it. Generalization gap = 0.40 -> 0.156.
  Both corrections verified against primary evidence (scratchpad/
  rlpd_s0_confirm.log) rather than the wandb summary.

- 2026-08-13 (assistant, newbox_supp): **SELECTION vs CONFIRMATION FLAG on the
  RLPD row.** 0.40 is the best checkpoint (150k of 4 snapshots) of the best seed
  (s0 of 7), chosen after seeing the evals. Under the project's own
  selection-not-confirmation rule it is a SELECTED maximum and must not be
  quoted as the RLPD row's expected performance. The seed-mean over the 6
  ignited seeds (snapshot peaks 0.2-0.4) is the defensible row statistic; at
  n=10-15 eval episodes per snapshot (SD ~0.13) single snapshots are noisy.
  What is safely claimable: (a) RLPD IGNITES where SACfD never did (6/7 vs
  0/16), which is a claim about the ensemble/UTD/symmetric-sampling machinery,
  not about the demo source; (b) the ignition itself is the finding. A
  confirmation eval on a HELD-OUT criterion (fixed checkpoint chosen before
  eval, or stochastic-sample n=45 like the r2dreamer champion protocol) is what
  would turn 0.40 into a reportable number.

- 2026-08-13 (assistant, newbox_supp): **dv3 correction carried in from the
  Fable handoff (peer session):** `bounded_normal` is NOT a dv3 gap — dv3's
  actor is already bounded (`ContDist(absmax=1.0)`, models.py:278). It was an
  r2dreamer-only bug. The real dv3 gap is that it never received the recipe
  that made r2dreamer ignite: delta-joint actions + short horizon +
  terminate-on-success + single sparse terminal reward
  (dreamerv3-torch/MANISKILL_VS_GENESIS.md "Top-3"). The dv3 null is therefore
  NOT yet a clean architecture verdict — it is a null under a recipe the
  working arm never used, and the paper must say so.

- 2026-08-13 (assistant, newbox_supp): **Video-directory accumulation hazard.**
  `<ckpt>/rlpd_150000_steps_eval_videos/random/` held 22 mp4s for a 15-episode
  eval: filenames encode the OUTCOME (`ep4_picked` vs `ep4_fail`), so a re-eval
  writes a NEW file instead of overwriting, and stale episodes from earlier eval
  seeds accumulate. Counting `*_picked.mp4` in such a directory would have read
  7/15 = 0.47 where the true es3 rate is 1/15 = 0.07 — a 6x overstatement
  available to anyone who eyeballs the directory instead of the log. The
  deterministic demo-IC dir is immune (identical outcomes -> identical
  filenames -> true overwrite), which is why only the random dir shows it.
  Mitigation used: filter by mtime to isolate one run. Proper fix: stamp the
  eval-seed/run id into the video path.

- 2026-08-14 (assistant, newbox_supp): **ACTION-REPEAT CLOSED — CLEAN NEGATIVE.**
  All three arms post-hoc evaluated at the matched 100k-decision checkpoint under
  ONE documented protocol (explicit --action-mode delta_joint + --action-repeat,
  demo-IC 15 + random-IC 15, 400 env steps = equal SIM time, picks read from
  stdout):

  | arm | mean demo-IC | picks | seeds >=2 picks |
  |---|---|---|---|
  | stride-1 (7 seeds) | 0.039 | 4/105 | 1 |
  | ar4 (6 seeds)      | 0.023 | 2/90  | 0 |
  | ar8 (6 seeds)      | 0.000 | 0/90  | 0 |

  Monotone decreasing in N, no arm above the noise floor, stride-1 vs ar4
  indistinguishable (Fisher exact p=0.69). **Action-repeat bought nothing
  measurable at this budget; the horizon hypothesis from
  paper/rlpd_literature_comparison_2026-08-13.md is NOT supported by our data.**
  ar8 ran under a pre-registration committed before the data existed (>=3/15
  demo-IC = "worth pursuing"); 0/6 seeds cleared it. SCOPE LIMIT for the caption:
  these policies were TRAINED at repeat-N and evaluated at equal SIM time, so
  ar8's 0/90 does not establish that repeat-8 fails as a TRAINING regime -- it may
  be horizon-truncated at eval. The defensible claim is operational: at equal
  physical time, repeat-8 produces no picks in any seed.

- 2026-08-14 (assistant, newbox_supp + peer): **EVAL-LAYER DEFECT FOUND, AND
  BOUNDED.** Eval episodes were not independent: `step()` issued
  control_dofs_position targets that `reset()` never re-issued, so each reset's
  scene.step() ran under the PREVIOUS episode's controller target (peer's probe:
  identical-command divergence across histories 0.0506 rad; fixed 00:49:57, now
  7.4e-06; residual fresh-vs-used 0.0022 on grip effort remains = solver/contact
  caches, so the fix is PARTIAL). Per-episode outcomes at floor-level rates are
  chaotic (uid 243 dose-response over prefix lengths 1-8: T,F,T,F,F,F,F,T --
  neither accumulation nor parity).
  **BUT the measured rates held.** Two checkpoints x three cells (pre-fix
  sequence / post-fix sequence / post-fix 15-fresh-processes):
  ar4_s2@100k = 0.07 / 0.07 / 0.07 (same uid picking); dH_s0@150k = 0.40 / 0.40 /
  0.40 with the demo-IC pattern P.P...P..P..P.P **episode-identical** between the
  original confirm and 15 independent fresh processes.
  **Structure: real capability yields ROBUST picks (wide basin); floor-level
  rates yield BORDERLINE picks that flip under perturbation.** Mid/high rates are
  trustworthy; single-pick cells are not -- and every cell of the action-repeat
  table above is the latter, which is why it is reported as indistinguishable.
  `placed` did NOT hold (0.00 -> 0.07 on ar4_s2): the robustness conclusion
  covers `picked` only.
  **CONSEQUENCES.** (1) The fix CHANGED THE BASELINE rather than cleaning old
  numbers (uid 243 alone: False pre-fix -> True post-fix), so pre- and post-fix
  numbers are different experiments and "re-run everything post-fix" was never a
  repair path. (2) Re-running an identical eval gains NOTHING -- the eval is
  deterministic given a sequence, so repetition returns the same arbitrary draw
  with zero variance. Perfect reproducibility here is evidence of nothing (this
  is why the RLPD "0.40 confirmed x3" was a null operation). (3) Paper-facing
  standard going forward: ONE EPISODE PER FRESH PROCESS. Never compare across
  different n (in-train n=10 vs post-hoc n=15 are different experiments).

- 2026-08-14 (assistant, newbox_supp): **BC TABLE — RECOMMENDATION: DO NOT
  RE-RUN.** [PROPOSED, user decision] The n=8 BC rates (0.6-0.8) sit in the
  robust regime for which mid-range invariance was just demonstrated across both
  the reset fix and the protocol change. Re-measuring is unlikely to move
  P(model>human)=0.994. Proposed instead: a methods paragraph documenting the
  eval-layer defect, the fresh-process standard, and the two-checkpoint x
  three-cell robustness check. Cheaper, and more informative to a reader than a
  silently re-run table. Joint recommendation with the peer session.

- 2026-08-14 (peer session; recorded by newbox_supp): **RLPD ROOT CAUSE FOUND —
  ENTROPY BACKUP IN THE CRITIC TARGET** (paper/rlpd_audit_2026-08-14.md, commit
  2402184). SB3 SAC includes the entropy term in the critic target; RLPD disables
  it for every sparse domain (Ball 2023 Table 2). Ours was ON. At gamma=0.998 the
  zero-reward fixed point is 500*alpha*H, and that formula reproduces every logged
  Q window (269..2400 against a max task return of 1.0). Terminals -- the picks --
  received target 1.0 while non-terminals received ~400: **the critic paid 400:1
  AGAINST completing the task.** Explains marginal ignition, post-ignition decay,
  SACfD's uniform zeros (same defect), and the fixed-alpha explosion (that
  pre-registered prescription is retired). Fix wave running: dH_RLPD-nb_s{0,1,2},
  backup_entropy off as a SINGLE lever (two further suspected defects deliberately
  deferred for attribution).
  **CONSEQUENCE FOR THE ACTION-REPEAT ROW ABOVE: re-declared PROVISIONAL.** The
  measurement is correct as taken, but a flat result across N is exactly what a
  signal-invisible critic produces at ANY N, so no mechanism claim can hang on it.
  Caption: "measured under the entropy-backup defect; repeat-N to be re-tested
  post-fix if the fix wave ignites." RLPD > SACfD survives (shared defect; strict
  zero vs occasional picks).
  **METHODS EXHIBIT:** the Q-watchdog fired correctly at step 1001 (Q=2.82) and
  was waved off by the operator. The instrument worked and the human-in-the-loop
  overrode it -- the same pattern as the poller-filter false alarm (08-13) and the
  syncer false alarm (08-11). Belongs in the methods section beside the P2 chain.

- 2026-08-14 (assistant, newbox_supp): **HORIZON MISMATCH — RLPD AND DP ROWS ARE
  NOT MATCHED MEASUREMENTS.** RLPD evaluates at 400 sim steps
  (`train_rlpd.py:100` default); the DP/SACfD matrix takes `wandb_eval.py:30`'s
  default of **1200**. The median demo pick frame is **662** -- past the RLPD
  cutoff, inside the DP one. So `RLPD 0.039 @400` vs `dH_DP 0.62 @1200` cannot be
  compared as reported, and the action-repeat table's "equal SIM time" caption is
  correct WITHIN the repeat-N arms while silently unmatched against every other
  arm. Rule adopted: **state the eval horizon on every row, and never compare rows
  across horizons.**
  **A/B RESOLVED — THE MISMATCH COSTS RLPD NOTHING.** dH_s0@150k, demo-IC, 15
  fresh processes at each horizon (fresh processes make horizon the only variable;
  this is what the earlier sequence-based attempt could not do):
  **@400 = 6/15, @1200 = 6/15, pattern P.P...P..P..P.P IDENTICAL.** Tripling the
  horizon changes neither the rate nor which episodes succeed. The trained policy
  completes its picks well inside 400 steps -- the 662 median is a DEMO statistic,
  and the policy is simply faster than the demonstrations it learned from. So the
  horizon gap is real as a CODE FACT but has zero measured effect on the RLPD row:
  no re-measurement campaign is warranted, and the caveat reduces to a stated
  caption. LIMIT: measured on the strongest RLPD checkpoint (0.40). Weaker
  policies could plausibly be slower to grasp, so the finding is not automatically
  transitive to the floor-level 100k cells -- but it does cover the checkpoint that
  any RLPD-vs-DP comparison would actually cite.

- 2026-08-14 (assistant, newbox_supp): **ENTROPY-BACKUP FIX CONFIRMED BY
  INTERVENTION — THE NB WAVE IGNITES.** Pre-registered sweep (§4a-2: fixed 100k
  checkpoint, 15 fresh processes per IC mode per seed, explicit flags):
  dH_RLPD-nb s0 = 1/15, **s1 = 4/15 demo-IC (uids 235/243/246/248, each its own
  process) — clears the >=3/15 bar**, s2 = 1/15. First RLPD number that survives
  every methodological control now in force (pre-registration, fresh processes,
  contamination-derived threshold, authored sidecars). Establishes the
  entropy-backup diagnosis causally: one lever (backup off) restores ignition at
  100k where the defective config needed 150k on its best seed. Caption limits:
  ignition RESTORED, not RELIABLE (1/3 seeds; the defective config also had one
  good seed); s0/s2 are floor-level noise; Q still runs to 250-1600 with backup
  off, so the second overestimation channel (shared-LN diversity collapse, audit
  bug 2) remains live — per-member-LN wave running as the next single lever.
  PAPER CONSEQUENCE: the "every reward-optimizing learner found the predicate
  exploit / RLfD row is zero" framing (08-10) is superseded — the RLfD failure
  was a CONFIG DEFECT (SB3 entropy backup in a sparse task), not a fundamental
  limit; the honest RLfD row is "zero under the defect, ignites without it,"
  which strengthens the methods narrative (formula-matched diagnosis ->
  single-lever intervention -> pre-registered confirmation).

- 2026-08-14 (assistant, newbox_supp): **THE PAIR — THE RLfD HUMAN-VS-MODEL ROW
  LANDS: NO DETECTABLE SOURCE EFFECT.** First matched dH-vs-dDP RLPD comparison
  on a WORKING config (backup_entropy off, stride-1, 100k), identical everything
  except demo source (dH = 66 human pick-trunc; dDP = m1all_harvest, 93 eps
  incl 30 zero-reward fails). §4a-2 protocol: fixed 100k checkpoint, 15 fresh
  processes per IC mode per seed, explicit flags, authored sidecars.

  | arm | s0 | s1 | s2 | seeds >=3/15 | pooled picks |
  |---|---|---|---|---|---|
  | dH  | 1/15 | **4/15** | 1/15 | 1/3 | 6/45 |
  | dDP | 1/15 | 0/15 | 1/15 | 0/3 | 2/45 |

  Pooled Fisher p = 0.27. Ignition difference = 1 seed. The registered
  pre-exposure prediction (newbox_supp: both arms 0-2 seeds, |diff| <= 1, no
  separation beyond floor noise) SURVIVES. The registered power caveat governs
  the claim: **"no source effect detectable at n=3 per arm at 100k" — NOT "no
  effect."** At ~1/3 ignition power the contrast is weak by design; the
  hold-reward lever (1.96% reward density, 25x signal rate) is what raises both
  ignition and contrast power, so if hold ignites, the DEFINITIVE source
  comparison is a hold-reward pair (the two workstreams merge).
  Replication note: the dH arm reproduced the nb wave's profile EXACTLY (1/4/1
  on fresh training seeds) — arm-level ignition behavior of this config is
  stable, which strengthens both waves retroactively.
  **[STRUCK 2026-08-17, seed audit 2fbed2a: this was NOT a replication. The
  demo-RNG was hard-seeded to 0 in train_rlpd, so pair-dH with training seeds
  0/1/2 was a LITERAL RE-EXECUTION of the nb wave — same computation measured
  twice. The "exact" agreement was determinism, not stability. The pair's
  SOURCE comparison survives (the dDP arm was genuinely new; dH numbers remain
  a valid dH measurement, just not an independent one), but pair-dH contributes
  ZERO additional dH evidence. RNG now per-run (seed-0 back-compat proven).]**
  H4-adjacent reading: RLfD consumes demos as off-policy value data, and at
  this budget neither success-filtered model demos nor human demos separate —
  consistent with the BC-only source effect (in-dist imitation quality) being
  the paper's one confirmed source-sensitive pathway.

- 2026-08-14 (assistant, newbox_supp): **FOUR-WAVE INVARIANCE — THE IGNITION
  SIGNATURE IS A CONFIG CONSTANT AT 100k.** All §4a-2 fresh-process sweeps,
  fixed 100k checkpoints, three seeds per wave:

  | wave | lever varied | per-seed demo-IC | seeds >=3/15 | pooled |
  |---|---|---|---|---|
  | nb | backup off (base) | 1/4/1 | 1/3 | 6/45 |
  | pair-dH | fresh seeds | 1/4/1 | 1/3 | 6/45 |
  | hold | 25x reward density | 2/1/4 | 1/3 | 7/45 |
  | mref | measured-ref actions | 0/3/2 | 1/3 | 5/45 |

  Reward density, action reference, and (from the pair) demo source all fail to
  move the signature: **exactly one seed in three clears the bar in every wave,
  pooled picks 5-7/45.** The only variation that BROKE it was per-member LN
  (0/3, actively harmful). The only lever never varied is BUDGET — every wave
  stopped at 100k, and the sole result ever above 4/15 (dH_s0's 6/15 = 0.40)
  came at 150k. Per-seed picked-demo sets barely overlap across waves (hold-s2
  vs nb-s1 share 1 of 4 uids): WHICH demos become pickable is seed lottery,
  the COUNT is the config property. Morning decision (user): 200k x n>=5, or
  continuation from the three igniting checkpoints, or take the row as
  measured. Hold-pair merged experiment NOT launched — hold did not raise
  ignition, so it would re-measure the pair's floor at GPU-day cost.
  **[CORRECTED 2026-08-17, seed audit 2fbed2a: the pair-dH row is STRUCK — it
  was a literal re-execution of nb (demo-RNG hard-seeded 0), not an independent
  wave. The invariance is THREE-wave (nb, hold, mref): 3/9 independent dH
  seeds at the bar, pooled 5-7/45 per wave. Weaker than "four-for-four" but
  the conclusion stands on the three genuine waves; budget remains the
  never-varied lever. Later fresh-process evidence (cell-B clean demos, s1
  9/15, 08-16) exceeded the 4/15 ceiling this entry describes — see the
  handoff ignition section for the current table; this entry is historical.]**

- 2026-08-14 (assistant, newbox_supp): **RLPD 0.40 PROVENANCE.** The headline
  RLPD figure was NOT reproducible from its own record: rlpd_s0_confirm.log
  carries no action-mode line, no action_repeat line, no horizon, and no invoking
  script survives -- and for a delta_joint policy wandb_eval always prints the
  action_repeat line, so that run took a different code path. The number itself
  is CORRECT (reproduced exactly, episode-for-episode, by 15 fresh processes
  under a documented protocol). Rule adopted: eval logs must record
  mode/repeat/horizon/checkpoint and the invoking script must be kept.

- 2026-08-13 (peer session newbox_genesis, user-flagged; recorded here by
  newbox_supp): **P1 OPEN PROBLEM — THE DELTA-JOINT REPRESENTATION DOES NOT
  REPRODUCE THE DEMONSTRATED DOWNSTREAM PHASES.** All three learner arms
  (RLPD, dv3, r2dreamer) now train on delta_joint actions. But open-loop
  replay of the human demos THROUGH the delta representation loses the
  downstream outcomes those demos are labelled with. Canonical VELOCITY replay
  (collect_all_classified -> GenesisCanEnv), which produced the labels, gives
  contact 9 / nested 22. Delta replay through FullTaskEnv measures **contact 5
  / nested 4** at repeat-1 over the 72 resettable demos.
  **Localization (22 nested-labelled demos): 6 unresettable, 8 LOSE THE PICK
  ENTIRELY, 2 stop at picked, 5 reach placed, 1 reaches nested.** So the
  dominant failure is delta-replay FIDELITY — it fails to reproduce even the
  GRASP on half the resettable nested demos — not a downstream-predicate
  mismatch, which is what one would naively assume from the aggregate counts.
  Hypothesis tests: cap saturation REFUTED (pick-losers and pick-keepers both
  peak ~2x delta_cap and exceed cap on only 0.4-2% of frames — statistically
  identical). Length CORRELATES: pick-losers median ~2600 frames vs keepers
  ~1480. Leading diagnosis: open-loop delta integration accumulates tracking
  divergence over long/complex (multi-attempt, drag, regrasp) trajectories, so
  the arm drifts off the demonstrated path before the grasp. Velocity replay
  does not have this. Echoes the historical #26 env-vs-replay divergence theme.
  **CONSEQUENCES:** (1) the PICK phase — the core paper's scope — is
  UNAFFECTED, and every BC/RLPD number above stands; (2) **Phase 2 (place/slide
  specialists) and the stage-wise matrix are BLOCKED** until this is understood,
  because the demo buffer cannot carry an uncorrupted downstream reward/dynamics
  signal in the representation the learners consume; (3) the pick-gate "5/5" and
  "repeat 1..8 safe" results used the 5 GENTLEST hand-picked pick demos and must
  NOT be cited as full-task evidence. Options on the table: larger delta_cap /
  longer leash for the full task; closed-loop demo re-recording in the delta env
  (the derive_cartesian_realized pattern); or accept delta for pick-scope only
  and use velocity/absolute downstream. Detail: FABLE_HANDOFF_2026-08-13.md §8.

- 2026-08-13 (assistant, newbox_supp): **cross-check on the above — the
  downstream delta measurement disagrees with the labels in BOTH directions,
  not just by loss.** From the same sweep: repeat-1 env-measured nested = 4 over
  the resettable set, yet only 1 of the 22 nested-LABELLED demos actually
  reaches nested. So ~3 of the 4 env-measured nested successes come from demos
  the canonical velocity replay did NOT label nested. Combined with the sweep's
  own observation that 11-15 demos GAIN a stage under subsampling (coarser
  control should not improve outcomes), the downstream stage measurement is not
  merely lossy — it is high-variance in both directions on a 4-5 demo baseline.
  Practical rule: no contact/nested claim should be reported from delta
  open-loop replay at all until the fidelity issue is resolved, including
  claims that would FAVOUR us. The pick baseline (34 demos) is large enough to
  trust; the downstream baselines are not.
