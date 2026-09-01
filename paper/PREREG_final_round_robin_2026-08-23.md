# PRE-REGISTRATION — final round robin (human vs model demonstrations)

Drafted 2026-08-23 on the monitoring box from the 08-22/23 audits
(paper/AUDIT_sources_2026-08-23.md, paper/CRITIQUE_design_final_rr_2026-08-23.md,
PAPER_PLAN decision log 2026-08-23). Status: **DRAFT for user sign-off; nothing below
is launched.** Once signed, this file is frozen for the block; amendments are appended
with a date, never edited in place.

User decisions already taken (08-23): one decision clock (repeat 4) for every learner
and every tape; retrain everything; run **sparse AND dense** for the RL and WM learners;
keep dv3 at a longer budget but **queue it last**; implement every confound protection
in CRITIQUE §4 except the TD-target clamp (kept out of the matrix; optional ablation).

---

## 0. The one-sentence design

Three demonstration sources (dH human, dDP diffusion-policy teacher, dR2D r2dreamer
teacher), **recorded by ONE recorder through the learners' own MDP**
(FullTaskEnv, pick scope, delta_joint, cap 0.025, leash 0.125, action_repeat 4, tip rule,
1200 sim-step horizon), matched in N and in IC coverage, success-only in the primary
matrix; consumed by four learners (DP, RLPD, r2dreamer, dv3) at that clock, evaluated by
ONE harness (fresh process per episode, 1200 sim steps, one shared IC file, K=5 archived
checkpoints, selection/confirmation split); RL/WM learners run sparse and dense.

Notation unchanged: d{source}_{learner}[-dense], e.g. dR2D_RLPD-dense_s3.

---

## 1. Matrix

| learner | sources | seeds | reward | budget (UNIT) | secondary arms |
|---|---|---|---|---|---|
| DP | dH, dDP, dR2D | 5 | — | 100k grad steps AND 300k (epoch-matched view; see §6) | dHunpruned_DP (preprocessing control) ×3 |
| RLPD | dH, dDP, dR2D | 10 | sparse **and** dense | 100k DECISIONS (= 400k sim steps) | dDP+fails, dR2D+DPfails ×6 (sparse; N1 mechanism arms) |
| r2dreamer | dH, dDP, dR2D | 6 | sparse **and** dense | 3M SIM steps (750k decisions) | — |
| dv3 (queued LAST) | dH, dDP, dR2D | 4 | sparse **and** dense | 1M SIM steps (250k decisions = 62.5k updates at train_ratio 256) | — |

- Seed indices shared across sources within a learner (paired blocks); new seed values
  never used before for that learner (disjoint from RUN_REGISTRY history).
- Queue order: pilots → DP → RLPD → r2dreamer → RLPD secondary → dv3.
- Job count: DP 15(+3) · RLPD 60(+12) · r2d 36 · dv3 24 = 150 jobs; GPU-h ≈ DP 54 +
  RLPD 216 + r2d 576 + dv3 ~400 ≈ 1250 (r2d and dv3 dominate; dv3 last by construction).
  Cut order if forced: dv3 seeds 4→3; r2d 6→5 (not lower); RLPD secondary 6→4; DP 300k view.

## 2. Held fixed (all learners, all sources, all arms)

| knob | value | where enforced |
|---|---|---|
| decision clock | action_repeat 4 | env ctor arg + tape stamp + sidecar + registry knob |
| action space | delta_joint, delta_ref=target, cap 0.025 rad/sim-step, leash 0.125 | FullTaskEnv; DP emits absolute window-end targets and is *executed* through the same integrator at eval (hold-4) |
| training horizon | 1200 sim steps = 300 decisions | env max_steps; r2d/dv3 time_limit 1200 (pilot-gated) |
| terminal | hardened pick (+1; ×100 for WMs), tip rule (tilt>60° ∧ grip<0.3 → terminate, r=0), timeout = truncation | FullTaskEnv online; `terminal_from_tape` offline (§4.3) |
| dense variant | r + γφ(s') − φ(s), φ = −2‖eef−can‖, **φ(terminal)=0**, γ = the learner's discount (RLPD 0.998, dv3 0.997, r2d = its config value, VERIFIED), demo transitions relabelled with the same potential from recorded eef_pos | full_env.step; encoders |
| demo preprocessing | identical for all sources: recorded-as-executed tapes (§4), success-only primary, fails only in the named secondary arms, tape ends where the env terminated | recorder + set builder |
| eval | §5 | cluster/eval_sweep.sh + eval_ics.json |
| code | one commit hash for the block; a fix restarts that learner's block | registry |

## 3. Demo-set protocol

### 3.1 Sources
- **dH**: the 66 IL-usable success demos (PAPER_PLAN §3). Leading-idle pruning applied
  ONCE (make_dp_pruned rule) before re-recording — the pruned set is the primary for all
  learners; the unpruned set is the dHunpruned_DP control only.
  Recorded by the human-waypoint adapter (§4.2) = closed-loop follower of the demo's own
  command stream at repeat 4 in the target-ref delta env. Kept iff it re-earns the
  hardened pick (replay gate). Report survivors /66.
- **dDP**: teacher = the DP trained on dH in THIS block's DP-r4 pilot (the median in-dist
  seed of the new dH_DP row — rule fixed now, not the best seed). Recorded by the dp
  adapter (hold-4, absolute targets → delta via the env integrator).
- **dR2D**: teacher = r2dreamer CHAMPION_1576820.pt (fixed). Recorded by the r2d adapter.
- Both teachers: `--mode sample`, attempts ≤3 per IC, IC list = the same success uids
  cycled, sim cap 1200, `--verify`, random-teacher negative control (must keep ≈0),
  ALL fails kept to `<set>_fails/`, manifest records teacher success rate on the harvest ICs.
- Teacher circularity and competence are disclosure (both teachers descend from dH).

### 3.2 Matching
N = min(dH survivors, dDP successes, dR2D successes), capped at 66; larger sets are
subsampled uniformly (fixed seed) **stratified by ic_uid** so the three sets cover the
same uids with the same multiplicity. Frames are NOT matched (intrinsic; reported).
Per-set census (analysis/characterize_demo_sets.py, incl. the stride table) is published
BEFORE launch; the per-episode list is sha256'd and every sbatch asserts the sha.

### 3.3 Fails arms (RLPD sparse only)
dDP+fails = the N dDP successes + dDP fails (tip-terminated by the recorder, guard on),
fail share capped at the human no-pick share (~30% of episodes); dR2D+DPfails = the same
fails injected into dR2D. Pre-registered predictions in PAPER_NOTES N1 carry over
(ignition of dDP success-only ≥ dH; adding DP fails to dR2D lowers it by ≥0.15).

## 4. THE TAPE CONTRACT (v1) — what the recorder writes, what every consumer reads

One recorder, `baselines/record_demos.py` (NEW), teacher adapters `human | dp | r2d |
random`, env = `FullTaskEnv(scope='pick', action_mode='delta_joint', delta_ref='target',
delta_cap=0.025, delta_leash=0.125, action_repeat=4, max_steps=1200, camera_rig=True)`
(CPU). One npz per episode, filename = rollout index ≥100000 (never a human uid); keys:

| key | shape | meaning |
|---|---|---|
| `states` | (n,17) f32 | obs BEFORE each decision (joints 6, grip, grip effort, can xyz, can quat wxyz, goal xy) |
| `final_state` | (17,) f32 | obs after the last decision |
| `actions_delta` | (n,7) f32 | the normalized [-1,1]^7 decision the env EXECUTED (arm deltas, grip) — the learners' action space, exact |
| `actions` | (n,7) f32 | absolute command at the window END ([6 joint targets rad, grip 0..1]) — DP/jact and legacy consumers |
| `rewards` | (n,) f32 | env reward per decision (sparse; shaping is NOT baked in) |
| `terminated` | (n,) bool | env terminated flag per decision |
| `truncated` | (n,) bool | env truncated flag (timeout) |
| `picked` | (n,) bool | env hardened-pick flag per decision |
| `tipped` | (n,) bool | env tip flag per decision |
| `eef_pos` | (n+1,3) f32 | tool position before each decision + final (for demo shaping / offline hardened predicate) |
| `images` | (n+1,64,64,6) u8 | top ++ wrist RGB before each decision + final (camera rig; required for WM sets) |
| `sim_states`, `sim_actions` | (4n,17), (4n,7) f32 | per-sim-step sub-tape (states before each sim step; absolute command each sim step) — lets any stride ≤4 be derived later; optional but recorded by default |
| scalars | | `uid` (rollout idx), `ic_uid`, `label` ('success'|'fail'), `stage`, `teacher`, `teacher_ckpt`, `act_mode` (mode|sample), `action_repeat`=4, `delta_cap`, `delta_leash`, `delta_ref`='target', `pick_z`, `n`, `git_sha`, `env_class`, `recorder`='record_demos.py v1', `contract`='v1' |

Rules: the tape ENDS at the env's terminal (terminated=True on the last row) or at the
cap (truncated=True on the last row) — never later. `label`='success' iff the last row
has picked=True. The recorder asserts `terminated|truncated` on exactly the last row.
Directory gets `manifest.json` (teacher, ICs, attempts, kept/dropped, negctl result,
content sha256, git sha, contract version).

### 4.1 Consumers (no consumer re-derives anything the tape already carries)
- **RLPD** `--demo-format native`: transitions (states[t], actions_delta[t], rewards[t]
  (+shaping from eef_pos when dense), next = states[t+1] or final_state, done =
  terminated[t]); truncation bootstraps. No delta re-encoding, no relabel predicate.
- **DP**: lerobot dataset at fps 30/4 = 7.5 from (states, actions) per decision; eval and
  harvest execute DP through the same hold-4 integrator.
- **WM** (r2d/dv3): dreamer npz written DIRECTLY from the tape (`to_dreamer_native.py`,
  NEW): image[t], action[t] (backward-shifted, zeros at 0), reward ×100 at the terminal
  decision, is_terminal at terminated, is_last at the end, discount = 1−is_terminal; no
  grant slack; repeat.json stamp `action_repeat 4, contract v1`.
- **Census**: characterize_demo_sets.py reads contract-v1 tapes natively (stride table
  becomes exact-by-construction, reported anyway).

### 4.2 Adapters (inside record_demos.py)
- `human`: waypoint follower on the recorded absolute command stream of the human demo
  (rerecord_delta_demos.py logic generalized to delta_ref=target): per decision
  a_arm = clip((cmd_j − target_now)/(4·cap), −1, 1), a_grip = grip_j·2−1; advance j by
  arrival (‖q_meas − ref_j‖∞ < tol) up to 4 waypoints per decision, or dwell cap; time
  dilation counted; settle; kept iff hardened pick re-earned.
- `dp`: lerobot policy (dp_runner) queried once per decision on the current obs;
  output absolute target q* → a_arm = clip((q* − target_now)/(4·cap)); grip from the
  chunk; `--mode sample|mode`.
- `r2d`: champion loader lifted from harvest_champion_demos.py; native delta actions.
- `random`: uniform [-1,1]^7 per decision (negative control, must keep ≈0).

### 4.3 Terminal unification (offline)
`full_env.terminal_from_tape(tape) → (t_term, kind, reward)`: for contract-v1 tapes it
READS `terminated/picked/tipped`; for legacy stride-1 tapes it computes the relabeler
pick predicate + the tip rule and WARNS that the hardened eef distance is unavailable.
Every legacy encoder/converter (train_sacfd_full delta encoders, to_dreamer_demos.py,
convert_genesis_demos_repeat.py, make_dDPsucc tiptrunc) gets `--demo-terminal-guard`
DEFAULT ON: tip → done=True r=0 and drop later frames; pick → done=True; cap → bootstrap.
Unit tests on synthetic tapes: tipped tape ends done=True r=0; pick ends done=True;
timeout ends done=False; contract-v1 tapes pass through unchanged.

## 5. Evaluation protocol

- **Harness**: `cluster/eval_sweep.sh` (NEW) drives `baselines/wandb_eval.py` ONE episode
  per fresh process, CPU, ≤1 world per 2 cores, `--max-steps 1200`, repeat/action-mode
  from the checkpoint sidecar (no silent defaults), for sac AND dp (dp gets the hold-4
  path). r2d/dv3 evals go through their own eval scripts with the SAME IC file, horizon
  and fresh-process rule (patches shipped from here; cluster-side apply).
- **ICs**: `baselines/eval_ics.json` (NEW, generated once, committed): `sel` = 15 demo
  uids (selection), `hold` = 15 OTHER success uids (confirmation), `rnd` = 30 support ICs
  drawn once from rng(0). Same file for every learner/source/arm.
- **Checkpoints**: K=5 archived at 20/40/60/80/100% of budget for every run (RLPD
  callback; DP save_freq; r2d/dv3 archive step). Selection = best `sel` score;
  **headline per seed = the selected checkpoint on `hold` (15) + `rnd` (30)**; also
  report final-checkpoint on hold+rnd and time-to-first-training-pick.
- **Action selection**: RLPD deterministic; DP sampled (seeded); r2d sampled (+mode
  reported); dv3 deterministic — recorded in the result JSON. Denominators asserted;
  missing cells reported missing, never 0. Node class recorded per eval process.

## 6. Hypotheses (falsifiable; one primary statistic each)

Primary statistic = per-seed success count on `hold` (15) and on `rnd` (30) of the
selected checkpoint; unit = seed; paired by seed index.
- **P-DP**: mean per-seed hold success differs between dH and each model set (2 contrasts,
  Holm). Prediction (from prior data, to be re-tested): dR2D > dDP > dH on hold; dR2D >
  {dDP, dH} on rnd. Falsified if the paired-difference CI covers 0. Epoch check: if the
  300k-step view closes the dH gap, the 100k gap is attributed to under-training.
- **P-RLPD (sparse)**: success-only dDP ignites ≥ dH (N1); falsified if dDP remains below
  dH by ≥0.15 with CI excluding 0. **P-RLPD (dense)**: source ordering dR2D ≥ dH ≥ dDP on
  hold mean; dense raises every source's mean vs sparse (paired by seed).
- **P-R2D**: sparse — ignition lottery persists on all sources, dR2D pixel set ignites
  least (team's registered expectation); dense — 4/4-class ignition on dH replicates and
  extends to dDP/dR2D (falsifier: dR2D-dense ≥ dH-dense breaks "model pixel sets don't
  enter the basin").
- **P-MECH** (secondary): adding DP fails to dR2D lowers RLPD-sparse success by ≥0.15;
  removing them from dDP raises it to within 0.1 of dH.
- **P-DV3** (exploratory, last in queue): at 62.5k updates ≥1 seed per source sustains
  nonzero hold success at its last two archived checkpoints.
- **H4'** (exploratory): the SIGN of (model − human) differs between DP and r2dreamer;
  labelled exploratory because modality and budget differ across learners (CRITIQUE N2).

## 7. Analysis plan
Paired permutation test on seed means within learner; hierarchical Beta-Binomial
P(A>B) (analysis/bayes_source_effect.py) as the reported posterior; 95% bootstrap CI on
the mean difference as effect size. Two primary contrasts per learner → Holm; everything
else BH q=0.1 and labelled exploratory. Ignition counts descriptive only. No pooling of
episodes across seeds. Per cell report: n seeds, per-seed sel/hold/rnd, mean ± SD, CI,
posterior, selected ckpt step, final-ckpt rate, time-to-first-pick, demo sha,
N episodes/frames/rewarded frames, horizon/repeat/action mode, node class, video grid.

## 8. Pilots (positive controls at the new clock) — pass bars fixed now
| pilot | runs | pass bar | on fail |
|---|---|---|---|
| dH re-record at repeat 4 (recorder, human adapter) | all 66 | ≥50 survivors re-earn the hardened pick | lower tol / investigate; if <45, fallback: DP at stride 1 on the same episodes (per-learner clock) |
| DP-r4 on dH | 2 seeds | ≥0.5 in-dist on `sel` (prior 0.62) | DP stays stride 1 on the same episodes; amendment logged |
| RLPD-r4 dH dense | 3 seeds | pooled hold ≥0.16 and ≥1 seed ≥4/15 (08-19 dense verdict) | keep repeat 1 for RLPD; amendment |
| r2d dense dH at time_limit 1200 | 2 seeds | ≥1 seed ignites by 1M with best-ckpt ≥0.5 | keep 400; amendment |
| dp-adapter hold-4 harvest yield | 30 rollouts | ≥50% of the teacher's stride-1 yield | switch teacher or stride; amendment |
| random-teacher negctl | 30 | keeps ≈0 (≤1) | recorder keep predicate broken — stop |

## 9. Build list (what is being implemented now, from this doc)
B1 `baselines/record_demos.py` + adapters, contract v1, manifest, negctl. (genesis-side;
   runs on devbox/cluster)
B2 `full_env.terminal_from_tape`; `--demo-terminal-guard` default-on in every legacy
   encoder/converter; φ(terminal)=0 in full_env.step; RLPD `--demo-format native` with
   demo shaping from eef_pos; unit tests on synthetic tapes.
B3 `baselines/eval_ics.json` generator; wandb_eval `--ic-file` + DP hold-N; `cluster/
   eval_sweep.sh`; K=5 checkpoint archiving (train_rlpd callback, sbatch_dp save_freq);
   sbatch knobs (horizon, repeat, budget unit, demo sha), node class, CPU evals, registry
   at job start.
B4 `baselines/make_matched_sets.py` (N=min, stratified by ic_uid, sha, manifest, per-
   learner derivations: npz/lerobot fps 7.5/dreamer-native) + `to_dreamer_native.py` +
   census reading contract v1; patch file for dreamerv3-torch-genesis (converter slack,
   eval IC file, time_limit, archive).
B5 cluster-side TODO for the user (cannot be done from this box): pull 08-20 artifacts;
   verify r2dreamer discount in its config; r2d time_limit/archive/eval patches.

## 10. Disclosure paragraph (write now, spend no GPU)
State vs pixel modality; per-learner budgets and units; human 30 Hz tapes only ~half
exactly representable at repeat 4 before re-recording (census) — after re-recording,
representability is exact by construction for all sources and the human set's time
dilation is reported; teacher circularity and competence; recipes tuned on dH; eval env
has no tip termination; RLPD has no passing positive control; dv3 is last-in-queue and
exploratory; single task, single simulator, N≈66; the 08-19 dDP_RLPD 0/6 is the
motivation for the terminal guard and the recorder, not a result about model demos.

---
## Amendments (appended, dated; never edited in place)
- **A1 (2026-08-23, pre-launch).** dH N = 51 of 66 (recorder: 15 human demos not executable inside
  the MDP — 2 lying-can ICs, 1 knock-over, 5 over-horizon, 7 over-cap; deterministic). Reported as
  a source property; no horizon stretch for the human adapter (§2 horizon held fixed).
- **A2 (2026-08-23, pre-launch, conditional).** dR2D teacher = r2dreamer round-robin dH-dense s51
  `BEST_selected.pt` (cluster) if CHAMPION_1576820.pt is not available before the matched sets are
  built; same learner family, trained on the same human set, confirmed ~1.0 on sel. Logged in the
  set manifest (teacher_ckpt) and in the census.
- **A3 (2026-08-23).** `--ic-from-tape`: the 12 demos without a recovered placement reset to the can
  pose recorded at frame 0 of their own tape; applied identically to all three sources so every
  source records from the same 66 ICs.

## 11. Independent-audit schedule (locked by user 2026-08-23)
Milestone-gated, not calendar-driven. Each audit = one fresh-context agent (no conversation
history; repo + docs only), read-only, must cite file:line, report archived under paper/AUDIT_*.
| gate | audit | must answer |
|---|---|---|
| before any block launches (after sets + census exist) | **Pre-launch**: PREREG vs code vs manifests | every sbatch implements §2/§5 (clock, horizon, harness, registry knobs); set shas/census match §3.2; no silent default reappeared |
| first results readout of each learner family | **Results**: numbers vs logs vs protocol | headline = selected-on-hold+rnd; denominators, missing cells, node classes, seeds disjoint; no cross-learner sentence without its caveat |
| before a claim enters the paper text | **Claim** (adversarial): the claim, its evidence, its falsifier | what a reviewer rejects it on; cheapest experiment that would kill it |
| after any trainer/env/encoder change mid-block | **Change** (narrow): diff + tests | does it restart that learner's block (§2 "one commit hash")? what else reads the changed path? |
| once, before submission | **Reproducibility**: a stranger reruns one cell from repo + rsynced sets | runbook, shas, registry, one cell re-executed from scratch |
Next due: Pre-launch audit, 2026-08-24, after Phase 4 of the session runbook and before the DP/RLPD block.
- **A4 (2026-08-23, pre-launch audit B2).** Policy device: the DP policy runs on the GPU when one is visible
  (eval_sweep.sh dp path, recorder dp adapter, dDP recording jobs with GRES=gpu:1); the SIM is CPU always and
  SAC policies stay CPU. §5's "CPU" wording referred to the sim.
- **A5 (2026-08-23).** §8 pilots as actually run: random-teacher negctl n=3 (0 kept) not 30; the dp-adapter
  yield pilot was skipped — the full dDP harvest's first-attempt rate (0.55-1.0 per shard, 64/66 ICs) stands
  in; the RLPD-r4 dH dense pilot was NOT run separately — a 3k-step dense smoke (W1) plus the block's dH
  dense seeds 10-13 double as it; if dense RLPD at repeat 4 fails its bar the RLPD block restarts at repeat 1.
- **A6 (2026-08-23).** §4.2 human adapter arrival test: `--arrival either` (advance when the measured joints
  reach the human's measured pose OR the env's integrated target reaches the human's command; lab result
  paper/real2sim_follower_lab_2026-08-23.md §3: 49→55 local, zero dwell stalls, same path fidelity). If its
  cluster confirmation keeps ≥55, the block's dH is the `either` set, built into a NEW root (matched_v2;
  matched_v1 is never rebuilt in place). The dDP teacher (DP-r4 pilot s0 ckpt 020000) was trained on the 51
  `meas`-arrival tapes (sha 58fd89bc) — a superset/variant relation to the block's dH, disclosed.
- **A7 (2026-08-23).** Checkpoint selection ties → the LATER checkpoint (sbatch_rlpd.sh, dp_select_confirm.sh).
- **A8 (2026-08-23, audit W8).** `hold` is a CONFIRMATION split of demo ICs (11/15 are training ICs for a
  51-demo set; for RLPD every sel/hold uid is a training-reset IC), not a held-out set; only `rnd` (30 support
  ICs) is novel. Report "sel / hold / rnd" by name; never call hold "held-out".

### A9 (2026-08-28, from AUDIT_results_2026-08-28 §2) — run identity
A seed id names exactly one training run. A relaunch of an (arm, seed) that has already produced a
readout REPLACES it: the earlier readouts are dropped from every pooled statistic and listed as lost.
Relaunches must therefore use fresh seed ids; in-place reruns of completed seeds are forbidden.
Retroactive: dR2DDPfails s83 (completed 08-26, sel 0.80) was overwritten 08-27 and is lost; dH s102's
in-place relaunch was cancelled 08-28 and resubmitted as s108; dDP s101's relaunch never ran.
`analysis/results_table.py` dedups on (arm, seed). Registry rows must carry a `warn` field when a
semantic duplicate is accepted.
### A10 (2026-08-28) — N15 controls (N16)
Two success-content controls bound (do not remove) the length/volume confound of the fails arm:
dR2Ddup13 (share-matched) and dR2DDPsucc (8 longest dDP successes). Predictions in PAPER_NOTES N16.

### A11 (2026-08-28) — ONE primary world for the learner x source matrix
The learner x source question (1a/1b) is answered in the OLD world (`base`, `matched_v2`): it is the only
world in which all three sources exist for every learner, and every fails arm and every world-model run
lives there. The corrected world (`gc_kp4_riser3_shelf6`, `matched_w3`) is a robustness replicate for
dH/dDP under DP and RLPD, and the sim-fidelity work is reported as its own contribution. Re-teachering
dR2D in the corrected world and re-running the matrix (~1 week of the 10-GPU cap) is out of scope for the
deadline; the corrected-world sets are kept. Consequences applied 08-28: DP old-world top-up seeds 15-19
(n=5 -> 10 per source); RLPD old-world sparse top-up seeds 14-19 (n=4 -> 10); old-world split halves
dH/dDP/dR2D _A/_B x seeds 50-52 (N14 in the primary world; the matched_w3 halves keep running as the
replicate). Disclosure: the old world carries the three physics defects (no arm gravity compensation,
buried shelf, soft contact) -- identical for every learner and source, and inert for the shelf in pick
scope.

### A12 (2026-08-28) — matrix simplified to human vs DP demonstrations
USER: "If DP does well, we can simplify to human vs DP demos until we're satisfied that everything is
working correctly." DP does (rnd 0.5-0.6, hold ~0.9, stable across worlds). PRIMARY DESIGN from here:
{dH, dDP} x {success-only, + the source's OWN fails} x {DP, RLPD, r2dreamer, dv3}, old world (A11).
dR2D arms are RETAINED where already paid (old-world DP n=5, RLPD n=4, r2dreamer 80-91, N15/N16
mechanism cells) and reported as a secondary lineage note, not a matrix row. Cancelled 08-28 before
start: DP dR2D top-ups 15-19 and dR2D split halves; RLPD dR2D top-up 14-19 and dR2DR2Dfails; r2dreamer
sparse dR2D 80-83, dR2DR2Dfails 80-83, dR2Ddup13 200/201 (the packed pilot 202/203 kept as the packing
test). Launched: r2dreamer dDPfails 80-83 (the WM 2x2 was missing its dDP+DPfails cell). The
`dR2DR2Dfails` set (built, sha 248d0ade) is kept on disk for a later lineage appendix.

### A13 (2026-08-28) — old vs corrected world: decide by the WM ignition gate, not in the abstract
USER: the ecological-validity question is old sim vs corrected sim. With dR2D dropped (A12) the corrected
world (gc_kp4_riser3_shelf6, matched_w3) can carry the full 2x2x4: DP and RLPD dH/dDP success-only cells
are already n=10 there. Missing and now queued: corrected same-source fails arms (matched_w3/dHHfails from
dH_w2_fails, matched_w3/dDPfails from dDP_w2_fails) for RLPD sparse x seeds 20-23 (8), and the WM
IGNITION GATE r2dreamer dense dH/dDP x seeds 80-83 on matched_w3 native tapes, TL 400 (8). RULE: if the
gate ignites (>= 2/4 seeds per arm above 0.3 on the protocol readouts), the corrected world becomes the
PRIMARY world for the matrix on validity grounds and the WM fails arms are run there too; the old world
becomes the replicate. If it does not ignite, that is reported as a result and A11 stands. Known cost
either way: RLPD sits near the floor (~0.2) in the corrected world, so the old world remains the
sensitive readout for RLPD's source contrast.

### A14 (2026-08-28) — learner health before source comparisons
USER: "the top priority is ensuring RLPD and WM runs are behaving appropriately before we worry about human
vs machine demos." Queue reprioritised: all human-vs-machine power jobs HELD (35 DP top-ups/halves/N7,
29 r2d seed top-ups/sparse/fails arms, 24 old-recipe RLPD); runnable = the diagnostics only (18 RLPD fix
factorial N18, 9 dv3 baseline+clamp/fix, 4 dv3 touchgoal, 2 r2dreamer touchgoal, 8 corrected-world WM
gate, 1 packing pilot). A health audit of r2dreamer (critic pinned at the clamp? checkpoint bistability?
selection bias?) runs in parallel. Held jobs are released only after each learner's recipe is judged
healthy; if a recipe changes, the held jobs are cancelled and resubmitted under the new recipe.

### A15 (2026-08-28) — validity rule
USER: "Don't do anything that would cause our implementations to diverge significantly from the standard
versions (we want validity most of all)." Applied: (i) RLPD fixes may only move the recipe TOWARD Ball et
al. 2023 (gamma 0.99 is the paper's value; our 0.998 was the deviation); (ii) r2dreamer's return clamp and
replay-critic loss are DEVIATIONS from DreamerV3 -- if the health audit finds them load-bearing, a
standard arm (clamp off, repval off) is run and reported, not hidden; (iii) the dv3 `return_clamp` overlay
is diagnostic only; the by-the-book dv3 arm is `genesis_dv3std` (reward_scale 1 with symlog/two-hot heads,
no clamp, stock train_ratio) on terminal-reward-1 native demos -- launched dH dense x seeds 20-22.

### A16 (2026-08-29 06:30, BEFORE any g99 s33-37 readout) — RLPD source statistic
The RLPD human-vs-DP contrast is scored on (i) **divergence rate** per arm (a run diverges if max critic_loss ≥ 1 or
mean actor-state Q exceeds the attainable return on a sparse arm at any logged step after 30k; Fisher exact, two-sided)
and (ii) **rnd-30 success of the LAST checkpoint** (not the selected one; Welch + exact permutation as in analysis/stats.py).
hold is reported but is not the statistic: dH sits at its ceiling (14/15 in 6/6 runs) and hold ICs overlap training ICs (A8).
The Q-watchdog trip count is NOT a health criterion on dense arms (potential shaping legitimately lifts Q above 2.0;
train_rlpd.py:274-276 rescales the threshold only for hold reward); dense arms use max critic_loss and final≈selected only.
Predictions: divergence dH 0-1/8 vs dDP ≥3/8; LAST-ckpt rnd dH > dDP by ≥0.10.

### A17 (2026-08-29) — RLPD block restarted at γ = 0.99
PREREG §2 listed γ 0.998 (a deviation from Ball et al. 2023). Every γ 0.998 run (seeds 10-19, 24-29, waves final/w2final)
is superseded and diagnostic-only (E9). The block is re-run at γ 0.99, UTD 10, seeds 30-37 (fresh ids, A9).
Dense arms relabel demo rows with the run's γ at load (train_rlpd.py:311), so γ 0.998 and γ 0.99 dense runs may never share a table.
Same-source row-matched controls added: dHsucc_dup / dDPsucc_dup (56 successes + the 8 longest own successes duplicated,
6xxxxx stems; added rows 1755 / 1521 vs the fails arms' 2145 fail rows), RLPD sparse seeds 30-33. Reading rule for (1b):
the fails effect is claimed only if +fails differs from +succ_dup in the same direction on both sources.

### A18 (2026-08-29) — world-model fallback
If neither dv3 (standard recipe: genesis_dv3std) nor r2dreamer NOCLAMP (return clamp off, replay critic loss kept — the
official DreamerV3 v2 component) shows a sustained pick with a bounded value estimate by ~1M sim steps, the WM arm becomes
MoDem as published (via the DEMO3 codebase), including its BC-initialised actor; the actor-BC confound is disclosed and
H4 is reworded to "world-model learners that back up value through demonstration trajectories". First dv3-std picks were
logged at 284k (s20) on 08-29 05:40 — before this amendment; the 1M/3M readouts decide.

### A19 (2026-08-29) — one world per learner table
A13's corrected-world WM gate is not load-bearing: the WM arm reports in whichever world it is healthy in (old world,
where every WM run to date lives), stated in the table header; no table mixes worlds. The DP corrected-world result and
the RLPD/WM old-world results are separate tables with the world named.

### A20 (2026-08-31, BEFORE any readout of these runs) — RLPD corrected-world replication (user request)
User (08-31): the cross-learner story needs like-to-like worlds — "run RLPD on the corrected world (or DP on the old
world)". DP already has both worlds (corrected n=10, old n=5); RLPD is the only headline learner without a
corrected-world cell at the valid recipe (the w2final corrected-world block, seeds 20-29, ran at γ 0.998 and is
diagnostic-only per A17 — it never enters a table). New block: RLPD sparse, γ 0.99, UTD 10, corrected world
(SIM_VARIANT=gc_kp4_riser3_shelf6, DEMO_ROOT=baselines/matched_w3, N=58 per arm, matched ICs), arms dH vs dDP,
fresh seeds 40-47 (A9), wave g99w3. Statistic identical to A16: LAST-checkpoint rnd-30 (Welch + exact permutation)
plus divergence rate (max critic loss ≥ 1, Fisher exact). Predictions (registered before submission): the old-world
result replicates in direction — LAST rnd dH > dDP by ≥ 0.10; divergence dH ≤ 2/8, dDP ≥ 3/8. Note the demo sets
differ from the old-world block in world and N (58 vs 56) by construction; the comparison is within-world per A19,
and this block exists to let the cross-learner figure cite two learners in the SAME world, not to pool worlds.

### A21 (2026-09-01, BEFORE any v2 collection readout) — full-pool v2 block (user request: use the recovered demos)
The 08-31 recovery (DEMO_RECOVERY_RESULTS) solved 74/75 successes, adding 7 nested demos and the arm-base edge
cluster the frozen sets undersample. v2 sets: **fresh collection at the current validated placements** (kills the
2 cm training-IC drift), both worlds, all clean solved trials (excluded with reasons: 303 no-video/unsolved,
237 camera-convicted control-limited, 294 basin-flip pending) → target N≈72/arm. dHv2 = the human tapes;
dDPv2 = per-IC matched harvest from a DP teacher trained on that world's dHv2 (same recipe as the frozen sets;
make_matched_sets tooling). Learners: DP (n=5/arm/world, headline = selected ckpt hold+rnd) and RLPD (n=8/arm/world,
γ 0.99 sparse, statistic = A16: LAST-ckpt rnd-30 + divergence max CL ≥ 1). Fresh seed ids (A9): DP 50-54, RLPD 50-57.
**Registered predictions:** (i) old-world RLPD gap replicates at v2, dHv2 > dDPv2 by ≥ 0.10; (ii) corrected-world
RLPD stays null, |diff| < 0.10; (iii) DP indifferent in both worlds (|diff| ≤ 0.06 band). The frozen-set results
(A16/A20) remain the results of record for the 56-IC sets; v2 is an additive robustness block, not a correction.

### A22 (2026-09-01, amends A21 BEFORE any v2 readout) — per-seed train/test splits (user request)
v2 evaluation replaces the static hold-15 (⊂ training ICs, the A8 weakness) with SEED-SPECIFIC demo-IC holdouts:
for seed k, a deterministic split (fixed RNG on k; tooling committed) reserves ~15 of the ~72 demo ICs as that
seed's test set; the learner trains on the remaining ~57 ICs' tapes only. Rules that make this a blocking factor,
not a confound: (i) split-k is IDENTICAL across arms (dHv2/dDPv2), learners (DP/RLPD), and worlds — every contrast
at seed k is on the same train and test ICs; (ii) checkpoint selection uses train-IC episodes only — each seed's
test ICs are scored exactly once, by the selected (DP) / LAST (RLPD, per A16) checkpoint; (iii) the FIXED rnd-30
remains the common generalization statistic across all seeds and the axis comparable to the frozen-set blocks.
Statistics: paired-by-seed differences (arm A − arm B at the same split), Welch + exact permutation as before;
divergence rate unchanged. Teacher note (disclosed, not hidden): one DP teacher per world trains on the FULL dHv2
pool, so the machine-demo GENERATOR has seen all ICs — symmetric with the human demonstrator, who performed every
trial; students in both arms see train-IC tapes only. A21's registered predictions carry over with "hold" read as
"per-seed held-out demo ICs".

### A23 (2026-09-01, ~1 h after A22, BEFORE any v2 readout) — A22 WITHDRAWN; v2 trains on ALL ICs (user decision)
A22's per-seed demo-IC holdouts are withdrawn before any use: the generalization claim rides on the FIXED rnd-30
(random ICs, off-demo-support), so holding out demo ICs bought rigor on an axis the paper does not claim, at the
cost of shrinking exactly the coverage the v2 pool exists to add. v2 protocol of record: train on ALL ~72 demo ICs;
checkpoint selection on a fixed 15-IC training subset (as the frozen blocks); final checkpoint scored ONCE on
(a) ALL training ICs — reported strictly as IN-DISTRIBUTION, never as generalization — and (b) the fixed rnd-30
(the headline statistic, unchanged, comparable to A16/A20). RLPD LAST-ckpt rule and divergence statistic unchanged.
A21's registered predictions stand with "hold" read as "full training-IC in-distribution score". No v2 data was
collected, split, trained, or read under A22 between its registration and this withdrawal.
