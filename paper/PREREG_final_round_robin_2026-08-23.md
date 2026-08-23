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
