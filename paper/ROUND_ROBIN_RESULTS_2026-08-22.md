# Round robin results — every non-smoke run started since Wed 2026-08-19 (wandb pull 2026-08-22)

Written from the monitoring box (no cluster access; wandb entity `jambotime` only).
Naming: a run/cell is **d{demo_source}_{algorithm}** — e.g. `dHpruned_DP` = diffusion policy
trained on the pruned human demos; sources dH / dHpruned / dDP / dR2D, algorithms DP / RLPD /
r2dreamer (R2D) / dv3 (DV3). Figures follow `paper/figs/STYLE_RULE.md`: marker SHAPE = demo
source (dH ○, dDP □, dR2D △), COLOUR = algorithm (DP green, RLPD blue, r2dreamer red, dv3 purple);
hatched cell / dashed line = dense (potential-shaped) reward variant.

Scripts: `analysis/pull_rr_wandb_{detail,curves}_20260822.py` (pull), `analysis/make_rr_figs_20260822.py`
(figs), numbers in `paper/figs/rr_numbers_20260822.json`. Figures:
`paper/figs/rr_headline_per_seed_20260822.png`, `rr_training_curves_20260822.png`,
`rr_r2d_dense_dH_seeds_20260822.png`.

## Status: the whole 33-job block is DONE
Every wandb run created since 08-19 is `finished` (last activity 08-20 22:45 UTC); nothing is
still running. 33/33 trained; 31 evaluated per protocol; 2 lack an eval (r2d dR2D s43, dv3 dR2D s0).
Non-smoke runs seen since 08-19 that are NOT part of the cluster block: the two local
dense-verdict retrains `dH_RLPD-dense_s{0,2}` (crashed at ~60k at the 08-19 power loss, re-run to
100k — numbers of record are the sweep JSONs in `paper/dense_verdict_2026-08-19/`, already closed as
PASS) and the local dv3 shaped live-fire `dv3_dH_shaped-joint` (**failed at 02:46 UTC 08-20, 0 rows**;
the cluster shaped dv3 runs ran fine, so this was confirmatory only — still unverified locally).
Smoke runs (5k/3k-step, 20k dv3) excluded.

## Headline table (per seed; protocol per algorithm)
| cell | seeds | headline per seed | mean | ignited | protocol |
|---|---|---|---|---|---|
| dDP_RLPD (core) | 0-5 | demo-IC 1,0,1,0,0,0 /15 | 0.02 | **0/6** (bar ≥3/15) | fresh-process sweep; random-IC 3,1,1,0,4,0 /15 → 0.10 |
| dH_RLPD dense | 3-8 | demo-IC 1,6,3,6,0,9 /15 | 0.28 | **4/6** | same sweep; random-IC 2,4,2,5,0,5 /15 → 0.20 |
| dR2D_DP (core) | 0-2 | in-dist 0.93, 0.93, 1.00 | **0.96** | n/a (BC) | wandb_eval --ic-mode both, 15+15; random 0.73, 0.80, 0.73 → 0.76 |
| dR2D_R2D (core) | 40-43 | best-ckpt confirm 0.00, 0.00, 0.00, (s43 no eval) | 0.00 | **0/3** (+1 partial) | x3 confirm @ best ckpt; s40-42 all 30 in-train evals 0, zero training picks |
| dH_R2D dense | 50-53 | best-ckpt confirm 0.91, 0.98, 0.96, 0.76 | **0.90** | **4/4** (bar ≥0.20) | best ckpt @ 0.67M / 2.97M / 2.77M / 0.67M; final-3M 0.00 / 0.93 / 0.00 / 0.27; mode evals 1.0 ×4 |
| dH_DV3 (core) | 0-2 | best periodic eval 0, 0, 0.83 | 0.28 | 1/3 (nonzero) | 6-ep periodic evals at ~60k/190k/320k; s2's 5/6 is its LAST eval (320k) |
| dDP_DV3 (core) | 0-1 | 0, 0 | 0.00 | 0/2 | s0 train_success 0.12 + log_picked at the very end (329k) but last periodic eval was 193k |
| dR2D_DV3 (core) | 0-1 | (s0 never evaluated), s1 0.83 | 0.83 | 1/1 | s1: 5/6 at 140k, then 0 at 180k/222k/260k/310k (transient); s0 crashed at step 0, requeued, finished 322k with NO periodic evals |
| dH_DV3 dense | 0-2 | 0, 0, 0 | 0.00 | 0/3 | s1 train_success up to 0.27 / log_picked at 298-322k, evals 0; s0/s2 negative train_return = shaping flowing |

## What the round says
1. **RLPD on model (DP-teacher) demos does not ignite: 0/6** (max 1/15; pooled demo-IC 2/90 = 0.02,
   random 9/90 = 0.10). Prior rounds: dH 8/16 and dR2D 10/16 ignited. dDP is now the only demo set
   that fails RLPD — mirrors the r2dreamer dDP null (0/20). Possible common cause to register
   before over-reading: the dDP set is the one whose demos were harvested from a closed-loop DP
   (93 npz incl. fails as zero-reward negatives) — check its action-density/encoding-cap audit the
   same way the dH 13.3%-over-cap caveat was checked.
2. **Dense reward (potential-based, gamma-matched) raises RLPD ignition as registered:** 4/6 vs
   sparse-dH 8/16; pooled 0.278 vs 0.221; ceiling unchanged (best seed 9/15). Random-IC pooled 0.20.
3. **dR2D_DP is the best BC cell ever**: in-dist 0.96 (vs dDP_DP 0.80, dHpruned_DP 0.62) and
   **random-IC 0.76 — ~3× the previous best (0.23 for both prior arms)**. Caveats already noted
   (same-machine rule; 66 champion tapes; the dHallpruned_DP action-density control is still the
   registered follow-up).
4. **r2dreamer + dense reward on human demos: 4/4 seeds ignite** (best-ckpt confirmations 0.76-0.98,
   mode eval 1.0 on every seed, first training pick at 209k-430k env steps) vs the sparse-dH lottery
   of 8/34. Bistability persists: s50 and s52 read 0.00 at the final 3M checkpoint (s52 was ≥0.8 for
   ~2.4M steps and collapsed at the very last checkpoint), s53 drifts 0.2-0.8, s51 sustains 0.93.
   The BEST-checkpoint protocol is what makes this readable — the final-checkpoint column alone
   would say 1/4.
5. **r2dreamer on dR2D (model) demos: 0/3 complete seeds, zero training picks in 3M steps** —
   s40-42 never picked once in 4000 sampled training episodes; s43 stopped at 1.81M env steps with
   no final eval / confirmation (wandb says `finished`; likely walltime/preemption without resume —
   check the .out if the cluster is still reachable). Consistent with dDP_R2D 0/20: model-demo
   pixel sets do not ignite r2dreamer.
6. **dv3 on genesis: first genuine picks, but only as transients.** Two 6-episode periodic evals read
   5/6: `rr_dH_s2` @320k (its final eval) and `rr_dR2D_s1` @140k (then 0 for the remaining four
   evals). I decoded both rollout videos (`policy_eval/rollouts`): the five credited episodes show
   approach → grasp → lift within ~10 agent steps and the failing episode drifts off — i.e. real
   policy behaviour, not a reset artifact. Still single 6-ep evals; the in-job `eval/picked` for
   dH_s2 never saw the 320k checkpoint. Sparse dH/dDP and shaped dH are otherwise null at eval; dDP_s0
   and shaped_s1 show end-of-run training picks (train_success 0.12-0.27) that no eval caught.

## Gaps / things that need a human or the cluster
- **Cluster shutdown (~08-24 per the 08-18 scope note): pull off the cluster NOW if not done** —
  r2d `BEST_selected.pt` + `ckpt_scores.tsv` + logdirs for s50-53 (the 4/4 result rests on them),
  dv3 logdir for `rr_dH_s2` (its final checkpoint ≈ the 5/6 one) and `rr_dR2D_s1`, the `.out`
  files (SWEEP-/DP-/R2D-/DV3-RESULT lines), and `cluster/RUN_REGISTRY.jsonl`.
- Fresh-process re-eval (15+ eps, several seeds) of dv3 `rr_dH_s2` final checkpoint before any
  "dv3 ignites on genesis" claim; likewise confirm `rr_dR2D_s1` only if a 140k checkpoint survived.
- r2d dR2D s43: find out why it stopped at 1.8M (no eval-of-record); dv3 dR2D s0: why the periodic
  evaluator never ran after the requeue (eval_rollouts None, no -eval-step runs).
- dv3 dDP_s0 / dH_shaped_s1 finished with training picks but no eval after them — a post-hoc eval
  of their final checkpoints would close those cells honestly.
- Registered follow-up still open: dHallpruned_DP (all-zero-frame pruning as the action-density
  control for the dR2D_DP result).

## This monitoring box
- wandb API venv: `~/.wandb-venv` (uv-managed, no pip; wandb 0.28.2, matplotlib; I added
  imageio + imageio-ffmpeg + av via `uv pip install --python ~/.wandb-venv/bin/python` to decode
  rollout mp4s). `~/.netrc` carries the api.wandb.ai login.
- `~/workspace/genesis_pickaplace` has the git history (HEAD 39fa492, branch 4dof-cartesian) but its
  working tree is empty — all 2002 tracked files show as staged deletions. The files live in
  `~/workspace/genesis_pickaplace-4dof-cartesian/` (a copy with a fresh, commit-less `.git`). The
  figs/scripts/doc from this pull were written into the history-bearing repo as untracked files.
- wandb note: `scan_history` (0.28) errors on runs with no `_step` column; `run.history(keys=[k],
  samples=N, pandas=False)` works for every run here.

## In context of the code and the registered expectations (added 08-22, after the py/md checkout)
Sources: ALGORITHM_STATE_2026-08-18 §1-3 (registered dense expectation), RESULTS_MATRIX_2026-08-15,
CLUSTER_ROUND_2026-08-17 RESULTS (prior-round rates), FABLE_HANDOFF §37-39, cluster/sbatch_*.sh,
baselines/rl/{full_env,train_rlpd}.py, dreamerv3-torch genesis_eval.py. Fisher p-values computed here.

### What produced each number (so the columns are read right)
- RLPD headline = `sbatch_rlpd.sh` §4a-2 in-job sweep: `wandb_eval.py --kind sac`, one FRESH process per
  episode, 15 demo-IC uids + 15 random-IC seeds, `--max-steps 400`, pushed as `sweep/demoIC|randomIC`.
  The in-train `eval_indist/picked` curves (VideoEvalCallback, 10 eps every 25k, same process) are
  monitoring only — they agree in rank with the sweeps this round (dDP all ≤0.2; s8-shaped 0.6).
- DP headline = `wandb_eval.py --kind dp --ic-mode both` (15+15, fresh process), same protocol as the
  n=8 dHpruned/dDP verdict rows; dR2D set = lerobot_dR2D_pick, 66 champion success tapes (9,184 frames).
- r2dreamer headline = `sbatch_r2dreamer.sh` lottery-coverage loop: every `latest.pt` write is archived
  and scored (15 eps, seed 0, sample), best-2-plus-newest kept; post-train the best is copied to
  `BEST_selected.pt` and CONFIRMED x3 on seeds 1-3 (+1 mode eval). Headline = confirm mean; selection and
  confirmation are independent draws. The -eval-step{N} wandb runs are exactly these (1 final + 4 best).
- dv3 headline = `sbatch_genesis_multi.sh` periodic evaluator (`genesis_eval.py`, 6 eps, deterministic
  actor, demo ICs) run against `latest.pt` every ~2h starting ~1h in → 2-3 evals per run at whatever
  step training had reached; dreamer.py ingests `eval_results.jsonl` into the -joint run's `eval/picked`.
  So a dv3 "hit" is one 6-episode draw of one checkpoint; nothing archives that checkpoint except
  `latest.pt` (overwritten) — only rr_dH_s2's hit (last eval, 320k of ~328k) is likely still on disk.
- Dense variants: all three trainers ran post the 08-19 23:34 timescale fix (r2d/dv3 shaped wandb runs
  created 08-20 08:24-08:53 UTC, after the scancel+resubmit; RLPD repeat-1 was exact before and after).
  Potential φ = -2·‖eef−can‖, applied once per decision, γ matched per agent (0.998 / 0.999 / 0.997),
  training-only; every headline above is still the sparse fresh pick.

### Against the registered expectations
1. **RLPD dense (ALGORITHM_STATE §3.1: "raise ignition, not the ceiling")** — confirmed directionally, not
   significantly: 4/6 vs 8/16 sparse (Fisher two-sided p=0.65); pooled 0.278 vs 0.221; best seed 9/15
   (0.60) = the sparse best (0.60). Random-IC pooled 0.20 vs 0.17. Matches the local verdict (s1/s2 4-5/15).
   Reading: basin-entry is part of the lottery but the §3.1 "0.7-0.9 if basin-entry-limited" band was not
   reached at n=6 — consistent with "partly credit/plasticity-limited" (§2.4 collapse is untouched).
2. **dDP_RLPD 0/6** — the prior dDP evidence was the superseded pair wave (0/3, pre demo-RNG fix). This is
   the first clean n=6: 0/6 vs dH 8/16 (p=0.05) and vs dR2D 10/16 (p=0.015). The CLUSTER_ROUND null
   ("no demo-SET effect at n=16/arm") was dH-vs-dR2D; dDP now breaks the null. Amendment-2 confounds do
   NOT explain it: the normalization audit put dH and dDP at similar over-cap rates (~11-14%), so the
   dH-vs-dDP RLfD contrast is the unconfounded one — and it separates. Candidate cause to check in the
   .out `[demos]` line: episode length / reward density of m1all_harvest (DP-teacher rollouts, 1200-cap)
   vs dR2D (short, ~139 frames/ep) vs dH (~917/ep, 33% no-pick). Note the odd random>demo-IC (9 vs 2 of
   90): the dDP demo ICs are the harvest ICs; worth one look at which uids those are.
3. **dR2D_DP 0.96 / 0.76** — extends the BC claim (dDP_DP 0.80 > dHpruned_DP 0.62) by another step and
   breaks the "random 0.23 for every arm" tie. RESULTS_MATRIX listed dR2D_DP as "trivial to add"; it is
   now the best cell in the matrix. Caveat stack unchanged (same-machine; pruned-vs-model idle-frame
   density → dHallpruned_DP control; 66 vs 63/8-seed arms).
4. **r2dreamer dense dH 4/4** — the §3.4 expectation was "smaller effect because r2d already ignites via
   imagination". Wrong direction: 4/4 vs 8/34 sparse (p=0.007; vs same-era wave-3 2/10, p=0.015), first
   training pick at 209-430k env steps vs ~1M for the wave-3 igniters, ceiling unchanged (0.91-1.00 ≈
   champion). What dense did NOT change: post-ignition collapse (s50, s52 read 0 at 3M; s53 drifts) — §5's
   multi-policy/reset program is still the lever for that. Because `ckpt_scores.tsv` keeps only best-2 +
   newest, the confirmations rest on `BEST_selected.pt` files on the cluster.
5. **r2dreamer dR2D 0/3 (+s43 stopped at 1.8M)** — with dDP 0/20, model-demo pixel sets are 0/23 vs dH
   8/34 (p=0.016). Both model sets are short, idle-free, reward-dense — the opposite of the dH tapes —
   and the WM arms consume them only as dynamics/reward data (actor_bc_lambda 0). Whatever r2dreamer
   needs from demos, the human tapes have it and the model tapes don't; dense-reward on dR2D_R2D would
   test whether that is basin-entry (cheap add: CONFIG=..._shaped ARM=dR2D).
6. **dv3** — §3.4 predicted dv3 as the LARGEST dense beneficiary; dense dv3 read 0/3 at eval (s1 logged
   training picks, train_success 0.06-0.27 at 285-322k, yet its 320k deterministic 6-ep eval read 0 —
   sampled-policy picks that the mode actor does not reproduce, or too few episodes to catch them). Sparse dv3 instead produced the project's
   first two nonzero genesis evals (5/6 twice). That retires the ALGORITHM_STATE sentence "dv3 never
   leaves 0 on genesis", but replaces it with "dv3 reaches transient takeoffs at 140-320k that are not
   sustained within the 300k budget" — the MS-HEAD control (0.9 at 200k) says the budget is ~2.2-2.7x the
   MS takeoff window, and genesis takeoffs here start at the END of that budget. A budget extension
   (600k) on a few seeds is the honest next test, with per-step checkpoint archiving like r2d's.
7. **Round-robin framing (§3.5)** — "under dense reward, does the demo-source effect appear?" is NOT yet
   answerable: dense ran only on dH for all three trainers. The sparse source effect now reads: BC
   prefers model demos (dR2D > dDP > dH); RLPD works on dH and dR2D but not dDP; r2dreamer works only on
   dH. No single "human vs model" sign — it is algorithm × source.

## Why dDP_RLPD < dH_RLPD (08-22 investigation; user question)
Data: `baselines/m1all_harvest` (on this box, sibling dir), its manifest, the n=16 dH/dR2D (08-17) +
n=6 dDP + n=6 dH-dense RLPD wandb histories (`train/actor_q_mean`, `train/critic_loss`,
`train/ent_coef`, `rollout/*`), AUDIT_normalization_2026-08-17 tables.

What the dDP stream is: DP-teacher (ouro gen0 dp_joint) rollouts on the HUMAN demo ICs (ic_mode=demo,
seed 7, 320 rollouts, 19.7% yield) → 63 successes (first lift p50 512 frames, tape cut 10 frames after
the lift) + 30 FAILS kept whole at the 1200-step cap (36,000 of the 70,028 transitions = 51% of the
buffer; dH no-pick share 33%, dR2D 0%). Over-cap frames 8.1% (dH 4.4%, dR2D 0.1%) — the DP teacher
moves faster than the 0.025 rad/step cap more often than any other set; zero held/zero-delta frames
(not an action-repeat artefact). In the 30 fails the DP flails for 1200 steps: can shoved 9 cm median,
8/30 tapes tip the can; ~20% of fail frames (~7k transitions, ~10% of the buffer) carry a tipped can, and
6.0% of the buffer meets the env's strict termination predicate (tilt>60° AND grip open) — states at which
FullTaskEnv(pick) TERMINATES online (tip rule, TIP_PENALTY 0), so they are never visited online and the
encoder leaves them done=False → the critic bootstraps through them. (3 of the 30 fails START tipped:
lying-can ICs the env would terminate at reset.)

What the trainer does with it (same config everywhere: 50% immutable demo batch, UTD 10, LN ensemble):
| arm | median actor_q @25k / 50k / 100k | median ent_coef @100k | median ep_rew @100k |
|---|---|---|---|
| dR2D (16) | 0.6 / 20 / 8 | 0.02 | 0.20 |
| dH (16) | 0.5 / 212 / 1,090 | 0.88 | 0.095 |
| dH dense (6) | 0.3 / 105 / 648 | 0.49 | 0.53 (shaped) |
| **dDP (6)** | **32 / 983 / 12,900** | **9.4** | **0.015** |
With a +1 terminal the true Q is ≤1. Critic overestimation is a project-wide late pathology (it is the
collapse-after-ignition mechanism; dH seeds reach Q 1-4k by 100k), but dDP hits it 50x earlier
(4/6 seeds Q≥64 at 25k vs 2/16 dH, 2/16 dR2D ≥30) and 10x harder (4/6 seeds Q 12-26k at 100k; max of
the other 38 runs 4.4k), BEFORE any online pick (dDP ep_rew 0 at 25k). Saturated actions → entropy
collapse → alpha 7-18 → near-random policy (ep_len ~790) → no picks. The other two dDP seeds did not
diverge (s3 Q 5, s5 Q 819) and simply never found the pick — the ordinary ~50% non-ignition.
So 0/6 ≈ 4 poisoned + 2 ordinary misses; the dDP-specific part is demo-driven critic divergence.

Ordering across sets — divergence dR2D < dH < dDP — follows both (i) cap-clipped, dynamics-inconsistent
transitions (0.1% / 4.4% / 8.1%) and (ii) failure share, esp. post-termination (tipped-can) frames
(0 / 33% / 51%, ~10% tipped-can / 6% strictly post-termination in dDP). Reward density is NOT the lever (dDP 0.090% ≈ dH 0.079%; the
hold-reward arm showed 25x density does not move ignition). It is not "model demos are worse": dR2D is
model demos and is the best RLPD arm — it is the cleanest tape (exact labels, all-success, short).
Cheap discriminating tests: (a) dDP success-only (63 eps) x6 — if it ignites like dH, the fail tapes
(post-tip OOD frames) are the culprit; (b) dDP re-encoded closed-loop (delta_rerecord-style) — tests
the cap-clipping channel; (c) truncate demo fails at the first tip frame (what the env would do).
Caption-level statement for now: under RLPD the demo-SET effect is about tape consistency with the
training MDP (representable actions, no post-terminal frames), not human-vs-model origin.

### Follow-up (08-22): are the DP fails different in KIND, and do the DP successes matter?
Human comparison set available on this box = `episodes_delta_rerecord_pick_all` (72 tapes, closed-loop
re-recorded human pick set, 54 success + 18 no-pick; proxy for the live dH set, which is not here).
| | dDP m1all | human re-record pick_all (proxy) | dR2D |
|---|---|---|---|
| fail share of buffer | 0.51 (30 x 1200 steps) | 0.17 (live dH: 0.33) | 0 |
| tapes that tip the can / tipped-can share of buffer (strict post-termination share) | 8/30 / **0.101 (0.060)** | 2/14 / 0.023 (0.016) | 0 |
| longest post-tip chain (frames) | **1200** (whole episode) | 563 | — |
| can max-displacement in fails, p50 | 0.10 m | 0.07 m | — |
| NN-distance of fail frames to ANY success frame (std units) p50 / p90 | **2.7 / 42** | 0.9 / 4.8 | — |
| fail frames farther than the success set's own 99th-pct self-distance | **0.81** (0.94 first pass) | 0.43 (0.70) | — |
| over-cap frames, successes / fails | 0.081 / 0.079 | 0.042 / 0.056 | 0.001 |
| success first-lift p50 | 512 | 784-826 | ~120 |
Reading: the DP fails are not just more numerous, they are OFF-MANIFOLD — long (1200-step, done=False
at the end, no terminal anchor) chains of states no success tape and no online rollout ever visits
(tipped can, can shoved 10-25 cm, DP oscillating). In SAC the actor loss runs over the mixed batch, so
the actor is pushed to maximise Q at those states, the target uses its actions there, and the chain
bootstraps on itself with gamma=0.998 (loop gain ~1) — the offline-RL extrapolation loop, which RLPD
normally avoids because demo states lie where online data soon goes. Human fails sit close to the
success manifold (p50 0.9 std) and are grounded by it; dR2D has no fails at all. The DP SUCCESSES show
no destructive signature beyond the 2x over-cap rate (a bounded bias, same in their fails) — nothing
measured implicates them; the success-only dDP arm is the test that would settle it.

## 08-22 independent audits (two fresh-context reviewers; full reports paper/AUDIT_impl_2026-08-22.md, paper/AUDIT_design_2026-08-22.md) — what I verified and what changes above
Verified against the code/patches by me:
1. **Encoder has no env-terminal guard** (train_sacfd_full.py relabel_full / delta_encode_transitions): no-pick
   tapes are emitted whole, every transition r=0 done=False incl. the last; rlpd_sac bootstraps through all
   of it and runs the actor loss over the mixed batch. The correct pattern (cut at the terminal) exists in
   relabel_hold_region. This is the code-level mechanism behind "Why dDP_RLPD < dH_RLPD"; it applies to every
   arm, dDP just supplies the most/longest post-terminal chains. A `--demo-terminal-guard` flag in the encoder
   is the real fix but is a trainer change (breaks byte-identity with prior waves) — registered, not applied.
   Consequence for dDPtiptrunc: it removes the chains but the cut tape's last transition still bootstraps
   (done=False) — one dangling step per tape; stated in make_dDPsucc.py.
2. **DP vs RLPD eval protocols differ**: sbatch_dp passes no --max-steps → wandb_eval default **1200** and all
   30 episodes in ONE process; RLPD sweep = **400** steps, fresh process per episode. RESULTS_MATRIX already
   carries "RLPD 400 / DP 1200, measured effect nil" but ROUND_ROBIN_RUNNING/REVIEW_GUIDE describe both as
   fresh-process — correct those. Cross-algorithm absolute comparisons (e.g. "dR2D_DP is the best cell in the
   matrix") must state the horizon; within-algorithm contrasts are unaffected.
3. **r2dreamer shaping γ probably mismatched**: cluster/r2dreamer_dense_lever.patch adds the shaped config with
   `horizon: 333  # discount 0.997` while the shaping term is hard-coded γ=0.999 ("horizon-1000 discount").
   If the agent discount is 0.997 the residual is (0.999−0.997)·φ(s') ≈ −0.4·d per agent step at scale 100
   (a distance-proportional per-step cost; benign direction, ~−12/episode vs +100 terminal) and the "exactly
   policy-invariant" wording for dH_R2D dense must be softened to "approximately". VERIFY on the cluster
   r2dreamer tree: `grep -n "discount\|horizon" configs/env/genesis_pick_v5d4c_delta*.yaml`.
4. **WM "dH" is the pruned, success-only human set**: r2dreamer dH = genesis_pick_pruned_delta25 (67), dv3 dH =
   genesis_pick_msr_delta25_r4 (67) — while WM dDP = genesis_m1all_delta25 (93 incl. the 30 DP fail tapes)
   and dR2D = 52 successes. ROUND_ROBIN_2026-08-20 §4.3 ("dH means UNPRUNED for every RL/WM row") is wrong
   for the WM rows. So the flagship r2d contrast dH 8/34 vs dDP 0/20 carries the fail-tape variable of this
   doc's §"Why dDP_RLPD"; the fail-free model comparison is dR2D_R2D 0/3(+1 partial) only. Zero-GPU check:
   `python analysis/characterize_demo_sets.py dH=.../genesis_pick_pruned_delta25 dDP=.../genesis_m1all_delta25
   dR2D=.../genesis_r2dchamp_delta25 --out ...` (dreamer-format supported) — count zero-reward episodes.
   Cheap fix direction if confirmed: a success-only dDP pixel set for r2d (same recipe as dDPsucc).
5. **Other verified items**: the 15 demo-IC uids are identical for every arm and algorithm (not an IC
   artifact; closes the "random>demo" question above); the 15 "random" ICs are one fixed set shared by all
   RLPD runs (pooling 90 episodes as independent overstates n); `R2D-RESULT picked=` in the .out is the
   best-ckpt MODE eval (tail -1 of metrics.jsonl), not the final checkpoint — wandb -eval-step runs are the
   record; demo pick reward is granted one frame early with a weaker predicate than the env (uniform across
   arms); RUN_REGISTRY refuses sequential duplicates but not concurrent submissions; run-to-run the sweep
   denominators here all read 15 (checked in the wandb summaries).
Design-level (not re-verified, but read and endorsed as the prioritization): no cell is single-variable
(buffer size / fail share / reward density / fidelity / tape length all co-vary with "source"); ignition is a
threshold on a 15-episode binomial; r2d ignition counts depend on checkpoint-scoring coverage (lead with
time-to-first-pick); RLPD has no passing positive control; H4 reads as refuted in the opposite direction
(BC is the arm indifferent-to-favorable toward model demos; the WM discriminates hardest). Recommended
thesis: demo-source effects are learner-specific and driven by measurable tape properties (idle density,
action representability, failure-tape geometry/termination labelling), not human-vs-model per se.

## Built 08-22 (in this commit)
- `baselines/make_dDPsucc.py` — `--mode succ` (dDPsucc: 63 DP successes), `--mode tiptrunc` (fails cut at
  the env tip rule; lying-can-IC fails dropped), `--mode r2dfails` (dR2D successes + the 30 DP fails: the
  sufficiency arm). Writes manifest.json; `cluster/sbatch_rlpd.sh` gained ARM=dDPsucc | dDPtiptrunc | dR2Dfails
  with a manifest provenance gate (refuses hand-made dirs; DRYRUN-tested here).
- `analysis/characterize_demo_sets.py` — the demo-set census (episodes-format full metrics incl. OOD-ness
  and cross-set NN matrix; dreamer-format composition). Dev-box run on the sets available here:
  paper/demo_set_census_devbox_2026-08-22.md. Run on the cluster with all five episodes-format sets + the
  six WM dirs.
- `paper/PAPER_NOTES.md` — prose-ready findings N1-N4 with pre-registered predictions for the new arms.
