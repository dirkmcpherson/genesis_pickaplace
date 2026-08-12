# Canary analysis: r2dreamer pick-task — local discovery vs cluster non-discovery (2026-08-12)

**Question.** With demo checksums, configs, code heads, prefill counts, and pixel renders certified
identical, why do local runs "discover" (jump to high pick success) while cluster runs never do?
This report compares leading-indicator training metrics in the early window (first ~430k env steps
after prefill, with special attention to the 100–300k region) between:

- **LOCAL discovery:** `pick_delta25d4_s0` (first train pick / ignition at env_step 228,057; became
  the 0.91 champion) and `pick_d4clamp_s0` (first train pick 503,413; cascade by ~690k).
- **LOCAL non-discovery:** `pick_delta25d4_s1` (killed at 1.91M), `pick_delta25d4_s2` (3M; one eval
  blip 0.067 @1.57M).
- **CLUSTER (all non-discovery):** `dH_R2D_s0..s3`, `dDP_R2D_s0..s4` (wandb `jambotime/r2dreamer_genesis`).

**Data provenance.** The local training histories for s1, s2, and clamp were **never synced to wandb
as training runs** (only their `-eval-step*` one-shots are there); this analysis reads their complete
local `runs/<name>/metrics.jsonl` (`/home/j/workspace/r2dreamer/runs/`). `pick_delta25d4_s0` exists
both places (jsonl used; last-step/env_step agree). Cluster runs pulled via per-key-group
`scan_history` (no downsampling). Cached series: scratchpad `canary/cache/*.npz`.

---

## 1. Structural findings (found before any canary comparison)

These reframe what "9 cluster runs" means and must be read first.

### 1a. Cluster wandb runs contain silent FROM-SCRATCH restarts

5 of 9 cluster runs have `env_step` jumping backward to the post-prefill step with
`train/opt/updates` reset to 1000 — i.e., the job was restarted **from scratch** (same seed) inside
the same wandb run:

| wandb run | attempt 0 reached | attempt 1 reached | notes |
|---|---|---|---|
| dH_R2D_s0 | 1,437,390 | 2,286,850 | |
| dH_R2D_s1 | 2,999,532 | — | single attempt |
| dH_R2D_s2 | 868,287 | 2,452,740 | |
| dH_R2D_s3 | 821,548 | 2,192,696 | |
| dDP_R2D_s0/s1/s2 | ~3.0M | — | single attempts |
| dDP_R2D_s3 | 1,890,016 | 1,821,716 | |
| dDP_R2D_s4 | — | — | **1103 backward jumps: two CONCURRENT writers interleaved** (two jobs logging the same run at the same time; writer A to ~1.79M w/ 32 picks, writer B to ~2.38M w/ 9 picks; 231 exact-duplicate rows) |

Consequences: (i) the cluster actually produced ~13 shorter independent attempts, not 9×3M;
(ii) `dDP_R2D_s4`'s two simultaneous jobs presumably shared a run/checkpoint directory —
checkpoint/buffer clobbering risk; (iii) any full-run summary read off these wandb runs mixes
attempts. All canary comparisons below use **attempt 0 only**.

### 1b. dDP condition has a different prefill than local / dH — despite "certified identical prefill"

First logged train row / first episode row:

- local runs and all dH_R2D runs: env_step **172,416 / 174,816**
- all dDP_R2D runs: env_step **281,832 / 284,232** (+63%)

Downstream effect (real, systematic): demo reward frames per training batch are diluted —
`train/data/reward_frames` ≈ **5.0–5.4** for local and dH vs **2.6–3.4** for dDP (window 0–130k after
prefill), i.e. the dDP arm trains on **~40% less sparse-reward signal per gradient step** in the
critical early window. The ratio 172k/284k ≈ 0.61 predicts the dilution almost exactly. If the
prefill certification was supposed to cover dDP, it missed this. (If the DP demo set is intentionally
larger, this is still an uncontrolled confound for the H-vs-DP comparison arm.)

### 1c. Update ratio is identical everywhere (synchronous training confirmed)

`train/opt/updates` advances at exactly **0.125 updates/env_step (1 update per 8 steps)** on every
run, local and cluster (e.g. 17,249±1 updates @300k for all local+dH runs). Throughput differences
cannot change data/update interleaving.

---

## 2. Key inventory and parity

Same logger everywhere. Local jsonl and cluster wandb share the identical `train/*`, `episode/*`,
`fps/fps` key set (54 keys). Cluster runs (and the two later local runs, s2 + clamp) additionally log
in-training `eval/*` (`eval/picked`, `eval/tipped`, `eval/timeout`, `eval/mean_steps`,
`eval/mode_flag`); s0 and s1 predate that patch and used external eval runs instead. **No key is
missing on the cluster.** Not logged anywhere: gradient norms (only `train/opt/loss`,
`train/opt/grad_scale`); no per-batch demo-row counter beyond `train/data/reward_frames`,
`train/data/reward_sum`, `train/data/demo_reinjections`.

In-training eval on cluster: `eval/picked` = 0 in every eval of every run, except one blip
`dH_R2D_s3` = 0.067 @1,072,416 — the same magnitude as local s2's lone blip.

---

## 3. Train-pick events (the "fuel" question — analysis c)

Counts of train episodes with `episode/train_picked==1`. "≤600k" = absolute env_step (includes
prefill; note dDP effectively gets ~110k fewer post-prefill steps in that window — their aligned
counts would only go up slightly).

| run (attempt) | group | picks ≤600k | picks total | first pick @env_step | horizon |
|---|---|---:|---:|---:|---:|
| pick_delta25d4_s0 | LOCAL disc | 183 | 5004 | 228,057 | 3.0M |
| pick_d4clamp_s0 | LOCAL disc | 1 | 18,305 | 503,413 | 3.0M |
| pick_delta25d4_s1 | LOCAL non | 49 | 56 | 206,557 | 1.91M |
| pick_delta25d4_s2 | LOCAL non | 1 | 3 | 190,128 | 3.0M |
| dH_R2D_s0 (a0) | CLUSTER | 0 | 1 | 1,091,540 | 1.44M |
| dH_R2D_s0 (a1) | CLUSTER | 0 | 0 | — | 2.29M |
| dH_R2D_s1 | CLUSTER | 17 | 17 | 217,856 | 3.0M |
| dH_R2D_s2 (a0) | CLUSTER | 13 | 14 | 204,922 | 868k |
| dH_R2D_s2 (a1) | CLUSTER | 0 | 0 | — | 2.45M |
| dH_R2D_s3 (a0) | CLUSTER | 12 | 14 | 217,753 | 821k |
| dH_R2D_s3 (a1) | CLUSTER | 4 | 12 | 206,139 | 2.19M |
| dDP_R2D_s0 | CLUSTER | 2 | 3 | 298,870 | 3.0M |
| dDP_R2D_s1 | CLUSTER | 3 | 7 | 286,805 | 3.0M |
| dDP_R2D_s2 | CLUSTER | 2 | 5 | 298,872 | 3.0M |
| dDP_R2D_s3 (a0) | CLUSTER | 5 | 5 | 286,847 | 1.89M |
| dDP_R2D_s3 (a1) | CLUSTER | 0 | 0 | — | 1.82M |
| dDP_R2D_s4 (writer A) | CLUSTER | 6 | 32 | ~287k | 1.79M |
| dDP_R2D_s4 (writer B) | CLUSTER | 2 | 9 | ~299k | 2.38M |

**The hypothesized headline ("cluster gets ZERO train picks ever") is FALSE.** Cluster runs get
occasional train picks at the same env_steps local runs do (first picks at 205–300k), at rates inside
the local non-discovery envelope (local non-disc: 1–49 by 600k; cluster attempts: 0–17). The fuel
exists on the cluster; the cascade never ignites.

Equally important on the local side: **pick events do not predict ignition.** s1 collected 49 picks
by 600k (more than any cluster attempt) and still died; clamp ignited from its very first pick at
503k. Discovery looks like a rare ignition event conditional on picks, not a monotone function of
pick count.

Discovery-rate arithmetic: local 2/4 attempts vs cluster 0/13 attempts (0/17 if second writers and
short attempts all count). Fisher exact (hypergeometric), one-sided: p ≈ 0.03 (0/13) — but for the
like-for-like dH-vs-local comparison only (0/7), p ≈ 0.11. **Suggestive, not decisive.**

---

## 4. Per-canary verdicts (analysis a, b, d, e)

Windows are **prefill-aligned** (rel = env_step − 172,416 local/dH; − 281,832 dDP), attempt 0 only.
"Envelope" = [min,max] of the two local non-discovery run means ±50% span slack (only 2 runs — the
envelope is narrow, so "outside" is a weak signal by construction). Full per-run numbers:
`window_stats.json` / `verdicts.json` in the scratchpad `canary/` dir.

| canary | separates local disc from non-disc pre-ignition? | cluster vs local envelope | verdict |
|---|---|---|---|
| episode/train_picked (events) | NO — s1 out-picked s0 pre-ignition | inside (0–17 vs 1–49) | **inconclusive as a separator; fuel present on cluster** |
| episode/length (mean, 0–130k) | no (94–97 everywhere; s0 drops to 83 only *post*-ignition) | inside (93.5–97.4) | no difference; **no 300-step timeouts anywhere — timeout is 100 in all runs** |
| episode/train_tipped rate | no (2–8% all runs, window-dependent) | overlapping (1.5–13%) | inconclusive / no systematic offset |
| train/action_entropy | no — bimodal oscillation (−6.2 ↔ +9.93) in every run; means overlap | inside for dH; dDP slightly lower early | inconclusive (weak canary due to bimodality) |
| train/action_std (130–430k) | disc lower (0.49–0.53) but s1=0.59, s2=0.64 | cluster 0.54–0.80, 7/9 above envelope | mildly higher exploration noise on cluster; likely effect of not-learning, not cause |
| train/val, train/tar, train/ret_replay_mean, train/value_replay_mean | NO — dH_s1 shows a genuine value ramp to ~170 (rel ~310k) resembling s0's early ramp, then collapses; local s1 ramps to ~280 and also collapses | inside | value cascade *starts* on cluster too and fizzles, exactly like local non-discovery |
| train/loss/value | no | overlapping | inconclusive |
| train/loss/rew | no | cluster slightly lower, overlaps clamp | inconclusive |
| train/loss/dyn (130–430k) | no (clamp 1.51 ≈ cluster) | cluster 1.48–1.74 vs non-disc 1.78–1.83, but *matches the discovery run* | inconclusive |
| train/dyn_entropy | no | overlapping | inconclusive |
| train/data/reward_frames, reward_sum | no (all local ≈ equal) | **dH inside (4.5–5.3 ≈ local 5.0–5.4); ALL dDP 40% low (2.6–3.4)** | **SYSTEMATIC for dDP only — prefill dilution (§1b)** |
| train/data/demo_reinjections | no | comparable (reaches 13–18 by ~600k everywhere) | no difference |
| train/rew (predicted reward) | no | dH inside; dDP higher early (different demo reward structure) | dDP condition difference, mirrors §1b |
| train/con | no | minor early dip on dDP (more termination frames) | inconclusive |
| fps/fps (analysis e) | s0 faster (60–64) than s1/s2/clamp (43–53) — mostly post-ignition short episodes | cluster 46–71, overlaps | record only; update ratio identical (§1c) so throughput can't change dynamics |
| updates per env_step | identical | identical (0.125) | no difference |

**Bottom line of the canary sweep: for the like-for-like condition (dH_R2D vs local pick_delta25d4),
not a single training-dynamics canary places the cluster runs outside the local non-discovery
envelope.** The only systematic cluster-vs-local differences found are (i) the operational restart
mess (§1a) and (ii) the dDP prefill/reward-dilution difference (§1b) — which does not apply to dH.

### Figures (`paper/figs_canary/`)

- `cum_train_picks_600k.png`, `cum_train_picks_full.png` — cumulative train picks (discovery bold
  blue, local non-disc green, cluster dashed; x = steps after prefill)
- `train_val.png` — critic value; note dH_s1's aborted ramp alongside local s1's aborted ramp
- `action_entropy.png`, `episode_length.png`, `loss_value.png`, `loss_dyn.png`, `reward_sum.png`
  (the dDP dilution is visible), `train_rew.png`, `fps.png`

---

## 5. Most likely explanation(s), ranked by evidence strength

1. **Discovery is a low-probability stochastic ignition, and the cluster's sample is consistent with
   unlucky draws — there is no measurable training-dynamics defect on the cluster (dH arm).**
   Evidence: every canary inside the local non-discovery envelope; cluster runs get first train picks
   at the same 205–300k steps; dH_s1 even reproduces the early value-ramp-then-collapse seen in local
   s1; locally, ignition is not predicted by pick count (s1: 49 picks, no ignition; clamp: ignited on
   pick #1 at 503k) nor by any canary we measured. Fisher p for 0/7 like-for-like attempts vs 2/4
   local ≈ 0.11. This does not *prove* equivalence — it bounds any systematic effect below what these
   canaries can see — but "same process, unlucky draws" is the hypothesis the data supports.

2. **The cluster's effective attempt budget is much smaller than nominal, hiding some of the
   shortfall.** Silent from-scratch restarts (§1a) mean the "9×3M" cluster campaign was really ~13
   attempts of 0.8–3.0M, several truncated at 821k–1.44M — clamp ignited at 503–692k, i.e. some
   truncated attempts died inside the local discovery window, and dDP_s4 ran as two concurrent jobs
   with a shared run identity (checkpoint clobbering risk). Fixing the requeue behavior (resume from
   checkpoint, or at least new run names) is cheap and removes this confound.

3. **The dDP arm specifically trains on ~40% less demo-reward signal per batch** because its prefill
   is 281,832 vs 172,416 (§1b) — `train/data/reward_sum` ≈ 260–340 vs ≈ 500 local/dH in the early
   window. For a sparse-reward FD setup this plausibly lowers ignition probability by itself, and it
   contradicts (or was missed by) the "prefill counts certified identical" certification. It does not
   explain the dH arm.

Not supported by the data: missing/renamed metrics on cluster; update-ratio or synchronicity
differences; tip-rate or episode-length (timeout-flailing) differences; throughput coupling;
zero-pick cluster environments; entropy collapse or value blow-up unique to the cluster.

**Actionable next steps implied:** (1) fix cluster requeue → resume-from-checkpoint + unique run
names per attempt; (2) re-certify dDP prefill / demo-reward density (equalize reward frames per
batch, not just "prefill count"); (3) since ignition is rare-but-early (both local ignitions ≤700k),
run many short (≤1M) cluster seeds instead of few 3M ones to measure the ignition rate directly —
~10 more attempts distinguishes p_ignite=0.5 from p≈0.05 decisively; (4) if a mechanism hunt
continues, the next canary layer is inside the ignition window itself (per-episode value of picked
episodes, replay-buffer age of reward frames at sample time), which is not in the current logs.
