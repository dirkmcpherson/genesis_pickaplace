# AUDIT — zero-action / idle / stops-vs-creep analysis (2026-09-02, read-only, adversarial)

Scope: what "action" and "zero action" mean in every tape format behind idle_frac, strict_stop_frac,
fig7/10/12 and the pruning step; whether the human-vs-machine gaps are behaviour or provenance.
Sparse checkout (no npz datasets); numbers below come from the committed CSVs and
`paper/figures/speed_series_w3.npz`. Nothing was modified except this file.

## Verdict

- **(a) "human demos have idle decisions, machine demos have none (0.19 vs 0)" — FLAWED as a behavioural
  claim.** Machine idle_frac = 0 is guaranteed by the recording adapter (absolute DP target minus running
  target, continuous sampler); human zeros are produced by the waypoint follower re-issuing an already-reached
  command. True as a statement about tape bytes; says nothing about the teacher's behaviour.
- **(b) strict-stop 0.37 vs 0.22 (d +1.16) as the WM mechanism candidate — SOUND WITH CAVEATS.** It is an
  EEF-displacement metric (state, not action), so it is provenance-safe in *definition*; but the entire gap
  lives below 0.3 mm/decision, the machine has a hard ~0.1–0.5 mm jitter floor, and the dwell-episode
  structure of the two sources is IDENTICAL at a 2 mm threshold. "Stops vs creep" is more precisely
  "frozen target vs jittering target during the same dwells" — plausibly sampling noise of an
  absolute-target diffusion policy, not a different motion strategy. The A28 ablation remains the right test.
- **(c) "pruning removes time, not space" — SOUND WITH CAVEATS.** Coverage numbers support it. But the pruner's
  idle definition differs from the metric's (different signal, units, columns, segment), the raw set also
  differs in horizon cap and N, and the attribution of DP's raw penalty to "idle/dwell mass" is weak
  (common-IC idle_frac differs by 0.06; row ratio 1.16–1.20; 70 % of extra rows are 8 long tapes).
- **(d) fig7 real-vs-sim — FLAWED labelling, incomparable idle.** The "real command tapes" are MEASURED
  `/joint_states` positions binned at 60 Hz (not commands, not velocities); the real idle metric is a
  joint-speed threshold ~80× looser than the sim one; path-length/coverage multipliers include encoder-noise
  inflation; the 60 Hz→30 Hz replay clock is unresolved in the docs.

## Q1. Action layout per format

| format | rows | arm cols | grip col | semantics | units | clock |
|---|---|---|---|---|---|---|
| real `inthewild_trials/<uid>_episodes.npy` `vel_cmd` (T,6) | 1 per 60 Hz bin | 0–5 | separate `gripper_pos` 0..100 | **mean of measured `/my_gen3_lite/joint_states` position[:6] per bin** — `trial_reader.py:7-8,46-51` | rad (absolute, measured) | 60 Hz (`trial_reader.py:31,44`) |
| stride-1 human sim tapes `episodes_pick_phase{,_dppruned,_all}` `actions` (n,7) | 1 per env step | 0–5 absolute joint target = `vel[i]` | 6 = clip(gp/100,0,1) | `collect_all_classified.py:81-97`; env applies as PD position target `genesis_can_env.py:243` | rad / [0,1] | env = 30 Hz, 3 physics steps of 0.01 s (`genesis_can_env.py:1,14`) fed ONE 60 Hz frame per step |
| contract-v1 npz (all matched sets: dH, dHv2raw, dHv2, dDP, dDPv2…) `actions_delta` (n,7) | 1 per DECISION = 4 sim steps | 0–5 normalized delta in [-1,1]; 1.0 = 4·0.025 = 0.1 rad over the window | 6 = ABSOLUTE grip target in [-1,1] (= 2·g−1) | `record_demos.py:20-21,45,290-296`; `full_env.py:568-586` (`delta_ref=target`, cap 0.025/sim-step, leash 0.125) | dimensionless; 1e-3 ≙ 1e-4 rad/decision | 7.5 Hz decisions (`METHODS_draft:184`) |
| contract-v1 `actions` (n,7) | same | 0–5 absolute window-end target `_dj_target` | 6 = grip 0..1 | `record_demos.py:271-273` | rad | same |

Human vs machine sit on the SAME contract, but the adapters that fill `actions_delta` differ in kind:
- **human** (`HumanFollower.act`, `record_demos.py:428-444`): `a_arm = clip((cmd[jt] − target_now)/0.1)` where `cmd`
  is the stride-1 absolute stream and `jt` advances up to 4 source frames per decision; after the window the
  integrated target equals `cmd[jt]` to float32 rounding (~1e-8 rad), so a decision is EXACTLY zero iff the
  follower re-issues a waypoint the target already sits on: (i) the source holds still for ≥4 frames (arm at
  rest — the source is measured joint state, so encoder-level stillness), (ii) settle rows after the waypoints
  are exhausted (`:420-425`, up to 25), (iii) dwell stalls (rare under `--arrival either`, the setting of record,
  `V2_BUILD:56-60`).
- **DP** (`DPTeacher.act`, `record_demos.py:561-566`): `a_arm = clip((q* − target_now)/0.1)` with `q*` the policy's
  absolute joint target from a 100-step DDPM sample (`--mode sample`, `record_demos.py:705-712`). `q* == target_now`
  to 1e-4 rad has essentially zero probability, so **`max|a_arm| < 1e-3` can never fire — by construction.**
  A28's `SchedFollower` (repeated waypoints) DOES produce exact zeros, which is why dDPretimed "gains" stops.

## Q2. Definitions of idle / zero action / strict stop — every discrepancy

| script | signal | columns | threshold | segment | grip-during-stop counts idle? |
|---|---|---|---|---|---|
| `extract_tape_stats.py:31` (fig6/fig7 sim side) | `actions_delta` per decision | 0–5 | `max|a| < 1e-3` (normalized ≙ 1e-4 rad) | whole tape | yes (grip col ignored) |
| `extract_raw_vs_pruned.py:15,23-24` (fig12) | same + `idle_frac_7col` + `strict_stop` | 0–5 / 0–6 / EEF | 1e-3 / 1e-3 / 0.5 mm | whole | yes |
| `tape_dynamics_metrics.py:158-166` (fig8, WM_METRIC) | **no action-idle metric at all**; `pause_frac` relative 0.2·median EEF speed; `strict_stop_frac` EEF step < 0.5 mm; sweep 0.2/1/2 mm | EEF | absolute | whole | n/a |
| `make_fig10_burstiness*.py:25,38,48` | EEF speed from `speed_series_w3.npz` (`extract_speed_series.py:7`) | EEF | < 0.5 mm | whole | n/a |
| `make_dp_pruned.py:79-81` (THE pruner behind every dH/dHv2 set) | `|actions[t+1]−actions[t]|` on the STRIDE-1 absolute stream | **all 7 incl. grip on 0..1 scale** | `< 1e-3` (rad for arm; dimensionless for grip) | only `[0, j_pick − 150 frames)`; runs collapsed to their FIRST frame | no — a moving grip breaks the run |
| `make_pruned_bc_set.py:29-49` (dHallpruned controls only) | `actions_delta` | 0–5 AND grip unchanged (5e-3) | 1e-3 normalized | whole minus terminal | no |
| `extract_real_tape_stats.py:69-70` | `|Δ vel_cmd|` between consecutive **60 Hz measured** frames (full rate, not the stride-2 FK) | 0–5 | `< 1e-3 rad/frame` ≙ joint speed < 0.06 rad/s | whole real trial (all phases) | yes |

Discrepancies that matter:
1. `FIGURES_2026-08-31.md:216` says "idle threshold 1e-3 rad on commanded deltas". Wrong by 10×: 1e-3 is
   normalized, = 1e-4 rad per decision (`record_demos.py:45`; `make_pruned_bc_set.py:29` states 2.5e-5 rad/sim-step).
   fig12b's title "|Δaction| < 1e-3" also mislabels — the tested quantity is |action| (already a delta).
2. The pruner tests a frame-to-frame CHANGE of an absolute 30 Hz-stepped stream; the metric tests the magnitude
   of an executed decision-level delta. Different signal, rate, units, segment. `FIGURES:216-217` discloses this
   in one clause; `RESULTS:37-38` and `CLAUDE.md` ("idle_frac metric fixed … human pruned ~0.19") do not.
3. The metric counts the grasp dwell (arm frozen while the grip closes) as idle; the pruner deliberately never
   touches it (150-frame margin + grip column). So "pruned ≈ 0.19 idle" is mostly closure dwell + the 5 s
   pre-pick margin + settle rows, not a pruning failure (answers Q4).
4. Real idle (Q2 last row) is a different physical quantity from sim idle: measured joint speed < 0.06 rad/s
   at 60 Hz vs commanded target change < 7.5e-4 rad/s at 7.5 Hz. Not comparable; fig7a's idle panel
   (`make_fig7:44`) plots them side by side (0.54 vs 0.19).

## Q3. Is the idle / stop gap provenance rather than behaviour?

**idle_frac (action-based): yes, entirely.** `tape_stats.csv`: 114/114 machine tapes have idle_frac exactly
0.0000 (old 56, w3 58), human 0.193/0.197 with 1–2 tapes at 0. An action-tolerance sweep needs the npz
(cluster `baselines/matched_w3/{dH,dDP}/*.npz`, `matched_v2/…`) — not computable here. Recommended sweep on
`max|actions_delta[:, :6]|`: eps ∈ {1e-3, 1e-2, 3e-2, 0.1} normalized (= 1e-4 … 1e-2 rad/decision) plus the
same sweep on `|Δ actions[:, :6]|` (window-end targets). Prediction from the EEF data below: the gap collapses
between 1e-2 and 3e-2.

**strict-stop (EEF-based): the definition is provenance-safe but the gap sits exactly where a sampler jitter
floor would put it.** Computed from the committed `speed_series_w3.npz` (58+58 tapes, per-decision EEF step):

| threshold (mm/decision) | dH | dDP | d | ratio |
|---|---|---|---|---|
| 0.01 | 0.034 | 0.000 | +0.67 | ∞ |
| 0.05 | 0.101 | 0.001 | +1.60 | 88× |
| 0.1 | 0.153 | 0.009 | +2.01 | 17× |
| 0.2 | 0.237 | 0.052 | +2.02 | 4.5× |
| 0.5 (headline) | 0.365 | 0.221 | +1.15 | 1.6× |
| 1.0 | 0.437 | 0.372 | +0.50 | 1.2× |
| 2.0 | 0.503 | 0.483 | +0.17 | 1.0× |
| 5.0 | 0.634 | 0.634 | −0.01 | 1.0× |

Pooled low-speed histogram: human has 7.3 % of decisions below 0.03 mm and 16 % below 0.1 mm; machine has
0.01 % and 1.0 %. The machine's mass piles up in [0.1, 0.5) mm (23 % vs 22 % human) and [0.5, 1) mm (15 % vs
7 %). Median sub-0.5 mm speed: human 0.13 mm, machine 0.29 mm. No exact-zero EEF steps in either source.
Run structure: sub-2 mm "slow" runs — human 424 runs, mean 8.5 decisions; machine 419 runs, mean 8.7 decisions.
Sub-0.5 mm runs — human 345 × 7.7; machine 581 × 3.0 (fragmented by threshold crossings).

Reading: **both sources dwell equally often and equally long; the human's arm is frozen (target re-issued
bit-exactly, PD settles to <0.03 mm) while the machine's target jitters by ~0.2–1 mrad per decision (0.1–0.5 mm
at the tool)**, which is what a stochastic absolute-target sampler produces and what the delta adapter passes
through unchanged. "Creeps at uniform speed" (`WM_METRIC` A2.2) is not what the data show at 2 mm; "moving_speed_mean
7.3 vs 5.8 mm" (`tape_dynamics_metrics.py:167`) is partly the same artefact — the machine's 0.5–1 mm jitter
decisions are counted as "moving" and pull its moving mean down. Whether a world model cares about frozen-vs-
jittering dwells is a legitimate question, and A28 (dDPretimed inserts bit-exact holds via `SchedFollower`)
tests exactly it — but the paper should describe the mechanism as *rest-state exactness / target jitter*, not
as a difference in motion strategy, until a deterministic teacher (`--mode mode` or a delta-action policy) shows
the same floor.

## Q4. Pruner vs metric; why pruned ≈ 0.19 and not ≈ 0

`make_dp_pruned.py:8-12,73-81`: only frames before `j_pick − 150` (5 s at 30 Hz) are touched; a run of
consecutive frames with `max|Δa| < 1e-3` (7 cols) collapses to its first frame. Everything from 5 s before the
grant on is untouched, including the closure dwell (arm frozen, grip ramping) — the single largest source of
arm-only zeros in a pick-scope tape. Then `HumanFollower` re-records at repeat 4 and adds settle rows
(`raw_vs_pruned_tapes.csv`: trailing_idle mean 1.6 decisions old world, 0 in w3). So 0.19 is expected;
"leading_idle = 0 in both sets" (`FIGURES:205`) is likewise expected because the pruner keeps the first frame of
every run and the follower's first decision always moves. Consistent numerically: raw 0.25/0.27 → pruned
0.19/0.20 despite 22–30 % of source frames removed — the pruner removes pre-approach teleop pauses, the metric
mostly counts the grasp.

## Q5. Real tapes

1. `vel_cmd` is measured `/joint_states` position averaged per 60 Hz bin (`trial_reader.py:46-50`); not a
   velocity, not a command. `METHODS_draft:39,521` (CHECK 2) already records this, but
   `extract_real_tape_stats.py:2-5` ("60 Hz joint-position targets… commanded"), `FIGURES:113,117,128` ("commanded
   signal", "60 Hz command dithering"), `make_fig7:4-5`, and `WM_METRIC` caveat 4 ("real commanded path 3.5× longer")
   all say commanded. The "dithering" explanation of the 3.5× path length is therefore wrong in kind: the
   inflation is finite-difference ENCODER NOISE on ~1000 FK samples (no smoothing, `extract_real_tape_stats.py:57-64`),
   plus the fact that the real tape covers the whole trial (place + slide) while the sim tapes are pick-scope.
2. Real idle = `|Δq_meas| < 1e-3 rad` per 1/60 s, all 6 joints (`:69-70`), i.e. joint speed < 0.06 rad/s. Exact
   zeros are neither impossible nor ubiquitous (encoder quantisation); the 0.54 mean is dominated by the
   loose speed threshold, not by rest. Sim idle is target change < 7.5e-4 rad/s. Ratio ≈ 80×. Not comparable.
3. Clock: `trial_reader.py:31` bins at 60 Hz; the replay/collector feeds one bin per 30 Hz env step
   (`genesis_can_env.py:1,14`; `replay_harness.py:2` calls them "30 Hz commands"; `collect_all_classified.py:81,97`
   no stride). If the bins really are 1/60 s, every human sim tape is 2× time-dilated versus real, the follower's
   4 frames/decision = 1/15 s real per 1/7.5 s sim, and `real_tape_stats.csv` `dur_s = n/60` is inconsistent
   with the sim's clock. Does not affect human-vs-machine (both on the sim clock) but affects fig7 and any
   "seconds" axis (fig10a divides by 7.5). Settle with the bag topic rates (`trial_reader.py` prints bag runtime
   vs frame count) — not available on this box.

## Q6. Other pattern-matches to the known failure family

- The 7-column idle bug is disclosed and the CSV confirms it (`idle_frac_7col` ≤ 0.008 everywhere). Good.
- `make_dp_pruned.py:79` mixes units in one max: arm rad vs grip [0,1]; the docstring says "action delta < 1e-3
  rad". Conservative direction only (grip motion prevents collapse), disclose.
- `FIGURES:216` "1e-3 rad" (10× unit error) and `CLAUDE.md` "machine demos have 0 idle decisions" (reads as
  finding; it is a format property, cf. `results_core_matrix.md:58` "no idle frames by construction").
- Claim (c)'s raw-vs-pruned contrast also flips `max_sim_steps` 1200→2400 (column in
  `raw_vs_pruned_tapes.csv`), changes N (69/66 vs 60/60) and uses a different source recording (`V2_BUILD` §2,
  §A). fig11's "pruning is load-bearing for DP" therefore bundles pruning + horizon + 8 long tapes; the
  row-matched dHunpruned control (n=3, no penalty, `RESULTS:23-29`) is the only clean pruning-only test and it
  was null. `FIGURES:181` discloses "set-matched, not row-matched" but `RESULTS:32-40` reads causally.
- `tape_dynamics_metrics.py:189` `grip_cmd_flip` uses `sign(act[:,6])` on an ABSOLUTE grip target in [-1,1]
  (sign flip = crossing half-open), not a command flip. Not load-bearing.
- Old-world contract tapes carry settle rows (trailing_idle 1.6) that the w3 tapes do not — a small
  world-dependent idle offset unrelated to the human.

## Findings ranked by severity

1. **[HIGH] Machine idle_frac = 0 is by construction** (`record_demos.py:561-566`, `extract_tape_stats.py:31`;
   114/114 tapes exactly 0). Impact: claim (a) is not a behavioural finding. Fix: state it as a recording-format
   property, or replace with an action-tolerance sweep on the cluster npz and report where the gap closes.
2. **[HIGH] strict-stop gap is confined to < 0.3 mm/decision with identical dwell structure at 2 mm**
   (table above; `speed_series_w3.npz`). Impact: claim (b)'s wording ("teacher creeps at uniform speed", "moves
   faster while moving") overstates; the defensible statement is "human dwells are bit-exact rests, machine dwells
   jitter at the sampler's floor". Fix: add the run-structure numbers and the jitter reading to WM_METRIC A2;
   recompute `moving_speed_mean` with a 2 mm cut; optionally record 10 dDP tapes with `--mode mode` to measure the
   floor. A28 stays valid as the causal test of *tape* stops.
3. **[HIGH] fig7 mislabels measured joint states as commands and compares incomparable idle metrics**
   (`trial_reader.py:46-50` vs `extract_real_tape_stats.py:2-5,69-70`, `FIGURES:113-128`). Fix: relabel throughout;
   drop the idle panel or recompute real stops as FK-EEF displacement per matched physical interval with the
   same mm threshold after downsampling to the sim decision clock; smooth before path length; scope the real
   tape to the pick phase for the multipliers.
4. **[MED] Unit error in the disclosed threshold** (`FIGURES:216` "1e-3 rad" → 1e-4 rad/decision) and fig12b title.
5. **[MED] Pruner ≠ metric** (Q2/Q4); document that pruned-set idle is grasp dwell + margin, and that
   `make_dp_pruned.py` thresholds 7 mixed-unit columns.
6. **[MED] Claim (c)'s DP-penalty attribution to idle mass is unsupported** — common-IC idle diff 0.06, row
   ratio 1.16–1.20, horizon/N confounds; keep "removes time not space" (coverage 578 ≡ 577), drop the causal
   reading in `RESULTS:32-40` or run a row-matched/horizon-matched leg.
7. **[LOW] 60 Hz vs 30 Hz clock** unresolved in docs; settle from bag rates.

## What this checkout cannot settle
- Action-space tolerance sweep and DP jitter magnitude (std of `Δactions[:, :6]` on EEF-still decisions):
  `baselines/matched_w3/{dH,dDP}/*.npz`, `matched_v2/…` on the cluster.
- Whether the frozen dH sets were recorded with `--arrival either` for ALL tapes (V2_BUILD says yes for v2 and
  the block of record): `baselines/demos_v1/dH*/manifest.json` (`arrival` in per-rollout stats).
- Real frame rate: bag message rates / `trial_reader.py` runtime prints.
