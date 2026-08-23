# PRE-LAUNCH AUDIT — final round robin, DP + RLPD block (gate 1 of PREREG §11)

Auditor: fresh-context agent, read-only, 2026-08-23 ~14:10 EDT. Local repo HEAD at audit time
70897ea (the task said 655223d; 33d5bc9/70897ea landed mid-audit — see BLOCKER 1); cluster repo
/cluster/tufts/shortlab/jstale02/genesis_pickaplace at 33d5bc9 (pulled 14:05:59), sets built at
0e1fda1. Cluster reads went through `~/bin/ccmd` (rows appended to SESSION_LOG by that tool).
Spec = paper/PREREG_final_round_robin_2026-08-23.md (§2/§3/§4/§5/§8, A1–A3).

## Verdict

Launchable **after** two zero-GPU-cost actions (BLOCKERS 1–2). Code paths of both launch
commands implement §2/§5 knob-for-knob (traced below); the matched sets satisfy §3.2/A1–A3; the
teachers are the ones the rules name. The remaining findings are disclosures and cheap fixes.

| # | item | result |
|---|---|---|
| 1 | launch commands implement §2/§5 (clock, horizons, reward, demo format, harness, K=5, registry, node) | **PASS** (with the DP-policy-on-GPU deviation from §5's "CPU" wording — W1) |
| 2 | sets match §3.2 / A1–A3 (N, IC multisets, shas, contract, lerobot provenance, fps, census) | **PASS** (census gaps W5–W7) |
| 3 | teacher provenance (dDP rule as written; dR2D = A2; `--verify`; verify-rejects excluded) | **PASS** (stale dDP_fails manifest W4) |
| 4 | eval IC file (sel = historic 15, hold disjoint, rnd 30; overlap with training ICs) | **PASS** — but "hold" is NOT held-out in the IC sense (W8) |
| 5 | registry refuse/warn; collisions with today's pilots/smokes | **PASS** — no full-key or semantic-key collision possible (different wave/steps/dataset_root) |
| 6 | session log reconstructibility vs PREREG | **PARTIAL** — N1–N4 |

## BLOCKERS (must resolve before `sbatch`; no GPU cost)

**B1. The matched sets are not final by the team's own note, and an in-place rebuild would break
running jobs.** SESSION_LOG 14:05:59/NOTE 14:06 (commit 70897ea): `dH_either` re-record (recorder
`--arrival either`, 33d5bc9) was submitted with the rule "if kept ≥55 it replaces dH for the block
(sets rebuilt at N=min)". Every rebuild so far was `rm -rf baselines/matched_v1; make_matched_sets
...` (SESSION_LOG 13:51:39, 13:52:20, 13:53:25). sbatch_dp trains FROM `$DEMO_ROOT/<ARM>/lerobot`
for ~3 h (cluster/sbatch_dp.sh:219,442-448) and sbatch_rlpd gates/fingerprints `$DEMO_ROOT/<ARM>`
at job start (cluster/sbatch_rlpd.sh:151-177, 220-223); deleting matched_v1 under queued/running
jobs = crashed DP runs and a block split across two demo shas. Resolve ONE of: (a) decide now that
the block uses the current 51-sets (then dH_either is a post-hoc control only), or (b) wait for
dH_either (~1 h batch), rebuild into a NEW root (e.g. `baselines/matched_v2`) and launch with
`DEMO_ROOT=baselines/matched_v2` — never rebuild matched_v1 in place once anything is queued.
Note for (b): the dDP teacher (DP-r4 pilot s0) was trained on the 51 arrival=meas tapes (verified:
matched_v1_pilot/dH/lerobot/genesis_source.json == matched_v1/dH/episode_list.txt, sha
58fd89bce03ab13d), so §3.1's "teacher = DP trained on dH in THIS block" would no longer be literally
true — needs an amendment sentence, and the dR2D/dDP successes on the 6 newly-kept human uids
already exist in demos_v1 (64/66 each), so a rebuild needs no new model recording.

**B2. Append the amendments for deviations already adopted (PREREG says frozen + appended, never
silently diverged).** All are disclosure-only; none changes code:
- A4 — eval/record POLICY device: §5 says "CPU"; DP policy runs on the GPU when visible
  (cluster/eval_sweep.sh:121-126, baselines/dp_runner.py:39; recorder baselines/record_demos.py
  DPTeacher dev selection; dDP recorded with GRES=gpu:1, SESSION_LOG 13:37:43). Sim is CPU in all
  cases; sac policy CPU. Reason: 6 s/DDPM query on CPU (SESSION_LOG #22).
- A5 — §8 pilots not run as written: (i) random-teacher negctl ran n=3 (kept 0/3; SESSION_LOG #5),
  spec says 30; (ii) the dp-adapter hold-4 yield pilot (30 rollouts) was skipped — the full harvest's
  first-attempt rate stands in (recorder_manifest_dDP.json records: 66 first attempts, 14 fails of
  which 6 are the two lying-can ICs); (iii) the RLPD-r4 dH dense pilot (3 seeds, bar pooled hold
  ≥0.16 and ≥1 seed ≥4/15) has NOT been run and the runbook launches the RLPD block directly
  (SESSION_RUNBOOK Phase 5) — state that dH dense seeds 10-13 double as that pilot and what
  happens on failure (§8: repeat 1 for RLPD + amendment → the RLPD block restarts).
- A6 (only if B1 option b) — §4.2 fixes the human arrival test as ‖q_meas−ref_j‖∞<tol;
  `--arrival either` adds "OR ‖target_env−cmd_j‖∞<tol" (baselines/record_demos.py 33d5bc9 diff).
- A7 — selection tie rule "ties → later checkpoint" (cluster/sbatch_rlpd.sh:261,
  cluster/dp_select_confirm.sh:29); §5 says only "best sel score".

## WARNINGS (launch, but disclose / fix after)

W1. **Before the flood, run one dense RLPD smoke** (`ARM=dH SEED=0 STEPS=3000 REWARD=dense WAVE=smoke
DEMO_ROOT=baselines/matched_v1_pilot`): the sparse pipeline is proven end-to-end (rlpd_2825918.out:
REGISTRY → [native-demos] 6491 transitions → ckpt_020..100 → 5 SELECT-RESULT → SWEEP-HEADLINE → JOB
DONE), but the dense path (`--pick-shaping on` → env φ(terminal)=0, baselines/rl/full_env.py:552-554;
`--demo-shaping auto`→on, baselines/rl/train_rlpd.py:204-213; native_demo_transitions shaping,
baselines/rl/train_sacfd_full.py:535-544) has only unit tests (baselines/tests/test_terminal_guard.py
case 7, passes locally) and no live run.

W2. **Explicit > auto for two RLPD knobs**: sbatch_rlpd.sh:187-192 does not pass `--demo-shaping`
(auto → on iff native ∧ pick_shaping on) nor `--pick-shaping-terminal-zero` (default on). Both are
printed in [cfg] and written to the sidecar (train_rlpd.py:382-399), so they are auditable, but
PREREG §2's "no silent defaults" wants them on the command line and in REG_KNOBS
(`demo_shaping=on|off`, `phi_terminal_zero=on`). Cosmetic; does not change behaviour.

W3. **"every sbatch asserts the sha" is weaker than stated**: both launchers READ
`manifest.content_sha256` (sbatch_rlpd.sh:170-175, sbatch_dp.sh:300-304) and assert only
`n_kept == #npz`; they do not recompute the content hash against the files. The registry adds a
(filename,size) fingerprint (cluster/run_registry.py:57-67). Recompute-and-compare is a cheap
post-launch hardening (51 npz/arm).

W4. **dDP_fails manifest is stale and the verify-reject move is only partially logged.** The 2
verify-rejected successes (103004 ic276, 103010 ic317; recorder writes them to the fails dir with
label=success, baselines/record_demos.py:716-724) were moved to `baselines/demos_v1/dDP_fails_verifyrej/`
(14 npz remain; make_matched_sets.py:216 `read_dir(..., 'fail')` would otherwise FATAL), but
`demos_v1/dDP_fails/manifest.json` still lists 16 files / records_complete False, and the mv
command is truncated in SESSION_LOG (13:51:16; ccmd cuts at 600 chars). Fix: re-run
`record_demos.py --teacher dp --merge --outdir baselines/demos_v1/dDP`; write the mv + destination
into the session log. Both verify-rejected ICs were re-rolled and passed (dDP histogram has 276,
317 ×1), so the primary dDP set is unaffected.

W5. **Fails-arm composition differs from §3.3's description.** The 14 dDP fails = 6 lying-can-at-t0
tapes (ic 234 ×3, 318 ×3: tilt 90° & grip open → tip rule fires at decision 1, n=1) + 8
cap-truncated no-picks (n=300) — census "tapes tipped at t0 (lying-can IC) 6", "tapes ending
truncated 8". None is a grasp-time tip. Fail share 0.215 of episodes / 0.277 of transitions
(≤0.30 cap). P-MECH's predictions were written for DP grasp failures; disclose, and consider
excluding the two lying-can ICs from the fails pool (they are IC artifacts, identical for every
teacher). Not launching today.

W6. **Census gaps vs §10/A2**: (a) "human re-record: tapes with dilation stat 0 / p50 —" is blank
because the census keys records by `name`/`uid` (analysis/characterize_demo_sets.py:104-109) while
the recorder writes `rollout`/`file`; the data exist (recorder_manifest_dH.json: all 51 kept tapes
have dilation, p50 1.037, max 1.359) — fix the lookup, re-render; (b) A2 says the provisional
teacher is "logged … in the census" — census.md does not name the s51 teacher (only the set
manifests do, `source_manifests…checkpoint`); (c) "tapes at max len (timed out)" = 1 for each
primary set is just "len == max len in set" (characterize_demo_sets.py:265), not a timeout — the
"ending truncated (cap) 0" row is the honest one.

W7. **Merged recorder manifests drop the per-shard summary fields** (`teacher_success_rate_first_attempt`,
`rejected_by_verify`, `yield_frac`; record_demos.py merge():543-575 keeps configs+records only);
§3.1 wants the teacher success rate in the manifest. Recoverable from `records`; add to merge().

W8. **"hold" is a confirmation split, not a held-out-IC set — and it is MORE in-distribution than
"sel".** From baselines/eval_ics.json vs manifest_dH.json ic histogram: sel ∩ training-IC uids = 7/15
(232,235,242,243,247,248,251); hold ∩ training = 11/15 (all but 256,294,295,331). Of sel's 8
non-training uids, 4 are dH recorder failures (234 lying-can, 245, 246, 250) and 4 are non-IL
success uids (236,237,239,244). For RLPD, every sel/hold uid is also a training-RESET IC
(FullTaskEnv.reset samples all 61 placements, baselines/rl/full_env.py:468-469). Only `rnd` is
novel for every learner. Paper text must not call hold "held-out"; expect hold ≥ sel by
construction. Also 234 (lying can) sits in sel: unwinnable under the training MDP, same for all arms.

W9. **Preemption**: both launchers use `-p gpu,preempt --requeue` (sbatch_rlpd.sh:63-64,
sbatch_dp.sh:155-156). DP resumes from `checkpoints/last` (sbatch_dp.sh:436-439); RLPD has no
resume — a preempted RLPD job restarts training from scratch (registry skipped on
SLURM_RESTART_COUNT>0, same OUT, wandb run name duplicated). Results stay valid (sweep runs only
after a full training) but wall time doubles; watch `squeue` for requeues.

W10. **Node class differs between learners**: DP `--constraint="l40s|a100"` (sbatch_dp.sh:161), RLPD
`l40s|a100|l40|h200` (sbatch_rlpd.sh:66). Recorded per job/eval (NODE_CLASS, sweep json `node`); keep
it in the per-cell report (§7).

## NOTES (no action)

N1. SESSION_LOG rows 8-14 (reconstructed times) are mutually inconsistent: the pilot dataset/launch
(rows 13-14, 06:25/06:27) depend on the 51st dH tape + merged manifest that rows 10-11 place at
06:33-06:37. Disk settles it: matched_v1_pilot/dH/manifest.json mtime 06:26:35, lerobot info
06:26:54, DP pilot start 06:27:19 EDT, content = the same 51 tapes as matched_v1/dH (sha
58fd89bce03ab13d). The teacher trained on exactly the block's dH tapes.

N2. 28 of the 66 dH recorder records are `source: reconstructed_from_log` (shard manifests were
overwritten before fix e2b3bd2); the tapes themselves carry every stamp (contract scalars) and all
51 kept tapes have a dilation record. Keep `baselines/demos_v1/logs/dH_shard*.log`.

N3. Recorder code spans 91cd7b1 → bde726d (dH), 3167d12 → f6d937f (dR2D), 1dcd268 (dDP); every tape
stamps its git_sha. Diffs in that range touch IC-plan/manifest/GPU-device handling only (git log
91cd7b1^..655223d -- baselines/record_demos.py); the MDP (FullTaskEnv ctor, record_demos.py:187-207)
is unchanged. Sets built at 0e1fda1; launch will be at ≥33d5bc9 (recorder-only diff) — "one commit
hash for the block" holds for the trainers.

N4. Teacher circularity as predicted: dDP teacher = DP-r4 pilot s0 ckpt 020000 — rule "median in-dist
seed, 2 seeds → the lower" applied as written (s0 sel 13/15 @020000, s1 14/15 @040000; dp_pilot
select logs) — trained on the identical 51 dH tapes; dR2D teacher = A2's s51 BEST_selected.pt
(recorder_manifest_dR2Dprov.json configs[*].checkpoint/teacher_ckpt). Both teachers reached 64/66
ICs (only 234/318 lying-can missing); matching discarded their 13 successes on dH-fail uids.

N5. DP at fps 7.5 keeps lerobot defaults n_obs_steps 2 / horizon 16 / n_action_steps 8 (pilot
config.json): the replan interval is now 8 decisions = 32 sim steps ≈ 1.07 s (was 0.27 s at stride
1) and horizon 16 ≈ a whole dR2D episode (p50 17 decisions). Same for all three sources in the
block; different from the 08-19 DP rows.

N6. Lying-can ICs 234/318 are in FullTaskEnv.success_uids, so ~2/61 RLPD training resets terminate
at decision 1 (tip rule). Harmless, identical across arms.

N7. Prior RLPD 100k-decision runs at repeat 1 took 1.7-2.5 h (sacct 2667183/4); at repeat 4 expect
~4-5 h + ~40 min sweep — inside the 12 h limit.

## Evidence trace — item 1 (launch commands → §2/§5)

| knob | DP (`ARM=$A SEED=$S WAVE=final sbatch cluster/sbatch_dp.sh`) | RLPD (`… REWARD=$R … sbatch_rlpd.sh`) |
|---|---|---|
| action_repeat 4 | sbatch_dp.sh:190 → fps gate 30/4=7.5 (:326-330, verified `PROVENANCE-OK … fps=7.5`) → dp_sidecar action_repeat (:466-469) → eval_sweep.sh:91-99 → wandb_eval.py:256-271 hold-4 (refuses mismatch :268-271) | sbatch_rlpd.sh:81,188 → train_rlpd.py:235-238 FullTaskEnv(action_repeat) asserted :246-248 → sidecar :382 → eval_sweep.sh:71-83 → wandb_eval.py:206-235 |
| train horizon 1200 sim | n/a (offline) | sbatch_rlpd.sh:82,188 `--train-max-steps` → full_env.py:363,686 (`truncated = _t ≥ max_steps`); 300 decisions printed :259 |
| eval horizon 1200 sim | sbatch_dp.sh:191,349 → dp_select_confirm.sh:16 → eval_sweep.sh:43 → wandb_eval.py:127 GenesisCanEnv(max_steps) | sbatch_rlpd.sh:83,201 → same path |
| reward sparse/dense, φ(terminal)=0, γ match, demo shaping from eef_pos | n/a | sbatch_rlpd.sh:96-99,181 → `--pick-shaping on` → full_env.py:327-337,540-554 (γ=args.gamma :242, terminal_zero :552); demo half: train_rlpd.py:204-213,304-308 → train_sacfd_full.py:535-544 (same `pick_shaping_phi`, full_env.py:92-100; can = states[:,8:11], eef = tape eef_pos = genv.tool_pos(), record_demos.py:233-234) |
| demo format + terminal guard | native lerobot from (states, actions) at fps 7.5 (convert_to_lerobot.py:41-52,103-114); genesis_source.json == episode_list.txt (verified on cluster, all three arms) | `--demo-format native` (sbatch_rlpd.sh:182) → native_demo_transitions (train_sacfd_full.py:494-559): done=terminated[t], truncation bootstraps, stamps asserted :518-522, terminal-before-last refused :532 |
| eval ICs + fresh process | eval_sweep.sh:110-132 one `wandb_eval.py … --ic-file --ic-set --ic-index k` per episode; `--seed k` (DP sampled, seeded; wandb_eval.py:103-108,411) | same; sac deterministic (wandb_eval.py:211) |
| K=5 archive, selection/confirmation | lerobot `--save_freq=STEPS/5` (sbatch_dp.sh:202,446) → 020000…100000 (pilot: 5 sidecars) → dp_select_confirm.sh:23-38 sel → best (ties later) → hold,rnd; final too | train_rlpd.py ArchiveCheckpointCallback (:434-457) ckpt_020…100 → sbatch_rlpd.sh:243-272 |
| registry knobs, register at start | REG_KNOBS sbatch_dp.sh:346-348 incl. demo_sha, action_repeat, eval_horizon, save_freq, wave; check+register before conda :372-377 | REG_KNOBS sbatch_rlpd.sh:195-199 incl. reward, demo_sha, horizons, budget_unit; :219-224 |
| node class | NODE_CLASS :345 → sidecar, headline; wandb_eval.py:406-410 per episode | sbatch_rlpd.sh:185; same |
| denominators / missing | eval_sweep.sh:138-166 (`missing never 0`), selection reads -1 if incomplete (sbatch_rlpd.sh:259, dp_select_confirm.sh:27) | same |
| CPUs under `-n 8` | eval_sweep.sh:60-66 SLURM_CPUS_ON_NODE=8 → max_jobs 4 (verified rlpd_2825918.out "[sweep] cpus=8 -> max_jobs=4") | same |
| DP hold-4 integrator vs _step_once | wandb_eval.py:284-295 `sp=clip(target+a·cap)`, `target=q+clip(sp−q,±leash)`, target seeded from measured q, grip absolute — identical to full_env.py:557-575 target branch and to the recorder's DPTeacher (record_demos.py:443-448) | sac branch wandb_eval.py:206-230 identical |

DRYRUN on the cluster at 33d5bc9 for `ARM=dDP SEED=10 WAVE=final` (dp) and `ARM=dR2D SEED=10
REWARD=dense WAVE=final` (rlpd) printed the expected plans (DEMO-SHA 9be55219…/fe93dbdd… from
manifests; PROVENANCE-OK fps 7.5; REG_KNOBS as above; OUT dirs dp_final/…, rlpd_final_…_dense).

## Evidence trace — item 2 (sets)

MATCHED_SETS.json: N=51, cap 66, seed 0, git 0e1fda1, source_counts dH 51 / dDP 64 / dR2D 64,
`ic_multiset_identical: true`, unmatched null. Manifests: dH/dDP/dR2D ic_uid_histogram identical
(51 uids ×1; recomputed), n_kept==chosen==51==npz on disk (cluster ls), contract v1, role "primary
success-only", n_fail 0; shas 58fd89bc… / 9be55219… / fe93dbdd…; fails arms 65 = 51 + 14 (same 14
files in both, renamed 1xxxxx→5xxxxx, `renamed_fail_tapes` map present). lerobot: fps 7.5, 51
episodes, frames 6491/6294/861 == decisions_total; features state(8)+environment_state(9)+action(7),
no cameras (state-only DP, as in every prior DP row). Census: contract rows populated (repeat stamp
[4], terminated 51/51/51, truncated 0, terminal-before-last 0, post-termination frames 0 in the
primaries, eef_pos present), dilation row blank (W6).

## Evidence trace — items 3-5

3: SELECT-RESULT/DP-HEADLINE lines (dp_pilot_dH_s{0,1}_select.log) → s0 020000 sel 13/15, s1
040000 14/15 → lower = s0 ✓; recorder_manifest_dDP.json configs[*].checkpoint =
`baselines/outputs/dp_pilot/dH_DP_s0/checkpoints/020000/pretrained_model`, verify true, attempts 3,
mode sample, ic_from_tape true; dp_sidecar at that checkpoint: action_repeat 4, demo_sha
58fd89bce03ab13d. dR2D: recorder_manifest_dR2Dprov.json checkpoint = `…/pick_v5d4c_delta_shaped_dH_s51/BEST_selected.pt`
(A2). Verify: dDP 2 rejects (re-rolled, both ICs present), dR2D 1 reject; none in any set.
4: `make_eval_ics.py --check` MATCHES (local and cluster SESSION_LOG #4); sel == SEL_HISTORIC;
sel∩hold=∅; rnd n=30. 5: cluster RUN_REGISTRY.jsonl (47 lines): sbatch_dp.sh seeds {0,1,2},
sbatch_rlpd.sh {0..8}; today's lines carry wave=pilot/smoke, steps 3000 (smoke), dataset_root
matched_v1_pilot → different knobs → neither REFUSE nor WARN for WAVE=final seeds 10-14.
