# Adversarial review — ROBOMIMIC LEG, LAUNCHERS AND OPERATIONS (2026-09-07)

*Brief: `~/wm_fix_2026-09-03/agent_brief_adversarial_common.md`. Mandate: try to break the robomimic RLPD result
(MH200 0.455 v MG200s 0.147, p 0.008) and the queued A2 controls; audit the converters, env, adapters, evaluators,
the launchers, the QOS move, the post-hoc eval scripts, and the 10:26 mis-fire. Read-only: nothing was modified,
no job was submitted or touched; every cluster command was a read (`sacct`, `scontrol`, `cat`, and three CPU python
probes that printed to stdout and wrote no files).*

Severity: **S1** changes a number or claim of record · **S2** weakens a claim · **S3** fragility that could bite the
runs now queued · **S4** hygiene. Status: **CONFIRMED** = I reproduced it · **PLAUSIBLE** = reasoned, not run.

---

## S1 findings

### S1-1. The one non-source difference no queued control can remove: MG's action process (CONFIRMED)
`baselines/robomimic/convert_arms.py:46` copies the hdf5 actions verbatim, so each arm carries its generator's
action statistics into RLPD's demo half. Measured on the arms' own `rlpd/transitions.npz` (cluster,
`$LAB/robomimic_data/arms/<ARM>/rlpd/transitions.npz`, robo_venv, 2026-09-07):

| arm | rows | mean \|a[0:6]\| | frac \|a[0:6]\| ≥ 0.99 | mean per-step \|Δa[0:6]\| | frac \|gripper\| ≥ 0.99 | gripper mean |
|---|---|---|---|---|---|---|
| MH200 | 41,134 | **0.144** | 2.4 % | **0.044** | **1.000** | −0.168 |
| PH200 | 22,400 | 0.236 | 7.5 % | 0.050 | 1.000 | −0.088 |
| MG200s | 16,501 | **0.664** | 12.2 % | **0.272** | **0.293** | +0.177 |

MG demonstrations command **4.6× larger** OSC deltas, are **6.2× less smooth** step to step, and use a *continuous*
gripper channel where every human tape uses a binary ±1 toggle. C1 (`MGall`), C2 (`MG718s`) and C3 (`MG200s@300k`)
are all drawn from the same SAC rollouts and therefore have **identical** action statistics to MG200s. So the A2
decision rule ("if all three stay ≥ 0.15 below MH200 (p < 0.05) → falsifier (a) stands as a source effect",
`ROBOMIMIC_PLAN_2026-09-05.md:135-138`) will attribute to *demonstration source* an effect that is equally
consistent with *action amplitude / smoothness / gripper parameterisation*. A reviewer who has read
`OG_DEMO_ACTION_DISCRETENESS_2026-09-06.md` and `DISCRETE_ACTION_REPLAY_2026-09-06.md` — where this project already
established that action discreteness is a first-class variable — will ask this immediately.

**Scenario:** all three controls read out below MH200; the paper writes "RLPD is source-sensitive on an independent
machine generator"; the true statement available from the data is "RLPD learns worse from bang-bang, high-frequency
demonstrations than from smooth human ones, and the SAC generator produces the former".

**Smallest fix (pick one, all cheap):** (a) add the covariate table above to the log/plan and re-word the conclusion
to name the mechanism; (b) register a fourth control that holds the action process and changes the source —
Can-Paired good-half vs bad-half (same operator, same action process, 100 v 100, already downloaded and manifested
as `PH200pb`'s `extra`); (c) an MH200 arm whose actions are passed through a saturating/roughening filter matched to
MG's statistics (no re-execution needed for RLPD: the critic only needs (s,a,r,s′) tuples, but the actions would no
longer be the ones that produced s′ — so (b) is the honest one).

### S1-2. The falsifier that "fired" is not, as registered, the contrast that was measured (CONFIRMED)
Registered falsifier (a), `ROBOMIMIC_PLAN_2026-09-05.md:64`: "**PH200 ≫ MG200s** for RLPD or r2dreamer (Δ ≥ 0.15,
p < 0.05)". RLPD has **no PH200 data**: jobs 3346687–94 (`rlpd PH200 s0-7`, `SUBMITTED.log`) started at 10:26:24 and
were cancelled at 10:26:26 (sacct: Elapsed 00:00:02); the 10:53 matrix is MH200 + MG200s only. The contrast that
fired, MH200 v MG200s, is the *third* contrast in §5 and was re-pointed to falsifier (a) by amendment A3
(`ROBOMIMIC_PLAN_2026-09-05.md:143-145`), whose own header says "user, 2026-09-07 10:45; **recorded 13:45**" — i.e.
the document that redefines which contrast carries the falsifier was written 15 minutes *after* the 13:30 readout.
Mitigating: the user's design instruction (drop PH) predates the runs, and MH v MG was a registered contrast.
`REVIEW_GUIDE_2026-09-07.md:110` still states the falsifier as "**MG vs PH**", so the guide and the plan now
disagree about which contrast is of record.

**Fix:** state the result as "the pre-registered MH200 v MG200s contrast", not "the registered falsifier fired",
until PH200 RLPD is run (8 × ~2 GPU-h); and correct REVIEW_GUIDE line 110.

---

## S2 findings

### S2-1. Registered gate G2b (no-demo RLPD ≤ 0.10) was never run, and it is the control this result most needs (CONFIRMED)
Plan §5 G2: "**negative controls before any readout** — random policy ≤ 2/50; **RLPD and r2dreamer WITHOUT demos at
the budget of record ≤ 0.10**". Only G2a exists (`robomimic_data/eval_random_bank50/metrics.json`, 0/50). I searched
the whole runs tree: no `*none*` run directory under `$LAB/robomimic_runs/{rlpd,r2d,dp}`, and no slurm `.out`
containing `arm=none`. The launcher already supports it (`sbatch_rlpd_robo.sh:25,27` accept `ARM=none`;
`train_rlpd_robosuite.py:70-74`).

This is not a formality. MG200s scores 0.147 with 0/8 seeds ≥ 0.5 (G3 failed). Without a demo-free RLPD number at
100k decisions we cannot say whether MG demonstrations are *worse than human* or simply *worth nothing* — the two
readings support different sentences in the paper, and only the second is compatible with "the online learner makes
its own data". Cost: 3–8 runs at ~2 GPU-h.

Caveat on the control as implemented: `train_rlpd_robosuite.py:73` feeds a single all-zero transition as the demo
half, so "no demos" is really "128 copies of one informationless transition per batch" — disclosed in the code
comment, but it must also be disclosed wherever the number is reported.

### S2-2. The effect is significant but its size is not established at the falsifier's ≥ 0.15 threshold (CONFIRMED)
Recomputed from the run files (`rlpd_<ARM>_s<k>/eval_bank50_{mode,sample}/metrics.json`; all 16 carry
`bank_sha256 72b75550…` and `ckpt_step 100000`): MH200 mode [27,44,30,1,20,9,24,27] = 0.455, MG200s
[10,2,2,13,6,12,9,5] = 0.1475, Δ = 0.3075, **exact two-sided permutation over all 12,870 splits p = 0.00777**;
sample 0.4575 v 0.1675, Δ = 0.290, p = 0.01103. The published p-values reproduce exactly.

But MH200's per-seed sd is **0.263** (1/50 to 44/50) — 3.3× the 0.08 the registration assumed when it sized n = 8
and its TOST margin (`plan §5`, "TOST passes at n = 8 if per-seed sd ≤ 0.08"). Interval estimates:
bootstrap 95 % CI on Δ = **[0.125, 0.483]** (200k resamples); Welch 95 % CI = **[0.082, 0.533]**. The falsifier
clause conjoins "Δ ≥ 0.15" with "p < 0.05"; only the point estimate meets the first. Dropping the dead MH seed (s3,
1/50) makes it worse for the null, not better (Δ 0.370, p 0.00093), so the dead seed is not driving the result —
but the *magnitude* claim should be reported with its interval, not as "+0.307".

### S2-3. The registered t0-placement covariate was never computed, and the caveat it exists to test is wrong (CONFIRMED)
Plan §3/§5 register "can-xy at t0 per source (2-D histogram overlap)" as the substitute for matched ICs, and the
13:30 readout carries the caveat "**MG starts come from SAC's own reset distribution**" (also `plan §6` risk 2 and
`REVIEW_GUIDE:107`). Nobody computed it. I did (robo_venv, world `CAN_POS = state[16:19]`, over each arm's manifest
demo list, plus `bank_can50.npz`'s `can_xy0`):

| set | n | x min/med/max | y min/med/max | mean (x, y) | sd (x, y) |
|---|---|---|---|---|---|
| MH200 | 200 | −0.018 / 0.093 / 0.219 | −0.417 / −0.246 / −0.082 | (0.095, −0.250) | (0.073, 0.102) |
| MG200s | 200 | −0.020 / 0.104 / 0.218 | −0.416 / −0.237 / −0.085 | (0.098, −0.241) | (0.072, 0.097) |
| PH200 | 200 | −0.020 / 0.099 / 0.219 | −0.419 / −0.260 / −0.082 | (0.099, −0.259) | (0.071, 0.100) |
| bank50 | 50 | −0.006 / 0.122 / 0.215 | −0.419 / −0.238 / −0.084 | (0.111, −0.236) | (0.067, 0.098) |

The three sources and the eval bank draw from the **same** placement distribution — as they must, since the MG
rollouts were generated in the same robosuite env with the same `UniformRandomSampler`. The caveat is unsupported
and should be **deleted** (it currently weakens a result it has no right to weaken); the table above is the
registered covariate and should be pasted into the log before the controls read out.

### S2-4. The demo subsample is a fixed nuisance shared by all 8 seeds (CONFIRMED by construction)
`make_arms.py:82,94` draw MH200 (200 of 300) and MG200s (200 of 718) once, with `seed 0`, and every learner seed
trains on that same draw. The 8 v 8 permutation test therefore conditions on the subsample: its CI contains
learner-seed variance only, none of the "which 200 tapes" variance. An unlucky MG200s draw is inherited by all 8
seeds. C2 (all 718 MG tapes) removes this for MG; **nothing removes it for MH200**, which remains a single
200-of-300 draw. Disclose, or (cheap) re-draw MH200 and MG200s under a second seed for one or two learner seeds.

### S2-5. MH200 over-weights the two "better" operators, undisclosed (CONFIRMED)
`make_arms.py:79-85`: `quota = [34, 34, 33, 33, 33, 33]` is zipped against `sorted(ops.items())`, and the mask keys
sort alphabetically. The manifest's own `per_operator` field confirms the result:
`{better_operator_1: 34, better_operator_2: 34, okay_operator_1: 33, okay_operator_2: 33, worse_operator_1: 33,
worse_operator_2: 33}` → **68 better / 66 okay / 66 worse**. Small, but it is a skew of the human arm toward the
skilled operators on precisely the quality axis the leg is testing, and it appears in no document. Fix: one line in
the plan's disclosures, or `quota` assigned round-robin over a shuffled operator order.

### S2-6. The known t=0 observation artefact is 2.5× denser in MG200s than in MH200 (CONFIRMED)
`robo_common.py:38-40` records that the files' t=0 rows carry robosuite's empty-cache artefact in `object[0:7]`, and
the log concludes it is "identical in PH and MH (one generator), so it cannot bias the source comparison". That is
true of its *presence*, not its *density*. One corrupt row per tape, so from the transitions files: MG200s
200/16,501 = **1.21 %** of rows, MH200 200/41,134 = **0.49 %**, PH200 0.89 %. Every learner sees the artefact 2.5×
more often in the MG arm, and always at exactly the state each evaluation episode starts from. For DP and BC-RNN it
is additionally a train/eval mismatch: the online path re-reads `env.get_observation()` (`robo_common.py:126-130`),
so at evaluation t=0 the can→eef block is *correct*, which the policy has never seen at that state. Direction of the
bias: against MG. Disclose per-arm density; the honest fix (recomputing row 0's relative block) is a re-encoding the
plan deliberately forbids, so disclosure is the right call — but the current wording overstates the neutrality.

### S2-7. "The 72-job mis-fire was cancelled before any job started" is false (CONFIRMED)
`ROBOMIMIC_PLAN_2026-09-05.md:145` (A3). sacct: **16 of the 72 jobs started** — 3346687–3346702 (rlpd PH200 s0-7 and
rlpd MH200 s0-7), Submit 10:26:0x, **Start 10:26:24, End 10:26:26, Elapsed 00:00:02**, nodes assigned
(pax047/064/141/151/152/153/110); the remaining 56 never left PENDING ("None assigned"). Their own slurm outs
confirm it (`robo_rlpd_3346695.out`: `arm=MH200 seed=0 … out=…/rlpd_MH200_s0 restart=0`, then
`slurmstepd: … CANCELLED`). The 11:00 log entry elsewhere in the same document *does* record that
3346687–94 left a train.log, so the two statements of record contradict each other.

**No contamination of the readout** (verified: `rlpd_MH200_s0/` contains only 11:02–11:43 artefacts from the real
10:53 run, and all 16 primary runs report `restart=0`). But the safety claim matters, because
`cluster/robomimic/sbatch_dp_robo.sh:41` executes `rm -rf "$OUT"` on any non-restart start: had the mis-fire's DP
jobs been scheduled, they would have deleted existing DP run directories two seconds before being cancelled. Also
worth recording precisely: the mis-fire went **through** the `GO=1` gate (it wrote `SUBMITTED.log` lines), so it was
not a repeat of the `GO=0` leak fixed in dd2fabe — it was a correct submission of a superseded arm list.

---

## S3 findings (fragility for the runs now queued)

### S3-1. `submit_primary.sh`'s G0 gate is weaker than the registered gate (CONFIRMED)
`cluster/robomimic/submit_primary.sh:20` gates on `g0_report.json`'s `verdict` field. The report of record is
`{"n": 10, "need": 4, "n_pass": 7, "verdict": "PASS"}` — i.e. `g0_replay.py` was run with `--n 10` while keeping the
default `--need 4`, so the automated gate passes at **4/10 = 40 %**, where the registered clause is **4/5 = 80 %**.
The registered form does hold on this data (first five tapes `pass_` = T,T,F,T,T = 4/5), so no claim is wrong today;
the gate simply would not have caught a failure. Fix: `--need $(( (n*4+4)/5 ))`, or gate on `n_pass/n ≥ 0.8`.

### S3-2. `wmfix_s2.sbatch` / `wmfix_full.sbatch` throw away completed 6-hour trainings (CONFIRMED)
`wmfix_full.sbatch:58-60` (identical lines in `wmfix_s2.sbatch:56-58`) hard-code
`SLURM_OUT=$W/slurm/wmfix_<name>_${SLURM_JOB_ID}.out` and then `n=$(grep -c '\[sim-variant\] …' $SLURM_OUT || true)`.
When the string does not match the job's real `%x_%j.out` name, `grep` writes to stderr and `n` is **empty**, so
`[ "$n" -ge 1 ]` errors and the `||` branch fires. **All nine runs 3290390–3290398** hit exactly this: their outs end
`# train rc=0` → `grep: …/wmfix_s2_3290391.out: No such file or directory` → `line 60: [: : integer expression
expected` → `FATAL: no [sim-variant] line`, exit 3, State FAILED, ~6.6 h of training each and **no in-job eval**.
They were rescued afterwards by `full_posthoc_evals.sbatch` (17 `fresh_eval_rnd30_mode/metrics.json` now exist), so
no number of record is lost — but the mechanism is live in both launchers. Fix: `SLURM_OUT=$(scontrol show job
$SLURM_JOB_ID | grep -oP 'StdOut=\K\S+')` and `n=${n:-0}`.

### S3-3. `--requeue` is dead code in `wmfix_s2`/`wmfix_full` (CONFIRMED by reading; never fired)
Line 26 `[ -e "$LOGDIR" ] && { echo "FATAL: $LOGDIR exists"; exit 2; }` executes **before** the clean-restart block at
line 43 (`if SLURM_RESTART_COUNT > 0 … rm -rf "$LOGDIR"`). A preempted run is requeued, finds its own logdir, and
exits 2 — it never reaches the code written to handle exactly that. No slurm out contains the message (0 hits), so
it has not yet cost a run. `wmfix_phase.sbatch:50` has no such guard and is correct. The robomimic launchers
(`sbatch_rlpd_robo.sh:32`, `sbatch_r2d_robo.sh`) are also correct.

### S3-4. `full_eval_sweep.sh` can lose a run permanently (CONFIRMED by reading)
`full_eval_sweep.sh:7,9` writes `.posthoc_submitted` at submit time and never clears it, while
`full_posthoc_evals.sbatch:3` runs on `-p batch,preempt` **without `--requeue`**. A preempted post-hoc eval leaves
the marker behind and the run is skipped by every later sweep, silently. Its trigger also tests only
`fresh_eval_rnd30_mode` (line 5), so a run missing any other cell is never resubmitted. Fix: clear the marker when
the referenced job ends without producing the cells, or condition on all four cells.

### S3-5. `polE_contact_sweep.sh` has no in-flight marker and can double-submit onto one output dir (CONFIRMED by reading)
`polE_contact_sweep.sh:7` skips a cell only once its `metrics.json` exists. Re-running the sweep before the first
eval finishes submits a second job writing the same `fresh_eval_polE_<mode>/` directory. CLAUDE.md records the
polE-dDP sweep being driven "via a detached loop" — that is the race. `full_eval_sweep.sh` shows the right pattern
(a marker file); this one should use it.

### S3-6. RLPD launcher has no budget check; DP launcher has no `|| true` on its eval grep (CONFIRMED by reading)
`sbatch_r2d_robo.sh` verifies `LAST ≥ STEPS − 5000` before evaluating; `sbatch_rlpd_robo.sh:36` verifies only that
`rlpd_final.zip` exists, and never compares the sidecar's `ckpt_step` with `--steps`. Today they are equivalent
(SB3 writes the zip only after `learn()` returns; I verified `ckpt_step == 100000` in all 16 primary runs), but this
is precisely the check whose absence produced the "evaluated an untrained checkpoint" incident in the Genesis
launchers (CLAUDE.md, 09-05). Separately, `sbatch_dp_robo.sh:55` pipes the evaluator into `grep …` under
`set -euo pipefail` **without** the `{ … || true; }` guard that the smoke-round-1 fix added to the two online
launchers — an eval that dies before printing a matching line fails the job after 1.6 h of training.

### S3-7. `--mem=32g` is hard-coded for arms spanning 16.5k to 536k demo rows (PLAUSIBLE)
`sbatch_rlpd_robo.sh:17`. C1 (`MGall`, 3349120–27, queued) loads 536,522 transitions through
`train_rlpd_robosuite.py:32`, which materialises a **Python list of 536,522 tuples of numpy views** (~0.4 GB) before
`DemoData` stacks and pins them (~0.12 GB) — so 32 g should hold, but the log's own memory note says "48 g holds a
5.2e5-row buffer; MGall (1.6e6 rows) would need ~96 g" (that figure is the r2d buffer, not RLPD's, which the log
does not distinguish). There is no assert and no per-arm memory scaling; an OOM manifests as requeue →
`rm -rf $OUT` → clean restart → OOM, i.e. a silent loop that burns the 10 h limit. Cheapest fix: scale `--mem` off
the arm's `manifest.json` `rows_after_cut`, or stream the npz into `DemoData` without the intermediate list.

### S3-8. Env-var silent defaults in the wmfix launchers (PARTLY CONFIRMED)
`wmfix_full.sbatch` reads `DEMO_ROOT`, `ENVCFG`, `RECIPE`, and (inside the demo gate) `REPEAT`, all with defaults.
Two of these are cross-checked and will abort loudly: the gate at line 38 asserts the demo dir's `action_repeat`
equals `int(os.environ.get("REPEAT","4"))` and its `scope == "full"`, which catches a stride mismatch and a
phase-demos-into-full-config mix-up. **`ENVCFG` and `RECIPE` are checked against nothing** — `RECIPE` in particular
silently defines the recipe of record (`env.actor_dist`, `env.act_entropy`) and leaves no trace outside
`$W/runs/COMMANDS.log`. A wrong `RECIPE` produces a plausible run under the right name. Fix: echo `RECIPE`/`ENVCFG`
into the run's logdir as a sidecar and assert them against the config the trainer composed.

---

## S4

- `convert_arms.py:121` names an episode count `rows` (`assert rows == info["total_episodes"] == len(tapes)`), which
  reads as a frame-count assertion and is not one. Harmless, confusing.
- `eval_random_robosuite.py` draws the gripper channel from `U(−1,1)`, so the random control almost never closes the
  gripper — a weaker negative control than a policy that toggles ±1 like the human data. 0/50 either way.
- `REVIEW_GUIDE_2026-09-07.md:110` vs `ROBOMIMIC_PLAN` A3: the falsifier's contrast (PH-v-MG vs MH-v-MG) disagrees
  between the two documents of record (see S1-2).

---

## What the queued controls can and cannot separate

Non-source differences between MH200 and MG200s, measured (rows/rewarded/density from
`robomimic_data/arms/arm_counts.json`; lengths from the manifests; action stats and t0 xy computed here):

| difference | MH200 | MG200s | separated by C1 MGall | C2 MG718s | C3 300k |
|---|---|---|---|---|---|
| tapes | 200 | 200 | yes (3,900) | yes (718) | no |
| transitions | 41,134 | 16,501 | yes (536,522) | **yes (59,222 > 41,134)** | no |
| rewarded terminals | 200 | 200 | yes (718) | yes (718) | no |
| reward density | 0.486 % | **1.212 %** | yes (0.134 %) | no (1.212 %) | no |
| failure rows present | none | none | **yes** | no | no |
| tape length after cut (min/med/max) | 97/171/1046 | 45/77/146 | no (≤150) | no | no |
| **mean \|a\|, smoothness, gripper coding** | 0.144 / 0.044 / binary | 0.664 / 0.272 / continuous | **no** | **no** | **no** |
| t0 can-xy distribution | ≡ bank | ≡ bank (S2-3) | — | — | — |
| corrupt t=0 row density | 0.49 % | 1.21 % | no (0.73 %) | no | no |
| online budget | 100k | 100k | no | no | **yes (300k)** |
| demo subsample variance | fixed seed-0 draw | fixed seed-0 draw | yes | yes | no |
| **no-demo floor for this budget** | — | — | **no control exists** (S2-1) | no | no |

So the A2 set is a good quantity/failure/budget design and a **null design for the action process**, which is the
largest measured difference between the arms. If C1/C2/C3 all land ≥ 0.15 below MH200, the defensible sentence is
"not quantity, not failure rows, not budget" — the residual is source *and* action statistics, jointly.

Per-seed spread and dead seeds: the registered statistic is an exact permutation on per-seed counts with no
exclusion rule, and none is needed — MH200's near-dead seed s3 (1/50) *reduces* the gap, and removing it moves
p from 0.0078 to 0.0009. But MH200's sd (0.263) means the arm is strongly spread (five seeds at 24–44/50, three at
1–20/50), which is worth a sentence: the "0.455" is a mean over a wide, possibly bistable per-seed distribution,
not a typical seed.

---

## Claims that survived

1. **The headline numbers and p-values.** Re-read from the 16 run directories: MH200 mode [27,44,30,1,20,9,24,27],
   MG200s [10,2,2,13,6,12,9,5]; sample [29,41,31,2,22,7,28,23] and [7,1,4,16,12,11,9,7]. All 16 carry the same
   `bank_sha256 72b75550…`, `episodes 50`, `ckpt_step 100000`. My own exact permutation over all 12,870 splits gives
   **p = 0.00777 (mode) and 0.01103 (sample)** — the published 0.008 / 0.011 are right, and Δ = 0.3075 / 0.290.
2. **No preemption touched the readout.** All 16 primary slurm outs print `restart=0`; sacct shows COMPLETED for
   3347082–97 with no requeues. The clean-restart branch never ran, so no run mixed two attempts.
3. **The mis-fire did not contaminate the MH200 runs.** `rlpd_MH200_s0/` contains only 11:02–11:43 artefacts
   (ckpt_020…ckpt_100, rlpd_final.zip, train.log) from the 10:53 job; a 2-second job cannot write a checkpoint, and
   `tee` truncated the stale train.log.
4. **MG's differing `env_args` are cosmetic.** MG's hdf5 declares `damping_ratio`/`damping_ratio_limits` and
   `robots: "Panda"` where PH/MH declare `damping`/`damping_limits` and `robots: ["Panda"]`. I built all three envs
   and compared: identical `kp` (150) and `kd` (24.4949…) on every axis, and a 40-step fixed pseudo-random action
   sequence from bank entry 0 gives a **bit-identical** state (L2 = 0.0) in all three. So `eval_dp_robosuite.py`'s
   per-arm env choice (`side["hdf5"]`, i.e. the arm's own file) is safe, and the yaml's "identical env_kwargs in
   every Can file" comment is true where it matters.
5. **r2dreamer budgets are matched in online decisions.** From the runs' own step accounting: MH200 prefill 41,334
   rows → trainer starts at 41,628, `env.steps` 541,334 → **499,706 online**; MG200s prefill 16,701 → starts 16,902,
   `env.steps` 516,701 → **499,799 online**. `buffer.max_size` exceeds `env.steps` in both, and both prefill reports
   say the first demo frame would be evicted only after the budget ends — so no demo row is lost and the WM arm's
   "500k online decisions" is honest for both sources.
6. **G0's registered form holds.** Per-tape `pass_` = [T,T,F,T,T,T,F,T,F,T] with final world-can errors
   0.64/0.71/1.33/0.86/0.37/0.43/1.16/0.42/1.15/0.98 cm → the first five tapes give 4/5, matching the registered
   clause; success flag agrees 10/10. (The gate *implementation* is still S3-1.)
7. **The QOS move cost nothing.** `sacct -o Constraints` for 3290391 and 3290398 (`wmfix_full`) shows
   `l40s|a100|l40|h200` intact and `ReqTRES` `gres/gpu=1, mem=48G, cpu=8` for both, one moved to preempt/preempt and
   one left on gpu/normal. 3291435/3291440/3291450/3291458 (`wmfix_ph`) show an **empty** Constraints field — but
   `wmfix_phase.sbatch` never declares `--constraint` (its header has only `-p gpu,preempt`, `--gres=gpu:1`,
   `--requeue`, `--exclude=pax077`, `-n 8`, `--mem=48g`, `-t 8:00`), so there was nothing to lose; all four
   COMPLETED. The pending A2 controls still show `Requeue=1`, `Features=l40s|a100|l40|h200`, `gres/gpu=1`,
   `MinMemoryNode=32G`, `ExcNodeList=pax077` (scontrol, 3349120 and 3349128).
8. **The post-hoc eval scripts cannot double-count a completed cell or score an unfinished run.**
   `full_posthoc_evals.sbatch:22` skips any cell whose `metrics.json` exists; `:18` refuses a run whose last logged
   step is < 1,995,000; every eval is on `latest.pt` (the final checkpoint) with `--seed 0` and the fixed IC file.
   `bcrnn_reeval_last.sh` is idempotent, sorts checkpoints on the basename's epoch number, and preserves the
   superseded evaluation as `eval_bank50_ep<N>` (the epoch-950 dirs are all still present) — the LAST-checkpoint
   re-scoring of the BC-RNN control is clean, and its correction is properly logged.
9. **The conversion is internally consistent.** `convert_arms.py:141` asserts converted rows equal the manifest's
   `rows_after_cut`; the manifests match `arm_counts.json` (MH200 41,134 / MG200s 16,501; 200 rewarded rows and 200
   terminals each — I re-derived both from `transitions.npz`); the cut is `rows 0..k` with `rew[-1]=1, done[-1]=1`
   and `next_obs` taken from the file's own group, so `next_obs[k]` is the real successor state and the reward is
   granted exactly once. MG200s' 300-block histogram [0,0,0,0,0,0,2,6,10,29,42,55,56] is proportional to MG718s'
   [0,0,0,0,0,0,3,24,31,130,132,195,203] at 200/718 = 27.9 %, consistent with the uniform draw it claims to be.
10. **The stale-reset artefact is handled on every path.** `robo_common.reset_env_to` / `reset_env` re-read through
    `env.get_observation()` (`:126-138`), and both the gym env (`robosuite_can_env.py:42,44`) and the r2dreamer
    adapter (`r2d/robosuite.py:73-77`) go through them; `Bank.entry` always includes the model xml, so
    `EnvRobosuite.reset_to` performs a full `reset()` and the OSC controller's out-of-MuJoCo state is cleared
    between bank episodes. Bank exactness is recorded as restore-twice 0 and restore-vs-reseed 1e-7.
11. **The (T,17) → 23-dim WM change is guarded, not silent.** `r2d/apply_patches.py` asserts exactly 8 marked edits
    and makes the dims read `config.env.state_dim` / `config.env.size` × `image_channels`; a missing key falls back
    to the Genesis values and then fails the shape assert loudly. `envs/__init__.py` gets `seed=config.seed + id`,
    so the six parallel workers do not share a placement stream.

---

## Summary (15 lines)

1. The MH200 0.455 v MG200s 0.147 numbers, the exact permutation p = 0.008 / 0.011, the shared bank, the LAST
   checkpoint and the absence of any preemption all reproduce from the run files. The result is real.
2. **S1-1:** MG demo actions are 4.6× larger, 6.2× less smooth, and use a continuous gripper where humans use ±1.
   None of C1/C2/C3 changes that, so the A2 decision rule would call an action-process effect a source effect.
3. **S1-2:** as registered, falsifier (a) is PH200 v MG200s, and RLPD has no PH200 data; the contrast was re-pointed
   in an amendment recorded after the readout, and REVIEW_GUIDE still names the old contrast.
4. **S2-1:** the registered no-demo control (G2b) was never run — the number that decides "MG is worse" versus
   "MG is worthless" does not exist. ~2 GPU-h × 8.
5. **S2-2:** Δ = 0.3075 with bootstrap CI [0.125, 0.483]; MH's per-seed sd is 0.263, 3.3× the registration's
   assumption, so "Δ ≥ 0.15" holds only at the point estimate.
6. **S2-3:** I computed the registered t0-placement covariate: all three sources and the bank share one placement
   distribution. The standing "MG starts come from SAC's own reset distribution" caveat is wrong; delete it.
7. **S2-5/6:** MH200 quietly over-weights the two best operators (68/66/66); the known t=0 artefact is 2.5× denser
   in MG200s than MH200 — both small, both undisclosed, both pointed the same way.
8. **S2-7:** "the 10:26 mis-fire was cancelled before any job started" is false — 16 jobs ran for 2 seconds. No
   contamination, but the DP launcher's `rm -rf "$OUT"` makes the claim worth getting right.
9. **S3:** the G0 gate in `submit_primary.sh` passes at 4/10 where 4/5 is registered; nine wmfix_full runs lost
   their in-job evals to a hard-coded slurm-out name; `--requeue` is dead code in wmfix_s2/full; the post-hoc
   markers can strand a run; `polE_contact_sweep.sh` can double-submit; MGall is queued at a hard-coded 32 g.
10. Highest-value next actions, in order: run the no-demo RLPD control; paste the covariate table and delete the
    start-distribution caveat; report Δ with its CI; add a same-action-process control (Can-Paired good v bad);
    fix the two contradictory falsifier statements before the controls read out.

*Out of scope S1: none found outside the mandate. Nothing was modified; no job was submitted, cancelled or altered.*
