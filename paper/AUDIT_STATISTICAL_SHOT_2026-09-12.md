# Audit: is each algorithm × reward recipe getting a fair statistical shot? (2026-09-12, 00:20–01:30 EDT)

Independent, read-only audit of the state on the cluster and in the repo at the moment of writing.
Question put to the auditor: the project will choose, within a day, which reward recipe to run at
≥ 16 seeds per arm for the human-vs-machine-demonstration comparison; has each learner × recipe had
a fair statistical shot, and what would make the choice decidable?

Scope: learners {RLPD}, {r2dreamer} (the world-model port of record), and the DreamerV3-loss
variant run locally; recipes `staged` (pilot, (z)), `sparse` (pilot, (z)), `nested_sparse` and
`nested_ramp` v2 ((aa) + revisions 1–2); arms human `dHfull_all` v machine `dDPfull_first`.

Sources read: `paper/HANDOFF_2026-09-11.md`, `paper/LADDER_UNIFY_BRIEF_2026-09-10.md`,
`paper/PHASE_PLAN_2026-09-04.md` §(z), §(aa), (aa) rev 1, SUBMITTED, rev 2,
`paper/LADDER_PILOT_LOG_2026-09-11.md`, `paper/LADDER_N_PILOT_LOG_2026-09-11.md`,
`paper/LADDER_N_DEMO_CHECK_2026-09-11.md`, `paper/LADDER_N_RAMP_V2_2026-09-11.md`,
`paper/PILOT_RESCORE_2026-09-11.md`, `paper/ROUTE_CENSUS_2026-09-11.md` (+LOG),
`paper/ROUTE_CENSUS_RC2_2026-09-11.md`, `paper/DV3_LOCAL_2026-09-11.md`,
`paper/DV3_LOCAL_EVAL_2026-09-11.md`, `paper/DV3_LOCAL_E2E_COLLAPSE_2026-09-11.md`,
`paper/TIP_RULE_2026-09-11.md` §10, `paper/TIP_GUARD_IMPL_2026-09-11.md`,
`HRI_results/DEMO_SETS_2026-09-11.md`, `HRI_results/CLAIMS_LEDGER.md`, `HRI_results/hri_stats.py`.
`paper/RL100_READOUT_2026-09-11.md` **does not exist** (the handoff cites it); its cells do
(`$W/rl100_2026-09-11/`, 12 jobs 3594375–3594885) and are read here directly.

Live state was read at **2026-09-12 00:14–00:21 EDT** with `squeue`, `sacct -S 2026-09-11`, the
Slurm logs, every `metrics.json` under the run trees, and every `episode_rollouts.jsonl` /
`metrics.jsonl`. `LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`. Nothing was
modified, submitted or cancelled. Every number below carries its source; an absent value is
written as absent, never as 0.

---

## VERDICT (one page)

**Under the project's own registered rule ("highest `home` rate at the matched milestone; tie →
ignition count → `slide_event`"; brief lines 268–277) the recipe choice is NOT decidable today for
either learner, and the rule as written cannot produce a defensible choice at n = 2 v 2 even when
every cell lands.** Two structural reasons, independent of what the cells will say:

1. **The statistic cannot separate anything at n = 2.** The exact permutation test of record
   (`hri_stats.perm_test`) has 6 splits at 2 v 2, so the smallest attainable two-sided p is
   **0.333**. With the per-seed spread actually observed on `home` (SD 0.02–0.19 across the
   route-census cells), the MDE at 80 % power is **0.27–0.74 on the rate scale** — larger than
   any `home` rate ever measured in this project (max 0.267, one cell). "Highest rate" at n = 2
   is a comparison of two-seed means with no test behind it.
2. **The matched-milestone cells the rule needs do not exist and nothing is scheduled to make
   them.** The {r2dreamer} launcher evaluates only `latest.pt` at the end of the job
   (`cluster/wmfix_full.sbatch:153–165`); every milestone checkpoint written so far
   (`online_500000/1000000/2000000.pt`, 26 of them) is unevaluated; the post-hoc sweep that
   exists is scoped to the old `_rx_` seeds (`cluster/e2e_posthoc_sweep.sh:15–24`). The only
   Ladder-N {r2dreamer} `home` numbers in existence are training-record counters, which RC2
   showed can read 0.79 on a checkpoint that reloads to 0/90.

Per cell (n = seeds per arm; "shot" = trained to a registered budget AND scored for `home` on a
pinned, stamped cell under the ladder it trained on):

| learner | recipe | n/arm | budget | `home` cells | verdict | why |
|---|---|---:|---|---|---|---|
| {RLPD} | `staged` (pilot, guard `grip`) | 2 | 100k | 4 × rnd30 mode (record, 64p) | **no** | 100k is 40 % of the budget every competitor got; excluded by the brief on design grounds (`slide_success` is a transient detector), never on a matched-budget result. `home` = 0/120. |
| {RLPD} | `sparse` (pilot, `grip`) | 2 | 250k | 2 of 4 seeds (census, sampled) | **yes — as a negative, at n = 4** | `nested_v2` = 0.000 in all 40 in-job cells of all 4 seeds and 0/90 in the census; eliminated on P7 for this learner. The route split is undefined (no arrivals). |
| {RLPD} | `nested_sparse` | 2 | 250k | 5 cells at 100k (4 seeds) | **not yet** | 110–122k of 250k at 00:21; `home` = 0 in every 100k cell. Final cells ≈ 05:00–08:00 EDT 09-12. |
| {RLPD} | `nested_ramp` v2 | 2 (+4 at 500k) | 250k / 500k | 9 cells at 100k | **not yet** | 119–166k of 250k; `home` = 0 in every 100k cell; P-aa-3 human arm NOT met at 100k (`slide_event` 0 in all six human cells). 500k: one seed at 15k, three PENDING. |
| {RLPD} | `staged` + guard (control) | 2 | 100k | 8 cells + rl100 | **yes, for its purpose** | Complete. P-aa-6 (tip rate within ±0.05 of the pilot's staged arm) is **NOT met** on the whole-run read (+0.094) and borderline on the last-100 read (+0.05). |
| {r2dreamer} | `staged` (pilot, `grip`) | 2 | 1M | **none** | **no** | ¼ of the sparse budget; the in-job cells predate the `home` column (`farside/slide_event/home` absent); no pinned re-score of any r2dreamer pilot checkpoint exists. |
| {r2dreamer} | `sparse` (pilot, `grip`) | 2 | 4M | 4 seeds × 2 sets (census, sampled, mid-run) | **partly** | s965/s966 finished 4M (census at 3.98M: `home` rnd30 0.067/0.167); s945 at 3.2M with an unexplained 0/90 (RC2); s946 preempted, restarted, 2.2M. P8 confirmed for the machine arm (both seeds majority-drop), human arm on ONE seed (6 v 5). Old guard. |
| {r2dreamer} | `nested_sparse` | 2 | 4M | **none** | **not yet** | 0.85–1.78M of 4M; s975/s976 (machine) are not picking (last-300 `picked` 0.023/0.007). s955's training counter shows `home` 0.233 — the only strong positive signal in the project and inadmissible until a cell exists (RC2). 4M cells ≈ 10:30–14:30 EDT 09-12 at the earliest. |
| {r2dreamer} | `nested_ramp` v2 | 2 at 2M (+4 at 4M) | 2M / 4M | **none** | **not yet** | 1.23–1.40M of 2M (cells ≈ 04:30–05:30); 4M seeds at 0.23–0.68M. The 2M matched read (2M seeds v 4M seeds' milestone) needs milestone cells nobody has scripted. Ramp seeds ignite in training records (18/3/20/1 `home` events) — same caveat. |
| {dv3-loss, local} | `nested_ramp` v2 | 1 (human only) | 2M | 2 sampled cells at 500k | **no — and it should not be in the table** | One seed, one arm, local AVX2 box, tree `a35534b` ≠ the batch's `a40c8aa1`, 0/15 picks at 500k (`DV3_LOCAL_E2E_COLLAPSE` §2). It is evidence about the port's critic/clamp calibration, not about the recipe. |
| cross-learner | any | — | — | — | **no** | Budgets differ 4× in env frames (250k decisions = 1M frames v 4M frames), QOS split by learner, three different evaluation protocols, guard-v-ladder confounded for {r2dreamer} (no control arm). |

**Safe now:** (a) plain `sparse` is not a viable {RLPD} recipe at 250k (n = 4, zero arrivals in
evaluation); (b) plain `sparse` on {r2dreamer} pays drops on the machine arm (n = 2, both seeds);
(c) the pilot's staged {RLPD} policies produce 0 `home` (4 seeds × 30 pinned episodes);
(d) {RLPD} at 100k produces 0 `home` under every Ladder-N recipe (27 cells, 12 seeds);
(e) the new tip guard changes {RLPD} training termination (P-aa-6 not met on the whole run);
(f) the batch trains on 12/12 `home` tapes, not 13/14 (P-aa-7 failed on the machine set).

**Not defensible at the next readout:** any "recipe A > recipe B" rate statement at 2 v 2; any
{r2dreamer} `home` number read from `metrics.jsonl`; "ramp ignites earlier than nested_sparse"
(s955 ignited at 0.58M online, before any ramp seed, per the same untrusted counters); any
cross-learner ordering; any 2M-milestone comparison until milestone cells exist.

**What makes it decidable:** (1) evaluate the milestones (CPU only, ~0 GPU-h); (2) bring
`nested_sparse` to the same seed count as `nested_ramp` (4 v 4 per learner: +4 {r2dreamer} at 4M,
+4 {RLPD} at 250k or 500k ≈ 110–145 GPU-h); (3) decide on an explicitly-labelled IGNITION read
(seeds with ≥ 1 `home` in a pinned cell), not on a rate; (4) hold the 16 v 16 until (1)–(3).
Plan with hours in §7.

---

## 1. Budget parity

### 1.1 Registered v actual, per learner × recipe

| learner | recipe | registered budget (PHASE_PLAN) | milestones | actual at 00:21 EDT 09-12 | source |
|---|---|---|---|---|---|
| {RLPD} | `staged` pilot | 100k decisions ((z), line 1028) | 40k, 100k | 4/4 COMPLETED 100k (3539249–52, 4:27–5:18) | `sacct`; `e2e_rlpd_35392{49..52}.out` `TRAIN-OK … 100000` |
| {RLPD} | `sparse` pilot | 250k ((z), line 1030) | 40k/100k/250k | 4/4 COMPLETED 250k (3539253–56, 11:13–13:07) | same |
| {RLPD} | `nested_sparse` | 250k ((aa), line 1424) | 40k/100k/250k | s955 121,963 / s956 116,194 / s975 111,132 / s976 113,034 decisions | `episode_rollouts.jsonl` last `step` |
| {RLPD} | `nested_ramp` | 250k ((aa), line 1422) | 40k/100k/250k | s950 165,905 / s951 165,702 / s970 165,940 / s971 120,916 | same |
| {RLPD} | `nested_ramp` ext | 500k (rev 2, line 1544) | 100k/250k/500k | s952 16,843 RUNNING; s953/s972/s973 **PENDING `QOSMaxGRESPerUser`** | `squeue` |
| {RLPD} | `staged` + guard ctl | 100k ((aa), line 1426) | 40k/100k | 4/4 `TRAIN-OK 100000`; in-job eval running | logs 3581566–69 |
| {r2dreamer} | `staged` pilot | 1M online ((z), line 1029) | 0.5M/1M | 4/4 COMPLETED (3539257–60, 3:49–4:45) | `sacct` |
| {r2dreamer} | `sparse` pilot | 4M online ((z), line 1031) | 0.5/1/2/4M | s965/s966 COMPLETED 4M (13:30/13:46); **s945 RUNNING 14:23, at 3.19M online**; **s946 preempted once, restarted 16:48, at 2.23M online** | `sacct`; `metrics.jsonl` last step − origin 117,624 |
| {r2dreamer} | `nested_sparse` | 4M ((aa), line 1425) | 0.5/1/2/4M | s955 1.79M / s956 1.36M / s975 0.96M / s976 0.85M online | `metrics.jsonl` − origin (117,624 human / 149,952 machine) |
| {r2dreamer} | `nested_ramp` | 2M ((aa), line 1423) | 0.5/1/2M | s950 1.40M / s951 1.23M / s970 1.27M / s971 1.39M online | same |
| {r2dreamer} | `nested_ramp` ext | 4M (rev 2, line 1524) | 0.5/1/2/4M | s952 0.59M / s953 0.68M / s972 0.23M / s973 0.67M online | same |

Command for the {r2dreamer} rows: `python3 - <<EOF` over `$W/runs/full_r2d_state_<set>_s<seed>/metrics.jsonl`
(last `step`; origin from the `Step accounting` line in `$W/slurm/<job>.out`).

### 1.2 Where a recipe is compared at a different budget than another

1. **`staged` v everything.** The pilot's staged arm ran 100k decisions / 1M frames; its
   competitors ran 250k / 4M. The brief excludes staged on design grounds (its top rung fired on
   set-down transients, `PILOT_RESCORE` §5), which is a legitimate reason — but the record should
   say "excluded by design, never compared at a matched budget", not "lost". The control arm
   ({RLPD} 2 v 2 at 100k) inherits the same budget.
2. **`nested_ramp` v `nested_sparse` — unequal seed counts at every milestone.** Revision 2 added
   four seeds per learner to `nested_ramp` (4M / 500k) and none to `nested_sparse`. At the 4M
   milestone {r2dreamer} ramp will have 2 v 2 (the extension) and nested_sparse 2 v 2 (the
   batch); at 2M ramp will have 4 v 4 (2M seeds + the 4M seeds' milestone) and nested_sparse
   2 v 2 (a milestone that is not scheduled for evaluation). For {RLPD} the same: ramp 4 v 4 at
   250k (if the 500k seeds ever start) and nested_sparse 2 v 2. The tie-breaker "ignition count"
   is therefore biased toward the recipe with more seeds unless normalised.
3. **The pilot's `sparse` at 4M v the batch's `nested_sparse` at 4M is NOT a matched pair.** Same
   step, different tip guard (`grip` v `not_in_hand@4f`), different `stage_predicates.py`
   (`de4ffde57cd7` v `a589b4f05632`; `released` now requires `picked`), different demo-set build
   (direct re-execution on pax080 v records on pax146), different `full_env.py` hash. The P1
   stamps make this explicit — the two ladders' lines differ in more than the four fields P1
   allows (`grep -h '^\[ladder\] unified' … | sort -u` gives 5 distinct lines; the pilot lines
   carry no `far_release=` and no `tip=` field at all).
4. **The 2M ramp seeds v the 4M extension seeds** are matched in code, sets and stamps
   (`git=known-good-2026-08-27-895-ga40c8aa1-dirty` on all 8 logs) but not in QOS: the four 2M
   seeds and two of the four 4M seeds are on `normal`; two 4M seeds are on `preempt` (rev 2
   discloses it). No preemption has occurred yet on any `ln_*` job (all logs have exactly one
   `Logdir` line; every {RLPD} log says `restart=0`).
5. **{RLPD} v {r2dreamer} budgets are not commensurable.** 250k decisions × repeat 4 = 1M env
   frames; the world model's 4M is 4× that. A recipe that needs > 1M frames to ignite will look
   dead on {RLPD} and alive on {r2dreamer}; that is a learner × budget interaction, not a recipe
   result. The brief does not claim cross-learner comparability, but the verdict table it asks
   for will invite it.

### 1.3 Which "matched milestone" comparisons are genuinely matched

| comparison | step | evaluator | protocol | start sets | matched? |
|---|---|---|---|---|---|
| {RLPD} ramp s950/951/970/971 @100k v nested_sparse s955/956/975/976 @100k | 100k ✔ | `gp_ladderN` `eval_e2e.py` ✔ | rl100: record role, 64-core nodelist, rnd30 mode+sample, hold15 mode, shared-process ✔ | rnd30 + hold15 ✔ | **yes** (only 2 of the 4 nested_sparse seeds have all 3 cells so far; s975 has 1) |
| {RLPD} ramp @250k v nested_sparse @250k (final) | 250k | in-job `e2e_eval_cells.sh` (preview role) | hold15/rnd30/spots60 × sample/mode + `_iso` | ✔ | **yes when they land**, but preview-role cells; the record-role pass is a separate CPU job |
| {RLPD} ctl @100k v pilot staged @100k | 100k ✔ | pilot: `gp_unified` evaluator (no `home` column); ctl: `gp_ladderN` | in-job preview | ✔ | **partly** — `home` exists for the pilot only through the re-score array (rnd_mode, record) and for ctl through rl100 (rnd30 mode, record); those two ARE comparable |
| {r2dreamer} ramp 2M seeds @2M v 4M seeds @2M milestone | 2M | none for the milestone | — | — | **no cell exists or is scheduled** |
| {r2dreamer} nested_sparse @4M v ramp ext @4M | 4M | in-job `eval_genesis.py` on `latest.pt` | sample+mode, hold15+rnd30, `--device cpu`, no role/hardware stamp | ✔ | **yes when they land** (≈ 10:30–17:00 EDT 09-12), n = 2 v 2 |
| {r2dreamer} pilot sparse @4M v nested_sparse @4M | 4M | in-job | as above | ✔ | **no** — guard + predicates + set build differ (§1.2 item 3) |

---

## 2. Seeds and power

### 2.1 What the project's statistic can do at n = 2, 4, 8, 16

`HRI_results/hri_stats.py` (`perm_test`, lines 65–98; `mde_ci`, lines 101–129). Computed with the
module itself (`~/workspace/genesis_sim2real/venv/bin/python`, `sys.path` → `HRI_results`):

| n per arm | exact splits | **minimum attainable two-sided p** |
|---:|---:|---:|
| 2 | 6 | **0.333** |
| 3 | 20 | 0.100 |
| 4 | 70 | 0.029 |
| 8 | 12,870 | 0.0002 |
| 16 | 6.0 × 10⁸ (Monte-Carlo above 400k) | ≈ 0 |

MDE at 80 % power on the rate scale, pooled-SD t convention (the convention that reproduces the
ledger's RLPD pick MDE 0.345), as a function of the per-seed SD of the rate:

| per-seed SD | n = 2 | n = 4 | n = 8 | n = 16 | n for MDE ≤ 0.10 |
|---:|---:|---:|---:|---:|---:|
| 0.05 | 0.268 | 0.119 | 0.075 | 0.051 | 6 |
| 0.10 | 0.536 | 0.237 | 0.151 | 0.102 | 17 |
| 0.15 | 0.804 | 0.356 | 0.226 | 0.154 | 37 |
| 0.20 | 1.073 | 0.474 | 0.301 | 0.205 | 64 |

### 2.2 The observed per-seed spread on `home`

The only cells with `home` and ≥ 2 seeds per arm are the route census's {r2dreamer} sparse-pilot
cells (sampled, seed 0, preview protocol, mid-training snapshots; `$W/route_census_2026-09-11/*/
{hold15,rnd30}_sample/metrics.json`):

| cell | human seeds | machine seeds | Δ | SD_h | SD_m | MDE@80 % | 95 % CI | exact p |
|---|---|---|---:|---:|---:|---:|---|---:|
| rnd30 `home` | [0, 1]/30 | [2, 5]/30 | −0.100 | 0.024 | 0.071 | **0.283** | [−0.327, +0.127] | 0.333 |
| hold15 `home` | [0, 4]/15 | [2, 3]/15 | −0.033 | 0.189 | 0.047 | **0.737** | [−0.625, +0.558] | 1.000 |
| rnd30 `nested_v2` | [0, 3]/30 | [17, 11]/30 | −0.417 | 0.071 | 0.141 | 0.600 | [−0.898, +0.064] | 0.333 |
| hold15 `nested_v2` | [0, 8]/15 | [11, 11]/15 | −0.467 | 0.377 | 0.000 | 1.430 | [−1.614, +0.681] | 0.333 |

The seed-to-seed spread on `home` (0.02–0.19) is the same order as the rates themselves. At n = 2
the MDE exceeds every `home` rate ever recorded (max 0.267, s946 hold15). At n = 4 it is 0.12–0.36;
at n = 8, 0.08–0.23; **at n = 16 the ±0.10 margin the ledger's rule R1 enforces is reachable only
if the per-seed SD is ≤ 0.10** — which the hold15 human cell (SD 0.19, one seed at 0 and one at
0.27) already violates. The old-batch e2e reference (CLAIMS_LEDGER `no_source_e2e`, n = 8 v 8):
`e2e_nested_honest` MDE 0.088, `e2e_picked` MDE 0.210.

### 2.3 Can the registered decision rule produce a defensible choice at n = 2?

No. "Highest `home` rate at the matched milestone" compares two means of two numbers each, with a
test whose floor is p = 0.333 and an MDE of 0.27–0.74. "Tie → ignition count" at n = 2 takes
values 0, 1, 2 and is dominated by single-seed variance (s945 v s946: 0 v 0.267 on the same arm,
same recipe, same budget class). "→ `slide_event`" adds another 2-v-2 mean. Any recipe ordering
this rule emits is a description of four seeds, not an inference about the recipes.

**The honest alternative** (recommended, §7): decide on IGNITION evidence and say so. Define the
decision statistic as "number of seeds, per learner, with ≥ 1 `home` in a pinned record-role cell
at the largest common milestone", require the same n for every candidate, and label the choice as a
feasibility selection ("the recipe under which the behaviour was observed at all"), not as "the
recipe with the higher rate". If two recipes both ignite in ≥ 2 of 4 seeds per arm, the rate
question is genuinely open and the 16 v 16 should be split (8 v 8 per recipe) rather than spent on
one.

---

## 3. Evaluation-protocol parity

Four different protocols produced the cells now on the table. From the launchers in the tree that
ran (`$LAB/gp_ladderN/cluster/`), the sbatch in `$W/rl100_2026-09-11/`, and the cells' own fields:

| producer | role | node class | sets | modes | isolation | `home` column | which cells |
|---|---|---|---|---|---|---|---|
| {RLPD} in-job, `sbatch_rlpd_e2e.sh:202` → `e2e_eval_cells.sh` (`SETS="hold15 rnd30 spots60" MODES="sample mode" ISO=1 ISO_SETS="rnd30 spots60"`) | `preview` | whatever GPU node ran training (pax007/011/048/049/051/105/110/112/150/152) | 3 | 2 | shared + `_iso` | pilot: **absent**; Ladder N: present | pilot staged/sparse (40 cells × 4 seeds each), ctl (8 cells/seed) |
| {r2dreamer} in-job, `wmfix_full.sbatch:153–165` (`for MODE in sample mode; for SET in hold rnd`, `--device cpu`, `latest.pt` only) | **none stamped** (`role=None`, `cores=None`) | GPU node | 2 | 2 | shared | pilot: **absent**; Ladder N: present (smoke verified) | pilot staged s940/941/960/961, pilot sparse s965/966 |
| pinned re-score array 3581786 (`ln13_rescore.sbatch`) | `record` | 64p (pax019/146/149) | rnd30 | mode | shared | present | pilot {RLPD} staged, 4 seeds |
| route census (Lane RC) | {RLPD} `record` / {r2dreamer} none | {RLPD} pax015/027 64p; {r2dreamer} GPU nodes | hold15, rnd30 | **sample** | shared | present | pilot sparse: {RLPD} s945/s965 only; {r2dreamer} 4 seeds (copies at 2.90M/1.82M/3.98M/3.99M) |
| rl100 lane (`rl100_eval.sbatch`) | `record` | `--nodelist` 64p | rnd30 (mode+sample), hold15 (mode) | 2 | shared | present | Ladder N {RLPD} `ckpt_100` of a copy = 100k, 12 seeds (27 cells written, 5 jobs still running at 00:21) |

Consequences:

1. **The registered decision cells ("rnd30 mode + hold15 mode at every milestone", (aa) line
   1445) exist for {RLPD} at 100k (rl100) and will exist at 250k (in-job, but preview role); they
   exist for NO {r2dreamer} Ladder-N checkpoint and will exist only at the END of each run.**
   The {r2dreamer} 500k/1M/2M milestones (26 `.pt` files with `.json` sidecars, all verified
   present) have never been rolled out. RC2's finding makes the training-record substitute
   inadmissible, so **for {r2dreamer} the "matched milestone" statistic is currently
   unimplemented.**
2. **Sampled v mode.** The route census (the P8 evidence) is sampled; the decision statistic is
   mode. On the pilot's {r2dreamer} sparse s965 the two differ by 0 on `nested_v2` rnd30
   (0.433 both) but the pilot's staged s940 differs by 0.27 on `picked` (mode 0.633 v sample
   0.567, hold15 0.867 v 0.933); no `home` cell exists in both modes for the same checkpoint, so
   the mode/sample sensitivity of `home` is unmeasured.
3. **Shared-process v fresh-process.** Only the {RLPD} in-job path writes `_iso` cells. They
   disagree with the shared cells by up to 0.34 on `picked` at n = 60 (pilot s960 spots60 sample
   0.283 v `_iso` 0.417; s961 0.250 v 0.083) — the order-dependence PHASE_PLAN (s) registered. No
   {r2dreamer} `_iso` cell exists anywhere. Any {RLPD} ↔ {r2dreamer} comparison mixes protocols.
4. **Hardware class.** The rl100 and re-score cells are pinned to 64-physical-core nodes; the
   in-job cells of both learners are not (the {r2dreamer} cells carry no core stamp at all —
   `cores=None` on all 24 pilot cells). The project's own finding is that 36-core boxes diverge.
5. **RC2.** {r2dreamer} s945's training counter reads `nested_v2` 0.787 over its last 300 episodes
   while six independent cells (old tree, new tree, sample, mode, hold15, rnd30) read 0/90. The
   mechanism is not found. Until it is, any {r2dreamer} number read from `metrics.jsonl` — including
   the ramp-seed ignition counts that motivated revision 2 and the nested_sparse s955 counter in
   this audit — is an exploration diagnostic. Whether the defect is seed-specific is unknown: s946's
   counter tracks its cells "reasonably" (RC2 Check 3), and no third seed has been checked.
6. **The {RLPD} record gap.** `episode_rollouts.jsonl` carries `episode/train_ep_{picked, placed,
   placed_v2, contact, contact_push, nested, nested_v2, slide_success, tipped}` and **not**
   `farside`/`slide_event`/`home` (verified on the last row of s940, s945, s950, s958). So P-aa-3
   ("`slide_gain_m > 0` in training rollouts by 50 % of budget") cannot be read from the {RLPD}
   record at all, only from cells; and no {RLPD} training curve for `home` can ever be drawn for
   this batch.

**Which numbers are comparable across learners today:** none on `home`. The closest pair is
{RLPD} rl100 (record, 64p, mode) v nothing on the {r2dreamer} side. On `nested_v2`/`picked` the
pilot's in-job cells of both learners share sets and modes but not role, hardware or the `home`
column.

---

## 4. Confounds between learner and recipe

1. **Learner × ladder budget.** {RLPD} 250k decisions (1M frames) v {r2dreamer} 4M frames for the
   "full" budget; 100k v 1M for staged. A recipe's viability is budget-dependent (the brief's own
   rev-2 argument: "2M is the budget most likely to stop just before the behaviour"), so the
   learner and the budget move together.
2. **QOS placement.** Pilot: all 16 registered on `normal`; the 4 {r2dreamer} sparse were moved to
   `preempt` at ~14:20 (pilot log §"deviation") and s946 was preempted once, restarting from 0 at
   16:48 (`lz_r2_sparse_dH_s946_3539262.out`: `# requeued (restart 1): clearing the partial
   logdir`). Batch: 8 {r2dreamer} on `normal`, 12 {RLPD} on `preempt`; extension: 2 {r2dreamer}
   normal + 2 preempt, 4 {RLPD} preempt (3 still pending). Preemption restarts from step 0
   (`wmfix_full.sbatch:103`, `sbatch_rlpd_e2e.sh:161`), so a preempted seed reaches a milestone at
   a different wall-clock time but the same step — a delay, not a bias — unless the batch is read
   before it finishes, in which case the preempt-QOS seeds are systematically the ones missing.
3. **The tip guard.** Pilot ran `grip` (stamps carry no `tip=` field; sidecars `tip_guard: None`);
   batch runs `not_in_hand@4f`. The {RLPD} control arm was built to separate this. Read from the
   training records (`episode/train_ep_tipped`):

   | arm | pilot staged (`grip`) whole run / last 100 | ctl staged (`not_in_hand`) whole run / last 100 |
   |---|---|---|
   | human s940, s941 → s958, s959 | 0.346, 0.333 / 0.44, 0.41 | 0.392, 0.434 / 0.30, 0.42 |
   | machine s960, s961 → s978, s979 | 0.379, 0.347 / 0.52, 0.40 | 0.428, 0.525 / 0.54, 0.71 |
   | mean | 0.351 / 0.443 | 0.445 / 0.493 |

   P-aa-6 registered "within ±0.05 of the pilot's staged arm at the matched step" without naming
   the window. Whole-run: **+0.094, not met**; last-100: +0.05, at the boundary. The registered
   disconfirm branch — "the guard changed policy termination and the ladder comparison must be read
   with that caveat" — fires on the whole-run read. **For {r2dreamer} there is no control arm**, so
   the guard's effect on the world model is unmeasured; its ramp seeds tip in 68–76 % of their last
   300 training episodes against 37 % for the pilot's staged seeds and 9–26 % for the pilot's sparse
   seeds (`metrics.jsonl`, same counters, same caveat). Ladder and guard cannot be separated for the
   learner the decision matters most for.
4. **Return clamps.** {r2dreamer} clamp = ladder ceiling by construction (9 ramp / 1 nested_sparse
   / 8 staged / 1 sparse; `[ladder] return_clamp=… (env and model agree)` on every log). The recipe
   comparison for the world model is therefore also a clamp comparison. `DV3_LOCAL_E2E_COLLAPSE`
   §3 attributes the local run's collapse to critic/continuation miscalibration under clamp 9 with
   an earned ceiling of 1; the cluster's clamp-9 ramp seeds are NOT collapsed (last-300 `picked`
   0.81–0.88, `placed_v2` 0.49–0.64), so the effect is not universal — but it is a learner-specific
   hyperparameter that moves with the recipe.
5. **Demonstration sets.** Pilot `_rz`/`_rs` (pax080, direct re-execution, `grip`); batch
   `_rnrh`/`_rnsh`/`_rzh` (pax146, records, `not_in_hand`). The `_rzh` sums reproduce `_rz` to the
   unit (171.0 / 183.0), so the two builds agree on the staged column. On `home`: local 13 human /
   14 machine, cluster **12 / 12**; P-aa-7 (±1) failed on the machine set (−2). The arms are
   matched at 12/12 by coincidence (different tapes lost). Every "the demonstrations slide 13/14
   times" sentence must now read 12/12 and cite job 3575953. Not independently re-opened here;
   taken from `ladderN_verify_sets.sh` output quoted in the log.
6. **The DreamerV3-loss variant.** One seed, human arm only, `nested_ramp` only, RTX 3080 Ti /
   Ryzen 5950X AVX2, genesis tree `a35534b` at launch (the cluster batch is `a40c8aa1`; the three
   ladder-file hashes match, the rest of the tree does not), r2dreamer `0cf3d9e` (same as the
   cluster). Its 500k checkpoint scores 0/15 `picked` on hold15 where the pick-scope run scored
   15/15. It is still training (step 1,112,556 raw ≈ 1.0M online of 2M at 00:14; PID 716613). **It
   is not in the comparison and should not be**: n = 1, one arm, one recipe, different hardware
   class, and a diagnosed calibration failure. It is useful as a warning about clamp 9 for
   dreamer-loss critics, nothing more.

---

## 5. What is decidable now, and what is not

### 5a. Safe conclusions (with their n and caveats)

| claim | evidence | n | caveat |
|---|---|---|---|
| Plain `sparse` does not produce settled contact for {RLPD} at 250k in evaluation | `nested_v2` = 0.000 in all 40 in-job cells of s945/s946/s965/s966 (`$LAB/gp_unified/…/fresh_eval_*/metrics.json`); census 0/90 on s945/s965 | **4 v 4 seeds** (the docs say n = 2; the in-job cells make it 4) | Old guard; `nested_honest` reaches 0.03–0.13 in 7 of those cells, so the settle sometimes says yes where the tracker says no; training records show 4–22 `nested_v2` events per seed, so the behaviour exists at a rate below what 45–60 episodes can see |
| Plain `sparse` on {r2dreamer} reaches `nested_v2` mostly by a drop (P8) | machine 38/50 drop (s965 85.7 %, s966 63.6 %); human 6/11, all from s946 | machine **2 seeds**, human **1 seed** | Sampled, preview protocol, mid-run copies; s945's 0/90 is unexplained (RC2). The human-arm reading is one flipped episode from 50/50 (`ROUTE_CENSUS` §3) |
| The pilot's staged {RLPD} policies do not slide | pinned re-score 3581786: `home` 0, `slide_event` 0 on 4 × 30 rnd30 mode episodes (`$W/pilot_rescore_2026-09-11/*/rnd_mode/metrics.json`); Lane 11 locally 0/300 on two of them | 4 seeds | 100k budget |
| {RLPD} at 100k reaches `home` under no Ladder-N recipe | rl100: `home` = 0.0 in all 27 cells across 12 seeds (ramp 4, nested_sparse 4, ctl 4) | 12 seeds | 100k = 40 % of budget; `slide_event` > 0 in 3 cells (s970 hold15 0.067, s970 rnd30 sample 0.033, s955 rnd30 sample 0.033) |
| The new guard changes {RLPD} training termination | §4 item 3 | 4 v 4 | Window unspecified in P-aa-6 |
| The batch trains on 12/12 `home` tapes | LADDER_N_PILOT_LOG §3.5 | — | Not re-opened here |
| The staged `slide_success` rung is a transient detector | PILOT_RESCORE §5: 4 of 32 positives survive | 600 episodes, local AVX2 | — |

### 5b. Conclusions the decision rule would draw at the next readout that would NOT be defensible

1. "`nested_ramp` beats `nested_sparse` for {r2dreamer}" from the 2M in-job cells of the ramp
   seeds (≈ 04:30–05:30 EDT) — no nested_sparse cell exists at 2M and none is scheduled.
2. "`nested_sparse` works for {r2dreamer}" from s955's training counter (`home` 0.233 over its
   last 300 episodes, 89 events since 0.58M online) — RC2 makes this inadmissible; the checkpoint
   `online_1000000.pt` is on disk and has not been rolled out.
3. "`nested_ramp` ignites in all four {r2dreamer} seeds" — true in the training records
   (18/3/20/1 events at 1.23–1.40M online) and unverified in any cell.
4. Any rate ordering at 2 v 2 (§2.3).
5. "The pilot's `sparse` reached `nested_v2` at ≥ the staged rate, so sparse becomes primary" — the
   (z) decision rule is moot: the brief eliminated plain sparse on the route census, but note the
   rule as registered would have FIRED for {r2dreamer} (sparse s965/s966 `nested_v2` rnd30 mode
   0.43/0.30 at 4M v staged 0.07/0.00 at 1M — a budget-confounded comparison the rule did not
   guard against).
6. Any cross-learner recipe ordering (§4 items 1–3).
7. "{RLPD} never slides" from `episode_rollouts.jsonl` — the key is absent (§3 item 6).

### 5c. Minimal additional runs that make the choice defensible for BOTH learners

Measured throughputs (this audit, from the logs): {RLPD} 163,850 decisions in 5 h 33 m
(3581558) = **29.5k decisions/h**, SB3 `fps` 6–8; in-job eval adds ≈ 1 h (staged pilot: 4:27 total
for 100k) to ≈ 3 h (sparse pilot: 11:13–13:07 for 250k). {r2dreamer} `fps/fps` 63–112 across the
12 `ln_r2` logs, 52–80 on the pilot; s950: 1.40M online in 5 h 33 m = **70 frames/s**; 4M ≈
16 h + ≈ 1 h eval. GPU ceiling 30 (10 normal + 20 preempt).

| item | runs | GPU-h each | GPU-h | purpose |
|---|---:|---:|---:|---|
| A. Milestone cells for every existing {r2dreamer} Ladder-N/pilot checkpoint (0.5M/1M/2M/4M as they exist; rnd30 + hold15, mode, record role, 64p) | ~30 CPU jobs | 0 GPU (≈ 0.3–0.5 CPU-h per cell) | **0** | Implements the matched-milestone rule; tests RC2 on more seeds |
| B. {r2dreamer} `nested_sparse` +2 seeds per arm at 4M | 4 | 17 | 68 | Brings nested_sparse to 4 v 4 like ramp |
| C. {RLPD} `nested_sparse` +2 seeds per arm at 250k (or 500k to match the ramp extension) | 4 | 11.5 (20) | 46 (80) | Same |
| D. {r2dreamer} `staged` + `not_in_hand` control, 1M, 2 v 2 | 4 | 4.5 | 18 | Separates guard from ladder for the world model (the P-aa-6 analogue) |
| E. Live-instrument one {r2dreamer} training process (RC2 follow-up) | 1 | ~4 | 4 | Decides whether ANY training counter can ever be used |
| **total** | | | **136–170** | ≈ 6 h of the 30-GPU ceiling; wall ≈ 18–21 h for B/C |

Without A the rule is unimplementable; without B/C the ignition count is a seed-count artefact;
without D every {r2dreamer} recipe result carries "guard OR ladder".

---

## 6. Registration hygiene

### 6.1 Timing (commit author dates, EDT; job starts from `sacct`)

| registration | commit | jobs | before submission? |
|---|---|---|---|
| (z) pilot | `a89e26a` 03:10:14 | 3539249–64 submitted 03:2x | yes (tree fast-forwarded twice while all 16 PENDING — disclosed, pilot log §1/§3) |
| (aa) Ladder N | `aac4131` 13:55; rev 1 `dda07d0` 16:50 | 3581558–77 started 18:41 | yes |
| decision rule for ≥ 16 seeds | `ec5b8ea` 22:22 | (no job) | written before any Ladder-N `home` cell existed (first rl100 cell 23:43) — but AFTER the ramp seeds' training-record ignition was visible, and its tie-breaker (ignition count) favours what was already visible. Timing exposure, not a violation |
| (aa) rev 2 extension | `5dcd380` **22:24:30** | 3591975/77/78 **started 22:25:03** | yes, by 33 s. The revision text says "registered 22:23 EDT"; the commit is 22:24:30 |

### 6.2 Falsifiability

- P-aa-2 (structural, count = 0), P-aa-4 (< 50 % slide share in every arm), P-aa-5 (clamp = max),
  P-aa-7 (±1 tape) are falsifiable and P-aa-7 was recorded as FAILED. Good.
- **P-aa-3** ("by 50 % of budget ≥ 1 seed per arm shows `slide_gain_m > 0` in training
  rollouts") is not readable as written: `slide_gain_m` is not logged (LADDER_N_PILOT_LOG §4.3),
  and for {RLPD} `slide_event` is not in the record either. Read through cells at 100k (40 % of
  budget): machine ramp arm met (s970 `slide_event` 0.067/0.033), **human ramp arm NOT met**
  (`slide_event` 0 in all six s950/s951 cells). The registered disconfirm branch ("rerun that arm
  with D7 shaping ON, disclosed") is due to be evaluated at 125k and has not been.
- **P-aa-6** omits its window (§4 item 3) and is not met on one reading.
- The (z) decision rule ("sparse ≥ staged rate at the matched milestone") never named a matched
  milestone the two arms actually share with cells (staged stops at 1M; the sparse 1M milestone
  was never evaluated).

### 6.3 Numbers asserted in the docs, checked against their sources

| assertion | source doc | measured here | verdict |
|---|---|---|---|
| "two of the four `nested_ramp` 2M seeds already show `home` … at ~1M (`s950`: 1, `s970`: 2)" | PHASE_PLAN (aa) rev 2, line 1512 | `metrics.jsonl`, `home` events with online step ≤ 1.0M: s950 **1**, s951 **2**, s970 **6**, s971 0; s970 has 4 events before 850k online | direction confirmed (ignition visible), **counts do not reproduce**; the cut-off used is not stated; three seeds, not two, by 1.0M |
| "`s941` was sliding in 12 % of rollouts at 0.8M" | HANDOFF §2 table | s941 `slide_success` per 100k online window: 0.079 [700–800k), 0.098 [750–850k), 0.055 [800–900k) | **not reproduced** (max 9.8 %); and the column is the withdrawn `slide_success` rung the same document says not to quote |
| s946 hold15 0.533, rnd30 0.100; s945 0/45; machine s965 0.733/0.567, s966 0.733/0.367 | ROUTE_CENSUS §2 | cells read directly | **reproduce exactly** |
| s945 fresh copy 0/90 across six cells | RC2 Check 4 | 6 cells, all `nested_v2` 0 | **reproduces** |
| {RLPD} sparse pilot "0/90 `nested_v2` in evaluation" | brief line 294 | census 0/90 on 2 seeds; in-job 0.000 in 40 cells on 4 seeds | reproduces, and is stronger than stated (n = 4) |
| "{r2dreamer} ≈ 75 frames/s, {RLPD} ≈ 29k decisions/h" | brief lines 283–285 (implied) | 63–112 fps; 29.5k dec/h | reproduces |
| pilot `_rz` / `_rs` sums 171/14/183/15; `_rzh` 171/183 | pilot log §1.1, Ladder-N log §3.3 | `DEMO-SHA` lines in the job logs | reproduce (not re-derived from the npz) |
| "the 4 `lz_r2_sparse` moved to preempt, the 2 staged dM stay on normal" | pilot log deviation | `sacct`: 3539261–64 preempt; 3539259/60 normal | reproduces |

---

## 7. Recommended plan for the next 48 h (GPU-hours at the measured throughputs)

**Do not submit the 16 v 16 in this window.** Nothing in it would rest on a defensible recipe
choice, and 32 {r2dreamer} runs at 4M (≈ 512 GPU-h, two waves on the 30-GPU ceiling ≈ 34 h wall)
cannot be undone cheaply.

### Hours 0–8 (until ≈ 08:30 EDT 09-12) — make the rule implementable; cost ≈ 0 GPU-h

1. **Write `paper/RL100_READOUT_2026-09-11.md`** from `$W/rl100_2026-09-11/` (5 jobs were still
   running at 00:21; 27 cells exist). The handoff cites a document that does not exist.
2. **Script a Ladder-N milestone evaluator** (a copy of `rl100_eval.sbatch`'s pattern for
   `eval_genesis.py`: copy `milestones/online_<N>.pt` + `.hydra/config.yaml` out of the live
   logdir, evaluate on a 64-core CPU node, rnd30 + hold15, mode AND sample, `--out` outside the
   tree) and submit it for every existing milestone: pilot sparse s945/s946 `online_2000000.pt`,
   s965/s966 `online_2000000.pt` and `online_4000000.pt`; ramp s950/s951/s970/s971
   `online_1000000.pt`; nested_sparse s955/s956 `online_1000000.pt`, s975/s976/s973/s952/s953
   `online_500000.pt`. ≈ 20 CPU jobs, ≈ 0.5 CPU-h each. This is the only way the "matched
   milestone" statistic can exist for {r2dreamer}, and it tests RC2 on eight more seeds.
3. **Re-score under one protocol** (record role, 64p, mode, `home` column) the checkpoints that
   were never scored for `home`: {RLPD} sparse pilot finals s946/s966 (the census did s945/s965
   only) and {r2dreamer} pilot staged/sparse `latest.pt` (4 + 4). CPU only.
4. Read the {r2dreamer} ramp 2M in-job cells as they land (≈ 04:30–05:30) and the {RLPD} 250k
   in-job cells (ramp ≈ 04:00–06:00, nested_sparse ≈ 05:00–08:00) — as ignition evidence only.
5. Record P-aa-3's human-ramp disconfirm at 125k (the 40 % read is already negative) and decide
   whether the registered D7-shaping rerun is invoked; record P-aa-6 with both windows.

### Hours 8–30 — equalise seeds; cost ≈ 136–170 GPU-h (≈ 6 h of the ceiling)

6. Submit §5c items B, C, D (and E on one GPU): `nested_sparse` +2 seeds/arm on both learners
   (4M / 250k), a {r2dreamer} `staged`+guard control 2 v 2 at 1M. Use the next free seed pairs
   in `cluster/RUN_REGISTRY.jsonl`; register as (aa) rev 3 BEFORE submission with the ignition
   statistic named as the decision statistic and the same n for every candidate.
7. Read the {r2dreamer} nested_sparse 4M cells (s955 ≈ 10:30, s956 ≈ 13:00, s975/s976 ≈ 14:00)
   and the ramp 4M extension cells (≈ 12:00–17:00); {RLPD} 500k s952 ≈ 20:00, the other three
   later (still pending at 00:21).

### Hours 30–48 — decide, labelled

8. Decision table: per learner × recipe, at the largest milestone at which BOTH candidates have
   ≥ 4 seeds per arm with record-role cells: seeds with ≥ 1 `home` (ignition), pooled `home`
   count, `slide_event` count, P8 route split. Choose on ignition; if both candidates ignite in
   ≥ 2 of 4 seeds per arm on a learner, split that learner's 16 v 16 into 8 v 8 per recipe rather
   than pick. State in the registration that the choice is a feasibility selection at n = 4 and
   that the recipe contrast is not a tested claim.
9. Only then submit the ≥ 16-seed batch: {r2dreamer} 32 × ≈ 17 GPU-h ≈ 544 GPU-h; {RLPD} 32 ×
   ≈ 11.5 GPU-h ≈ 368 GPU-h (250k) or ≈ 640 GPU-h (500k). Add the three Ladder-N stages to the
   {RLPD} episode record first (handoff §7b), or the batch will again have no `home` curve.

---

## Appendix — commands used (all read-only)

```
ssh -o BatchMode=yes jstale02@login.pax.tufts.edu
squeue -u jstale02 -o "%.10i %.28j %.9T %.11M %.10q %.10P %.20R" | grep -E "lz_|ln_"
sacct -S 2026-09-11T00:00 -u jstale02 -X -n -P --format=JobID,JobName%30,State%16,Elapsed,ExitCode,QOS,Partition,NodeList,Start,End
for f in $W/slurm/ln_r2_*.out $LAB/gp_ladderN/e2e_rlpd_*.out $W/slurm/lz_r2_*.out $LAB/gp_unified/e2e_rlpd_*.out; do grep -h '^\[ladder\] unified' "$f" | head -1 | sed 's/ *git=.*//'; done | sort | uniq -c
grep -o "total_timesteps *| *[0-9]*" $LAB/gp_ladderN/e2e_rlpd_3581558.out | tail -1     # 163850 at 5h33
grep -o "fps/fps [0-9.]*" $W/slurm/ln_r2_*.out | tail -1                                   # 63.3–112.0
sed -n 140,185p $LAB/gp_ladderN/cluster/wmfix_full.sbatch                                  # latest.pt-only eval loop
grep -n -E "SETS|MODES|ISO" $LAB/gp_ladderN/cluster/sbatch_rlpd_e2e.sh $LAB/gp_ladderN/cluster/e2e_eval_cells.sh
python3 - < inventory.py   # every metrics.json under gp_unified, gp_ladderN, $W/runs, pilot_rescore, route_census, rl100
python3 - < records3.py    # episode/train_ep_* rates from episode_rollouts.jsonl and metrics.jsonl
python3 - < verify.py      # rev-2 ignition counts, s941 window rates, milestone sidecars
git log -1 --format="%h %ad %s" --date=format-local:"%Y-%m-%d %H:%M:%S %Z" a89e26a 266998a dda07d0 a40c8aa 5dcd380 ec5b8ea
~/workspace/genesis_sim2real/venv/bin/python -c "from hri_stats import perm_test, mde_ci; ..."   # §2 tables
ps aux | grep train.py; tail -c 3000 ~/runs_dv3_local/dv3e2e_ramp_dHfull_all_rnrh_rlDreamer_s0/metrics.jsonl
```
