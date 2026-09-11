# {Diffusion Policy} end-to-end on the PRUNED human set — execution log (2026-09-11)

Registration: `paper/PHASE_PLAN_2026-09-04.md` **amendment (ab)**, commit `847f635` on branch
`ladder-unify-2026-09-11`, pushed **before** any job was submitted.

Code tree of record for this work: **`$LAB/gp_dp_e2e` @ `847f635d`** — a SEPARATE clone, not
`$LAB/gp_unified`. `gp_unified` is frozen at `92f46b7c` for the ladder pilot and pulling it would
change `git describe` in the `[ladder]` stamp of jobs that start after the pull. Verified that this
separation costs nothing: the six files that determine an evaluation are **byte-identical** between
the two trees —

    for f in baselines/rl/full_env.py baselines/eval_e2e.py baselines/stage_predicates.py \
             baselines/genesis_can_env.py cluster/e2e_eval_cells.sh baselines/merge_e2e_iso.py; do
      sha256sum $LAB/gp_dp_e2e/$f $LAB/gp_unified/$f; done
    # SAME f9e9538d9fc62e18 full_env.py / d4f09a39ba7285f4 eval_e2e.py / de4ffde57cd7573f stage_predicates.py
    #      40544bf73c8c69bd genesis_can_env.py / ac71f19ba50ce638 e2e_eval_cells.sh / 4a2812ba56bb0bef merge_e2e_iso.py

and the D6 ladder stamp from the new clone carries the pilot's own content hashes:

    unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 | max_return=8
    | terminal=slide_success+tipped | shaping=off
    | full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7
    | git=known-good-2026-08-27-840-g847f635d

Only the `git=` suffix differs, which is precisely the documentation-only delta the pilot freeze
was protecting.

Nothing under `$W/gp_root`, `$LAB/gp_e2e`, `$W/r2dreamer_fix` or `$LAB/gp_unified` was written to.
The existing checkpoints are reached through **read-only symlinks** into this tree, so the
re-scored cells land under `$LAB/gp_dp_e2e/.../rec/` and `gp_e2e` is untouched.

---

## 1. The three facts, with their commands

### (a) Where the {DP} end-to-end runs of record live

20 runs under `$LAB/gp_e2e/baselines/outputs/dp_e2e/`, each keeping exactly one checkpoint
(`checkpoints/100000/pretrained_model`) plus a `last` alias:

| arm | run dirs | seeds | demo set | lerobot dataset | demo sha |
|---|---|---|---|---|---|
| human, RAW | `e2e_dp_dH_s0..s7` | 0–7 | `matched_w3/dHfull_all` (74 tapes) | `dHfull_all/lerobot` (72 ep / 29 219 frames) | `70027c899be26f60` |
| machine, best-of-3 | `e2e_dp_dDP_s0..s7` | 0–7 | `matched_w3/dDPfull` (72) | `dDPfull/lerobot` (70 ep / 32 849) | `433fda6e…` |
| machine, de-selected (v) | `e2e_dp_dDPfirst_s0..s3` | 0–3 | `matched_w3/dDPfull_first` (72) | — | `ebb7bd84…` |

    ls -d $LAB/gp_e2e/baselines/outputs/dp_e2e/*/
    cat $LAB/gp_e2e/baselines/outputs/dp_e2e/e2e_dp_dH_s0/checkpoints/100000/dp_sidecar.json
    # arm dH, seed 0, scope full, raw_demo_dir .../dHfull_all, git 68a41cd, amendment n

**Their cells are `role: preview`, and there is NO pinned pass for this learner.** Every
`fresh_eval_*/metrics.json` under those 20 runs reads `role = preview`, `git = 68a41cd`, and
carries no `ladder_provenance` and no `nested_v2` (the pre-unification evaluator). No `rec/`
directory exists anywhere under `dp_e2e`.

    ls -d $LAB/gp_e2e/baselines/outputs/dp_e2e/*/rec 2>/dev/null    # nothing

The per-seed `rnd30` picked counts reproduce PHASE_RESULTS §5.2 exactly — human
6,6,4,5,10,9,10,7 = **57/240 = 0.237**; machine 14,13,13,18,18,11,16,16 = **119/240 = 0.496**:

    for A in dH dDP; do for S in $(seq 0 7); do
      head -1 $LAB/gp_e2e/baselines/outputs/dp_e2e/e2e_dp_${A}_s${S}/E2E_HEADLINE.txt; done; done

### (b) A full-scope PRUNED human set already exists in the e2e lerobot format

`$LAB/genesis_pickaplace/baselines/matched_w3/dHfull_pruned` — **nothing had to be built.** It was
built on 2026-09-04 as the training set of the full-task {DP} teacher, and is reused here rather
than rebuilt, so this arm trains bit-for-bit on the set the teacher saw.

| field | value |
|---|---|
| tapes | 64 `1NNNNN.npz`, contract v1, scope full, `gc_kp4_riser3_shelf6` |
| `content_sha256` | `841c5dd547c565e63…` (prefix of record `841c5dd547c565e6`) |
| built by | `baselines/prune_full_v1.py --margin 38` over `baselines/demos_v2/dHfull_w3` |
| decisions | 23 307 (from 25 123 recorded on the same ICs → **7.2 % pre-pick idle collapsed**) |
| lerobot | `fps 7.5`, `total_episodes 64`, `total_frames 23307`, same features as the raw arm |

**It is the SAME RECORDINGS as the raw arm's — verified, not assumed.** All 64 ICs are a strict
subset of the raw set's 74; for **64/64** the tape's stamped `prune_orig_n` equals the raw tape's
`n`, and the **last 50 decisions are bit-identical** (the pruner never touches anything from
`j_pick − 38` on). Σ tape reward is **118.0 in both sets**.

    python3 /tmp/lineage_check.py .../matched_w3/dHfull_all .../matched_w3/dHfull_pruned
    # shared ICs 64 | prune_orig_n == raw n 64/64 | last-50 actions bit-identical 64/64
    # decisions raw(shared) 25123 -> pruned 23307 (7.2% removed)

The 10 ICs present in the raw set and absent from the pruned one are
**234, 278, 286, 293, 294, 295, 300, 301, 318, 319** — exactly the 10 `stage = none` (no-pick)
tapes, which carried 0 reward. Stage yields: raw `picked 43 / contact 18 / nested 3 / none 10`
→ pruned `43 / 18 / 3 / 0`.

### (c) LINEAGE OF THE MACHINE SET'S TEACHER — the coordinator's statement is WRONG

The coordinator told the user the machine set's teacher was trained on the **raw** set. **It was
trained on the PRUNED set.** Every one of the 195 harvested machine tapes stamps the same
`teacher_ckpt`:

    python3 - # over $W/demos_state_full/src_dDPfull/*.npz
    # src_dDPfull n_files= 195
    #   teacher: dp 195
    #   teacher_ckpt: .../baselines/outputs/dp_phase/dHfull_pruned_DP_s0/checkpoints/100000/pretrained_model  195

and that teacher's own configuration reads the pruned dataset:

    python3 -c "import json; print(json.load(open('$LAB/genesis_pickaplace/baselines/outputs/dp_phase/\
    dHfull_pruned_DP_s0/checkpoints/100000/pretrained_model/train_config.json'))['dataset']['root'])"
    # baselines/matched_w3/dHfull_pruned/lerobot
    # sidecar: arm dHfull_pruned, demo_sha 841c5dd547c565e6, script sbatch_dp.sh, 2026-09-04

**Consequence.** §5.2 compares a student trained on the RAW human set against a student trained on
tapes produced by a teacher trained on the PRUNED human set. The two arms differ in demonstration
source **and** in whether the human data that reached them was pruned. The new arm is the cell that
separates those two, which is why it matters more than a convention fix.

For completeness, the human arm's own lineage: `src_dHfull_all` = 74 tapes,
`dHfull_w3_partial` 61 + `dHfull_w3_fails` 10 + `dHfull_w3` 3, all `teacher = human`.

---

## 2. Code change

`cluster/sbatch_dp_e2e.sh` gains `ARM=dHpruned` → `matched_w3/dHfull_pruned`, `N_EXP=64`, and
stamps `amendment=ab` in the run registry and every sidecar. The pruned set carries the **pruner's**
manifest schema, not `full_demos.py select`'s, so it gets its **own** provenance gate rather than a
loosened version of the existing one. The gate asserts positively (`contract v1`, `sim_variant`,
`pruner` beginning `baselines/prune_full_v1.py`, `N == 64 == len(files)`; per tape
`contract`/`scope`/`sim_variant`, `teacher == 'human'`, and the presence of `prune_rule`) and
negatively (the selector's keys `builder`/`scope`/`one_per_ic_best`/`one_per_ic_first` must be
ABSENT). The lerobot cross-check is computed from the tapes (`total_episodes == 64`,
`total_frames == Σ n == 23307`, `fps == 7.5`) because the pruner's manifest has no `n_lerobot`.

Written this way because the failure mode that has actually bitten this project is a manifest
**attesting** something its data does not support (the `one_per_ic_first` incident). Both
directions were tested before submitting:

    # PASS on the real pruned set
    GENESIS_PICKAPLACE_ROOT=$LAB/gp_dp_e2e DRYRUN=1 ARM=dHpruned SEED=100 bash cluster/sbatch_dp_e2e.sh
    # DEMO-SHA dHpruned full n=64 sha=841c5dd547c565e6 decisions=23307 (from 25123 recorded; 7.2% pre-pick
    #   idle collapsed) pruner=baselines/prune_full_v1.py --margin 38 source=baselines/demos_v2/dHfull_w3
    # PROVENANCE-OK total_episodes=64/64 total_frames=23307/23307 fps=7.5 short_tapes=[]

    # REFUSES the RAW set presented under the pruned arm's name
    mkdir -p /tmp/fakeroot && ln -sfn .../matched_w3/dHfull_all /tmp/fakeroot/dHfull_pruned
    ... DEMO_ROOT=/tmp/fakeroot ARM=dHpruned ... bash cluster/sbatch_dp_e2e.sh
    # AssertionError: ('select-built manifest under ARM=dHpruned', {...'builder': 'baselines/rl/full_demos.py select'})

Run-registry pre-check before submission: `REGISTRY-OK key=fd13d4826e2a no prior match`.

---

## 3. Jobs

### 3.1 Training — {Diffusion Policy}, pruned human arm, 8 seeds

Seeds **100–107** = the raw arm's 0–7 plus 100, so no run directory, wandb name or registry row can
collide. `preempt` partition and `preempt` QOS (QOS `normal` is left to the ladder pilot). 100k grad
steps, batch 64, final checkpoint only, 150 GB disk guard (288 GB free at submission).

    cd $LAB/gp_dp_e2e
    for S in $(seq 100 107); do
      GENESIS_PICKAPLACE_ROOT=$LAB/gp_dp_e2e ARM=dHpruned SEED=$S \
        sbatch -J dpP_e2e_dHpruned_s$S cluster/sbatch_dp_e2e.sh
    done

| job | name | seed | run dir |
|---|---|---|---|
| 3562638 | `dpP_e2e_dHpruned_s100` | 100 | `$LAB/gp_dp_e2e/baselines/outputs/dp_e2e/e2e_dp_dHpruned_s100` |
| 3562639 | `dpP_e2e_dHpruned_s101` | 101 | … `_s101` |
| 3562640 | `dpP_e2e_dHpruned_s102` | 102 | … `_s102` |
| 3562641 | `dpP_e2e_dHpruned_s103` | 103 | … `_s103` |
| 3562642 | `dpP_e2e_dHpruned_s104` | 104 | … `_s104` |
| 3562643 | `dpP_e2e_dHpruned_s105` | 105 | … `_s105` |
| 3562644 | `dpP_e2e_dHpruned_s106` | 106 | … `_s106` |
| 3562645 | `dpP_e2e_dHpruned_s107` | 107 | … `_s107` |

First-hour check (seed 100, `e2e_dp_3562638.out`): `dataset.num_frames=23307`,
`dataset.num_episodes=64` — the pruned set, confirmed from the trainer's own reading of it —
`updt_s 0.058`, loss 0.552 → 0.026 by step 3k. ≈ **1.6 h** per run at 100k steps.

**GPU budget:** this puts 8 of my GPU jobs on `preempt`; with the pilot's 4 sparse {r2dreamer}
runs that is **12 preempt GPU slots**, the stated ceiling. QOS `normal` is untouched (10 running +
1 pending, all the pilot's).

### 3.2 Re-score of the EXISTING checkpoints — queued immediately, not waiting on training

Pinned CPU-only pass through the **unified** evaluator, `ROLE=record`, cells under `<run>/rec/`.
20 jobs: 8 raw human + 8 machine (best-of-3) + 4 machine (de-selected, amendment (v)).

    # read-only staging: gp_e2e checkpoints reached by symlink, so rec/ lands in MY tree
    for A in dH dDP; do for S in $(seq 0 7); do
      mkdir -p $LAB/gp_dp_e2e/baselines/outputs/dp_e2e/e2e_dp_${A}_s${S}/checkpoints
      ln -sfn $LAB/gp_e2e/baselines/outputs/dp_e2e/e2e_dp_${A}_s${S}/checkpoints/100000 \
              $LAB/gp_dp_e2e/baselines/outputs/dp_e2e/e2e_dp_${A}_s${S}/checkpoints/100000
    done; done   # + dDPfirst s0..s3

    EX=$(cat $LAB/gp_dp_e2e/.excl64.txt)     # every node that is NOT 64 physical cores
    GENESIS_PICKAPLACE_ROOT=$LAB/gp_dp_e2e SWEEP_VERDICT=cores REQUIRE_CORES=64 THREADS=4 PAR=2 \
      LEARNER=dp ARM=<arm> SEED=<seed> SETS="hold15 rnd30" ISO=1 ISO_SETS="rnd30" VIDEO_SETS=none \
      sbatch --exclude=$EX -J dpRS_<arm>_s<seed> cluster/sbatch_e2e_rescore.sh

| arm | jobs | seeds |
|---|---|---|
| `dH` (human, raw) | **3562831–3562838** | 0–7 |
| `dDP` (machine, best-of-3) | **3562839–3562846** | 0–7 |
| `dDPfirst` (machine, de-selected) | **3562847–3562850** | 0–3 |

All 20 were RUNNING within a minute, 0 FATAL. Confirmed configuration from `dpRS_dH_s0` on pax044:

    == E2E-RESCORE learner=dp arm=dH seed=0 node=pax044 cores=64 verdict=cores require_cores='64' threads='4'
    == e2e_eval_cells kind=dp ... cells=.../e2e_dp_dH_s0/rec role=record sets='hold15 rnd30' modes='sample'
       iso=1 iso_sets='rnd30' par=2 cores=64 isa=avx512
    [eval-e2e] ... cores=64p/128l affinity=8 threads=4 role=record
    [eval-e2e] ladder 'staged' from the fallback (sidecar records none)

`LADDER=staged` is what a {DP} sidecar resolves to (it records no ladder, and `eval_e2e.py`'s
fallback is `staged`); the evaluator prints where it came from, and it is stamped on every cell.

### 3.3 Re-score of the PRUNED checkpoints — a DEPENDENCY CHAIN, not a follow-up script

Submitted now with `--dependency=afterok:<training job>`, so nothing has to be watched:

| job | name | waits on |
|---|---|---|
| 3562856 | `dpRS_dHpruned_s100` | 3562638 |
| 3562857 | `dpRS_dHpruned_s101` | 3562639 |
| 3562858 | `dpRS_dHpruned_s102` | 3562640 |
| 3562859 | `dpRS_dHpruned_s103` | 3562641 |
| 3562860 | `dpRS_dHpruned_s104` | 3562642 |
| 3562861 | `dpRS_dHpruned_s105` | 3562643 |
| 3562862 | `dpRS_dHpruned_s106` | 3562644 |
| 3562863 | `dpRS_dHpruned_s107` | 3562645 |

**Failure mode to watch:** if a training job ends non-zero its dependent sits forever in
`DependencyNeverSatisfied` and must be cancelled and resubmitted by hand after the training is
re-run. Check with `squeue -u jstale02 -o "%.10i %.24j %.9T %R" | grep dpRS_dHpruned`.

**A trap hit and fixed, recorded because it is the second time this project has been bitten by it.**
The `--exclude` list was first written to `/tmp`, which is **per-login-node**; the next `ssh` landed
on a different login node, `cat` failed, and those 8 dependent jobs were submitted with an EMPTY
exclude list. Caught by reading `ExcNodeList` back off the submitted jobs, and repaired in place
with `scontrol update JobId=<id> ExcNodeList=...` while they were still PENDING. The list now lives
on shared storage at `$LAB/gp_dp_e2e/.excl64.txt`. The 64-core guard inside the job would have
caught a wrong landing anyway — it exits FATAL rather than producing a cell — but it exits without
requeue, so the jobs would simply have died.

Node list built from Slurm's own socket × cores-per-socket topology, not from `%c` (logical CPUs)
and never from `AvailableFeatures` (which are wrong on this cluster):

    sinfo -h -N -o "%n %X %Y %Z" | awk '{if ($2*$3!=64) print $1}' | sort -u | paste -sd, -
    # physical-core census: 32c, 36c, 40c, 48c, 64c (85 nodes), 96c, 112c, 128c

---

## 4. Cells this pass produces, and the cost

Per run: `hold15` and `rnd30`, sampled, shared-process (the protocol §5.2 used), plus the
**isolated** `rnd30` cell (amendment (s) correctness check, one fresh process per start). `spots60`
is NOT in this pass — {DP} evaluates at ~425 s/episode on a 64-core node, so a shared `spots60` cell
is ~7 h serial per run; it can be added post hoc with `SETS=spots60` against the same `rec/` root.

`e2e_eval_cells.sh` runs the shared cells FIRST and the isolated cells second, and skips any cell
(or any isolated per-start shard) that already has a `metrics.json`. So a preemption resumes, and if
time runs short the isolated cell is what is lost, not the cell of record.

Configuration held IDENTICAL across all four arms, which is what makes the table internally
comparable: `REQUIRE_CORES=64`, `THREADS=4`, `PAR=2` (4 threads × 2 concurrent = the 8 allocated
CPUs exactly; no oversubscription), `EVAL_SEED=0`, `--max-steps 1200`, `MODES=sample`.

**{DP} has no deterministic mode in this pipeline** (`e2e_eval_cells.sh` uses `MODES=sample` for
`KIND=dp`, and every {DP} cell on record is sampled). The word "deterministic" must not appear
against a {DP} row.

## 5. Readout

    cd $LAB/gp_dp_e2e && python3 baselines/e2e_table_all.py --cell-root rec --strat
    # per-run: baselines/outputs/dp_e2e/e2e_dp_<ARM>_s<SEED>/rec/fresh_eval_{hold15,rnd30}_sample/metrics.json
    #          ... /rec/fresh_eval_rnd30_sample_iso/metrics.json
    # HEADLINE stage columns: picked, placed_v2, contact_push, slide_success, nested_v2, nested_honest

Then score amendment (ab) P1/P2/P3 against their disconfirm branches and write the result as a NEW
dated section of `paper/PHASE_RESULTS_2026-09-05.md` — **do not rewrite §5.2**; it is the
as-recorded preview row and becomes a dated static note once the pinned numbers exist. Report
movement between §5.2's preview numbers and the pinned numbers in the DIFFERENCE, per the standing
rule.

Reminders that belong in the write-up, all registered in (ab).4: the pruned set differs from raw in
**two** ways (idle collapse AND the 10 no-pick tapes), so a met P1 does not attribute; `placed_v2`
must be reported net of the four `rnd30` starts inside the shelf footprint (CONFOUNDS row 82,
symmetric across arms); the machine arm keeps its best-of-three selection confound and `dDPfirst`
is the cell that removes it; and at n = 8 v 8 a non-significant P3 reads "no effect detectable at
this sample size", never "equivalent".
