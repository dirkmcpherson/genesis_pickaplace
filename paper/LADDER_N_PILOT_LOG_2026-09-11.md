# Ladder N batch — job log and readouts (Lane 13, 2026-09-11)

Operational log for PHASE_PLAN amendment **(aa)** and its **REVISION 1** (`nested_ramp` v2).
The amendment is the registration (commits `266998a` + `dda07d0`, both BEFORE any Ladder-N job);
this file is the record of what was actually built, smoked and submitted. Every number carries
the command that produced it; every claim about a job carries its id.

Convention, per CLAUDE.md: results carry an explicit `{learner}` header.

Environment: `LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`,
local checkout `/home/j/workspace/genesis_pickaplace` on branch `ladder-unify-2026-09-11`.

---

## Step 1 — cluster state before anything was touched (2026-09-11 ~17:00 EDT)

    ssh jstale02@login.pax.tufts.edu 'squeue -u jstale02 -o "%.10i %.26j %.9T %.6M %.12q %.10P %R"'

| job | name | state | elapsed | QOS | partition | node |
|---|---|---|---|---|---|---|
| 3539253 | `lz_rl_sparse_dH_s945` | RUNNING | 8:21 | normal | gpu | pax049 |
| 3539254 | `lz_rl_sparse_dH_s946` | RUNNING | 8:08 | normal | gpu | pax049 |
| 3539255 | `lz_rl_sparse_dM_s965` | RUNNING | 7:21 | normal | gpu | pax051 |
| 3539256 | `lz_rl_sparse_dM_s966` | RUNNING | 7:20 | normal | gpu | pax105 |
| 3539261 | `lz_r2_sparse_dH_s945` | RUNNING | 7:09 | preempt | preempt | pax141 |
| 3539262 | `lz_r2_sparse_dH_s946` | RUNNING | **0:11** | preempt | preempt | pax113 |
| 3539263 | `lz_r2_sparse_dM_s965` | RUNNING | 7:09 | preempt | preempt | pax111 |
| 3539264 | `lz_r2_sparse_dM_s966` | RUNNING | 7:09 | preempt | preempt | pax112 |
| 3562861–63 | `dpRS_dHpruned_s105–107` | RUNNING | 1:37–2:03 | preempt | preempt | pax080/146 |

**GPU occupancy at this moment: normal 4 of 10, preempt 4 of 20** (the three `dpRS_*` are CPU).
So 6 normal GPU slots and ~16 preempt GPU slots are free for the 20 `ln_*` jobs.

`lz_r2_sparse_dH_s946` (3539262) was **preempted and requeued** — elapsed reset to 00:11 while its
three siblings are at 7:09. It did NOT die: `sacct` shows it `RUNNING 00:11:27 0:0`, not
`FAILED 2:0 00:00:00`, so amendment (z)'s requeue-guard fix held. It restarts clean from step 0
(logdir cleared by the guard), which costs ~7 h of its 4M-step budget, not the run.

Completed since 09-11 00:00 (`sacct -S 2026-09-11T00:00 -u jstale02 -X`):
all four `lz_rl_staged_*` (3539249–52, 4:27–5:18, `0:0`), all four `lz_r2_staged_*` (3539257–60,
3:49–4:45, `0:0`), all eight `dpP_e2e_dHpruned_s100–107` (3562638–45) and 29 of 32 `dpRS_*`
re-scores. **No job in the whole listing ended `FAILED 2:0 00:00:00`** (P6 of the pilot).
Two `dpP_e2e_dHpruned` rows (s106 3562644 at 00:00:06, s107 3562645 at 00:01:57) COMPLETED
almost instantly — that is the DP launcher's already-trained short-circuit, not a failure, and it
is Lane 10's to read, not mine.

Health checker (the 09-10 long-run batch, which the user cancelled 09-11 13:10):

    ssh … 'python3 $LAB/e2e_health.py'
    R2 run=0 pend=0 nokeys=0/32 | RLPD run=0 pend=0 nokeys=0/20 eps=23000 picked=11911
    disagree=0 young=0 | done=22 deaths=0 r2trained=29/32 benign_posttrain=0 preempted=0
    | disk=277G/OK | ALL-ENDED

Disk:

    df -BG /cluster/tufts/shortlab
    10.235.106.32:/projects/shortlab  1863G  1586G  277G  86%

**277 GB free against the registered 150 GB floor — DISK-OK.**

---

## Step 2 — the two trees of record for this batch

### `$LAB/gp_ladderN` (genesis_pickaplace)

    git clone -b ladder-unify-2026-09-11 https://github.com/dirkmcpherson/genesis_pickaplace.git \
      $LAB/gp_ladderN

HEAD **`c606b3ad`** = `known-good-2026-08-27-884-gc606b3ad`, which is the step-1 log commit on top
of the registration; `git merge-base --is-ancestor dda07d0 HEAD` → **YES**, so the tree contains
amendment (aa) and its revision 1. The three ladder-defining files and the relabel are byte-identical
to this checkout:

| file | sha256 |
|---|---|
| `baselines/rl/full_env.py` | `23fe428f222f1a3a…` |
| `baselines/genesis_can_env.py` | `40544bf73c8c69bd…` |
| `baselines/stage_predicates.py` | `a589b4f056322094…` |
| `baselines/rl/relabel_reward.py` | `8d7f8d705366bad3…` |

and so are the three launchers (`wmfix_full.sbatch` `872d0ed1…`, `sbatch_rlpd_e2e.sh` `030c3ea3…`,
`relabel_e2e_sets.sbatch` `deb263c5…`).

Note `genesis_can_env.py` carries the SAME hash as the pilot's stamp (`40544bf73c8c`) while
`full_env.py` and `stage_predicates.py` differ — Ladder N is entirely in those two files.

**The tree is PINNED here for the duration of the batch** (Lane 4's rule): pulling would change
`git describe` inside the `[ladder]` stamp, so jobs started after a pull would carry a different
`git=` suffix from jobs started before it, for no gain. Log commits after this point go to origin
only. If a CODE fix becomes necessary mid-batch, this log says which jobs ran which commit.

### `$W/r2dreamer_ladderN` (world-model port)

    rsync -a --exclude .venv --exclude .venv-ms --exclude runs --exclude wandb \
          --exclude '*.pt' --exclude __pycache__ --exclude r2dreamer.egg-info \
          ~/workspace/r2dreamer/ jstale02@login…:$W/r2dreamer_ladderN/
    # then, on the cluster:
    cd $W/r2dreamer_ladderN && git checkout -- .

653 MB, 757 files, 35 s. `git -C $W/r2dreamer_ladderN rev-parse --short HEAD` → **`0cf3d9e`**
("train.py: the step-accounting preflight compares env.steps with the loop's REAL starting
counter"), and `git status --porcelain --untracked-files=all` is **empty** — the cluster copy is
exactly the committed tree.

**Defect I made and fixed before it could matter:** `--exclude runs` is right for run OUTPUT but
`runs/` in this repo also holds seven TRACKED upstream launch scripts (`atari.sh`, `dmc.sh`, …), so
the first copy was missing tracked files (` D runs/*.sh` in `git status`). `git checkout -- .`
restored them; the status above is from after that. Nothing on our path reads `runs/*.sh` (our
logdir is `$W/runs`, a different place entirely), but a tree that does not equal its own HEAD is
exactly the provenance hole this branch exists to close.

Hashes verified against this box, after the checkout:

| file | sha256 (both machines) |
|---|---|
| `trainer.py` | `9a499792eb7a6850…` |
| `envs/genesis.py` | `2a2c0368836b44a0…` |
| `eval_genesis.py` | `b671550420269aad…` |
| `train.py` | `991535a16aad85a2…` |
| `demo_prefill.py` | `c59e031c838366f1…` |
| `configs/env/genesis_full_state.yaml` | `cfe8c088958a9cf5…` |

`tip_guard` appears 10× in `envs/genesis.py`, 8× in `train.py`, 3× in `eval_genesis.py` — Lane 12a's
wiring is present in the tree that will run.

**The launcher takes the world-model tree from a variable, not a hard-wire** (the Lane 4 trap):
`cluster/wmfix_full.sbatch:65` is
`R2_TREE=${R2_TREE:?set R2_TREE to the r2dreamer tree this run must use …}` — required, no default.
The old hardcoded `$W/r2dreamer_fix` survives only in the comment at line 62 explaining why it was
removed. Every submission below passes `R2_TREE=$W/r2dreamer_ladderN` explicitly, so no pilot tree
is touched.

Disk after both trees: **273 GB free** (the r2dreamer copy cost 4 GB, mostly `.git`).

---

## Step 3 — stage records and the six demonstration sets

### 3.0 The builder

`cluster/relabel_e2e_sets.sbatch` (Lane 4's) re-EXECUTES once per ladder. Six sets that way is six
simulations of the same 146 tapes (~3.5 h) and — the part that matters more than the time — it
leaves each set free to differ from the others by something other than its reward column.
`cluster/ladderN_sets.sbatch` (new, commit `b65bd07`) does what
`paper/LADDER_N_DEMO_CHECK_2026-09-11.md` §1 established instead:

* **phase 1** — one termination-suppressed re-execution per tape → a per-env-frame stage record.
  Ladder- and guard-INDEPENDENT by construction (`relabel_reward.run_record_shard`: `never_terminate`
  means nothing stops, and the record holds both guards' inputs).
* **phase 2** — every ladder applied OFFLINE to those records in numpy. Verified against direct
  re-execution by `--verify-against` and, independently, against Lane 5's own census re-execution,
  146/146 tapes identical in reward column AND terminal decision.

So all six sets come from ONE simulation on ONE node, and a difference between two of them can only
be the reward column. The script also re-opens both sets after each build and compares action
sha256 itself (`ACTION-SHA` lines) rather than trusting the builder's internal assertions.

Set names come from `relabel_reward.LADDER_SUFFIX` + `TIP_GUARD_SUFFIX`, which the tool ASSERTS
against `--ladder`/`--tip-guard` and refuses to write if they disagree. With `far_release` OFF and
`tip_guard=not_in_hand` (amendment (aa)) that is:

| ladder | suffix | human set | machine set |
|---|---|---|---|
| `nested_ramp` (v2) | `_rnrh` | `dHfull_all_rnrh` | `dDPfull_first_rnrh` |
| `nested_sparse` | `_rnsh` | `dHfull_all_rnsh` | `dDPfull_first_rnsh` |
| `staged` (control arm) | `_rzh` | `dHfull_all_rzh` | `dDPfull_first_rzh` |

Not `_rh`/`_rn` as the handoff brief guessed — `TIP_GUARD_IMPL_2026-09-11.md` §7(b) flagged exactly
this and declined to guess; these are the names the tool computes, and they are the names of record.

Sources (`$W/demos_state_full`), unchanged, checked before the build:

    dHfull_all      n_written 74  total_reward 118.0  n_pick 64  n_nopick 10  one_per_ic_first None
    dDPfull_first   n_written 72  total_reward 131.0  n_pick 63  n_nopick  9  one_per_ic_first True

No `stage_records*` directory existed on the cluster before this step.

### 3.1 FAILURE, verbatim: the 64-core guard fired on a node Slurm calls 64-core

First submission, job **3575659**, excluding the 33 batch nodes `sinfo` reports as not-64-CPU:

    [hw] host=pax012 cores=128 isa=avx512 model=Intel(R) Xeon(R) Gold 6438M
    FATAL: 128 cores != REQUIRE_CORES=64 -- refusing to build a set on the wrong hardware class
    JOB ENDED: STATE=FAILED

**The guard was right and my node filter was wrong.** `REQUIRE_CORES` is compared against
`grep -c '^processor' /proc/cpuinfo` = LOGICAL processors, while `sinfo -o %c` reports Slurm's
CONFIGURED CPUs. On pax012 Slurm advertises `64 CPUs, S2 C32 T1` and the machine actually presents
**128** logical processors (same Xeon Gold 6438M as pax080, hyper-threading enabled where pax080 has
it off). This is the documented "Slurm labels lie on this cluster" hazard (pax001 advertises
`broadwell` and is Cascade Lake) in a new place: not the feature string this time but the CPU count.

Corrected by using the 09-08 hardware census `$LAB/gp_e2e/hw_map.json` (READ ONLY — gp_e2e is a
do-not-touch tree), which records `cores` and `logical` separately from `/proc/cpuinfo`:

    of the 47 batch nodes Slurm calls 64-CPU:  20 are census logical==64
                                               10 are census logical==128  (pax036-046, pax056)
                                               17 are not in the 09-08 census at all (pax012 among them)

The 20 confirmed ones —
`pax004,005,015,019,030,031,032,033,054,055,058,059,060,061,078,079,080,146,148,149` — include
**pax080, the node that built the pilot's `_rz`/`_rs` sets** (`relabel_node` in all four
`repeat.json` files: `host pax080, cores 64, isa avx512, Xeon Gold 6438M`), so the new sets land in
the same hardware class as the sets they will be compared against. Resubmitted as job **3575953**
with the other 60 batch nodes excluded.

Cost: one minute of compute. Nothing was built on the wrong class, because the assertion runs before
the first tape.

### 3.2 What the cluster build has to reproduce, and from whom

Lane 12b landed `paper/R2D_LADDERN_SMOKE_2026-09-11.md` (commit `f9828d4`) into this shared
checkout while step 2 was running. It built the four Ladder-N sets LOCALLY, on a 32-core AVX2 box,
from Lane 7's records, under the same names this lane uses — so the cluster build is a
**reproduction on the 64-core class**, which is P-aa-7, and the local numbers are the targets:

| set | tapes | Σ reward recorded → relabelled | `home` tapes |
|---|---:|---|---:|
| `dHfull_all_rnrh` (ramp v2) | 74 | 118.0 → **215.9** | **13** |
| `dHfull_all_rnsh` (sparse) | 74 | 118.0 → **13.0** | **13** |
| `dDPfull_first_rnrh` (ramp v2) | 72 | 131.0 → **217.8** | **14** |
| `dDPfull_first_rnsh` (sparse) | 72 | 131.0 → **14.0** | **14** |

P-aa-7 allows ±1 tape per set. The `_rzh` control sets have no local counterpart; their comparison
is the pilot's own cluster-built `_rz` pair (Σ **171.0** human / **183.0** machine, built on pax080
by job 3537411), which differ from `_rzh` only in the tip guard.

The stamps Lane 12b measured, which mine must match field for field:

    ramp (v2): unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4
               ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | shaping=off |
               far_release=off | tip=tilt>60deg&not_in_hand@4f |
               full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
    sparse:    unified-2026-09-10 | ladder=nested_sparse | home=1 | max_return=1 |
               terminal=home+tipped | … same three hashes …

Those three hashes are the ones `$LAB/gp_ladderN` carries (§2), so the trees agree before a single
job runs. `farside` is absent from the ramp stamp's reward list — that absence IS revision 1.

**Two caveats inherited from Lane 12b, recorded here because this batch runs under them**
(`HANDOFF_2026-09-11.md` §7b):

1. **The r2dreamer trainer does not itself refuse legacy gates.** `train.py` never calls
   `full_env.refuse_legacy_gates()`, so a stray `FULLENV_*` gate is refused by
   `cluster/wmfix_full.sbatch`'s preflight and by nothing else. The fix exists on an r2dreamer
   worktree branch (`lane12b-fullenv-tipguard-fix`, `5e3a627`) and was deliberately NOT merged,
   because this lane was copying the committed tree to the cluster at that moment. **This batch
   therefore runs with launcher-side gate refusal only**, and `$W/r2dreamer_ladderN` is `0cf3d9e`
   without that commit. Both launchers do refuse, and no submission below sets any gate.
2. **Do not smoke this port on CPU with `model.compile=True`** — an inductor crash in CPU codegen
   on the first `agent.update()`, unrelated to the ladder work, reproduced on all three of Lane
   12b's CPU smokes. My {r2dreamer} smokes run on GPU, where it does not occur.

