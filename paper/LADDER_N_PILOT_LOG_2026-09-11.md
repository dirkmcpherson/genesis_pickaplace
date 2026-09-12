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

### 3.3 The build (job 3575953, pax146, 64 cores, AVX-512, Xeon Gold 6438M)

`COMPLETED`, **48:55**. Records: 10 min for the human set, 10 min for the machine set; the six
offline builds ~4.5 min each. `stage_records_2026-09-11/{dHfull_all,dDPfull_first}` = 146 records.

    GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN TIP_GUARD=not_in_hand \
      sbatch -J ln_sets --exclude=<the 60 non-verified batch nodes> cluster/ladderN_sets.sbatch

**THE SIX SETS OF RECORD** (`bash cluster/ladderN_verify_sets.sh`, which re-opens every tape
independently of the builder):

| set | ladder | tapes | Σ reward | `home` | `slide_event` | `nested_v2` | actions vs source |
|---|---|---:|---:|---:|---:|---:|---|
| `dHfull_all_rnrh` | nested_ramp v2 | 74 | **204.68** | **12** | 25 | 14 | 74/74 identical |
| `dDPfull_first_rnrh` | nested_ramp v2 | 72 | **206.22** | **12** | 23 | 15 | 72/72 identical |
| `dHfull_all_rnsh` | nested_sparse | 74 | **12.00** | **12** | 25 | 14 | 74/74 identical |
| `dDPfull_first_rnsh` | nested_sparse | 72 | **12.00** | **12** | 23 | 15 | 72/72 identical |
| `dHfull_all_rzh` | staged (control) | 74 | **171.00** | 12 | 25 | 14 | 74/74 identical |
| `dDPfull_first_rzh` | staged (control) | 72 | **183.00** | 12 | 23 | 15 | 72/72 identical |

Every set: `tip_guard=not_in_hand` (sustain 4 frames), `far_release=off`,
`relabel_node = {host pax146, cores 64, isa avx512, Xeon Gold 6438M, slurm_job 3575953}`, and both
machine sets carry `one_per_ic_first=True` **inherited** from the source manifest
(`selection_inherited {'one_per_ic_best': False, 'one_per_ic_first': True}`) rather than re-derived
from a CLI flag — the 2026-09-09 false-claim defect cannot recur here.

**The three builds of one arm have IDENTICAL grant tables** — `picked 65 / placed_v2 40 /
contact_push 9 / slide_success 12 / nested_v2 14 / farside 38 / slide_event 25 / home 12` for every
human set, and `64 / 41 / 13 / 13 / 15 / 40 / 23 / 12` for every machine set. Only the reward column
and the terminal differ. That is the record-once/score-many guarantee holding on real data.

Stamps, verbatim, and identical to Lane 12b's local ones in every field P1 compares:

    nested_ramp: unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4
                 ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | shaping=off |
                 far_release=off | tip=tilt>60deg&not_in_hand@4f |
                 full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
    nested_sparse: … | ladder=nested_sparse | home=1 | max_return=1 | terminal=home+tipped | …
    staged:        … | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 |
                   max_return=8 | terminal=slide_success+tipped | …

**Both launcher gates pass on all six.** The {RLPD} gate (`DRYRUN=1`) and the {r2dreamer} gate (its
own embedded block, run directly):

    DEMO-SHA dH       n=74 sha=ec8e7e2ab95f835a total_reward=204.675…  # dHfull_all_rnrh
    DEMO-SHA dDPfirst n=72 sha=b57f7c7944475d12 total_reward=206.224…  # dDPfull_first_rnrh
    DEMO-SHA dH       n=74 sha=32b3e4ed36e3b2a4 total_reward=12.0      # dHfull_all_rnsh
    DEMO-SHA dDPfirst n=72 sha=38f5e4a71ea95a6a total_reward=12.0      # dDPfull_first_rnsh
    DEMO-SHA dH       n=74 sha=4b3c0d62135fc015 total_reward=171.0     # dHfull_all_rzh
    DEMO-SHA dDPfirst n=72 sha=e98190455d408ffb total_reward=183.0     # dDPfull_first_rzh

### 3.4 An independent cross-check nobody asked for, and it PASSES

`dHfull_all_rzh` sums to **171.00** and `dDPfull_first_rzh` to **183.00** — *exactly* the pilot's
`dHfull_all_rz` and `dDPfull_first_rz`, which Lane 4 built on **pax080** by **direct re-execution**
under the **other** tip guard (job 3537411). The full grant tables match too (human
`picked 65 / placed_v2 40 / contact_push 9 / slide_success 12 / nested_v2 14`).

So the offline-from-records path on pax146 reproduces, to the unit, an independently produced
cluster set made by a different method on a different node. That is the strongest available
validation of the record-once/score-many pipeline, and it also says the **new tip guard costs zero
staged reward on the 64-core class** (locally Lane 7 measured a 2.0 cost on one human tape). The
guard does move terminations — `_rzh` ends 22 human / 23 machine tapes `tipped` — but those tapes
had already banked their rungs, so the total is unchanged. Good news for **P-aa-6**: the control arm
starts from a demonstration set that is reward-identical to the pilot's staged arm.

### 3.5 P-aa-7: MET on the human set, NOT MET on the machine set

P-aa-7 predicted the demonstration `home` counts reproduce on the 64-core class **within ±1 tape per
set** against this box's 32-core measurement.

| set | local (32-core, Lane 12b) | cluster (64-core, this build) | Δ | verdict |
|---|---:|---:|---:|---|
| human | 13 | **12** | −1 | **MET** (at the boundary) |
| machine | 14 | **12** | **−2** | **NOT MET** |

Σ reward moves the same way: human 215.89 → **204.68**, machine 217.83 → **206.22**.

**This is a registered prediction failing, and it is recorded as failed.** The direction is
consistent — the cluster class completes fewer slides than the 32-core box on both arms — which is
the hardware sensitivity of full-scope re-execution the amendment anticipated, at a magnitude that
exceeds the tolerance it registered on one of the two sets. It does NOT invalidate the batch: the
sets are internally consistent, hardware-homogeneous, and built on the class of record. What it
changes is what may be said about the demonstrations: **the sets these 20 jobs train on contain 12
human and 12 machine `home` tapes, not the 13/14 of amendment (aa) §P-aa-1.** Every later statement
about demonstration slide counts must use 12/12 and cite this build.

Note also that the count coincidence (12 = 12) is NOT a matched design — the two arms lost different
numbers of tapes to reach it.

The `home` tapes, by file (the npz carry no `ic_uid` key, so the per-uid identity check against
(aa)'s list `232 233 237 247 251 256 259 273 275 302 304 316 317` needs the source manifest's
index→uid map and was **not** run; only the counts above are established):

    human   : genesis-{100000-013, 100004-017, 101000-023, 103006-041, 103007-042, 104000-044,
                       104001-045, 104006-049, 104007-050, 105000-052, 106003-061, 107001-066}
    machine : genesis-{101000-000, 103016-003, 100009-020, 101007-026, 101014-028, 105006-052,
                       105012-054, 106000-057, 106003-058, 106019-064, 107000-066, 107006-067}

Re-execution fidelity, for the reader who wants to judge the pin rather than take it: can_dev p50
**13.0 mm**, max 685.7 (human) / 754.0 (machine), 39/41 tapes over 1 cm. The pilot's own pax080
build reads p50 8.1 / 10.8 mm with the same maxima — same family, so the large per-tape divergences
are a property of 600-decision full-scope re-execution, not of this build.

### 3.6 Tree note (see also §4.2)

The set build ran at `gp_ladderN` = **`b65bd076`** (that is the `git=` suffix inside every set's
stamp). The tree was then fast-forwarded to **`1f12d056`** to pick up `ladderN_verify_sets.sh` —
legitimate because **no `ln_*` training job had been submitted**, and verified harmless: the three
ladder file hashes are unchanged by the pull (`23fe428f222f`, `40544bf73c8c`, `a589b4f05632`). The
jobs will therefore stamp a different `git describe` from the sets they train on, with identical
ladder code. From the first job start the tree is PINNED.

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


---

## Step 4 — the five smokes

| job | name | learner | ladder | set | where |
|---|---|---|---|---|---|
| 3579596 | `ln_smoke_rl_ctl` | {RLPD} | staged | `dHfull_all_rzh` | batch/normal, CPU |
| 3579597 | `ln_smoke_rl_ramp` | {RLPD} | nested_ramp | `dHfull_all_rnrh` | batch/normal, CPU — **FAILED, see §4.1** |
| 3579598 | `ln_smoke_rl_sparse` | {RLPD} | nested_sparse | `dHfull_all_rnsh` | batch/normal, CPU |
| 3579619 | `ln_smoke_r2_ramp` | {r2dreamer} | nested_ramp | `dHfull_all_rnrh` | gpu/interactive |
| 3579620 | `ln_smoke_r2_sparse` | {r2dreamer} | nested_sparse | `dHfull_all_rnsh` | gpu/interactive |
| 3579832 | `ln_smoke_rl_ramp2` | {RLPD} | nested_ramp | `dHfull_all_rnrh` | re-run of 3579597 after the fix |

Command of record: `bash cluster/submit_ln_smokes.sh` (`ONLY=rl|r2` submits one half).

**Submission defect, trivial but logged:** both {r2dreamer} smokes were refused outright —

    sbatch: error: QOSMaxWallDurationPerJobLimit
    sbatch: error: Batch job submission failed: Job violates accounting/QOS policy

`cluster/wmfix_full.sbatch` bakes `-t 2-00:00:00`; the `interactive` QOS caps walltime at
**04:00:00** (`sacctmgr show qos interactive` → `MaxWall 04:00:00`, `cpu=16,gres/gpu=1,mem=64G`). Fixed
by adding `-t 0-03:00:00` to the smoke script's interactive block — the same 3 h the pilot's r2 smoke
used. That QOS also allows only one job at a time, so `ln_smoke_r2_sparse` queues behind
`ln_smoke_r2_ramp` by design, not by fault.

### 4.1 DEFECT: the {RLPD} demo gate was hardcoded to the STAGED rungs and refused every Ladder-N set

`ln_smoke_rl_ramp` (3579597) died **64 seconds in**, `FAILED 1:0`, with its `[ladder]` stamp already
printed correctly. Verbatim:

    Traceback (most recent call last):
      File ".../baselines/rl/full_demos.py", line 93, in segment_transitions_full
        assert round(float(v), 6) in ladder, f'{f}: reward {v} is not a sum of the staged ladder {sorted(ladder)}'
    AssertionError: .../demos_state_full/dHfull_all_rnrh/genesis-100000-013-256.npz:
      reward 0.017715517431497574 is not a sum of the staged ladder [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]

`segment_transitions_full` validated every per-decision reward against `_reachable_reward_sums()`,
built from `full_demos`' own legacy `STAGE_REWARD`. That is right for `staged` and wrong for Ladder N:
`nested_ramp` pays a **continuous** rung — `3.0 × min(1, slide_gain_m / 0.05)` credited on new minima —
so fractional per-decision values (0.0177 here) are the design working, not corruption.

**Blast radius had the smoke not run: all four {RLPD} `nested_ramp` jobs of amendment (aa) would have
died at startup**, i.e. a fifth of the batch, silently missing from the table rather than visibly
failing later. `nested_sparse` (pays 1.0) and the `staged` control were never at risk.

Fix (`a40c8aa`), in two parts:

1. The value check now comes from `full_env.LADDERS` via the named ladder, not from a second copy of
   the rungs in this module — a discrete ladder keeps the exact reachable-sum test with its own rungs,
   and a ramp ladder is bounded per decision **and on the episode return**, which is the tightest
   statement that is true of a continuous rung. Verified that `staged`'s value set is unchanged element
   for element, so no pre-existing run's gate moves.
2. **A new check, because the defect showed nothing was watching this:** the set's recorded ladder
   (`repeat.json` → `relabel.ladder`) must equal the ladder the run trains under (`train_rlpd` passes
   `args.ladder`). A buffer paying one objective inside an environment paying another is precisely the
   confound this branch exists to remove, and on the {RLPD} path nothing asserted it.

Tested locally against the real sets before resubmitting: `_rnrh` and `_rnsh` both load (74 tapes,
29 221 transitions, Σ 215.89 / 13.00 on this box's copies) and a deliberate `nested_ramp`-set-under-a-
`staged`-run is refused with the new message. Re-run as **3579832**.

### 4.2 Tree note for the batch

The fix landed while three smokes were running and **before any `ln_*` training job was submitted**,
so the pin is not yet in force. `$LAB/gp_ladderN` is now at `a40c8aa1`. Note the `[ladder]` stamps
carry a `-dirty` suffix: the launcher writes `cluster/RUN_REGISTRY.jsonl` and its Slurm `.out` files
inside the tree. The pilot's stamps read `-dirty` for the same reason; it is the launcher's own
bookkeeping, not an uncommitted code edit.

### 4.3 Smoke outcomes — all five stamp correctly

| job | outcome |
|---|---|
| 3579596 `ln_smoke_rl_ctl` | COMPLETED 5:15, TRAIN-OK, eval cell written |
| 3579597 `ln_smoke_rl_ramp` | FAILED 1:04 — the §4.1 defect |
| 3579598 `ln_smoke_rl_sparse` | COMPLETED 4:39, TRAIN-OK, eval cell written |
| 3579619 `ln_smoke_r2_ramp` | COMPLETED 36:02, train rc=0, milestone + eval cell written |
| 3579620 `ln_smoke_r2_sparse` | RUNNING, stamps printed (below) |
| 3579832 `ln_smoke_rl_ramp2` | COMPLETED 4:27, TRAIN-OK — the re-run after the fix |

**The stamps, verbatim.** The three ladders differ only in the four fields P1 allows (`ladder`, the
rungs, `max_return`, `terminal`); the three file hashes are identical everywhere, and within a
ladder the {RLPD} and {r2dreamer} lines are identical character for character apart from the
`git=` suffix (the three smokes ran at three commits as the fix landed):

    # nested_ramp -- {RLPD} 3579832 and {r2dreamer} 3579619, identical but for git=
    [ladder] unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4
             ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | shaping=off |
             far_release=off | tip=tilt>60deg&not_in_hand@4f |
             full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
    # nested_sparse -- {RLPD} 3579598 and {r2dreamer} 3579620, likewise
    [ladder] … | ladder=nested_sparse | home=1 | max_return=1 | terminal=home+tipped | …
    # staged control -- {RLPD} 3579596
    [ladder] … | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 |
             max_return=8 | terminal=slide_success+tipped | …

Checklist, item by item:

* `tip=tilt>60deg&not_in_hand@4f` — present on all five, plus the explicit
  `[ladder] tip_guard not_in_hand sustain 4 env frames`.
* `ramp:slide_gain_m=3/0.05m` — present on both `nested_ramp` stamps, i.e. **revision 1's scale 3
  and span 0.05 m**, and `farside=` is absent from the reward list, which is the revision's signature.
* `max_return` 9 / 1 / 8 as registered.
* **P-aa-5 holds on both {r2dreamer} arms:** `[ladder] ladder=nested_ramp tip_guard=not_in_hand
  return_clamp=9.0 (env.return_clamp AND model.return_clamp)` plus the trainer's own independent
  check `[ladder] return_clamp=9.0 (env and model agree)`; sparse reads `1.0` in both places.
* Demo gates pass on every set (`total_reward` 204.675 / 12.0 / 171.0 as built).
* Step accounting is right: `prefill 29406 decisions; trainer starts at counter step 117624;
  env.steps=15000 ONLINE env steps -> counter target 132624`.

**Evaluator, both learners, on a NESTED ladder — the success key follows the ladder.**

    # {r2dreamer} 3579619 (nested_ramp)
    [eval] ladder='nested_ramp' … [eval] tip_guard='not_in_hand' from the run config
    [eval] outcome success_key='home' (the ladder's paid terminal)
    [eval] 15 episodes (mode, demo ICs): home 0.00  tipped 0.00  timeout 1.00  mean_steps 300
    # its metrics.json: success_key home | ladder nested_ramp | tip_guard not_in_hand
    #                   max_return 9.0 | terminal_stages ['home','tipped']

    # {RLPD} 3579598 (nested_sparse) metrics.json
    terminal_stage: home | ladder_provenance: ladder nested_sparse, tip_guard not_in_hand,
      max_return 1.0, terminal ['home','tipped']
    outcomes {'home': 0.0, 'tipped': 0.0, 'timeout': 1.0}
    stages   {… 'farside': 0.0, 'slide_event': 0.0, 'home': 0.0 …}

All zeros, as they must be for a 1000-decision / 15k-step untrained policy; what is being checked
here is the plumbing, not the policy. The `stages` block carries `farside`, `slide_event` and
`home`, so the Ladder-N columns exist in the cells the batch will produce.

**Known limitation carried into the batch, not introduced by it** (Lane 12a §7(f)):
`eval_genesis.py` never passes `far_release` to the adapter, so a {r2dreamer} cell always scores
`far_release=False`. Harmless here — this batch runs `far_release` OFF — but it must be fixed
before any `far_release` run.

**One readout gap worth flagging now rather than at analysis time:** `slide_gain_m` is emitted in
`info` by `full_env` but is NOT in `full_env.LOGGED_STAGES` (which carries `farside`,
`slide_event`, `home`), nor in the adapter's `FULL_EXTRA_KEYS`. **P-aa-3** asks for
`slide_gain_m > 0` in training rollouts. The available proxies are `slide_event` (which requires
≥ 1 cm of credited gain) and, under `nested_ramp`, any episode return above 2.0 — which is ramp
money and therefore proves gain > 0. A gain of, say, 5 mm would be invisible to both. Flagged for
the readout lane; no code changed, since changing what is logged mid-batch would split the batch.

---

## Step 5 — the 20 `ln_*` jobs, submitted

`bash cluster/submit_ln_batch.sh` from `$LAB/gp_ladderN` @ **`a40c8aa1`**
(`known-good-2026-08-27-895-ga40c8aa1-dirty` — the `-dirty` is the launcher's own
`RUN_REGISTRY.jsonl`, §4.2), r2 tree `$W/r2dreamer_ladderN` @ **`0cf3d9e`**, 272 GB free.

| job | name | learner | ladder | arm | seed | demo set | budget | checkpoints / milestones | QOS |
|---|---|---|---|---|---:|---|---|---|---|
| 3581558 | `ln_rl_ramp_dH_s950` | {RLPD} | nested_ramp | human | 950 | `dHfull_all_rnrh` | 250k dec | 0.16/0.4/1.0 | preempt |
| 3581559 | `ln_rl_ramp_dH_s951` | {RLPD} | nested_ramp | human | 951 | `dHfull_all_rnrh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581560 | `ln_rl_ramp_dM_s970` | {RLPD} | nested_ramp | machine-first | 970 | `dDPfull_first_rnrh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581561 | `ln_rl_ramp_dM_s971` | {RLPD} | nested_ramp | machine-first | 971 | `dDPfull_first_rnrh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581562 | `ln_rl_sparse_dH_s955` | {RLPD} | nested_sparse | human | 955 | `dHfull_all_rnsh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581563 | `ln_rl_sparse_dH_s956` | {RLPD} | nested_sparse | human | 956 | `dHfull_all_rnsh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581564 | `ln_rl_sparse_dM_s975` | {RLPD} | nested_sparse | machine-first | 975 | `dDPfull_first_rnsh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581565 | `ln_rl_sparse_dM_s976` | {RLPD} | nested_sparse | machine-first | 976 | `dDPfull_first_rnsh` | 250k | 0.16/0.4/1.0 | preempt |
| 3581566 | `ln_rl_ctl_dH_s958` | {RLPD} | staged (control) | human | 958 | `dHfull_all_rzh` | 100k | 0.4/1.0 | preempt |
| 3581567 | `ln_rl_ctl_dH_s959` | {RLPD} | staged (control) | human | 959 | `dHfull_all_rzh` | 100k | 0.4/1.0 | preempt |
| 3581568 | `ln_rl_ctl_dM_s978` | {RLPD} | staged (control) | machine-first | 978 | `dDPfull_first_rzh` | 100k | 0.4/1.0 | preempt |
| 3581569 | `ln_rl_ctl_dM_s979` | {RLPD} | staged (control) | machine-first | 979 | `dDPfull_first_rzh` | 100k | 0.4/1.0 | preempt |
| 3581570 | `ln_r2_ramp_dH_s950` | {r2dreamer} | nested_ramp | human | 950 | `dHfull_all_rnrh` | 2M online | 0.5M/1M/2M | normal |
| 3581571 | `ln_r2_ramp_dH_s951` | {r2dreamer} | nested_ramp | human | 951 | `dHfull_all_rnrh` | 2M | 0.5M/1M/2M | normal |
| 3581572 | `ln_r2_ramp_dM_s970` | {r2dreamer} | nested_ramp | machine-first | 970 | `dDPfull_first_rnrh` | 2M | 0.5M/1M/2M | normal |
| 3581573 | `ln_r2_ramp_dM_s971` | {r2dreamer} | nested_ramp | machine-first | 971 | `dDPfull_first_rnrh` | 2M | 0.5M/1M/2M | normal |
| 3581574 | `ln_r2_sparse_dH_s955` | {r2dreamer} | nested_sparse | human | 955 | `dHfull_all_rnsh` | 4M online | 0.5M/1M/2M/4M | normal |
| 3581575 | `ln_r2_sparse_dH_s956` | {r2dreamer} | nested_sparse | human | 956 | `dHfull_all_rnsh` | 4M | 0.5M/1M/2M/4M | normal |
| 3581576 | `ln_r2_sparse_dM_s975` | {r2dreamer} | nested_sparse | machine-first | 975 | `dDPfull_first_rnsh` | 4M | 0.5M/1M/2M/4M | normal |
| 3581577 | `ln_r2_sparse_dM_s976` | {r2dreamer} | nested_sparse | machine-first | 976 | `dDPfull_first_rnsh` | 4M | 0.5M/1M/2M/4M | normal |

Exactly the (aa) table: {RLPD} ramp 2v2 at 250k, {r2dreamer} ramp 2v2 at 2M, {RLPD} sparse 2v2 at
250k, {r2dreamer} sparse 2v2 at 4M, {RLPD} staged control 2v2 at 100k, every job
`TIP_GUARD=not_in_hand`, `far_release` off, each ladder paired with the set whose suffix matches it.

**QOS split, disclosed:** the 8 {r2dreamer} runs on QOS `normal` (they are the long pole and a
preempted world-model run restarts from step 0), the 12 {RLPD} runs on the preempt allocation where
the capacity is. The split is BETWEEN LEARNERS and never between arms, so it cannot touch the
human-versus-machine contrast this batch makes; a cross-learner reading carries it as a scheduling
difference. State at submission: 2 running (`ln_rl_ramp_dH_s950/951` on pax048), 18 pending on
Priority/Resources behind the four `lz_rl_sparse` pilot jobs still holding normal-QOS GPUs.

**From here the tree is PINNED at `a40c8aa1`.** Run dirs: {RLPD}
`$LAB/gp_ladderN/baselines/rl/checkpoints/e2e/e2e_rlpd_<ARM>_s<seed>` with logs
`$LAB/gp_ladderN/e2e_rlpd_<jobid>.out`; {r2dreamer} `$W/runs/full_r2d_state_<set>_s<seed>` with logs
`$W/slurm/ln_r2_<ladder>_<arm>_s<seed>_<jobid>.out`.

**Pick up with**

    squeue -u jstale02 -o "%.10i %.22j %.9T %.6M %.11q %R" | grep ln_
    # P1 audit: the four compared fields must be identical within a ladder
    grep -h '^\[ladder\] unified' $W/slurm/ln_r2_*.out $LAB/gp_ladderN/e2e_rlpd_*.out | sort -u
    # P6: no job may end FAILED 2:0 00:00:00
    sacct -S 2026-09-11 -u jstale02 -X -n --format=JobName%24,State%14,Elapsed,ExitCode | grep ln_

---

## Step 6 (optional) — pinned re-score of the pilot's staged {RLPD} checkpoints

`cluster/ln_pilot_rescore.sbatch`, submitted as array **3581709** (`--array=0-3`), one task per
pilot staged checkpoint: `e2e_rlpd_{dH_s940, dH_s941, dDPfirst_s960, dDPfirst_s961}/rlpd_final.zip`
(all four verified present, sidecars `ladder staged`, `steps 100000`).

It rolls each out on `rnd30` in `mode` — the pilot's cell of record — **under the ladder and guard
the policy trained on**, and writes a per-episode stage record, so `slide_event` and `home` can be
scored offline afterwards with no second simulation. The sidecars say `tip_guard: None`, i.e. they
predate the argument, so the evaluator takes the class default `grip`, which IS the rule of record
those runs ran under; nothing passes `--tip-guard`, because `not_in_hand` would move where episodes
terminate and make the cells incomparable with the pilot's own.

`$LAB/gp_unified` is **read only** here — checkpoints in, every byte out to
`$W/pilot_rescore_2026-09-11/`, code from `$LAB/gp_ladderN` — so no running `lz_*` job is disturbed.

Worth knowing before anyone compares guards: `eval_e2e.py --require-cores` counts **physical**
cores (the project's guard of record for cells), while `cluster/ladderN_sets.sbatch`'s
`REQUIRE_CORES` counts **logical** processors from `/proc/cpuinfo`. Same name, same number 64, two
different measurements — which is why §3.1's failure happened on a node that satisfies one and not
the other.

### 6.1 MY ERROR: I broke the tree pin to deploy that script, then restored it

To get `ln_pilot_rescore.sbatch` onto the cluster I ran `git pull` in `$LAB/gp_ladderN` **after the
20 `ln_*` jobs were already submitted and 9 were running**, moving the tree `a40c8aa1 → 279a3134`.
That is exactly the split §4.2 and Lane 4's rule exist to prevent: jobs that had already started
stamped `…-895-ga40c8aa1-dirty`, and the 12 still pending would have stamped `…-898-g279a3134-dirty`.
I also ran `git stash` first, which briefly removed `cluster/RUN_REGISTRY.jsonl` — a file the
running launchers append to.

Both are repaired, and the repair is verifiable rather than asserted:

* `git stash pop` restored the registry (4 lines, intact).
* `git diff --name-only a40c8aa1 279a3134` is **exactly two files**:
  `cluster/ln_pilot_rescore.sbatch` (new, imported by no training job) and this log. `git diff
  --stat` over `full_env.py`, `genesis_can_env.py`, `stage_predicates.py`, `full_demos.py`,
  `train_rlpd.py`, `eval_e2e.py` is **empty** — no code any `ln_*` job loads was touched.
* `git reset a40c8aa1` (mixed, so the working tree and the untracked launcher files are untouched)
  put the pin back. `git describe --always --dirty` now prints
  **`known-good-2026-08-27-895-ga40c8aa1-dirty`**, character for character what the running jobs
  printed, and the three ladder hashes are unchanged (`23fe428f222f`, `40544bf73c8c`,
  `a589b4f05632`). So all 20 jobs will stamp ONE commit after all.

The re-score array keeps its own spooled copy of the script (Slurm spools at submission), so it runs
regardless; its `git describe` line will read `a40c8aa1` while its script text is the `279a3134`
version. That affects a re-score, not the batch, and it is stated here rather than left to be found.

**What I should have done:** put the re-score script somewhere outside the pinned tree (the
scratch dir, or a second clone) and pointed `GENESIS_PICKAPLACE_ROOT` at `gp_ladderN` from there.
The pin exists so that "which code ran" has ONE answer per batch; reaching for `git pull` to move a
file is how that guarantee gets spent for no gain.

### 6.2 Re-score resubmitted and running

First array **3581709** FAILED `1:0` on all four tasks in 4 s, verbatim:

    /cluster/tufts/apps/manual/9/.../conda/deactivate.d/qt-main_deactivate.sh: line 5:
    CONDA_BACKUP_QT_XCB_GL_INTEGRATION: unbound variable

My `set -euo pipefail`: `conda activate` sources the cluster's qt deactivate hook, which reads an
unset variable and dies under `-u`. Every other launcher in `cluster/` uses `set -eo pipefail` for
exactly this reason. Fixed to `set -eo pipefail` with the reason in the header.

**Deployed the fix the way §6.1 says it should have been done the first time:** the corrected script
was `scp`-ed to `$W/ln13_rescore.sbatch` — **outside** the pinned tree — and submitted from there
with `GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN`. The pin did not move:
`known-good-2026-08-27-895-ga40c8aa1-dirty` before and after.

Array **3581786**, all four tasks RUNNING, and the hardware guard reports what it landed on:

    [eval-e2e] 30 start(s) from baselines/eval_ics.json:rnd (offset 0, isolation shared_process)
      node=pax019 pid=571154 cores=64p/64l affinity=8 threads=1 isa=avx512
      cpu="Intel(R) Xeon(R) Gold 6438M" role=record

`64p/64l` — this node satisfies both the physical guard `eval_e2e` enforces and the logical one the
set builder uses. Output lands in `$W/pilot_rescore_2026-09-11/<run>/{rnd_mode,records_rnd_mode}/`.

---

## Status at handoff

* **20 `ln_*` jobs live** (3581558–3581577): 8 running, 12 pending, **0 failures**. Every started
  job stamps `known-good-2026-08-27-895-ga40c8aa1-dirty` and its registered ladder.
* **Six demonstration sets built and verified** on pax146 (64 cores, AVX-512), actions byte-identical
  to their sources, both launcher gates passing.
* **P-aa-7 FAILED on the machine set** (§3.5) — the batch trains on **12 human / 12 machine** `home`
  tapes, not 13/14.
* **Re-score array 3581786** running on the pilot's four staged checkpoints.
* Trees: `$LAB/gp_ladderN` PINNED at `a40c8aa1`; `$W/r2dreamer_ladderN` at `0cf3d9e`. No
  do-not-touch tree was modified; `$LAB/gp_unified` was read only, for checkpoints.
* Disk 270 GB free.

**Not done, for whoever picks this up:** no health monitor is armed for this batch (this session's
would die with it). The readout commands are at the end of §5, and the `slide_gain_m` logging gap is
in §4.3.

---

## Extension (aa rev 2)

Registered `paper/PHASE_PLAN_2026-09-04.md` (aa) REVISION 2, commit `5dcd380`, BEFORE any of the
eight jobs below was submitted. Trees unchanged and unpulled: `$LAB/gp_ladderN` @ **`a40c8aa1`**,
`$W/r2dreamer_ladderN` @ **`0cf3d9e`**. Disk at submission: **265 GB free** (≥ 150 GB floor).

**Why (repeated from the registration):** two of the four running `nested_ramp` 2M {r2dreamer}
seeds (3581570–73) already show `home` in their training records at ~1M steps (`s950`: 1, `s970`:
2), and the pilot's own staged ladder gave ~0 honest nests at 2M but 0.167 at 4.1M
(PHASE_RESULTS §5.6) — 2M is the budget most likely to stop just before the behaviour. The
launcher's requeue path restarts a run from step 0, not from its buffer, so an extension is new
seeds, never a resumed one.

### QOS state at submission (disclosed discrepancy)

    squeue -u jstale02 -o "%.10i %.28j %.9T %.10P %.10q %.8b"
    -> normal QOS: 9 of 10 GPU slots in use (8 `ln_r2_*` + 1 surviving pilot job
       `lz_rl_sparse_dM_s965`) -- only 1 free, not the 2 the brief assumed.

Two {r2dreamer} seeds were submitted to normal QOS anyway (one human, one machine, so the split
does not sit on one side of the arm contrast); the second of those queued rather than being
refused (Slurm holds a submission over the running cap as PENDING/`QOSMaxGRESPerUser`, it does not
reject it). The other two {r2dreamer} seeds and all four {RLPD} seeds went to preempt.

### Jobs submitted (8)

**{r2dreamer}, 4M online steps, milestones `[500000,1000000,2000000,4000000]`, ladder
`nested_ramp`, `tip_guard=not_in_hand`, `far_release` off — exact command form of
`cluster/submit_ln_batch.sh`'s `sub_r2`, only seed/steps/milestones changed:**

    MILES="[500000,1000000,2000000,4000000]"
    env R2_TREE=$W/r2dreamer_ladderN GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN LADDER=nested_ramp \
        TIP_GUARD=not_in_hand R2_LONG_RUN=1 R2_MILESTONES="$MILES" \
        sbatch -J ln_r2_ramp4M_dH_s952 -p gpu --qos=normal \
        $LAB/gp_ladderN/cluster/wmfix_full.sbatch dHfull_all_rnrh 952 4000000
    # (same form) -J ln_r2_ramp4M_dM_s972 -p gpu --qos=normal        ... dDPfull_first_rnrh 972 4000000
    # (same form) -J ln_r2_ramp4M_dH_s953 -p preempt --qos=preempt   ... dHfull_all_rnrh    953 4000000
    # (same form) -J ln_r2_ramp4M_dM_s973 -p preempt --qos=preempt   ... dDPfull_first_rnrh 973 4000000

| job | name | QOS/partition | state at submit |
|---|---|---|---|
| **3591975** | `ln_r2_ramp4M_dH_s952` | normal / gpu | RUNNING immediately (pax010) |
| **3591976** | `ln_r2_ramp4M_dM_s972` | normal / gpu | PENDING, `QOSMaxGRESPerUser` |
| **3591977** | `ln_r2_ramp4M_dH_s953` | preempt / preempt | RUNNING immediately (pax064) |
| **3591978** | `ln_r2_ramp4M_dM_s973` | preempt / preempt | RUNNING immediately (pax064) |

**{RLPD}, 500k decisions, checkpoints 100k/250k/500k (`--ckpt-fracs 0.2,0.5,1.0`), ladder
`nested_ramp`, `tip_guard=not_in_hand`, `far_release` off — exact command form of
`cluster/submit_ln_batch.sh`'s `sub_rl`, only seed/steps/fracs changed (user: "just put in the
extensions"):**

    env GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN LADDER=nested_ramp TIP_GUARD=not_in_hand \
        ARM=dH SEED=952 STEPS=500000 DEMO=$W/demos_state_full/dHfull_all_rnrh \
        CKPT_FRACS=0.2,0.5,1.0 \
        sbatch -J ln_rl_ramp500k_dH_s952 -p preempt --qos=preempt --nice=0 cluster/sbatch_rlpd_e2e.sh
    # (same form) -J ln_rl_ramp500k_dH_s953  ARM=dH       SEED=953  DEMO=.../dHfull_all_rnrh
    # (same form) -J ln_rl_ramp500k_dM_s972  ARM=dDPfirst SEED=972  DEMO=.../dDPfull_first_rnrh
    # (same form) -J ln_rl_ramp500k_dM_s973  ARM=dDPfirst SEED=973  DEMO=.../dDPfull_first_rnrh

| job | name | QOS/partition | state at submit |
|---|---|---|---|
| **3591981** | `ln_rl_ramp500k_dH_s952` | preempt / preempt | PENDING, `Priority` |
| **3591982** | `ln_rl_ramp500k_dH_s953` | preempt / preempt | PENDING, `Priority` |
| **3591983** | `ln_rl_ramp500k_dM_s972` | preempt / preempt | PENDING, `Priority` |
| **3591984** | `ln_rl_ramp500k_dM_s973` | preempt / preempt | PENDING, `Priority` |

All four queue behind the running preempt jobs; that is a scheduling delay, not a failure or a
refusal — nothing about the submission itself was rejected.

### Stamp verification

The three {r2dreamer} jobs that started immediately (3591975, 3591977, 3591978) print the
identical ladder line, character for character but for `git=` and the run name, matching the
batch's own ramp stamp exactly:

    [ladder] unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4
             ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | shaping=off |
             far_release=off | tip=tilt>60deg&not_in_hand@4f |
             full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 |
             git=known-good-2026-08-27-895-ga40c8aa1-dirty
    [ladder] tip_guard not_in_hand sustain 4 env frames
    [ladder] ladder=nested_ramp tip_guard=not_in_hand return_clamp=9.0 (env.return_clamp AND model.return_clamp)
    [ladder] return_clamp=9.0 (env and model agree)

Tree pin confirmed unmoved after all eight submissions: `git -C $LAB/gp_ladderN describe
--always --dirty` still reads `known-good-2026-08-27-895-ga40c8aa1-dirty`.

**Still pending at the time of this entry, to be confirmed once the jobs actually run:**
`3591976` (queued on `QOSMaxGRESPerUser`) and all four `ln_rl_ramp500k_*` (queued on `Priority`);
the `Step accounting [R2_LONG_RUN]` target line (expected `start + 4000000`) had not yet appeared
in any of the three running {r2dreamer} logs at first check (still in the model-compile /
env-creation stage). Follow-up check appended below once available.

### Follow-up (~5 min later): `Step accounting` confirmed on all three running {r2dreamer} jobs

    # s952 (dH, normal), s953 (dH, preempt) -- prefill 29406 decisions, start counter 117624
    Step accounting [R2_LONG_RUN]: prefill 29406 decisions; trainer starts at counter step
      117624 (env frames); env.steps=4000000 ONLINE env steps -> counter target 4117624.
    # s973 (dM, preempt) -- prefill 37488 decisions (72-tape machine set), start counter 149952
    Step accounting [R2_LONG_RUN]: prefill 37488 decisions; trainer starts at counter step
      149952 (env frames); env.steps=4000000 ONLINE env steps -> counter target 4149952.

Both targets are exactly `start + 4000000`, as required. Demo prefill matches the running batch's
own sets exactly (74 human / 72 machine tapes, `transitions_added` 29406 / 37488, same
`terminal_reward_values` and `eviction_note`) — same demo sets, not rebuilt. Tree pin re-confirmed
unmoved: `known-good-2026-08-27-895-ga40c8aa1-dirty`.

**Not yet confirmable** at handoff: job `3591976` (`ln_r2_ramp4M_dM_s972`, normal QOS, still
`PENDING QOSMaxGRESPerUser`) and all four `ln_rl_ramp500k_*` (preempt, `PENDING
QOSMaxGRESPerUser` — the preempt allocation reached its own 20-GPU cap once the extension's
preempt jobs joined the running batch's 12). None of the eight jobs has failed, been refused, or
required a code change; the remaining five are a queueing wait, not a defect. `sacct` at this
point shows all eight `0:0` (no exit yet, none `FAILED 2:0 00:00:00`). Whoever next has cluster
access should re-check `3591976`'s stamp (expect the same ramp/tip/hash line, machine-set prefill
37488/149952) and the four RLPD jobs' `[ladder]` lines and `nested_ramp`-vs-set assertion once they
start, per the same P1 checklist as the main batch.

---

## RLPD 100k readout

Lane RL100 (2026-09-11 into 09-12). Full readout: `paper/RL100_READOUT_2026-09-11.md`. Answers
HANDOFF §7b: `episode_rollouts.jsonl` lacks `ep_farside`/`ep_slide_event`/`ep_home`, so whether any
{RLPD} Ladder N run has slid was unknown until its checkpoints were evaluated. Read-only on
`gp_ladderN` and on every checkpoint it wrote (each copied to `$W/rl100_2026-09-11/<run>/ckpt_100/`
— with `.provenance.txt` recording source dir + mtime — before any evaluator touched it); the tree
stayed pinned at `a40c8aa1` throughout, unmoved.

All 12 checkpoints reporting `ckpt_step=100000` in their own sidecar appeared within 18 minutes
(inside the 60-minute polling budget); `ckpt_040` for the two 250k-budget recipes (`nested_ramp`,
`nested_sparse`, since 100 000/250 000 = 40 %), `ckpt_100` (the final checkpoint) for the 100k-budget
staged+tip-guard control. One Slurm job per run, 3 cells each (`rnd30_mode`, `hold15_mode`,
`rnd30_sample` — the third added because it was cheap), `--role record --require-cores 64`,
`-p batch --qos=normal`, `--nodelist` restricted to the 55 nodes `$LAB/gp_e2e/hw_map.json`
(read-only) confirms are 64 physical cores — the census that also explains which nodes satisfied
Lane 13's `dpRS_*`/`ln_rescore` jobs. No `--ladder`/`--tip-guard` passed (default-from-sidecar).

| job | run | recipe | arm | seed | node | elapsed | state |
|---:|---|---|---|---:|---|---|---|
| 3594375 | `e2e_rlpd_dH_s950` | nested_ramp | human | 950 | pax146 | 30:27 | COMPLETED |
| 3594376 | `e2e_rlpd_dH_s951` | nested_ramp | human | 951 | pax146 | 34:06 | COMPLETED |
| 3594377 | `e2e_rlpd_dDPfirst_s970` | nested_ramp | machine-first | 970 | pax146 | 22:37 | COMPLETED |
| 3594378 | `e2e_rlpd_dDPfirst_s971` | nested_ramp | machine-first | 971 | pax146 | 26:56 | COMPLETED |
| 3594379 | `e2e_rlpd_dH_s955` | nested_sparse | human | 955 | pax146 | 25:35 | COMPLETED |
| 3594593 | `e2e_rlpd_dH_s956` | nested_sparse | human | 956 | pax146 | 25:57 | COMPLETED |
| 3594869 | `e2e_rlpd_dDPfirst_s975` | nested_sparse | machine-first | 975 | pax046 | 31:10 | COMPLETED |
| 3594724 | `e2e_rlpd_dDPfirst_s976` | nested_sparse | machine-first | 976 | pax019 | 20:59 | COMPLETED |
| 3594729 | `e2e_rlpd_dH_s958` | staged (control) | human | 958 | pax046 | 22:49 | COMPLETED |
| 3594735 | `e2e_rlpd_dH_s959` | staged (control) | human | 959 | pax046 | 29:20 | COMPLETED |
| 3594877 | `e2e_rlpd_dDPfirst_s978` | staged (control) | machine-first | 978 | pax045 | 28:15 | COMPLETED |
| 3594885 | `e2e_rlpd_dDPfirst_s979` | staged (control) | machine-first | 979 | pax045 | 33:24 | COMPLETED |

All 12 `COMPLETED 0:0`; all 36 expected `metrics.json` (900 episodes) exist; every cell stamps
`git=a40c8aa1`, `require_cores=64`, `core_counts=[64]`.

**Headline: of 900 episodes, exactly ONE reaches `home`** — `e2e_rlpd_dDPfirst_s979` (staged
control, machine-first), `rnd30_sample` ep19, the full `farside`→`slide_event`→`nested_v2`→`home`
chain firing together on a run that is not even paid for it. **No `nested_ramp` or `nested_sparse`
run — the ladders built to pay for this — has a single `home` at 100k decisions** (630 episodes
across those 8 runs). `farside` is comparatively common (both ladders, both arms); `slide_event` is
rare (7/900, 4 seeds); the conjunction essentially never lands yet. Full per-run × per-cell tables,
recipe × arm means, and the per-episode trace of every `farside`/`slide_event`-without-`home` case:
`paper/RL100_READOUT_2026-09-11.md` §4–§6. NOT a source or ladder comparison (n=2 per arm, one
early checkpoint, exactly as amendment (aa) registered).

Scripts of record (outside the pinned tree, per §6.1's own corrected practice):
`$W/rl100_2026-09-11/rl100_eval.sbatch`, `$W/rl100_2026-09-11/summarize.py`,
`$W/rl100_2026-09-11/nodelist64.txt`.

---

## Revision 3 (Lane 14, 2026-09-12) — 12 jobs, equal-n ignition read + the world-model guard control

Registered `paper/PHASE_PLAN_2026-09-04.md` **(aa) REVISION 3**, commit `4cf58af`, BEFORE any of the
twelve jobs below was submitted. Trees unchanged and unpulled: `$LAB/gp_ladderN` @ **`a40c8aa1`**,
`$W/r2dreamer_ladderN` @ **`0cf3d9e`** (the launcher printed both back at submission, verbatim:
`DISK-OK 259 GB free | tree /cluster/tufts/shortlab/jstale02/gp_ladderN
(known-good-2026-08-27-895-ga40c8aa1-dirty) | r2 …/r2dreamer_ladderN (0cf3d9e)`). Disk at
submission: **259 GB free** (≥ the registered 150 GB floor).

Submit command of record: `cluster/submit_ln_rev3.sh` (commit `acabb4b` in this branch), deployed to
the cluster as `$W/ln14_submit_rev3.sh` — **outside** the pinned tree, per §6.1's lesson — and run
from there. `DRYRUN=1` printed the twelve commands first; they are reproduced by the script itself.

### A defect in the registration, found before submission and worked around, not improvised past

Revision 3 registers {RLPD} `nested_sparse` seeds **s957–958 / s977–978**. The {RLPD} launcher names
its run directory `$OUT_ROOT/e2e_rlpd_${ARM}_s${SEED}` (`cluster/sbatch_rlpd_e2e.sh:98–100`) with **no
ladder, set or budget component**, and the batch's staged-control arm already owns seeds **s958 (dH)**
and **s978 (dDPfirst)**:

    ls $LAB/gp_ladderN/baselines/rl/checkpoints/e2e/
    -> ... e2e_rlpd_dH_s958  e2e_rlpd_dDPfirst_s978 ...     # jobs 3581566 and 3581568

`sbatch_rlpd_e2e.sh` has **no exists-guard on a fresh start** (the `rm -rf "$OUT"` at line 161 runs
only on a requeue), and `cluster/run_registry.py check` refuses only a **FULL-key** match — different
ladder, demo set and budget, so it would have passed. Submitting the registered seeds into the default
root would therefore have had two new jobs write into the run directories of `ln_rl_ctl_dH_s958`
(3581566, which COMPLETED `0:0` at 04:10:01, ~5 min before this submission, and whose cells live in
that directory) and `ln_rl_ctl_dM_s978` (3581568, **still RUNNING** at submission time).

**Resolution, disclosed:** all four {RLPD} rev-3 runs use
`OUT_ROOT=baselines/rl/checkpoints/e2e_rev3`. `OUT_ROOT` is the launcher's own documented knob
(header line 28). Seeds, arms, demo sets, ladder, budget, checkpoint fractions, tip guard and job
names are **exactly as registered**; only the storage root moves, and it moves symmetrically for both
arms, so it cannot touch the human-versus-machine contrast. `submit_ln_rev3.sh` additionally refuses
to submit over any run dir that already exists, so the class of accident cannot recur silently.

**Readout consequence, for whoever writes the {RLPD} table:** the four rev-3 {RLPD} runs are NOT under
`baselines/rl/checkpoints/e2e/`. Any glob that assumes one root will miss them.

### Jobs submitted (12), all PENDING at submission

**{r2dreamer} `nested_sparse`, 4M online steps, milestones `[500000,1000000,2000000,4000000]`,
`TIP_GUARD=not_in_hand`, `far_release` off, clamp 1.0 (from the ladder):**

| job | name | arm | seed | demo set | QOS/partition | state at submit |
|---|---|---|---:|---|---|---|
| **3596021** | `ln_r2_sparse_dH_s957` | human | 957 | `dHfull_all_rnsh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596022** | `ln_r2_sparse_dH_s958` | human | 958 | `dHfull_all_rnsh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596023** | `ln_r2_sparse_dM_s977` | machine-first | 977 | `dDPfull_first_rnsh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596024** | `ln_r2_sparse_dM_s978` | machine-first | 978 | `dDPfull_first_rnsh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |

**{r2dreamer} `staged` + the new tip guard (the world-model CONTROL), 1M online steps, milestones
`[500000,1000000]`, clamp 8.0 (from the ladder):**

| job | name | arm | seed | demo set | QOS/partition | state at submit |
|---|---|---|---:|---|---|---|
| **3596025** | `ln_r2_ctl_dH_s962` | human | 962 | `dHfull_all_rzh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596026** | `ln_r2_ctl_dH_s963` | human | 963 | `dHfull_all_rzh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596027** | `ln_r2_ctl_dM_s982` | machine-first | 982 | `dDPfull_first_rzh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |
| **3596028** | `ln_r2_ctl_dM_s983` | machine-first | 983 | `dDPfull_first_rzh` | normal / gpu | PENDING `QOSMaxGRESPerUser` |

**{RLPD} `nested_sparse`, 500k decisions, checkpoints 100k/250k/500k (`--ckpt-fracs 0.2,0.5,1.0`),
`OUT_ROOT=baselines/rl/checkpoints/e2e_rev3`:**

| job | name | arm | seed | demo set | QOS/partition | state at submit |
|---|---|---|---:|---|---|---|
| **3596029** | `ln_rl_sparse500k_dH_s957` | human | 957 | `dHfull_all_rnsh` | preempt / preempt | PENDING `Priority` |
| **3596030** | `ln_rl_sparse500k_dH_s958` | human | 958 | `dHfull_all_rnsh` | preempt / preempt | PENDING `Priority` |
| **3596031** | `ln_rl_sparse500k_dM_s977` | machine-first | 977 | `dDPfull_first_rnsh` | preempt / preempt | PENDING `Priority` |
| **3596032** | `ln_rl_sparse500k_dM_s978` | machine-first | 978 | `dDPfull_first_rnsh` | preempt / preempt | PENDING `Priority` |

### QOS, disclosed

The same rule as the batch and revision 2: the split is **between learners, never between arms**.
All 8 {r2dreamer} jobs went to `-p gpu --qos=normal`; all 4 {RLPD} jobs to `-p preempt --qos=preempt`.
**Neither allocation had a free slot at submission** — `normal` was at 10 of 10 GPU jobs running
(8 `ln_r2_*` of the batch + 2 of the rev-2 extension) and `preempt` was at its own cap — so every one
of the twelve queued. Slurm holds a submission over a running cap as PENDING rather than refusing it;
this is a queueing delay, not a failure, and no job was refused or required a code change.

Nothing was cancelled. No do-not-touch tree was modified: `$LAB/gp_ladderN` was read (and is written
only by the launchers' own run dirs and `RUN_REGISTRY.jsonl`, exactly as the batch writes them), and
its `git describe` re-read after all twelve submissions still prints
`known-good-2026-08-27-895-ga40c8aa1-dirty`.

### Stamp verification

Deferred: all twelve were PENDING at submission, and a `[ladder]` stamp exists only once a job starts.
The checklist is §5's: within a ladder the line must be identical character for character but for the
`git=` suffix and the run name —

    nested_sparse: [ladder] unified-2026-09-10 | ladder=nested_sparse | home=1 | max_return=1 |
                   terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f |
                   full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
    staged (ctl): [ladder] … | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 |
                   max_return=8 | terminal=slide_success+tipped | … same tip= and same three hashes …

plus, on the {r2dreamer} jobs, `[ladder] return_clamp=1.0` (sparse) / `8.0` (staged control)
`(env and model agree)` — P-aa-5.
