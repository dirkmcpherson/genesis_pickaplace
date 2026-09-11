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
